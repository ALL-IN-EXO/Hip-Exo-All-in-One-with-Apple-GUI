#include "Motor_Control_Sig.h"

// 人在 GUI 里输入和看的“力矩”，统一为输出轴侧 Nm。
// 电机驱动（SIG/ODrive）内部的Torque和Iq，统一是转子侧。
// 两者关系用这三条：
// tau_rotor = tau_out / (gear_ratio * eta)
// Iq = tau_rotor / Kt
// tau_rotor = Kt * Iq
// gear_ratio = 9.67
// Kt = 0.097 Nm/A（转子侧）
// eta 先按 1.0（理想）用，后续可按实测修正



#include <math.h>
#include <string.h>

namespace {
static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}
}  // namespace

Motor_Control_Sig::Motor_Control_Sig(uint8_t node_id) {
  node_id_ = node_id;
}

void Motor_Control_Sig::initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(200);
}

void Motor_Control_Sig::set_node_id(uint8_t node_id) {
  node_id_ = node_id;
}

uint8_t Motor_Control_Sig::get_node_id() const {
  return node_id_;
}

uint32_t Motor_Control_Sig::compose_id(uint8_t node_id, uint8_t cmd_id) {
  return ((uint32_t)(node_id & 0x3F) << 5) | (uint32_t)(cmd_id & 0x1F);
}

uint32_t Motor_Control_Sig::read_u32_le(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

float Motor_Control_Sig::read_f32_le(const uint8_t *p) {
  float v = 0.0f;
  memcpy(&v, p, sizeof(float));
  return v;
}

void Motor_Control_Sig::write_u32_le(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

void Motor_Control_Sig::write_f32_le(uint8_t *p, float v) {
  memcpy(p, &v, sizeof(float));
}

float Motor_Control_Sig::clamp_float(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void Motor_Control_Sig::send_empty_cmd(uint8_t cmd_id) {
  msgW.id = compose_id(node_id_, cmd_id);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  memset(msgW.buf, 0, 8);
  Can3.write(msgW);
}

void Motor_Control_Sig::send_u32x2_cmd(uint8_t cmd_id, uint32_t v0, uint32_t v1) {
  msgW.id = compose_id(node_id_, cmd_id);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  write_u32_le(&msgW.buf[0], v0);
  write_u32_le(&msgW.buf[4], v1);
  Can3.write(msgW);
}

void Motor_Control_Sig::send_f32x2_cmd(uint8_t cmd_id, float a, float b) {
  msgW.id = compose_id(node_id_, cmd_id);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  write_f32_le(&msgW.buf[0], a);
  write_f32_le(&msgW.buf[4], b);
  Can3.write(msgW);
}

void Motor_Control_Sig::clear_errors() {
  send_empty_cmd(0x18);
}

void Motor_Control_Sig::reboot() {
  send_empty_cmd(0x16);
}

void Motor_Control_Sig::set_axis_state(uint32_t requested_state) {
  send_u32x2_cmd(0x07, requested_state, 0u);
}

void Motor_Control_Sig::set_controller_mode(uint32_t control_mode, uint32_t input_mode) {
  send_u32x2_cmd(0x0B, control_mode, input_mode);
}

void Motor_Control_Sig::send_torque_Nm(float torque) {
  // 常见量程在 +-50Nm，超过仍可发送，这里做保护避免误输入。
  torque = clamp_float(torque, -120.0f, 120.0f);

  msgW.id = compose_id(node_id_, 0x0E);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  write_f32_le(&msgW.buf[0], torque);
  memset(&msgW.buf[4], 0, 4);
  Can3.write(msgW);
}

void Motor_Control_Sig::send_velocity_rev_s(float vel_rev_s, float torque_ff_Nm) {
  vel_rev_s = clamp_float(vel_rev_s, -200.0f, 200.0f);
  torque_ff_Nm = clamp_float(torque_ff_Nm, -120.0f, 120.0f);
  send_f32x2_cmd(0x0D, vel_rev_s, torque_ff_Nm);
}

void Motor_Control_Sig::send_position_rev(float pos_rev, float vel_ff_rev_s, float torque_ff_Nm) {
  const int16_t vel_ff_x1000 = clamp_i16((int32_t)lroundf(vel_ff_rev_s * 1000.0f));
  const int16_t torque_ff_x1000 = clamp_i16((int32_t)lroundf(torque_ff_Nm * 1000.0f));

  msgW.id = compose_id(node_id_, 0x0C);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;

  write_f32_le(&msgW.buf[0], pos_rev);
  msgW.buf[4] = (uint8_t)(vel_ff_x1000 & 0xFF);
  msgW.buf[5] = (uint8_t)((vel_ff_x1000 >> 8) & 0xFF);
  msgW.buf[6] = (uint8_t)(torque_ff_x1000 & 0xFF);
  msgW.buf[7] = (uint8_t)((torque_ff_x1000 >> 8) & 0xFF);
  Can3.write(msgW);
}

void Motor_Control_Sig::request_encoder_estimates() {
  send_empty_cmd(0x09);
}

void Motor_Control_Sig::request_iq() {
  send_empty_cmd(0x14);
}

void Motor_Control_Sig::request_torques() {
  send_empty_cmd(0x1C);
}

void Motor_Control_Sig::request_error_detail(uint8_t error_type) {
  msgW.id = compose_id(node_id_, 0x03);
  msgW.len = 8;
  msgW.flags.extended = 0;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  memset(msgW.buf, 0, 8);
  msgW.buf[0] = error_type;
  pending_error_type_ = error_type;
  Can3.write(msgW);
}

bool Motor_Control_Sig::unpack_can_message(const CAN_message_t &msg) {
  if (msg.flags.extended) return false;
  if (msg.len < 8) return false;

  const uint8_t rx_node = (uint8_t)((msg.id >> 5) & 0x3F);
  const uint8_t cmd_id = (uint8_t)(msg.id & 0x1F);
  if (rx_node != node_id_) return false;

  const uint32_t now_ms = millis();
  bool touched = true;

  switch (cmd_id) {
    case 0x01: {
      axis_error = read_u32_le(&msg.buf[0]);
      axis_state = msg.buf[4];
      axis_flags = msg.buf[5];
      axis_life = msg.buf[7];
      seen_heartbeat = true;
      last_heartbeat_ms = now_ms;
      break;
    }
    case 0x09: {
      pos_rev = read_f32_le(&msg.buf[0]);
      vel_rev_s = read_f32_le(&msg.buf[4]);
      seen_encoder = true;
      break;
    }
    case 0x14: {
      iq_setpoint_A = read_f32_le(&msg.buf[0]);
      iq_measured_A = read_f32_le(&msg.buf[4]);
      seen_iq = true;
      break;
    }
    case 0x1C: {
      torque_setpoint_Nm = read_f32_le(&msg.buf[0]);
      torque_measured_Nm = read_f32_le(&msg.buf[4]);
      seen_torque = true;
      break;
    }
    case 0x03: {
      detail_error_u32 = read_u32_le(&msg.buf[0]);
      detail_error_hi32 = (msg.len >= 8) ? read_u32_le(&msg.buf[4]) : 0u;
      last_error_type = pending_error_type_;
      break;
    }
    default: {
      touched = false;
      break;
    }
  }

  if (touched) {
    last_feedback_ms = now_ms;
  }
  return touched;
}
