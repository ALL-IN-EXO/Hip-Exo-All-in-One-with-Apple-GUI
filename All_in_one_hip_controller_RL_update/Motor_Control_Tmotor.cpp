#include "Motor_Control_Tmotor.h"
#include <math.h>
// GUI里模式1输入的是扭矩(Nm)，不是电流。
// Teensy内部换算：Iq(A) = 扭矩(Nm) / Kt。
// 当前统一常数：Kt = 0.73 Nm/A（在 Tmotor_Teensy_Project.ino 里）。
// 下发给驱动的是Iq电流（send_current_A(Iq)）。
// 回读到的是实际Iq电流；GUI“实际扭矩”显示为：实际扭矩 = 实际Iq * Kt。
// 转子侧 torque constant 在0.7这个数量级
// 输出侧 torque constant 在0.09这个数量级


static inline void pack_i32_be(uint8_t *buf, int32_t v) {
  buf[0] = (uint8_t)((v >> 24) & 0xFF);
  buf[1] = (uint8_t)((v >> 16) & 0xFF);
  buf[2] = (uint8_t)((v >> 8) & 0xFF);
  buf[3] = (uint8_t)(v & 0xFF);
}

static inline void pack_i16_be(uint8_t *buf, int16_t v) {
  buf[0] = (uint8_t)((v >> 8) & 0xFF);
  buf[1] = (uint8_t)(v & 0xFF);
}

static inline int16_t unpack_i16_be(const uint8_t *buf) {
  const uint16_t u = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
  return (int16_t)u;
}

Motor_Control_Tmotor::Motor_Control_Tmotor(uint8_t id, int can_id) {
  ID = id;
  drive_id = static_cast<uint8_t>(can_id);
}

void Motor_Control_Tmotor::initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(200);
}

void Motor_Control_Tmotor::set_drive_id(uint8_t id) {
  drive_id = id;
}

uint8_t Motor_Control_Tmotor::get_drive_id() const {
  return drive_id;
}

void Motor_Control_Tmotor::send_current_A(float iq_A)
{
  // 伺服文档：-60~60A -> -60000~60000
  if (iq_A > 60.0f) iq_A = 60.0f;
  if (iq_A < -60.0f) iq_A = -60.0f;

  int32_t iq_mA = (int32_t)(iq_A * 1000.0f);

  msgW.len = 4;
  msgW.flags.extended = 1;   // 关键：扩展帧
  msgW.flags.remote   = 0;
  msgW.flags.overrun  = 0;
  msgW.flags.reserved = 0;

  // control mode ID = 1 (CAN_PACKET_SET_CURRENT)
  uint32_t ext_id = ((uint32_t)drive_id) | ((uint32_t)1u << 8);
  msgW.id = ext_id;

  // 大端：最高字节先发（与你文档 buffer_append_int32 一致）
  msgW.buf[0] = (uint8_t)((iq_mA >> 24) & 0xFF);
  msgW.buf[1] = (uint8_t)((iq_mA >> 16) & 0xFF);
  msgW.buf[2] = (uint8_t)((iq_mA >>  8) & 0xFF);
  msgW.buf[3] = (uint8_t)((iq_mA >>  0) & 0xFF);

  Can3.write(msgW);
}

void Motor_Control_Tmotor::send_duty(float duty) {
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  int32_t duty_x100000 = (int32_t)lroundf(duty * 100000.0f);

  msgW.len = 4;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  msgW.id = ((uint32_t)drive_id) | ((uint32_t)0u << 8);
  pack_i32_be(msgW.buf, duty_x100000);
  Can3.write(msgW);
}

void Motor_Control_Tmotor::send_current_brake_A(float iq_brake_A) {
  if (iq_brake_A > 60.0f) iq_brake_A = 60.0f;
  if (iq_brake_A < -60.0f) iq_brake_A = -60.0f;
  int32_t cur_x1000 = (int32_t)lroundf(iq_brake_A * 1000.0f);

  msgW.len = 4;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  msgW.id = ((uint32_t)drive_id) | ((uint32_t)2u << 8);
  pack_i32_be(msgW.buf, cur_x1000);
  Can3.write(msgW);
}

void Motor_Control_Tmotor::send_rpm(float rpm) {
  int32_t rpm_i32 = (int32_t)lroundf(rpm);
  msgW.len = 4;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  msgW.id = ((uint32_t)drive_id) | ((uint32_t)3u << 8);
  pack_i32_be(msgW.buf, rpm_i32);
  Can3.write(msgW);
}

void Motor_Control_Tmotor::send_pos(float pos) {
  int32_t pos_x1e6 = (int32_t)lroundf(pos * 1000000.0f);
  msgW.len = 4;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  msgW.id = ((uint32_t)drive_id) | ((uint32_t)4u << 8);
  pack_i32_be(msgW.buf, pos_x1e6);
  Can3.write(msgW);
}

void Motor_Control_Tmotor::send_pos_spd(float pos, float spd, float acc) {
  int32_t pos_x1e6 = (int32_t)lroundf(pos * 1000000.0f);
  int16_t spd_i16 = (int16_t)lroundf(spd);
  int16_t acc_i16 = (int16_t)lroundf(acc);

  msgW.len = 8;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;
  msgW.id = ((uint32_t)drive_id) | ((uint32_t)6u << 8);
  pack_i32_be(&msgW.buf[0], pos_x1e6);
  pack_i16_be(&msgW.buf[4], spd_i16);
  pack_i16_be(&msgW.buf[6], acc_i16);
  Can3.write(msgW);
}

void Motor_Control_Tmotor::set_origin_here() {
  msgW.len = 0;
  msgW.flags.extended = 1;
  msgW.flags.remote = 0;
  msgW.flags.overrun = 0;
  msgW.flags.reserved = 0;

  // control mode ID = 5 (CAN_PACKET_SET_ORIGIN_HERE)
  uint32_t ext_id = ((uint32_t)drive_id) | ((uint32_t)5u << 8);
  msgW.id = ext_id;
  Can3.write(msgW);
}

bool Motor_Control_Tmotor::unpack_servo_telemetry(const CAN_message_t &msgR) {
  if (msgR.len != 8) return false;

  if (msgR.flags.extended) {
    uint8_t rx_drive = (uint8_t)(msgR.id & 0xFF);
    if (rx_drive != drive_id) return false;
  } else {
    if ((uint8_t)msgR.id != drive_id) return false;
  }

  const int16_t pos_int = unpack_i16_be(&msgR.buf[0]);
  const int16_t spd_int = unpack_i16_be(&msgR.buf[2]);
  const int16_t cur_int = unpack_i16_be(&msgR.buf[4]);

  int8_t temp_raw = (int8_t)msgR.buf[6];
  uint8_t err_raw = (uint8_t)msgR.buf[7];

  servo_pos_raw_x10 = pos_int;
  servo_pos_deg = (float)pos_int * 0.1f;
  servo_spd_rpm = (float)spd_int * 10.0f;
  servo_cur_A = (float)cur_int * 0.01f;
  servo_temp_C = temp_raw;
  servo_error = err_raw;
  return true;
}
