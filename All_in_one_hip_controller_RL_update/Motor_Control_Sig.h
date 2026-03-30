#ifndef MOTOR_CONTROL_SIG_H
#define MOTOR_CONTROL_SIG_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

class Motor_Control_Sig {
 public:
  explicit Motor_Control_Sig(uint8_t node_id);

  void initial_CAN();
  void set_node_id(uint8_t node_id);
  uint8_t get_node_id() const;

  void clear_errors();
  void reboot();
  void set_axis_state(uint32_t requested_state);
  void set_controller_mode(uint32_t control_mode, uint32_t input_mode);

  void send_torque_Nm(float torque);
  void send_velocity_rev_s(float vel_rev_s, float torque_ff_Nm = 0.0f);
  void send_position_rev(float pos_rev, float vel_ff_rev_s = 0.0f, float torque_ff_Nm = 0.0f);

  void request_encoder_estimates();
  void request_iq();
  void request_torques();
  void request_error_detail(uint8_t error_type);

  bool unpack_can_message(const CAN_message_t &msg);

  float pos_rev = NAN;
  float vel_rev_s = NAN;
  float iq_setpoint_A = NAN;
  float iq_measured_A = NAN;
  float torque_setpoint_Nm = NAN;
  float torque_measured_Nm = NAN;

  uint32_t axis_error = 0;
  uint8_t axis_state = 0;
  uint8_t axis_flags = 0;
  uint8_t axis_life = 0;
  uint8_t last_error_type = 255;
  uint32_t detail_error_u32 = 0;
  uint32_t detail_error_hi32 = 0;

  bool seen_heartbeat = false;
  bool seen_encoder = false;
  bool seen_iq = false;
  bool seen_torque = false;

  uint32_t last_heartbeat_ms = 0;
  uint32_t last_feedback_ms = 0;

  FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

 private:
  uint8_t node_id_ = 1;
  uint8_t pending_error_type_ = 255;
  CAN_message_t msgW{};

  static uint32_t compose_id(uint8_t node_id, uint8_t cmd_id);
  static uint32_t read_u32_le(const uint8_t *p);
  static float read_f32_le(const uint8_t *p);
  static void write_u32_le(uint8_t *p, uint32_t v);
  static void write_f32_le(uint8_t *p, float v);
  static float clamp_float(float v, float lo, float hi);

  void send_empty_cmd(uint8_t cmd_id);
  void send_u32x2_cmd(uint8_t cmd_id, uint32_t v0, uint32_t v1);
  void send_f32x2_cmd(uint8_t cmd_id, float a, float b);
};

#endif
