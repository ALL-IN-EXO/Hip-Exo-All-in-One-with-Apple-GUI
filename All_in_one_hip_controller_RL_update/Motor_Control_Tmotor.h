#ifndef MOTOR_CONTROL_TMOTOR_H
#define MOTOR_CONTROL_TMOTOR_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

class Motor_Control_Tmotor {
 public:
  Motor_Control_Tmotor(uint8_t id, int can_id);

  void initial_CAN();
  void send_current_A(float iq_A);
  void send_duty(float duty);
  void send_current_brake_A(float iq_brake_A);
  void send_rpm(float rpm);
  void send_pos(float pos);
  void send_pos_spd(float pos, float spd, float acc);
  void set_origin_here();
  bool unpack_servo_telemetry(const CAN_message_t &msg);
  void set_drive_id(uint8_t id);
  uint8_t get_drive_id() const;

  float torque = 0.0f;
  float servo_pos_deg = NAN;
  int16_t servo_pos_raw_x10 = 0;
  float servo_spd_rpm = NAN;
  float servo_cur_A = NAN;
  float servo_temp_C = NAN;
  float servo_error = NAN;
  FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

 private:
  uint8_t ID = 0;
  uint8_t drive_id = 104;
  CAN_message_t msgW{};
};

#endif
