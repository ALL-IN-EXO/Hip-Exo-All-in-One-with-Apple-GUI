#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

/*************************************************************
 * MotorDriver — 统一电机抽象层
 *
 * 电机 ID 分配（硬件接线决定，不可随意修改）:
 * ┌──────────┬────────────┬────────────┐
 * │          │  左腿 (L)  │  右腿 (R)  │
 * ├──────────┼────────────┼────────────┤
 * │ SIG      │ node_id=1  │ node_id=2  │
 * │ TMOTOR   │ drive_id=104│ drive_id=105│
 * └──────────┴────────────┴────────────┘
 *
 * 扭矩约定: send_torque_Nm() 接受 **输出轴** 扭矩 (Nm),
 *           内部自动处理减速比/KT 转换。
 *           get_torque_meas() 返回 **输出轴** 扭矩 (Nm)。
 *************************************************************/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Motor_Control_Sig.h"
#include "Motor_Control_Tmotor.h"

/*
 * CAN 初始化策略（真机验证结论，不可改动）:
 *   单纯 extern 共享全局 Can3 → 持续发送后 Can3.write() 失稳/卡死。
 *   正确做法: 全局 Can3 + 类内 Can3 + 二次初始化。
 *   - .ino 保留全局 Can3, initial_CAN() 做全局 begin()+setBaudRate()
 *   - 每个电机类内有自己的 Can3 成员, init() 里调 hw_.initial_CAN() 做类内初始化
 *   - 电机类用类内 Can3.write() 发送（已自带）
 *   - .ino 用全局 Can3.read() 接收反馈（receive_motor_feedback）
 */

enum MotorBrand : uint8_t {
  BRAND_NONE   = 0,
  BRAND_SIG    = 1,
  BRAND_TMOTOR = 2,
  // BRAND_FUTURE = 3,  // 留给第三种电机
};

// ===================== 抽象基类 =====================
class MotorDriver {
public:
  virtual ~MotorDriver() {}

  // --- 生命周期 ---
  virtual void init()  = 0;   // 初始化电机（含 CAN、模式设置、读初始位置）
  virtual void stop()  = 0;   // 发零扭矩

  // --- 指令 ---
  virtual void send_torque_Nm(float tau_output) = 0;  // 输出轴 Nm

  // --- 反馈解析（从 CAN RX 消息中解） ---
  virtual bool parse_feedback(const CAN_message_t &msg) = 0;

  // --- 请求反馈（SIG 需要主动请求，TMOTOR 自动推送）---
  virtual void request_feedback() {}

  // --- 统一读取 ---
  virtual float  get_torque_meas()  const = 0;   // 输出轴 Nm
  virtual float  get_pos_deg()      const = 0;   // 输出轴度
  virtual float  get_vel_deg_s()    const = 0;   // 输出轴 °/s
  virtual float  get_temp_C()       const { return NAN; }
  virtual uint32_t get_error()      const { return 0; }
  // --- 驱动原生遥测 (用于 BLE 上行 motor_cur/rpm 量化字段) ---
  virtual float  get_motor_current_A() const { return NAN; } // 驱动原生电流 (A)
  virtual float  get_motor_speed_rpm() const { return NAN; } // 驱动原生转速 (rpm)

  // --- 品牌标识 ---
  virtual MotorBrand  brand()      const = 0;
  virtual const char* brand_name() const = 0;
};

// ===================== SIG (ODrive) 包装 =====================
class MotorDriver_Sig : public MotorDriver {
  Motor_Control_Sig hw_;
  float gear_ratio_;
  float initial_pos_rev_ = 0.0f;

public:
  // gear_ratio: 减速比（输出轴 Nm = 电机 Nm × gear_ratio）
  MotorDriver_Sig(uint8_t node_id, float gear_ratio = 9.76f)
    : hw_(node_id), gear_ratio_(gear_ratio) {}

  void init() override {
    // 类内 CAN 二次初始化（真机验证必须）
    hw_.initial_CAN();

    hw_.clear_errors();
    delay(50);
    hw_.set_controller_mode(1, 1);   // torque control, direct input
    delay(50);
    hw_.set_axis_state(8);           // closed-loop
    delay(200);

    // 读初始位置
    hw_.request_encoder_estimates();
    delay(20);
    CAN_message_t msg;
    uint32_t t0 = millis();
    while (millis() - t0 < 500) {
      while (hw_.Can3.read(msg)) {
        hw_.unpack_can_message(msg);
      }
      if (hw_.seen_encoder) break;
      delay(5);
    }
    initial_pos_rev_ = isnan(hw_.pos_rev) ? 0.0f : hw_.pos_rev;

    hw_.send_torque_Nm(0.0f);
    delay(50);

    // 读一轮反馈
    for (int i = 0; i < 200; i++) {
      while (hw_.Can3.read(msg)) hw_.unpack_can_message(msg);
    }
    delay(200);
  }

  void stop() override {
    hw_.send_torque_Nm(0.0f);
  }

  void send_torque_Nm(float tau_output) override {
    hw_.send_torque_Nm(tau_output / gear_ratio_);
  }

  bool parse_feedback(const CAN_message_t &msg) override {
    return hw_.unpack_can_message(msg);
  }

  void request_feedback() override {
    hw_.request_encoder_estimates();
    hw_.request_torques();
  }

  float get_torque_meas() const override {
    return isnan(hw_.torque_measured_Nm)
           ? 0.0f
           : hw_.torque_measured_Nm * gear_ratio_;
  }

  float get_pos_deg() const override {
    float rev = isnan(hw_.pos_rev) ? 0.0f : hw_.pos_rev;
    return (rev - initial_pos_rev_) * 360.0f;
  }

  float get_vel_deg_s() const override {
    return isnan(hw_.vel_rev_s) ? 0.0f : hw_.vel_rev_s * 360.0f;
  }

  // SIG (ODrive) 不直接暴露电机相电流 → 上层量化标志会标无效
  float get_motor_current_A() const override {
    return NAN;
  }

  float get_motor_speed_rpm() const override {
    return isnan(hw_.vel_rev_s) ? NAN : hw_.vel_rev_s * 60.0f;
  }

  uint32_t get_error() const override {
    return hw_.axis_error;
  }

  MotorBrand  brand()      const override { return BRAND_SIG; }
  const char* brand_name() const override { return "SIG"; }
};

// ===================== TMOTOR (VESC Servo) 包装 =====================
class MotorDriver_Tmotor : public MotorDriver {
  Motor_Control_Tmotor hw_;
  float kt_output_;           // 输出轴 Nm/A (含减速比+效率)
  float initial_pos_deg_ = 0.0f;

public:
  // kt_output: 输出轴扭矩常数 = kt_motor × gear_ratio × efficiency
  //            例如 0.091 × 9 × 0.9 ≈ 0.73
  MotorDriver_Tmotor(uint8_t id, int drive_id, float kt_output = 0.73f)
    : hw_(id, drive_id), kt_output_(kt_output) {}

  void init() override {
    // 类内 CAN 二次初始化（真机验证必须）
    hw_.initial_CAN();

    hw_.send_current_A(0.0f);
    delay(20);

    // 清空 CAN 缓冲
    CAN_message_t tmp;
    while (hw_.Can3.read(tmp)) {}

    hw_.send_current_A(0.0f);
    delay(20);

    // 读初始位置
    uint32_t t0 = millis();
    bool got = false;
    while (millis() - t0 < 500) {
      CAN_message_t msg;
      while (hw_.Can3.read(msg)) {
        if (hw_.unpack_servo_telemetry(msg)) got = true;
      }
      if (got) break;
      delay(5);
    }
    initial_pos_deg_ = isnan(hw_.servo_pos_deg) ? 0.0f : hw_.servo_pos_deg;
  }

  void stop() override {
    hw_.send_current_A(0.0f);
  }

  void send_torque_Nm(float tau_output) override {
    float iq = tau_output / kt_output_;
    hw_.send_current_A(iq);
  }

  bool parse_feedback(const CAN_message_t &msg) override {
    return hw_.unpack_servo_telemetry(msg);
  }

  float get_torque_meas() const override {
    return isnan(hw_.servo_cur_A) ? 0.0f : fabsf(hw_.servo_cur_A) * kt_output_;
  }

  float get_pos_deg() const override {
    return isnan(hw_.servo_pos_deg)
           ? 0.0f
           : hw_.servo_pos_deg - initial_pos_deg_;
  }

  float get_vel_deg_s() const override {
    // servo_spd_rpm 是电气 RPM×10 的解码值
    // 输出轴 °/s = servo_spd_rpm / pole_pairs / gear_ratio × 6
    // 简化：这里先返回 rpm 原始值，后续标定再调
    return isnan(hw_.servo_spd_rpm) ? 0.0f : hw_.servo_spd_rpm;
  }

  // TMOTOR 驱动原生暴露 Iq + 电机转速, 直接透传 (BLE 上行做量化)
  float get_motor_current_A() const override {
    return isnan(hw_.servo_cur_A) ? NAN : hw_.servo_cur_A;
  }

  float get_motor_speed_rpm() const override {
    return isnan(hw_.servo_spd_rpm) ? NAN : hw_.servo_spd_rpm;
  }

  float get_temp_C() const override {
    return isnan(hw_.servo_temp_C) ? NAN : hw_.servo_temp_C;
  }

  uint32_t get_error() const override {
    return isnan(hw_.servo_error) ? 0 : (uint32_t)hw_.servo_error;
  }

  MotorBrand  brand()      const override { return BRAND_TMOTOR; }
  const char* brand_name() const override { return "TMOTOR"; }
};

#endif // MOTOR_DRIVER_H
