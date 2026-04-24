#ifndef CONTROLLER_SOGI_H
#define CONTROLLER_SOGI_H

/************************************************************
 * Controller_SOGI — SOGI-FLL 相位同步助力
 *
 * 输入: 左右髋角速度 ω (deg/s)
 * 每侧独立跑一个 SOGI-FLL 提取瞬时相位 φ 和幅值 amp
 * 输出: τ = A_gain * ramp * gate(amp) * gate(angle_stand) * sin(φ + phi_lead)
 *
 * 算法细节见 Docs/SOGI-FLL-CONTROLLER.md
 *
 * GUI 可调参数:
 *   A_gain        力矩幅值 (Nm)
 *   phi_lead_deg  相位超前补偿 (°)
 *   amp_min       幅值看门狗门限 (deg/s)
 *
 * 其他参数硬编码 (见 .cpp 顶部常量)
 ************************************************************/

#include "Controller.h"
#include "AutoDelayOptimizer.h"

class Controller_SOGI : public Controller {
public:
  Controller_SOGI();

  void compute(const CtrlInput& in, CtrlOutput& out) override;
  void parse_params(const BleDownlinkData& dl) override;
  void reset() override;
  const char* name() const override { return "SOGI"; }
  AlgoID id() const override { return ALGO_SOGI; }

  // 1Hz 侧环：让 ADO 持续计算 ratio/pos/neg 指标（enabled 恒 false，不改 delay）
  void tick_auto_delay(unsigned long now_us, float gait_freq_hz);

  // 填充 BLE 上行 RPi 透传槽（40B v3 格式；GUI 用于显示 +Ratio）
  void fill_ble_status(uint8_t* buf40) const;

  AutoDelayOptimizer ado_;  // public: .ino 可读取状态

private:
  // === GUI 参数 ===
  float A_gain_;          // Nm
  float phi_lead_rad_;    // 由 deg 转换
  float amp_min_;         // deg/s

  // 站立角度门控持续计时（用于抑制站立低频速度噪声）
  float stand_hold_elapsed_s_;

  // === 单侧 SOGI-FLL 状态 ===
  struct Sogi {
    float wn;      // 当前中心角频率 (rad/s)
    float x1;      // 同相分量
    float x2;      // 正交分量
  };
  Sogi L_;
  Sogi R_;

  // 冷启动 ramp
  float ramp_elapsed_;   // 秒

  // 单步 SOGI-FLL 更新，返回 sin(phi+lead) 和 amp
  static void step_sogi(Sogi& s, float omega, float dt,
                        float phi_lead_rad,
                        float& sin_lead_out, float& amp_out);
};

#endif // CONTROLLER_SOGI_H
