#include "Controller_SOGI.h"
#include <math.h>

// === 硬编码常量 (见 Docs/SOGI-FLL-CONTROLLER.md §6) ===
static constexpr float SOGI_F0_HZ    = 1.0f;    // FLL 初值，自适应
static constexpr float SOGI_K        = 1.41f;   // √2 阻尼
static constexpr float SOGI_GAMMA    = 40.0f;   // FLL 学习率
static constexpr float SOGI_F_MIN    = 0.3f;    // 频率夹紧下限 (Hz)
static constexpr float SOGI_F_MAX    = 3.5f;    // 频率夹紧上限 (Hz)
static constexpr float SOGI_RAMP_SEC = 1.5f;    // 冷启动斜坡时长
// 站立门控：双腿角度长时间处于小角度窗时，强制 τ=0，抑制站立噪声触发
static constexpr float SOGI_STAND_ANGLE_DEG  = 10.0f;   // 站立角度窗 (deg)
static constexpr float SOGI_STAND_HOLD_SEC   = 0.25f;  // 连续保持时长 (s)
static constexpr float TWO_PI_F      = 6.28318530718f;

Controller_SOGI::Controller_SOGI() {
  A_gain_        = 0.0f;
  phi_lead_rad_  = 0.0f;
  amp_min_       = 20.0f;
  stand_hold_elapsed_s_ = 0.0f;
  reset();
}

void Controller_SOGI::reset() {
  L_.wn = TWO_PI_F * SOGI_F0_HZ;
  L_.x1 = 0.0f;
  L_.x2 = 0.0f;
  R_.wn = TWO_PI_F * SOGI_F0_HZ;
  R_.x1 = 0.0f;
  R_.x2 = 0.0f;
  ramp_elapsed_ = 0.0f;
  stand_hold_elapsed_s_ = 0.0f;
  // ADO 用于显示功率指标；delay 永不启用。
  ado_.reset(0.0f);
}

void Controller_SOGI::parse_params(const BleDownlinkData& dl) {
  A_gain_ = dl.sogi_A_gain;
  if (A_gain_ < 0.0f)  A_gain_ = 0.0f;
  if (A_gain_ > 15.0f) A_gain_ = 15.0f;

  float lead_deg = dl.sogi_phi_lead_deg;
  if (lead_deg < -90.0f) lead_deg = -90.0f;
  if (lead_deg >  90.0f) lead_deg =  90.0f;
  phi_lead_rad_ = lead_deg * (float)M_PI / 180.0f;

  amp_min_ = dl.sogi_amp_min;
  if (amp_min_ < 0.0f)   amp_min_ = 0.0f;
  if (amp_min_ > 500.0f) amp_min_ = 500.0f;
}

void Controller_SOGI::step_sogi(Sogi& s, float omega, float dt,
                                float phi_lead_rad,
                                float& sin_lead_out, float& amp_out) {
  // --- SOGI 更新 (前向欧拉) ---
  float err = omega - s.x1;
  float dx1 = SOGI_K * s.wn * err - s.wn * s.x2;
  float dx2 = s.wn * s.x1;
  s.x1 += dx1 * dt;
  s.x2 += dx2 * dt;

  // --- FLL 频率锁定 ---
  float denom = s.x1 * s.x1 + s.x2 * s.x2 + 1e-6f;
  float dwn   = -SOGI_GAMMA * err * s.x2 / denom;
  s.wn += dwn * dt;
  const float w_min = TWO_PI_F * SOGI_F_MIN;
  const float w_max = TWO_PI_F * SOGI_F_MAX;
  if (s.wn < w_min) s.wn = w_min;
  if (s.wn > w_max) s.wn = w_max;

  // --- 输出: sin(φ+lead), amp ---
  // φ = atan2(x1, -x2) 使 sin(φ) 与 ω 同相
  float phi = atan2f(s.x1, -s.x2);
  sin_lead_out = sinf(phi + phi_lead_rad);
  amp_out      = sqrtf(s.x1 * s.x1 + s.x2 * s.x2);
}

void Controller_SOGI::compute(const CtrlInput& in, CtrlOutput& out) {
  float dt = in.Ts;
  if (dt <= 0.0f) dt = 0.01f;

  float sinL, ampL, sinR, ampR;
  step_sogi(L_, in.LTAVx, dt, phi_lead_rad_, sinL, ampL);
  step_sogi(R_, in.RTAVx, dt, phi_lead_rad_, sinR, ampR);

  // 冷启动斜坡 (SOGI-FLL 需 1-2 周期锁定)
  ramp_elapsed_ += dt;
  float ramp = ramp_elapsed_ / SOGI_RAMP_SEC;
  if (ramp > 1.0f) ramp = 1.0f;

  // 幅值看门狗: 静止时不出力
  float gateL = (ampL > amp_min_) ? 1.0f : 0.0f;
  float gateR = (ampR > amp_min_) ? 1.0f : 0.0f;

  float tau_L = A_gain_ * ramp * gateL * sinL;
  float tau_R = A_gain_ * ramp * gateR * sinR;

  // 角度门控：用于抑制站立时低频速度噪声造成的虚假助力
  // 仅当双腿同时长时间停留在小角度窗内，才强制置零，避免走路过中立位被误触发。
  const bool in_stand_angle_window =
      (fabsf(in.LTx_filtered) <= SOGI_STAND_ANGLE_DEG) &&
      (fabsf(in.RTx_filtered) <= SOGI_STAND_ANGLE_DEG);
  if (in_stand_angle_window) {
    stand_hold_elapsed_s_ += dt;
  } else {
    stand_hold_elapsed_s_ = 0.0f;
  }
  if (stand_hold_elapsed_s_ >= SOGI_STAND_HOLD_SEC) {
    tau_L = 0.0f;
    tau_R = 0.0f;
  }

  // 限幅
  float max_abs = fabsf(in.max_torque_cfg);
  if (tau_L >  max_abs) tau_L =  max_abs;
  if (tau_L < -max_abs) tau_L = -max_abs;
  if (tau_R >  max_abs) tau_R =  max_abs;
  if (tau_R < -max_abs) tau_R = -max_abs;

  out.tau_L = tau_L;
  out.tau_R = tau_R;

  // 推入 ADO 缓冲（SOGI 无 pre/post delay 之分，tau_src 即最终 tau）
  ado_.push_sample(tau_L, tau_R, in.LTAVx, in.RTAVx);
}

void Controller_SOGI::tick_auto_delay(unsigned long now_us, float gait_freq_hz) {
  // ADO::tick 即使 enabled=false 也会更新 ratio/pos/neg 指标；仅跳过 delay 应用
  ado_.tick(now_us, gait_freq_hz);
}

void Controller_SOGI::fill_ble_status(uint8_t* buf40) const {
  ado_.fill_ble_status(buf40, 1.0f);
}
