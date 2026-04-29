#include "Controller_SOGI.h"
#include <math.h>

// === 硬编码常量 (见 Docs/SOGI-FLL-CONTROLLER.md §6) ===
static constexpr float SOGI_F0_HZ    = 1.0f;    // FLL 初值，自适应
static constexpr float SOGI_K        = 1.41f;   // √2 阻尼
static constexpr float SOGI_GAMMA    = 40.0f;   // FLL 学习率
static constexpr float SOGI_F_MIN    = 0.3f;    // 频率夹紧下限 (Hz)
static constexpr float SOGI_F_MAX    = 3.5f;    // 频率夹紧上限 (Hz)
static constexpr float SOGI_RAMP_SEC = 0.5f;    // 冷启动斜坡时长
// 运动状态机默认值（可由 GUI 通过 BLE 覆盖）
static constexpr float SOGI_AMP_ON_DEFAULT       = 22.0f;  // STOPPED->MOVING
static constexpr float SOGI_AMP_OFF_DEFAULT      = 18.0f;  // MOVING->STOPPED
static constexpr float SOGI_MOVE_ON_SEC_DEFAULT  = 0.15f;  // amp > AMP_ON 持续时间
static constexpr float SOGI_MOVE_OFF_SEC_DEFAULT = 0.20f;  // amp < AMP_OFF 持续时间
static constexpr float SOGI_STOP_HOLD_SEC  = 0.40f;  // STOPPED 最短保持时间
// 站立门控：双腿角度长时间处于小角度窗时，强制 τ=0，抑制站立噪声触发
static constexpr float SOGI_AMP_REF         = 200.0f;    // 速度幅值归一化参考 (deg/s)，达到此幅值时输出满增益
static constexpr float SOGI_STAND_ANGLE_DEG  = 10.0f;   // 站立角度窗 (deg)
static constexpr float SOGI_STAND_HOLD_SEC   = 0.25f;  // 连续保持时长 (s)
static constexpr float TWO_PI_F      = 6.28318530718f;
// 高频过零防抖门控
static constexpr float   SOGI_ZC_MIN_INTERVAL = 0.12f;  // 过零间隔下限 (s)，对应 ~4 Hz 半周期
static constexpr float   SOGI_ZC_WINDOW_S     = 0.50f;  // 统计窗口 (s)
static constexpr uint8_t SOGI_ZC_FAKE_THRESH  = 2;      // 窗口内假过零次数阈值
static constexpr float   SOGI_ZC_HOLD_S       = 0.050f;  // 触发后门控关断保持时间 (s)

Controller_SOGI::Controller_SOGI() {
  A_gain_        = 0.0f;
  phi_lead_rad_  = 0.0f;
  amp_min_       = 20.0f;
  amp_on_        = SOGI_AMP_ON_DEFAULT;
  amp_off_       = SOGI_AMP_OFF_DEFAULT;
  move_on_sec_   = SOGI_MOVE_ON_SEC_DEFAULT;
  move_off_sec_  = SOGI_MOVE_OFF_SEC_DEFAULT;
  vel_lpf_fc_    = 10.0f;
  vel_filt_L_    = 0.0f;
  vel_filt_R_    = 0.0f;
  sym_score_     = 0.0f;
  is_bilateral_  = false;
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
  vel_filt_L_   = 0.0f;
  vel_filt_R_   = 0.0f;
  sym_score_    = 0.0f;
  is_bilateral_ = false;
  stand_hold_elapsed_s_ = 0.0f;
  motion_state_ = MOTION_STOPPED;
  moving_candidate_s_ = 0.0f;
  stopped_candidate_s_ = 0.0f;
  stop_hold_elapsed_s_ = 0.0f;
  reset_zc_tracker(zc_L_);
  reset_zc_tracker(zc_R_);
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

  // 兼容旧 GUI：若新字段全为 0，沿用默认值而不是把状态机门限清零
  const bool has_ext_sogi_params =
      (dl.sogi_amp_on > 0.0f) ||
      (dl.sogi_amp_off > 0.0f) ||
      (dl.sogi_move_on_sec > 0.0f) ||
      (dl.sogi_move_off_sec > 0.0f);

  if (!has_ext_sogi_params) {
    amp_on_ = SOGI_AMP_ON_DEFAULT;
    amp_off_ = SOGI_AMP_OFF_DEFAULT;
    move_on_sec_ = SOGI_MOVE_ON_SEC_DEFAULT;
    move_off_sec_ = SOGI_MOVE_OFF_SEC_DEFAULT;
    return;
  }

  amp_on_ = dl.sogi_amp_on;
  if (amp_on_ < 0.0f)   amp_on_ = 0.0f;
  if (amp_on_ > 500.0f) amp_on_ = 500.0f;

  amp_off_ = dl.sogi_amp_off;
  if (amp_off_ < 0.0f)   amp_off_ = 0.0f;
  if (amp_off_ > 500.0f) amp_off_ = 500.0f;
  // 保证迟滞方向：off 不高于 on
  if (amp_off_ > amp_on_) amp_off_ = amp_on_;

  move_on_sec_ = dl.sogi_move_on_sec;
  if (move_on_sec_ < 0.0f) move_on_sec_ = 0.0f;
  if (move_on_sec_ > 2.0f) move_on_sec_ = 2.0f;

  move_off_sec_ = dl.sogi_move_off_sec;
  if (move_off_sec_ < 0.0f) move_off_sec_ = 0.0f;
  if (move_off_sec_ > 2.0f) move_off_sec_ = 2.0f;

  vel_lpf_fc_ = dl.sogi_vel_lpf_fc;
  if (vel_lpf_fc_ < 0.0f)    vel_lpf_fc_ = 0.0f;
  if (vel_lpf_fc_ > 100.0f)  vel_lpf_fc_ = 100.0f;

  Serial.printf("[SOGI] A=%.2f lead=%.1fdeg amp_min=%.1f amp_on=%.1f amp_off=%.1f"
                " move_on=%.2fs move_off=%.2fs lpf=%.1fHz\n",
                A_gain_, lead_deg, amp_min_,
                amp_on_, amp_off_, move_on_sec_, move_off_sec_, vel_lpf_fc_);
}

void Controller_SOGI::reset_zc_tracker(ZcTracker& z) {
  z.prev_v                = 0.0f;
  z.since_last_cross_s    = 9999.0f;
  z.fake_window_elapsed_s = 0.0f;
  z.hold_elapsed_s        = 0.0f;
  z.fake_count_in_window  = 0;
  z.initialized           = false;
}

bool Controller_SOGI::update_zc_tracker(ZcTracker& z, float v, float dt) {
  if (!z.initialized) {
    z.prev_v      = v;
    z.initialized = true;
    return true;
  }

  z.since_last_cross_s    += dt;
  z.fake_window_elapsed_s += dt;
  if (z.hold_elapsed_s > 0.0f) {
    z.hold_elapsed_s -= dt;
    if (z.hold_elapsed_s < 0.0f) z.hold_elapsed_s = 0.0f;
  }

  // 统计窗口滚动
  if (z.fake_window_elapsed_s >= SOGI_ZC_WINDOW_S) {
    z.fake_window_elapsed_s = 0.0f;
    z.fake_count_in_window  = 0;
  }

  // 符号变化即视为过零（排除从 0 出发的情况）
  const bool crossed = (v > 0.0f) != (z.prev_v > 0.0f) &&
                       (v != 0.0f) && (z.prev_v != 0.0f);
  z.prev_v = v;

  if (crossed) {
    if (z.since_last_cross_s < SOGI_ZC_MIN_INTERVAL) {
      z.fake_count_in_window++;
      if (z.fake_count_in_window >= SOGI_ZC_FAKE_THRESH) {
        z.hold_elapsed_s        = SOGI_ZC_HOLD_S;
        z.fake_count_in_window  = 0;
        z.fake_window_elapsed_s = 0.0f;
      }
    }
    z.since_last_cross_s = 0.0f;
  }

  return (z.hold_elapsed_s <= 0.0f);
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

void Controller_SOGI::update_motion_state(float ampL, float ampR, float dt) {
  const float amp_pair = fmaxf(ampL, ampR);

  if (motion_state_ == MOTION_STOPPED) {
    stop_hold_elapsed_s_ += dt;
    stopped_candidate_s_ = 0.0f;

    if (amp_pair > amp_on_) {
      moving_candidate_s_ += dt;
    } else {
      moving_candidate_s_ = 0.0f;
    }

    if ((stop_hold_elapsed_s_ >= SOGI_STOP_HOLD_SEC) &&
        (moving_candidate_s_ >= move_on_sec_)) {
      motion_state_ = MOTION_MOVING;
      moving_candidate_s_ = 0.0f;
      stopped_candidate_s_ = 0.0f;
    }
    return;
  }

  moving_candidate_s_ = 0.0f;
  stop_hold_elapsed_s_ = 0.0f;

  if (amp_pair < amp_off_) {
    stopped_candidate_s_ += dt;
  } else {
    stopped_candidate_s_ = 0.0f;
  }

  if (stopped_candidate_s_ >= move_off_sec_) {
    motion_state_ = MOTION_STOPPED;
    stopped_candidate_s_ = 0.0f;
    stop_hold_elapsed_s_ = 0.0f;
  }
}

void Controller_SOGI::compute(const CtrlInput& in, CtrlOutput& out) {
  float dt = in.Ts;
  if (dt <= 0.0f) dt = 0.01f;

  // 速度低通滤波 + 自动相位补偿
  // 一阶 LPF 在信号频率 f 处引入相位滞后 arctan(f/fc) = atan2(wn, 2π·fc)
  // SOGI 已跟踪当前 wn，故可精确计算并通过 phi_lead 抵消，无需手动调整
  float velL, velR;
  float phi_comp_L = 0.0f, phi_comp_R = 0.0f;
  if (vel_lpf_fc_ > 0.1f) {
    float wc = TWO_PI_F * vel_lpf_fc_;
    float alpha_v = dt / (dt + 1.0f / wc);
    vel_filt_L_ += alpha_v * (in.LTAVx - vel_filt_L_);
    vel_filt_R_ += alpha_v * (in.RTAVx - vel_filt_R_);
    velL = vel_filt_L_;
    velR = vel_filt_R_;
    // 用上一步的 wn 估计相位滞后（wn 变化慢，误差可忽略）
    phi_comp_L = atan2f(L_.wn, wc);  // = arctan(f_signal / fc)
    phi_comp_R = atan2f(R_.wn, wc);
  } else {
    velL = in.LTAVx;
    velR = in.RTAVx;
  }

  // 高频过零防抖：速度在短时间内频繁过零 → 虚假信号 → 关断输出
  const float gate_zc_L = update_zc_tracker(zc_L_, velL, dt) ? 1.0f : 0.0f;
  const float gate_zc_R = update_zc_tracker(zc_R_, velR, dt) ? 1.0f : 0.0f;

  float sinL, ampL, sinR, ampR;
  step_sogi(L_, velL, dt, phi_lead_rad_ + phi_comp_L, sinL, ampL);
  step_sogi(R_, velR, dt, phi_lead_rad_ + phi_comp_R, sinR, ampR);

  // 双侧对称性检测: cos(φ_L - φ_R) = (x1_L·x1_R + x2_L·x2_R) / (ampL·ampR)
  // 步行→反相→cos≈-1；深蹲/STS→同相→cos≈+1
  // 仅在双侧都有足够幅值时更新，避免静止噪声干扰
  if (ampL > amp_min_ && ampR > amp_min_) {
    float cos_diff = (L_.x1 * R_.x1 + L_.x2 * R_.x2) / (ampL * ampR + 1e-6f);
    float alpha = dt / (dt + 0.5f);  // IIR 时间常数 0.5 s
    sym_score_ += alpha * (cos_diff - sym_score_);
  }
  // 迟滞切换：+0.4 判为双侧，-0.1 回到交替
  if (!is_bilateral_ && sym_score_ >  0.4f) is_bilateral_ = true;
  if ( is_bilateral_ && sym_score_ < -0.1f) is_bilateral_ = false;
  update_motion_state(ampL, ampR, dt);

  // 冷启动斜坡 (SOGI-FLL 需 1-2 周期锁定)
  ramp_elapsed_ += dt;
  float ramp = ramp_elapsed_ / SOGI_RAMP_SEC;
  if (ramp > 1.0f) ramp = 1.0f;

  const float gate_motion = (motion_state_ == MOTION_MOVING) ? 1.0f : 0.0f;

  // 幅值看门狗: 静止时不出力
  float gateL = (ampL > amp_min_) ? 1.0f : 0.0f;
  float gateR = (ampR > amp_min_) ? 1.0f : 0.0f;

  float tau_L, tau_R;
  if (is_bilateral_) {
    // 深蹲/STS：升正弦 (1+sin)/2，值域 [0,1]，覆盖完整周期，两端平滑过渡无硬截断
    // 不做幅值自适应，固定 A_gain 由 max_torque_cfg 限幅
    float raisedL = (1.0f + sinL) * 0.5f;
    float raisedR = (1.0f + sinR) * 0.5f;
    tau_L = A_gain_ * ramp * gate_motion * gate_zc_L * gateL * raisedL;
    tau_R = A_gain_ * ramp * gate_motion * gate_zc_R * gateR * raisedR;
  } else {
    // 步行：正弦 × 幅值自适应增益（SOGI_AMP_REF 对应满增益基准）
    float scaleL = ampL / SOGI_AMP_REF;
    float scaleR = ampR / SOGI_AMP_REF;
    tau_L = A_gain_ * scaleL * ramp * gate_motion * gate_zc_L * gateL * sinL;
    tau_R = A_gain_ * scaleR * ramp * gate_motion * gate_zc_R * gateR * sinR;
  }

  // 角度门控：双腿长时间停留在小角度窗时强制置零，抑制站立噪声
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
