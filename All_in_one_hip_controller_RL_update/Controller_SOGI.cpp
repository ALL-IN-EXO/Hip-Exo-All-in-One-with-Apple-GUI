#include "Controller_SOGI.h"
#include <math.h>

// === 硬编码常量 (见 Docs/SOGI-FLL-CONTROLLER.md §6) ===
static constexpr float SOGI_F0_HZ    = 1.0f;    // FLL 初值，自适应
static constexpr float SOGI_K        = 1.41f;   // √2 阻尼
static constexpr float SOGI_GAMMA    = 2.0f;   // FLL 学习率
static constexpr float SOGI_F_MIN    = 0.3f;    // 频率夹紧下限 (Hz)
static constexpr float SOGI_F_MAX    = 3.5f;    // 频率夹紧上限 (Hz)
static constexpr float SOGI_RAMP_SEC = 0.5f;    // 冷启动斜坡时长
// 运动状态机默认值（可由 GUI 通过 BLE 覆盖）
static constexpr float SOGI_AMP_ON_DEFAULT       = 22.0f;  // STOPPED->MOVING
static constexpr float SOGI_AMP_OFF_DEFAULT      = 18.0f;  // MOVING->STOPPED
static constexpr float SOGI_MOVE_ON_SEC_DEFAULT  = 0.15f;  // amp > AMP_ON 持续时间
static constexpr float SOGI_MOVE_OFF_SEC_DEFAULT = 0.20f;  // amp < AMP_OFF 持续时间
static constexpr float SOGI_STOP_HOLD_SEC  = 0.40f;  // STOPPED 最短保持时间
// 过零频率防抖：过零过快判定为虚假过零
static constexpr float SOGI_ZC_MIN_VEL_DEG_S     = 6.0f;   // 忽略极小幅值抖动
static constexpr float SOGI_ZC_FREQ_MAX_HZ       = 3.0f;   // 超过该步频视为异常
static constexpr float SOGI_ZC_FAKE_WIN_SEC      = 0.30f;  // 假过零统计窗
static constexpr uint8_t SOGI_ZC_FAKE_COUNT_TRIP = 2;      // 窗内假过零触发次数
static constexpr float SOGI_ZC_FAKE_HOLD_SEC     = 0.35f;  // 触发后输出关断时间
// 站立门控：双腿角度长时间处于小角度窗时，强制 τ=0，抑制站立噪声触发
static constexpr float SOGI_STAND_ANGLE_DEG  = 10.0f;   // 站立角度窗 (deg)
static constexpr float SOGI_STAND_HOLD_SEC   = 0.25f;  // 连续保持时长 (s)
static constexpr float TWO_PI_F      = 6.28318530718f;
static constexpr float SOGI_AMP_REF  = 200.0f;  // 参考角速度幅值 (deg/s)，步行模式力矩按此比例缩放

Controller_SOGI::Controller_SOGI() {
  A_gain_        = 0.0f;
  phi_lead_rad_  = 0.0f;
  amp_min_       = 20.0f;
  amp_on_        = SOGI_AMP_ON_DEFAULT;
  amp_off_       = SOGI_AMP_OFF_DEFAULT;
  move_on_sec_   = SOGI_MOVE_ON_SEC_DEFAULT;
  move_off_sec_  = SOGI_MOVE_OFF_SEC_DEFAULT;
  filter_fc_hz_  = 5.0f;
  filter_vel_on_ = false;
  filter_butter_ = true;
  sym_score_     = 0.0f;
  is_bilateral_  = false;
  stand_hold_elapsed_s_ = 0.0f;
  amp_peak_L_ = amp_peak_R_ = 0.0f;
  amp_hold_L_ = amp_hold_R_ = SOGI_AMP_REF;  // 首周期用参考值避免增益为零
  sin_prev_L_ = sin_prev_R_ = 0.0f;
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
  sym_score_    = 0.0f;
  is_bilateral_ = false;
  stand_hold_elapsed_s_ = 0.0f;
  motion_state_ = MOTION_STOPPED;
  moving_candidate_s_ = 0.0f;
  stopped_candidate_s_ = 0.0f;
  stop_hold_elapsed_s_ = 0.0f;
  reset_zc_tracker(zc_L_);
  reset_zc_tracker(zc_R_);
  amp_peak_L_ = amp_peak_R_ = 0.0f;
  amp_hold_L_ = amp_hold_R_ = SOGI_AMP_REF;
  sin_prev_L_ = sin_prev_R_ = 0.0f;
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

  // 全局速度滤波参数（无论 has_ext_sogi_params 都更新）
  filter_fc_hz_  = dl.filter_fc_hz > 0.0f ? dl.filter_fc_hz : 5.0f;
  filter_vel_on_ = (dl.filter_flags & 0x02) != 0;  // bit1 = 速度滤波
  filter_butter_ = (dl.filter_flags & 0x08) != 0;

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

void Controller_SOGI::reset_zc_tracker(ZcTracker& z) {
  z.prev_v = 0.0f;
  z.since_last_cross_s = 1.0f;
  z.fake_window_elapsed_s = 0.0f;
  z.hold_elapsed_s = 0.0f;
  z.fake_count_in_window = 0;
  z.initialized = false;
}

// 返回 true 表示当前处于关断保持期（输出应置零）
bool Controller_SOGI::update_zc_tracker(ZcTracker& z, float v, float dt) {
  z.since_last_cross_s += dt;
  z.fake_window_elapsed_s += dt;
  if (z.fake_window_elapsed_s >= SOGI_ZC_FAKE_WIN_SEC) {
    z.fake_window_elapsed_s = 0.0f;
    z.fake_count_in_window = 0;
  }
  if (z.hold_elapsed_s > 0.0f) {
    z.hold_elapsed_s -= dt;
    if (z.hold_elapsed_s < 0.0f) z.hold_elapsed_s = 0.0f;
  }

  if (!z.initialized) {
    z.prev_v = v;
    z.initialized = true;
    return z.hold_elapsed_s > 0.0f;
  }

  const bool crossed =
      ((z.prev_v <= 0.0f) && (v > 0.0f)) ||
      ((z.prev_v >= 0.0f) && (v < 0.0f));

  if (crossed) {
    const float abs_peak = fmaxf(fabsf(z.prev_v), fabsf(v));
    const float dt_cross = fmaxf(z.since_last_cross_s, 1e-3f);
    z.since_last_cross_s = 0.0f;

    if (abs_peak >= SOGI_ZC_MIN_VEL_DEG_S) {
      const float f_cross_hz = 0.5f / dt_cross;
      if (f_cross_hz > SOGI_ZC_FREQ_MAX_HZ) {
        if (z.fake_count_in_window < 255) z.fake_count_in_window++;
        if (z.fake_count_in_window >= SOGI_ZC_FAKE_COUNT_TRIP) {
          z.hold_elapsed_s = SOGI_ZC_FAKE_HOLD_SEC;
        }
      } else {
        z.fake_count_in_window = 0;
      }
    }
  }

  z.prev_v = v;
  return z.hold_elapsed_s > 0.0f;
}

void Controller_SOGI::compute(const CtrlInput& in, CtrlOutput& out) {
  float dt = in.Ts;
  if (dt <= 0.0f) dt = 0.01f;

  // 速度滤波引入的相位滞后补偿
  float phi_lead_eff = phi_lead_rad_;
  if (filter_vel_on_ && filter_fc_hz_ > 0.0f) {
    const float f_hz = fmaxf(L_.wn, R_.wn) / TWO_PI_F;
    const float r = f_hz / filter_fc_hz_;
    float lag;
    if (filter_butter_) {
      lag = atan2f(1.41421f * r, 1.0f - r * r);
    } else {
      lag = atanf(r);
    }
    phi_lead_eff += lag;
  }

  float sinL, ampL, sinR, ampR;
  step_sogi(L_, in.LTAVx, dt, phi_lead_eff, sinL, ampL);
  step_sogi(R_, in.RTAVx, dt, phi_lead_eff, sinR, ampR);

  // 双侧对称性检测: cos(φ_L - φ_R) = (x1_L·x1_R + x2_L·x2_R) / (ampL·ampR)
  // 步行→反相→cos≈-1；深蹲/STS→同相→cos≈+1
  if (ampL > amp_min_ && ampR > amp_min_) {
    float cos_diff = (L_.x1 * R_.x1 + L_.x2 * R_.x2) / (ampL * ampR + 1e-6f);
    float alpha = dt / (dt + 0.5f);  // IIR 时间常数 0.5 s
    sym_score_ += alpha * (cos_diff - sym_score_);
  }
  if (!is_bilateral_ && sym_score_ >  0.4f) is_bilateral_ = true;
  if ( is_bilateral_ && sym_score_ < -0.1f) is_bilateral_ = false;

  update_motion_state(ampL, ampR, dt);
  const bool gate_zc_L = !update_zc_tracker(zc_L_, L_.x1, dt);
  const bool gate_zc_R = !update_zc_tracker(zc_R_, R_.x1, dt);

  ramp_elapsed_ += dt;
  float ramp = ramp_elapsed_ / SOGI_RAMP_SEC;
  if (ramp > 1.0f) ramp = 1.0f;

  const float gate_motion = (motion_state_ == MOTION_MOVING) ? 1.0f : 0.0f;
  // 双条件门控：amp_hold 防止噪声抖动，瞬时 amp 兜底快速关断（停止后立即生效）
  const float gateL = (amp_hold_L_ > amp_min_ && ampL > amp_min_ * 0.5f) ? 1.0f : 0.0f;
  const float gateR = (amp_hold_R_ > amp_min_ && ampR > amp_min_ * 0.5f) ? 1.0f : 0.0f;
  const float gzL = gate_zc_L ? 1.0f : 0.0f;
  const float gzR = gate_zc_R ? 1.0f : 0.0f;

  // 步行模式：逐周期幅值保持
  // 在 sinL/sinR 正向过零时更新 amp_hold（此时力矩=0，增益切换无突变）
  if (!is_bilateral_) {
    if (sin_prev_L_ <= 0.0f && sinL > 0.0f) {
      amp_hold_L_ = (amp_peak_L_ > 0.0f) ? amp_peak_L_ : SOGI_AMP_REF;
      amp_peak_L_ = 0.0f;
    }
    if (sin_prev_R_ <= 0.0f && sinR > 0.0f) {
      amp_hold_R_ = (amp_peak_R_ > 0.0f) ? amp_peak_R_ : SOGI_AMP_REF;
      amp_peak_R_ = 0.0f;
    }
    if (ampL > amp_peak_L_) amp_peak_L_ = ampL;
    if (ampR > amp_peak_R_) amp_peak_R_ = ampR;
  }
  sin_prev_L_ = sinL;
  sin_prev_R_ = sinR;

  float tau_L, tau_R;
  if (is_bilateral_) {
    // 深蹲/STS：升正弦 (1+sin)/2，绕开 gate_motion
    // 用软幅值门控：amp 从 0→amp_on_ 线性升为 1，随运动平滑启动，无突变
    float amp_now = fmaxf(ampL, ampR);
    float soft_gate = fminf(1.0f, amp_now / fmaxf(amp_on_, 1e-3f));
    float raisedL = (1.0f + sinL) * 0.5f;
    float raisedR = (1.0f + sinR) * 0.5f;
    tau_L = A_gain_ * ramp * soft_gate * gzL * raisedL;
    tau_R = A_gain_ * ramp * soft_gate * gzR * raisedR;
  } else {
    // 步行：用上一周期峰值幅值缩放，周期内固定，力矩为纯正弦
    float scaleL = amp_hold_L_ / SOGI_AMP_REF;
    float scaleR = amp_hold_R_ / SOGI_AMP_REF;
    tau_L = A_gain_ * scaleL * ramp * gate_motion * gzL * gateL * sinL;
    tau_R = A_gain_ * scaleR * ramp * gate_motion * gzR * gateR * sinR;
  }

  // 角度门控：双腿长时间处于小角度窗时强制置零（抑制站立噪声触发）
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

  ado_.push_sample(tau_L, tau_R, in.LTAVx, in.RTAVx);
}

void Controller_SOGI::tick_auto_delay(unsigned long now_us, float gait_freq_hz) {
  // ADO::tick 即使 enabled=false 也会更新 ratio/pos/neg 指标；仅跳过 delay 应用
  ado_.tick(now_us, gait_freq_hz);
}

void Controller_SOGI::fill_ble_status(uint8_t* buf40) const {
  ado_.fill_ble_status(buf40, 1.0f);
}
