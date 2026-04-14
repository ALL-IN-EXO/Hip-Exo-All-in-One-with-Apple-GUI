#include "AutoDelayOptimizer.h"

// ============================================================
// 构造 / 重置
// ============================================================

AutoDelayOptimizer::AutoDelayOptimizer() {
  memset(tau_src_L_, 0, sizeof(tau_src_L_));
  memset(tau_src_R_, 0, sizeof(tau_src_R_));
  memset(vel_L_,     0, sizeof(vel_L_));
  memset(vel_R_,     0, sizeof(vel_R_));
  buf_head_    = 0;
  buf_count_   = 0;
  enabled      = false;
  prev_enabled_ = false;
  delay_ms_L   = 0.0f;
  delay_ms_R   = 0.0f;
  best_delay_ms_L = 0.0f;
  best_delay_ms_R = 0.0f;
  cur_ratio_L  = 0.0f;  cur_ratio_R  = 0.0f;
  cur_pos_per_s_L = 0.0f; cur_pos_per_s_R = 0.0f;
  cur_neg_per_s_L = 0.0f; cur_neg_per_s_R = 0.0f;
  motion_valid_L = false; motion_valid_R = false;
  last_eval_us_     = 0;
  last_change_us_L_ = 0;
  last_change_us_R_ = 0;
}

void AutoDelayOptimizer::reset(float base_delay_ms) {
  memset(tau_src_L_, 0, sizeof(tau_src_L_));
  memset(tau_src_R_, 0, sizeof(tau_src_R_));
  memset(vel_L_,     0, sizeof(vel_L_));
  memset(vel_R_,     0, sizeof(vel_R_));
  buf_head_    = 0;
  buf_count_   = 0;
  if (base_delay_ms < ADO_MIN_DELAY_MS) base_delay_ms = ADO_MIN_DELAY_MS;
  if (base_delay_ms > ADO_MAX_DELAY_MS) base_delay_ms = ADO_MAX_DELAY_MS;
  delay_ms_L   = base_delay_ms;
  delay_ms_R   = base_delay_ms;
  best_delay_ms_L = base_delay_ms;
  best_delay_ms_R = base_delay_ms;
  cur_ratio_L  = 0.0f;  cur_ratio_R  = 0.0f;
  cur_pos_per_s_L = 0.0f; cur_pos_per_s_R = 0.0f;
  cur_neg_per_s_L = 0.0f; cur_neg_per_s_R = 0.0f;
  motion_valid_L = false; motion_valid_R = false;
  last_eval_us_     = 0;
  last_change_us_L_ = 0;
  last_change_us_R_ = 0;
  prev_enabled_ = false;
  enabled = false;
}

void AutoDelayOptimizer::set_config(bool auto_en, float base_delay_ms) {
  // 检测 false→true 上升沿
  bool rising_edge = (auto_en && !prev_enabled_);
  prev_enabled_ = auto_en;
  enabled = auto_en;

  if (base_delay_ms < ADO_MIN_DELAY_MS) base_delay_ms = ADO_MIN_DELAY_MS;
  if (base_delay_ms > ADO_MAX_DELAY_MS) base_delay_ms = ADO_MAX_DELAY_MS;

  if (!auto_en) {
    // 关闭 auto: 两腿同步到 GUI 下发的基础 delay
    delay_ms_L = base_delay_ms;
    delay_ms_R = base_delay_ms;
    best_delay_ms_L = base_delay_ms;
    best_delay_ms_R = base_delay_ms;
  } else if (rising_edge) {
    // 刚打开: 同步到基础 delay，并重置 dwell 冷启动 + 清历史缓冲
    delay_ms_L = base_delay_ms;
    delay_ms_R = base_delay_ms;
    best_delay_ms_L = base_delay_ms;
    best_delay_ms_R = base_delay_ms;
    unsigned long now_us = micros();
    last_change_us_L_ = now_us;
    last_change_us_R_ = now_us;
    // 清历史，避免旧数据污染首轮评估
    memset(tau_src_L_, 0, sizeof(tau_src_L_));
    memset(tau_src_R_, 0, sizeof(tau_src_R_));
    memset(vel_L_,     0, sizeof(vel_L_));
    memset(vel_R_,     0, sizeof(vel_R_));
    buf_head_  = 0;
    buf_count_ = 0;
    last_eval_us_ = 0;
  }
}

// ============================================================
// push_sample — 每 100Hz tick 调用
// ============================================================
void AutoDelayOptimizer::push_sample(float tau_src_l, float tau_src_r,
                                      float vel_l_dps, float vel_r_dps) {
  tau_src_L_[buf_head_] = tau_src_l;
  tau_src_R_[buf_head_] = tau_src_r;
  vel_L_[buf_head_]     = vel_l_dps;
  vel_R_[buf_head_]     = vel_r_dps;
  buf_head_ = (buf_head_ + 1) % ADO_BUF_LEN;
  if (buf_count_ < ADO_BUF_LEN) buf_count_++;
}

// ============================================================
// compute_metrics — 对单腿在窗口内以指定 delay 计算功率指标
// ============================================================
AutoDelayOptimizer::LegMetrics AutoDelayOptimizer::compute_metrics(
    const float* tau_src, const float* vel,
    int delay_frames, int window_frames) const
{
  LegMetrics m;
  m.valid = false;
  m.ratio = 0.0f;
  m.pos_per_s = 0.0f;
  m.neg_per_s = 0.0f;

  if (window_frames <= 0 || buf_count_ < window_frames + delay_frames) return m;

  const float DEG2RAD = 3.14159265f / 180.0f;
  const float dt = 1.0f / ADO_CTRL_HZ;

  float pos_work = 0.0f;
  float neg_work = 0.0f;
  float sum_abs_vel = 0.0f;
  float sum_abs_pwr = 0.0f;

  // 遍历窗口内每帧：读 (i) 时刻的 vel 和 (i - delay_frames) 时刻的 tau_src
  for (int i = 0; i < window_frames; i++) {
    // vel 下标：从 buf_head_ 往前数 (i+1) 帧（最近的在 buf_head_-1）
    int v_idx = buf_head_ - 1 - i;
    if (v_idx < 0) v_idx += ADO_BUF_LEN;

    // tau_src 下标：再往前 delay_frames 帧
    int t_idx = v_idx - delay_frames;
    if (t_idx < 0) t_idx += ADO_BUF_LEN;

    float v   = vel[v_idx];
    float tau = tau_src[t_idx];
    float pwr = tau * v * DEG2RAD;

    sum_abs_vel += fabsf(v);
    sum_abs_pwr += fabsf(pwr);

    if (pwr > 0.0f) pos_work += pwr * dt;
    else             neg_work += pwr * dt;  // neg_work <= 0
  }

  float T_total = (float)window_frames * dt;
  float mean_abs_vel = sum_abs_vel / (float)window_frames;
  float abs_pwr_per_s = sum_abs_pwr / T_total;

  // 运动有效门控（scale=1.0，与 runtime_scale 解耦）
  if (mean_abs_vel < ADO_VALID_MIN_VEL_DPS || abs_pwr_per_s < ADO_VALID_MIN_ABS_PWR) {
    return m;  // invalid
  }

  float total_work = pos_work + fabsf(neg_work);
  m.ratio     = (total_work > 1e-9f) ? (pos_work / total_work) : 0.5f;
  m.pos_per_s = pos_work / T_total;
  m.neg_per_s = neg_work / T_total;  // <= 0
  m.valid     = true;
  return m;
}

// ============================================================
// scan_leg — 局部扫描 + pick_best + 限步长
// ============================================================
float AutoDelayOptimizer::scan_leg(
    const float* tau_src, const float* vel,
    float cur_delay, int window_frames,
    LegMetrics& out_metrics, float& out_best,
    bool& out_motion_valid) const
{
  const float dt_ms = 1000.0f / ADO_CTRL_HZ;

  // 计算当前 delay 的指标
  int cur_frames = (int)lrintf(cur_delay / dt_ms);
  if (cur_frames < 0) cur_frames = 0;
  out_metrics = compute_metrics(tau_src, vel, cur_frames, window_frames);
  out_motion_valid = out_metrics.valid;

  if (!out_metrics.valid) {
    out_best = cur_delay;
    return cur_delay;  // 运动无效，不调参
  }

  // 扫描候选
  const int N_STEPS = (int)lrintf(ADO_SCAN_HALF_MS / ADO_SCAN_STEP_MS);
  const int N_CANDS = 2 * N_STEPS + 1;

  float best_delay_good = cur_delay;   // 候选集中 ratio >= 0.95 的最优
  float best_pos_good   = -1e9f;
  bool  found_good      = false;

  float best_delay_fall = cur_delay;   // fallback: 全局最大 ratio
  float best_ratio_fall = -1e9f;

  for (int k = -N_STEPS; k <= N_STEPS; k++) {
    float cand_delay = cur_delay + (float)k * ADO_SCAN_STEP_MS;
    if (cand_delay < ADO_MIN_DELAY_MS) cand_delay = ADO_MIN_DELAY_MS;
    if (cand_delay > ADO_MAX_DELAY_MS) cand_delay = ADO_MAX_DELAY_MS;

    int cand_frames = (int)lrintf(cand_delay / dt_ms);
    if (cand_frames < 0) cand_frames = 0;

    LegMetrics cm = compute_metrics(tau_src, vel, cand_frames, window_frames);
    if (!cm.valid) continue;

    // fallback track
    if (cm.ratio > best_ratio_fall) {
      best_ratio_fall = cm.ratio;
      best_delay_fall = cand_delay;
    }
    // primary track
    if (cm.ratio >= ADO_TARGET_RATIO) {
      if (cm.pos_per_s > best_pos_good) {
        best_pos_good = cm.pos_per_s;
        best_delay_good = cand_delay;
        found_good = true;
      }
    }
  }

  float raw_best = found_good ? best_delay_good : best_delay_fall;
  out_best = raw_best;

  // 限步长
  float diff = raw_best - cur_delay;
  if (diff >  ADO_MAX_STEP_MS) diff =  ADO_MAX_STEP_MS;
  if (diff < -ADO_MAX_STEP_MS) diff = -ADO_MAX_STEP_MS;

  float new_delay = cur_delay + diff;
  if (new_delay < ADO_MIN_DELAY_MS) new_delay = ADO_MIN_DELAY_MS;
  if (new_delay > ADO_MAX_DELAY_MS) new_delay = ADO_MAX_DELAY_MS;
  return new_delay;
}

// ============================================================
// tick — 1Hz 侧环
// ============================================================
void AutoDelayOptimizer::tick(unsigned long now_us, float gait_freq_hz) {
  // Always run 1Hz metric evaluation even when auto is disabled so the GUI shows
  // live power ratio. Only the "apply new delay" step is gated by enabled.

  // 1Hz 节奏
  if (last_eval_us_ != 0 &&
      (now_us - last_eval_us_) < (unsigned long)(ADO_UPDATE_S * 1e6f)) {
    return;
  }
  last_eval_us_ = now_us;

  // 自适应窗口长度（固定 dwell；AUTO_DWELL_ADAPTIVE 未在 Teensy 侧暴露，默认 False）
  float window_s = ADO_WINDOW_FALLBACK_S;
  if (gait_freq_hz > 0.3f) {
    window_s = ADO_WINDOW_CYCLES / gait_freq_hz;
    if (window_s < ADO_WINDOW_MIN_S) window_s = ADO_WINDOW_MIN_S;
    if (window_s > ADO_WINDOW_MAX_S) window_s = ADO_WINDOW_MAX_S;
  }
  int window_frames = (int)lrintf(window_s * ADO_CTRL_HZ);
  if (window_frames > ADO_BUF_LEN) window_frames = ADO_BUF_LEN;

  const unsigned long dwell_us = (unsigned long)(ADO_DWELL_S * 1e6f);

  // ---- 左腿 ----
  {
    bool dwell_ok = (last_change_us_L_ == 0) ||
                    ((now_us - last_change_us_L_) >= dwell_us);
    LegMetrics cur_m;
    float raw_best;
    bool  mv;
    float new_delay = scan_leg(tau_src_L_, vel_L_,
                                delay_ms_L, window_frames,
                                cur_m, raw_best, mv);
    motion_valid_L = mv;
    if (cur_m.valid) {
      cur_ratio_L     = cur_m.ratio;
      cur_pos_per_s_L = cur_m.pos_per_s;
      cur_neg_per_s_L = cur_m.neg_per_s;
    }
    best_delay_ms_L = raw_best;

    if (enabled && mv && dwell_ok && fabsf(new_delay - delay_ms_L) > 0.5f) {
      delay_ms_L = new_delay;
      last_change_us_L_ = now_us;
    }
  }

  // ---- 右腿 ----
  {
    bool dwell_ok = (last_change_us_R_ == 0) ||
                    ((now_us - last_change_us_R_) >= dwell_us);
    LegMetrics cur_m;
    float raw_best;
    bool  mv;
    float new_delay = scan_leg(tau_src_R_, vel_R_,
                                delay_ms_R, window_frames,
                                cur_m, raw_best, mv);
    motion_valid_R = mv;
    if (cur_m.valid) {
      cur_ratio_R     = cur_m.ratio;
      cur_pos_per_s_R = cur_m.pos_per_s;
      cur_neg_per_s_R = cur_m.neg_per_s;
    }
    best_delay_ms_R = raw_best;

    if (enabled && mv && dwell_ok && fabsf(new_delay - delay_ms_R) > 0.5f) {
      delay_ms_R = new_delay;
      last_change_us_R_ = now_us;
    }
  }
}

// ============================================================
// fill_ble_status — 填充 40B v3 BLE 上行 RPi 透传槽
// ============================================================
void AutoDelayOptimizer::fill_ble_status(uint8_t* buf40,
                                          float runtime_scale) const {
  memset(buf40, 0, 40);

  buf40[0] = 0x52;  // 'R'
  buf40[1] = 0x4C;  // 'L'
  buf40[2] = ADO_BLE_VERSION;  // 0x03
  buf40[3] = ADO_BLE_SRC_TEENSY;  // 0xFE: Teensy-native source (not RPi)
  // buf40[4..15]: filter/scale fields — not applicable for Teensy-native, leave 0

  // [16..17] delay_ms_L ×10
  ado_put_i16(buf40, 16, delay_ms_L,   10.0f);
  // [18..19] delay_ms_R ×10
  ado_put_i16(buf40, 18, delay_ms_R,   10.0f);

  // [20] auto_flags
  uint8_t flags = 0;
  if (enabled)        flags |= 0x01;
  if (motion_valid_L) flags |= 0x02;
  if (motion_valid_R) flags |= 0x04;
  buf40[20] = flags;
  // [21..23] reserved = 0

  // [24..25] ratio_L ×10000, [26..27] ratio_R ×10000
  ado_put_i16(buf40, 24, cur_ratio_L, 10000.0f);
  ado_put_i16(buf40, 26, cur_ratio_R, 10000.0f);

  // [28..29] pos_per_s_L ×100, [30..31] pos_per_s_R ×100
  ado_put_i16(buf40, 28, cur_pos_per_s_L * runtime_scale, 100.0f);
  ado_put_i16(buf40, 30, cur_pos_per_s_R * runtime_scale, 100.0f);

  // [32..33] neg_per_s_L ×100, [34..35] neg_per_s_R ×100
  ado_put_i16(buf40, 32, cur_neg_per_s_L * runtime_scale, 100.0f);
  ado_put_i16(buf40, 34, cur_neg_per_s_R * runtime_scale, 100.0f);

  // [36..37] best_delay_L ×10, [38..39] best_delay_R ×10
  ado_put_i16(buf40, 36, best_delay_ms_L, 10.0f);
  ado_put_i16(buf40, 38, best_delay_ms_R, 10.0f);
}
