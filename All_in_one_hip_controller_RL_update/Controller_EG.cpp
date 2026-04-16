#include "Controller_EG.h"
#include <math.h>

Controller_EG::Controller_EG() {
  // 默认参数
  Rescaling_gain_   = 1.0f;
  Flex_Assist_gain_ = 1.0f;
  Ext_Assist_gain_  = 1.0f;
  // GUI delay index uses gait-phase semantics (not absolute ms):
  // index is calibrated at BASE_FREQ=0.7Hz and auto-scales with current gait frequency.
  Assist_delay_gain_= 40.0f;
  phase_offset_L_   = 0;
  phase_offset_R_   = 0;
  gate_k_           = 1.0f;
  gate_p_on_        = 8.0f;
  lead_frac_        = 0.06f;
  ext_phase_frac_L_ = 0.3f;
  ext_phase_frac_R_ = 0.3f;
  ext_gain_         = 0.5f;
  scale_all_        = 0.2f;
  post_delay_ms_base_ = 0.0f;

  reset();
}

void Controller_EG::reset() {
  memset(RLTx_delay_, 0, sizeof(RLTx_delay_));
  memset(flexL_hist_, 0, sizeof(flexL_hist_));
  memset(flexR_hist_, 0, sizeof(flexR_hist_));
  memset(post_buf_L_, 0, sizeof(post_buf_L_));
  memset(post_buf_R_, 0, sizeof(post_buf_R_));
  doi_         = 0;
  ext_i_       = 0;
  post_buf_idx_ = 0;
  xL_prev_ = 0.0f;
  xR_prev_ = 0.0f;
  tau_cmd_L_filt_ = 0.0f;
  tau_cmd_R_filt_ = 0.0f;
  ado_.reset(post_delay_ms_base_);
}

void Controller_EG::parse_params(const BleDownlinkData& dl) {
  // GUI 默认会发 Rescaling=0（历史占位值），这里按 1.0 处理为”中性缩放”。
  Rescaling_gain_    = dl.Rescaling_gain;
  if (Rescaling_gain_ <= 0.0f) Rescaling_gain_ = 1.0f;
  if (Rescaling_gain_ > 10.0f) Rescaling_gain_ = 10.0f;

  Flex_Assist_gain_  = dl.Flex_Assist_gain;
  if (Flex_Assist_gain_ < -3.0f) Flex_Assist_gain_ = -3.0f;
  if (Flex_Assist_gain_ >  3.0f) Flex_Assist_gain_ =  3.0f;

  Ext_Assist_gain_   = dl.Ext_Assist_gain;
  if (Ext_Assist_gain_ < -3.0f) Ext_Assist_gain_ = -3.0f;
  if (Ext_Assist_gain_ >  3.0f) Ext_Assist_gain_ =  3.0f;

  Assist_delay_gain_ = dl.Assist_delay_gain;
  if (Assist_delay_gain_ < 0.0f)  Assist_delay_gain_ = 0.0f;
  if (Assist_delay_gain_ > 99.0f) Assist_delay_gain_ = 99.0f;
  phase_offset_L_    = dl.phase_offset_L;
  phase_offset_R_    = dl.phase_offset_R;
  gate_k_            = dl.gate_k;
  gate_p_on_         = dl.gate_p_on;
  lead_frac_         = dl.lead_frac;
  ext_phase_frac_L_  = dl.ext_phase_frac_L;
  ext_phase_frac_R_  = dl.ext_phase_frac_R;
  ext_gain_          = dl.ext_gain;
  scale_all_         = dl.scale_all;

  post_delay_ms_base_ = (float)dl.eg_post_delay_ms;
  if (post_delay_ms_base_ < 0.0f)    post_delay_ms_base_ = 0.0f;
  if (post_delay_ms_base_ > 1500.0f) post_delay_ms_base_ = 1500.0f;

  // 更新 auto delay 配置；rising edge 自动冷启动
  ado_.set_config(dl.auto_delay_enable, post_delay_ms_base_);
}

float Controller_EG::smooth_gate_p(float x, float k, float p_on) {
  return 0.5f * (tanhf(k * (x - p_on)) + 1.0f);
}

float Controller_EG::lead_predict(float x, float x_prev, float lead_ms, float Ts) {
  const float lead_s = 0.001f * lead_ms;
  float dx = (x - x_prev) / Ts;
  const float DX_MAX = 200.0f;
  if (dx >  DX_MAX) dx =  DX_MAX;
  if (dx < -DX_MAX) dx = -DX_MAX;
  return x + lead_s * dx;
}

void Controller_EG::compute(const CtrlInput& in, CtrlOutput& out) {
  const float Ts = in.Ts;

  // 差分角度
  float RLTx_filtered = (in.RTx_filtered - in.LTx_filtered) * Rescaling_gain_;

  // 延迟缓冲
  RLTx_delay_[doi_] = RLTx_filtered;

  // 动态延迟（按步态周期比例）:
  // Assist_delay_gain_ 是 phase index（0..99），在 BASE_FREQ 下定义，
  // 通过 BASE_FREQ / gait_freq 自动换算，保持“相位比例”基本一致。
  const float BASE_FREQ = 0.7f;
  float f_for_delay = (in.gait_inited && in.gait_freq > 0.05f) ? in.gait_freq : BASE_FREQ;
  int Assist_delay_dynamic = (int)lrintf(Assist_delay_gain_ * (BASE_FREQ / f_for_delay));
  if (Assist_delay_dynamic < 0) Assist_delay_dynamic = 0;
  if (Assist_delay_dynamic >= EG_DELAY_BUF) Assist_delay_dynamic = EG_DELAY_BUF - 1;

  int delayindex = doi_ - Assist_delay_dynamic;
  if (delayindex < 0)             delayindex += EG_DELAY_BUF;
  else if (delayindex >= EG_DELAY_BUF) delayindex -= EG_DELAY_BUF;

  int idx_L = (delayindex + phase_offset_L_ + EG_DELAY_BUF) % EG_DELAY_BUF;
  int idx_R = (delayindex + phase_offset_R_ + EG_DELAY_BUF) % EG_DELAY_BUF;
  doi_ = (doi_ + 1) % EG_DELAY_BUF;

  const float phL = RLTx_delay_[idx_L];
  const float phR = RLTx_delay_[idx_R];

  float tau_raw_L = 0.0f, tau_raw_R = 0.0f;
  if (fabsf(phL) <= 120.0f)
    tau_raw_L = (phL >= 0.0f ? -1.0f : +1.0f) * phL;
  if (fabsf(phR) <= 120.0f)
    tau_raw_R = (phR >= 0.0f ? +1.0f : -1.0f) * phR;

  // 功率估算
  const float LTx_vel = (in.LTx_filtered - in.LTx_filtered_last) * (PI / 180.0f) / Ts;
  const float RTx_vel = (in.RTx_filtered - in.RTx_filtered_last) * (PI / 180.0f) / Ts;
  const float xL_raw = tau_raw_L * LTx_vel;
  const float xR_raw = tau_raw_R * (-RTx_vel);

  // 提前补偿
  float lead_ms = 35.0f;
  if (in.gait_inited && in.gait_freq > 0.01f) {
    float T_gait_ms = 1000.0f / in.gait_freq;
    lead_ms = lead_frac_ * T_gait_ms;
  }
  if (lead_ms < 5.0f)   lead_ms = 5.0f;
  if (lead_ms > 120.0f) lead_ms = 120.0f;

  const float xL_pred = lead_predict(xL_raw, xL_prev_, lead_ms, Ts);
  const float xR_pred = lead_predict(xR_raw, xR_prev_, lead_ms, Ts);
  xL_prev_ = xL_raw;
  xR_prev_ = xR_raw;

  // 门控
  const float gate_L = smooth_gate_p(xL_pred, gate_k_, gate_p_on_);
  const float gate_R = smooth_gate_p(xR_pred, gate_k_, gate_p_on_);
  float tau_gate_L = tau_raw_L * gate_L;
  float tau_gate_R = tau_raw_R * gate_R;

  // 低通
  tau_cmd_L_filt_ = 0.85f * tau_cmd_L_filt_ + 0.15f * tau_gate_L;
  tau_cmd_R_filt_ = 0.85f * tau_cmd_R_filt_ + 0.15f * tau_gate_R;

  float S_src_L = tau_cmd_L_filt_ * scale_all_;
  float S_src_R = tau_cmd_R_filt_ * scale_all_;

  // === 伸展复制 ===
  const int8_t flex_sign_L = -1;
  const int8_t flex_sign_R = +1;

  auto keep_if_flex = [](float S, int8_t sign) {
    return ((sign > 0) ? (S > 0.0f) : (S < 0.0f)) ? S : 0.0f;
  };
  float flexL_now = keep_if_flex(S_src_L, flex_sign_L);
  float flexR_now = keep_if_flex(S_src_R, flex_sign_R);

  int w = ext_i_ % EG_EXT_BUF;
  flexL_hist_[w] = flexL_now;
  flexR_hist_[w] = flexR_now;

  float S_L_ext = 0.0f, S_R_ext = 0.0f;
  if (in.gait_freq > 0.01f) {
    float T_gait_ms = 1000.0f / in.gait_freq;
    float ext_ms_L = ext_phase_frac_L_ * T_gait_ms;
    float ext_ms_R = ext_phase_frac_R_ * T_gait_ms;

    auto clamp = [](float x, float a, float b) { return (x<a)?a:((x>b)?b:x); };
    ext_ms_L = clamp(ext_ms_L, 30.0f, 1200.0f);
    ext_ms_R = clamp(ext_ms_R, 30.0f, 1200.0f);

    int extN_L = (int)lrintf(ext_ms_L / (1000.0f * Ts));
    int extN_R = (int)lrintf(ext_ms_R / (1000.0f * Ts));
    if (extN_L >= EG_EXT_BUF) extN_L = EG_EXT_BUF - 1;
    if (extN_R >= EG_EXT_BUF) extN_R = EG_EXT_BUF - 1;
    if (extN_L < 1) extN_L = 1;
    if (extN_R < 1) extN_R = 1;

    int rL = ext_i_ - extN_L;
    int rR = ext_i_ - extN_R;
    if (rL < 0) rL += ((-rL / EG_EXT_BUF) + 1) * EG_EXT_BUF;
    if (rR < 0) rR += ((-rR / EG_EXT_BUF) + 1) * EG_EXT_BUF;
    rL %= EG_EXT_BUF;
    rR %= EG_EXT_BUF;

    S_L_ext = -ext_gain_ * flexL_hist_[rL];
    S_R_ext = -ext_gain_ * flexR_hist_[rR];
  }

  float S_L = S_src_L + S_L_ext;
  float S_R = S_src_R + S_R_ext;

  // GUI 的 R/L gain 分别作用于右/左腿最终输出。
  S_L *= Ext_Assist_gain_;
  S_R *= Flex_Assist_gain_;

  // 步频缩放
  S_L *= in.gait_freq * 1.2f;
  S_R *= in.gait_freq * 1.2f;

  ext_i_++;

  // 限幅
  float max_abs = fabsf(in.max_torque_cfg);
  if (S_L >  max_abs) S_L =  max_abs;
  if (S_L < -max_abs) S_L = -max_abs;
  if (S_R >  max_abs) S_R =  max_abs;
  if (S_R < -max_abs) S_R = -max_abs;

  // === 后处理延迟缓冲 + AutoDelayOptimizer 数据推入 ===
  // tau_src (pre-post-delay) 与 IMU 角速度一起送入 ADO ring buffer
  // 注意：out.tau_R 最终取反，故此处推入 -S_R 使 ADO 功率计算与实际输出一致
  ado_.push_sample(S_L, -S_R, in.LTAVx, in.RTAVx);

  // 写入后处理延迟缓冲
  post_buf_L_[post_buf_idx_] = S_L;
  post_buf_R_[post_buf_idx_] = S_R;

  // 读取各腿延迟输出（delay_ms_L/R 由 ADO 管理，auto=off 时等于 base_delay_ms）
  auto read_post = [&](const float* buf, float delay_ms) -> float {
    int d_frames = (int)lrintf(delay_ms / (1000.0f * in.Ts));
    if (d_frames < 0) d_frames = 0;
    if (d_frames >= EG_POST_DELAY_BUF) d_frames = EG_POST_DELAY_BUF - 1;
    int ridx = post_buf_idx_ - d_frames;
    if (ridx < 0) ridx += EG_POST_DELAY_BUF;
    return buf[ridx];
  };

  float tau_L_delayed = read_post(post_buf_L_, ado_.delay_ms_L);
  float tau_R_delayed = read_post(post_buf_R_, ado_.delay_ms_R);

  post_buf_idx_ = (post_buf_idx_ + 1) % EG_POST_DELAY_BUF;

  out.tau_L = tau_L_delayed;
  out.tau_R = -tau_R_delayed;
}

void Controller_EG::tick_auto_delay(unsigned long now_us, float gait_freq_hz) {
  ado_.tick(now_us, gait_freq_hz);
}

void Controller_EG::fill_ble_status(uint8_t* buf40) const {
  ado_.fill_ble_status(buf40, 1.0f);
}
