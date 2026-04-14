#include "Controller_Samsung.h"
#include <math.h>

Controller_Samsung::Controller_Samsung() {
  kappa_         = 3.0f;
  base_delay_ms_ = 250.0f;
  reset();
}

void Controller_Samsung::reset() {
  memset(delay_buf_, 0, sizeof(delay_buf_));
  buf_idx_ = 0;
  ado_.reset(base_delay_ms_);
}

void Controller_Samsung::parse_params(const BleDownlinkData& dl) {
  kappa_ = dl.sam_kappa;
  if (kappa_ < 0.0f)  kappa_ = 0.0f;
  if (kappa_ > 20.0f) kappa_ = 20.0f;

  base_delay_ms_ = (float)dl.sam_delay_ms;
  if (base_delay_ms_ < 0.0f)    base_delay_ms_ = 0.0f;
  if (base_delay_ms_ > 1500.0f) base_delay_ms_ = 1500.0f;

  // 更新 auto delay 配置；rising edge 自动冷启动
  ado_.set_config(dl.auto_delay_enable, base_delay_ms_);
}

void Controller_Samsung::compute(const CtrlInput& in, CtrlOutput& out) {
  const float DEG2RAD = PI / 180.0f;

  // y = sin(q_R) - sin(q_L)  ← 施加 delay 前的原始信号
  float y_src = sinf(in.RTx * DEG2RAD) - sinf(in.LTx * DEG2RAD);
  float tau_src = kappa_ * y_src;

  // tau_src_R 语义上是 -tau_src（Samsung 两腿镜像）
  // 速度取对应腿的角速度 (deg/s)
  ado_.push_sample(tau_src, -tau_src, in.LTAVx, in.RTAVx);

  // 从 AutoDelayOptimizer 读取当前生效 delay（两腿可能不同）
  // 注：Samsung 算法单环形缓冲针对 y_src，用 L 侧 delay 驱动（两腿共用同一 y_src）
  // R 侧 delay 可能和 L 不同，但 y_src 是公共信号 ⇒ 用 L/R 均值作为实际 delay。
  // 这在 auto 关闭时 delay_L == delay_R，差异仅在 auto 运行后的漂移。
  float eff_delay_ms = 0.5f * (ado_.delay_ms_L + ado_.delay_ms_R);

  // 写入 y_src 到延迟缓冲
  delay_buf_[buf_idx_] = y_src;

  // 计算延迟样本数
  int delay_samples = (int)lrintf(eff_delay_ms / (1000.0f * in.Ts));
  if (delay_samples < 0) delay_samples = 0;
  if (delay_samples >= SAM_DELAY_BUF) delay_samples = SAM_DELAY_BUF - 1;

  // 读取延迟值
  int read_idx = buf_idx_ - delay_samples;
  if (read_idx < 0) read_idx += SAM_DELAY_BUF;

  float y_delayed = delay_buf_[read_idx];
  buf_idx_ = (buf_idx_ + 1) % SAM_DELAY_BUF;

  // 输出扭矩
  float tau = kappa_ * y_delayed;

  // 限幅
  float max_abs = fabsf(in.max_torque_cfg);
  if (tau >  max_abs) tau =  max_abs;
  if (tau < -max_abs) tau = -max_abs;

  // Samsung: 两腿同号反向 (左腿正=屈助力, 右腿负=伸助力)
  out.tau_L =  tau;
  out.tau_R = -tau;
}

void Controller_Samsung::tick_auto_delay(unsigned long now_us, float gait_freq_hz) {
  ado_.tick(now_us, gait_freq_hz);
}

void Controller_Samsung::fill_ble_status(uint8_t* buf40) const {
  ado_.fill_ble_status(buf40, 1.0f);  // Samsung 无独立 runtime_scale
}
