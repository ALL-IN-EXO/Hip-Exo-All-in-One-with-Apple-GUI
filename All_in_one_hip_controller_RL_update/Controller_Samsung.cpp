#include "Controller_Samsung.h"
#include <math.h>

Controller_Samsung::Controller_Samsung() {
  kappa_    = 3.0f;
  delay_ms_ = 250;
  reset();
}

void Controller_Samsung::reset() {
  memset(delay_buf_, 0, sizeof(delay_buf_));
  buf_idx_ = 0;
}

void Controller_Samsung::parse_params(const BleDownlinkData& dl) {
  kappa_    = dl.sam_kappa;
  delay_ms_ = dl.sam_delay_ms;
  // 限幅
  if (kappa_ < 0.0f)    kappa_ = 0.0f;
  if (kappa_ > 20.0f)   kappa_ = 20.0f;
  if (delay_ms_ < 0)    delay_ms_ = 0;
  if (delay_ms_ > 1500) delay_ms_ = 1500;
}

void Controller_Samsung::compute(const CtrlInput& in, CtrlOutput& out) {
  const float DEG2RAD = PI / 180.0f;

  // y = sin(q_R) - sin(q_L)
  float y = sinf(in.RTx * DEG2RAD) - sinf(in.LTx * DEG2RAD);

  // 写入环形缓冲
  delay_buf_[buf_idx_] = y;

  // 计算延迟样本数
  int delay_samples = (int)(delay_ms_ / (1000.0f * in.Ts));
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
