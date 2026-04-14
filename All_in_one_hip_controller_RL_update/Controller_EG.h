#ifndef CONTROLLER_EG_H
#define CONTROLLER_EG_H

/************************************************************
 * Controller_EG — Energy Gate 算法
 *
 * 原理: 功率门控助力 + 伸展复制
 *   1) 差分角度 RLTx = RTx - LTx，经延迟环形缓冲
 *   2) 功率估算 x = tau_raw * angular_vel
 *   3) smooth gate: 0.5*(tanh(k*(x-p_on))+1)
 *   4) 一阶 lead predict (提前补偿)
 *   5) 伸展相(extension)复制: 屈相力矩延迟半周期取反
 *
 * 超参数 (~12个):
 *   Rescaling_gain, Flex_Assist_gain, Ext_Assist_gain,
 *   Assist_delay_gain, phase_offset_L/R,
 *   gate_k, gate_p_on, lead_frac,
 *   ext_phase_frac_L/R, ext_gain, scale_all
 *
 * Auto Delay (后处理延迟):
 *   在现有 Assist_delay_gain（相位延迟）之后，再加一段
 *   可由 AutoDelayOptimizer 自动优化的绝对 ms 延迟缓冲。
 *   auto_delay_enable 关闭时使用 GUI 下发的 eg_post_delay_ms。
 ************************************************************/

#include "Controller.h"
#include "AutoDelayOptimizer.h"

#define EG_DELAY_BUF      100
#define EG_EXT_BUF        400
#define EG_POST_DELAY_BUF 200  // 200 samples @100Hz = 2s max post-delay

class Controller_EG : public Controller {
public:
  Controller_EG();

  void compute(const CtrlInput& in, CtrlOutput& out) override;
  void parse_params(const BleDownlinkData& dl) override;
  void reset() override;
  const char* name() const override { return "EG"; }
  AlgoID id() const override { return ALGO_EG; }

  // 1Hz 自动 delay 侧环（由 .ino 主循环调用）
  void tick_auto_delay(unsigned long now_us, float gait_freq_hz);

  // 填充 BLE 上行 RPi 透传槽（40B v3 格式）
  void fill_ble_status(uint8_t* buf40) const;

  AutoDelayOptimizer ado_;  // public: .ino 可读取状态

private:
  // === 参数 ===
  float Rescaling_gain_;
  float Flex_Assist_gain_;
  float Ext_Assist_gain_;
  float Assist_delay_gain_;
  int8_t phase_offset_L_;
  int8_t phase_offset_R_;
  float gate_k_;
  float gate_p_on_;
  float lead_frac_;
  float ext_phase_frac_L_;
  float ext_phase_frac_R_;
  float ext_gain_;
  float scale_all_;
  float post_delay_ms_base_;  // GUI 下发的后处理延迟基础值 (ms)

  // === 内部状态 ===
  double RLTx_delay_[EG_DELAY_BUF];
  int    doi_;

  float  flexL_hist_[EG_EXT_BUF];
  float  flexR_hist_[EG_EXT_BUF];
  int    ext_i_;

  float  xL_prev_;
  float  xR_prev_;
  float  tau_cmd_L_filt_;
  float  tau_cmd_R_filt_;

  // === 后处理延迟缓冲 (auto delay output stage) ===
  float  post_buf_L_[EG_POST_DELAY_BUF];
  float  post_buf_R_[EG_POST_DELAY_BUF];
  int    post_buf_idx_;

  // === 辅助函数 ===
  static float smooth_gate_p(float x, float k, float p_on);
  static float lead_predict(float x, float x_prev, float lead_ms, float Ts);
};

#endif // CONTROLLER_EG_H
