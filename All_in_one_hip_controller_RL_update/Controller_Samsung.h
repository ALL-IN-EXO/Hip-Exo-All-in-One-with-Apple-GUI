#ifndef CONTROLLER_SAMSUNG_H
#define CONTROLLER_SAMSUNG_H

/************************************************************
 * Controller_Samsung — Samsung 算法
 *
 * 原理: y = sin(q_R) - sin(q_L), 延迟后 tau = kappa * y_delayed
 * 参数: kappa (~3.0), delay_ms (~250ms)
 * 极简，只需 2 个超参数
 *
 * Auto Delay: AutoDelayOptimizer ado_ 在 auto_delay_enable 时
 *   自动优化 delay_ms_，使正功占比最大化。
 ************************************************************/

#include "Controller.h"
#include "AutoDelayOptimizer.h"

#define SAM_DELAY_BUF 200  // 200 samples @100Hz = 2s max delay

class Controller_Samsung : public Controller {
public:
  Controller_Samsung();

  void compute(const CtrlInput& in, CtrlOutput& out) override;
  void parse_params(const BleDownlinkData& dl) override;
  void reset() override;
  const char* name() const override { return "Samsung"; }
  AlgoID id() const override { return ALGO_SAMSUNG; }

  // 1Hz 自动 delay 侧环（由 .ino 主循环调用，非控制主链路）
  void tick_auto_delay(unsigned long now_us, float gait_freq_hz);

  // 填充 BLE 上行 RPi 透传槽（40B v3 格式）
  void fill_ble_status(uint8_t* buf40) const;
  bool legacy_internal_lpf_enabled() const { return legacy_internal_lpf_; }

  // 供 .ino 读取当前生效 delay（auto on: 取优化值; auto off: 取 GUI 值）
  float get_delay_ms_L() const { return ado_.delay_ms_L; }
  float get_delay_ms_R() const { return ado_.delay_ms_R; }

  AutoDelayOptimizer ado_;  // public: .ino 可读取状态

private:
  float   kappa_;
  float   base_delay_ms_;  // GUI 下发的基础 delay
  bool    legacy_internal_lpf_;  // 占位开关：后续用于复现 Samsung legacy 内部 LPF

  float delay_buf_[SAM_DELAY_BUF];
  int   buf_idx_;
};

#endif // CONTROLLER_SAMSUNG_H
