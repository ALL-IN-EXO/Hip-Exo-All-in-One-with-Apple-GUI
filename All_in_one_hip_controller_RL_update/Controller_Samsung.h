#ifndef CONTROLLER_SAMSUNG_H
#define CONTROLLER_SAMSUNG_H

/************************************************************
 * Controller_Samsung — Samsung 算法
 *
 * 原理: y = sin(q_R) - sin(q_L), 延迟后 tau = kappa * y_delayed
 * 参数: kappa (~3.0), delay_ms (~250ms)
 * 极简，只需 2 个超参数
 ************************************************************/

#include "Controller.h"

#define SAM_DELAY_BUF 200  // 200 samples @100Hz = 2s max delay

class Controller_Samsung : public Controller {
public:
  Controller_Samsung();

  void compute(const CtrlInput& in, CtrlOutput& out) override;
  void parse_params(const BleDownlinkData& dl) override;
  void reset() override;
  const char* name() const override { return "Samsung"; }
  AlgoID id() const override { return ALGO_SAMSUNG; }

private:
  float kappa_;
  int16_t delay_ms_;

  float delay_buf_[SAM_DELAY_BUF];
  int   buf_idx_;
};

#endif // CONTROLLER_SAMSUNG_H
