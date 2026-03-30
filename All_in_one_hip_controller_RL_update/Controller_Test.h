#ifndef CONTROLLER_TEST_H
#define CONTROLLER_TEST_H

/************************************************************
 * Controller_Test — 测试模式
 *
 * 支持两种波形:
 *   waveform=0: 固定力矩 (Constant)
 *   waveform=1: 正弦波 (Sin Wave) — 可选频率和幅值
 ************************************************************/

#include "Controller.h"

class Controller_Test : public Controller {
public:
  Controller_Test()
    : amplitude_(0.0f), waveform_(0), freq_hz_(1.0f), phase_(0.0f) {}

  void compute(const CtrlInput& in, CtrlOutput& out) override {
    float torque = 0.0f;
    if (waveform_ == 0) {
      torque = amplitude_;
    } else {
      // Sin wave: amplitude * sin(2*pi*freq*t)
      phase_ += 2.0f * 3.14159265f * freq_hz_ * in.Ts;
      if (phase_ > 6.2831853f) phase_ -= 6.2831853f;
      torque = amplitude_ * sinf(phase_);
    }
    out.tau_L = torque;
    out.tau_R = torque;
  }

  void parse_params(const BleDownlinkData& dl) override {
    amplitude_ = dl.test_torque_Nm;
    if (amplitude_ > 15.0f)  amplitude_ = 15.0f;
    if (amplitude_ < -15.0f) amplitude_ = -15.0f;

    uint8_t new_waveform = dl.test_waveform;
    if (new_waveform != waveform_) {
      phase_ = 0.0f;  // 切换波形时重置相位
    }
    waveform_ = new_waveform;

    freq_hz_ = dl.test_freq_hz;
    if (freq_hz_ < 0.1f) freq_hz_ = 0.1f;
    if (freq_hz_ > 10.0f) freq_hz_ = 10.0f;
  }

  void reset() override {
    amplitude_ = 0.0f;
    phase_ = 0.0f;
    waveform_ = 0;
  }

  const char* name() const override { return "Test"; }
  AlgoID id() const override { return ALGO_TEST; }

private:
  float   amplitude_;    // 力矩幅值 (Nm)
  uint8_t waveform_;     // 0=constant, 1=sin
  float   freq_hz_;      // 正弦频率 (Hz)
  float   phase_;        // 相位累积 (rad)
};

#endif // CONTROLLER_TEST_H
