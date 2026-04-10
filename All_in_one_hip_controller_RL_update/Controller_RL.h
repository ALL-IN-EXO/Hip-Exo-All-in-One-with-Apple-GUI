#ifndef CONTROLLER_RL_H
#define CONTROLLER_RL_H

/************************************************************
 * Controller_RL — 强化学习神经网络算法
 *
 * 工作流程:
 *   1) Teensy 每个控制周期通过 Serial8 发送 IMU 数据给 RPi
 *   2) RPi 运行 LSTM 推理，返回扭矩命令
 *   3) Teensy 读取 RPi 返回的扭矩作为输出
 *
 * Serial8 协议:
 *   Teensy→Pi: [0xA5 0x5A] [28] [0x01] [Lpos(4B)] [Rpos(4B)]
 *              [Lvel(4B)] [Rvel(4B)] [logtag(11B)] [cksum(1B)]
 *              总长 = 32 bytes
 *
 *   Pi→Teensy: [0xAA 0x55] [tauL(4B)] [tauR(4B)] [Lp(4B)] [Ld(4B)]
 *              [Rp(4B)] [Rd(4B)]
 *              总长 = 26 bytes (PI_SIZE)
 *
 * 注意: Serial8 需要在 setup() 中初始化
 ************************************************************/

#include "Controller.h"

#define PI_TORQUE_SIZE 26    // AA 55 + 6×float32
#define PI_STATUS_SIZE 42    // AA 56 + 40 bytes status

class Controller_RL : public Controller {
public:
  Controller_RL();

  void compute(const CtrlInput& in, CtrlOutput& out) override;
  void parse_params(const BleDownlinkData& dl) override;
  void reset() override;
  const char* name() const override { return "RL"; }
  AlgoID id() const override { return ALGO_RL; }

  // 初始化 Serial8 (主 setup 中调用)
  void init_serial();

  // 额外的 RPi 反馈数据 (可选读取)
  float tau_pi_Lp, tau_pi_Rp;  // 位置增益输出
  float tau_pi_Ld, tau_pi_Rd;  // 阻尼增益输出

  // 外部可设置的 logtag (用于转发给 RPi)
  char logtag[11];

  // RPi 上行状态 (供 BLE 透传给 GUI)
  uint8_t rpi_status_buf[40];
  bool    rpi_status_valid;

private:
  float tau_pi_L_;   // RPi 返回的左腿扭矩
  float tau_pi_R_;   // RPi 返回的右腿扭矩
  bool  pi_connected_;
  uint32_t last_rx_ms_;

  void send_imu_to_pi(float Lpos_rad, float Rpos_rad,
                       float Lvel, float Rvel);
  bool read_torque_from_pi();

  static uint8_t cksum8(const uint8_t* p, size_t n);

  // 接收状态机
  enum RxState { WAIT_HEADER1, WAIT_HEADER2, WAIT_TYPE, READING_TORQUE, READING_STATUS };
  RxState rx_state_;
  uint8_t rx_torque_buf_[PI_TORQUE_SIZE];
  uint8_t rx_status_buf_[PI_STATUS_SIZE];
  int rx_idx_;
  uint8_t rx_pkt_type_;   // 0x55=torque, 0x56=status
};

#endif // CONTROLLER_RL_H
