#ifndef CONTROLLER_H
#define CONTROLLER_H

/************************************************************
 * Controller.h — 算法抽象基类
 *
 * 所有控制算法（EG, Samsung, RL, Test）都继承此基类。
 * 主循环通过统一接口调用，不需要 if/else 分支。
 *
 * 使用方式:
 *   Controller* ctrl = &ctrl_eg;  // 或 &ctrl_samsung, &ctrl_rl
 *   ctrl->compute(input, output);
 ************************************************************/

#include <Arduino.h>
#include "BleProtocol.h"

// 算法输入（每个控制周期由主循环填充）
struct CtrlInput {
  float LTx;              // 左腿角度 (deg)
  float RTx;              // 右腿角度 (deg)
  float LTAVx;            // 左腿角速度 (deg/s), IMU 直接输出
  float RTAVx;            // 右腿角速度 (deg/s), IMU 直接输出
  float LTx_filtered;     // 左腿滤波角度
  float RTx_filtered;     // 右腿滤波角度
  float LTx_filtered_last;
  float RTx_filtered_last;
  float gait_freq;        // 步态频率 (Hz)
  uint8_t gait_inited;    // 步态是否已初始化
  float Ts;               // 控制周期 (s)
  float max_torque_cfg;   // 最大扭矩限制 (Nm)
  int8_t l_ctl_dir;       // 左腿方向
  int8_t r_ctl_dir;       // 右腿方向
  unsigned long current_time_us;
};

// 算法输出
struct CtrlOutput {
  float tau_L;  // 左腿输出扭矩 (Nm, 输出轴)
  float tau_R;  // 右腿输出扭矩 (Nm, 输出轴)
};

class Controller {
public:
  virtual ~Controller() {}

  // 核心：根据输入计算输出扭矩
  virtual void compute(const CtrlInput& in, CtrlOutput& out) = 0;

  // 从 BLE 下行帧解析参数（仅解析本算法相关部分）
  virtual void parse_params(const BleDownlinkData& dl) = 0;

  // 重置内部状态（切换算法时调用）
  virtual void reset() = 0;

  // 算法名称（用于日志/串口打印）
  virtual const char* name() const = 0;

  // 算法 ID
  virtual AlgoID id() const = 0;
};

#endif // CONTROLLER_H
