#ifndef AUTO_DELAY_OPTIMIZER_H
#define AUTO_DELAY_OPTIMIZER_H

/************************************************************
 * AutoDelayOptimizer.h — Teensy 本地自动 Delay 调参工具类
 *
 * 功能:
 *   - 维护左右腿 tau_src / vel 的环形缓冲（最大 ADO_BUF_LEN 帧）
 *   - 1Hz 侧环：局部扫描 ±100ms，挑选最优 delay（最大化正功占比）
 *   - 左右腿独立扫描、独立 dwell、独立 best_delay
 *   - fill_ble_status(buf40) 将结果以 v3 int16 格式写入 BLE 上行的 40B RPi 透传槽
 *
 * 使用方式（在 Controller_Samsung / Controller_EG 中）:
 *   AutoDelayOptimizer ado;
 *   // 每个 100Hz 控制 tick（compute() 末尾）:
 *   ado.push_sample(tau_src_L, tau_src_R, vel_L, vel_R);
 *   // 每秒在 .ino 主循环的 1Hz 侧环:
 *   ado.tick(micros(), gait_freq_hz);
 *   // BLE 发送时:
 *   ado.fill_ble_status(rpi_uplink_buf);
 ************************************************************/

#include <Arduino.h>
#include <cstring>
#include <math.h>

// ============================================================
// 调参常量
// ============================================================
static const int   ADO_BUF_LEN            = 1200;   // 12s @ 100Hz
static const float ADO_CTRL_HZ            = 100.0f;
static const float ADO_SCAN_HALF_MS       = 100.0f;
static const float ADO_SCAN_STEP_MS       = 10.0f;
static const float ADO_MAX_STEP_MS        = 20.0f;
static const float ADO_TARGET_RATIO       = 0.95f;
static const float ADO_DWELL_S            = 3.0f;
static const float ADO_UPDATE_S           = 1.0f;
static const float ADO_VALID_MIN_VEL_DPS  = 5.0f;
static const float ADO_VALID_MIN_ABS_PWR  = 0.1f;   // W/s (abs power per second)
static const float ADO_WINDOW_MIN_S       = 4.0f;
static const float ADO_WINDOW_MAX_S       = 12.0f;
static const float ADO_WINDOW_FALLBACK_S  = 8.0f;
static const float ADO_WINDOW_CYCLES      = 4.0f;
static const float ADO_MIN_DELAY_MS       = 0.0f;
static const float ADO_MAX_DELAY_MS       = 1500.0f;

// BLE status version tag (v3 = per-leg packed int16, same as RPi)
static const uint8_t ADO_BLE_VERSION      = 0x03;
// algo source tag for non-RL Teensy-native status
static const uint8_t ADO_BLE_SRC_TEENSY   = 0xFE;

// ============================================================
// 辅助：把 float 以 int16 packed 写入 buf (little-endian)
// ============================================================
static inline void ado_put_i16(uint8_t* buf, int off, float v, float scale) {
  int32_t iv = (int32_t)lrintf(v * scale);
  if (iv >  32767) iv =  32767;
  if (iv < -32768) iv = -32768;
  buf[off]   = (uint8_t)(iv & 0xFF);
  buf[off+1] = (uint8_t)((iv >> 8) & 0xFF);
}

// ============================================================
// AutoDelayOptimizer 类
// ============================================================
class AutoDelayOptimizer {
public:
  // ---- 公开状态（供 .ino 读取或调试）----
  bool   enabled;            // auto delay 开关
  float  delay_ms_L;         // 当前生效 delay (左腿, ms)
  float  delay_ms_R;         // 当前生效 delay (右腿, ms)
  float  best_delay_ms_L;    // 本轮推荐 delay (左腿)
  float  best_delay_ms_R;    // 本轮推荐 delay (右腿)
  float  cur_ratio_L;        // 当前正功占比 (左腿)
  float  cur_ratio_R;
  float  cur_pos_per_s_L;    // 当前每秒正功 (W/s, 乘 scale 后)
  float  cur_pos_per_s_R;
  float  cur_neg_per_s_L;
  float  cur_neg_per_s_R;
  bool   motion_valid_L;
  bool   motion_valid_R;

  AutoDelayOptimizer();

  // 重置（切换算法时调用；base_delay = GUI 下发的初始 delay ms）
  void reset(float base_delay_ms);

  // 更新 auto enable 和基础 delay（parse_params 中调用）
  // rising edge 自动触发冷启动 dwell 重置
  void set_config(bool auto_en, float base_delay_ms);

  // 每 100Hz tick：推入一帧 (tau_src 是施加 delay 前的原始信号)
  void push_sample(float tau_src_L, float tau_src_R,
                   float vel_L_dps, float vel_R_dps);

  // 1Hz tick（在 .ino 的 1Hz 侧环调用）
  // gait_freq_hz: 若步频估计有效(>0.3) 则传入，否则传 0
  void tick(unsigned long now_us, float gait_freq_hz);

  // 填充 40B BLE 上行 RPi 透传槽（v3 int16 格式）
  // scale: 回传 pos/neg_per_s 时乘以的 runtime_scale（对功率语义有意义；ratio 不乘）
  void fill_ble_status(uint8_t* buf40, float runtime_scale = 1.0f) const;

private:
  // ---- 环形缓冲 ----
  float tau_src_L_[ADO_BUF_LEN];
  float tau_src_R_[ADO_BUF_LEN];
  float vel_L_[ADO_BUF_LEN];     // deg/s
  float vel_R_[ADO_BUF_LEN];
  int   buf_head_;   // 下一个写入位置
  int   buf_count_;  // 当前有效样本数

  // ---- 时间 ----
  unsigned long last_eval_us_;
  unsigned long last_change_us_L_;
  unsigned long last_change_us_R_;
  bool prev_enabled_;

  // ---- 功率指标结构 ----
  struct LegMetrics {
    float ratio;
    float pos_per_s;
    float neg_per_s;
    bool  valid;
  };

  // 对单腿在当前 window 中以指定 delay 计算功率指标
  // tau_src[] / vel[] 均从 buf_head_ 往前数 window_frames 帧
  LegMetrics compute_metrics(const float* tau_src, const float* vel,
                              int delay_frames, int window_frames) const;

  // 对单腿执行一轮局部扫描 + pick_best + 限步长
  // 返回推荐 delay; 若无变化返回 cur_delay
  float scan_leg(const float* tau_src, const float* vel,
                 float cur_delay, int window_frames,
                 LegMetrics& out_metrics, float& out_best,
                 bool& out_motion_valid) const;

  // 从环形缓冲中以 delay_frames 的偏移读取下标
  inline int ring_idx(int from_head, int offset_back) const {
    int idx = from_head - offset_back - 1;
    if (idx < 0) idx += ADO_BUF_LEN;
    return idx;
  }
};

#endif // AUTO_DELAY_OPTIMIZER_H
