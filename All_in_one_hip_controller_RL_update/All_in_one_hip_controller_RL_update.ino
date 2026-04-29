/************************************************************
 * Motor_Servo_Switch_EG_6_IMU.ino — 主循环
 *
 * 架构概览:
 *   MotorDriver.h      → 电机抽象层 (SIG/TMOTOR 运行时切换)
 *   BleProtocol.h      → 128 字节 BLE 帧打包/解析
 *   Controller.h        → 算法抽象基类
 *   Controller_EG       → Energy Gate 算法
 *   Controller_Samsung  → Samsung 算法
 *   Controller_RL       → RL 神经网络 (Serial8 ↔ RPi)
 *   Controller_Test     → 测试模式
 ************************************************************/



/******************** 包含 ********************/
#include <Arduino.h>
#include <math.h>
#include <cstring>
#include <FlexCAN_T4.h>
#include "IMU_Adapter.h"
#include "SdFat.h"
#include "RingBuf.h"
#include "sdlogger.h"

#include "MotorDriver.h"
#include "BleProtocol.h"
#include "Controller.h"
#include "Controller_EG.h"
#include "Controller_Samsung.h"
#include "Controller_RL.h"
#include "Controller_Test.h"
#include "Controller_SOGI.h"

#define GUI_WRITE_ENABLE 1

/******************** LOGTAG ********************/
static char logtag[11] = {0};
static bool logtag_valid = false;
static bool logtag_persist = false;

/******************** 调试打印 ********************/
const unsigned long PRINT_INTERVAL_US = 100000; // 10 Hz
static unsigned long prev_print_us = 0;

/******************** 全局硬件 ********************/
static uint8_t imu_ok_bits = 0;

// 1Hz 自动 delay 侧环时间戳 (Samsung / EG auto delay)
static unsigned long previous_time_ado_us = 0;

static inline bool is_inactive(float v) {
  return fabsf(v + 97.0f) < 0.5f;
}

// Teensy CAN3 (全局实例，用于 read; 各电机类内有自己的 Can3 用于 write)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// SD 日志
SdLogger logger(BUILTIN_SDCARD, F("walking_log_"), F(".csv"));
uint16_t g_init_status = 0;

// 频率
double cyclespersec_ctrl = 100;
double cyclespersec_ble  = 20;
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000.0 / cyclespersec_ctrl);
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000.0 / cyclespersec_ble);

/******************** 电机 ********************/
/*
 * 电机 ID 分配（硬件接线决定，不可随意修改）:
 *   SIG:    左腿 node_id=1,  右腿 node_id=2
 *   TMOTOR: 左腿 drive_id=104, 右腿 drive_id=105
 *   M1 = 右腿, M2 = 左腿
 */
static MotorDriver_Sig    sig_L(1, 9.76f);
static MotorDriver_Sig    sig_R(2, 9.76f);
static MotorDriver_Tmotor tm_L(0x001, 104, 0.73f);
static MotorDriver_Tmotor tm_R(0x002, 105, 0.73f);

MotorDriver* motor_L = &tm_L;
MotorDriver* motor_R = &tm_R;
uint8_t current_brand = BRAND_TMOTOR;
bool motor_brand_switch_pending = false;
uint8_t pending_brand = BRAND_NONE;

/******************** 算法实例（静态分配，无 new/delete）********************/
static Controller_EG      ctrl_eg;
static Controller_Samsung  ctrl_samsung;
static Controller_RL       ctrl_rl;
static Controller_Test     ctrl_test;
static Controller_SOGI     ctrl_sogi;

Controller* active_ctrl = &ctrl_eg;   // 默认 EG
uint8_t active_algo_id = ALGO_EG;

/******************** 通讯与传感 ********************/
IMU_Adapter imu;

/******************** BLE 缓冲 (128 字节) ********************/
uint8_t data_ble[BLE_FRAME_LEN] = {0};
uint8_t data_rs232_rx[BLE_PAYLOAD_LEN] = {0};

volatile uint8_t imu_init_ok = 0;
int l_ctl_dir = 1;
int r_ctl_dir = 1;

/******************** 控制状态 ********************/
float  M1_torque_meas = 0.0f, M2_torque_meas = 0.0f;
double M1_torque_command = 0.0, M2_torque_command = 0.0;

// IMU 滤波
double LTx_filtered = 0, LTx_filtered_last = 0;
double RTx_filtered = 0, RTx_filtered_last = 0;
double RLTx_filtered = 0;

float  gait_freq = 0;
float  max_torque_cfg = 0.0f;
float  filter_fc_hz       = 5.0f;   // 统一滤波截止频率 (Hz)，三通道共用
bool   filter_enable_ang  = false;
bool   filter_enable_vel  = false;
bool   filter_enable_tau  = true;   // 力矩滤波默认开启
bool   filter_type_butter = true;   // true=Butterworth, false=1st-order LPF
static uint8_t gait_inited = 0;

// === 统一信号滤波器：支持 1st-order LPF 和 2nd-order Butterworth ===
struct SignalFilter {
  bool   butter_mode = true;
  // Butterworth 2nd-order 系数 + 状态
  float b0=1,b1=0,b2=0, a1=0,a2=0;
  float x1=0,x2=0, y1=0,y2=0;
  // 1st-order 状态
  float fo_alpha=0, fo_y=0;

  void setup(float fc_hz, float fs_hz, bool butterworth) {
    butter_mode = butterworth;
    reset();
    if (!(fc_hz > 0.0f)) return;
    float nyq = 0.5f * fs_hz;
    if (fc_hz < 0.3f)         fc_hz = 0.3f;
    if (fc_hz > nyq - 0.5f)   fc_hz = nyq - 0.5f;
    if (butterworth) {
      const float q     = 0.70710678f;
      const float omega = 2.0f * PI * fc_hz / fs_hz;
      const float sn    = sinf(omega);
      const float cs    = cosf(omega);
      const float alpha = sn / (2.0f * q);
      const float aa0   = 1.0f + alpha;
      b0 = (1.0f - cs) * 0.5f / aa0;
      b1 = (1.0f - cs)        / aa0;
      b2 = (1.0f - cs) * 0.5f / aa0;
      a1 = -2.0f * cs          / aa0;
      a2 = (1.0f - alpha)     / aa0;
    } else {
      float dt = 1.0f / fs_hz;
      float wc = 2.0f * PI * fc_hz;
      fo_alpha = wc * dt / (1.0f + wc * dt);
    }
  }

  float process(float x) {
    if (butter_mode) {
      float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
      x2=x1; x1=x; y2=y1; y1=y;
      return y;
    } else {
      fo_y += fo_alpha * (x - fo_y);
      return fo_y;
    }
  }

  void reset() { x1=x2=y1=y2=fo_y=0.0f; }
};

static SignalFilter ang_filt_L, ang_filt_R;
static SignalFilter vel_filt_L, vel_filt_R;
static SignalFilter tau_filt_L, tau_filt_R;
static float filter_fc_prev       = -1.0f;
static bool  filter_type_prev     = true;

// RPi 透传缓冲
static uint8_t rpi_uplink_buf[40] = {0};
static uint8_t rpi_downlink_buf[40] = {0};

/******************** 时间 ********************/
unsigned long t_0 = 0;
unsigned long current_time = 0;
unsigned long previous_time = 0;
unsigned long previous_time_ble = 0;

/******************** 前置声明 ********************/
void initial_CAN();
void receive_motor_feedback();
void Receive_ble_Data();
void Transmit_ble_Data();
float estimateFreqFromRLTx(float signal, unsigned long nowMicros);
void forward_rpi_passthrough_to_pi(const uint8_t* data, uint8_t len);

bool imu_reinit_pending = false;
bool motor_reinit_pending = false;

// 限幅
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static void update_filters_if_needed() {
  bool changed = (fabsf(filter_fc_hz - filter_fc_prev) > 1e-3f) ||
                 (filter_type_butter != filter_type_prev);
  if (!changed) return;
  filter_fc_prev   = filter_fc_hz;
  filter_type_prev = filter_type_butter;
  const float fs = 100.0f;
  ang_filt_L.setup(filter_fc_hz, fs, filter_type_butter);
  ang_filt_R.setup(filter_fc_hz, fs, filter_type_butter);
  vel_filt_L.setup(filter_fc_hz, fs, filter_type_butter);
  vel_filt_R.setup(filter_fc_hz, fs, filter_type_butter);
  tau_filt_L.setup(filter_fc_hz, fs, filter_type_butter);
  tau_filt_R.setup(filter_fc_hz, fs, filter_type_butter);
}

static void reset_filter_states() {
  ang_filt_L.reset(); ang_filt_R.reset();
  vel_filt_L.reset(); vel_filt_R.reset();
  tau_filt_L.reset(); tau_filt_R.reset();
}

static uint8_t rpi_cksum8(const uint8_t* p, size_t n) {
  uint8_t s = 0;
  while (n--) s += *p++;
  return s;
}

void forward_rpi_passthrough_to_pi(const uint8_t* data, uint8_t len) {
  if (!data) return;
  if (len > 40) len = 40;
  if (len == 0) return;

  // Packet format: A5 5A | LEN=(TYPE+payload) | TYPE=0x02 | payload[len] | cksum
  uint8_t pkt[45];
  pkt[0] = 0xA5;
  pkt[1] = 0x5A;
  pkt[2] = (uint8_t)(1 + len);
  pkt[3] = 0x02;
  memcpy(&pkt[4], data, len);
  pkt[4 + len] = rpi_cksum8(&pkt[3], (size_t)(1 + len));

  Serial8.write(pkt, (size_t)(5 + len - 0));
}

/******************** 电机品牌切换 ********************/
void switch_motor_brand(uint8_t new_brand) {
  if (new_brand == current_brand || new_brand == BRAND_NONE) return;

  Serial.printf("[MOTOR] Switching from %s to %s\n",
    motor_L->brand_name(),
    (new_brand == BRAND_SIG) ? "SIG" : "TMOTOR");

  motor_L->stop();
  motor_R->stop();
  delay(50);

  CAN_message_t tmp;
  while (Can3.read(tmp)) {}

  switch (new_brand) {
    case BRAND_SIG:
      motor_L = &sig_L;
      motor_R = &sig_R;
      break;
    case BRAND_TMOTOR:
      motor_L = &tm_L;
      motor_R = &tm_R;
      break;
    default:
      Serial.println("[MOTOR] Unknown brand, abort");
      return;
  }

  initial_CAN();
  motor_L->init();
  motor_R->init();

  M1_torque_command = 0.0f;
  M2_torque_command = 0.0f;
  reset_filter_states();

  current_brand = new_brand;
  Serial.printf("[MOTOR] Switch done. Brand=%s\n", motor_L->brand_name());
}

/******************** 算法切换 ********************/
void switch_algorithm(uint8_t new_algo) {
  if (new_algo == active_algo_id) return;

  Controller* new_ctrl = nullptr;
  switch (new_algo) {
    case ALGO_EG:      new_ctrl = &ctrl_eg;      break;
    case ALGO_SAMSUNG: new_ctrl = &ctrl_samsung;  break;
    case ALGO_RL:      new_ctrl = &ctrl_rl;       break;
    case ALGO_TEST:    new_ctrl = &ctrl_test;     break;
    case ALGO_SOGI:    new_ctrl = &ctrl_sogi;     break;
    default:
      Serial.printf("[ALGO] Unknown algo ID: %d\n", new_algo);
      return;
  }

  // 停电机
  motor_L->stop();
  motor_R->stop();
  M1_torque_command = 0.0f;
  M2_torque_command = 0.0f;
  reset_filter_states();

  Serial.printf("[ALGO] Switching from %s to %s\n",
    active_ctrl->name(), new_ctrl->name());

  new_ctrl->reset();
  active_ctrl = new_ctrl;
  active_algo_id = new_algo;
  previous_time_ado_us = micros();  // 避免切换后立即触发 1Hz ADO tick
}

/******************** setup ********************/
void setup() {
  delay(3000);

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  // BLE 串口
  Serial5.begin(115200);
  // 128-byte frame takes about 11 ms at 115200 baud; 5 ms causes frequent partial reads.
  Serial5.setTimeout(20);

  delay(300);

  // CAN 初始化
  initial_CAN();
  Serial.println("[OK] CAN bus setup");

  // IMU 初始化
  imu.INIT();
  delay(2000);
  imu.INIT_MEAN();
  Serial.println("[OK] IMU setup done");
  imu_init_ok = 1;

  // RL Serial8 初始化
  ctrl_rl.init_serial();

  // 电机初始化（默认 TMOTOR）
  motor_L->init();
  motor_R->init();
  Serial.printf("[OK] Motor init done (brand=%s)\n", motor_L->brand_name());

  // SD 日志
  if (logger.begin()) {
    Serial.print(F("SD Logging file: "));
    Serial.println(logger.filename());
    logger.println(F(
      "Time_ms,teensy_t_cs_u16,teensy_t_s_unwrapped,"
      "imu_LTx,imu_RTx,imu_Lvel,imu_Rvel,"
      "gait_freq_Hz,gait_period_ms,"
      "L_command_actuator,R_command_actuator,L_torque_meas,R_torque_meas,"
      "L_pwr_W,R_pwr_W,brand,algo,L_pos_deg,R_pos_deg,tag"
    ));
    logger.flush();
    g_init_status = 1;
  } else {
    Serial.println(F("SD card init or file create failed!"));
  }

  update_filters_if_needed();
  reset_filter_states();

  Serial.printf("[OK] Algorithm: %s\n", active_ctrl->name());
  t_0 = micros();
}

/******************** loop ********************/
void loop() {
  // === 异步命令处理 ===
  if (imu_reinit_pending) {
    imu_reinit_pending = false;
    Serial.println("[CMD] IMU reinit requested by GUI");
    imu.REZERO_LR(600, 200);
    Serial.println("[IMU] Reinitialization done");
    imu_init_ok = 1;
  }

  if (motor_reinit_pending) {
    motor_reinit_pending = false;
    Serial.println("[CMD] MOTOR reinit requested by GUI");
    motor_L->stop();
    motor_R->stop();
    delay(50);
    CAN_message_t tmp;
    while (Can3.read(tmp)) {}
    initial_CAN();
    motor_L->init();
    motor_R->init();
    M1_torque_command = 0.0f;
    M2_torque_command = 0.0f;
    reset_filter_states();
    Serial.println("[MOTOR] Reinitialization done");
  }

  if (motor_brand_switch_pending) {
    motor_brand_switch_pending = false;
    switch_motor_brand(pending_brand);
  }

  // === IMU 读取 ===
  imu.READ();
  {
    uint8_t okL = (!is_inactive(imu.LTx)) ? 1 : 0;
    uint8_t okR = (!is_inactive(imu.RTx)) ? 1 : 0;
    uint8_t ok1 = (!is_inactive(imu.TX1)) ? 1 : 0;
    uint8_t ok2 = (!is_inactive(imu.TX2)) ? 1 : 0;
    uint8_t ok3 = (!is_inactive(imu.TX3)) ? 1 : 0;
    uint8_t ok4 = (!is_inactive(imu.TX4)) ? 1 : 0;
    imu_ok_bits = (okL<<0) | (okR<<1) | (ok1<<2) | (ok2<<3) | (ok3<<4) | (ok4<<5);
  }

  current_time = micros();
  const float Ts = (float)Tinterval_ctrl_micros / 1e6f;

  // === IMU 安全检测 ===
  if (fabs(imu.RTx) > 80.0f || fabs(imu.LTx) > 80.0f) {
    if (imu_init_ok) Serial.println("[WARN] IMU > 80 deg, disabling assist");
    imu_init_ok = 0;
  } else {
    if (!imu_init_ok) Serial.println("[OK] IMU back in safe range");
    imu_init_ok = 1;
  }

  // === 控制频率节拍 (100 Hz) ===
  if (current_time - previous_time >= Tinterval_ctrl_micros) {
    previous_time = current_time;

    // 1) CAN 反馈
    receive_motor_feedback();

    // 2) BLE 下发
    Receive_ble_Data();

    // 3) IMU 滤波
    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = 0.9f * LTx_filtered_last + 0.1f * imu.LTx;
    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = 0.9f * RTx_filtered_last + 0.1f * imu.RTx;
    RLTx_filtered     = RTx_filtered - LTx_filtered;

    // 4) 步态频率估计
    gait_freq = estimateFreqFromRLTx(RLTx_filtered, current_time);

    // 5) 填充算法输入（角度/速度可选滤波；功率计算始终用原始值）
    update_filters_if_needed();
    CtrlInput cin;
    cin.LTx   = filter_enable_ang ? ang_filt_L.process(imu.LTx)   : imu.LTx;
    cin.RTx   = filter_enable_ang ? ang_filt_R.process(imu.RTx)   : imu.RTx;
    cin.LTAVx = filter_enable_vel ? vel_filt_L.process(imu.LTAVx) : imu.LTAVx;
    cin.RTAVx = filter_enable_vel ? vel_filt_R.process(imu.RTAVx) : imu.RTAVx;
    cin.LTx_filtered      = LTx_filtered;
    cin.RTx_filtered      = RTx_filtered;
    cin.LTx_filtered_last = LTx_filtered_last;
    cin.RTx_filtered_last = RTx_filtered_last;
    cin.gait_freq    = gait_freq;
    cin.gait_inited  = gait_inited;
    cin.Ts           = Ts;
    cin.max_torque_cfg = max_torque_cfg;
    cin.l_ctl_dir    = l_ctl_dir;
    cin.r_ctl_dir    = r_ctl_dir;
    cin.current_time_us = current_time;

    // 6) 算法计算
    CtrlOutput cout;
    cout.tau_L = 0.0f;
    cout.tau_R = 0.0f;

    // Keep RL uplink alive even when IMU safety gate is false; clamp torque to zero for safety.
    if (active_algo_id == ALGO_RL) {
      active_ctrl->compute(cin, cout);
      if (!imu_init_ok) {
        cout.tau_L = 0.0f;
        cout.tau_R = 0.0f;
      }
    } else if (imu_init_ok || active_algo_id == ALGO_TEST) {
      // Test mode bypasses IMU init/safety gate so manual waveform output is uninterrupted.
      active_ctrl->compute(cin, cout);
    }

    // Apply per-leg direction from GUI (bit-coded in dl.dir_bits).
    // This is centralized here so all algorithms (EG/Samsung/RL/Test) behave consistently.
    cout.tau_L *= (cin.l_ctl_dir >= 0) ? 1.0f : -1.0f;
    cout.tau_R *= (cin.r_ctl_dir >= 0) ? 1.0f : -1.0f;

    // 统一电机前滤波（仅 Teensy-native 算法；RL 路径保持 Pi→Teensy→Motor 透明）
    bool use_unified_prefilter = (active_algo_id != ALGO_RL);
    if (active_algo_id == ALGO_EG && ctrl_eg.legacy_internal_lpf_enabled()) {
      // Legacy EG 内部已经包含 0.85/0.15 LPF，为避免双滤波，这里旁路统一滤波。
      use_unified_prefilter = false;
    }
    static bool prev_use_unified_prefilter = true;
    if (use_unified_prefilter != prev_use_unified_prefilter) {
      // 切换滤波路径时清一次状态，避免旁路/恢复瞬间的滤波尾迹。
      reset_torque_filter_state();
      prev_use_unified_prefilter = use_unified_prefilter;
    }
    if (use_unified_prefilter) {
      update_torque_filter_if_needed();
      if (!imu_init_ok && active_algo_id != ALGO_TEST) {
        tau_filt_L.reset(); tau_filt_R.reset();
        cout.tau_L = 0.0f;
        cout.tau_R = 0.0f;
      } else {
        cout.tau_L = tau_filt_L.process(cout.tau_L);
        cout.tau_R = tau_filt_R.process(cout.tau_R);
      }
    }

    // 最终安全限幅（滤波后再限幅，确保 Max Torque 严格生效）
    {
      const float max_abs_cmd = fabsf(max_torque_cfg);
      if (cout.tau_L >  max_abs_cmd) cout.tau_L =  max_abs_cmd;
      if (cout.tau_L < -max_abs_cmd) cout.tau_L = -max_abs_cmd;
      if (cout.tau_R >  max_abs_cmd) cout.tau_R =  max_abs_cmd;
      if (cout.tau_R < -max_abs_cmd) cout.tau_R = -max_abs_cmd;
    }

    M2_torque_command = cout.tau_L;  // M2 = 左腿
    M1_torque_command = cout.tau_R;  // M1 = 右腿

    // 7) 下发电机命令
    if (current_brand == BRAND_SIG) motor_L->request_feedback();
    motor_R->send_torque_Nm(M1_torque_command);  // M1 = 右腿
    motor_L->send_torque_Nm(M2_torque_command);  // M2 = 左腿

    // 8) SD 日志
    if (logger.isOpen()) {
      uint16_t t_cs_u16 = (uint16_t)((current_time / 10000UL) & 0xFFFF);
      float t_s_unwrapped = current_time / 1000000.0f;
      float L_torque_meas = motor_L->get_torque_meas();
      float R_torque_meas = motor_R->get_torque_meas();
      float gait_period_ms = (gait_freq > 0.01f) ? (1000.0f / gait_freq) : 0.0f;
      float L_pwr_w = L_torque_meas * imu.LTAVx * (PI / 180.0f);
      float R_pwr_w = R_torque_meas * imu.RTAVx * (PI / 180.0f);

      logger.print(current_time / 1000UL); logger.print(',');
      logger.print(t_cs_u16);              logger.print(',');
      logger.print(t_s_unwrapped, 4);      logger.print(',');
      logger.print(imu.LTx, 4);            logger.print(',');
      logger.print(imu.RTx, 4);            logger.print(',');
      logger.print(imu.LTAVx, 4);          logger.print(',');
      logger.print(imu.RTAVx, 4);          logger.print(',');
      logger.print(gait_freq, 4);          logger.print(',');
      logger.print(gait_period_ms, 2);     logger.print(',');
      logger.print(M2_torque_command, 4);  logger.print(',');
      logger.print(M1_torque_command, 4);  logger.print(',');
      logger.print(L_torque_meas, 4);      logger.print(',');
      logger.print(R_torque_meas, 4);      logger.print(',');
      logger.print(L_pwr_w, 4);            logger.print(',');
      logger.print(R_pwr_w, 4);            logger.print(',');
      logger.print(motor_L->brand_name());  logger.print(',');
      logger.print(active_ctrl->name());    logger.print(',');
      logger.print(motor_L->get_pos_deg(), 2); logger.print(',');
      logger.print(motor_R->get_pos_deg(), 2); logger.print(',');
      logger.print(logtag_valid ? logtag : "");
      logger.println();
      static int log_flush_count = 0;
      if (++log_flush_count >= 10) {
        logger.flush();
        log_flush_count = 0;
      }
    }
  }

  // === 1Hz Auto Delay 侧环 (Samsung / EG) ===
  if (current_time - previous_time_ado_us >= 1000000UL) {
    previous_time_ado_us = current_time;
    if (active_algo_id == ALGO_SAMSUNG) {
      ctrl_samsung.tick_auto_delay(current_time, gait_freq);
    } else if (active_algo_id == ALGO_EG) {
      ctrl_eg.tick_auto_delay(current_time, gait_freq);
    } else if (active_algo_id == ALGO_SOGI) {
      ctrl_sogi.tick_auto_delay(current_time, gait_freq);
    }
  }

  // === BLE 发送节拍 (20 Hz) ===
  if (current_time - previous_time_ble >= Tinterval_ble_micros) {
    previous_time_ble = current_time;
    Transmit_ble_Data();
  }

  // === 10 Hz 串口调试 ===
  if (current_time - prev_print_us >= PRINT_INTERVAL_US) {
    prev_print_us = current_time;
    Serial.printf("[%s] brand=%s  L_cmd=%.2f  R_cmd=%.2f  L_meas=%.2f  R_meas=%.2f\r\n",
      active_ctrl->name(),
      motor_L->brand_name(),
      M2_torque_command,
      M1_torque_command,
      motor_L->get_torque_meas(),
      motor_R->get_torque_meas()
    );
  }
}

/******************** 步态频率估计 ********************/
float estimateFreqFromRLTx(float signal, unsigned long nowMicros) {
  const float HYST_POS = +2.0f;
  const float HYST_NEG = -2.0f;
  const unsigned long MIN_DT_US = 250000;
  const unsigned long MAX_DT_US = 2000000;

  static bool armed = false;
  static unsigned long lastCross = 0;
  static float freqHz = 0.0f;

  if (!armed && signal <= HYST_NEG) armed = true;

  if (armed && signal >= HYST_POS) {
    unsigned long dt = nowMicros - lastCross;
    if (lastCross > 0 && dt >= MIN_DT_US && dt <= MAX_DT_US) {
      freqHz = 1.0e6f / (float)dt;
      gait_inited = 1;
    }
    lastCross = nowMicros;
    armed = false;
  }

  return freqHz;
}

/******************** CAN 初始化 ********************/
void initial_CAN() {
  Can3.begin();
  Can3.setBaudRate(1000000);
  delay(400);
  Serial.println("CAN bus setup done...");
  delay(200);
}

/******************** CAN 反馈接收 ********************/
void receive_motor_feedback() {
  CAN_message_t msg;
  while (Can3.read(msg)) {
    motor_L->parse_feedback(msg);
    motor_R->parse_feedback(msg);
  }
  M1_torque_meas = motor_R->get_torque_meas();
  M2_torque_meas = motor_L->get_torque_meas();
}

/******************** BLE 接收 (128 字节帧) ********************/
void Receive_ble_Data() {
  static uint8_t hdr[3] = {0};

  while (Serial5.available()) {
    hdr[0] = hdr[1];
    hdr[1] = hdr[2];
    hdr[2] = Serial5.read();

    if (!(hdr[0] == 0xA5 && hdr[1] == 0x5A && hdr[2] == BLE_FRAME_LEN)) {
      continue;
    }

    size_t n = Serial5.readBytes((char*)data_rs232_rx, BLE_PAYLOAD_LEN);
    if (n < BLE_PAYLOAD_LEN) {
      Serial.printf("[BLE] Payload incomplete: got %d / %d bytes\n", (int)n, BLE_PAYLOAD_LEN);
      return;
    }

    // LOGTAG 帧检测
    if (data_rs232_rx[0] == 'L' && data_rs232_rx[1] == 'G') {
      uint8_t tlen = data_rs232_rx[2];
      if (tlen > 10) tlen = 10;
      memset(logtag, 0, sizeof(logtag));
      memcpy(logtag, &data_rs232_rx[3], tlen);
      logtag_valid = true;
      logtag_persist = (data_rs232_rx[13] & 0x01);
      Serial.print("LOGTAG received: ");
      Serial.println(logtag);
      // 同步给 RL 控制器
      memcpy(ctrl_rl.logtag, logtag, 11);
      return;
    }

    // 解析下行数据
    BleDownlinkData dl = ble_parse_downlink(data_rs232_rx);

#if GUI_WRITE_ENABLE
    // 通用参数
    max_torque_cfg = clampf(dl.max_torque_cfg, 0.0f, 30.0f);
    filter_fc_hz       = clampf(dl.filter_fc_hz, 0.3f, 49.9f);
    filter_enable_ang  = (dl.filter_flags & 0x01) != 0;
    filter_enable_vel  = (dl.filter_flags & 0x02) != 0;
    filter_enable_tau  = (dl.filter_flags & 0x04) != 0;
    filter_type_butter = (dl.filter_flags & 0x08) != 0;
    int prev_l_dir = l_ctl_dir;
    int prev_r_dir = r_ctl_dir;
    l_ctl_dir = (dl.dir_bits & 0x01) ? 1 : -1;
    r_ctl_dir = (dl.dir_bits & 0x02) ? 1 : -1;
    if (l_ctl_dir != prev_l_dir || r_ctl_dir != prev_r_dir) {
      Serial.printf("[DIR] L=%c R=%c (dir_bits=0x%02X)\n",
                    (l_ctl_dir > 0 ? '+' : '-'),
                    (r_ctl_dir > 0 ? '+' : '-'),
                    dl.dir_bits);
    }

    // IMU/Motor reinit
    if (dl.imu_reinit)   imu_reinit_pending = true;
    if (dl.motor_reinit) motor_reinit_pending = true;

    // 电机品牌切换
    if (dl.brand_request == BRAND_SIG || dl.brand_request == BRAND_TMOTOR) {
      if (dl.brand_request != current_brand) {
        pending_brand = dl.brand_request;
        motor_brand_switch_pending = true;
      }
    }

    // 算法切换
    if (dl.algo_select != active_algo_id) {
      switch_algorithm(dl.algo_select);
    }

    // 算法参数更新
    active_ctrl->parse_params(dl);

    // RPi 透传下行 (GUI → Teensy → RPi via Serial8)
    if (dl.has_rpi_data) {
      memcpy(rpi_downlink_buf, dl.rpi_passthru, 40);
      forward_rpi_passthrough_to_pi(rpi_downlink_buf, 40);
    }
#endif
  }
}

/******************** BLE 发送 (128 字节帧) ********************/
void Transmit_ble_Data() {
  memset(data_ble, 0, BLE_FRAME_LEN);
  auto sat_i16 = [](float v) -> int16_t {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)roundf(v);
  };

  BleUplinkData ud;
  ud.t_cs         = (uint16_t)((millis() / 10) & 0xFFFF);
  ud.L_ang100     = (int16_t)roundf(imu.LTx * 100.0f);
  ud.R_ang100     = (int16_t)roundf(imu.RTx * 100.0f);
  ud.L_tau100     = (int16_t)roundf(motor_L->get_torque_meas() * 100.0f);
  ud.R_tau100     = (int16_t)roundf(motor_R->get_torque_meas() * 100.0f);
  ud.L_cmd100     = (int16_t)roundf(M2_torque_command * 100.0f);
  ud.R_cmd100     = (int16_t)roundf(M1_torque_command * 100.0f);
  ud.imu_init_ok  = imu_init_ok ? 1 : 0;
  ud.mt100        = (int16_t)roundf(max_torque_cfg * 100.0f);
  ud.g_init_status = g_init_status & 0xFF;
  ud.gf100        = (int16_t)roundf(gait_freq * 100.0f);
  ud.logtag_valid = logtag_valid ? 1 : 0;
  ud.logtag_char  = logtag[0];
  ud.imu_ok_bits  = imu_ok_bits;
  ud.current_brand = current_brand;

  float tL = motor_L->get_temp_C();
  float tR = motor_R->get_temp_C();
  ud.temp_L       = isnan(tL) ? 0 : (int8_t)tL;
  ud.temp_R       = isnan(tR) ? 0 : (int8_t)tR;
  ud.active_algo  = active_algo_id;

  // IMU 1-4 angles
  ud.TX1_100 = (int16_t)roundf(imu.TX1 * 100.0f);
  ud.TX2_100 = (int16_t)roundf(imu.TX2 * 100.0f);
  ud.TX3_100 = (int16_t)roundf(imu.TX3 * 100.0f);
  ud.TX4_100 = (int16_t)roundf(imu.TX4 * 100.0f);

  // IMU left/right + 1~4 angular velocities (deg/s), packed at 0.1 deg/s resolution.
  ud.LTAVx_10 = (int16_t)roundf(imu.LTAVx * 10.0f);
  ud.RTAVx_10 = (int16_t)roundf(imu.RTAVx * 10.0f);
  ud.VTX1_10  = (int16_t)roundf(imu.VTX1 * 10.0f);
  ud.VTX2_10  = (int16_t)roundf(imu.VTX2 * 10.0f);
  ud.VTX3_10  = (int16_t)roundf(imu.VTX3 * 10.0f);
  ud.VTX4_10  = (int16_t)roundf(imu.VTX4 * 10.0f);
  ud.battL_pct = imu.BattL;
  ud.battR_pct = imu.BattR;
  ud.batt1_pct = imu.Batt1;
  ud.batt2_pct = imu.Batt2;
  ud.batt3_pct = imu.Batt3;
  ud.batt4_pct = imu.Batt4;

  ble_pack_uplink(data_ble, ud);

  // RPi 透传上行:
  //   RL 模式: 来自 RPi Serial8 回传
  //   Samsung/EG 模式: 来自 Teensy 本地 AutoDelayOptimizer（v3 格式）
  if (active_algo_id == ALGO_RL) {
    if (ctrl_rl.rpi_status_valid) {
      memcpy(rpi_uplink_buf, ctrl_rl.rpi_status_buf, 40);
    }
  } else if (active_algo_id == ALGO_SAMSUNG) {
    ctrl_samsung.fill_ble_status(rpi_uplink_buf);
  } else if (active_algo_id == ALGO_EG) {
    ctrl_eg.fill_ble_status(rpi_uplink_buf);
  } else if (active_algo_id == ALGO_SOGI) {
    // SOGI 也上报 ADO 指标到同一 40B 状态槽，供 GUI Power Sign overlay 显示 ratio/+P/-P。
    ctrl_sogi.fill_ble_status(rpi_uplink_buf);
  } else {
    memset(rpi_uplink_buf, 0, 40);
  }
  ble_pack_rpi_uplink(data_ble, rpi_uplink_buf, 40);

  // ===== 控制同步扩展区 [101..127] =====
  const uint8_t TELE_FLAG_VALID      = 0x01;
  const uint8_t TELE_FLAG_PHYS_VALID = 0x02;
  const uint8_t TELE_FLAG_CTRL_VALID = 0x04;
  const uint8_t TELE_FLAG_SYNC_VALID = 0x08;
  const uint8_t TELE_FLAG_SYNC_FROM_PI = 0x10;

  float phys_pwr_L = motor_L->get_torque_meas() * imu.LTAVx * (PI / 180.0f);
  float phys_pwr_R = motor_R->get_torque_meas() * imu.RTAVx * (PI / 180.0f);

  int16_t sync_ang_L100 = ud.L_ang100;
  int16_t sync_ang_R100 = ud.R_ang100;
  int16_t sync_vel_L10  = ud.LTAVx_10;
  int16_t sync_vel_R10  = ud.RTAVx_10;
  int16_t sync_cmd_L100 = ud.L_cmd100;
  int16_t sync_cmd_R100 = ud.R_cmd100;
  int16_t ctrl_pwr_L100 = sat_i16((sync_cmd_L100 / 100.0f) * (sync_vel_L10 / 10.0f) * (PI / 180.0f) * 100.0f);
  int16_t ctrl_pwr_R100 = sat_i16((sync_cmd_R100 / 100.0f) * (sync_vel_R10 / 10.0f) * (PI / 180.0f) * 100.0f);
  uint16_t sync_sample_id = 0;
  uint8_t ext_flags = TELE_FLAG_VALID | TELE_FLAG_PHYS_VALID | TELE_FLAG_CTRL_VALID;
  uint8_t ext_age_ms_10 = 0xFF;

  // RL + Pi 同步链路有效时，优先使用 Pi 对齐输入/功率。
  if (active_algo_id == ALGO_RL && ctrl_rl.sync_valid && ctrl_rl.sync_from_pi) {
    sync_sample_id = ctrl_rl.sync_sample_id;
    sync_ang_L100 = ctrl_rl.sync_ang_L100;
    sync_ang_R100 = ctrl_rl.sync_ang_R100;
    sync_vel_L10 = ctrl_rl.sync_vel_L10;
    sync_vel_R10 = ctrl_rl.sync_vel_R10;
    ctrl_pwr_L100 = ctrl_rl.sync_ctrl_pwr_L100;
    ctrl_pwr_R100 = ctrl_rl.sync_ctrl_pwr_R100;
    ext_flags |= (TELE_FLAG_SYNC_VALID | TELE_FLAG_SYNC_FROM_PI);
    ext_age_ms_10 = 0;
  } else {
    // 非 RL / 无 Pi 同步时，sync 通道回退到本地同采样值。
    ext_flags |= TELE_FLAG_SYNC_VALID;
    ext_age_ms_10 = 0xFF;
  }

  BleTelemetryExt te;
  te.flags = ext_flags;
  te.sample_id = sync_sample_id;
  te.phys_pwr_L100 = sat_i16(phys_pwr_L * 100.0f);
  te.phys_pwr_R100 = sat_i16(phys_pwr_R * 100.0f);
  te.sync_ang_L100 = sync_ang_L100;
  te.sync_ang_R100 = sync_ang_R100;
  te.sync_vel_L10 = sync_vel_L10;
  te.sync_vel_R10 = sync_vel_R10;
  te.sync_cmd_L100 = sync_cmd_L100;
  te.sync_cmd_R100 = sync_cmd_R100;
  te.ctrl_pwr_L100 = ctrl_pwr_L100;
  te.ctrl_pwr_R100 = ctrl_pwr_R100;
  te.age_ms_10 = ext_age_ms_10;
  ble_pack_telem_ext(data_ble, te);

  Serial5.write(data_ble, BLE_FRAME_LEN);
}
