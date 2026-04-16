#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

/************************************************************
 * BleProtocol.h — 128 字节 BLE 帧打包/解析
 *
 * 帧格式: [0xA5] [0x5A] [0x80(=128)] [payload 125 bytes]
 *
 * =========================================================
 * 上行 Teensy → GUI (data_ble[0..127])
 * =========================================================
 * [0..2]     帧头: 0xA5, 0x5A, 0x80
 * --- Teensy 本地数据 [3..60] ---
 * [3..4]     t_cs         uint16  时间戳(厘秒)
 * [5..6]     L_ang100     int16   左腿角度×100
 * [7..8]     R_ang100     int16   右腿角度×100
 * [9..10]    L_tau100     int16   左腿实测扭矩×100
 * [11..12]   R_tau100     int16   右腿实测扭矩×100
 * [13..14]   L_cmd100     int16   左腿命令扭矩×100
 * [15..16]   R_cmd100     int16   右腿命令扭矩×100
 * [17]       imu_init_ok  uint8   IMU 状态
 * [18..19]   mt100        int16   max_torque_cfg×100
 * [20]       g_init_status uint8  SD 状态
 * [21..22]   (reserved)
 * [23..24]   gf100        int16   步态频率×100
 * [25]       logtag_valid uint8
 * [26]       logtag_char  uint8   首字母
 * [27]       imu_ok_bits  uint8   6路IMU状态位
 * [28]       current_brand uint8  当前电机品牌
 * [29]       temp_L       int8    左电机温度
 * [30]       temp_R       int8    右电机温度
 * [31]       active_algo  uint8   当前算法 ID
 * [32..33]   TX1_100      int16   IMU1 角度×100
 * [34..35]   TX2_100      int16   IMU2 角度×100
 * [36..37]   TX3_100      int16   IMU3 角度×100
 * [38..39]   TX4_100      int16   IMU4 角度×100
 * [40..41]   LTAVx_10     int16   左腿角速度×10 (deg/s)
 * [42..43]   RTAVx_10     int16   右腿角速度×10 (deg/s)
 * [44..45]   VTX1_10      int16   IMU1角速度×10 (deg/s)
 * [46..47]   VTX2_10      int16   IMU2角速度×10 (deg/s)
 * [48..49]   VTX3_10      int16   IMU3角速度×10 (deg/s)
 * [50..51]   VTX4_10      int16   IMU4角速度×10 (deg/s)
 * [52..60]   (reserved for future Teensy data)
 * --- RPi 透传区 [61..100] ---
 * [61..100]  rpi_passthru  40 bytes, Teensy 不解析，原样透传
 * --- 预留 [101..127] ---
 * [101..127] reserved
 *
 * =========================================================
 * 下行 GUI → Teensy (data_rs232_rx[0..124], 去掉3字节帧头)
 * =========================================================
 * payload 索引 (帧头后的偏移):
 * [0]        algo_select   uint8   算法选择: 0=EG, 1=Samsung, 2=RL, 3=Test, 4=SOGI
 * [1]        brand_request uint8   电机品牌: 0=不变, 1=SIG, 2=TMOTOR
 * [2]        ctrl_flags    uint8   控制标志
 *              bit0: imu_reinit
 *              bit1: motor_reinit
 *              bit2-3: dir_bits (L,R)
 * [3..4]     max_torque_cfg int16  最大扭矩×100
 * --- 算法专用参数区 [5..57] (53 bytes) ---
 *   EG 参数 (algo=0):
 *     [5..6]   Rescaling_gain ×100    int16
 *     [7..8]   Flex_Assist_gain ×100  int16
 *     [9..10]  Ext_Assist_gain ×100   int16
 *     [11]     Assist_delay_gain      uint8 (0-99, phase index @BASE_FREQ=0.7Hz; auto-scaled by gait freq)
 *     [12]     phase_offset_L         int8
 *     [13]     phase_offset_R         int8
 *     [14..15] gate_k ×100            int16
 *     [16..17] gate_p_on ×100         int16
 *     [18..19] lead_frac ×1000        int16
 *     [20..21] ext_phase_frac_L ×1000 int16
 *     [22..23] ext_phase_frac_R ×1000 int16
 *     [24..25] ext_gain ×100          int16
 *     [26..27] scale_all ×100         int16
 *     [29..30] eg_post_delay_ms       int16 (auto delay 后处理基础延迟 ms, 仅 EG 用)
 *   Samsung 参数 (algo=1):
 *     [5..6]   kappa ×100             int16
 *     [7..8]   delay_ms               int16
 * --- 通用自动延迟控制 [28] ---
 *   [28]     auto_delay_flags  uint8
 *              bit0: auto_delay_enable (EG/Samsung/非RL 通用)
 *   RL 参数 (algo=2):
 *     (暂无 Teensy 侧参数, RPi 参数通过透传区)
 *   Test 参数 (algo=3):
 *     [5..6]   test_torque_Nm ×100    int16  (amplitude)
 *     [7]      test_waveform          uint8  (0=const, 1=sin)
 *     [8..9]   test_freq_hz ×100      int16  (sin 频率)
 *   SOGI 参数 (algo=4):
 *     [5..6]   sogi_A_gain ×100       int16  (Nm, 力矩幅值)
 *     [7..8]   sogi_phi_lead_deg ×100 int16  (相位超前°)
 *     [9..10]  sogi_amp_min ×10       int16  (deg/s, 幅值看门狗)
 * --- RPi 透传区 [58..97] (40 bytes) ---
 * [58..97]   rpi_passthru  GUI→Teensy→RPi (Serial8 转发)
 * --- 预留 [98..124] ---
 * [98..124]  reserved
 ************************************************************/

#include <Arduino.h>
#include <cstring>

// 帧常量
static const uint8_t BLE_FRAME_LEN   = 128;
static const uint8_t BLE_HEADER_LEN  = 3;
static const uint8_t BLE_PAYLOAD_LEN = BLE_FRAME_LEN - BLE_HEADER_LEN; // 125

// 算法 ID
enum AlgoID : uint8_t {
  ALGO_EG      = 0,
  ALGO_SAMSUNG = 1,
  ALGO_RL      = 2,
  ALGO_TEST    = 3,
  ALGO_SOGI    = 4,
};

// ========== 辅助函数 ==========

static inline void ble_put_u8(uint8_t* buf, int idx, uint8_t v) {
  buf[idx] = v;
}
static inline void ble_put_i16(uint8_t* buf, int idx, int16_t v) {
  buf[idx]   = (uint8_t)(v & 0xFF);
  buf[idx+1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline int16_t ble_rd_i16(const uint8_t* buf, int idx) {
  return (int16_t)(buf[idx] | (buf[idx+1] << 8));
}
static inline uint8_t ble_rd_u8(const uint8_t* buf, int idx) {
  return buf[idx];
}

// ========== 上行打包 (Teensy → GUI) ==========
// 直接操作 data_ble[128] 数组，不需要额外结构体

static inline void ble_pack_header(uint8_t* data_ble) {
  data_ble[0] = 0xA5;
  data_ble[1] = 0x5A;
  data_ble[2] = BLE_FRAME_LEN;
}

struct BleUplinkData {
  uint16_t t_cs;
  int16_t  L_ang100, R_ang100;
  int16_t  L_tau100, R_tau100;
  int16_t  L_cmd100, R_cmd100;
  uint8_t  imu_init_ok;
  int16_t  mt100;
  uint8_t  g_init_status;
  int16_t  gf100;
  uint8_t  logtag_valid;
  uint8_t  logtag_char;
  uint8_t  imu_ok_bits;
  uint8_t  current_brand;
  int8_t   temp_L, temp_R;
  uint8_t  active_algo;
  int16_t  TX1_100, TX2_100, TX3_100, TX4_100;  // IMU 1-4 angles
  int16_t  LTAVx_10, RTAVx_10;                  // Left/Right IMU angular velocity
  int16_t  VTX1_10, VTX2_10, VTX3_10, VTX4_10; // IMU 1-4 angular velocity
};

static inline void ble_pack_uplink(uint8_t* data_ble, const BleUplinkData& d) {
  ble_pack_header(data_ble);
  // [3..4] time
  ble_put_i16(data_ble, 3, (int16_t)d.t_cs);
  // [5..16] angles, torques, commands
  ble_put_i16(data_ble, 5,  d.L_ang100);
  ble_put_i16(data_ble, 7,  d.R_ang100);
  ble_put_i16(data_ble, 9,  d.L_tau100);
  ble_put_i16(data_ble, 11, d.R_tau100);
  ble_put_i16(data_ble, 13, d.L_cmd100);
  ble_put_i16(data_ble, 15, d.R_cmd100);
  // [17] IMU
  data_ble[17] = d.imu_init_ok;
  // [18..19] max torque
  ble_put_i16(data_ble, 18, d.mt100);
  // [20] SD status
  data_ble[20] = d.g_init_status;
  // [21..22] reserved
  data_ble[21] = 0;
  data_ble[22] = 0;
  // [23..24] gait freq
  ble_put_i16(data_ble, 23, d.gf100);
  // [25..27]
  data_ble[25] = d.logtag_valid;
  data_ble[26] = d.logtag_char;
  data_ble[27] = d.imu_ok_bits;
  // [28..31]
  data_ble[28] = d.current_brand;
  data_ble[29] = d.temp_L;
  data_ble[30] = d.temp_R;
  data_ble[31] = d.active_algo;
  // [32..39] IMU 1-4 angles
  ble_put_i16(data_ble, 32, d.TX1_100);
  ble_put_i16(data_ble, 34, d.TX2_100);
  ble_put_i16(data_ble, 36, d.TX3_100);
  ble_put_i16(data_ble, 38, d.TX4_100);
  // [40..51] IMU velocities
  ble_put_i16(data_ble, 40, d.LTAVx_10);
  ble_put_i16(data_ble, 42, d.RTAVx_10);
  ble_put_i16(data_ble, 44, d.VTX1_10);
  ble_put_i16(data_ble, 46, d.VTX2_10);
  ble_put_i16(data_ble, 48, d.VTX3_10);
  ble_put_i16(data_ble, 50, d.VTX4_10);
}

// 上行 RPi 透传：Teensy 把从 Serial8 收到的数据放到 [61..100]
static inline void ble_pack_rpi_uplink(uint8_t* data_ble, const uint8_t* rpi_data, uint8_t len) {
  if (len > 40) len = 40;
  memcpy(&data_ble[61], rpi_data, len);
}

// ========== 下行解析 (GUI → Teensy) ==========
// payload = 帧头之后的 125 bytes

struct BleDownlinkData {
  // 通用字段
  uint8_t  algo_select;     // [0]
  uint8_t  brand_request;   // [1]
  bool     imu_reinit;      // ctrl_flags bit0
  bool     motor_reinit;    // ctrl_flags bit1
  uint8_t  dir_bits;        // ctrl_flags bit2-3
  float    max_torque_cfg;  // [3..4] / 100

  // EG 参数
  float Rescaling_gain;     // [5..6]
  float Flex_Assist_gain;   // [7..8]
  float Ext_Assist_gain;    // [9..10]
  uint8_t Assist_delay_gain;// [11]
  int8_t phase_offset_L;    // [12]
  int8_t phase_offset_R;    // [13]
  float gate_k;             // [14..15]
  float gate_p_on;          // [16..17]
  float lead_frac;          // [18..19]
  float ext_phase_frac_L;   // [20..21]
  float ext_phase_frac_R;   // [22..23]
  float ext_gain;           // [24..25]
  float scale_all;          // [26..27]

  // Samsung 参数
  float sam_kappa;          // [5..6]
  int16_t sam_delay_ms;     // [7..8]

  // Test 参数
  float test_torque_Nm;     // [5..6] amplitude
  uint8_t test_waveform;    // [7] 0=const, 1=sin
  float test_freq_hz;       // [8..9]

  // SOGI 参数
  float sogi_A_gain;         // [5..6]   Nm
  float sogi_phi_lead_deg;   // [7..8]   deg
  float sogi_amp_min;        // [9..10]  deg/s

  // 通用自动延迟控制 (EG/Samsung 适用; RL 用 rpi_passthru 内的位)
  bool    auto_delay_enable;  // payload[28] bit0
  int16_t eg_post_delay_ms;   // payload[29..30], EG 后处理延迟基础值 (ms)

  // RPi 透传区
  uint8_t rpi_passthru[40]; // [58..97]
  bool    has_rpi_data;
};

static inline BleDownlinkData ble_parse_downlink(const uint8_t* payload) {
  BleDownlinkData d;
  memset(&d, 0, sizeof(d));

  d.algo_select    = payload[0];
  d.brand_request  = payload[1];

  uint8_t flags    = payload[2];
  d.imu_reinit     = (flags & 0x01) != 0;
  d.motor_reinit   = (flags & 0x02) != 0;
  d.dir_bits       = (flags >> 2) & 0x03;

  d.max_torque_cfg = ble_rd_i16(payload, 3) / 100.0f;

  // 根据算法类型解析参数区 [5..57]
  switch (d.algo_select) {
    case ALGO_EG:
      d.Rescaling_gain    = ble_rd_i16(payload, 5)  / 100.0f;
      d.Flex_Assist_gain  = ble_rd_i16(payload, 7)  / 100.0f;
      d.Ext_Assist_gain   = ble_rd_i16(payload, 9)  / 100.0f;
      d.Assist_delay_gain = payload[11];
      d.phase_offset_L    = (int8_t)payload[12];
      d.phase_offset_R    = (int8_t)payload[13];
      d.gate_k            = ble_rd_i16(payload, 14) / 100.0f;
      d.gate_p_on         = ble_rd_i16(payload, 16) / 100.0f;
      d.lead_frac         = ble_rd_i16(payload, 18) / 1000.0f;
      d.ext_phase_frac_L  = ble_rd_i16(payload, 20) / 1000.0f;
      d.ext_phase_frac_R  = ble_rd_i16(payload, 22) / 1000.0f;
      d.ext_gain          = ble_rd_i16(payload, 24) / 100.0f;
      d.scale_all         = ble_rd_i16(payload, 26) / 100.0f;
      break;

    case ALGO_SAMSUNG:
      d.sam_kappa    = ble_rd_i16(payload, 5) / 100.0f;
      d.sam_delay_ms = ble_rd_i16(payload, 7);
      break;

    case ALGO_RL:
      // RL 参数通过 RPi 透传区
      break;

    case ALGO_TEST:
      d.test_torque_Nm = ble_rd_i16(payload, 5) / 100.0f;
      d.test_waveform  = payload[7];
      d.test_freq_hz   = ble_rd_i16(payload, 8) / 100.0f;
      break;

    case ALGO_SOGI:
      d.sogi_A_gain        = ble_rd_i16(payload, 5) / 100.0f;
      d.sogi_phi_lead_deg  = ble_rd_i16(payload, 7) / 100.0f;
      d.sogi_amp_min       = ble_rd_i16(payload, 9) / 10.0f;
      break;
  }

  // 通用自动延迟控制 [28..30] (EG/Samsung; RL 通过 rpi_passthru 传)
  d.auto_delay_enable = (payload[28] & 0x01) != 0;
  d.eg_post_delay_ms  = ble_rd_i16(payload, 29);

  // RPi 透传区 [58..97]
  memcpy(d.rpi_passthru, &payload[58], 40);
  // 检查是否有非零数据
  d.has_rpi_data = false;
  for (int i = 0; i < 40; i++) {
    if (d.rpi_passthru[i] != 0) { d.has_rpi_data = true; break; }
  }

  return d;
}

#endif // BLE_PROTOCOL_H
