#include "Controller_RL.h"
#include <math.h>

// Serial8 on Teensy 4.1: RX8=pin34, TX8=pin35
#define PI_SERIAL  Serial8
#define PI_BAUD    115200

// Teensy HardwareSerial 默认 RX 缓冲 = 64B。Pi 100Hz × 42B(AA59) + 周期性 AA56,
// 一旦主循环被 BLE/SD/算法拖 >6ms, 64B 会 overrun, AA59 没校验和, 状态机会把
// 下一帧头几个字节当成本帧尾 → 解出接近 int16 饱和的角度/速度 (现场观测 ~2.1%)。
// 扩到 512B, 可容忍 ~45ms 的主循环停顿 (512/11520 s), 实际足够覆盖所有已知抖动源。
static uint8_t PI_SERIAL_RX_EXTRA[512];

Controller_RL::Controller_RL() {
  reset();
  memset(logtag, 0, sizeof(logtag));
  memset(rpi_status_buf, 0, sizeof(rpi_status_buf));
  rpi_status_valid = false;
  bad_sync_frames = 0;
}

void Controller_RL::reset() {
  tau_pi_L_ = 0.0f;
  tau_pi_R_ = 0.0f;
  tau_pi_Lp = 0.0f;
  tau_pi_Rp = 0.0f;
  tau_pi_Ld = 0.0f;
  tau_pi_Rd = 0.0f;
  pi_connected_ = false;
  last_rx_ms_ = 0;
  rx_state_ = WAIT_HEADER1;
  rx_idx_ = 0;
  rx_pkt_type_ = 0;
  sync_valid = false;
  sync_from_pi = false;
  sync_flags = 0;
  sync_sample_id = 0;
  sync_ang_L100 = 0;
  sync_ang_R100 = 0;
  sync_vel_L10 = 0;
  sync_vel_R10 = 0;
  sync_ctrl_pwr_L100 = 0;
  sync_ctrl_pwr_R100 = 0;
}

void Controller_RL::init_serial() {
  PI_SERIAL.addMemoryForRead(PI_SERIAL_RX_EXTRA, sizeof(PI_SERIAL_RX_EXTRA));
  PI_SERIAL.begin(PI_BAUD);
}

void Controller_RL::parse_params(const BleDownlinkData& dl) {
  // RL 的参数主要通过 RPi 透传区传递
  // Teensy 侧不解析额外参数
}

uint8_t Controller_RL::cksum8(const uint8_t* p, size_t n) {
  uint8_t s = 0;
  while (n--) s += *p++;
  return s;
}

void Controller_RL::send_imu_to_pi(uint16_t t_cs, float Lpos_rad, float Rpos_rad,
                                    float Lvel, float Rvel) {
  // v2 packet (34 bytes): 4(header) + 29(payload) + 1(checksum)
  // payload = t_cs(2) + Lpos(4) + Rpos(4) + Lvel(4) + Rvel(4) + logtag(11)
  uint8_t pkt[34];
  pkt[0] = 0xA5;
  pkt[1] = 0x5A;
  pkt[2] = 30;      // TYPE(1) + payload(29)
  pkt[3] = 0x01;    // IMU packet type

  memcpy(&pkt[4],  &t_cs,      2);
  memcpy(&pkt[6],  &Lpos_rad,  4);
  memcpy(&pkt[10], &Rpos_rad,  4);
  memcpy(&pkt[14], &Lvel,      4);
  memcpy(&pkt[18], &Rvel,      4);
  memcpy(&pkt[22], logtag,    11);

  pkt[33] = cksum8(&pkt[3], 30);
  PI_SERIAL.write(pkt, sizeof(pkt));
}

bool Controller_RL::read_torque_from_pi() {
  /*
   * RPi sends two packet types:
   *   AA 55 + 24B (6×float32) = torque      → PI_TORQUE_SIZE=26
   *   AA 56 + 40B (status)    = status       → PI_STATUS_SIZE=42
   *   AA 59 + 40B (sync+torque)= sync torque → PI_SYNC_SIZE=42
   *
   * State machine parses both, byte by byte.
   */
  bool got_torque = false;

  while (PI_SERIAL.available()) {
    uint8_t b = PI_SERIAL.read();

    switch (rx_state_) {
      case WAIT_HEADER1:
        if (b == 0xAA) rx_state_ = WAIT_HEADER2;
        break;

      case WAIT_HEADER2:
        if (b == 0x55 || b == 0x56 || b == 0x59) {
          rx_pkt_type_ = b;
          rx_idx_ = 2;  // header already consumed
          if (b == 0x55) {
            rx_state_ = READING_TORQUE;
            rx_torque_buf_[0] = 0xAA;
            rx_torque_buf_[1] = 0x55;
          } else if (b == 0x56) {
            rx_state_ = READING_STATUS;
            rx_status_buf_[0] = 0xAA;
            rx_status_buf_[1] = 0x56;
          } else {
            rx_state_ = READING_SYNC;
            rx_sync_buf_[0] = 0xAA;
            rx_sync_buf_[1] = 0x59;
          }
        } else {
          rx_state_ = WAIT_HEADER1;
        }
        break;

      case READING_TORQUE:
        rx_torque_buf_[rx_idx_++] = b;
        if (rx_idx_ == PI_TORQUE_SIZE) {
          // Parse 6 floats from offset 2
          int o = 2;
          memcpy(&tau_pi_L_, rx_torque_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_R_, rx_torque_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Lp, rx_torque_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Ld, rx_torque_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Rp, rx_torque_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Rd, rx_torque_buf_ + o, 4);

          sync_valid = false;
          sync_from_pi = false;
          sync_flags = 0;
          last_rx_ms_ = millis();
          pi_connected_ = true;
          got_torque = true;
          rx_state_ = WAIT_HEADER1;
        }
        break;

      case READING_STATUS:
        rx_status_buf_[rx_idx_++] = b;
        if (rx_idx_ == PI_STATUS_SIZE) {
          // Copy 40-byte payload (after AA 56 header) to rpi_status_buf
          memcpy(rpi_status_buf, rx_status_buf_ + 2, 40);
          rpi_status_valid = true;
          last_rx_ms_ = millis();
          pi_connected_ = true;
          rx_state_ = WAIT_HEADER1;
        }
        break;

      case READING_SYNC:
        rx_sync_buf_[rx_idx_++] = b;
        if (rx_idx_ == PI_SYNC_SIZE) {
          // AA59 无校验和; Serial8 RX overrun 时本帧尾部可能混入下一帧头几个字节,
          // 解出的 int16 会接近饱和边界。先解到临时变量, 通过物理量健壮性检查后
          // 才提交到成员; 坏帧直接丢弃, 沿用上一帧有效值 (Pi→Teensy 控制链继续用
          // 上一帧扭矩即可, 无需 reset tau_pi_L_/R_)。
          int o = 2;
          uint16_t tmp_sample_id;
          float tmp_tau_L, tmp_tau_R, tmp_Lp, tmp_Ld, tmp_Rp, tmp_Rd;
          int16_t tmp_ang_L, tmp_ang_R, tmp_vel_L, tmp_vel_R, tmp_pwr_L, tmp_pwr_R;

          memcpy(&tmp_sample_id, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_tau_L, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_tau_R, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_Lp,    rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_Ld,    rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_Rp,    rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_Rd,    rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tmp_ang_L, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_ang_R, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_vel_L, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_vel_R, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_pwr_L, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&tmp_pwr_R, rx_sync_buf_ + o, 2); o += 2;
          uint8_t tmp_flags = rx_sync_buf_[o];

          // 物理健壮性检查 (scale: ang×100, vel×10, pwr×100)
          //   |angle| ≤ 200°  ⇒ |raw| ≤ 20000
          //   |vel|   ≤ 2500 dps ⇒ |raw| ≤ 25000
          //   |pwr|   ≤ 300 W ⇒ |raw| ≤ 30000
          // float 字段不检 (NaN/Inf 也会被下游限幅到 ±max_torque_cfg)。
          const bool sane =
              (tmp_ang_L > -20000 && tmp_ang_L < 20000) &&
              (tmp_ang_R > -20000 && tmp_ang_R < 20000) &&
              (tmp_vel_L > -25000 && tmp_vel_L < 25000) &&
              (tmp_vel_R > -25000 && tmp_vel_R < 25000) &&
              (tmp_pwr_L > -30000 && tmp_pwr_L < 30000) &&
              (tmp_pwr_R > -30000 && tmp_pwr_R < 30000);

          if (sane) {
            sync_sample_id = tmp_sample_id;
            tau_pi_L_ = tmp_tau_L; tau_pi_R_ = tmp_tau_R;
            tau_pi_Lp = tmp_Lp;    tau_pi_Ld = tmp_Ld;
            tau_pi_Rp = tmp_Rp;    tau_pi_Rd = tmp_Rd;
            sync_ang_L100 = tmp_ang_L;
            sync_ang_R100 = tmp_ang_R;
            sync_vel_L10  = tmp_vel_L;
            sync_vel_R10  = tmp_vel_R;
            sync_ctrl_pwr_L100 = tmp_pwr_L;
            sync_ctrl_pwr_R100 = tmp_pwr_R;
            sync_flags = tmp_flags;
            sync_valid = true;
            sync_from_pi = true;
            last_rx_ms_ = millis();
            pi_connected_ = true;
            got_torque = true;
          } else {
            bad_sync_frames++;
          }
          rx_state_ = WAIT_HEADER1;
        }
        break;

      default:
        rx_state_ = WAIT_HEADER1;
        break;
    }
  }
  return got_torque;
}

void Controller_RL::compute(const CtrlInput& in, CtrlOutput& out) {
  // 1) 发送 IMU 数据给 RPi (compat with legacy pipeline: deg / deg/s)
  float Lpos = in.LTx;
  float Rpos = in.RTx;
  float Lvel = in.LTAVx;
  float Rvel = in.RTAVx;
  uint16_t t_cs = (uint16_t)((in.current_time_us / 10000UL) & 0xFFFF);

  send_imu_to_pi(t_cs, Lpos, Rpos, Lvel, Rvel);

  // 2) 尝试读取 RPi 返回的扭矩 (同时也处理状态包)
  read_torque_from_pi();

  // 3) 超时检测 (500ms 没收到就归零)
  if (millis() - last_rx_ms_ > 500) {
    tau_pi_L_ = 0.0f;
    tau_pi_R_ = 0.0f;
    pi_connected_ = false;
    sync_valid = false;
    sync_from_pi = false;
    sync_flags = 0;
  }

  // 4) 限幅输出
  float max_abs = fabsf(in.max_torque_cfg);
  float tL = tau_pi_L_;
  float tR = tau_pi_R_;
  if (tL >  max_abs) tL =  max_abs;
  if (tL < -max_abs) tL = -max_abs;
  if (tR >  max_abs) tR =  max_abs;
  if (tR < -max_abs) tR = -max_abs;

  out.tau_L = tL;
  out.tau_R = tR;
}
