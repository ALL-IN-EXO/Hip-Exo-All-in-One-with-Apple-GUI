#include "Controller_RL.h"
#include <math.h>

// Serial8 on Teensy 4.1: RX8=pin34, TX8=pin35
#define PI_SERIAL  Serial8
#define PI_BAUD    115200

Controller_RL::Controller_RL() {
  reset();
  memset(logtag, 0, sizeof(logtag));
  memset(rpi_status_buf, 0, sizeof(rpi_status_buf));
  rpi_status_valid = false;
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
          int o = 2;
          memcpy(&sync_sample_id, rx_sync_buf_ + o, 2); o += 2;

          memcpy(&tau_pi_L_, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_R_, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Lp, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Ld, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Rp, rx_sync_buf_ + o, 4); o += 4;
          memcpy(&tau_pi_Rd, rx_sync_buf_ + o, 4); o += 4;

          memcpy(&sync_ang_L100, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&sync_ang_R100, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&sync_vel_L10, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&sync_vel_R10, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&sync_ctrl_pwr_L100, rx_sync_buf_ + o, 2); o += 2;
          memcpy(&sync_ctrl_pwr_R100, rx_sync_buf_ + o, 2); o += 2;
          sync_flags = rx_sync_buf_[o];

          sync_valid = true;
          sync_from_pi = true;
          last_rx_ms_ = millis();
          pi_connected_ = true;
          got_torque = true;
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
