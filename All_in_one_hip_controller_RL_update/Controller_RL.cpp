#include "Controller_RL.h"
#include <math.h>

// Serial8 on Teensy 4.1: RX8=pin34, TX8=pin35
#define PI_SERIAL  Serial8
#define PI_BAUD    115200

Controller_RL::Controller_RL() {
  reset();
  memset(logtag, 0, sizeof(logtag));
  exo_delay = 0.0f;
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

void Controller_RL::send_imu_to_pi(float Lpos_rad, float Rpos_rad,
                                    float Lvel, float Rvel) {
  // 36 bytes: 4(header) + 31(payload) + 1(checksum)
  uint8_t pkt[36];
  pkt[0] = 0xA5;
  pkt[1] = 0x5A;
  pkt[2] = 32;      // TYPE(1) + payload(31)
  pkt[3] = 0x01;    // IMU packet type

  memcpy(&pkt[4],  &Lpos_rad, 4);
  memcpy(&pkt[8],  &Rpos_rad, 4);
  memcpy(&pkt[12], &Lvel,     4);
  memcpy(&pkt[16], &Rvel,     4);
  memcpy(&pkt[20], &exo_delay, 4);
  memcpy(&pkt[24], logtag,    11);

  pkt[35] = cksum8(&pkt[3], 32);
  PI_SERIAL.write(pkt, sizeof(pkt));
}

bool Controller_RL::read_torque_from_pi() {
  /*
   * RPi sends two packet types:
   *   AA 55 + 24B (6×float32) = torque      → PI_TORQUE_SIZE=26
   *   AA 56 + 40B (status)    = status       → PI_STATUS_SIZE=42
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
        if (b == 0x55 || b == 0x56) {
          rx_pkt_type_ = b;
          rx_idx_ = 2;  // header already consumed
          rx_state_ = (b == 0x55) ? READING_TORQUE : READING_STATUS;
          if (b == 0x55) {
            rx_torque_buf_[0] = 0xAA;
            rx_torque_buf_[1] = 0x55;
          } else {
            rx_status_buf_[0] = 0xAA;
            rx_status_buf_[1] = 0x56;
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

  send_imu_to_pi(Lpos, Rpos, Lvel, Rvel);

  // 2) 尝试读取 RPi 返回的扭矩 (同时也处理状态包)
  read_torque_from_pi();

  // 3) 超时检测 (500ms 没收到就归零)
  if (millis() - last_rx_ms_ > 500) {
    tau_pi_L_ = 0.0f;
    tau_pi_R_ = 0.0f;
    pi_connected_ = false;
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
