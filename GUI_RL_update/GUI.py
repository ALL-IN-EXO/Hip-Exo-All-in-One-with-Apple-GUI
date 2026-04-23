import serial
from serial.tools import list_ports
import struct
import time
import csv
import json
import bisect
from math import *
from collections import deque
import configparser
import shlex
import re
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import sys
import os
import subprocess
import tempfile
import shutil
import threading
import queue
from datetime import datetime
"""
Hip-Exo GUI v4.1 — Apple HIG Design
-------------------------------------
Apple Human Interface Guidelines inspired:
- SF Pro typography, 8pt grid spacing
- iOS toggle switches, segmented controls
- Card containers with shadows
- Pill status badges, connection dot
- Vibrancy-style dark/light themes
"""

# ============== Frame Constants ==============
BLE_FRAME_LEN   = 128
BLE_HEADER      = bytes([0xA5, 0x5A, BLE_FRAME_LEN])
BLE_PAYLOAD_LEN = BLE_FRAME_LEN - 3

ALGO_EG      = 0
ALGO_SAMSUNG = 1
ALGO_RL      = 2
ALGO_TEST    = 3
ALGO_SOGI    = 4
ALGO_NAMES = {ALGO_EG: "EG", ALGO_SAMSUNG: "Samsung", ALGO_RL: "RL",
              ALGO_TEST: "Test", ALGO_SOGI: "SOGI"}
# UI segment order (SOGI before Test); decoupled from algo ID because SOGI_id=4 > TEST_id=3.
_SEG_TO_ALGO = [ALGO_EG, ALGO_SAMSUNG, ALGO_RL, ALGO_SOGI, ALGO_TEST]
_ALGO_TO_SEG = {a: i for i, a in enumerate(_SEG_TO_ALGO)}

# RPi passthrough payload format (40 bytes, packed into BLE payload[58:98])
RPI_PT_MAGIC0 = 0x52  # 'R'
RPI_PT_MAGIC1 = 0x4C  # 'L'
RPI_PT_VERSION = 0x01            # downlink (GUI → RPi) passthrough version
RPI_STATUS_VERSION_LEGACY = 0x02  # uplink status v2: single-leg float32 metrics
RPI_STATUS_VERSION_PER_LEG = 0x03  # uplink status v3: per-leg int16-packed metrics
RPI_AUTO_FLAG_ENABLE = 0x01
RPI_AUTO_FLAG_MOTION_VALID_L = 0x02
RPI_AUTO_FLAG_MOTION_VALID_R = 0x04
RPI_AUTO_FLAG_METHOD_BO = 0x08

# Teensy telemetry extension payload [98..124] (maps data_ble[101..127])
TELEM_EXT_OFFSET = 98
TELEM_EXT_LEN = 27
TELEM_EXT_MAGIC0 = 0x58  # 'X'
TELEM_EXT_MAGIC1 = 0x54  # 'T'
TELEM_EXT_VERSION = 0x01
TELEM_EXT_FLAG_VALID = 0x01
TELEM_EXT_FLAG_PHYS_VALID = 0x02
TELEM_EXT_FLAG_CTRL_VALID = 0x04
TELEM_EXT_FLAG_SYNC_VALID = 0x08
TELEM_EXT_FLAG_SYNC_FROM_PI = 0x10

# Filter type codes (keep in sync with RL_controller_torch.py)
RL_FILTER_TYPES = [
    ("Butterworth", 1),
    ("Bessel", 2),
    ("Chebyshev2", 3),
]

RPI_RL_TMUX_SESSION = "hip_rl"
RPI_RL_POLL_INTERVAL_S = 1.0
RPI_RL_SSH_TIMEOUT_S = 8.0

CONN_TIMEOUT_S = 2.0
AUTO_CONNECT_COOLDOWN_S = 2.0
AUTO_CONNECT_ADAFRUIT_VIDS = {0x239A}

REPLAY_AUTO_COMPUTE = "__AUTO_COMPUTE__"
REPLAY_POWER_COLUMNS = {"L_pwr_W", "R_pwr_W"}
REPLAY_PROGRESS_STEPS = 10000
REPLAY_OPTIONAL_COLUMNS = {"L_vel_dps", "R_vel_dps", "L_pwr_W", "R_pwr_W"}
REPLAY_TIME_CANDIDATES = [
    "teensy_t_s_unwrapped", "teensy_t_unwrapped_s", "t_unwrapped_s",
    "teensy_t_ms_unwrapped",
    "wall_elapsed_s", "t_s", "time_s", "timestamp_s",
    "Time_ms", "time_ms",
    "teensy_t_cs_u16", "t_cs",
]
REPLAY_PLOT_COLUMN_ALIASES = {
    "L_angle_deg": ["L_angle_deg", "L_angle", "left_angle_deg", "imu_LTx", "sync_LTx"],
    "R_angle_deg": ["R_angle_deg", "R_angle", "right_angle_deg", "imu_RTx", "sync_RTx"],
    "L_cmd_Nm": ["L_cmd_Nm", "L_cmd", "left_cmd_Nm", "L_command_actuator", "sync_L_cmd_Nm", "sync_L_cmd"],
    "R_cmd_Nm": ["R_cmd_Nm", "R_cmd", "right_cmd_Nm", "R_command_actuator", "sync_R_cmd_Nm", "sync_R_cmd"],
    "L_est_Nm": ["L_est_Nm", "L_est", "left_est_Nm", "L_torque_meas", "filtered_LExoTorque", "raw_LExoTorque", "L_command_actuator"],
    "R_est_Nm": ["R_est_Nm", "R_est", "right_est_Nm", "R_torque_meas", "filtered_RExoTorque", "raw_RExoTorque", "R_command_actuator"],
    "L_vel_dps": ["L_vel_dps", "L_vel", "left_vel_dps", "imu_Lvel", "sync_Lvel"],
    "R_vel_dps": ["R_vel_dps", "R_vel", "right_vel_dps", "imu_Rvel", "sync_Rvel"],
    "L_pwr_W": ["L_pwr_W", "L_pwr", "left_pwr_W", "sync_ctrl_pwr_L", "phys_L_pwr_W", "ctrl_L_pwr_W", "L_pwr"],
    "R_pwr_W": ["R_pwr_W", "R_pwr", "right_pwr_W", "sync_ctrl_pwr_R", "phys_R_pwr_W", "ctrl_R_pwr_W", "R_pwr"],
}

# ============== Apple Color System ==============
class AppleColors:
    """Apple HIG system colors"""
    # Light mode
    class Light:
        bg          = '#f2f2f7'
        card        = '#ffffff'
        text        = '#1c1c1e'
        text2       = '#8e8e93'
        separator   = '#c6c6c8'
        blue        = '#007aff'
        green       = '#34c759'
        red         = '#ff3b30'
        orange      = '#ff9500'
        yellow      = '#ffcc00'
        teal        = '#5ac8fa'
        purple      = '#af52de'
        fill        = '#e5e5ea'
        group_bg    = '#ffffff'
        plot_bg     = '#ffffff'
        plot_fg     = '#1c1c1e'
        grid        = '#e5e5ea'

    # Dark mode
    class Dark:
        bg          = '#000000'
        card        = '#1c1c1e'
        text        = '#ffffff'
        text2       = '#8e8e93'
        separator   = '#38383a'
        blue        = '#0a84ff'
        green       = '#30d158'
        red         = '#ff453a'
        orange      = '#ff9f0a'
        yellow      = '#ffd60a'
        teal        = '#64d2ff'
        purple      = '#bf5af2'
        fill        = '#2c2c2e'
        group_bg    = '#1c1c1e'
        plot_bg     = '#1c1c1e'
        plot_fg     = '#ffffff'
        grid        = '#38383a'

C = AppleColors.Dark  # current palette reference (default dark)

# ============== Apple QSS Templates ==============
def _build_qss(c):
    """Build Apple-style QSS from a color palette"""
    return f"""
* {{
    font-family: -apple-system, 'SF Pro Display', 'Helvetica Neue', 'Segoe UI', sans-serif;
}}
QWidget {{
    background-color: {c.bg};
    color: {c.text};
    font-size: 13px;
}}
QGroupBox {{
    background-color: {c.card};
    border: none;
    border-radius: 12px;
    margin-top: 8px;
    padding: 10px 10px 8px 10px;
    font-size: 13px;
    font-weight: 600;
    color: {c.text};
}}
QGroupBox::title {{
    subcontrol-position: top left;
    padding: 4px 12px;
    color: {c.text2};
    font-size: 12px;
    font-weight: 500;
}}
QPushButton {{
    background-color: {c.fill};
    color: {c.text};
    border: none;
    border-radius: 8px;
    padding: 6px 14px;
    font-size: 13px;
    font-weight: 500;
}}
QPushButton:hover {{
    background-color: {c.separator};
}}
QPushButton:pressed {{
    background-color: {c.separator};
    opacity: 0.7;
}}
QComboBox {{
    background-color: {c.fill};
    color: {c.text};
    border: none;
    border-radius: 8px;
    padding: 5px 10px;
    font-size: 13px;
    min-height: 22px;
    combobox-popup: 0;
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}
QComboBox::down-arrow {{
    image: none;
    border: none;
}}
QComboBox QAbstractItemView {{
    background-color: #ffffff;
    color: #111111;
    border: 1px solid {c.separator};
    border-radius: 8px;
    selection-background-color: #dfe8ff;
    selection-color: #000000;
    padding: 4px;
    outline: none;
}}
QComboBox QAbstractItemView::item {{
    min-height: 22px;
    padding: 4px 8px;
}}
QSpinBox, QDoubleSpinBox {{
    background-color: {c.fill};
    color: {c.text};
    border: none;
    border-radius: 8px;
    padding: 4px 8px;
    font-size: 13px;
    min-height: 22px;
}}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {{
    border: none;
    width: 16px;
}}
QLineEdit {{
    background-color: {c.fill};
    color: {c.text};
    border: none;
    border-radius: 8px;
    padding: 5px 10px;
    font-size: 13px;
    selection-background-color: {c.blue};
}}
QCheckBox {{
    color: {c.text};
    font-size: 13px;
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 18px;
    height: 18px;
    border-radius: 4px;
    border: 2px solid {c.separator};
    background-color: {c.card};
}}
QCheckBox::indicator:checked {{
    background-color: {c.blue};
    border-color: {c.blue};
}}
QLabel {{
    color: {c.text};
    background: transparent;
}}
QScrollBar:vertical {{
    background: transparent;
    width: 8px;
    border: none;
}}
QScrollBar::handle:vertical {{
    background: {c.separator};
    border-radius: 4px;
    min-height: 20px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QToolTip {{
    color: #f5f5f7;
    background-color: rgba(28, 28, 30, 245);
    border: 1px solid #8e8e93;
    border-radius: 6px;
    padding: 6px 8px;
    font-size: 12px;
}}
"""


# ============== Custom iOS Toggle Switch ==============
class ToggleSwitch(QWidget):
    """iOS-style toggle switch with animation"""
    toggled = pyqtSignal(bool)

    def __init__(self, checked=False, on_color=None, parent=None):
        super().__init__(parent)
        self._checked = checked
        self._on_color = on_color or '#34c759'
        self._off_color = '#e5e5ea'
        self._handle_color = '#ffffff'
        self._anim_pos = 1.0 if checked else 0.0
        self.setFixedSize(51, 31)
        self.setCursor(Qt.PointingHandCursor)

        self._anim = QPropertyAnimation(self, b"handle_pos")
        self._anim.setDuration(200)
        self._anim.setEasingCurve(QEasingCurve.InOutCubic)

    def get_handle_pos(self):
        return self._anim_pos

    def set_handle_pos(self, val):
        self._anim_pos = val
        self.update()

    handle_pos = pyqtProperty(float, get_handle_pos, set_handle_pos)

    def isChecked(self):
        return self._checked

    def setChecked(self, val):
        if self._checked == val:
            return
        self._checked = val
        self._animate()
        self.toggled.emit(val)

    def _animate(self):
        self._anim.stop()
        self._anim.setStartValue(self._anim_pos)
        self._anim.setEndValue(1.0 if self._checked else 0.0)
        self._anim.start()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._checked = not self._checked
            self._animate()
            self.toggled.emit(self._checked)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = h / 2

        # Track
        t = self._anim_pos
        off_c = QColor(self._off_color)
        on_c = QColor(self._on_color)
        track_color = QColor(
            int(off_c.red()   + (on_c.red()   - off_c.red())   * t),
            int(off_c.green() + (on_c.green() - off_c.green()) * t),
            int(off_c.blue()  + (on_c.blue()  - off_c.blue())  * t),
        )
        p.setPen(Qt.NoPen)
        p.setBrush(track_color)
        p.drawRoundedRect(0, 0, w, h, r, r)

        # Handle
        margin = 2
        handle_d = h - 2 * margin
        handle_x = margin + t * (w - handle_d - 2 * margin)
        p.setBrush(QColor(self._handle_color))
        # Subtle shadow
        p.setPen(Qt.NoPen)
        shadow = QColor(0, 0, 0, 30)
        p.setBrush(shadow)
        p.drawEllipse(QRectF(handle_x, margin + 1, handle_d, handle_d))
        p.setBrush(QColor(self._handle_color))
        p.drawEllipse(QRectF(handle_x, margin, handle_d, handle_d))
        p.end()


# ============== Segmented Control ==============
class SegmentedControl(QWidget):
    """iOS-style segmented control"""
    currentIndexChanged = pyqtSignal(int)

    def __init__(self, items, parent=None):
        super().__init__(parent)
        self._items = items
        self._index = 0
        self._buttons = []
        layout = QHBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(0)
        self.setFixedHeight(32)

        for i, text in enumerate(items):
            btn = QPushButton(text)
            btn.setCheckable(True)
            btn.setCursor(Qt.PointingHandCursor)
            btn.setFixedHeight(28)
            btn.clicked.connect(lambda checked, idx=i: self._on_click(idx))
            self._buttons.append(btn)
            layout.addWidget(btn)

        self._buttons[0].setChecked(True)
        self._update_style()

    def _on_click(self, idx):
        if idx == self._index:
            self._buttons[idx].setChecked(True)
            return
        self._index = idx
        for i, b in enumerate(self._buttons):
            b.setChecked(i == idx)
        self._update_style()
        self.currentIndexChanged.emit(idx)

    def currentIndex(self):
        return self._index

    def setCurrentIndex(self, idx):
        if 0 <= idx < len(self._buttons):
            self._on_click(idx)

    def _update_style(self):
        c = C
        for i, b in enumerate(self._buttons):
            is_sel = (i == self._index)
            # Determine border radius
            is_first = (i == 0)
            is_last  = (i == len(self._buttons) - 1)
            if is_first and is_last:
                radius = "8px"
            elif is_first:
                radius = "8px 0px 0px 8px"
            elif is_last:
                radius = "0px 8px 8px 0px"
            else:
                radius = "0px"

            if is_sel:
                b.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {c.blue};
                        color: white;
                        border: none;
                        border-radius: {radius};
                        font-size: 13px;
                        font-weight: 600;
                        padding: 4px 12px;
                    }}
                """)
            else:
                b.setStyleSheet(f"""
                    QPushButton {{
                        background-color: {c.fill};
                        color: {c.text};
                        border: none;
                        border-radius: {radius};
                        font-size: 13px;
                        font-weight: 400;
                        padding: 4px 12px;
                    }}
                    QPushButton:hover {{
                        background-color: {c.separator};
                    }}
                """)

    def update_palette(self):
        self._update_style()


# ============== Pill Badge ==============
class PillBadge(QLabel):
    """iOS-style pill status badge"""
    def __init__(self, text="", color="#34c759", parent=None):
        super().__init__(text, parent)
        self._color = color
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(22)
        self._refresh_style()

    def set_color(self, color):
        self._color = color
        self._refresh_style()

    def _refresh_style(self):
        # Compute text color (white for dark bg, black for light bg)
        qc = QColor(self._color)
        lum = 0.299 * qc.red() + 0.587 * qc.green() + 0.114 * qc.blue()
        text_c = '#ffffff' if lum < 160 else '#000000'
        self.setStyleSheet(f"""
            background-color: {self._color};
            color: {text_c};
            border-radius: 11px;
            padding: 2px 10px;
            font-size: 11px;
            font-weight: 600;
        """)


# ============== Card Frame ==============
class CardFrame(QFrame):
    """Apple-style card with rounded corners"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.NoFrame)
        self._apply_style()

    def _apply_style(self):
        self.setStyleSheet(f"""
            CardFrame {{
                background-color: {C.card};
                border-radius: 12px;
                border: none;
            }}
        """)
        # Drop shadow
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(20)
        shadow.setXOffset(0)
        shadow.setYOffset(2)
        shadow.setColor(QColor(0, 0, 0, 25))
        self.setGraphicsEffect(shadow)

    def update_palette(self):
        self._apply_style()


# ============== Connection Dot ==============
class ConnectionDot(QWidget):
    """Colored dot indicator for connection status"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(12, 12)
        self._color = '#8e8e93'  # gray = idle

    def set_state(self, state):
        """state: 'idle', 'connected', 'warning', 'error'"""
        colors = {
            'idle':      '#8e8e93',
            'connected': C.green,
            'warning':   C.orange,
            'error':     C.red,
        }
        self._color = colors.get(state, '#8e8e93')
        self.update()

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        # Glow
        glow = QColor(self._color)
        glow.setAlpha(50)
        p.setBrush(glow)
        p.drawEllipse(0, 0, 12, 12)
        # Dot
        p.setBrush(QColor(self._color))
        p.drawEllipse(2, 2, 8, 8)
        p.end()


# ============== Helper ==============
def find_available_ports():
    # List only; do not open/close ports here to avoid unnecessary device reset.
    return [p["device"] for p in list_available_port_infos()]


def list_available_port_infos():
    infos = []
    for p in list_ports.comports():
        infos.append({
            "device": str(getattr(p, "device", "") or ""),
            "description": str(getattr(p, "description", "") or ""),
            "manufacturer": str(getattr(p, "manufacturer", "") or ""),
            "hwid": str(getattr(p, "hwid", "") or ""),
            "vid": (int(p.vid) if getattr(p, "vid", None) is not None else None),
            "pid": (int(p.pid) if getattr(p, "pid", None) is not None else None),
        })
    infos.sort(key=lambda it: it["device"])
    return infos


_section_labels = []  # track for theme updates

def _make_section_label(text):
    """Apple-style section header"""
    lbl = QLabel(text)
    lbl.setStyleSheet(f"""
        color: {C.text2};
        font-size: 12px;
        font-weight: 500;
        text-transform: uppercase;
        padding: 4px 0px;
        background: transparent;
    """)
    _section_labels.append(lbl)
    return lbl


# ============== Main Window ==============
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Hip-Exo Controller")
        self.resize(1440, 860)

        self.ser = None
        self.connected = False
        self._dark_mode = True

        # Data buffers
        self.win_size = 200
        self._init_buffers()
        # Render scheduling: keep data ingest independent from plot repaint.
        self._render_dirty = False
        self._last_render_ts = 0.0
        self._render_fps_normal = 24.0
        self._last_render_values = None
        self._prev_L_angle = 0.0
        self._prev_R_angle = 0.0
        self._prev_wall_time = 0.0

        # Control state
        self._imu_init_request = False
        self._motor_init_request = False
        self._dir_bits = 0x03
        self._visual_sign_L = 1
        self._visual_sign_R = 1
        self._brand_request = 0
        self._brand_pending = 2  # 1=SIG, 2=TMOTOR (default TMOTOR)
        self._current_brand = 0
        self._algo_select = ALGO_EG
        self._algo_pending = ALGO_EG
        self._current_tag = ""
        self._last_rx_tag_char = ""
        self._tag_started_at = None
        self._last_rx_tag_text = ""
        self._align_mark_counter = 1
        self._align_events_pending = []
        self._uplink_prev_t_cs = None
        self._uplink_wrap_count = 0
        self._uplink_t_unwrapped_s = 0.0
        self._csv_sample_idx = 0
        self._csv_session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_start_wall_time = time.time()
        self._last_gait_freq = None
        self._signal_source_mode = "Auto"   # Auto / Raw / Sync
        self._power_source_mode = "Auto"    # Auto / Physical / Control
        self._signal_source_active = "Raw"
        self._power_source_active = "Physical"
        self._pwr_band_scale_right = 5.0
        self._pwr_band_scale_left = 5.0
        # Display-only power deglitch guards (do NOT affect controller torque path).
        # Purpose: suppress occasional IMU velocity spikes from creating unrealistic
        # one-frame power bursts on GUI.
        self._pwr_vel_abs_limit_dps = 900.0
        self._pwr_vel_jump_limit_dps = 500.0
        self._pwr_vel_good_L = 0.0
        self._pwr_vel_good_R = 0.0
        self._pwr_glitch_count_L = 0
        self._pwr_glitch_count_R = 0
        self._rl_cfg_tx_seq = 0
        self._rl_last_tx_ts = 0.0
        self._rl_status_ui_last_ts = 0.0
        self._rl_status_ui_min_interval_s = 0.25  # ~4 Hz
        self._replay_status_last_ts = 0.0
        self._replay_status_min_interval_s = 0.12  # ~8 Hz
        self._replay_budget_ms = 5.0
        self._power_overlay_min_interval_s = 0.25  # ~4 Hz
        self._power_overlay_local_min_abs_w = 0.05
        self._power_overlay_left_sig = None
        self._power_overlay_right_sig = None
        self._power_overlay_left_last_ts = 0.0
        self._power_overlay_right_last_ts = 0.0

        # Latest raw/sync/control telemetry snapshot (for CSV + diagnostics)
        self._raw_ang_L = 0.0
        self._raw_ang_R = 0.0
        self._raw_vel_L = 0.0
        self._raw_vel_R = 0.0
        self._raw_cmd_L = 0.0
        self._raw_cmd_R = 0.0
        self._sync_ang_L = 0.0
        self._sync_ang_R = 0.0
        self._sync_vel_L = 0.0
        self._sync_vel_R = 0.0
        self._sync_cmd_L = 0.0
        self._sync_cmd_R = 0.0
        self._phys_pwr_L = 0.0
        self._phys_pwr_R = 0.0
        self._ctrl_pwr_L = 0.0
        self._ctrl_pwr_R = 0.0
        self._sync_sample_id = 0
        self._sync_from_pi = False
        self._telem_ext_valid = False
        self._telem_phys_valid = False
        self._telem_ctrl_valid = False
        self._telem_sync_valid = False

        # RPi status (received via BLE uplink passthrough)
        self._rpi_nn_type = -1          # -1=unknown, 0=dnn, 1=lstm, 2=lstm_leg_dcp, 3=lstm_pd
        # Teensy-native auto delay enable flags (Samsung / EG)
        self._sam_auto_delay_enable = False
        self._eg_auto_delay_enable  = False
        self._rpi_status_version = 0    # 2 = legacy float, 3 = per-leg int16 pairs
        self._rpi_filter_source = 0     # 0=base, 1=runtime_override
        self._rpi_filter_type_code = 0
        self._rpi_filter_order = 2
        self._rpi_enable_mask = 0
        self._rpi_cutoff_hz = 0.0
        self._rpi_scale = 1.0
        # Per-leg delay / metrics (v3). For v2 fallback, both L and R store same value.
        self._rpi_delay_ms_L = 0.0
        self._rpi_delay_ms_R = 0.0
        self._rpi_auto_delay_enable = False
        self._rpi_auto_method_bo = False
        self._rpi_auto_motion_valid_L = False
        self._rpi_auto_motion_valid_R = False
        self._rpi_power_ratio_L = 0.0
        self._rpi_power_ratio_R = 0.0
        self._rpi_pos_per_s_L = 0.0
        self._rpi_pos_per_s_R = 0.0
        self._rpi_neg_per_s_L = 0.0
        self._rpi_neg_per_s_R = 0.0
        self._rpi_best_delay_ms_L = 0.0
        self._rpi_best_delay_ms_R = 0.0
        # Legacy aliases — kept so other code paths referencing the old single-value
        # fields don't break. They now mirror the L/R averages.
        self._rpi_delay_ms = 0.0
        self._rpi_auto_motion_valid = False
        self._rpi_power_ratio = 0.0
        self._rpi_pos_per_s = 0.0
        self._rpi_neg_per_s = 0.0
        self._rpi_best_delay_ms = 0.0
        self._rpi_status_valid = False
        self._rpi_last_rx_time = 0.0      # 上次收到 RPi 状态的时间
        self._rpi_online = False           # RPi 在线标志
        # RPi status display guards: reject obviously corrupted passthrough samples.
        self._rpi_delay_max_ms = 1000.0
        self._rpi_power_abs_max_w = 200.0
        self._rpi_delay_jump_guard_ms = 300.0

        # Remote Pi RL launcher state (GUI local SSH -> Pi tmux)
        self._pi_rl_tmux_session = RPI_RL_TMUX_SESSION
        self._pi_rl_remote_running = False
        self._pi_rl_remote_nn = ""
        self._pi_rl_remote_last_error = ""
        self._pi_rl_remote_last_poll_ts = 0.0
        self._pi_rl_poll_inflight = False
        self._pi_rl_cmd_inflight = False
        self._pi_rl_remote_status_sig = None
        self._async_job_next_id = 1
        self._async_job_result_q = queue.Queue()
        self._async_job_active = {}

        # Connection health
        self._last_rx_time = 0.0
        self._conn_healthy = False
        self._imu34_counter = 0
        self._imu_ok_state_cache = [None] * 6
        self._imu_batt_cache = [None] * 6
        self._imu_batt_style_cache = [None] * 6
        self._rx_buf = bytearray()
        self._last_port_scan = 0.0
        self._port_infos = []
        self._auto_connect_enabled = True
        self._auto_connect_cooldown_s = AUTO_CONNECT_COOLDOWN_S
        self._last_auto_connect_try_ts = 0.0
        self._last_auto_connect_port = ""
        self._last_connected_port = ""
        self._last_auto_connect_err_sig = None

        # Screenshot & recording
        self._capture_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")
        self._recording = False
        self._record_tmp_dir = None
        self._record_frame_idx = 0
        self._record_timer = None
        self._record_start_time = 0.0
        self._record_status_last_ts = 0.0
        self._record_fps = 15
        self._record_ext = "jpg"
        self._record_jpg_quality = 90
        self._record_queue_max = 10
        self._record_queue = None
        self._record_writer_thread = None
        self._record_drop_count = 0
        self._record_saved_count = 0
        self._ffmpeg_path = self._resolve_ffmpeg_path()
        self._replay_mode = False
        self._replay_paused = False
        self._replay_samples = []
        self._replay_idx = 0
        self._replay_play_t = 0.0
        self._replay_last_wall_time = 0.0
        self._replay_speed = 1.0
        self._replay_csv_path = ""
        self._replay_time_axis = []
        self._replay_finished = False
        self._replay_slider_internal = False
        self._replay_slider_dragging = False
        self._replay_mapping_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "mapping.json")
        self._replay_col_mapping = self._load_replay_mapping()

        # Collect updatable themed widgets
        self._cards = []
        self._segmented_ctrls = []
        self._combo_boxes = []

        self._build_layout()
        self._set_pi_rl_remote_state("Pi RL Remote: idle", C.text2, force=True)
        self._reload_pi_profiles_ui()
        self._update_rl_filter_state_label()
        self._update_brand_apply_label()
        self._build_plots()
        self._apply_plot_visibility()
        self._build_value_displays()

        self._maxT_before_off = 15.0
        self.sb_max_torque_cfg.setValue(0.0)
        self._set_power_ui(False)

        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        os.makedirs(log_dir, exist_ok=True)
        filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        self.csv_path = os.path.join(log_dir, filename)
        self._csv_file = open(self.csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            't_s', 'L_angle_deg', 'R_angle_deg',
            'teensy_t_cs_u16', 'teensy_t_s_unwrapped',
            'L_cmd_Nm', 'R_cmd_Nm', 'L_est_Nm', 'R_est_Nm',
            'L_vel_dps', 'R_vel_dps', 'L_pwr_W', 'R_pwr_W',
            'signal_source_mode', 'signal_source_active',
            'power_source_mode', 'power_source_active',
            'raw_L_angle_deg', 'raw_R_angle_deg',
            'raw_L_vel_dps', 'raw_R_vel_dps',
            'raw_L_cmd_Nm', 'raw_R_cmd_Nm',
            'sync_L_angle_deg', 'sync_R_angle_deg',
            'sync_L_vel_dps', 'sync_R_vel_dps',
            'sync_L_cmd_Nm', 'sync_R_cmd_Nm',
            'phys_L_pwr_W', 'phys_R_pwr_W',
            'ctrl_L_pwr_W', 'ctrl_R_pwr_W',
            'sync_sample_id', 'sync_from_pi',
            'gait_freq_Hz',
            't_unwrapped_s', 'wall_time_s', 'wall_elapsed_s',
            'csv_sample_idx', 'csv_session_id',
            'algo_name', 'gui_tag', 'rx_tag_char', 'rx_tag_text',
            'align_event',
            'rl_cfg_tx_seq',
            'rpi_online', 'rpi_status_version', 'rpi_nn_type',
            'rpi_delay_ms_L', 'rpi_delay_ms_R',
            'rpi_power_ratio_L', 'rpi_power_ratio_R',
            'rpi_pos_per_s_L', 'rpi_pos_per_s_R',
            'rpi_neg_per_s_L', 'rpi_neg_per_s_R',
        ])
        self._csv_last_flush = 0.0

        # Wheel guard: scroll wheel only changes spinbox value when it has focus
        for sb in self.findChildren((QDoubleSpinBox, QSpinBox)):
            sb.setFocusPolicy(Qt.StrongFocus)
            sb.installEventFilter(self)

        self._apply_theme()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._update_everything)
        self.timer.start()

    def _init_buffers(self):
        n = self.win_size
        self.t_buffer    = deque([0.0]*n, maxlen=n)
        self.L_IMU_buf   = deque([0.0]*n, maxlen=n)
        self.R_IMU_buf   = deque([0.0]*n, maxlen=n)
        self.L_tau_buf   = deque([0.0]*n, maxlen=n)
        self.R_tau_buf   = deque([0.0]*n, maxlen=n)
        self.L_tau_d_buf = deque([0.0]*n, maxlen=n)
        self.R_tau_d_buf = deque([0.0]*n, maxlen=n)
        self.L_vel_buf   = deque([0.0]*n, maxlen=n)
        self.R_vel_buf   = deque([0.0]*n, maxlen=n)
        self.L_pwr_buf   = deque([0.0]*n, maxlen=n)
        self.R_pwr_buf   = deque([0.0]*n, maxlen=n)

    # ================================================================ UI BUILD
    def _build_layout(self):
        main = QHBoxLayout(self)
        main.setContentsMargins(12, 10, 12, 10)
        main.setSpacing(12)

        # Left panel (scrollable)
        scroll = QScrollArea()
        # Keep widget resizable to avoid blank/zero-size content on some platforms;
        # horizontal/vertical scrollbars remain enabled as-needed.
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        left_widget = QWidget()
        left_widget.setStyleSheet("background: transparent;")
        left_widget.setMinimumWidth(480)
        left = QVBoxLayout(left_widget)
        left.setContentsMargins(0, 0, 8, 0)
        left.setSpacing(8)
        scroll.setWidget(left_widget)
        scroll.setFixedWidth(480)
        main.addWidget(scroll)

        plots = QVBoxLayout()
        plots.setSpacing(12)
        main.addLayout(plots, 1)

        # ---- Connection Card ----
        conn_card = CardFrame()
        self._cards.append(conn_card)
        conn_lay = QHBoxLayout(conn_card)
        conn_lay.setContentsMargins(12, 8, 12, 8)
        conn_lay.setSpacing(8)

        self.conn_dot = ConnectionDot()
        conn_lay.addWidget(self.conn_dot)

        self.cmb_port = QComboBox()
        self.cmb_port.addItems(find_available_ports())
        self.cmb_port.setFixedWidth(140)
        self.cmb_port.setToolTip("Serial port list. While disconnected, GUI auto-connect prefers Adafruit devices.")
        self._setup_combo(self.cmb_port)
        conn_lay.addWidget(self.cmb_port)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedHeight(32)
        self.btn_connect.setCursor(Qt.PointingHandCursor)
        self.btn_connect.setToolTip("Manually connect to the selected serial port.")
        self.btn_connect.clicked.connect(self._connect_clicked)
        conn_lay.addWidget(self.btn_connect)

        conn_lay.addStretch(1)

        # Manual serial port refresh (replaces Eco render toggle).
        self.btn_refresh_ports = QPushButton("Refresh")
        self.btn_refresh_ports.setFixedHeight(28)
        self.btn_refresh_ports.setCursor(Qt.PointingHandCursor)
        self.btn_refresh_ports.setToolTip("Refresh serial port list")
        self.btn_refresh_ports.clicked.connect(self._on_refresh_ports_clicked)
        conn_lay.addWidget(self.btn_refresh_ports)

        # Theme toggle
        self.lbl_theme_icon = QLabel("Dark")
        self.lbl_theme_icon.setStyleSheet(f"color:{C.text2}; font-size:12px; background:transparent;")
        conn_lay.addWidget(self.lbl_theme_icon)
        self.sw_theme = ToggleSwitch(checked=True, on_color='#5856d6')
        self.sw_theme.toggled.connect(self._on_theme_toggled)
        conn_lay.addWidget(self.sw_theme)

        left.addWidget(conn_card)

        # ---- Power Button ----
        self.btn_power = QPushButton("POWER OFF")
        self.btn_power.setCheckable(True)
        self.btn_power.setChecked(False)
        self.btn_power.setMinimumHeight(56)
        self.btn_power.setCursor(Qt.PointingHandCursor)
        self.btn_power.toggled.connect(self._on_power_toggled)
        left.addWidget(self.btn_power)

        # ---- Algorithm Card ----
        algo_card = CardFrame()
        self._cards.append(algo_card)
        algo_lay = QVBoxLayout(algo_card)
        algo_lay.setContentsMargins(12, 10, 12, 10)
        algo_lay.setSpacing(6)

        algo_header = QHBoxLayout()
        algo_header.addWidget(_make_section_label("ALGORITHM"))
        self.badge_algo = PillBadge("EG", C.green)
        algo_header.addWidget(self.badge_algo)
        algo_header.addStretch(1)
        algo_lay.addLayout(algo_header)

        self.seg_algo = SegmentedControl(["EG", "Samsung", "RL", "SOGI", "Test"])
        self.seg_algo.currentIndexChanged.connect(self._on_algo_selected)
        self._segmented_ctrls.append(self.seg_algo)
        algo_lay.addWidget(self.seg_algo)

        self.btn_algo_confirm = QPushButton("Apply Algorithm")
        self.btn_algo_confirm.setCursor(Qt.PointingHandCursor)
        self.btn_algo_confirm.setFixedHeight(36)
        self.btn_algo_confirm.clicked.connect(self._on_algo_confirm)
        algo_lay.addWidget(self.btn_algo_confirm)

        left.addWidget(algo_card)

        # ---- Parameters Card ----
        param_card = CardFrame()
        self._cards.append(param_card)
        param_lay = QVBoxLayout(param_card)
        param_lay.setContentsMargins(12, 10, 12, 10)
        param_lay.setSpacing(6)

        param_lay.addWidget(_make_section_label("PARAMETERS"))

        max_torque_tip = (
            "Maximum output torque clamp (Nm).\n"
            "This is the top-level safety limit sent to controller."
        )

        # Max Torque row
        mt_row = QHBoxLayout()
        lbl_max_torque = QLabel("Max Torque (Nm)")
        lbl_max_torque.setToolTip(max_torque_tip)
        mt_row.addWidget(lbl_max_torque)
        mt_row.addStretch(1)

        def make_dspin(val=0.0, mn=-20, mx=20, step=0.01, dec=2, tip=""):
            sb = QDoubleSpinBox()
            sb.setDecimals(dec); sb.setSingleStep(step)
            sb.setRange(mn, mx); sb.setValue(val)
            sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params)
            return sb

        self.sb_max_torque_cfg = make_dspin(15.0, 0.0, 30.0, 0.1, 1, max_torque_tip)
        self.sb_max_torque_cfg.setFixedWidth(90)
        mt_row.addWidget(self.sb_max_torque_cfg)
        param_lay.addLayout(mt_row)

        torque_filter_tip = (
            "Teensy unified filter before motor torque command (non-RL algorithms).\n"
            "Type is fixed to Butterworth IIR (2nd order).\n"
            "RL mode keeps Pi->Teensy torque path transparent (no Teensy pre-motor filtering)."
        )
        tf_row = QHBoxLayout()
        lbl_tf_fc = QLabel("Filter Before Torque (Hz)")
        lbl_tf_fc.setToolTip(torque_filter_tip)
        tf_row.addWidget(lbl_tf_fc)
        tf_row.addStretch(1)
        self.sb_torque_filter_fc = make_dspin(5.0, 0.3, 20.0, 0.1, 1, torque_filter_tip)
        self.sb_torque_filter_fc.setFixedWidth(90)
        tf_row.addWidget(self.sb_torque_filter_fc)
        param_lay.addLayout(tf_row)

        # Separator
        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"background-color: {C.separator}; max-height:1px; border:none;")
        param_lay.addWidget(sep)

        # Algorithm parameter stack
        self.algo_stack = QStackedWidget()
        param_lay.addWidget(self.algo_stack)

        # -- EG panel --
        eg_panel = QWidget()
        eg_panel.setStyleSheet("background:transparent;")
        eg_grid = QGridLayout(eg_panel)
        eg_grid.setSpacing(4)
        eg_tip_r_gain = "Right-leg assist gain multiplier."
        eg_tip_l_gain = "Left-leg assist gain multiplier."
        eg_tip_gate_k = "Gate sharpness. Higher value means steeper on/off transition."
        eg_tip_gate_on = "Gate-on threshold for positive-power region."
        eg_tip_scale_all = "Global EG torque scale applied to both sides."
        eg_tip_ext_frac_l = "Left extension phase fraction (0.0~0.5 of gait cycle)."
        eg_tip_ext_frac_r = "Right extension phase fraction (0.0~0.5 of gait cycle)."
        eg_tip_ext_gain = "Extension channel gain."
        eg_tip_delay_idx = (
            "Assist_delay_gain phase index (0..99), auto-scaled by gait frequency "
            "(not fixed ms)."
        )
        self.sb_Flex_Assist_gain  = make_dspin(1.0, -2, 2, 0.01, 2, eg_tip_r_gain)
        self.sb_Ext_Assist_gain   = make_dspin(1.0, -2, 2, 0.01, 2, eg_tip_l_gain)
        self.sb_gate_k            = make_dspin(1.0, 0, 10, 0.1, 2, eg_tip_gate_k)
        self.sb_gate_p_on         = make_dspin(8, 0, 50, 1, 2, eg_tip_gate_on)
        self.sb_scale_all         = make_dspin(0.20, -1.0, 1.0, 0.01, 2, eg_tip_scale_all)
        self.sb_ext_phase_frac_L  = make_dspin(0.300, 0.0, 0.500, 0.001, 3, eg_tip_ext_frac_l)
        self.sb_ext_phase_frac_R  = make_dspin(0.300, 0.0, 0.500, 0.001, 3, eg_tip_ext_frac_r)
        self.sb_ext_gain          = make_dspin(0.50, -3.0, 3.0, 0.01, 2, eg_tip_ext_gain)
        self.sb_Assist_delay_gain = QSpinBox()
        self.sb_Assist_delay_gain.setRange(0, 99); self.sb_Assist_delay_gain.setValue(40)
        self.sb_Assist_delay_gain.setToolTip(eg_tip_delay_idx)
        self.sb_Assist_delay_gain.valueChanged.connect(self._tx_params)

        eg_labels = ["R Gain", "L Gain", "gate_k", "gate_p_on",
                     "scale_all", "ext_frac_L", "ext_frac_R", "ext_gain", "Delay idx(phase)"]
        eg_spins  = [self.sb_Flex_Assist_gain, self.sb_Ext_Assist_gain,
                     self.sb_gate_k, self.sb_gate_p_on, self.sb_scale_all,
                     self.sb_ext_phase_frac_L, self.sb_ext_phase_frac_R,
                     self.sb_ext_gain, self.sb_Assist_delay_gain]
        eg_tips = [eg_tip_r_gain, eg_tip_l_gain, eg_tip_gate_k, eg_tip_gate_on,
                   eg_tip_scale_all, eg_tip_ext_frac_l, eg_tip_ext_frac_r,
                   eg_tip_ext_gain, eg_tip_delay_idx]
        for i, (lb, sb, tip) in enumerate(zip(eg_labels, eg_spins, eg_tips)):
            r, c = divmod(i, 3)
            lbl = QLabel(lb)
            lbl.setStyleSheet(f"color:{C.text2}; font-size:11px; background:transparent;")
            lbl.setToolTip(tip)
            eg_grid.addWidget(lbl, r*2, c)
            eg_grid.addWidget(sb, r*2+1, c)
        # Post-delay row (below the 3-column grid; 9 items → rows 0-5, so start at row 6)
        _eg_next_row = (((len(eg_labels) - 1) // 3) + 1) * 2
        self.sb_eg_post_delay = make_dspin(0, 0, 1500, 10, 0)
        self.sb_eg_post_delay.setToolTip(
            "EG output post-delay (ms). Applied after internal Assist_delay_gain. "
            "Auto Delay ON: optimized by Teensy-local AutoDelayOptimizer (L/R independent).")
        self.chk_eg_auto_delay = QCheckBox("Auto Delay")
        self.chk_eg_auto_delay.setChecked(False)
        self.chk_eg_auto_delay.setToolTip(
            "Teensy-local auto delay: optimizes post_delay_ms to maximize positive power ratio (L/R independent)")
        self.chk_eg_auto_delay.stateChanged.connect(self._on_eg_auto_delay_toggled)
        self.btn_eg_reset = QPushButton("Reset")
        self.btn_eg_reset.setFixedWidth(52)
        self.btn_eg_reset.setEnabled(False)
        self.btn_eg_reset.setToolTip("Reset Auto Delay: restart optimization from base post_delay_ms")
        self.btn_eg_reset.clicked.connect(self._on_eg_reset_clicked)
        self.lbl_eg_auto_delay_state = QLabel("L=-- idx  R=-- idx")
        self.lbl_eg_auto_delay_state.setStyleSheet(
            f"color:{C.purple}; font-size:11px; background:transparent;")
        _lbl_pd = QLabel("Post Delay (ms)")
        _lbl_pd.setStyleSheet(f"color:{C.text2}; font-size:11px; background:transparent;")
        _lbl_pd.setToolTip(self.sb_eg_post_delay.toolTip())
        eg_grid.addWidget(_lbl_pd, _eg_next_row, 0, 1, 2)
        eg_grid.addWidget(self.sb_eg_post_delay, _eg_next_row, 2)
        eg_grid.addWidget(self.chk_eg_auto_delay, _eg_next_row + 1, 0)
        eg_grid.addWidget(self.btn_eg_reset, _eg_next_row + 1, 1)
        eg_grid.addWidget(self.lbl_eg_auto_delay_state, _eg_next_row + 2, 0, 1, 3)
        self.algo_stack.addWidget(eg_panel)

        # -- Samsung panel --
        sam_panel = QWidget()
        sam_panel.setStyleSheet("background:transparent;")
        sam_grid = QGridLayout(sam_panel)
        sam_grid.setSpacing(4)
        sam_kappa_tip = (
            "Samsung controller gain kappa.\n"
            "Higher value generally increases assist magnitude."
        )
        sam_delay_tip = (
            "Samsung torque delay (ms).\n"
            "Auto Delay ON: optimized by Teensy-local AutoDelayOptimizer (L/R independent)."
        )
        self.sb_sam_kappa = make_dspin(3.0, 0, 20, 0.1, 1, sam_kappa_tip)
        self.sb_sam_delay = make_dspin(250, 0, 1500, 10, 0, sam_delay_tip)
        self.chk_sam_auto_delay = QCheckBox("Auto Delay")
        self.chk_sam_auto_delay.setChecked(False)
        self.chk_sam_auto_delay.setToolTip(
            "Teensy-local auto delay: optimizes delay_ms to maximize positive power ratio (L/R independent)")
        self.chk_sam_auto_delay.stateChanged.connect(self._on_sam_auto_delay_toggled)
        self.btn_sam_reset = QPushButton("Reset")
        self.btn_sam_reset.setFixedWidth(52)
        self.btn_sam_reset.setEnabled(False)
        self.btn_sam_reset.setToolTip("Reset Auto Delay: restart optimization from base delay_ms")
        self.btn_sam_reset.clicked.connect(self._on_sam_reset_clicked)
        self.lbl_sam_auto_delay_state = QLabel("L=-- ms  R=-- ms")
        self.lbl_sam_auto_delay_state.setStyleSheet(
            f"color:{C.purple}; font-size:11px; background:transparent;")
        lbl_sam_kappa = QLabel("Kappa")
        lbl_sam_kappa.setToolTip(sam_kappa_tip)
        sam_grid.addWidget(lbl_sam_kappa, 0, 0)
        sam_grid.addWidget(self.sb_sam_kappa, 0, 1)
        lbl_sam_delay = QLabel("Delay (ms)")
        lbl_sam_delay.setToolTip(sam_delay_tip)
        sam_grid.addWidget(lbl_sam_delay, 1, 0)
        sam_grid.addWidget(self.sb_sam_delay, 1, 1)
        sam_grid.addWidget(self.chk_sam_auto_delay, 2, 0)
        sam_grid.addWidget(self.btn_sam_reset, 2, 1)
        sam_grid.addWidget(self.lbl_sam_auto_delay_state, 3, 0, 1, 2)
        self.algo_stack.addWidget(sam_panel)

        # -- RL panel --
        rl_panel = QWidget()
        rl_panel.setStyleSheet("background:transparent;")
        rl_lay = QGridLayout(rl_panel)
        rl_lay.setSpacing(6)

        # Row 0: RPi NN type + connection status
        self.lbl_rpi_nn_type = QLabel("RPi: waiting...")
        self.lbl_rpi_nn_type.setStyleSheet(
            f"color:{C.orange}; font-size:13px; font-weight:600; background:transparent;")
        rl_lay.addWidget(self.lbl_rpi_nn_type, 0, 0, 1, 2)

        # Row 1: RPi current filter state (read-only, from RPi uplink)
        self.lbl_rpi_current_filter = QLabel("Current: -")
        self.lbl_rpi_current_filter.setWordWrap(True)
        self.lbl_rpi_current_filter.setStyleSheet(
            f"color:{C.teal}; font-size:12px; background:transparent;")
        rl_lay.addWidget(self.lbl_rpi_current_filter, 1, 0, 1, 2)

        # Row 2: Scale (staged, not auto-sent)
        self.sb_rl_scale = make_dspin(1.00, 0.00, 3.00, 0.01, 2,
                                      "RL torque scale (L/R same)")
        self.sb_rl_scale.valueChanged.disconnect(self._tx_params)
        lbl_rl_scale = QLabel("RL Scale (L/R)")
        lbl_rl_scale.setToolTip(self.sb_rl_scale.toolTip())
        rl_lay.addWidget(lbl_rl_scale, 2, 0)
        rl_lay.addWidget(self.sb_rl_scale, 2, 1)

        # Row 3: Delay (step=10ms, staged)
        self.sb_rl_torque_delay = make_dspin(0.0, 0.0, 1000.0, 10.0, 0,
                                             "RL torque delay absolute (ms), 0~1000")
        self.sb_rl_torque_delay.valueChanged.disconnect(self._tx_params)
        self.lbl_rl_torque_delay_auto = QLabel("L=--.- ms | R=--.- ms")
        self.lbl_rl_torque_delay_auto.setStyleSheet(
            f"color:{C.text2}; font-size:13px; background:transparent;"
        )
        self.lbl_rl_torque_delay_auto.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.rl_delay_stack = QStackedWidget()
        self.rl_delay_stack.addWidget(self.sb_rl_torque_delay)      # index 0: manual input
        self.rl_delay_stack.addWidget(self.lbl_rl_torque_delay_auto)  # index 1: auto display
        lbl_rl_delay = QLabel("Torque Delay (ms)")
        lbl_rl_delay.setToolTip(self.sb_rl_torque_delay.toolTip())
        rl_lay.addWidget(lbl_rl_delay, 3, 0)
        rl_lay.addWidget(self.rl_delay_stack, 3, 1)

        # Row 4: Filter Type
        self.cmb_rl_filter_type = QComboBox()
        self.cmb_rl_filter_type.addItems([x[0] for x in RL_FILTER_TYPES])
        self._setup_combo(self.cmb_rl_filter_type)
        self.cmb_rl_filter_type.setToolTip("RL runtime filter type on RPi.")
        self.lbl_rl_filter_type = QLabel("Filter Type")
        self.lbl_rl_filter_type.setToolTip(self.cmb_rl_filter_type.toolTip())
        rl_lay.addWidget(self.lbl_rl_filter_type, 4, 0)
        rl_lay.addWidget(self.cmb_rl_filter_type, 4, 1)

        # Row 5: Filter Cutoff (default 5Hz, staged)
        self.sb_rl_cutoff_hz = make_dspin(5.0, 0.5, 30.0, 0.5, 1,
                                          "RL runtime filter cutoff (Hz)")
        self.sb_rl_cutoff_hz.valueChanged.disconnect(self._tx_params)
        self.lbl_rl_cutoff = QLabel("Filter Cutoff (Hz)")
        self.lbl_rl_cutoff.setToolTip(self.sb_rl_cutoff_hz.toolTip())
        rl_lay.addWidget(self.lbl_rl_cutoff, 5, 0)
        rl_lay.addWidget(self.sb_rl_cutoff_hz, 5, 1)

        # Row 6: Filter Enable checkboxes (DNN: Vel+Ref + Torque; LSTM: only Torque)
        filt_row = QHBoxLayout()
        self.chk_rl_vr_filter = QCheckBox("Vel+Ref")
        self.chk_rl_torque_filter = QCheckBox("Torque")
        self.chk_rl_auto_delay = QCheckBox("Auto Delay")
        self.chk_rl_vr_filter.setToolTip("Enable RPi filter for velocity/reference channels.")
        self.chk_rl_torque_filter.setToolTip("Enable RPi filter on torque output channel.")
        self.chk_rl_auto_delay.setToolTip("Enable RPi auto delay optimization.")
        self.chk_rl_vr_filter.setChecked(True)
        self.chk_rl_torque_filter.setChecked(True)
        self.chk_rl_auto_delay.setChecked(False)
        self.chk_rl_vr_filter.stateChanged.connect(self._update_rl_filter_state_label)
        self.chk_rl_torque_filter.stateChanged.connect(self._update_rl_filter_state_label)
        self.chk_rl_auto_delay.stateChanged.connect(self._on_rl_auto_delay_toggled)
        filt_row.addWidget(self.chk_rl_vr_filter)
        filt_row.addWidget(self.chk_rl_torque_filter)
        filt_row.addWidget(self.chk_rl_auto_delay)
        filt_row.addStretch(1)
        self.lbl_rl_filter_en = QLabel("Filter Enable")
        self.lbl_rl_filter_en.setToolTip("Runtime filter and auto-delay toggles sent to RPi.")
        rl_lay.addWidget(self.lbl_rl_filter_en, 6, 0)
        rl_lay.addLayout(filt_row, 6, 1)

        # Row 7: Auto-delay optimizer method
        self.cmb_rl_auto_method = QComboBox()
        self.cmb_rl_auto_method.addItems(["Grid (Legacy)", "Bayes (BO)"])
        self._setup_combo(self.cmb_rl_auto_method)
        self.cmb_rl_auto_method.setCurrentIndex(0)
        self.cmb_rl_auto_method.setToolTip(
            "Auto Delay optimizer on RPi: Grid(local scan) or Bayes(1D GP BO)."
        )
        self.cmb_rl_auto_method.currentIndexChanged.connect(self._update_rl_filter_state_label)
        lbl_rl_auto_method = QLabel("Auto Method")
        lbl_rl_auto_method.setToolTip(self.cmb_rl_auto_method.toolTip())
        rl_lay.addWidget(lbl_rl_auto_method, 7, 0)
        rl_lay.addWidget(self.cmb_rl_auto_method, 7, 1)

        # Row 8: Apply RL button (staged send, not realtime)
        self.btn_apply_rl = QPushButton("Apply RL Settings")
        self.btn_apply_rl.setStyleSheet(
            f"background-color:{C.blue}; color:white; font-weight:600; "
            f"border-radius:8px; padding:8px 16px; font-size:13px;")
        self.btn_apply_rl.clicked.connect(self._on_apply_rl_clicked)
        rl_lay.addWidget(self.btn_apply_rl, 8, 0, 1, 2)

        # Row 9: Pi profile selector + config entry
        profile_row = QHBoxLayout()
        self.cmb_pi_profile = QComboBox()
        self._setup_combo(self.cmb_pi_profile)
        self.cmb_pi_profile.setToolTip("Active Pi SSH profile for RL remote start/stop.")
        self.cmb_pi_profile.currentIndexChanged.connect(self._on_pi_profile_changed)
        self.btn_pi_profile_cfg = QPushButton("Configure Pi...")
        self.btn_pi_profile_cfg.setToolTip("Add/edit Pi SSH profiles stored in user config directory.")
        self.btn_pi_profile_cfg.clicked.connect(self._on_configure_pi_clicked)
        profile_row.addWidget(self.cmb_pi_profile, 1)
        profile_row.addWidget(self.btn_pi_profile_cfg, 0)
        lbl_pi_profile = QLabel("Pi Profile")
        lbl_pi_profile.setToolTip("Choose which Pi SSH profile is used for Start/Stop Pi RL.")
        rl_lay.addWidget(lbl_pi_profile, 9, 0)
        rl_lay.addLayout(profile_row, 9, 1)

        # Row 10: Pi remote launcher controls (local SSH, not via Teensy)
        rl_remote_row = QHBoxLayout()
        self.btn_pi_rl_start_legdcp = QPushButton("Start LegDcp")
        self.btn_pi_rl_start_legdcp.setToolTip(
            "Start Pi RL in tmux session using nn=lstm_leg_dcp."
        )
        self.btn_pi_rl_start_legdcp.clicked.connect(
            lambda: self._on_pi_rl_start_clicked("lstm_leg_dcp")
        )
        self.btn_pi_rl_start_pd = QPushButton("Start LSTM-PD")
        self.btn_pi_rl_start_pd.setToolTip(
            "Start Pi RL in tmux session using nn=lstm_pd."
        )
        self.btn_pi_rl_start_pd.clicked.connect(
            lambda: self._on_pi_rl_start_clicked("lstm_pd")
        )
        self.btn_pi_rl_stop = QPushButton("Stop Pi RL")
        self.btn_pi_rl_stop.setToolTip("Stop Pi RL tmux session on Raspberry Pi.")
        self.btn_pi_rl_stop.clicked.connect(self._on_pi_rl_stop_clicked)
        rl_remote_row.addWidget(self.btn_pi_rl_start_legdcp)
        rl_remote_row.addWidget(self.btn_pi_rl_start_pd)
        rl_remote_row.addWidget(self.btn_pi_rl_stop)
        rl_remote_row.addStretch(1)
        lbl_pi_remote = QLabel("Pi RL Remote")
        lbl_pi_remote.setToolTip("Remote launcher controls for starting/stopping RL on Raspberry Pi via SSH.")
        rl_lay.addWidget(lbl_pi_remote, 10, 0)
        rl_lay.addLayout(rl_remote_row, 10, 1)

        # Row 11: Pi remote launcher status
        self.lbl_pi_rl_remote_state = QLabel("Pi RL Remote: idle")
        self.lbl_pi_rl_remote_state.setWordWrap(True)
        self.lbl_pi_rl_remote_state.setToolTip(
            "Shows remote RL launcher state, including running NN type and SSH/tmux errors."
        )
        self.lbl_pi_rl_remote_state.setStyleSheet(
            f"color:{C.text2}; font-size:11px; background:transparent; padding-top:1px;"
        )
        rl_lay.addWidget(self.lbl_pi_rl_remote_state, 11, 0, 1, 2)

        # Row 12: Status label
        self.lbl_rl_filter_state = QLabel("")
        self.lbl_rl_filter_state.setWordWrap(True)
        self.lbl_rl_filter_state.setStyleSheet(
            f"color:{C.text2}; font-size:11px; background:transparent; padding-top:2px;"
        )
        rl_lay.addWidget(self.lbl_rl_filter_state, 12, 0, 1, 2)

        # Row 13: Auto-delay telemetry (from RPi status uplink)
        self.lbl_rl_auto_state = QLabel("Auto Delay: waiting for RPi status...")
        self.lbl_rl_auto_state.setWordWrap(True)
        self.lbl_rl_auto_state.setStyleSheet(
            f"color:{C.purple}; font-size:11px; background:transparent; padding-top:1px;"
        )
        rl_lay.addWidget(self.lbl_rl_auto_state, 13, 0, 1, 2)
        self.algo_stack.addWidget(rl_panel)

        # -- SOGI panel (phase-locked sinusoidal assist) --
        sogi_panel = QWidget()
        sogi_panel.setStyleSheet("background:transparent;")
        sogi_grid = QGridLayout(sogi_panel)
        sogi_grid.setSpacing(4)
        sogi_a_tip = (
            "SOGI torque amplitude A_gain (Nm).\n"
            "Peak of tau = A * sin(phi + lead). Tune per subject."
        )
        sogi_lead_tip = (
            "Phase lead (deg) applied to the SOGI output.\n"
            "Compensates control-chain delay. Typical range 0~40°."
        )
        sogi_ampmin_tip = (
            "Amplitude watchdog threshold (deg/s).\n"
            "When instantaneous amp falls below this, torque is gated off (standing/stop)."
        )
        self.sb_sogi_A       = make_dspin(5.0, 0.0, 15.0, 0.1, 1, sogi_a_tip)
        self.sb_sogi_lead    = make_dspin(20.0, -90.0, 90.0, 1.0, 1, sogi_lead_tip)
        self.sb_sogi_amp_min = make_dspin(20.0, 0.0, 500.0, 1.0, 1, sogi_ampmin_tip)
        lbl_sogi_A = QLabel("A_gain (Nm)");       lbl_sogi_A.setToolTip(sogi_a_tip)
        lbl_sogi_lead = QLabel("Phi lead (°)");   lbl_sogi_lead.setToolTip(sogi_lead_tip)
        lbl_sogi_amp = QLabel("amp_min (deg/s)"); lbl_sogi_amp.setToolTip(sogi_ampmin_tip)
        sogi_grid.addWidget(lbl_sogi_A,          0, 0)
        sogi_grid.addWidget(self.sb_sogi_A,      0, 1)
        sogi_grid.addWidget(lbl_sogi_lead,       1, 0)
        sogi_grid.addWidget(self.sb_sogi_lead,   1, 1)
        sogi_grid.addWidget(lbl_sogi_amp,        2, 0)
        sogi_grid.addWidget(self.sb_sogi_amp_min,2, 1)
        self.algo_stack.addWidget(sogi_panel)

        # -- Test panel (sin wave) --
        test_panel = QWidget()
        test_panel.setStyleSheet("background:transparent;")
        test_grid = QGridLayout(test_panel)
        test_grid.setSpacing(4)
        test_grid.addWidget(QLabel("Waveform"), 0, 0)
        self.cmb_test_waveform = QComboBox()
        self.cmb_test_waveform.addItems(["Constant", "Sin Wave"])
        self._setup_combo(self.cmb_test_waveform)
        self.cmb_test_waveform.currentIndexChanged.connect(self._on_test_waveform_changed)
        test_grid.addWidget(self.cmb_test_waveform, 0, 1)

        self.sb_test_amplitude = make_dspin(0.0, -15.0, 15.0, 0.1, 1)
        test_grid.addWidget(QLabel("Amplitude (Nm)"), 1, 0)
        test_grid.addWidget(self.sb_test_amplitude, 1, 1)

        self.lbl_test_freq = QLabel("Frequency (Hz)")
        self.sb_test_freq = make_dspin(1.0, 0.1, 10.0, 0.1, 1)
        test_grid.addWidget(self.lbl_test_freq, 2, 0)
        test_grid.addWidget(self.sb_test_freq, 2, 1)
        self.lbl_test_freq.setVisible(False)
        self.sb_test_freq.setVisible(False)
        self.algo_stack.addWidget(test_panel)

        left.addWidget(param_card)

        # Hidden EG defaults
        self.Rescaling_gain_def = 0.00
        self.offL_def = 0
        self.offR_def = 0
        self.lead_frac_def = 0.060

        # ---- LogTag Card ----
        log_card = CardFrame()
        self._cards.append(log_card)
        log_lay = QVBoxLayout(log_card)
        log_lay.setContentsMargins(12, 10, 12, 10)
        log_lay.setSpacing(6)

        log_lay.addWidget(_make_section_label("LOG TAG"))
        tag_row = QHBoxLayout()
        self.edt_label = QLineEdit()
        self.edt_label.setPlaceholderText("Enter label (max 10 chars)")
        self.edt_label.setMaxLength(10)
        self.edt_label.returnPressed.connect(self._send_logtag)
        tag_row.addWidget(self.edt_label, 1)

        self.sw_persist = ToggleSwitch(checked=False, on_color=C.blue)
        tag_row.addWidget(QLabel("Persist"))
        tag_row.addWidget(self.sw_persist)
        log_lay.addLayout(tag_row)

        send_row = QHBoxLayout()
        btn_send = QPushButton("Send Label")
        btn_send.setCursor(Qt.PointingHandCursor)
        btn_send.clicked.connect(self._send_logtag)
        btn_send.setFixedHeight(32)
        send_row.addWidget(btn_send)

        self.btn_auto_mile = QPushButton("Auto Mile")
        self.btn_auto_mile.setCursor(Qt.PointingHandCursor)
        self.btn_auto_mile.setFixedHeight(32)
        self.btn_auto_mile.clicked.connect(self._auto_cycle_mile)
        self.btn_auto_mile.setToolTip("Cycle preset labels and send as persistent log tags.")
        send_row.addWidget(self.btn_auto_mile)
        self._mile_values = ['sit-to-stand', 'walk', 'run', 'squat']
        self._mile_index = 0

        self.btn_align_mark = QPushButton("Align Mark")
        self.btn_align_mark.setCursor(Qt.PointingHandCursor)
        self.btn_align_mark.setFixedHeight(32)
        self.btn_align_mark.clicked.connect(self._on_align_mark_clicked)
        self.btn_align_mark.setToolTip(
            "Insert a manual alignment marker into GUI CSV (align_event) and send a short tag to Pi.")
        send_row.addWidget(self.btn_align_mark)

        log_lay.addLayout(send_row)
        left.addWidget(log_card)

        # ---- Plot Controls Card ----
        plot_card = CardFrame()
        self._cards.append(plot_card)
        plot_card_vlay = QVBoxLayout(plot_card)
        plot_card_vlay.setContentsMargins(12, 6, 12, 6)
        plot_card_vlay.setSpacing(4)

        # --- Row 1: Trace toggles + Buffer controls ---
        row1 = QHBoxLayout(); row1.setSpacing(6)

        trace_defs = [
            ("Angle", "#34c759"),
            ("Cmd",   "#007aff"),
            ("Est",   "#ff3b30"),
            ("Vel",   "#5ac8fa"),
            ("Pwr",   "#af52de"),
        ]
        self.chk_plot_angle = None
        self.chk_plot_cmd   = None
        self.chk_plot_est   = None
        self.chk_plot_vel   = None
        self.chk_plot_pwr   = None
        chk_refs = []
        for label, color in trace_defs:
            dot = QLabel()
            dot.setFixedSize(8, 8)
            dot.setStyleSheet(f"background:{color}; border-radius:4px; border:none;")
            row1.addWidget(dot)
            chk = QCheckBox(label)
            chk.setChecked(True)
            chk.setStyleSheet(f"QCheckBox {{ color: {color}; font-weight:600; font-size:12px; }}"
                              f"QCheckBox::indicator:checked {{ background-color:{color}; border-color:{color}; }}")
            chk.stateChanged.connect(self._apply_plot_visibility)
            row1.addWidget(chk)
            chk_refs.append(chk)
        self.chk_plot_angle, self.chk_plot_cmd, self.chk_plot_est, self.chk_plot_vel, self.chk_plot_pwr = chk_refs

        raw_data_note_tip = (
            "显示数据来源说明（当前 GUI 曲线）:\n"
            "1) Angle 曲线: 来自 Teensy BLE 上行 payload[5..8]，即 imu.LTx / imu.RTx。\n"
            "   imu.LTx/RTx = IMU Euler-X(AngleXLeft/Right) - 零偏(offL/offR)。\n"
            "2) Vel 曲线: 优先来自 payload[40..43]，即 imu.LTAVx / imu.RTAVx（deg/s）。\n"
            "3) 若 payload[40..51] 全 0，GUI 才回退到 d(angle)/dt 差分速度。\n"
            "4) GUI 不对 Angle/Vel 做 LPF/IIR/Butterworth 滤波。\n"
            "5) GUI 不再本地计算功率；Pwr 仅来自 Teensy/Pi 上报值。\n"
            "6) 这里“原始数据”指 IMU 模块输出值（可包含模块自身配置滤波），\n"
            "   不是 Teensy 的 LTx_filtered/RTx_filtered 控制链路。"
        )
        self.note_raw_data = QFrame()
        self.note_raw_data.setFixedSize(16, 16)
        self.note_raw_data.setToolTip(raw_data_note_tip)
        self.note_raw_data.setStyleSheet(
            f"background:{C.fill}; border:1px solid {C.teal}; border-radius:4px;")
        row1.addWidget(self.note_raw_data)

        vsep1 = QFrame(); vsep1.setFrameShape(QFrame.VLine)
        vsep1.setStyleSheet(f"background-color:{C.separator}; max-width:1px; border:none;")
        vsep1.setFixedHeight(20)
        row1.addWidget(vsep1)

        self.btn_clear_buf = QPushButton("Clear")
        self.btn_clear_buf.setCursor(Qt.PointingHandCursor)
        self.btn_clear_buf.setFixedHeight(24)
        self.btn_clear_buf.setToolTip("Clear plotting buffers.")
        self.btn_clear_buf.clicked.connect(self._clear_buffers)
        row1.addWidget(self.btn_clear_buf)

        lbl_w = QLabel("Win:")
        lbl_w.setStyleSheet("font-size:12px; background:transparent;")
        lbl_w.setToolTip("Plot window length (number of samples).")
        row1.addWidget(lbl_w)
        self.sb_win_size = QSpinBox()
        self.sb_win_size.setRange(50, 2000)
        self.sb_win_size.setValue(self.win_size)
        self.sb_win_size.setFixedWidth(70)
        self.sb_win_size.setFixedHeight(24)
        self.sb_win_size.setToolTip("Plot window length (number of samples).")
        self.sb_win_size.valueChanged.connect(self._on_win_size_changed)
        row1.addWidget(self.sb_win_size)

        lbl_a = QLabel("Auto")
        lbl_a.setStyleSheet("font-size:12px; background:transparent;")
        lbl_a.setToolTip("Auto-scroll x-axis with incoming data.")
        row1.addWidget(lbl_a)
        self.sw_auto_scroll = ToggleSwitch(checked=True, on_color=C.blue)
        self.sw_auto_scroll.setToolTip("Auto-scroll x-axis with incoming data.")
        row1.addWidget(self.sw_auto_scroll)
        row1.addStretch(1)

        plot_card_vlay.addLayout(row1)

        # --- Row 2: Direction controls + Capture buttons ---
        row2 = QHBoxLayout(); row2.setSpacing(6)

        def _dir_btn(text, tip):
            b = QPushButton(text)
            b.setCursor(Qt.PointingHandCursor)
            b.setFixedSize(38, 24)
            b.setStyleSheet("font-size:11px; padding:2px 6px;")
            b.setToolTip(tip)
            return b

        lbl_md = QLabel("Motor:")
        lbl_md.setStyleSheet("font-size:12px; background:transparent;")
        row2.addWidget(lbl_md)
        self.btn_toggle_L = _dir_btn("L+", "Real L direction (actuator only, plot sign unchanged)")
        self.btn_toggle_R = _dir_btn("R+", "Real R direction (actuator only, plot sign unchanged)")
        self.btn_toggle_L.clicked.connect(self._toggle_left_dir)
        self.btn_toggle_R.clicked.connect(self._toggle_right_dir)
        row2.addWidget(self.btn_toggle_L)
        row2.addWidget(self.btn_toggle_R)

        lbl_vis = QLabel("Visual:")
        lbl_vis.setStyleSheet("font-size:12px; background:transparent;")
        row2.addWidget(lbl_vis)
        self.btn_visual_L = _dir_btn("VL+", "Visual L torque sign (plot only)")
        self.btn_visual_R = _dir_btn("VR+", "Visual R torque sign (plot only)")
        self.btn_visual_L.clicked.connect(self._toggle_visual_L)
        self.btn_visual_R.clicked.connect(self._toggle_visual_R)
        row2.addWidget(self.btn_visual_L)
        row2.addWidget(self.btn_visual_R)

        self.badge_original = PillBadge("Original", C.green)
        row2.addWidget(self.badge_original)

        lbl_data_src = QLabel("Data:")
        lbl_data_src.setStyleSheet("font-size:12px; background:transparent;")
        lbl_data_src.setToolTip("Select plotted signal source (Raw / Sync / Auto).")
        row2.addWidget(lbl_data_src)
        self.cmb_signal_source = QComboBox()
        self.cmb_signal_source.addItems(["Auto", "Raw", "Sync"])
        self.cmb_signal_source.setCurrentText("Auto")
        self.cmb_signal_source.setFixedWidth(76)
        self.cmb_signal_source.setFixedHeight(24)
        self.cmb_signal_source.setToolTip(
            "Plot signal source.\n"
            "Auto: RL+Pi uses control-synced input; otherwise Raw IMU.\n"
            "Raw: direct Teensy IMU stream.\n"
            "Sync: control-aligned input stream (Pi sync if available)."
        )
        self._setup_combo(self.cmb_signal_source)
        self.cmb_signal_source.currentTextChanged.connect(self._on_signal_source_changed)
        row2.addWidget(self.cmb_signal_source)

        lbl_pwr_src = QLabel("Power:")
        lbl_pwr_src.setStyleSheet("font-size:12px; background:transparent;")
        lbl_pwr_src.setToolTip("Select plotted power source (Physical / Control / Auto).")
        row2.addWidget(lbl_pwr_src)
        self.cmb_power_source = QComboBox()
        self.cmb_power_source.addItems(["Auto", "Physical", "Control"])
        self.cmb_power_source.setCurrentText("Auto")
        self.cmb_power_source.setFixedWidth(92)
        self.cmb_power_source.setFixedHeight(24)
        self.cmb_power_source.setToolTip(
            "Power source.\n"
            "Auto: RL+Pi uses Control power; otherwise Physical power.\n"
            "Physical: actuator measured torque * IMU velocity.\n"
            "Control: control-aligned torque * control-aligned velocity."
        )
        self._setup_combo(self.cmb_power_source)
        self.cmb_power_source.currentTextChanged.connect(self._on_power_source_changed)
        row2.addWidget(self.cmb_power_source)

        vsep_cap = QFrame(); vsep_cap.setFrameShape(QFrame.VLine)
        vsep_cap.setStyleSheet(f"background-color:{C.separator}; max-width:1px; border:none;")
        vsep_cap.setFixedHeight(20)
        row2.addWidget(vsep_cap)

        self.btn_screenshot = QPushButton("Screenshot")
        self.btn_screenshot.setCursor(Qt.PointingHandCursor)
        self.btn_screenshot.setFixedHeight(26)
        self.btn_screenshot.setStyleSheet(f"""
            QPushButton {{
                font-size:11px; padding:2px 10px; font-weight:600;
                background-color:{C.teal}; color:white;
                border:none; border-radius:8px;
            }}
            QPushButton:hover {{ background-color:#4ab8ea; }}
        """)
        self.btn_screenshot.setToolTip("Save current GUI as PNG image")
        self.btn_screenshot.clicked.connect(self._take_screenshot)
        row2.addWidget(self.btn_screenshot)

        self.btn_record = QPushButton("Record")
        self.btn_record.setCursor(Qt.PointingHandCursor)
        self.btn_record.setFixedHeight(26)
        self._apply_record_idle_style()
        self.btn_record.setToolTip("Start/stop screen recording (mp4 or image frames)")
        self.btn_record.clicked.connect(self._toggle_recording)
        row2.addWidget(self.btn_record)

        vsep_replay = QFrame(); vsep_replay.setFrameShape(QFrame.VLine)
        vsep_replay.setStyleSheet(f"background-color:{C.separator}; max-width:1px; border:none;")
        vsep_replay.setFixedHeight(20)
        row2.addWidget(vsep_replay)

        self.btn_replay_load = QPushButton("Load CSV")
        self.btn_replay_load.setCursor(Qt.PointingHandCursor)
        self.btn_replay_load.setFixedHeight(26)
        self.btn_replay_load.setStyleSheet(
            "font-size:11px; padding:2px 8px; font-weight:600;")
        self.btn_replay_load.setToolTip("Load a local CSV and start replay")
        self.btn_replay_load.clicked.connect(self._on_load_replay_csv)
        row2.addWidget(self.btn_replay_load)

        self.btn_replay_pause = QPushButton("Pause")
        self.btn_replay_pause.setCursor(Qt.PointingHandCursor)
        self.btn_replay_pause.setCheckable(True)
        self.btn_replay_pause.setFixedHeight(26)
        self.btn_replay_pause.setEnabled(False)
        self.btn_replay_pause.setStyleSheet(
            "font-size:11px; padding:2px 8px; font-weight:600;")
        self.btn_replay_pause.setToolTip("Pause or resume replay")
        self.btn_replay_pause.toggled.connect(self._on_replay_pause_toggled)
        row2.addWidget(self.btn_replay_pause)

        self.cmb_replay_speed = QComboBox()
        self.cmb_replay_speed.addItems(["0.2x", "0.5x", "1x", "2x", "4x", "8x"])
        self.cmb_replay_speed.setCurrentText("1x")
        self.cmb_replay_speed.setFixedWidth(70)
        self.cmb_replay_speed.setFixedHeight(24)
        self.cmb_replay_speed.setToolTip("Replay speed")
        self._setup_combo(self.cmb_replay_speed)
        self.cmb_replay_speed.currentIndexChanged.connect(self._on_replay_speed_changed)
        row2.addWidget(self.cmb_replay_speed)

        self.btn_replay_rw = QPushButton("<<5s")
        self.btn_replay_rw.setCursor(Qt.PointingHandCursor)
        self.btn_replay_rw.setFixedHeight(26)
        self.btn_replay_rw.setEnabled(False)
        self.btn_replay_rw.setStyleSheet(
            "font-size:11px; padding:2px 8px; font-weight:600;")
        self.btn_replay_rw.setToolTip("Rewind replay by 5 seconds")
        self.btn_replay_rw.clicked.connect(self._on_replay_rewind)
        row2.addWidget(self.btn_replay_rw)

        self.btn_replay_ff = QPushButton(">>5s")
        self.btn_replay_ff.setCursor(Qt.PointingHandCursor)
        self.btn_replay_ff.setFixedHeight(26)
        self.btn_replay_ff.setEnabled(False)
        self.btn_replay_ff.setStyleSheet(
            "font-size:11px; padding:2px 8px; font-weight:600;")
        self.btn_replay_ff.setToolTip("Fast-forward replay by 5 seconds")
        self.btn_replay_ff.clicked.connect(self._on_replay_fast_forward)
        row2.addWidget(self.btn_replay_ff)

        self.btn_replay_stop = QPushButton("Stop")
        self.btn_replay_stop.setCursor(Qt.PointingHandCursor)
        self.btn_replay_stop.setFixedHeight(26)
        self.btn_replay_stop.setEnabled(False)
        self.btn_replay_stop.setStyleSheet(
            "font-size:11px; padding:2px 8px; font-weight:600;")
        self.btn_replay_stop.setToolTip("Stop replay and return to live mode")
        self.btn_replay_stop.clicked.connect(self._on_replay_stop_clicked)
        row2.addWidget(self.btn_replay_stop)

        row2.addStretch(1)

        plot_card_vlay.addLayout(row2)

        # --- Row 3: Replay progress (time + draggable seek bar) ---
        row3 = QHBoxLayout(); row3.setSpacing(8)
        self.lbl_replay_progress_cur = QLabel("--:--")
        self.lbl_replay_progress_cur.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent; min-width:56px;")
        self.lbl_replay_progress_cur.setToolTip("Current replay time")
        row3.addWidget(self.lbl_replay_progress_cur)

        self.sld_replay_progress = QSlider(Qt.Horizontal)
        self.sld_replay_progress.setRange(0, REPLAY_PROGRESS_STEPS)
        self.sld_replay_progress.setValue(0)
        self.sld_replay_progress.setEnabled(False)
        self.sld_replay_progress.setToolTip("Drag to seek replay timeline")
        self.sld_replay_progress.sliderPressed.connect(self._on_replay_slider_pressed)
        self.sld_replay_progress.sliderReleased.connect(self._on_replay_slider_released)
        self.sld_replay_progress.sliderMoved.connect(self._on_replay_slider_moved)
        row3.addWidget(self.sld_replay_progress, 1)

        self.lbl_replay_progress_total = QLabel("/ --:--")
        self.lbl_replay_progress_total.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent; min-width:64px;")
        self.lbl_replay_progress_total.setToolTip("Total replay duration")
        row3.addWidget(self.lbl_replay_progress_total)

        self.lbl_replay_progress_rows = QLabel("[--/--]")
        self.lbl_replay_progress_rows.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent;")
        self.lbl_replay_progress_rows.setToolTip("Current sample index / total samples")
        row3.addWidget(self.lbl_replay_progress_rows)

        plot_card_vlay.addLayout(row3)

        # Display card goes to right side (plots area), added later in _build_plots
        self._plot_card = plot_card

        # ---- Hardware Card ----
        hw_card = CardFrame()
        self._cards.append(hw_card)
        hw_lay = QVBoxLayout(hw_card)
        hw_lay.setContentsMargins(12, 10, 12, 10)
        hw_lay.setSpacing(6)

        hw_lay.addWidget(_make_section_label("HARDWARE"))

        # IMU row
        imu_row = QHBoxLayout()
        self.btn_imu_init = QPushButton("IMU Init")
        self.btn_imu_init.setCursor(Qt.PointingHandCursor)
        self.btn_imu_init.setFixedHeight(28)
        self.btn_imu_init.clicked.connect(self._on_click_imu_init)
        imu_row.addWidget(self.btn_imu_init)

        self.lbl_imus = []
        for name in ["L", "R", "1", "2", "3", "4"]:
            lbl = QLabel(f"{name}:-")
            lbl.setStyleSheet(f"color:{C.text2}; font-size:11px; font-weight:500; background:transparent;")
            lbl.setFixedWidth(55)
            self.lbl_imus.append(lbl)
            imu_row.addWidget(lbl)
        imu_row.addStretch(1)
        hw_lay.addLayout(imu_row)

        # IMU battery row
        batt_row = QHBoxLayout()
        self.lbl_imu_batt_title = QLabel("Battery")
        self.lbl_imu_batt_title.setToolTip("IMU battery percentage from Teensy uplink (0-100%).")
        self.lbl_imu_batt_title.setStyleSheet(
            f"color:{C.text2}; font-size:11px; font-weight:600; background:transparent;"
        )
        self.lbl_imu_batt_title.setFixedWidth(76)
        batt_row.addWidget(self.lbl_imu_batt_title)
        self.lbl_imu_batts = []
        for name in ["L", "R", "1", "2", "3", "4"]:
            lbl = QLabel(f"{name}:--")
            lbl.setToolTip(f"IMU {name} battery percentage.")
            lbl.setStyleSheet(f"color:{C.text2}; font-size:11px; font-weight:500; background:transparent;")
            lbl.setFixedWidth(55)
            self.lbl_imu_batts.append(lbl)
            batt_row.addWidget(lbl)
        batt_row.addStretch(1)
        hw_lay.addLayout(batt_row)

        # Motor row
        motor_row = QHBoxLayout()
        self.btn_motor_init = QPushButton("Motor Init")
        self.btn_motor_init.setCursor(Qt.PointingHandCursor)
        self.btn_motor_init.setFixedHeight(28)
        self.btn_motor_init.setToolTip("Reinitialize motors for current brand. If pending brand differs, GUI will switch brand first.")
        self.btn_motor_init.clicked.connect(self._on_click_motor_init)
        motor_row.addWidget(self.btn_motor_init)

        lbl_brand = QLabel("Brand:")
        lbl_brand.setToolTip("Target motor brand selection.")
        motor_row.addWidget(lbl_brand)
        self.cmb_motor_brand = QComboBox()
        self.cmb_motor_brand.addItems(["SIG", "TMOTOR"])
        self.cmb_motor_brand.setCurrentIndex(1)  # default TMOTOR
        self.cmb_motor_brand.setFixedWidth(116)
        self.cmb_motor_brand.setToolTip("Select target brand, then click Apply.")
        self._setup_combo(self.cmb_motor_brand)
        self.cmb_motor_brand.currentIndexChanged.connect(self._on_brand_changed)
        motor_row.addWidget(self.cmb_motor_brand)

        self.btn_brand_apply = QPushButton("Apply SIG")
        self.btn_brand_apply.setCursor(Qt.PointingHandCursor)
        self.btn_brand_apply.setFixedHeight(28)
        self.btn_brand_apply.setToolTip("Apply selected brand switch on Teensy (switch + init).")
        self.btn_brand_apply.clicked.connect(self._on_brand_confirm)
        motor_row.addWidget(self.btn_brand_apply)

        self.badge_brand = PillBadge("Current:-", C.fill)
        self.badge_brand.setToolTip("Current active motor brand reported by Teensy.")
        motor_row.addWidget(self.badge_brand)
        self.lbl_temp_L = QLabel("")
        self.lbl_temp_L.setStyleSheet(f"color:{C.text2}; font-size:11px; background:transparent;")
        self.lbl_temp_R = QLabel("")
        self.lbl_temp_R.setStyleSheet(f"color:{C.text2}; font-size:11px; background:transparent;")
        motor_row.addWidget(self.lbl_temp_L)
        motor_row.addWidget(self.lbl_temp_R)
        motor_row.addStretch(1)
        hw_lay.addLayout(motor_row)

        # SD status
        sd_row = QHBoxLayout()
        self.badge_sd = PillBadge("SD: -", C.fill)
        sd_row.addWidget(self.badge_sd)
        sd_row.addStretch(1)
        hw_lay.addLayout(sd_row)

        left.addWidget(hw_card)

        # ---- Readout Card (angle/torque/power values) ----
        readout_card = CardFrame()
        self._cards.append(readout_card)
        readout_grid = QGridLayout(readout_card)
        readout_grid.setContentsMargins(12, 8, 12, 8)
        readout_grid.setSpacing(4)

        def _ro_lbl(txt, color, size=13, bold=False):
            l = QLabel(txt)
            w = 700 if bold else 400
            l.setStyleSheet(f"font-size:{size}px; font-weight:{w}; background:transparent; color:{color};")
            return l

        self.lbl_Lang = _ro_lbl("L: 0.0 deg", '#34c759', 13, True)
        self.lbl_Rang = _ro_lbl("R: 0.0 deg", '#34c759', 13, True)
        self.lbl_Lcmd = _ro_lbl("L cmd: 0.0 Nm", '#007aff')
        self.lbl_Rcmd = _ro_lbl("R cmd: 0.0 Nm", '#007aff')
        self.lbl_Ltau = _ro_lbl("L est: 0.0 Nm", '#ff3b30')
        self.lbl_Rtau = _ro_lbl("R est: 0.0 Nm", '#ff3b30')
        self.lbl_Lpwr = _ro_lbl("L pwr: 0.0 W", '#af52de')
        self.lbl_Rpwr = _ro_lbl("R pwr: 0.0 W", '#af52de')

        readout_grid.addWidget(self.lbl_Lang, 0, 0)
        readout_grid.addWidget(self.lbl_Rang, 0, 1)
        readout_grid.addWidget(self.lbl_Lcmd, 1, 0)
        readout_grid.addWidget(self.lbl_Rcmd, 1, 1)
        readout_grid.addWidget(self.lbl_Ltau, 2, 0)
        readout_grid.addWidget(self.lbl_Rtau, 2, 1)
        readout_grid.addWidget(self.lbl_Lpwr, 3, 0)
        readout_grid.addWidget(self.lbl_Rpwr, 3, 1)

        left.addWidget(readout_card)
        left.addStretch(1)

        # ---- Status bar ----
        self.lbl_status = QLabel("Ready")
        self.lbl_status.setStyleSheet(f"""
            color: {C.text2};
            font-size: 11px;
            padding: 4px 0;
            background: transparent;
        """)
        left.addWidget(self.lbl_status)

        self.plot_layout = plots

    # ================================================================ Port / Combo helpers
    def _setup_combo(self, combo: QComboBox):
        # Force Qt popup view so dark/light QSS works on macOS too.
        combo.setView(QListView(combo))
        self._combo_boxes.append(combo)

    def _refresh_ports(self):
        self._port_infos = list_available_port_infos()
        ports = [p["device"] for p in self._port_infos]
        selected = self.cmb_port.currentText()
        self.cmb_port.blockSignals(True)
        self.cmb_port.clear()
        if ports:
            self.cmb_port.addItems(ports)
            idx = self.cmb_port.findText(selected)
            self.cmb_port.setCurrentIndex(idx if idx >= 0 else 0)
        self.cmb_port.blockSignals(False)
        self._last_port_scan = time.time()
        return len(ports)

    def _score_auto_connect_port(self, info):
        dev = str(info.get("device", "") or "")
        dev_l = dev.lower()
        desc_l = str(info.get("description", "") or "").lower()
        mfg_l = str(info.get("manufacturer", "") or "").lower()
        hwid_l = str(info.get("hwid", "") or "").lower()
        blob = " ".join((desc_l, mfg_l, hwid_l))

        score = 0
        if info.get("vid") in AUTO_CONNECT_ADAFRUIT_VIDS:
            score += 120
        if "adafruit" in blob:
            score += 120
        if dev == self._last_connected_port and dev:
            score += 200
        if dev == self._last_auto_connect_port and dev:
            score += 40
        if "usbmodem" in dev_l:
            score += 15
        if "usbserial" in dev_l:
            score += 8
        # Avoid auto-connecting to generic Bluetooth serial endpoints.
        if ("bluetooth" in dev_l) or ("bluetooth" in blob):
            score -= 60
        return score

    def _pick_auto_connect_port(self):
        if not self._port_infos:
            return ""
        best_port = ""
        best_score = -10**9
        for info in self._port_infos:
            dev = str(info.get("device", "") or "")
            if not dev:
                continue
            score = self._score_auto_connect_port(info)
            if score > best_score:
                best_score = score
                best_port = dev
        # Require moderate confidence to avoid grabbing random ports while still
        # allowing plain usbmodem devices when metadata is sparse.
        return best_port if best_score >= 12 else ""

    def _on_refresh_ports_clicked(self):
        n_ports = self._refresh_ports()
        self.lbl_status.setText(f"Ports refreshed ({n_ports})")

    def _set_pi_rl_remote_buttons_enabled(self, enabled):
        for btn_name in ("btn_pi_rl_start_legdcp", "btn_pi_rl_start_pd", "btn_pi_rl_stop"):
            btn = getattr(self, btn_name, None)
            if btn is not None:
                btn.setEnabled(bool(enabled))

    def _set_pi_rl_remote_state(self, text, color=None, force=False):
        if not hasattr(self, "lbl_pi_rl_remote_state"):
            return
        if color is None:
            color = C.text2
        sig = (str(text), str(color))
        if (not force) and sig == self._pi_rl_remote_status_sig:
            return
        self._pi_rl_remote_status_sig = sig
        self.lbl_pi_rl_remote_state.setStyleSheet(
            f"color:{color}; font-size:11px; background:transparent; padding-top:1px;"
        )
        self.lbl_pi_rl_remote_state.setText(str(text))

    def _infer_pi_rl_remote_color(self, text):
        t = (text or "").lower()
        if "running" in t:
            if ("traceback" in t) or ("error" in t) or ("failed" in t):
                return C.orange
            return C.green
        if ("error" in t) or ("failed" in t):
            return C.red
        if ("starting" in t) or ("stopping" in t) or ("unknown" in t):
            return C.orange
        return C.text2

    def _app_user_config_dir(self):
        home = os.path.expanduser("~")
        if sys.platform == "darwin":
            return os.path.join(home, "Library", "Application Support", "HipExoController")
        if os.name == "nt":
            base = os.environ.get("APPDATA") or os.path.join(home, "AppData", "Roaming")
            return os.path.join(base, "HipExoController")
        return os.path.join(home, ".config", "HipExoController")

    def _legacy_rpi_profile_conf_path(self):
        return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "tools", "rpi_profiles.conf"))

    def _rpi_profile_conf_path(self):
        return os.path.join(self._app_user_config_dir(), "rpi_profiles.conf")

    def _ensure_rpi_profile_store(self):
        cfg_dir = self._app_user_config_dir()
        cfg_path = self._rpi_profile_conf_path()
        try:
            os.makedirs(cfg_dir, exist_ok=True)
        except Exception as exc:
            return False, f"cannot create config dir: {cfg_dir} ({exc})"

        if os.path.isfile(cfg_path):
            return True, None

        legacy = self._legacy_rpi_profile_conf_path()
        try:
            if os.path.isfile(legacy):
                shutil.copy2(legacy, cfg_path)
            else:
                with open(cfg_path, "w", encoding="utf-8") as f:
                    f.write("[active]\n")
                    f.write("profile = \n")
        except Exception as exc:
            return False, f"cannot initialize profile file: {cfg_path} ({exc})"
        return True, None

    def _load_rpi_profiles_cfg(self):
        ok, err = self._ensure_rpi_profile_store()
        if not ok:
            return None, err
        cfg_path = self._rpi_profile_conf_path()
        cfg = configparser.ConfigParser()
        try:
            cfg.read(cfg_path, encoding="utf-8")
        except Exception as exc:
            return None, f"failed to read profile config: {exc}"
        if not cfg.has_section("active"):
            cfg.add_section("active")
            cfg.set("active", "profile", "")
        return cfg, None

    def _save_rpi_profiles_cfg(self, cfg):
        cfg_path = self._rpi_profile_conf_path()
        try:
            os.makedirs(os.path.dirname(cfg_path), exist_ok=True)
            with open(cfg_path, "w", encoding="utf-8") as f:
                cfg.write(f)
        except Exception as exc:
            return False, f"failed to write profile config: {exc}"
        return True, None

    @staticmethod
    def _profile_section_names(cfg):
        return [s for s in cfg.sections() if s.lower() != "active"]

    @staticmethod
    def _active_profile_name(cfg):
        if cfg.has_section("active"):
            return (cfg.get("active", "profile", fallback="") or "").strip()
        return ""

    def _set_active_profile_name(self, profile_name):
        cfg, err = self._load_rpi_profiles_cfg()
        if err:
            return False, err
        if not cfg.has_section("active"):
            cfg.add_section("active")
        cfg.set("active", "profile", str(profile_name or "").strip())
        return self._save_rpi_profiles_cfg(cfg)

    def _reload_pi_profiles_ui(self, select_name=None):
        if not hasattr(self, "cmb_pi_profile"):
            return
        cfg, err = self._load_rpi_profiles_cfg()
        self.cmb_pi_profile.blockSignals(True)
        self.cmb_pi_profile.clear()
        if err:
            self.cmb_pi_profile.addItem("Config Error", "")
            self.cmb_pi_profile.blockSignals(False)
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red, force=True)
            return

        names = self._profile_section_names(cfg)
        active = self._active_profile_name(cfg)
        if not names:
            self.cmb_pi_profile.addItem("Not Configured", "")
            self.cmb_pi_profile.blockSignals(False)
            self._set_pi_rl_remote_state(
                "Pi RL Remote: Not Configured (click Configure Pi...)",
                C.orange,
                force=True,
            )
            return

        names_sorted = sorted(names)
        for n in names_sorted:
            self.cmb_pi_profile.addItem(n, n)
        target = (select_name or active or names_sorted[0]).strip()
        idx = self.cmb_pi_profile.findData(target)
        if idx < 0:
            idx = 0
            target = str(self.cmb_pi_profile.itemData(0) or self.cmb_pi_profile.itemText(0))
        self.cmb_pi_profile.setCurrentIndex(idx)
        self.cmb_pi_profile.blockSignals(False)
        # Ensure on-disk active profile follows UI selection.
        self._set_active_profile_name(target)

    def _on_pi_profile_changed(self, _idx):
        if not hasattr(self, "cmb_pi_profile"):
            return
        profile_name = (self.cmb_pi_profile.currentData() or "").strip()
        if not profile_name:
            return
        self._set_active_profile_name(profile_name)

    def _on_configure_pi_clicked(self, _checked=False, auto_prompt=False):
        cfg, err = self._load_rpi_profiles_cfg()
        if err:
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red, force=True)
            QMessageBox.warning(self, "Configure Pi", f"Failed to load profile config:\n{err}")
            return False

        dlg = QDialog(self)
        dlg.setWindowTitle("Configure Pi Profiles")
        dlg.setModal(True)
        dlg.resize(720, 430)

        lay = QVBoxLayout(dlg)
        path_label = QLabel(f"Profile store: {self._rpi_profile_conf_path()}")
        path_label.setWordWrap(True)
        path_label.setStyleSheet(f"color:{C.text2}; font-size:11px;")
        path_label.setToolTip("Local user config file where Pi SSH profiles are stored.")
        lay.addWidget(path_label)

        profile_sel_row = QHBoxLayout()
        lbl_profile = QLabel("Profile")
        lbl_profile.setToolTip("Select an existing profile to edit, or choose New Profile.")
        profile_sel_row.addWidget(lbl_profile)
        cmb_profiles = QComboBox()
        cmb_profiles.setView(QListView(cmb_profiles))
        cmb_profiles.setToolTip("Pi SSH profile list.")
        profile_sel_row.addWidget(cmb_profiles, 1)
        btn_new = QPushButton("New")
        btn_new.setToolTip("Create a new Pi SSH profile.")
        btn_delete = QPushButton("Delete")
        btn_delete.setToolTip("Delete the currently selected profile.")
        profile_sel_row.addWidget(btn_new)
        profile_sel_row.addWidget(btn_delete)
        lay.addLayout(profile_sel_row)

        form = QFormLayout()
        le_name = QLineEdit()
        le_name.setPlaceholderText("e.g. lab-pi5")
        le_name.setToolTip("Unique profile name shown in the Pi Profile dropdown.")
        le_host = QLineEdit()
        le_host.setPlaceholderText("e.g. 192.168.1.50")
        le_host.setToolTip("Raspberry Pi hostname or IP address.")
        sb_port = QSpinBox()
        sb_port.setRange(1, 65535)
        sb_port.setValue(22)
        sb_port.setToolTip("SSH port on Raspberry Pi (default 22).")
        le_user = QLineEdit()
        le_user.setPlaceholderText("e.g. pi")
        le_user.setToolTip("SSH username.")
        le_remote_dir = QLineEdit()
        le_remote_dir.setPlaceholderText("e.g. ~/Desktop/RPi_Unified")
        le_remote_dir.setToolTip("Remote directory containing RL_controller_torch.py on Pi.")

        auth_row = QHBoxLayout()
        cmb_auth = QComboBox()
        cmb_auth.setView(QListView(cmb_auth))
        cmb_auth.addItem("SSH Key", "key")
        cmb_auth.addItem("Password", "password")
        cmb_auth.setToolTip("Authentication method used by local SSH.")
        auth_row.addWidget(cmb_auth, 1)
        chk_set_active = QCheckBox("Set as active profile")
        chk_set_active.setChecked(True)
        chk_set_active.setToolTip("If checked, this profile becomes the default active profile.")
        auth_row.addWidget(chk_set_active, 0)

        identity_row = QHBoxLayout()
        le_identity = QLineEdit()
        le_identity.setPlaceholderText("e.g. ~/.ssh/id_ed25519")
        le_identity.setToolTip("Path to local SSH private key file.")
        btn_browse_id = QPushButton("Browse...")
        btn_browse_id.setToolTip("Browse for local SSH private key file.")
        identity_row.addWidget(le_identity, 1)
        identity_row.addWidget(btn_browse_id, 0)

        le_password = QLineEdit()
        le_password.setEchoMode(QLineEdit.Password)
        le_password.setPlaceholderText("Optional (requires sshpass on local machine)")
        le_password.setToolTip("SSH password (used only in Password mode, requires sshpass).")

        lbl_name = QLabel("Profile Name")
        lbl_name.setToolTip("Unique profile identifier.")
        lbl_host = QLabel("Host")
        lbl_host.setToolTip("Pi hostname or IP.")
        lbl_port = QLabel("Port")
        lbl_port.setToolTip("Pi SSH port.")
        lbl_user = QLabel("User")
        lbl_user.setToolTip("Pi SSH user.")
        lbl_remote_dir = QLabel("Remote Dir")
        lbl_remote_dir.setToolTip("Directory where RL script runs on Pi.")
        lbl_auth = QLabel("Auth")
        lbl_auth.setToolTip("Select SSH key or password authentication.")
        lbl_identity = QLabel("Identity File")
        lbl_identity.setToolTip("Local private key path for SSH key mode.")
        lbl_password = QLabel("Password")
        lbl_password.setToolTip("Used only in Password mode.")

        form.addRow(lbl_name, le_name)
        form.addRow(lbl_host, le_host)
        form.addRow(lbl_port, sb_port)
        form.addRow(lbl_user, le_user)
        form.addRow(lbl_remote_dir, le_remote_dir)
        form.addRow(lbl_auth, auth_row)
        form.addRow(lbl_identity, identity_row)
        form.addRow(lbl_password, le_password)
        lay.addLayout(form)

        info = QLabel(
            "Start/Stop Pi RL uses local SSH from this GUI app. "
            "Keep at least one valid profile configured."
        )
        info.setWordWrap(True)
        info.setStyleSheet(f"color:{C.text2}; font-size:11px;")
        info.setToolTip("Remote launcher uses your local machine SSH to control Pi tmux session.")
        lay.addWidget(info)

        action_row = QHBoxLayout()
        btn_save = QPushButton("Save Profile")
        btn_save.setToolTip("Save current profile changes to user config file.")
        btn_save.setStyleSheet(
            f"background-color:{C.blue}; color:white; font-weight:600; border-radius:8px; padding:6px 14px;"
        )
        action_row.addWidget(btn_save)
        action_row.addStretch(1)
        btn_close = QPushButton("Close")
        btn_close.setToolTip("Close dialog.")
        btn_close.clicked.connect(dlg.accept)
        action_row.addWidget(btn_close)
        lay.addLayout(action_row)

        new_tag = "__new__"

        def _apply_auth_mode_ui():
            mode = str(cmb_auth.currentData() or "key")
            use_key = (mode == "key")
            le_identity.setEnabled(use_key)
            btn_browse_id.setEnabled(use_key)
            le_password.setEnabled(not use_key)

        def _clear_form_defaults():
            le_name.clear()
            le_host.clear()
            sb_port.setValue(22)
            le_user.clear()
            le_remote_dir.setText("~/Desktop/RPi_Unified")
            cmb_auth.setCurrentIndex(0)
            le_identity.clear()
            le_password.clear()
            chk_set_active.setChecked(True)
            _apply_auth_mode_ui()

        def _populate_from_profile(profile_name):
            if not profile_name or (not cfg.has_section(profile_name)):
                _clear_form_defaults()
                btn_delete.setEnabled(False)
                return
            sec = cfg[profile_name]
            le_name.setText(profile_name)
            le_host.setText((sec.get("host") or "").strip())
            try:
                port_v = int((sec.get("port") or "22").strip())
            except Exception:
                port_v = 22
            sb_port.setValue(max(1, min(65535, port_v)))
            le_user.setText((sec.get("user") or "").strip())
            le_remote_dir.setText((sec.get("remote_dir") or "").strip())
            identity_file = (sec.get("identity_file") or "").strip()
            password = (sec.get("password") or "").strip()
            if password and (not identity_file):
                cmb_auth.setCurrentIndex(1)
            else:
                cmb_auth.setCurrentIndex(0)
            le_identity.setText(identity_file)
            le_password.setText(password)
            chk_set_active.setChecked(self._active_profile_name(cfg) == profile_name)
            _apply_auth_mode_ui()
            btn_delete.setEnabled(True)

        def _refresh_profiles_combo(select_name=""):
            names = sorted(self._profile_section_names(cfg))
            cmb_profiles.blockSignals(True)
            cmb_profiles.clear()
            for nm in names:
                cmb_profiles.addItem(nm, nm)
            cmb_profiles.addItem("New Profile...", new_tag)
            target = (select_name or "").strip()
            if not target:
                target = self._active_profile_name(cfg)
            idx = cmb_profiles.findData(target)
            if idx < 0:
                idx = cmb_profiles.findData(new_tag) if not names else 0
            cmb_profiles.setCurrentIndex(max(0, idx))
            cmb_profiles.blockSignals(False)
            current = str(cmb_profiles.currentData() or "")
            _populate_from_profile("" if current == new_tag else current)

        def _current_selected_profile():
            sel = str(cmb_profiles.currentData() or "").strip()
            if sel == new_tag:
                return ""
            return sel

        def _collect_profile_from_form(for_save=True):
            profile_name = le_name.text().strip()
            if for_save and (not profile_name):
                return None, "profile name is required"
            if profile_name.lower() == "active":
                return None, "profile name 'active' is reserved"
            sec_map = {
                "host": le_host.text().strip(),
                "user": le_user.text().strip(),
                "remote_dir": le_remote_dir.text().strip(),
                "port": str(int(sb_port.value())),
                "identity_file": le_identity.text().strip() if (str(cmb_auth.currentData() or "key") == "key") else "",
                "password": le_password.text().strip() if (str(cmb_auth.currentData() or "key") == "password") else "",
            }
            parsed, parse_err = self._validate_profile_entry(profile_name, sec_map, validate_identity=False)
            if parse_err:
                return None, parse_err
            auth_mode = str(cmb_auth.currentData() or "key")
            if auth_mode == "key" and parsed.get("identity_file"):
                id_path = str(parsed.get("identity_file"))
                if not os.path.isfile(id_path):
                    return None, f"identity_file not found: {id_path}"
            return parsed, None

        def _save_profile():
            parsed, parse_err = _collect_profile_from_form(for_save=True)
            if parse_err:
                QMessageBox.warning(dlg, "Configure Pi", parse_err)
                return

            new_name = str(parsed["profile_name"]).strip()
            old_name = _current_selected_profile()
            if old_name and (old_name != new_name) and cfg.has_section(new_name):
                QMessageBox.warning(
                    dlg,
                    "Configure Pi",
                    f"profile '{new_name}' already exists. Choose another name.",
                )
                return

            if old_name and (old_name != new_name) and cfg.has_section(old_name):
                cfg.remove_section(old_name)
            if not cfg.has_section(new_name):
                cfg.add_section(new_name)
            sec = cfg[new_name]
            sec["host"] = str(parsed["host"])
            sec["port"] = str(int(parsed["port"]))
            sec["user"] = str(parsed["user"])
            sec["remote_dir"] = str(parsed["remote_dir"])
            if parsed.get("identity_file"):
                sec["identity_file"] = str(parsed["identity_file"])
                sec["password"] = ""
            else:
                sec["identity_file"] = ""
                sec["password"] = str(le_password.text().strip())

            if not cfg.has_section("active"):
                cfg.add_section("active")
            if chk_set_active.isChecked() or (not self._active_profile_name(cfg)):
                cfg.set("active", "profile", new_name)

            ok, save_err = self._save_rpi_profiles_cfg(cfg)
            if not ok:
                QMessageBox.warning(dlg, "Configure Pi", save_err or "failed to save profile")
                return
            _refresh_profiles_combo(select_name=new_name)
            self._reload_pi_profiles_ui(select_name=new_name)
            self._set_pi_rl_remote_state(
                f"Pi RL Remote: profile [{new_name}] saved",
                C.green,
                force=True,
            )

        def _delete_profile():
            profile_name = _current_selected_profile()
            if not profile_name:
                return
            reply = QMessageBox.question(
                dlg,
                "Delete Profile",
                f"Delete Pi profile '{profile_name}'?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return
            if cfg.has_section(profile_name):
                cfg.remove_section(profile_name)
            names = sorted(self._profile_section_names(cfg))
            if not cfg.has_section("active"):
                cfg.add_section("active")
            active_name = self._active_profile_name(cfg)
            if (not names) or (active_name == profile_name):
                cfg.set("active", "profile", names[0] if names else "")
            ok, save_err = self._save_rpi_profiles_cfg(cfg)
            if not ok:
                QMessageBox.warning(dlg, "Configure Pi", save_err or "failed to save profile")
                return
            next_name = names[0] if names else ""
            _refresh_profiles_combo(select_name=next_name)
            self._reload_pi_profiles_ui(select_name=next_name)
            self._set_pi_rl_remote_state(
                f"Pi RL Remote: profile [{profile_name}] deleted",
                C.orange,
                force=True,
            )

        def _on_profile_choice_changed(_idx):
            selected = str(cmb_profiles.currentData() or "")
            _populate_from_profile("" if selected == new_tag else selected)

        def _switch_to_new_profile():
            idx = cmb_profiles.findData(new_tag)
            if idx < 0:
                _refresh_profiles_combo(select_name="")
                idx = cmb_profiles.findData(new_tag)
            if idx >= 0:
                cmb_profiles.setCurrentIndex(idx)
                _clear_form_defaults()
                le_name.setFocus()

        def _browse_identity():
            path, _ = QFileDialog.getOpenFileName(
                dlg,
                "Select SSH private key",
                os.path.expanduser("~/.ssh"),
                "All Files (*)",
            )
            if path:
                le_identity.setText(path)

        cmb_auth.currentIndexChanged.connect(_apply_auth_mode_ui)
        cmb_profiles.currentIndexChanged.connect(_on_profile_choice_changed)
        btn_save.clicked.connect(_save_profile)
        btn_new.clicked.connect(_switch_to_new_profile)
        btn_delete.clicked.connect(_delete_profile)
        btn_browse_id.clicked.connect(_browse_identity)

        _refresh_profiles_combo()
        if auto_prompt and (not self._profile_section_names(cfg)):
            QMessageBox.information(
                dlg,
                "Configure Pi",
                "No Pi profile is configured yet.\nPlease create one to start RL remotely.",
            )
            _switch_to_new_profile()

        dlg.exec_()
        self._reload_pi_profiles_ui()

        profile, load_err = self._load_rpi_ssh_profile()
        return bool(profile and (load_err is None))

    def _validate_profile_entry(self, profile_name, sec, validate_identity=True):
        host = (sec.get("host") or "").strip()
        user = (sec.get("user") or "").strip()
        remote_dir = (sec.get("remote_dir") or "").strip()
        password = (sec.get("password") or "").strip()
        port_txt = (sec.get("port") or "22").strip()
        identity_file = (sec.get("identity_file") or "").strip()

        if not host or not user or not remote_dir:
            return None, f"profile [{profile_name}] missing host/user/remote_dir"
        try:
            port = int(port_txt)
        except ValueError:
            return None, f"profile [{profile_name}] invalid port: {port_txt}"
        if not (1 <= port <= 65535):
            return None, f"profile [{profile_name}] port out of range: {port}"
        if identity_file:
            identity_file = os.path.expanduser(identity_file)
            if validate_identity and (not os.path.isfile(identity_file)):
                return None, f"profile [{profile_name}] identity_file not found: {identity_file}"
        else:
            identity_file = None

        return {
            "profile_name": profile_name,
            "host": host,
            "user": user,
            "remote_dir": remote_dir,
            "password": password or None,
            "port": port,
            "identity_file": identity_file,
        }, None

    def _load_rpi_ssh_profile(self, profile_name=None):
        cfg, err = self._load_rpi_profiles_cfg()
        if err:
            return None, err

        names = self._profile_section_names(cfg)
        if not names:
            return None, "no Pi profile configured"

        if profile_name is None and hasattr(self, "cmb_pi_profile"):
            profile_name = (self.cmb_pi_profile.currentData() or "").strip()
        if not profile_name:
            profile_name = self._active_profile_name(cfg)
        if not profile_name or (not cfg.has_section(profile_name)):
            profile_name = names[0]

        sec = cfg[profile_name]
        return self._validate_profile_entry(profile_name, sec, validate_identity=True)

    def _build_ssh_command(self, remote_body, profile=None):
        if profile is None:
            profile, err = self._load_rpi_ssh_profile()
            if err:
                return None, None, err

        ssh_cmd = [
            "ssh",
            "-p", str(profile["port"]),
            "-o", "StrictHostKeyChecking=accept-new",
            "-o", "LogLevel=ERROR",
        ]
        if profile.get("identity_file"):
            ssh_cmd.extend(["-i", str(profile["identity_file"])])
        ssh_cmd.append(f"{profile['user']}@{profile['host']}")
        ssh_cmd.append(f"bash -lc {shlex.quote(remote_body)}")

        password = profile.get("password")
        if password:
            if shutil.which("sshpass") is None:
                return None, profile, "sshpass not found; install sshpass or use SSH key login"
            ssh_cmd = ["sshpass", "-p", str(password)] + ssh_cmd

        return ssh_cmd, profile, None

    def _dispatch_async_job(self, job_type, cmd, timeout_s=RPI_RL_SSH_TIMEOUT_S, meta=None):
        if meta is None:
            meta = {}
        job_id = int(self._async_job_next_id)
        self._async_job_next_id += 1
        self._async_job_active[job_id] = {"type": str(job_type), "meta": dict(meta)}

        def _worker():
            t0 = time.time()
            result = {
                "job_id": job_id,
                "type": str(job_type),
                "meta": dict(meta),
                "returncode": -999,
                "stdout": "",
                "stderr": "",
                "elapsed_s": 0.0,
            }
            try:
                proc = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=float(timeout_s),
                    check=False,
                )
                result["returncode"] = int(proc.returncode)
                result["stdout"] = proc.stdout or ""
                result["stderr"] = proc.stderr or ""
            except subprocess.TimeoutExpired as exc:
                result["returncode"] = -124
                result["stdout"] = (exc.stdout or "") if isinstance(exc.stdout, str) else ""
                result["stderr"] = (exc.stderr or "") if isinstance(exc.stderr, str) else "timeout"
            except Exception as exc:
                result["returncode"] = -999
                result["stderr"] = str(exc)
            result["elapsed_s"] = max(0.0, time.time() - t0)
            self._async_job_result_q.put(result)

        threading.Thread(target=_worker, daemon=True).start()
        return job_id

    @staticmethod
    def _first_nonempty_line(*texts):
        for txt in texts:
            if not txt:
                continue
            for ln in str(txt).splitlines():
                ln = ln.strip()
                if ln:
                    return ln
        return ""

    def _build_pi_rl_start_body(self, profile, nn_type):
        session_q = shlex.quote(self._pi_rl_tmux_session)
        remote_dir = str(profile["remote_dir"]).rstrip("/")
        remote_dir_q = shlex.quote(remote_dir)
        rl_script_q = shlex.quote(f"{remote_dir}/RL_controller_torch.py")
        nn_q = shlex.quote(str(nn_type))
        run_body = (
            f"cd {remote_dir_q} && "
            f"source ~/venvs/pytorch-env/bin/activate && "
            f"python RL_controller_torch.py --nn {nn_q}"
        )
        return (
            "set -e; "
            "if ! command -v tmux >/dev/null 2>&1; then echo '__ERR__:tmux not found'; exit 101; fi; "
            f"if [ ! -d {remote_dir_q} ]; then echo '__ERR__:remote_dir missing'; exit 102; fi; "
            f"if [ ! -f {rl_script_q} ]; then echo '__ERR__:RL_controller_torch.py missing'; exit 103; fi; "
            "if [ ! -f ~/venvs/pytorch-env/bin/activate ]; then echo '__ERR__:venv activate missing'; exit 104; fi; "
            f"tmux kill-session -t {session_q} 2>/dev/null || true; "
            f"tmux new-session -d -s {session_q} bash -lc {shlex.quote(run_body)}; "
            "sleep 0.25; "
            f"if tmux has-session -t {session_q} 2>/dev/null; then echo '__OK__:started'; else echo '__ERR__:tmux start failed'; exit 105; fi"
        )

    def _build_pi_rl_stop_body(self):
        session_q = shlex.quote(self._pi_rl_tmux_session)
        return f"tmux kill-session -t {session_q} 2>/dev/null || true; echo '__OK__:stopped'"

    def _build_pi_rl_poll_body(self):
        session_q = shlex.quote(self._pi_rl_tmux_session)
        return (
            f"if tmux has-session -t {session_q} 2>/dev/null; then "
            "echo '__STATE__:RUNNING'; "
            f"tmux capture-pane -pt {session_q} -S -120; "
            "else "
            "echo '__STATE__:STOPPED'; "
            "fi"
        )

    def _on_pi_rl_start_clicked(self, nn_type):
        if self._pi_rl_cmd_inflight:
            return
        nn_type = str(nn_type).strip()
        if nn_type not in ("lstm_leg_dcp", "lstm_pd"):
            self._set_pi_rl_remote_state(f"Pi RL Remote: unsupported nn '{nn_type}'", C.red, force=True)
            return
        # Keep staged GUI display aligned with Pi-side runtime defaults even before
        # the first RPi status frame returns.
        if nn_type == "lstm_pd":
            self._set_rl_delay_spinbox_value(200.0)
        elif nn_type == "lstm_leg_dcp":
            self._set_rl_delay_spinbox_value(100.0)
        self._update_rl_filter_state_label()
        profile, err = self._load_rpi_ssh_profile()
        if err and ("no pi profile configured" in str(err).lower()):
            self._set_pi_rl_remote_state(
                "Pi RL Remote: no profile configured, opening Configure Pi...",
                C.orange,
                force=True,
            )
            ok = self._on_configure_pi_clicked(auto_prompt=True)
            if ok:
                profile, err = self._load_rpi_ssh_profile()
        if err or profile is None:
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red, force=True)
            return
        ssh_cmd, profile, err = self._build_ssh_command(
            self._build_pi_rl_start_body(profile, nn_type),
            profile=profile,
        )
        if err:
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red, force=True)
            return

        self._pi_rl_cmd_inflight = True
        self._set_pi_rl_remote_buttons_enabled(False)
        self._set_pi_rl_remote_state(
            f"Pi RL Remote: starting {nn_type} on [{profile['profile_name']}]...",
            C.orange,
            force=True,
        )
        self._queue_align_event(f"pi_rl_start_req:{nn_type}")
        self._dispatch_async_job(
            "pi_rl_start",
            ssh_cmd,
            timeout_s=RPI_RL_SSH_TIMEOUT_S,
            meta={"nn_type": nn_type, "profile_name": profile["profile_name"]},
        )

    def _on_pi_rl_stop_clicked(self):
        if self._pi_rl_cmd_inflight:
            return
        ssh_cmd, profile, err = self._build_ssh_command(self._build_pi_rl_stop_body())
        if err:
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red, force=True)
            return
        self._pi_rl_cmd_inflight = True
        self._set_pi_rl_remote_buttons_enabled(False)
        self._set_pi_rl_remote_state(
            f"Pi RL Remote: stopping on [{profile['profile_name']}]...",
            C.orange,
            force=True,
        )
        self._queue_align_event("pi_rl_stop_req")
        self._dispatch_async_job(
            "pi_rl_stop",
            ssh_cmd,
            timeout_s=RPI_RL_SSH_TIMEOUT_S,
            meta={"profile_name": profile["profile_name"]},
        )

    def _parse_pi_rl_poll_output(self, stdout_text):
        out = stdout_text or ""
        lines = out.splitlines()
        state = "UNKNOWN"
        body_lines = []
        for i, raw_ln in enumerate(lines):
            ln = raw_ln.strip()
            if ln.startswith("__STATE__:"):
                state = ln.split(":", 1)[1].strip().upper()
                body_lines = lines[i + 1:]
                break
        if not body_lines:
            body_lines = lines

        nn_found = ""
        pat_nn = re.compile(r"NN_TYPE\s*=\s*([A-Za-z0-9_]+)")
        pat_arg = re.compile(r"--nn\s+([A-Za-z0-9_]+)")
        for ln in reversed(body_lines):
            m = pat_nn.search(ln)
            if m:
                nn_found = m.group(1)
                break
            m2 = pat_arg.search(ln)
            if m2:
                nn_found = m2.group(1)
                break

        err_hint = ""
        text_blob = "\n".join(body_lines)
        if "Traceback (most recent call last)" in text_blob:
            err_hint = "traceback detected"
        else:
            err_pat = re.compile(r"(error|exception|failed|not found|No module named)", re.IGNORECASE)
            for ln in reversed(body_lines):
                ls = ln.strip()
                if ls and err_pat.search(ls):
                    err_hint = ls[:140]
                    break
        return state, nn_found, err_hint

    def _drain_async_jobs(self):
        while True:
            try:
                res = self._async_job_result_q.get_nowait()
            except queue.Empty:
                break
            job_id = int(res.get("job_id", -1))
            self._async_job_active.pop(job_id, None)
            job_type = str(res.get("type", ""))
            rc = int(res.get("returncode", -999))
            out = str(res.get("stdout", "") or "")
            err = str(res.get("stderr", "") or "")
            meta = res.get("meta", {}) or {}

            if job_type == "pi_rl_start":
                self._pi_rl_cmd_inflight = False
                self._set_pi_rl_remote_buttons_enabled(True)
                nn_type = str(meta.get("nn_type", "") or "")
                prof = str(meta.get("profile_name", "") or "")
                if rc == 0 and "__OK__:started" in out:
                    self._pi_rl_remote_running = True
                    self._pi_rl_remote_nn = nn_type
                    self._pi_rl_remote_last_error = ""
                    self._set_pi_rl_remote_state(
                        f"Pi RL Remote: RUNNING ({nn_type}) on [{prof}]",
                        C.green,
                        force=True,
                    )
                    nn_tag = "RLONPD" if nn_type == "lstm_pd" else "RLONLDC"
                    self._queue_align_event(f"pi_rl_started:{nn_type}", pi_tag=nn_tag)
                    self._pi_rl_remote_last_poll_ts = 0.0
                else:
                    msg = self._first_nonempty_line(out, err) or f"start failed (rc={rc})"
                    self._pi_rl_remote_running = False
                    self._pi_rl_remote_last_error = msg
                    self._set_pi_rl_remote_state(
                        f"Pi RL Remote: start failed | {msg}",
                        C.red,
                        force=True,
                    )
                    self._queue_align_event("pi_rl_start_failed")

            elif job_type == "pi_rl_stop":
                self._pi_rl_cmd_inflight = False
                self._set_pi_rl_remote_buttons_enabled(True)
                if rc == 0:
                    self._pi_rl_remote_running = False
                    self._pi_rl_remote_last_error = ""
                    self._set_pi_rl_remote_state("Pi RL Remote: STOPPED", C.text2, force=True)
                    self._queue_align_event("pi_rl_stopped", pi_tag="RLOFF")
                else:
                    msg = self._first_nonempty_line(out, err) or f"stop failed (rc={rc})"
                    self._pi_rl_remote_last_error = msg
                    self._set_pi_rl_remote_state(
                        f"Pi RL Remote: stop failed | {msg}",
                        C.red,
                        force=True,
                    )
                    self._queue_align_event("pi_rl_stop_failed")

            elif job_type == "pi_rl_poll":
                self._pi_rl_poll_inflight = False
                if rc != 0:
                    msg = self._first_nonempty_line(err, out) or f"poll failed (rc={rc})"
                    self._pi_rl_remote_last_error = msg
                    self._set_pi_rl_remote_state(
                        f"Pi RL Remote: poll error | {msg}",
                        C.red,
                    )
                    continue

                state, nn_found, err_hint = self._parse_pi_rl_poll_output(out)
                if nn_found:
                    self._pi_rl_remote_nn = nn_found
                if state == "RUNNING":
                    self._pi_rl_remote_running = True
                    if err_hint:
                        self._pi_rl_remote_last_error = err_hint
                        nn_txt = self._pi_rl_remote_nn if self._pi_rl_remote_nn else "--"
                        self._set_pi_rl_remote_state(
                            f"Pi RL Remote: RUNNING ({nn_txt}) | {err_hint}",
                            C.orange,
                        )
                    else:
                        self._pi_rl_remote_last_error = ""
                        nn_txt = self._pi_rl_remote_nn if self._pi_rl_remote_nn else "--"
                        self._set_pi_rl_remote_state(
                            f"Pi RL Remote: RUNNING ({nn_txt})",
                            C.green,
                        )
                elif state == "STOPPED":
                    self._pi_rl_remote_running = False
                    self._set_pi_rl_remote_state("Pi RL Remote: STOPPED", C.text2)
                else:
                    self._set_pi_rl_remote_state("Pi RL Remote: state unknown", C.orange)

    def _maybe_poll_pi_rl_remote(self, now=None, force=False):
        if now is None:
            now = time.time()
        if (not force) and (self._algo_select != ALGO_RL) and (not self._pi_rl_remote_running):
            return
        if self._pi_rl_cmd_inflight or self._pi_rl_poll_inflight:
            return
        if (not force) and (now - self._pi_rl_remote_last_poll_ts) < RPI_RL_POLL_INTERVAL_S:
            return

        ssh_cmd, profile, err = self._build_ssh_command(self._build_pi_rl_poll_body())
        self._pi_rl_remote_last_poll_ts = now
        if err:
            self._set_pi_rl_remote_state(f"Pi RL Remote: {err}", C.red)
            return

        self._pi_rl_poll_inflight = True
        self._dispatch_async_job(
            "pi_rl_poll",
            ssh_cmd,
            timeout_s=RPI_RL_SSH_TIMEOUT_S,
            meta={"profile_name": profile["profile_name"]},
        )

    def _build_rl_passthrough(self):
        """Pack RL runtime tuning payload for BLE payload[58:98]."""
        pt = bytearray(40)
        pt[0] = RPI_PT_MAGIC0
        pt[1] = RPI_PT_MAGIC1
        pt[2] = RPI_PT_VERSION
        pt[3] = 0x01  # command: apply runtime tuning

        filt_idx = max(0, min(self.cmb_rl_filter_type.currentIndex(), len(RL_FILTER_TYPES) - 1))
        _, filter_code = RL_FILTER_TYPES[filt_idx]
        pt[4] = filter_code & 0xFF
        pt[5] = 2  # filter order, fixed for now
        en_mask = 0
        if self.chk_rl_vr_filter.isChecked():
            # Vel/Ref packed together
            en_mask |= 0x01
            en_mask |= 0x02
        if self.chk_rl_torque_filter.isChecked():
            en_mask |= 0x04
        pt[6] = en_mask & 0xFF

        struct.pack_into('<f', pt, 8, float(self.sb_rl_scale.value()))
        struct.pack_into('<f', pt, 12, float(self.sb_rl_torque_delay.value()))
        struct.pack_into('<f', pt, 16, float(self.sb_rl_cutoff_hz.value()))
        auto_flags = 0
        if self.chk_rl_auto_delay.isChecked():
            auto_flags |= RPI_AUTO_FLAG_ENABLE
        if hasattr(self, 'cmb_rl_auto_method') and self.cmb_rl_auto_method.currentIndex() == 1:
            auto_flags |= RPI_AUTO_FLAG_METHOD_BO
        pt[20] = auto_flags & 0xFF
        return pt

    def _on_apply_rl_clicked(self):
        """Apply RL button: send staged RL params to RPi via BLE passthrough."""
        if not (self.connected and self.ser):
            return
        self._queue_align_event("rl_apply", pi_tag="RLAPPLY")
        self._tx_params()
        self._update_rl_filter_state_label()
        # Re-affirm motor direction after RPi params are sent
        QTimer.singleShot(150, self._tx_params)

    def _on_rl_auto_delay_toggled(self, _state):
        self._update_rl_delay_input_mode()
        self._update_rl_filter_state_label()

    def _on_sam_auto_delay_toggled(self, _state):
        auto_on = self.chk_sam_auto_delay.isChecked()
        self._sam_auto_delay_enable = auto_on
        self.sb_sam_delay.setEnabled(not auto_on)
        self.btn_sam_reset.setEnabled(auto_on)
        if auto_on:
            self.sb_sam_delay.setToolTip("Auto Delay ON: Teensy optimizing delay_ms (L/R independent).")
        else:
            self.sb_sam_delay.setToolTip("")
        self._tx_params()

    def _on_sam_reset_clicked(self):
        """Reset Samsung Auto Delay: send falling+rising edge on auto_delay_enable to cold-start ADO."""
        if not self._sam_auto_delay_enable:
            return
        self._sam_auto_delay_enable = False
        self._tx_params()
        self._sam_auto_delay_enable = True
        self._tx_params()

    def _on_eg_auto_delay_toggled(self, _state):
        auto_on = self.chk_eg_auto_delay.isChecked()
        self._eg_auto_delay_enable = auto_on
        self.sb_eg_post_delay.setEnabled(not auto_on)
        self.sb_Assist_delay_gain.setEnabled(not auto_on)
        self.btn_eg_reset.setEnabled(auto_on)
        if auto_on:
            self.sb_eg_post_delay.setToolTip("Auto Delay ON: Teensy optimizing post_delay_ms (L/R independent).")
            self.sb_Assist_delay_gain.setToolTip("Auto Delay ON: delay index locked.")
        else:
            self.sb_eg_post_delay.setToolTip(
                "EG output post-delay (ms). Applied after internal Assist_delay_gain.")
            self.sb_Assist_delay_gain.setToolTip("")
        self._tx_params()

    def _on_eg_reset_clicked(self):
        """Reset EG Auto Delay: send falling+rising edge on auto_delay_enable to cold-start ADO."""
        if not self._eg_auto_delay_enable:
            return
        self._eg_auto_delay_enable = False
        self._tx_params()
        self._eg_auto_delay_enable = True
        self._tx_params()

    def _set_rl_delay_spinbox_value(self, delay_ms: float):
        if not hasattr(self, "sb_rl_torque_delay"):
            return
        delay = max(0.0, min(1000.0, float(delay_ms)))
        if abs(float(self.sb_rl_torque_delay.value()) - delay) < 0.05:
            return
        self.sb_rl_torque_delay.blockSignals(True)
        self.sb_rl_torque_delay.setValue(delay)
        self.sb_rl_torque_delay.blockSignals(False)

    def _set_rl_delay_pair_text(self):
        if not hasattr(self, "lbl_rl_torque_delay_auto"):
            return
        if self._rpi_status_valid:
            txt = f"L={self._rpi_delay_ms_L:.1f} ms | R={self._rpi_delay_ms_R:.1f} ms"
        else:
            txt = "L=--.- ms | R=--.- ms"
        self.lbl_rl_torque_delay_auto.setText(txt)

    def _update_rl_delay_input_mode(self):
        """Auto Delay ON: lock delay input and mirror active delay from RPi status."""
        if (not hasattr(self, "sb_rl_torque_delay") or
                not hasattr(self, "chk_rl_auto_delay") or
                not hasattr(self, "rl_delay_stack")):
            return
        auto_on = self.chk_rl_auto_delay.isChecked()
        self.sb_rl_torque_delay.setEnabled(not auto_on)
        if auto_on:
            self.sb_rl_torque_delay.setToolTip("Auto Delay ON: follows RPi active runtime delay.")
            if self._rpi_status_valid:
                self._set_rl_delay_spinbox_value(self._rpi_delay_ms)
            self._set_rl_delay_pair_text()
            self.rl_delay_stack.setCurrentIndex(1)
        else:
            self.sb_rl_torque_delay.setToolTip("RL torque delay absolute (ms), 0~1000")
            self.rl_delay_stack.setCurrentIndex(0)

    def _update_rl_panel_for_nn_type(self):
        """根据 RPi 报告的 nn_type 调整 RL 面板控件可见性"""
        if not hasattr(self, "lbl_rpi_nn_type"):
            return

        nn = self._rpi_nn_type
        nn_names = {0: "DNN", 1: "LSTM", 2: "LSTM-LegDcp", 3: "LSTM-PD"}
        nn_name = nn_names.get(nn, f"Unknown({nn})")
        self.lbl_rpi_nn_type.setText(f"RPi: {nn_name}")
        self.lbl_rpi_nn_type.setStyleSheet(
            f"color:{C.green}; font-size:13px; font-weight:600; background:transparent;")

        is_dnn = (nn == 0)

        # Filter Type + Cutoff: 所有网络类型都显示 (DNN 和 LSTM 都有可调滤波器)
        self.lbl_rl_filter_type.setVisible(True)
        self.cmb_rl_filter_type.setVisible(True)
        self.lbl_rl_cutoff.setVisible(True)
        self.sb_rl_cutoff_hz.setVisible(True)

        # Vel+Ref: 仅 DNN 有意义, LSTM 只有 torque 前滤波
        self.chk_rl_vr_filter.setVisible(is_dnn)
        if not is_dnn:
            self.chk_rl_vr_filter.blockSignals(True)
            self.chk_rl_vr_filter.setChecked(False)
            self.chk_rl_vr_filter.blockSignals(False)

        # LSTM: 默认勾选 Torque 滤波 (这是 LSTM 的核心滤波器)
        if not is_dnn:
            self.chk_rl_torque_filter.blockSignals(True)
            self.chk_rl_torque_filter.setChecked(True)
            self.chk_rl_torque_filter.blockSignals(False)

        # ---- 按网络类型设置推荐预设 ----
        # nn=0: DNN     → scale=0.50, delay=200ms, cutoff=5.0Hz, Butterworth, Vel+Ref+Torque ON
        # nn=1: LSTM    → scale=1.00, delay=0ms,   cutoff=5.0Hz, Butterworth, Torque ON
        # nn=2: LegDcp  → scale=1.00, delay=100ms, cutoff=5.0Hz, Butterworth, Torque ON
        # nn=3: LSTM-PD → scale=0.40, delay=200ms, cutoff=5.0Hz, Butterworth, Torque ON
        _presets = {
            0: dict(scale=0.50, delay=200.0, cutoff=5.0,  vr=True,  torque=True),
            1: dict(scale=1.00, delay=0.0,   cutoff=5.0,  vr=False, torque=True),
            2: dict(scale=1.00, delay=100.0, cutoff=5.0,  vr=False, torque=True),
            3: dict(scale=0.40, delay=200.0, cutoff=5.0,  vr=False, torque=True),
        }
        preset = _presets.get(nn, _presets[1])
        self.sb_rl_scale.setValue(preset['scale'])
        self.sb_rl_torque_delay.setValue(preset['delay'])
        self.sb_rl_cutoff_hz.setValue(preset['cutoff'])
        # Filter type combo: index 0 = Butterworth for all presets
        self.cmb_rl_filter_type.blockSignals(True)
        self.cmb_rl_filter_type.setCurrentIndex(0)
        self.cmb_rl_filter_type.blockSignals(False)
        if is_dnn:
            self.chk_rl_vr_filter.setChecked(preset['vr'])
        self.chk_rl_torque_filter.blockSignals(True)
        self.chk_rl_torque_filter.setChecked(preset['torque'])
        self.chk_rl_torque_filter.blockSignals(False)
        self._update_rl_delay_input_mode()

    def _update_rpi_offline_ui(self):
        """RPi 超时断线时更新 UI"""
        if not hasattr(self, "lbl_rpi_nn_type"):
            return
        self._rpi_auto_method_bo = False
        self.lbl_rpi_nn_type.setText("RPi: Offline")
        self.lbl_rpi_nn_type.setStyleSheet(
            f"color:{C.red}; font-size:13px; font-weight:600; background:transparent;")
        self.lbl_rpi_current_filter.setText("RPi Active: no connection")
        if hasattr(self, "lbl_rl_auto_state"):
            self.lbl_rl_auto_state.setText("Auto Delay: no connection")
        self._update_power_strip_titles(force=True)

    def _update_rl_filter_state_label(self):
        if not hasattr(self, "lbl_rl_filter_state"):
            return
        self._update_rl_delay_input_mode()

        # --- RPi current filter display (from uplink) ---
        if self._rpi_status_valid:
            src = "Base" if self._rpi_filter_source == 0 else "Runtime Override"
            ftype_map = {0: "Preset", 1: "Butterworth", 2: "Bessel", 3: "Chebyshev2"}
            ftype = ftype_map.get(self._rpi_filter_type_code, "?")
            en_v = "ON" if (self._rpi_enable_mask & 0x01) else "OFF"
            en_r = "ON" if (self._rpi_enable_mask & 0x02) else "OFF"
            en_t = "ON" if (self._rpi_enable_mask & 0x04) else "OFF"
            auto_en = "ON" if self._rpi_auto_delay_enable else "OFF"
            method_rpi = "BO" if self._rpi_auto_method_bo else "Grid"

            # 非 auto 模式下 L/R 同步; auto 模式下可能分离 → 显示 "L/R" 对
            if abs(self._rpi_delay_ms_L - self._rpi_delay_ms_R) < 0.5:
                delay_str = f"Delay={self._rpi_delay_ms_L:.0f}ms"
            else:
                delay_str = f"Delay L/R={self._rpi_delay_ms_L:.0f}/{self._rpi_delay_ms_R:.0f}ms"
            if self._rpi_nn_type == 0:  # DNN
                current_str = (
                    f"[{src}] {ftype} {self._rpi_cutoff_hz:.1f}Hz order={self._rpi_filter_order} | "
                    f"Vel={en_v} Ref={en_r} Torque={en_t} | "
                    f"Scale={self._rpi_scale:.2f} {delay_str} | Auto={auto_en}/{method_rpi}"
                )
            else:  # LSTM
                current_str = (
                    f"[{src}] Torque Filter={en_t}"
                )
                if en_t == "ON" and self._rpi_filter_type_code > 0:
                    current_str += f" {ftype} {self._rpi_cutoff_hz:.1f}Hz order={self._rpi_filter_order}"
                current_str += f" | Scale={self._rpi_scale:.2f} {delay_str} | Auto={auto_en}/{method_rpi}"
            self.lbl_rpi_current_filter.setText(f"RPi Active: {current_str}")
        else:
            self.lbl_rpi_current_filter.setText("RPi Active: waiting for status...")

        # --- GUI pending/sent display ---
        idx = max(0, min(self.cmb_rl_filter_type.currentIndex(), len(RL_FILTER_TYPES) - 1))
        filt_name = RL_FILTER_TYPES[idx][0]
        vr = "ON" if self.chk_rl_vr_filter.isChecked() else "OFF"
        tq = "ON" if self.chk_rl_torque_filter.isChecked() else "OFF"
        ad = "ON" if self.chk_rl_auto_delay.isChecked() else "OFF"
        method_gui = "BO" if (hasattr(self, 'cmb_rl_auto_method') and self.cmb_rl_auto_method.currentIndex() == 1) else "Grid"
        cutoff = float(self.sb_rl_cutoff_hz.value())
        delay = float(self.sb_rl_torque_delay.value())
        scale = float(self.sb_rl_scale.value())
        if self._algo_select == ALGO_RL:
            if self.connected and self.ser:
                state = (
                    f"GUI Sent(seq={self._rl_cfg_tx_seq}): "
                    f"{filt_name} {cutoff:.1f}Hz, Vel+Ref={vr}, Torque={tq}, "
                    f"Scale={scale:.2f}, Delay={delay:.0f}ms, Auto={ad}/{method_gui}"
                )
            else:
                state = (
                    f"GUI Pending: "
                    f"{filt_name} {cutoff:.1f}Hz, Vel+Ref={vr}, Torque={tq}, "
                    f"Scale={scale:.2f}, Delay={delay:.0f}ms, Auto={ad}/{method_gui}"
                )
        else:
            state = f"Algo={ALGO_NAMES.get(self._algo_select, '?')}. Override sent only when RL active."
        self.lbl_rl_filter_state.setText(state)

        if hasattr(self, "lbl_rl_auto_state"):
            if self._rpi_status_valid:
                auto_state = "ON" if self._rpi_auto_delay_enable else "OFF"
                method_state = "BO" if self._rpi_auto_method_bo else "Grid"
                mL = "VALID" if self._rpi_auto_motion_valid_L else "HOLD"
                mR = "VALID" if self._rpi_auto_motion_valid_R else "HOLD"
                self.lbl_rl_auto_state.setText(
                    f"Auto Delay(RPi): {auto_state}/{method_state}\n"
                    f"  L: motion={mL} | ratio={self._rpi_power_ratio_L:.3f} | "
                    f"+P={self._rpi_pos_per_s_L:+.2f}W | -P={self._rpi_neg_per_s_L:+.2f}W | "
                    f"delay={self._rpi_delay_ms_L:.0f}ms | best={self._rpi_best_delay_ms_L:.0f}ms\n"
                    f"  R: motion={mR} | ratio={self._rpi_power_ratio_R:.3f} | "
                    f"+P={self._rpi_pos_per_s_R:+.2f}W | -P={self._rpi_neg_per_s_R:+.2f}W | "
                    f"delay={self._rpi_delay_ms_R:.0f}ms | best={self._rpi_best_delay_ms_R:.0f}ms"
                )
            else:
                self.lbl_rl_auto_state.setText("Auto Delay: waiting for RPi status...")

        # Samsung / EG auto delay state labels (Teensy-native source, v3 format same fields)
        if hasattr(self, "lbl_sam_auto_delay_state"):
            if not self._rpi_status_valid:
                self.lbl_sam_auto_delay_state.setText("L=-- ms  R=-- ms")
            else:
                mL = "V" if self._rpi_auto_motion_valid_L else "-"
                mR = "V" if self._rpi_auto_motion_valid_R else "-"
                base_ms = float(self.sb_sam_delay.value())
                oL = self._rpi_delay_ms_L - base_ms
                oR = self._rpi_delay_ms_R - base_ms
                if self._sam_auto_delay_enable:
                    self.lbl_sam_auto_delay_state.setText(
                        f"L:{oL:+.0f}ms({mL})  R:{oR:+.0f}ms({mR})")
                else:
                    self.lbl_sam_auto_delay_state.setText(
                        f"L:{self._rpi_delay_ms_L:.0f}ms  R:{self._rpi_delay_ms_R:.0f}ms")

        if hasattr(self, "lbl_eg_auto_delay_state"):
            if not self._rpi_status_valid:
                self.lbl_eg_auto_delay_state.setText("L=-- idx  R=-- idx")
            else:
                mL = "V" if self._rpi_auto_motion_valid_L else "-"
                mR = "V" if self._rpi_auto_motion_valid_R else "-"
                base_ms = float(self.sb_eg_post_delay.value())
                oL_idx = (self._rpi_delay_ms_L - base_ms) / 10.0
                oR_idx = (self._rpi_delay_ms_R - base_ms) / 10.0
                if self._eg_auto_delay_enable:
                    self.lbl_eg_auto_delay_state.setText(
                        f"L:{oL_idx:+.1f}idx({mL})  R:{oR_idx:+.1f}idx({mR})")
                else:
                    self.lbl_eg_auto_delay_state.setText(
                        f"L:{self._rpi_delay_ms_L/10:.1f}idx  R:{self._rpi_delay_ms_R/10:.1f}idx")

        self._update_power_strip_titles()

    def _maybe_update_rl_filter_state_label(self, now=None, force=False):
        if now is None:
            now = time.time()
        if (not force) and (now - self._rl_status_ui_last_ts) < self._rl_status_ui_min_interval_s:
            return
        self._update_rl_filter_state_label()
        self._rl_status_ui_last_ts = now

    # ================================================================ Test waveform
    def _on_test_waveform_changed(self, index):
        is_sin = (index == 1)
        self.lbl_test_freq.setVisible(is_sin)
        self.sb_test_freq.setVisible(is_sin)
        self._tx_params()

    # ================================================================ Algorithm
    def _on_algo_selected(self, index):
        if 0 <= index < len(_SEG_TO_ALGO):
            self._algo_pending = _SEG_TO_ALGO[index]
        else:
            self._algo_pending = ALGO_EG
        self.algo_stack.setCurrentIndex(index)
        self._update_rl_filter_state_label()

    def _on_algo_confirm(self):
        self._algo_select = self._algo_pending
        algo_name = ALGO_NAMES.get(self._algo_select, '?')
        print(f"[GUI] Algorithm CONFIRMED: {algo_name}")
        # Force fresh status parsing after algorithm switch (avoid stale overlay state).
        self._rpi_status_valid = False
        self._rpi_online = False
        self._rpi_last_rx_time = 0.0
        self._rpi_status_version = 0
        self._rpi_nn_type = -1
        self._prev_rpi_nn_type = -1
        algo_tag_map = {
            "EG": "ALG_EG",
            "Samsung": "ALG_SAM",
            "RL": "ALG_RL",
            "SOGI": "ALG_SOGI",
            "Test": "ALG_TEST",
        }
        self._queue_align_event(f"algo_apply:{algo_name}", pi_tag=algo_tag_map.get(algo_name))
        self._update_rl_filter_state_label()
        if self.connected and self.ser:
            # Send a short burst to survive occasional BLE/UART frame loss.
            for k in range(4):
                QTimer.singleShot(35 * k, self._tx_params)
            # Re-affirm motor direction after firmware processes algo change
            # (firmware may re-initialize direction state on algo switch)
            QTimer.singleShot(35 * 4 + 100, self._tx_params)

    # ================================================================ Callbacks
    def _on_brand_changed(self, index):
        self._brand_pending = index + 1
        self._update_brand_apply_label()
        target = "SIG" if self._brand_pending == 1 else "TMOTOR"
        self.lbl_status.setText(f"Brand selected: {target}. Click Apply {target}.")

    def _update_brand_apply_label(self):
        if not hasattr(self, "btn_brand_apply"):
            return
        target = "SIG" if self._brand_pending == 1 else "TMOTOR"
        if self._current_brand in (1, 2) and self._current_brand == self._brand_pending:
            self.btn_brand_apply.setText(f"{target} Active")
        else:
            self.btn_brand_apply.setText(f"Apply {target}")
        self._refresh_brand_apply_style()
        self._refresh_brand_combo_style()

    def _refresh_brand_apply_style(self):
        if not hasattr(self, "btn_brand_apply"):
            return
        active = (self._current_brand in (1, 2) and self._current_brand == self._brand_pending)
        bg = C.green if active else C.blue
        self.btn_brand_apply.setStyleSheet(f"""
            QPushButton {{
                background-color: {bg};
                color: white; border: none; border-radius: 8px;
                font-weight: 700; font-size: 12px; padding: 4px 10px;
            }}
        """)

    def _refresh_brand_combo_style(self):
        if not hasattr(self, "cmb_motor_brand"):
            return
        pending_diff = (
            self._current_brand in (1, 2) and
            self._brand_pending in (1, 2) and
            self._brand_pending != self._current_brand
        )
        border = C.orange if pending_diff else C.blue
        bg = C.fill if pending_diff else C.card
        self.cmb_motor_brand.setStyleSheet(f"""
            QComboBox {{
                background-color: {bg};
                color: {C.text};
                border: 2px solid {border};
                border-radius: 8px;
                padding: 2px 10px;
                font-size: 13px;
                font-weight: 700;
            }}
            QComboBox::drop-down {{
                border: none;
                width: 22px;
            }}
        """)

    def _on_brand_confirm(self):
        target = "SIG" if self._brand_pending == 1 else "TMOTOR"
        if not (self.connected and self.ser):
            self.lbl_status.setText(f"Brand pending: {target} (connect first)")
            return
        if self._current_brand in (1, 2) and self._current_brand == self._brand_pending:
            self.lbl_status.setText(f"Brand already {target}")
            self._update_brand_apply_label()
            return

        self._brand_request = self._brand_pending
        for k in range(4):
            QTimer.singleShot(35 * k, self._tx_params)
        QTimer.singleShot(200, lambda: setattr(self, "_brand_request", 0))
        self.lbl_status.setText(f"Applying brand: {target} ...")

    def _pulse_motor_init_request(self):
        self._motor_init_request = True
        self._tx_params()
        if self.ser:
            self.ser.flush()
        QTimer.singleShot(100, lambda: setattr(self, "_motor_init_request", False))

    def _on_click_imu_init(self):
        if not (self.connected and self.ser): return
        self._imu_init_request = True
        self._tx_params()
        self.ser.flush()
        QTimer.singleShot(100, lambda: setattr(self, "_imu_init_request", False))

    def _on_click_motor_init(self):
        if not (self.connected and self.ser):
            return
        target = "SIG" if self._brand_pending == 1 else "TMOTOR"
        cur = "SIG" if self._current_brand == 1 else ("TMOTOR" if self._current_brand == 2 else "-")
        if self._brand_pending in (1, 2) and self._brand_pending != self._current_brand:
            # More intuitive UX: if target brand differs, switch brand first then init.
            self._brand_request = self._brand_pending
            for k in range(4):
                QTimer.singleShot(35 * k, self._tx_params)
            QTimer.singleShot(200, lambda: setattr(self, "_brand_request", 0))
            QTimer.singleShot(280, self._pulse_motor_init_request)
            self.lbl_status.setText(f"Motor Init: switch {cur}->{target}, then re-init")
            return
        self._pulse_motor_init_request()
        self.lbl_status.setText(f"Motor Init sent ({cur})")

    def _toggle_left_dir(self):
        self._dir_bits ^= 0x01
        self._update_dir_btns()
        self._tx_params()

    def _toggle_right_dir(self):
        self._dir_bits ^= 0x02
        self._update_dir_btns()
        self._tx_params()

    def _toggle_visual_L(self):
        self._visual_sign_L *= -1
        self._update_dir_btns()

    def _toggle_visual_R(self):
        self._visual_sign_R *= -1
        self._update_dir_btns()

    def _update_dir_btns(self):
        lp = "+" if (self._dir_bits & 0x01) else "-"
        rp = "+" if (self._dir_bits & 0x02) else "-"
        self.btn_toggle_L.setText(f"L{lp}")
        self.btn_toggle_R.setText(f"R{rp}")
        vl = "+" if self._visual_sign_L > 0 else "-"
        vr = "+" if self._visual_sign_R > 0 else "-"
        self.btn_visual_L.setText(f"VL{vl}")
        self.btn_visual_R.setText(f"VR{vr}")

        all_orig = (self._dir_bits == 0x03 and
                   self._visual_sign_L == 1 and self._visual_sign_R == 1)
        if all_orig:
            self.badge_original.setText("Original")
            self.badge_original.set_color(C.green)
        else:
            parts = []
            if not (self._dir_bits & 0x01): parts.append("L")
            if not (self._dir_bits & 0x02): parts.append("R")
            if self._visual_sign_L < 0: parts.append("vL")
            if self._visual_sign_R < 0: parts.append("vR")
            self.badge_original.setText("Mod:" + ",".join(parts))
            self.badge_original.set_color(C.orange)

    def _set_power_ui(self, on: bool):
        if on:
            self.btn_power.setChecked(True)
            self.btn_power.setText("POWER ON")
            self.btn_power.setStyleSheet(f"""
                QPushButton {{
                    font-size: 22px; font-weight: 700; padding: 14px 24px;
                    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                        stop:0 #3cd070, stop:1 {C.green});
                    color: white; border-radius: 14px; border: none;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                        stop:0 #2fbf5f, stop:1 #28a745);
                }}
            """)
        else:
            self.btn_power.setChecked(False)
            self.btn_power.setText("POWER OFF")
            self.btn_power.setStyleSheet(f"""
                QPushButton {{
                    font-size: 22px; font-weight: 700; padding: 14px 24px;
                    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                        stop:0 #ff5555, stop:1 {C.red});
                    color: white; border-radius: 14px; border: none;
                }}
                QPushButton:hover {{
                    background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                        stop:0 #e53935, stop:1 #c62828);
                }}
            """)

    def _on_power_toggled(self, checked):
        if checked:
            val = self._maxT_before_off if self._maxT_before_off > 0.0 else 15.0
            self.sb_max_torque_cfg.setValue(val)
            self._set_power_ui(True)
        else:
            self._maxT_before_off = float(self.sb_max_torque_cfg.value())
            self.sb_max_torque_cfg.setValue(0.0)
            self._set_power_ui(False)

    def _auto_cycle_mile(self):
        if not self._mile_values: return
        value = self._mile_values[self._mile_index]
        self.edt_label.setText(value)
        self.sw_persist.setChecked(True)
        self._send_logtag()
        self._mile_index = (self._mile_index + 1) % len(self._mile_values)
        self.btn_auto_mile.setText(f"Auto Mile ({self._mile_values[self._mile_index]})")

    def _on_align_mark_clicked(self):
        mark_id = int(self._align_mark_counter)
        self._align_mark_counter += 1
        event_name = f"manual_align:{mark_id:03d}"
        pi_tag = f"ALN{mark_id % 1000:03d}"
        self._queue_align_event(event_name, pi_tag=pi_tag)
        self.lbl_status.setText(f"Align mark inserted: {event_name} (pi_tag={pi_tag})")

    def _clear_buffers(self):
        self._init_buffers()
        self._prev_L_angle = 0.0
        self._prev_R_angle = 0.0
        self._prev_wall_time = 0.0
        self._pwr_vel_good_L = 0.0
        self._pwr_vel_good_R = 0.0
        self._pwr_glitch_count_L = 0
        self._pwr_glitch_count_R = 0
        self._render_dirty = True
        self._last_render_values = None
        self._last_render_ts = 0.0

    def _on_signal_source_changed(self, text):
        self._signal_source_mode = str(text or "Auto")

    def _on_power_source_changed(self, text):
        self._power_source_mode = str(text or "Auto")

    def _resolve_live_sources(self, active_algo: int):
        signal_mode = str(self._signal_source_mode)
        power_mode = str(self._power_source_mode)

        sync_ok = bool(self._telem_sync_valid)
        sync_from_pi = bool(self._sync_from_pi)
        ctrl_ok = bool(self._telem_ctrl_valid)
        phys_ok = bool(self._telem_phys_valid)

        if signal_mode == "Raw":
            signal_active = "Raw"
        elif signal_mode == "Sync":
            signal_active = "Sync" if sync_ok else "Raw"
        else:
            if active_algo == ALGO_RL and sync_ok and sync_from_pi:
                signal_active = "Sync"
            else:
                signal_active = "Raw"

        if power_mode == "Physical":
            power_active = "Physical" if phys_ok else ("Control" if ctrl_ok else "None")
        elif power_mode == "Control":
            power_active = "Control" if ctrl_ok else ("Physical" if phys_ok else "None")
        else:
            # Auto power should follow controller-synchronous power first.
            if active_algo == ALGO_RL and ctrl_ok and sync_from_pi:
                power_active = "Control"
            elif ctrl_ok:
                power_active = "Control"
            elif phys_ok:
                power_active = "Physical"
            else:
                power_active = "None"

        self._signal_source_active = signal_active
        self._power_source_active = power_active

        if signal_active == "Sync":
            L_angle_disp = float(self._sync_ang_L)
            R_angle_disp = float(self._sync_ang_R)
            L_vel_disp = float(self._sync_vel_L)
            R_vel_disp = float(self._sync_vel_R)
            L_cmd_disp = float(self._sync_cmd_L)
            R_cmd_disp = float(self._sync_cmd_R)
        else:
            L_angle_disp = float(self._raw_ang_L)
            R_angle_disp = float(self._raw_ang_R)
            L_vel_disp = float(self._raw_vel_L)
            R_vel_disp = float(self._raw_vel_R)
            L_cmd_disp = float(self._raw_cmd_L)
            R_cmd_disp = float(self._raw_cmd_R)

        if power_active == "Control":
            L_pwr_disp = float(self._ctrl_pwr_L)
            R_pwr_disp = float(self._ctrl_pwr_R)
        elif power_active == "Physical":
            L_pwr_disp = float(self._phys_pwr_L)
            R_pwr_disp = float(self._phys_pwr_R)
        else:
            L_pwr_disp = 0.0
            R_pwr_disp = 0.0

        return (
            L_angle_disp, R_angle_disp, L_vel_disp, R_vel_disp,
            L_cmd_disp, R_cmd_disp, L_pwr_disp, R_pwr_disp,
            signal_active, power_active,
        )

    def _sanitize_power_velocity(self, vel_raw: float, prev_good: float):
        """Display-only velocity guard for power calculation."""
        v = float(vel_raw)
        if not isfinite(v):
            return float(prev_good), False
        if abs(v) > float(self._pwr_vel_abs_limit_dps):
            return float(prev_good), False
        if abs(v - float(prev_good)) > float(self._pwr_vel_jump_limit_dps):
            return float(prev_good), False
        return v, True

    def _append_data_point(
            self, t, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d,
            L_vel, R_vel, gait_freq, *, log_csv=True, power_override=None):
        self._last_gait_freq = float(gait_freq)
        vL, vR = self._visual_sign_L, self._visual_sign_R
        # Motor L+/R+ should only affect real actuator output, not plot sign.
        # Compensate uplink torque/cmd back to logical sign for display.
        if self._replay_mode:
            mL = 1
            mR = 1
        else:
            mL = 1 if (self._dir_bits & 0x01) else -1
            mR = 1 if (self._dir_bits & 0x02) else -1
        self.t_buffer.append(float(t))
        # Visual sign toggles affect torque display only (Cmd/Est).
        self.L_IMU_buf.append(float(L_angle))
        self.R_IMU_buf.append(float(R_angle))
        self.L_tau_buf.append(float(L_tau) * mL * vL)
        self.R_tau_buf.append(float(R_tau) * mR * vR)
        self.L_tau_d_buf.append(float(L_tau_d) * mL * vL)
        self.R_tau_d_buf.append(float(R_tau_d) * mR * vR)
        self.L_vel_buf.append(float(L_vel))
        self.R_vel_buf.append(float(R_vel))

        if power_override is None:
            L_pwr = 0.0
            R_pwr = 0.0
        else:
            L_pwr = float(power_override[0])
            R_pwr = float(power_override[1])
            if not isfinite(L_pwr):
                L_pwr = 0.0
            if not isfinite(R_pwr):
                R_pwr = 0.0

        self.L_pwr_buf.append(L_pwr)
        self.R_pwr_buf.append(R_pwr)

        if log_csv and hasattr(self, '_csv_writer'):
            now_wall = time.time()
            event_txt = ""
            if self._align_events_pending:
                event_txt = " | ".join(self._align_events_pending)
                self._align_events_pending.clear()
            rx_tag_text = self._last_rx_tag_text if self._last_rx_tag_text else self._last_rx_tag_char
            self._csv_writer.writerow([
                f'{float(t):.3f}',
                f'{float(L_angle):.3f}', f'{float(R_angle):.3f}',
                str(int(self._uplink_prev_t_cs) & 0xFFFF if self._uplink_prev_t_cs is not None else -1),
                f'{float(self._uplink_t_unwrapped_s):.3f}',
                f'{float(L_tau_d):.4f}', f'{float(R_tau_d):.4f}',
                f'{float(L_tau):.4f}',   f'{float(R_tau):.4f}',
                f'{float(L_vel):.3f}',   f'{float(R_vel):.3f}',
                f'{L_pwr:.4f}',   f'{R_pwr:.4f}',
                str(self._signal_source_mode),
                str(self._signal_source_active),
                str(self._power_source_mode),
                str(self._power_source_active),
                f'{float(self._raw_ang_L):.3f}', f'{float(self._raw_ang_R):.3f}',
                f'{float(self._raw_vel_L):.3f}', f'{float(self._raw_vel_R):.3f}',
                f'{float(self._raw_cmd_L):.4f}', f'{float(self._raw_cmd_R):.4f}',
                f'{float(self._sync_ang_L):.3f}', f'{float(self._sync_ang_R):.3f}',
                f'{float(self._sync_vel_L):.3f}', f'{float(self._sync_vel_R):.3f}',
                f'{float(self._sync_cmd_L):.4f}', f'{float(self._sync_cmd_R):.4f}',
                f'{float(self._phys_pwr_L):.4f}', f'{float(self._phys_pwr_R):.4f}',
                f'{float(self._ctrl_pwr_L):.4f}', f'{float(self._ctrl_pwr_R):.4f}',
                str(int(self._sync_sample_id) & 0xFFFF),
                str(int(1 if self._sync_from_pi else 0)),
                f'{float(gait_freq):.3f}',
                f'{float(self._uplink_t_unwrapped_s):.3f}',
                f'{now_wall:.6f}',
                f'{(now_wall - float(self._csv_start_wall_time)):.3f}',
                str(int(self._csv_sample_idx)),
                str(self._csv_session_id),
                ALGO_NAMES.get(self._algo_select, "?"),
                str(self._current_tag),
                str(self._last_rx_tag_char),
                str(rx_tag_text),
                str(event_txt),
                str(int(self._rl_cfg_tx_seq)),
                str(int(1 if self._rpi_online else 0)),
                str(int(self._rpi_status_version)),
                str(int(self._rpi_nn_type)),
                f'{float(self._rpi_delay_ms_L):.3f}',
                f'{float(self._rpi_delay_ms_R):.3f}',
                f'{float(self._rpi_power_ratio_L):.6f}',
                f'{float(self._rpi_power_ratio_R):.6f}',
                f'{float(self._rpi_pos_per_s_L):.4f}',
                f'{float(self._rpi_pos_per_s_R):.4f}',
                f'{float(self._rpi_neg_per_s_L):.4f}',
                f'{float(self._rpi_neg_per_s_R):.4f}',
            ])
            self._csv_sample_idx += 1
            if now_wall - self._csv_last_flush > 1.0:
                self._csv_file.flush()
                self._csv_last_flush = now_wall

        self._last_render_values = (
            float(L_angle), float(R_angle),
            float(L_tau_d), float(R_tau_d),
            float(L_tau), float(R_tau),
            float(L_pwr), float(R_pwr),
        )
        self._render_dirty = True
        return L_pwr, R_pwr

    def _render_interval_s(self):
        return 1.0 / max(1.0, float(self._render_fps_normal))

    def _maybe_render_plot(self, now=None, force=False):
        if now is None:
            now = time.time()
        if not force and not self._render_dirty:
            return
        if (not force) and (now - self._last_render_ts) < self._render_interval_s():
            return
        self._render_plot_frame(now)

    def _render_plot_frame(self, now=None):
        if now is None:
            now = time.time()
        tl = list(self.t_buffer)
        if not tl:
            return

        # Materialize once per frame to avoid repeated deque->list conversions.
        R_IMU = list(self.R_IMU_buf)
        R_tau = list(self.R_tau_buf)
        R_tau_d = list(self.R_tau_d_buf)
        R_vel = list(self.R_vel_buf)
        R_pwr = list(self.R_pwr_buf)
        L_IMU = list(self.L_IMU_buf)
        L_tau = list(self.L_tau_buf)
        L_tau_d = list(self.L_tau_d_buf)
        L_vel = list(self.L_vel_buf)
        L_pwr = list(self.L_pwr_buf)

        self.L_angle_line.setData(tl, R_IMU)
        self.L_tau_line.setData(tl, R_tau)
        self.L_tau_d_line.setData(tl, R_tau_d)
        self.L_vel_line.setData(tl, R_vel)
        self.L_pwr_line.setData(tl, R_pwr)
        self.R_angle_line.setData(tl, L_IMU)
        self.R_tau_line.setData(tl, L_tau)
        self.R_tau_d_line.setData(tl, L_tau_d)
        self.R_vel_line.setData(tl, L_vel)
        self.R_pwr_line.setData(tl, L_pwr)
        self._update_power_strip(tl, R_pwr, 'right')
        self._update_power_strip(tl, L_pwr, 'left')

        if self.sw_auto_scroll.isChecked() and len(tl) > 1:
            x0 = tl[0]
            x1 = tl[-1]
            self.plot_left.getViewBox().setXRange(x0, x1, padding=0.02)
            self.plot_right.getViewBox().setXRange(x0, x1, padding=0.02)

        if self._last_render_values is not None:
            (L_angle, R_angle, L_tau_d, R_tau_d,
             L_tau, R_tau, L_pwr_now, R_pwr_now) = self._last_render_values
            self.lbl_Lang.setText(f"L: {L_angle:.1f} deg")
            self.lbl_Rang.setText(f"R: {R_angle:.1f} deg")
            self.lbl_Lcmd.setText(f"L cmd: {L_tau_d:.1f} Nm")
            self.lbl_Rcmd.setText(f"R cmd: {R_tau_d:.1f} Nm")
            self.lbl_Ltau.setText(f"L est: {L_tau:.1f} Nm")
            self.lbl_Rtau.setText(f"R est: {R_tau:.1f} Nm")
            self.lbl_Lpwr.setText(f"L pwr: {L_pwr_now:.2f} W")
            self.lbl_Rpwr.setText(f"R pwr: {R_pwr_now:.2f} W")

        self._render_dirty = False
        self._last_render_ts = now

    def _on_win_size_changed(self, val):
        self.win_size = val
        for attr in ['t_buffer', 'L_IMU_buf', 'R_IMU_buf', 'L_tau_buf', 'R_tau_buf',
                     'L_tau_d_buf', 'R_tau_d_buf', 'L_vel_buf', 'R_vel_buf',
                     'L_pwr_buf', 'R_pwr_buf']:
            old = getattr(self, attr)
            setattr(self, attr, deque(old, maxlen=val))
        self._render_dirty = True

    # ================================================================ Theme
    def _on_theme_toggled(self, checked):
        self._dark_mode = checked
        self.lbl_theme_icon.setText("Dark" if checked else "Light")
        self._apply_theme()

    def _apply_theme(self):
        global C
        C = AppleColors.Dark if self._dark_mode else AppleColors.Light

        qss = _build_qss(C)
        app = QtWidgets.QApplication.instance()
        if app is not None:
            app.setStyleSheet(qss)
            # Keep tooltip rendering consistent across platforms/theme toggles.
            tip_pal = QPalette(QToolTip.palette())
            tip_pal.setColor(QPalette.ToolTipBase, QColor(28, 28, 30, 245))
            tip_pal.setColor(QPalette.ToolTipText, QColor("#f5f5f7"))
            QToolTip.setPalette(tip_pal)
            QToolTip.setFont(QFont("-apple-system", 12))
        self.setStyleSheet(qss)
        # Force one-pass style refresh for IMU badges after theme swap.
        self._imu_ok_state_cache = [None] * 6
        self._imu_batt_style_cache = [None] * 6

        # Update cards
        for card in self._cards:
            card.update_palette()

        # Update segmented controls
        for seg in self._segmented_ctrls:
            seg.update_palette()

        # Plots - color-matched axes
        for pw in [self.plot_left, self.plot_right]:
            pw.setBackground(C.plot_bg)
            # Bottom axis uses plot_fg
            pw.getPlotItem().getAxis('bottom').setPen(C.plot_fg)
            pw.getPlotItem().getAxis('bottom').setTextPen(C.plot_fg)
            # Left axis (Angle) = green
            pw.getPlotItem().getAxis('left').setPen(C.green)
            pw.getPlotItem().getAxis('left').setTextPen(C.green)
            pw.getPlotItem().getAxis('left').setLabel('Angle', units='deg', color=C.green)
            # Right axis (Torque) = blue
            pw.getPlotItem().getAxis('right').setPen(C.blue)
            pw.getPlotItem().getAxis('right').setTextPen(C.blue)
            pw.getPlotItem().getAxis('right').setLabel('Torque', units='Nm', color=C.blue)
            pw.getPlotItem().titleLabel.setText(
                pw.getPlotItem().titleLabel.text, color=C.plot_fg)
            # Grid
            pw.getPlotItem().getAxis('left').setGrid(100)
            pw.getPlotItem().getAxis('bottom').setGrid(100)

        # Velocity axes = teal
        for ax in [self._ax_vel_L, self._ax_vel_R]:
            ax.setPen(C.teal)
            ax.setTextPen(C.teal)

        # Power axes = purple
        for ax in [self._ax_pwr_L, self._ax_pwr_R]:
            ax.setPen(C.purple)
            ax.setTextPen(C.purple)

        # Compact power strips
        for strip in [self.pwr_strip_right, self.pwr_strip_left]:
            strip.setBackground(C.plot_bg)
            strip.getAxis('left').setPen(C.text2)
            strip.getAxis('left').setTextPen(C.text2)
            strip.getAxis('bottom').setPen(C.plot_fg)
            strip.getAxis('bottom').setTextPen(C.plot_fg)
        self._update_power_strip_titles(force=True)
        self.pwr_strip_right_zero.setPen(pg.mkPen(C.separator, width=1.2))
        self.pwr_strip_left_zero.setPen(pg.mkPen(C.separator, width=1.2))
        self.pwr_strip_right_pos.setPen(pg.mkPen(C.green, width=1.2))
        self.pwr_strip_left_pos.setPen(pg.mkPen(C.green, width=1.2))
        self.pwr_strip_right_neg.setPen(pg.mkPen(C.red, width=1.2))
        self.pwr_strip_left_neg.setPen(pg.mkPen(C.red, width=1.2))
        green_fill = QColor(C.green); green_fill.setAlpha(96)
        red_fill = QColor(C.red); red_fill.setAlpha(96)
        self.pwr_strip_right_fill_pos.setBrush(pg.mkBrush(green_fill))
        self.pwr_strip_left_fill_pos.setBrush(pg.mkBrush(green_fill))
        self.pwr_strip_right_fill_neg.setBrush(pg.mkBrush(red_fill))
        self.pwr_strip_left_fill_neg.setBrush(pg.mkBrush(red_fill))

        # Toggle colors
        self.sw_auto_scroll._on_color = C.blue
        self.sw_auto_scroll._off_color = C.fill
        self.sw_auto_scroll.update()
        self.sw_persist._on_color = C.blue
        self.sw_persist._off_color = C.fill
        self.sw_persist.update()
        self.sw_theme._off_color = C.fill
        self.sw_theme.update()

        # Ensure combo popup palette follows theme on all platforms.
        for combo in self._combo_boxes:
            view = combo.view()
            if view is None:
                continue
            pal = view.palette()
            # Keep dropdown popup readable on macOS: force light background + dark text.
            pal.setColor(QPalette.Base, QColor("#ffffff"))
            pal.setColor(QPalette.Text, QColor("#111111"))
            pal.setColor(QPalette.Highlight, QColor("#dfe8ff"))
            pal.setColor(QPalette.HighlightedText, QColor("#000000"))
            view.setPalette(pal)

        # Confirm button (accent)
        self.btn_algo_confirm.setStyleSheet(f"""
            QPushButton {{
                background-color: {C.blue};
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 14px;
                font-weight: 600;
                padding: 8px 16px;
            }}
            QPushButton:hover {{
                background-color: {C.blue};
                opacity: 0.85;
            }}
        """)

        # IMU Init / Motor Init
        self.btn_imu_init.setStyleSheet(f"""
            QPushButton {{
                background-color: {C.blue};
                color: white; border: none; border-radius: 8px;
                font-weight: 600; font-size: 12px; padding: 4px 10px;
            }}
        """)
        self.btn_motor_init.setStyleSheet(f"""
            QPushButton {{
                background-color: {C.orange};
                color: white; border: none; border-radius: 8px;
                font-weight: 600; font-size: 12px; padding: 4px 10px;
            }}
        """)
        self.btn_brand_apply.setStyleSheet(f"""
            QPushButton {{
                background-color: {C.blue};
                color: white; border: none; border-radius: 8px;
                font-weight: 600; font-size: 12px; padding: 4px 10px;
            }}
        """)
        self.btn_refresh_ports.setStyleSheet(f"""
            QPushButton {{
                background-color: {C.fill};
                color: {C.text};
                border: none;
                border-radius: 8px;
                font-weight: 600;
                font-size: 12px;
                padding: 4px 10px;
            }}
            QPushButton:hover {{
                background-color: {C.separator};
            }}
        """)
        if hasattr(self, "btn_pi_profile_cfg"):
            self.btn_pi_profile_cfg.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C.fill};
                    color: {C.text};
                    border: none;
                    border-radius: 8px;
                    font-weight: 600;
                    font-size: 12px;
                    padding: 4px 10px;
                }}
                QPushButton:hover {{ background-color: {C.separator}; }}
            """)
        if hasattr(self, "btn_pi_rl_start_legdcp"):
            self.btn_pi_rl_start_legdcp.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C.fill};
                    color: {C.text};
                    border: none;
                    border-radius: 8px;
                    font-weight: 600;
                    font-size: 12px;
                    padding: 4px 10px;
                }}
                QPushButton:hover {{ background-color: {C.separator}; }}
            """)
        if hasattr(self, "btn_pi_rl_start_pd"):
            self.btn_pi_rl_start_pd.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C.fill};
                    color: {C.text};
                    border: none;
                    border-radius: 8px;
                    font-weight: 600;
                    font-size: 12px;
                    padding: 4px 10px;
                }}
                QPushButton:hover {{ background-color: {C.separator}; }}
            """)
        if hasattr(self, "btn_pi_rl_stop"):
            self.btn_pi_rl_stop.setStyleSheet(f"""
                QPushButton {{
                    background-color: {C.red};
                    color: white;
                    border: none;
                    border-radius: 8px;
                    font-weight: 600;
                    font-size: 12px;
                    padding: 4px 10px;
                }}
                QPushButton:hover {{ background-color: #e02a20; }}
            """)
        if hasattr(self, "lbl_pi_rl_remote_state"):
            txt = self.lbl_pi_rl_remote_state.text()
            self._pi_rl_remote_status_sig = None
            self._set_pi_rl_remote_state(txt, self._infer_pi_rl_remote_color(txt), force=True)
        if hasattr(self, "note_raw_data"):
            self.note_raw_data.setStyleSheet(
                f"background:{C.fill}; border:1px solid {C.teal}; border-radius:4px;")
        if hasattr(self, "lbl_rl_filter_state"):
            self.lbl_rl_filter_state.setStyleSheet(
                f"color:{C.text2}; font-size:11px; background:transparent; padding-top:2px;"
            )
            self._update_rl_filter_state_label()
        if hasattr(self, "lbl_imu_batt_title"):
            self.lbl_imu_batt_title.setStyleSheet(
                f"color:{C.text2}; font-size:11px; font-weight:600; background:transparent;"
            )

        # Section labels
        for lbl in _section_labels:
            lbl.setStyleSheet(f"""
                color: {C.text2}; font-size: 12px; font-weight: 500;
                text-transform: uppercase; padding: 4px 0px; background: transparent;
            """)

        # Status bar labels (bottom of plots)
        self.lbl_imu.setStyleSheet(f"font-size:13px; font-weight:400; background:transparent; color:{C.text2};")
        self.lbl_maxt.setStyleSheet(f"font-size:13px; font-weight:400; background:transparent; color:{C.text2};")
        self.lbl_tag_state.setStyleSheet(f"font-size:13px; font-weight:600; background:transparent; color:{C.text};")
        self.lbl_tag_timer.setStyleSheet(f"font-size:13px; font-weight:400; background:transparent; color:{C.text2};")
        self.lbl_tag_rx.setStyleSheet(f"font-size:13px; font-weight:400; background:transparent; color:{C.text2};")
        self.lbl_tag_gait.setStyleSheet(f"font-size:13px; font-weight:400; background:transparent; color:{C.text2};")
        self.lbl_replay_progress_cur.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent; min-width:56px;")
        self.lbl_replay_progress_total.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent; min-width:64px;")
        self.lbl_replay_progress_rows.setStyleSheet(
            f"font-size:11px; color:{C.text2}; background:transparent;")
        self.sld_replay_progress.setStyleSheet(f"""
            QSlider::groove:horizontal {{
                height: 6px;
                background: {C.fill};
                border-radius: 3px;
            }}
            QSlider::sub-page:horizontal {{
                background: {C.blue};
                border-radius: 3px;
            }}
            QSlider::handle:horizontal {{
                width: 12px;
                margin: -4px 0;
                background: {C.text};
                border-radius: 6px;
            }}
        """)
        self._set_replay_load_btn_active(self._replay_mode)

        # Re-apply power style
        self._set_power_ui(self.btn_power.isChecked())
        self._update_dir_btns()
        self._update_brand_apply_label()

    # ================================================================ Capture
    def _apply_record_idle_style(self):
        self.btn_record.setStyleSheet(f"""
            QPushButton {{
                font-size:11px; padding:2px 10px; font-weight:600;
                background-color:{C.purple}; color:white;
                border:none; border-radius:8px;
            }}
            QPushButton:hover {{ background-color:#9f42ce; }}
        """)

    def _take_screenshot(self):
        os.makedirs(self._capture_dir, exist_ok=True)
        fname = datetime.now().strftime("screenshot_%Y%m%d_%H%M%S.png")
        path = os.path.join(self._capture_dir, fname)
        pixmap = self.grab()
        pixmap.save(path, "PNG")
        # Flash feedback on button
        self.btn_screenshot.setText("Saved!")
        self.btn_screenshot.setStyleSheet(f"""
            QPushButton {{
                font-size:11px; padding:2px 10px; font-weight:600;
                background-color:{C.green}; color:white;
                border:none; border-radius:8px;
            }}
        """)
        self.lbl_status.setText(f"Screenshot saved: {fname}")
        QTimer.singleShot(1500, self._reset_screenshot_btn)

    def _reset_screenshot_btn(self):
        self.btn_screenshot.setText("Screenshot")
        self.btn_screenshot.setStyleSheet(f"""
            QPushButton {{
                font-size:11px; padding:2px 10px; font-weight:600;
                background-color:{C.teal}; color:white;
                border:none; border-radius:8px;
            }}
            QPushButton:hover {{ background-color:#4ab8ea; }}
        """)

    def _toggle_recording(self):
        if self._recording:
            self._stop_recording()
        else:
            self._start_recording()

    def _resolve_ffmpeg_path(self):
        exe_name = "ffmpeg.exe" if os.name == "nt" else "ffmpeg"
        candidates = []

        # 1) Explicit override.
        env_path = os.environ.get("EXO_FFMPEG", "").strip()
        if env_path:
            candidates.append(env_path)

        # 2) Packaged app locations (PyInstaller one-folder / macOS .app).
        if getattr(sys, "frozen", False):
            exe_dir = os.path.dirname(sys.executable)
            candidates.append(os.path.join(exe_dir, "bin", exe_name))
            candidates.append(os.path.join(exe_dir, exe_name))
            meipass = getattr(sys, "_MEIPASS", "")
            if meipass:
                candidates.append(os.path.join(meipass, "bin", exe_name))
                candidates.append(os.path.join(meipass, exe_name))

        # 3) Development tree / common system locations.
        base_dir = os.path.dirname(os.path.abspath(__file__))
        candidates.append(os.path.join(base_dir, "bin", exe_name))
        found = shutil.which("ffmpeg")
        if found:
            candidates.append(found)
        if os.name != "nt":
            candidates.extend(["/opt/homebrew/bin/ffmpeg", "/usr/local/bin/ffmpeg", "/usr/bin/ffmpeg"])
        try:
            import imageio_ffmpeg  # type: ignore
            candidates.append(imageio_ffmpeg.get_ffmpeg_exe())
        except Exception:
            pass

        seen = set()
        for p in candidates:
            if not p:
                continue
            norm = os.path.abspath(p)
            if norm in seen:
                continue
            seen.add(norm)
            if os.path.isfile(norm) and os.access(norm, os.X_OK):
                return norm
        return None

    def _record_writer_loop(self, target_dir, ext, jpg_quality):
        """Background writer: consume QImage frames and persist to disk."""
        idx = 0
        q = self._record_queue
        if q is None:
            return
        while True:
            item = q.get()
            if item is None:
                q.task_done()
                break
            out_path = os.path.join(target_dir, f"frame_{idx:06d}.{ext}")
            ok = False
            try:
                if ext.lower() in ("jpg", "jpeg"):
                    ok = bool(item.save(out_path, "JPG", int(jpg_quality)))
                else:
                    ok = bool(item.save(out_path))
            except Exception:
                ok = False
            if ok:
                idx += 1
                self._record_saved_count = idx
            else:
                self._record_drop_count += 1
            q.task_done()

    def _enqueue_record_frame(self, image):
        q = self._record_queue
        if q is None:
            return False
        try:
            q.put_nowait(image)
            return True
        except queue.Full:
            # Keep UI responsive: drop one queued old frame, keep newest frame.
            try:
                _old = q.get_nowait()
                q.task_done()
                self._record_drop_count += 1
            except queue.Empty:
                pass
            try:
                q.put_nowait(image)
                return True
            except queue.Full:
                self._record_drop_count += 1
                return False

    def _start_recording(self):
        os.makedirs(self._capture_dir, exist_ok=True)
        self._record_tmp_dir = tempfile.mkdtemp(prefix="gui_rec_")
        self._record_frame_idx = 0
        self._record_drop_count = 0
        self._record_saved_count = 0
        self._recording = True
        self._record_start_time = time.time()
        self._record_status_last_ts = 0.0
        self._record_queue = queue.Queue(maxsize=int(self._record_queue_max))
        self._record_writer_thread = threading.Thread(
            target=self._record_writer_loop,
            args=(self._record_tmp_dir, self._record_ext, int(self._record_jpg_quality)),
            name="hip-gui-record-writer",
            daemon=True,
        )
        self._record_writer_thread.start()
        self.btn_record.setText("Stop")
        self.btn_record.setStyleSheet(f"""
            QPushButton {{
                font-size:11px; padding:2px 10px; font-weight:600;
                background-color:{C.red}; color:white;
                border:none; border-radius:8px;
            }}
            QPushButton:hover {{ background-color:#e02a20; }}
        """)
        self._record_timer = QTimer(self)
        self._record_timer.setInterval(max(1, int(round(1000.0 / float(self._record_fps)))))
        self._record_timer.timeout.connect(self._capture_frame)
        self._record_timer.start()
        self.lbl_status.setText("Recording...")

    def _capture_frame(self):
        if not self._recording or not self._record_tmp_dir:
            return
        image = self.grab().toImage().copy()
        self._enqueue_record_frame(image)
        self._record_frame_idx += 1
        now = time.time()
        if now - self._record_status_last_ts >= 0.2:
            elapsed = now - self._record_start_time
            mm = int(elapsed // 60)
            ss = elapsed - mm * 60
            self.lbl_status.setText(
                f"Recording... {mm:02d}:{ss:04.1f}  "
                f"(cap:{self._record_frame_idx} save:{self._record_saved_count} drop:{self._record_drop_count})")
            self._record_status_last_ts = now

    def _stop_recording(self):
        self._recording = False
        if self._record_timer:
            self._record_timer.stop()
            self._record_timer = None
        self.btn_record.setText("Record")
        self._apply_record_idle_style()

        # Drain writer queue before encode so frame sequence is complete.
        if self._record_queue is not None:
            pushed_stop = False
            while not pushed_stop:
                try:
                    self._record_queue.put(None, timeout=0.05)
                    pushed_stop = True
                except queue.Full:
                    QtWidgets.QApplication.processEvents(QEventLoop.AllEvents, 10)
                    time.sleep(0.01)
            deadline = time.time() + 8.0
            while getattr(self._record_queue, "unfinished_tasks", 0) > 0 and time.time() < deadline:
                QtWidgets.QApplication.processEvents(QEventLoop.AllEvents, 10)
                time.sleep(0.01)
        if self._record_writer_thread and self._record_writer_thread.is_alive():
            self._record_writer_thread.join(timeout=1.5)
        self._record_writer_thread = None
        self._record_queue = None

        if self._record_saved_count == 0:
            if self._record_tmp_dir:
                shutil.rmtree(self._record_tmp_dir, ignore_errors=True)
            self.lbl_status.setText("Recording cancelled (no frames)")
            self._record_tmp_dir = None
            return

        ts = datetime.now().strftime("recording_%Y%m%d_%H%M%S")
        mp4_path = os.path.join(self._capture_dir, ts + ".mp4")
        frame_pattern = os.path.join(self._record_tmp_dir, f"frame_%06d.{self._record_ext}")
        elapsed = time.time() - self._record_start_time
        dur_str = f"{int(elapsed // 60):02d}:{elapsed % 60:04.1f}"
        self._ffmpeg_path = self._resolve_ffmpeg_path()
        saved = int(self._record_saved_count)
        captured = int(self._record_frame_idx)
        dropped = int(self._record_drop_count)

        try:
            subprocess.run(
                [self._ffmpeg_path, "-y", "-hide_banner", "-loglevel", "error",
                 "-framerate", str(self._record_fps),
                 "-i", frame_pattern,
                 "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                 "-c:v", "libx264", "-preset", "ultrafast", "-pix_fmt", "yuv420p",
                 "-crf", "18", mp4_path],
                check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=180)
            shutil.rmtree(self._record_tmp_dir, ignore_errors=True)
            msg = (f"Recording saved!\n\nFile: {ts}.mp4\nDuration: {dur_str}\n"
                   f"Captured: {captured}\nSaved: {saved}\nDropped: {dropped}")
            self.lbl_status.setText(f"Recording saved: {ts}.mp4 ({dur_str}, save:{saved}, drop:{dropped})")
            # Brief green flash on button
            self.btn_record.setText("Saved!")
            self.btn_record.setStyleSheet(f"""
                QPushButton {{
                    font-size:11px; padding:2px 10px; font-weight:600;
                    background-color:{C.green}; color:white;
                    border:none; border-radius:8px;
                }}
            """)
            QTimer.singleShot(2000, self._reset_record_btn)
            QMessageBox.information(self, "Recording Saved", msg)
        except (FileNotFoundError, TypeError, subprocess.SubprocessError):
            fallback = os.path.join(self._capture_dir, ts)
            shutil.move(self._record_tmp_dir, fallback)
            msg = (f"ffmpeg not found, frames saved as images.\n\n"
                   f"Folder: {ts}/\nCaptured: {captured}\nSaved: {saved}\nDropped: {dropped}\nDuration: {dur_str}")
            self.lbl_status.setText(f"Frames saved: {ts}/ (save:{saved}, drop:{dropped})")
            QMessageBox.warning(self, "Recording Saved (frames)", msg)

        self._record_tmp_dir = None

    def _reset_record_btn(self):
        self.btn_record.setText("Record")
        self._apply_record_idle_style()

    # ================================================================ Replay
    def _load_replay_mapping(self):
        path = getattr(self, "_replay_mapping_path", "")
        if not path or not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                return {}
            clean = {}
            for k, v in data.items():
                if not isinstance(k, str) or not k:
                    continue
                if isinstance(v, str):
                    vv = v.strip()
                    if not vv:
                        continue
                    # Legacy: previously allowed "auto compute (vel*cmd)".
                    # Current policy keeps replay power as logged value only.
                    if k in REPLAY_POWER_COLUMNS and vv == REPLAY_AUTO_COMPUTE:
                        continue
                    clean[k] = [vv]
                    continue
                if isinstance(v, list):
                    candidates = []
                    for item in v:
                        if not isinstance(item, str):
                            continue
                        col = item.strip()
                        if not col:
                            continue
                        if k in REPLAY_POWER_COLUMNS and col == REPLAY_AUTO_COMPUTE:
                            continue
                        if col not in candidates:
                            candidates.append(col)
                    if candidates:
                        clean[k] = candidates
            return clean
        except (OSError, json.JSONDecodeError):
            return {}

    def _save_replay_mapping(self):
        serializable = {}
        for key in REPLAY_PLOT_COLUMN_ALIASES.keys():
            cands = self._get_replay_mapping_candidates(key)
            if isinstance(cands, list) and cands:
                serializable[key] = cands
        try:
            with open(self._replay_mapping_path, "w", encoding="utf-8") as f:
                json.dump(serializable, f, ensure_ascii=False, indent=2)
        except OSError as e:
            QMessageBox.warning(self, "Replay", f"映射保存失败:\n{e}")

    def _get_replay_mapping_candidates(self, key):
        mapped = self._replay_col_mapping.get(key, [])
        if isinstance(mapped, str):
            mapped = [mapped]
        if not isinstance(mapped, list):
            return []
        out = []
        for item in mapped:
            if not isinstance(item, str):
                continue
            col = item.strip()
            if not col:
                continue
            if key in REPLAY_POWER_COLUMNS and col == REPLAY_AUTO_COMPUTE:
                continue
            if col not in out:
                out.append(col)
        return out

    def _merge_replay_mapping_choice(self, key, mapped_value):
        if not isinstance(mapped_value, str):
            return
        col = mapped_value.strip()
        if not col:
            return
        existing = self._get_replay_mapping_candidates(key)
        existing = [x for x in existing if x != col]
        existing.insert(0, col)
        self._replay_col_mapping[key] = existing

    def _prompt_replay_column_mapping(self, headers, missing_keys):
        if not headers:
            return None

        dlg = QDialog(self)
        dlg.setWindowTitle("CSV 关键词映射")
        dlg.setModal(True)
        dlg.resize(560, 320)

        lay = QVBoxLayout(dlg)
        tip = QLabel(
            "检测到以下绘图关键词缺失，请选择 CSV 中对应列。\n"
            "功率列若缺失将按 0W 显示（不做本地估算）。\n\n"
            + ", ".join(missing_keys)
        )
        tip.setWordWrap(True)
        lay.addWidget(tip)

        form = QFormLayout()
        combos = {}
        for key in missing_keys:
            combo = QComboBox()
            for h in headers:
                combo.addItem(h, h)
            saved = self._replay_col_mapping.get(key, "")
            candidates = self._get_replay_mapping_candidates(key)
            if isinstance(candidates, list):
                for cand in candidates:
                    idx = combo.findData(cand)
                    if idx >= 0:
                        combo.setCurrentIndex(idx)
                        break
            form.addRow(f"{key} ->", combo)
            combos[key] = combo
        lay.addLayout(form)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        lay.addWidget(btns)

        if dlg.exec_() != QDialog.Accepted:
            return None

        resolved = {}
        for key, combo in combos.items():
            col = combo.currentData()
            if not isinstance(col, str) or not col.strip():
                QMessageBox.warning(self, "Replay", f"{key} 未选择映射列。")
                return None
            resolved[key] = col.strip()
        return resolved

    def _resolve_replay_plot_columns(self, fieldnames):
        headers = [h for h in (fieldnames or []) if isinstance(h, str) and h]
        if not headers:
            raise ValueError("CSV header missing")

        resolved = {}
        missing = []
        for key, aliases in REPLAY_PLOT_COLUMN_ALIASES.items():
            candidates = self._get_replay_mapping_candidates(key)
            if isinstance(candidates, list):
                hit = next((c for c in candidates if c in headers), None)
                if hit is not None:
                    resolved[key] = hit
                    continue
            found = None
            for alias in aliases:
                if alias in headers:
                    found = alias
                    break
            if found is not None:
                resolved[key] = found
            else:
                if key in REPLAY_OPTIONAL_COLUMNS:
                    resolved[key] = None
                else:
                    missing.append(key)

        if missing:
            user_map = self._prompt_replay_column_mapping(headers, missing)
            if user_map is None:
                raise ValueError("CSV 关键词映射已取消")
            for key, mapped in user_map.items():
                self._merge_replay_mapping_choice(key, mapped)
                resolved[key] = mapped
            self._save_replay_mapping()

        return resolved

    def _resolve_replay_time_key(self, headers):
        for key in REPLAY_TIME_CANDIDATES:
            if key in headers:
                return key
        return None

    def _format_replay_time(self, t_s):
        t = max(0.0, float(t_s))
        h = int(t // 3600.0)
        m = int((t % 3600.0) // 60.0)
        s = t - (h * 3600.0 + m * 60.0)
        if h > 0:
            return f"{h:d}:{m:02d}:{s:04.1f}"
        return f"{m:02d}:{s:04.1f}"

    def _set_replay_load_btn_active(self, active: bool):
        if active:
            self.btn_replay_load.setStyleSheet(f"""
                QPushButton {{
                    font-size:11px; padding:2px 8px; font-weight:700;
                    color:white; background-color:{C.green};
                    border:none; border-radius:8px;
                }}
                QPushButton:hover {{ background-color:#25b44d; }}
            """)
        else:
            self.btn_replay_load.setStyleSheet(f"""
                QPushButton {{
                    font-size:11px; padding:2px 8px; font-weight:600;
                    color:{C.text}; background-color:{C.fill};
                    border:none; border-radius:8px;
                }}
                QPushButton:hover {{ background-color:{C.separator}; }}
            """)

    def _set_replay_controls_active(self, active: bool):
        self.btn_replay_pause.setEnabled(active)
        self.btn_replay_rw.setEnabled(active)
        self.btn_replay_ff.setEnabled(active)
        self.btn_replay_stop.setEnabled(active)
        self.sld_replay_progress.setEnabled(active)
        if not active:
            self.btn_replay_pause.blockSignals(True)
            self.btn_replay_pause.setChecked(False)
            self.btn_replay_pause.blockSignals(False)
            self.btn_replay_pause.setText("Pause")
        self._set_replay_load_btn_active(bool(active))
        self._update_replay_progress_ui(force=True)

    def _set_replay_finished(self, finished: bool):
        self._replay_finished = bool(finished)
        if self._replay_finished:
            self._replay_paused = True
            self.btn_replay_pause.blockSignals(True)
            self.btn_replay_pause.setChecked(True)
            self.btn_replay_pause.blockSignals(False)
            self.btn_replay_pause.setText("Ended")
        else:
            self.btn_replay_pause.setText("Resume" if self._replay_paused else "Pause")

    def _update_replay_progress_ui(self, force=False):
        if not hasattr(self, "sld_replay_progress"):
            return
        if not self._replay_mode or not self._replay_samples:
            self.lbl_replay_progress_cur.setText("--:--")
            self.lbl_replay_progress_total.setText("/ --:--")
            self.lbl_replay_progress_rows.setText("[--/--]")
            if force or (not self._replay_slider_dragging):
                self._replay_slider_internal = True
                self.sld_replay_progress.setValue(0)
                self._replay_slider_internal = False
            return

        total = len(self._replay_samples)
        t_end = float(self._replay_time_axis[-1]) if self._replay_time_axis else float(self._replay_samples[-1][0])
        t_cur = max(0.0, min(float(self._replay_play_t), t_end if t_end > 0.0 else float(self._replay_play_t)))
        idx_show = min(total, max(0, int(self._replay_idx)))

        self.lbl_replay_progress_cur.setText(self._format_replay_time(t_cur))
        self.lbl_replay_progress_total.setText(f"/ {self._format_replay_time(t_end)}")
        self.lbl_replay_progress_rows.setText(f"[{idx_show}/{total}]")

        if (not self._replay_slider_dragging) or force:
            pos = 0
            if t_end > 1e-9:
                pos = int(round((t_cur / t_end) * REPLAY_PROGRESS_STEPS))
            pos = max(0, min(REPLAY_PROGRESS_STEPS, pos))
            self._replay_slider_internal = True
            self.sld_replay_progress.setValue(pos)
            self._replay_slider_internal = False

    def _on_replay_slider_pressed(self):
        self._replay_slider_dragging = True

    def _on_replay_slider_released(self):
        self._replay_slider_dragging = False
        self._update_replay_progress_ui(force=True)

    def _on_replay_slider_moved(self, value):
        if self._replay_slider_internal:
            return
        if not self._replay_mode or not self._replay_samples:
            return
        t_end = float(self._replay_time_axis[-1]) if self._replay_time_axis else float(self._replay_samples[-1][0])
        if t_end <= 1e-9:
            target_t = 0.0
        else:
            target_t = (max(0, min(REPLAY_PROGRESS_STEPS, int(value))) / float(REPLAY_PROGRESS_STEPS)) * t_end
        self._seek_replay_time(target_t)
        self._update_replay_progress_ui(force=True)

    def _set_replay_status(self, text, force=False):
        now = time.time()
        if force or (now - self._replay_status_last_ts) >= self._replay_status_min_interval_s:
            self.lbl_status.setText(str(text))
            self._replay_status_last_ts = now

    def _row_float(self, row, keys, default=0.0, allow_none=False):
        for key in keys:
            if key not in row:
                continue
            val = row.get(key)
            if val is None:
                continue
            txt = str(val).strip()
            if txt == "":
                continue
            try:
                return float(txt)
            except ValueError:
                continue
        if allow_none:
            return None
        return float(default)

    def _load_replay_samples(self, csv_path):
        samples = []
        with open(csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError("CSV header missing")
            headers = list(reader.fieldnames)
            plot_cols = self._resolve_replay_plot_columns(headers)
            time_key = self._resolve_replay_time_key(headers)
            time_key_l = str(time_key or "").lower()
            prev_raw_t = None
            replay_t = 0.0
            for row in reader:
                raw_t_native = None
                if time_key:
                    raw_t_native = self._row_float(row, [time_key], allow_none=True)

                raw_t = None
                if raw_t_native is not None:
                    if time_key_l in ("teensy_t_cs_u16", "t_cs"):
                        raw_t = float(raw_t_native) * 0.01
                    elif time_key_l in ("teensy_t_ms_unwrapped",):
                        raw_t = float(raw_t_native) * 0.001
                    else:
                        raw_t = float(raw_t_native)

                if raw_t is None:
                    raw_t = (prev_raw_t + 0.02) if prev_raw_t is not None else 0.0
                if prev_raw_t is not None:
                    dt = raw_t - prev_raw_t
                    if dt < -100.0:
                        dt += 655.36
                    if dt <= 0.0 or dt > 1.0:
                        dt = 0.02
                    replay_t += dt
                prev_raw_t = raw_t

                L_angle = self._row_float(row, [plot_cols['L_angle_deg']], 0.0)
                R_angle = self._row_float(row, [plot_cols['R_angle_deg']], 0.0)
                L_tau_d = self._row_float(row, [plot_cols['L_cmd_Nm']], 0.0)
                R_tau_d = self._row_float(row, [plot_cols['R_cmd_Nm']], 0.0)
                L_tau = self._row_float(row, [plot_cols['L_est_Nm']], 0.0)
                R_tau = self._row_float(row, [plot_cols['R_est_Nm']], 0.0)
                L_vel = self._row_float(row, [plot_cols['L_vel_dps']], 0.0) if plot_cols.get('L_vel_dps') else 0.0
                R_vel = self._row_float(row, [plot_cols['R_vel_dps']], 0.0) if plot_cols.get('R_vel_dps') else 0.0
                L_pwr = nan
                R_pwr = nan
                if plot_cols.get('L_pwr_W'):
                    L_pwr = self._row_float(row, [plot_cols['L_pwr_W']], nan)
                if plot_cols.get('R_pwr_W'):
                    R_pwr = self._row_float(row, [plot_cols['R_pwr_W']], nan)
                gait = self._row_float(row, ['gait_freq_Hz', 'gait_hz', 'gait_freq'], 0.0)

                samples.append((
                    replay_t, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d,
                    L_vel, R_vel, L_pwr, R_pwr, gait,
                ))
        return samples

    def _start_replay(self, samples, csv_path):
        self._replay_samples = samples
        self._replay_time_axis = [float(s[0]) for s in samples]
        self._replay_csv_path = csv_path
        self._replay_mode = True
        self._replay_paused = False
        self._replay_finished = False
        self._replay_idx = 0
        self._replay_play_t = 0.0
        self._replay_last_wall_time = time.time()
        self._replay_status_last_ts = 0.0
        self._clear_buffers()
        self._set_replay_controls_active(True)
        self._set_replay_finished(False)
        self.btn_replay_pause.blockSignals(True)
        self.btn_replay_pause.setChecked(False)
        self.btn_replay_pause.blockSignals(False)
        self.btn_replay_pause.setText("Pause")
        self.lbl_imu.setText("IMU: REPLAY")
        self.lbl_maxt.setText("MaxT: REPLAY")
        self._set_replay_status(
            f"Replay loaded: {os.path.basename(csv_path)} ({len(samples)} rows)",
            force=True,
        )
        self._consume_replay_samples(max_steps=1, budget_ms=1.0)
        self._update_replay_progress_ui(force=True)
        self._maybe_render_plot(force=True)

    def _stop_replay(self, finished=False):
        had_path = bool(self._replay_csv_path)
        csv_name = os.path.basename(self._replay_csv_path) if had_path else ""
        self._replay_mode = False
        self._replay_paused = False
        self._replay_samples = []
        self._replay_time_axis = []
        self._replay_idx = 0
        self._replay_play_t = 0.0
        self._replay_last_wall_time = 0.0
        self._replay_finished = False
        self._replay_slider_dragging = False
        self._set_replay_controls_active(False)
        if finished and csv_name:
            self.lbl_status.setText(f"Replay finished: {csv_name}")
        elif had_path:
            self.lbl_status.setText("Replay stopped")

    def _on_load_replay_csv(self):
        start_dir = os.path.dirname(self._replay_csv_path) if self._replay_csv_path else \
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Replay CSV", start_dir, "CSV Files (*.csv)")
        if not path:
            return
        try:
            samples = self._load_replay_samples(path)
        except (OSError, ValueError) as e:
            if isinstance(e, ValueError) and "已取消" in str(e):
                self.lbl_status.setText("Replay load cancelled.")
                return
            QMessageBox.warning(self, "Replay", f"Failed to load CSV:\n{e}")
            return
        if len(samples) < 2:
            QMessageBox.warning(self, "Replay", "CSV data is too short to replay.")
            return
        self._start_replay(samples, path)

    def _on_replay_pause_toggled(self, checked):
        if not self._replay_mode:
            self.btn_replay_pause.blockSignals(True)
            self.btn_replay_pause.setChecked(False)
            self.btn_replay_pause.blockSignals(False)
            self.btn_replay_pause.setText("Pause")
            return
        if self._replay_finished and (not checked):
            # Keep replay in ended-hold state until user seeks/rewinds or presses Stop.
            self.btn_replay_pause.blockSignals(True)
            self.btn_replay_pause.setChecked(True)
            self.btn_replay_pause.blockSignals(False)
            self.btn_replay_pause.setText("Ended")
            self._set_replay_status("Replay finished. Drag timeline / rewind or press Stop.", force=True)
            return
        self._replay_paused = bool(checked)
        self.btn_replay_pause.setText("Resume" if self._replay_paused else "Pause")
        self._replay_last_wall_time = time.time()
        if self._replay_paused:
            self._set_replay_status(
                f"Replay paused: {self._replay_idx}/{len(self._replay_samples)}",
                force=True,
            )
        self._update_replay_progress_ui(force=True)

    def _on_replay_speed_changed(self, _index):
        text = self.cmb_replay_speed.currentText().strip().lower().replace('x', '')
        try:
            speed = float(text)
        except ValueError:
            speed = 1.0
        self._replay_speed = max(0.1, min(16.0, speed))
        self._replay_last_wall_time = time.time()

    def _seek_replay_time(self, target_t: float):
        if not self._replay_mode or not self._replay_samples:
            return
        total = len(self._replay_samples)
        if not self._replay_time_axis:
            self._replay_time_axis = [float(s[0]) for s in self._replay_samples]
        t_end = float(self._replay_time_axis[-1]) if self._replay_time_axis else 0.0
        self._replay_play_t = max(0.0, min(t_end, float(target_t)))
        target_idx = bisect.bisect_right(self._replay_time_axis, self._replay_play_t) - 1

        self._clear_buffers()
        if target_idx < 0:
            self._replay_idx = 0
            self._set_replay_finished(False)
            self._replay_last_wall_time = time.time()
            self._set_replay_status(
                f"Replay seek to t={self._replay_play_t:.2f}s ({self._replay_speed:.1f}x)",
                force=True,
            )
            self._update_replay_progress_ui(force=True)
            self._maybe_render_plot(force=True)
            return

        start_idx = max(0, target_idx - self.win_size + 1)
        for i in range(start_idx, target_idx + 1):
            self._apply_replay_sample(self._replay_samples[i], i, total)
        self._replay_idx = target_idx + 1
        self._set_replay_finished(self._replay_idx >= total)
        self._replay_last_wall_time = time.time()
        self._set_replay_status(
            f"Replay seek to t={self._replay_play_t:.2f}s ({self._replay_speed:.1f}x)",
            force=True,
        )
        self._update_replay_progress_ui(force=True)
        self._maybe_render_plot(force=True)

    def _on_replay_rewind(self):
        if not self._replay_mode or not self._replay_samples:
            return
        target = max(0.0, self._replay_play_t - 5.0)
        self._seek_replay_time(target)
        if self._replay_mode:
            self._set_replay_status(
                f"Replay rewind to t={self._replay_play_t:.2f}s "
                f"({self._replay_speed:.1f}x)",
                force=True,
            )
        self._update_replay_progress_ui(force=True)

    def _on_replay_fast_forward(self):
        if not self._replay_mode or not self._replay_samples:
            return
        self._replay_play_t = min(self._replay_samples[-1][0], self._replay_play_t + 5.0)
        self._consume_replay_samples(budget_ms=8.0)
        self._maybe_render_plot(force=True)
        if self._replay_mode:
            self._set_replay_status(
                f"Replay fast-forward to t={self._replay_play_t:.2f}s "
                f"({self._replay_speed:.1f}x)",
                force=True,
            )
        self._update_replay_progress_ui(force=True)

    def _on_replay_stop_clicked(self):
        if not self._replay_mode:
            return
        self._stop_replay(finished=False)

    def _consume_replay_samples(self, max_steps=None, budget_ms=None):
        if not self._replay_mode or not self._replay_samples:
            return 0, -1, 0.0, 0
        if budget_ms is None:
            budget_ms = self._replay_budget_ms
        budget_s = max(0.0005, float(budget_ms) * 1e-3)
        total = len(self._replay_samples)
        steps = 0
        last_idx = -1
        last_t = 0.0
        t_start = time.perf_counter()
        while self._replay_idx < total and self._replay_samples[self._replay_idx][0] <= self._replay_play_t:
            sample = self._replay_samples[self._replay_idx]
            self._apply_replay_sample(sample, self._replay_idx, total)
            last_idx = self._replay_idx
            last_t = float(sample[0])
            self._replay_idx += 1
            steps += 1
            if max_steps is not None and steps >= int(max_steps):
                break
            if (time.perf_counter() - t_start) >= budget_s:
                break

        if steps > 0 and last_idx >= 0:
            self._set_replay_status(
                f"Replay {last_idx + 1}/{total}  t={last_t:.2f}s  speed={self._replay_speed:.1f}x",
                force=False,
            )
        if self._replay_idx >= total:
            t_end = float(self._replay_time_axis[-1]) if self._replay_time_axis else float(self._replay_samples[-1][0])
            self._replay_play_t = t_end
            self._set_replay_finished(True)
            self._set_replay_status(
                f"Replay finished at {self._format_replay_time(t_end)}. Press Stop to exit replay mode.",
                force=True,
            )
        else:
            self._set_replay_finished(False)
        self._update_replay_progress_ui(force=False)
        return steps, last_idx, last_t, total

    def _update_replay(self, now):
        if not self._replay_mode:
            return
        if self.connected and self.ser:
            try:
                avail = self.ser.in_waiting
                if avail > 0:
                    self.ser.read(avail)
            except (serial.SerialException, OSError):
                pass
        if self._replay_paused:
            self._replay_last_wall_time = now
            self._update_replay_progress_ui(force=False)
            return
        dt = now - self._replay_last_wall_time if self._replay_last_wall_time > 0 else 0.02
        self._replay_last_wall_time = now
        if dt < 0.0:
            dt = 0.02
        t_end = float(self._replay_time_axis[-1]) if self._replay_time_axis else float(self._replay_samples[-1][0])
        self._replay_play_t = min(t_end, self._replay_play_t + dt * self._replay_speed)
        self._consume_replay_samples()
        self._update_replay_progress_ui(force=False)
        self._maybe_render_plot(now)

    def _apply_replay_sample(self, sample, _idx, _total):
        (t, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d,
         L_vel, R_vel, L_pwr_csv, R_pwr_csv, gait_freq) = sample
        L_pwr = float(L_pwr_csv) if isfinite(L_pwr_csv) else 0.0
        R_pwr = float(R_pwr_csv) if isfinite(R_pwr_csv) else 0.0
        self._last_gait_freq = gait_freq
        self._append_data_point(
            t, L_angle, R_angle, L_tau, R_tau, L_tau_d, R_tau_d, L_vel, R_vel,
            gait_freq, log_csv=False, power_override=(L_pwr, R_pwr))

    # ================================================================ Plots
    def _build_plots(self):
        # Add Display card to right side (above plots)
        self.plot_layout.addWidget(self._plot_card)

        pen_angle = pg.mkPen('#34c759', width=2)
        pen_cmd   = pg.mkPen('#007aff', width=2)
        pen_est   = pg.mkPen('#ff3b30', width=2)
        pen_vel   = pg.mkPen('#5ac8fa', width=2)
        pen_pwr   = pg.mkPen('#af52de', width=2)

        # ---- Right Leg ----
        self.plot_left = pg.PlotWidget(background=C.plot_bg)
        self.plot_left.setTitle("Right Leg", color=C.plot_fg, size='13pt')
        self.plot_left.showGrid(x=True, y=True, alpha=0.15)
        self.plot_left.getAxis('left').setLabel('Angle', units='deg', color='#34c759')
        self.plot_left.getAxis('left').setPen('#34c759')
        self.plot_left.getAxis('left').setTextPen('#34c759')
        self.L_angle_line = self.plot_left.plot(pen=pen_angle)

        # Torque axis
        self.v_left_torque = pg.ViewBox()
        self.plot_left.scene().addItem(self.v_left_torque)
        self.v_left_torque.setXLink(self.plot_left.getViewBox())
        self.plot_left.showAxis('right')
        self.plot_left.getAxis('right').setLabel('Torque', units='Nm', color='#007aff')
        self.plot_left.getAxis('right').setPen('#007aff')
        self.plot_left.getAxis('right').setTextPen('#007aff')
        self.plot_left.getAxis('right').linkToView(self.v_left_torque)
        self.L_tau_d_line = pg.PlotDataItem(pen=pen_cmd)
        self.L_tau_line   = pg.PlotDataItem(pen=pen_est)
        self.v_left_torque.addItem(self.L_tau_d_line)
        self.v_left_torque.addItem(self.L_tau_line)

        # Velocity axis (3rd)
        self.v_left_vel = pg.ViewBox()
        self.plot_left.scene().addItem(self.v_left_vel)
        self.v_left_vel.setXLink(self.plot_left.getViewBox())
        ax_vel_L = pg.AxisItem('right')
        ax_vel_L.setLabel('Vel', units='deg/s', color='#5ac8fa')
        pi_L = self.plot_left.getPlotItem()
        pi_L.layout.addItem(ax_vel_L, 2, pi_L.layout.columnCount())
        ax_vel_L.linkToView(self.v_left_vel)
        self.L_vel_line = pg.PlotDataItem(pen=pen_vel)
        self.v_left_vel.addItem(self.L_vel_line)
        self._ax_vel_L = ax_vel_L

        # Power axis (4th)
        self.v_left_pwr = pg.ViewBox()
        self.plot_left.scene().addItem(self.v_left_pwr)
        self.v_left_pwr.setXLink(self.plot_left.getViewBox())
        ax_pwr_L = pg.AxisItem('right')
        ax_pwr_L.setLabel('Power', units='W', color='#af52de')
        ax_pwr_L.setPen('#af52de')
        ax_pwr_L.setTextPen('#af52de')
        pi_L.layout.addItem(ax_pwr_L, 2, pi_L.layout.columnCount())
        ax_pwr_L.linkToView(self.v_left_pwr)
        self.L_pwr_line = pg.PlotDataItem(pen=pen_pwr)
        self.v_left_pwr.addItem(self.L_pwr_line)
        self._ax_pwr_L = ax_pwr_L

        def sync_left():
            r = self.plot_left.getViewBox().sceneBoundingRect()
            self.v_left_torque.setGeometry(r)
            self.v_left_vel.setGeometry(r)
            self.v_left_pwr.setGeometry(r)
        self.plot_left.getViewBox().sigResized.connect(sync_left)
        self.plot_layout.addWidget(self.plot_left, 1)

        # Compact power-sign strip (Right leg): green above 0, red below 0
        self.pwr_strip_right = pg.PlotWidget(background=C.plot_bg)
        self.pwr_strip_right.setFixedHeight(72)
        self.pwr_strip_right.setTitle("Right Leg Power Sign", color=C.text2, size='10pt')
        self.pwr_strip_right.showGrid(x=True, y=False, alpha=0.10)
        self.pwr_strip_right.setXLink(self.plot_left)
        self.pwr_strip_right.setMouseEnabled(x=False, y=False)
        self.pwr_strip_right.getAxis('left').setWidth(24)
        self.pwr_strip_right.getAxis('left').setLabel('', units='W')
        self.pwr_strip_right.getAxis('left').setPen(C.text2)
        self.pwr_strip_right.getAxis('left').setTextPen(C.text2)
        self.pwr_strip_right.getAxis('bottom').setStyle(showValues=False)
        self.pwr_strip_right.getAxis('bottom').setPen(C.separator)
        self.pwr_strip_right.getAxis('bottom').setTextPen(C.separator)
        self.pwr_strip_right_zero = pg.PlotDataItem(pen=pg.mkPen(C.separator, width=1.2))
        self.pwr_strip_right_pos = pg.PlotDataItem(pen=pg.mkPen(C.green, width=1.2))
        self.pwr_strip_right_neg = pg.PlotDataItem(pen=pg.mkPen(C.red, width=1.2))
        green_fill_right = QColor(C.green); green_fill_right.setAlpha(96)
        red_fill_right = QColor(C.red); red_fill_right.setAlpha(96)
        self.pwr_strip_right_fill_pos = pg.FillBetweenItem(
            self.pwr_strip_right_pos, self.pwr_strip_right_zero, brush=pg.mkBrush(green_fill_right)
        )
        self.pwr_strip_right_fill_neg = pg.FillBetweenItem(
            self.pwr_strip_right_neg, self.pwr_strip_right_zero, brush=pg.mkBrush(red_fill_right)
        )
        self.pwr_strip_right.addItem(self.pwr_strip_right_fill_pos)
        self.pwr_strip_right.addItem(self.pwr_strip_right_fill_neg)
        self.pwr_strip_right.addItem(self.pwr_strip_right_zero)
        self.pwr_strip_right.addItem(self.pwr_strip_right_pos)
        self.pwr_strip_right.addItem(self.pwr_strip_right_neg)
        self.pwr_strip_right_overlay = pg.TextItem(anchor=(1, 0))
        self.pwr_strip_right_overlay.setZValue(20)
        self.pwr_strip_right.addItem(self.pwr_strip_right_overlay)
        self.pwr_strip_right.setYRange(-5.0, 5.0, padding=0.02)
        self.plot_layout.addWidget(self.pwr_strip_right, 0)

        # ---- Left Leg ----
        self.plot_right = pg.PlotWidget(background=C.plot_bg)
        self.plot_right.setTitle("Left Leg", color=C.plot_fg, size='13pt')
        self.plot_right.showGrid(x=True, y=True, alpha=0.15)
        self.plot_right.getAxis('left').setLabel('Angle', units='deg', color='#34c759')
        self.plot_right.getAxis('left').setPen('#34c759')
        self.plot_right.getAxis('left').setTextPen('#34c759')
        self.R_angle_line = self.plot_right.plot(pen=pen_angle)

        self.v_right_torque = pg.ViewBox()
        self.plot_right.scene().addItem(self.v_right_torque)
        self.v_right_torque.setXLink(self.plot_right.getViewBox())
        self.plot_right.showAxis('right')
        self.plot_right.getAxis('right').setLabel('Torque', units='Nm', color='#007aff')
        self.plot_right.getAxis('right').setPen('#007aff')
        self.plot_right.getAxis('right').setTextPen('#007aff')
        self.plot_right.getAxis('right').linkToView(self.v_right_torque)
        self.R_tau_d_line = pg.PlotDataItem(pen=pen_cmd)
        self.R_tau_line   = pg.PlotDataItem(pen=pen_est)
        self.v_right_torque.addItem(self.R_tau_d_line)
        self.v_right_torque.addItem(self.R_tau_line)

        self.v_right_vel = pg.ViewBox()
        self.plot_right.scene().addItem(self.v_right_vel)
        self.v_right_vel.setXLink(self.plot_right.getViewBox())
        ax_vel_R = pg.AxisItem('right')
        ax_vel_R.setLabel('Vel', units='deg/s', color='#5ac8fa')
        pi_R = self.plot_right.getPlotItem()
        pi_R.layout.addItem(ax_vel_R, 2, pi_R.layout.columnCount())
        ax_vel_R.linkToView(self.v_right_vel)
        self.R_vel_line = pg.PlotDataItem(pen=pen_vel)
        self.v_right_vel.addItem(self.R_vel_line)
        self._ax_vel_R = ax_vel_R

        # Power axis (4th)
        self.v_right_pwr = pg.ViewBox()
        self.plot_right.scene().addItem(self.v_right_pwr)
        self.v_right_pwr.setXLink(self.plot_right.getViewBox())
        ax_pwr_R = pg.AxisItem('right')
        ax_pwr_R.setLabel('Power', units='W', color='#af52de')
        ax_pwr_R.setPen('#af52de')
        ax_pwr_R.setTextPen('#af52de')
        pi_R.layout.addItem(ax_pwr_R, 2, pi_R.layout.columnCount())
        ax_pwr_R.linkToView(self.v_right_pwr)
        self.R_pwr_line = pg.PlotDataItem(pen=pen_pwr)
        self.v_right_pwr.addItem(self.R_pwr_line)
        self._ax_pwr_R = ax_pwr_R

        def sync_right():
            r = self.plot_right.getViewBox().sceneBoundingRect()
            self.v_right_torque.setGeometry(r)
            self.v_right_vel.setGeometry(r)
            self.v_right_pwr.setGeometry(r)
        self.plot_right.getViewBox().sigResized.connect(sync_right)
        self.plot_layout.addWidget(self.plot_right, 1)

        # Compact power-sign strip (Left leg): green above 0, red below 0
        self.pwr_strip_left = pg.PlotWidget(background=C.plot_bg)
        self.pwr_strip_left.setFixedHeight(72)
        self.pwr_strip_left.setTitle("Left Leg Power Sign", color=C.text2, size='10pt')
        self.pwr_strip_left.showGrid(x=True, y=False, alpha=0.10)
        self.pwr_strip_left.setXLink(self.plot_right)
        self.pwr_strip_left.setMouseEnabled(x=False, y=False)
        self.pwr_strip_left.getAxis('left').setWidth(24)
        self.pwr_strip_left.getAxis('left').setLabel('', units='W')
        self.pwr_strip_left.getAxis('left').setPen(C.text2)
        self.pwr_strip_left.getAxis('left').setTextPen(C.text2)
        # Keep same compact vertical layout as right strip; otherwise overlay text
        # can be clipped because bottom tick labels consume strip height.
        self.pwr_strip_left.getAxis('bottom').setStyle(showValues=False)
        self.pwr_strip_left.getAxis('bottom').setPen(C.plot_fg)
        self.pwr_strip_left.getAxis('bottom').setTextPen(C.plot_fg)
        self.pwr_strip_left_zero = pg.PlotDataItem(pen=pg.mkPen(C.separator, width=1.2))
        self.pwr_strip_left_pos = pg.PlotDataItem(pen=pg.mkPen(C.green, width=1.2))
        self.pwr_strip_left_neg = pg.PlotDataItem(pen=pg.mkPen(C.red, width=1.2))
        green_fill_left = QColor(C.green); green_fill_left.setAlpha(96)
        red_fill_left = QColor(C.red); red_fill_left.setAlpha(96)
        self.pwr_strip_left_fill_pos = pg.FillBetweenItem(
            self.pwr_strip_left_pos, self.pwr_strip_left_zero, brush=pg.mkBrush(green_fill_left)
        )
        self.pwr_strip_left_fill_neg = pg.FillBetweenItem(
            self.pwr_strip_left_neg, self.pwr_strip_left_zero, brush=pg.mkBrush(red_fill_left)
        )
        self.pwr_strip_left.addItem(self.pwr_strip_left_fill_pos)
        self.pwr_strip_left.addItem(self.pwr_strip_left_fill_neg)
        self.pwr_strip_left.addItem(self.pwr_strip_left_zero)
        self.pwr_strip_left.addItem(self.pwr_strip_left_pos)
        self.pwr_strip_left.addItem(self.pwr_strip_left_neg)
        self.pwr_strip_left_overlay = pg.TextItem(anchor=(1, 0))
        self.pwr_strip_left_overlay.setZValue(20)
        self.pwr_strip_left.addItem(self.pwr_strip_left_overlay)
        self.pwr_strip_left.setYRange(-5.0, 5.0, padding=0.02)
        self.plot_layout.addWidget(self.pwr_strip_left, 0)
        self._update_power_strip_titles(force=True)
        self._configure_plot_items_performance()

    def _configure_plot_items_performance(self):
        """Low-risk pyqtgraph perf tuning for long-running sessions."""
        plot_items = [
            self.L_angle_line, self.L_tau_d_line, self.L_tau_line, self.L_vel_line, self.L_pwr_line,
            self.R_angle_line, self.R_tau_d_line, self.R_tau_line, self.R_vel_line, self.R_pwr_line,
            self.pwr_strip_right_zero, self.pwr_strip_right_pos, self.pwr_strip_right_neg,
            self.pwr_strip_left_zero, self.pwr_strip_left_pos, self.pwr_strip_left_neg,
        ]
        for item in plot_items:
            if item is None:
                continue
            try:
                item.setClipToView(True)
            except Exception:
                pass
            try:
                item.setDownsampling(auto=True, method='peak')
            except Exception:
                pass

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Wheel and isinstance(obj, (QDoubleSpinBox, QSpinBox)):
            if not obj.hasFocus():
                event.ignore()
                return True
        return super().eventFilter(obj, event)

    def closeEvent(self, event):
        if hasattr(self, '_csv_file') and self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
        super().closeEvent(event)

    def _apply_plot_visibility(self):
        show_a = self.chk_plot_angle.isChecked()
        show_c = self.chk_plot_cmd.isChecked()
        show_e = self.chk_plot_est.isChecked()
        show_v = self.chk_plot_vel.isChecked()
        for a in ['L_angle_line', 'R_angle_line']:
            if hasattr(self, a): getattr(self, a).setVisible(show_a)
        for a in ['L_tau_d_line', 'R_tau_d_line']:
            if hasattr(self, a): getattr(self, a).setVisible(show_c)
        for a in ['L_tau_line', 'R_tau_line']:
            if hasattr(self, a): getattr(self, a).setVisible(show_e)
        for a in ['L_vel_line', 'R_vel_line']:
            if hasattr(self, a): getattr(self, a).setVisible(show_v)
        show_p = self.chk_plot_pwr.isChecked()
        for a in ['L_pwr_line', 'R_pwr_line']:
            if hasattr(self, a): getattr(self, a).setVisible(show_p)
        # Power Sign strip charts are always visible; they are NOT controlled by the Pwr legend checkbox

    def _update_power_strip(self, t, y, side):
        if not t or not y:
            return
        if side == 'right':
            zero = self.pwr_strip_right_zero
            pos_curve = self.pwr_strip_right_pos
            neg_curve = self.pwr_strip_right_neg
            strip = self.pwr_strip_right
            overlay = self.pwr_strip_right_overlay
            smooth_attr = '_pwr_band_scale_right'
        else:
            zero = self.pwr_strip_left_zero
            pos_curve = self.pwr_strip_left_pos
            neg_curve = self.pwr_strip_left_neg
            strip = self.pwr_strip_left
            overlay = self.pwr_strip_left_overlay
            smooth_attr = '_pwr_band_scale_left'

        pos = [v if v > 0.0 else 0.0 for v in y]
        neg = [v if v < 0.0 else 0.0 for v in y]
        z = [0.0] * len(y)
        zero.setData(t, z)
        pos_curve.setData(t, pos)
        neg_curve.setData(t, neg)

        max_abs_now = max((abs(v) for v in y), default=0.0)
        target = max(2.0, max_abs_now * 1.15)
        prev = getattr(self, smooth_attr, 5.0)
        smoothed = prev * 0.88 + target * 0.12
        setattr(self, smooth_attr, smoothed)
        strip.setYRange(-smoothed, smoothed, padding=0.02)
        self._position_power_ratio_overlay(strip, overlay)

    def _update_power_strip_titles(self, force=False):
        """Keep strip titles static, and show both live(strip) and eval(auto) metrics."""
        if not hasattr(self, 'pwr_strip_right') or not hasattr(self, 'pwr_strip_left'):
            return
        self.pwr_strip_right.setTitle("Right Leg Power Sign", color=C.text2, size='10pt')
        self.pwr_strip_left.setTitle("Left Leg Power Sign", color=C.text2, size='10pt')

        def _format_leg_overlay(
            live_ratio, live_pos_w, live_neg_w, live_valid,
            eval_ratio, eval_valid,
            motion_valid, auto_enable
        ):
            auto_txt = "ON" if auto_enable else "OFF"
            motion_txt = "VALID" if motion_valid else "HOLD"

            if not live_valid and not eval_valid:
                line1 = f"Live --.-% | Eval --.-% | WAIT"
                line2 = f"Live +P --.-- W  -P --.-- W"
                return line1, line2

            if live_valid:
                live_ratio_txt = f"{max(0.0, min(1.0, float(live_ratio))) * 100.0:.1f}%"
                line2 = f"Live +P {float(live_pos_w):+.2f} W  -P {float(live_neg_w):+.2f} W"
            else:
                live_ratio_txt = "--.-%"
                line2 = f"Live +P --.-- W  -P --.-- W"

            if eval_valid:
                eval_ratio_txt = f"{max(0.0, min(1.0, float(eval_ratio))) * 100.0:.1f}%"
            else:
                eval_ratio_txt = "--.-%"

            line1 = f"Live {live_ratio_txt} | Eval {eval_ratio_txt} | {auto_txt}/{motion_txt}"
            return line1, line2

        # Live strip metrics are always computed from the currently displayed power buffer.
        live_ratio_L, live_pos_L, live_neg_L, live_mv_L, live_valid_L = \
            self._compute_local_power_overlay_metrics('left')
        live_ratio_R, live_pos_R, live_neg_R, live_mv_R, live_valid_R = \
            self._compute_local_power_overlay_metrics('right')

        if self._rpi_status_valid:
            auto_enable = bool(self._rpi_auto_delay_enable)
            eval_ratio_L = float(self._rpi_power_ratio_L)
            eval_ratio_R = float(self._rpi_power_ratio_R)
            eval_valid_L = True
            eval_valid_R = True
            mv_L = bool(self._rpi_auto_motion_valid_L)
            mv_R = bool(self._rpi_auto_motion_valid_R)
        else:
            eval_ratio_L = 0.0
            eval_ratio_R = 0.0
            eval_valid_L = False
            eval_valid_R = False
            algo = int(getattr(self, "_algo_select", ALGO_EG))
            auto_enable = (
                (algo == ALGO_EG and bool(self._eg_auto_delay_enable)) or
                (algo == ALGO_SAMSUNG and bool(self._sam_auto_delay_enable))
            )
            mv_L = bool(live_mv_L)
            mv_R = bool(live_mv_R)

        l1_L, l2_L = _format_leg_overlay(
            live_ratio_L, live_pos_L, live_neg_L, live_valid_L,
            eval_ratio_L, eval_valid_L,
            mv_L, auto_enable
        )
        l1_R, l2_R = _format_leg_overlay(
            live_ratio_R, live_pos_R, live_neg_R, live_valid_R,
            eval_ratio_R, eval_valid_R,
            mv_R, auto_enable
        )

        self._set_power_overlay_html(
            side='left',
            strip=getattr(self, 'pwr_strip_left', None),
            overlay=getattr(self, 'pwr_strip_left_overlay', None),
            line1=l1_L,
            line2=l2_L,
            force=force,
        )
        self._set_power_overlay_html(
            side='right',
            strip=getattr(self, 'pwr_strip_right', None),
            overlay=getattr(self, 'pwr_strip_right_overlay', None),
            line1=l1_R,
            line2=l2_R,
            force=force,
        )

    def _compute_local_power_overlay_metrics(self, side):
        """Fallback overlay metrics from current GUI power strip window."""
        buf = self.L_pwr_buf if side == 'left' else self.R_pwr_buf
        vals = []
        for v in buf:
            fv = float(v)
            if isfinite(fv):
                vals.append(fv)
        n = len(vals)
        if n < 3:
            return 0.0, 0.0, 0.0, False, False

        pos_sum = 0.0
        neg_abs_sum = 0.0
        for v in vals:
            if v > 0.0:
                pos_sum += v
            elif v < 0.0:
                neg_abs_sum += (-v)
        denom = pos_sum + neg_abs_sum
        if denom <= 1e-6:
            return 0.0, 0.0, 0.0, False, False

        pos_per_s = pos_sum / float(n)
        neg_per_s = -neg_abs_sum / float(n)
        motion_valid = (
            (abs(pos_per_s) + abs(neg_per_s)) >= float(self._power_overlay_local_min_abs_w)
        )
        ratio = pos_sum / denom
        return ratio, pos_per_s, neg_per_s, motion_valid, True

    def _set_power_overlay_html(self, side, strip, overlay, line1, line2, force=False):
        if overlay is None or strip is None:
            return
        now = time.time()
        sig = (str(line1), str(line2), C.purple)
        if side == 'left':
            prev_sig = self._power_overlay_left_sig
            prev_ts = self._power_overlay_left_last_ts
        else:
            prev_sig = self._power_overlay_right_sig
            prev_ts = self._power_overlay_right_last_ts
        changed = (sig != prev_sig)
        if (not force) and (not changed):
            return
        if (not force) and (now - prev_ts) < self._power_overlay_min_interval_s:
            return

        overlay_html = (
            f"<div style='color:{C.purple}; font-size:13pt; font-weight:600; "
            f"background:rgba(0,0,0,0); text-align:right;'>{line1}<br/>{line2}</div>"
        )
        overlay.setHtml(overlay_html)
        self._position_power_ratio_overlay(strip, overlay)
        if side == 'left':
            self._power_overlay_left_sig = sig
            self._power_overlay_left_last_ts = now
        else:
            self._power_overlay_right_sig = sig
            self._power_overlay_right_last_ts = now

    def _position_power_ratio_overlay(self, strip, overlay_item):
        if overlay_item is None or strip is None:
            return
        vr = strip.viewRange()
        if not vr or len(vr) != 2:
            return
        x0, x1 = vr[0]
        y0, y1 = vr[1]
        if not (isfinite(x0) and isfinite(x1) and isfinite(y0) and isfinite(y1)):
            return
        dx = max(1e-6, float(x1 - x0))
        dy = max(1e-6, float(y1 - y0))
        x = x1 - 0.012 * dx
        # Left overlay is slightly higher than right.
        if overlay_item is getattr(self, 'pwr_strip_left_overlay', None):
            y = y1 - 0.001 * dy
        else:
            y = y1 - 0.005 * dy
        overlay_item.setPos(x, y)

    def _update_tag_panel(self, now=None):
        if not hasattr(self, 'lbl_tag_state'):
            return
        if now is None:
            now = time.time()

        if self._current_tag:
            self.lbl_tag_state.setText(f"Tag: {self._current_tag}")
            if self._tag_started_at is not None and now >= self._tag_started_at:
                dt = now - self._tag_started_at
                mm = int(dt // 60.0)
                ss = dt - mm * 60.0
                self.lbl_tag_timer.setText(f"Timer: {mm:02d}:{ss:04.1f}")
            else:
                self.lbl_tag_timer.setText("Timer: 00:00.0")
        else:
            self.lbl_tag_state.setText("Tag: (empty)")
            self.lbl_tag_timer.setText("Timer: --:--.-")

        self.lbl_tag_rx.setText(f"RX tag: {self._last_rx_tag_char if self._last_rx_tag_char else '-'}")
        if self._last_gait_freq is None:
            self.lbl_tag_gait.setText("Gait: - Hz")
        else:
            self.lbl_tag_gait.setText(f"Gait: {self._last_gait_freq:.2f} Hz")

    def _build_value_displays(self):
        # Compact status bar below plots: IMU, MaxT, Tag, Timer, RX, Gait — one row
        val_card = CardFrame()
        self._cards.append(val_card)
        row = QHBoxLayout(val_card)
        row.setContentsMargins(12, 6, 12, 6)
        row.setSpacing(12)

        def status_lbl(txt, color, bold=False):
            l = QLabel(txt)
            w = 600 if bold else 400
            l.setStyleSheet(f"font-size:13px; font-weight:{w}; background:transparent; color:{color};")
            return l

        self.lbl_imu  = status_lbl("IMU: -", C.text2)
        self.lbl_maxt = status_lbl("MaxT: -", C.text2)
        self.lbl_tag_state = status_lbl("Tag: (empty)", C.text, True)
        self.lbl_tag_timer = status_lbl("Timer: --:--.-", C.text2)
        self.lbl_tag_rx = status_lbl("RX tag: -", C.text2)
        self.lbl_tag_gait = status_lbl("Gait: - Hz", C.text2)

        row.addWidget(self.lbl_imu)
        row.addWidget(self.lbl_maxt)
        row.addWidget(self.lbl_tag_state)
        row.addWidget(self.lbl_tag_timer)
        row.addWidget(self.lbl_tag_rx)
        row.addWidget(self.lbl_tag_gait)
        row.addStretch(1)
        self.plot_layout.addWidget(val_card)

    # ================================================================ Serial
    def _on_serial_connected(self, port, now=None):
        if now is None:
            now = time.time()
        self.connected = True
        self._rx_buf.clear()
        self._last_rx_time = now
        self._conn_healthy = True
        self._last_connected_port = port
        self._last_auto_connect_port = port
        self._last_auto_connect_err_sig = None
        self._uplink_prev_t_cs = None
        self._uplink_wrap_count = 0
        self._uplink_t_unwrapped_s = 0.0
        idx = self.cmb_port.findText(port)
        if idx >= 0:
            self.cmb_port.setCurrentIndex(idx)
        # Reset RPi state on new connection
        self._rpi_status_valid = False
        self._rpi_online = False
        self._rpi_last_rx_time = 0.0
        self._rpi_nn_type = -1
        self._rpi_status_version = 0
        self._rpi_auto_delay_enable = False
        self._rpi_auto_method_bo = False
        self._rpi_auto_motion_valid = False
        self._rpi_auto_motion_valid_L = False
        self._rpi_auto_motion_valid_R = False
        self._rpi_power_ratio = 0.0
        self._rpi_power_ratio_L = 0.0
        self._rpi_power_ratio_R = 0.0
        self._rpi_pos_per_s = 0.0
        self._rpi_pos_per_s_L = 0.0
        self._rpi_pos_per_s_R = 0.0
        self._rpi_neg_per_s = 0.0
        self._rpi_neg_per_s_L = 0.0
        self._rpi_neg_per_s_R = 0.0
        self._rpi_best_delay_ms = 0.0
        self._rpi_best_delay_ms_L = 0.0
        self._rpi_best_delay_ms_R = 0.0
        self._rpi_delay_ms_L = 0.0
        self._rpi_delay_ms_R = 0.0
        self._sam_auto_delay_enable = False
        self._eg_auto_delay_enable = False
        if hasattr(self, 'lbl_rpi_nn_type'):
            self.lbl_rpi_nn_type.setText("RPi: waiting...")
            self.lbl_rpi_nn_type.setStyleSheet(
                f"color:{C.orange}; font-size:13px; font-weight:600; background:transparent;")
            self.lbl_rpi_current_filter.setText("RPi Active: waiting for status...")
            if hasattr(self, 'lbl_rl_auto_state'):
                self.lbl_rl_auto_state.setText("Auto Delay: waiting for RPi status...")
        self.conn_dot.set_state('connected')
        self.btn_connect.setText("Connected")
        self._set_power_ui(self.sb_max_torque_cfg.value() > 0.0)
        self._tx_params()
        self._queue_align_event(f"serial_connected:{port}", pi_tag="SERCON")

    def _connect_clicked(self):
        port = self.cmb_port.currentText()
        if not port:
            QMessageBox.warning(self, "Port", "No serial port found. Waiting for auto refresh.")
            return
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(port, 115200, timeout=0)
            self._on_serial_connected(port, now=time.time())
        except (serial.SerialException, OSError):
            QMessageBox.critical(self, "Error", f"Cannot open {port}")

    def _maybe_auto_connect(self, now):
        if self.connected or (not self._auto_connect_enabled):
            return
        if (now - self._last_auto_connect_try_ts) < self._auto_connect_cooldown_s:
            return
        port = self._pick_auto_connect_port()
        if not port:
            return
        self._last_auto_connect_try_ts = now
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(port, 115200, timeout=0)
            self._on_serial_connected(port, now=now)
            self.lbl_status.setText(f"Auto-connected: {port}")
        except (serial.SerialException, OSError) as e:
            err_sig = (port, type(e).__name__, str(e))
            if err_sig != self._last_auto_connect_err_sig:
                self._last_auto_connect_err_sig = err_sig
                self.lbl_status.setText(f"Auto-connect failed ({port}): {str(e)[:60]}")

    def _update_connect_btn(self, healthy):
        if not self.connected:
            self.conn_dot.set_state('idle')
            self.btn_connect.setText("Connect")
        elif healthy:
            self.conn_dot.set_state('connected')
            self.btn_connect.setText("Connected")
        else:
            self.conn_dot.set_state('warning')
            self.btn_connect.setText("No Data")

    # ================================================================ TX
    def _tx_params(self):
        self._update_rl_filter_state_label()
        if not (self.connected and self.ser): return

        payload = bytearray(BLE_PAYLOAD_LEN)
        def clip16(x): return max(-32768, min(32767, int(x)))
        def put_s16(ix, val, scale=100):
            v = clip16(round(val * scale))
            payload[ix]   = v & 0xFF
            payload[ix+1] = (v >> 8) & 0xFF

        payload[0] = self._algo_select & 0xFF
        payload[1] = self._brand_request & 0xFF
        flags = 0
        if self._imu_init_request:   flags |= 0x01
        if self._motor_init_request: flags |= 0x02
        flags |= (self._dir_bits & 0x03) << 2
        payload[2] = flags
        put_s16(3, max(0.0, min(30.0, float(self.sb_max_torque_cfg.value()))))
        # Unified Teensy pre-motor filter cutoff [31..32] (Hz*100).
        put_s16(31, float(self.sb_torque_filter_fc.value()))

        algo = self._algo_select
        if algo == ALGO_EG:
            put_s16(5,  float(self.Rescaling_gain_def))
            put_s16(7,  float(self.sb_Flex_Assist_gain.value()))
            put_s16(9,  float(self.sb_Ext_Assist_gain.value()))
            payload[11] = int(self.sb_Assist_delay_gain.value()) & 0xFF
            payload[12] = self.offL_def & 0xFF
            payload[13] = self.offR_def & 0xFF
            put_s16(14, float(self.sb_gate_k.value()))
            put_s16(16, float(self.sb_gate_p_on.value()))
            put_s16(18, float(self.lead_frac_def), 1000)
            put_s16(20, float(self.sb_ext_phase_frac_L.value()), 1000)
            put_s16(22, float(self.sb_ext_phase_frac_R.value()), 1000)
            put_s16(24, float(self.sb_ext_gain.value()))
            put_s16(26, float(self.sb_scale_all.value()))
            # [28] auto_delay_enable bit0; [29..30] eg_post_delay_ms
            payload[28] = 0x01 if self._eg_auto_delay_enable else 0x00
            put_s16(29, float(self.sb_eg_post_delay.value()), 1)
        elif algo == ALGO_SAMSUNG:
            put_s16(5, float(self.sb_sam_kappa.value()))
            put_s16(7, float(self.sb_sam_delay.value()), 1)
            # [28] auto_delay_enable bit0
            payload[28] = 0x01 if self._sam_auto_delay_enable else 0x00
        elif algo == ALGO_RL:
            payload[58:98] = self._build_rl_passthrough()
        elif algo == ALGO_TEST:
            put_s16(5, float(self.sb_test_amplitude.value()))
            payload[7] = self.cmb_test_waveform.currentIndex() & 0xFF
            put_s16(8, float(self.sb_test_freq.value()))
        elif algo == ALGO_SOGI:
            put_s16(5, float(self.sb_sogi_A.value()))
            put_s16(7, float(self.sb_sogi_lead.value()))
            put_s16(9, float(self.sb_sogi_amp_min.value()), 10)

        header = struct.pack('<BBB', 0xA5, 0x5A, BLE_FRAME_LEN)
        self.ser.write(header + payload)
        if algo == ALGO_RL:
            self._rl_cfg_tx_seq += 1
            self._rl_last_tx_ts = time.time()
        self._update_rl_filter_state_label()

    def _queue_align_event(self, label, pi_tag=None):
        txt = str(label or "").strip()
        if not txt:
            return
        self._align_events_pending.append(txt)
        if pi_tag:
            self._send_logtag_text(pi_tag, persist=False, update_current=False)

    def _send_logtag_text(self, tag_text, persist=False, update_current=True):
        if not (self.connected and self.ser):
            return False
        tag_raw = str(tag_text or "")
        tag = tag_raw.encode('ascii', 'ignore')[:10]
        tag_txt = tag.decode('ascii', 'ignore').strip()
        flags = 0x01 if persist else 0x00
        payload = bytearray(BLE_PAYLOAD_LEN)
        payload[0] = ord('L')
        payload[1] = ord('G')
        payload[2] = len(tag)
        payload[13] = flags
        if len(tag):
            payload[3:3+len(tag)] = tag
        header = struct.pack('<BBB', 0xA5, 0x5A, BLE_FRAME_LEN)
        self.ser.write(header + payload)
        if update_current:
            self._current_tag = tag_txt
            self._tag_started_at = time.time() if tag_txt else None
            self._update_tag_panel(self._tag_started_at if self._tag_started_at else time.time())
        return True

    def _send_logtag(self):
        tag_txt = self.edt_label.text()
        self._send_logtag_text(tag_txt, persist=self.sw_persist.isChecked(), update_current=True)

    # ================================================================ Main loop
    def _update_everything(self):
        now = time.time()
        self._drain_async_jobs()
        self._maybe_poll_pi_rl_remote(now)
        self._update_tag_panel(now)
        if self._replay_mode:
            self._update_replay(now)
            return
        # Refresh port list periodically while disconnected.
        if not self.connected:
            if now - self._last_port_scan > 1.0:
                self._refresh_ports()
            self._maybe_auto_connect(now)
            return
        if self._last_rx_time > 0 and (now - self._last_rx_time) > CONN_TIMEOUT_S:
            if self._conn_healthy:
                self._conn_healthy = False
                self._update_connect_btn(False)
                if self.btn_power.isChecked():
                    self._maxT_before_off = float(self.sb_max_torque_cfg.value())
                    self.sb_max_torque_cfg.setValue(0.0)
                    self._set_power_ui(False)
        # RPi offline detection (status sent every 0.5s, timeout at 2s)
        if self._rpi_online and self._rpi_last_rx_time > 0 and (now - self._rpi_last_rx_time) > 2.0:
            self._rpi_online = False
            self._rpi_status_valid = False
            self._update_rpi_offline_ui()
        self._read_serial()
        self._maybe_render_plot(now)

    # ================================================================ RX
    def _read_serial(self):
        try:
            avail = self.ser.in_waiting
        except (serial.SerialException, OSError):
            self._queue_align_event("serial_lost")
            self.connected = False
            self._conn_healthy = False
            self._rx_buf.clear()
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self._update_connect_btn(False)
            return

        if avail <= 0:
            return

        chunk = self.ser.read(avail)
        if not chunk:
            return
        self._rx_buf.extend(chunk)

        while True:
            head_idx = self._rx_buf.find(BLE_HEADER)
            if head_idx < 0:
                # Keep 2 tail bytes for cross-chunk header matches.
                if len(self._rx_buf) > 2:
                    del self._rx_buf[:-2]
                return

            if head_idx > 0:
                del self._rx_buf[:head_idx]

            if len(self._rx_buf) < BLE_FRAME_LEN:
                return

            payload = bytes(self._rx_buf[3:BLE_FRAME_LEN])
            del self._rx_buf[:BLE_FRAME_LEN]
            if len(payload) != BLE_PAYLOAD_LEN:
                continue

            self._handle_uplink_payload(payload)

    def _handle_uplink_payload(self, payload):
        self._last_rx_time = time.time()
        if not self._conn_healthy:
            self._conn_healthy = True
            self._update_connect_btn(True)

        # Parse uplink
        data = struct.unpack('<Hhhhhhh', payload[:14])
        t_cs, L_angle_i, R_angle_i, L_tau_i, R_tau_i, L_cmd_i, R_cmd_i = data
        L_angle = L_angle_i / 100.0
        R_angle = R_angle_i / 100.0
        L_tau   = L_tau_i   / 100.0
        R_tau   = R_tau_i   / 100.0
        L_tau_d = L_cmd_i   / 100.0
        R_tau_d = R_cmd_i   / 100.0
        t = (t_cs & 0xFFFF) / 100.0
        t_cs_u16 = int(t_cs) & 0xFFFF
        if self._uplink_prev_t_cs is not None:
            dt_cs = t_cs_u16 - int(self._uplink_prev_t_cs)
            if dt_cs < -30000:
                self._uplink_wrap_count += 1
            elif dt_cs > 30000:
                # Likely Teensy timer reset/restart: resync unwrapped baseline.
                self._uplink_wrap_count = 0
        self._uplink_prev_t_cs = t_cs_u16
        self._uplink_t_unwrapped_s = ((self._uplink_wrap_count * 65536) + t_cs_u16) / 100.0

        imu_ok_flag = payload[14]
        mt100 = struct.unpack('<h', payload[15:17])[0]
        maxT_rx = mt100 / 100.0
        sd_ok = payload[17]
        gf100 = struct.unpack('<h', payload[20:22])[0]
        gait_freq = gf100 / 100.0
        self._last_gait_freq = gait_freq
        tag_valid = payload[22]
        tag_char  = chr(payload[23]) if payload[23] else ''
        imu_bits = payload[24]
        brand_id = payload[25]
        temp_L   = payload[26] if payload[26] else None
        temp_R   = payload[27] if payload[27] else None
        active_algo = payload[28]
        self._last_rx_tag_char = tag_char if (tag_valid and tag_char) else ""
        self._last_rx_tag_text = self._last_rx_tag_char
        if (not self._current_tag) and tag_valid and tag_char:
            self._current_tag = tag_char
            self._tag_started_at = time.time()

        tx1 = struct.unpack('<h', payload[29:31])[0] / 100.0
        tx2 = struct.unpack('<h', payload[31:33])[0] / 100.0
        tx3 = struct.unpack('<h', payload[33:35])[0] / 100.0
        tx4 = struct.unpack('<h', payload[35:37])[0] / 100.0

        # IMU angular velocities packed by Teensy firmware (payload[37..48]).
        vel_blob = payload[37:49]
        has_imu_vel = any(b != 0 for b in vel_blob)
        ltavx = struct.unpack('<h', payload[37:39])[0] / 10.0
        rtavx = struct.unpack('<h', payload[39:41])[0] / 10.0
        vtx1 = struct.unpack('<h', payload[41:43])[0] / 10.0
        vtx2 = struct.unpack('<h', payload[43:45])[0] / 10.0
        vtx3 = struct.unpack('<h', payload[45:47])[0] / 10.0
        vtx4 = struct.unpack('<h', payload[47:49])[0] / 10.0
        imu_batt_raw = [
            int(payload[49]), int(payload[50]), int(payload[51]),
            int(payload[52]), int(payload[53]), int(payload[54]),
        ]

        # Velocity
        now = time.time()
        dt = now - self._prev_wall_time if self._prev_wall_time > 0 else 0.05
        if has_imu_vel:
            L_vel = ltavx
            R_vel = rtavx
        elif 0 < dt < 1.0:
            L_vel = (L_angle - self._prev_L_angle) / dt
            R_vel = (R_angle - self._prev_R_angle) / dt
        else:
            L_vel = R_vel = 0.0
        self._prev_wall_time = now
        self._prev_L_angle = L_angle
        self._prev_R_angle = R_angle

        # Raw stream snapshot (always from Teensy uplink baseline fields)
        self._raw_ang_L = float(L_angle)
        self._raw_ang_R = float(R_angle)
        self._raw_vel_L = float(L_vel)
        self._raw_vel_R = float(R_vel)
        self._raw_cmd_L = float(L_tau_d)
        self._raw_cmd_R = float(R_tau_d)

        # === Parse telemetry extension [98..124] ===
        ext_blob = payload[TELEM_EXT_OFFSET:TELEM_EXT_OFFSET + TELEM_EXT_LEN]
        self._telem_ext_valid = False
        self._telem_phys_valid = False
        self._telem_ctrl_valid = False
        self._telem_sync_valid = False
        self._sync_from_pi = False
        if (len(ext_blob) == TELEM_EXT_LEN and
                ext_blob[0] == TELEM_EXT_MAGIC0 and
                ext_blob[1] == TELEM_EXT_MAGIC1 and
                ext_blob[2] >= TELEM_EXT_VERSION):
            flags = int(ext_blob[3])
            self._sync_sample_id = int(struct.unpack_from('<H', ext_blob, 4)[0]) & 0xFFFF
            self._phys_pwr_L = struct.unpack_from('<h', ext_blob, 6)[0] / 100.0
            self._phys_pwr_R = struct.unpack_from('<h', ext_blob, 8)[0] / 100.0
            self._sync_ang_L = struct.unpack_from('<h', ext_blob, 10)[0] / 100.0
            self._sync_ang_R = struct.unpack_from('<h', ext_blob, 12)[0] / 100.0
            self._sync_vel_L = struct.unpack_from('<h', ext_blob, 14)[0] / 10.0
            self._sync_vel_R = struct.unpack_from('<h', ext_blob, 16)[0] / 10.0
            self._sync_cmd_L = struct.unpack_from('<h', ext_blob, 18)[0] / 100.0
            self._sync_cmd_R = struct.unpack_from('<h', ext_blob, 20)[0] / 100.0
            self._ctrl_pwr_L = struct.unpack_from('<h', ext_blob, 22)[0] / 100.0
            self._ctrl_pwr_R = struct.unpack_from('<h', ext_blob, 24)[0] / 100.0
            self._telem_ext_valid = bool(flags & TELEM_EXT_FLAG_VALID)
            self._telem_phys_valid = bool(flags & TELEM_EXT_FLAG_PHYS_VALID)
            self._telem_ctrl_valid = bool(flags & TELEM_EXT_FLAG_CTRL_VALID)
            self._telem_sync_valid = bool(flags & TELEM_EXT_FLAG_SYNC_VALID)
            self._sync_from_pi = bool(flags & TELEM_EXT_FLAG_SYNC_FROM_PI)
        else:
            self._sync_sample_id = 0
            self._phys_pwr_L = 0.0
            self._phys_pwr_R = 0.0
            self._ctrl_pwr_L = 0.0
            self._ctrl_pwr_R = 0.0

        if not self._telem_sync_valid:
            self._sync_ang_L = self._raw_ang_L
            self._sync_ang_R = self._raw_ang_R
            self._sync_vel_L = self._raw_vel_L
            self._sync_vel_R = self._raw_vel_R
            self._sync_cmd_L = self._raw_cmd_L
            self._sync_cmd_R = self._raw_cmd_R

        # === Parse RPi uplink passthrough [58..97] ===
        rpi_blob = payload[58:98]
        if (len(rpi_blob) >= 20 and rpi_blob[0] == RPI_PT_MAGIC0 and
                rpi_blob[1] == RPI_PT_MAGIC1 and
                rpi_blob[2] >= RPI_STATUS_VERSION_LEGACY):
            version = rpi_blob[2]
            self._rpi_status_version = version
            self._rpi_nn_type = rpi_blob[3]
            self._rpi_filter_source = rpi_blob[4]
            self._rpi_filter_type_code = rpi_blob[5]
            self._rpi_filter_order = rpi_blob[6]
            self._rpi_enable_mask = rpi_blob[7]
            self._rpi_cutoff_hz = struct.unpack_from('<f', rpi_blob, 8)[0]
            self._rpi_scale = struct.unpack_from('<f', rpi_blob, 12)[0]

            if version >= RPI_STATUS_VERSION_PER_LEG and len(rpi_blob) >= 40:
                # v3: int16-packed L/R pairs
                dL = struct.unpack_from('<h', rpi_blob, 16)[0] / 10.0
                dR = struct.unpack_from('<h', rpi_blob, 18)[0] / 10.0
                auto_flags = rpi_blob[20]
                rL = struct.unpack_from('<h', rpi_blob, 24)[0] / 10000.0
                rR = struct.unpack_from('<h', rpi_blob, 26)[0] / 10000.0
                pL = struct.unpack_from('<h', rpi_blob, 28)[0] / 100.0
                pR = struct.unpack_from('<h', rpi_blob, 30)[0] / 100.0
                nL = struct.unpack_from('<h', rpi_blob, 32)[0] / 100.0
                nR = struct.unpack_from('<h', rpi_blob, 34)[0] / 100.0
                bL = struct.unpack_from('<h', rpi_blob, 36)[0] / 10.0
                bR = struct.unpack_from('<h', rpi_blob, 38)[0] / 10.0

                valid_sample = (
                    isfinite(dL) and isfinite(dR) and isfinite(rL) and isfinite(rR) and
                    isfinite(pL) and isfinite(pR) and isfinite(nL) and isfinite(nR) and
                    isfinite(bL) and isfinite(bR) and
                    (0.0 <= dL <= self._rpi_delay_max_ms) and
                    (0.0 <= dR <= self._rpi_delay_max_ms) and
                    (0.0 <= bL <= self._rpi_delay_max_ms) and
                    (0.0 <= bR <= self._rpi_delay_max_ms) and
                    (-0.02 <= rL <= 1.02) and (-0.02 <= rR <= 1.02) and
                    (abs(pL) <= self._rpi_power_abs_max_w) and (abs(pR) <= self._rpi_power_abs_max_w) and
                    (abs(nL) <= self._rpi_power_abs_max_w) and (abs(nR) <= self._rpi_power_abs_max_w)
                )
                if self._rpi_status_valid:
                    valid_sample = valid_sample and (
                        abs(dL - self._rpi_delay_ms_L) <= self._rpi_delay_jump_guard_ms and
                        abs(dR - self._rpi_delay_ms_R) <= self._rpi_delay_jump_guard_ms
                    )
                if valid_sample:
                    self._rpi_delay_ms_L = dL
                    self._rpi_delay_ms_R = dR
                    self._rpi_auto_delay_enable = bool(auto_flags & RPI_AUTO_FLAG_ENABLE)
                    self._rpi_auto_method_bo = bool(auto_flags & RPI_AUTO_FLAG_METHOD_BO)
                    self._rpi_auto_motion_valid_L = bool(auto_flags & RPI_AUTO_FLAG_MOTION_VALID_L)
                    self._rpi_auto_motion_valid_R = bool(auto_flags & RPI_AUTO_FLAG_MOTION_VALID_R)
                    self._rpi_power_ratio_L = rL
                    self._rpi_power_ratio_R = rR
                    self._rpi_pos_per_s_L = pL
                    self._rpi_pos_per_s_R = pR
                    self._rpi_neg_per_s_L = nL
                    self._rpi_neg_per_s_R = nR
                    self._rpi_best_delay_ms_L = bL
                    self._rpi_best_delay_ms_R = bR
            else:
                # v2 legacy: single-leg float32 — mirror to both L/R for display.
                delay_ms = struct.unpack_from('<f', rpi_blob, 16)[0]
                auto_flags = rpi_blob[20] if len(rpi_blob) > 20 else 0
                motion_valid = bool(auto_flags & RPI_AUTO_FLAG_MOTION_VALID_L)
                if len(rpi_blob) >= 40:
                    ratio = struct.unpack_from('<f', rpi_blob, 24)[0]
                    pos_w = struct.unpack_from('<f', rpi_blob, 28)[0]
                    neg_w = struct.unpack_from('<f', rpi_blob, 32)[0]
                    best_d = struct.unpack_from('<f', rpi_blob, 36)[0]
                else:
                    ratio = 0.0
                    pos_w = 0.0
                    neg_w = 0.0
                    best_d = delay_ms
                valid_sample = (
                    isfinite(delay_ms) and isfinite(best_d) and isfinite(ratio) and
                    isfinite(pos_w) and isfinite(neg_w) and
                    (0.0 <= delay_ms <= self._rpi_delay_max_ms) and
                    (0.0 <= best_d <= self._rpi_delay_max_ms) and
                    (-0.02 <= ratio <= 1.02) and
                    (abs(pos_w) <= 2.0 * self._rpi_power_abs_max_w) and
                    (abs(neg_w) <= 2.0 * self._rpi_power_abs_max_w)
                )
                if self._rpi_status_valid:
                    valid_sample = valid_sample and (
                        abs(delay_ms - self._rpi_delay_ms_L) <= self._rpi_delay_jump_guard_ms
                    )
                if valid_sample:
                    self._rpi_auto_delay_enable = bool(auto_flags & RPI_AUTO_FLAG_ENABLE)
                    self._rpi_auto_method_bo = bool(auto_flags & RPI_AUTO_FLAG_METHOD_BO)
                    self._rpi_auto_motion_valid_L = motion_valid
                    self._rpi_auto_motion_valid_R = motion_valid
                    self._rpi_delay_ms_L = delay_ms
                    self._rpi_delay_ms_R = delay_ms
                    self._rpi_power_ratio_L = ratio
                    self._rpi_power_ratio_R = ratio
                    self._rpi_pos_per_s_L = 0.5 * pos_w
                    self._rpi_pos_per_s_R = 0.5 * pos_w
                    self._rpi_neg_per_s_L = 0.5 * neg_w
                    self._rpi_neg_per_s_R = 0.5 * neg_w
                    self._rpi_best_delay_ms_L = best_d
                    self._rpi_best_delay_ms_R = best_d

            # Legacy aliases for any other code paths: expose averaged/combined.
            self._rpi_delay_ms = 0.5 * (self._rpi_delay_ms_L + self._rpi_delay_ms_R)
            self._rpi_auto_motion_valid = (
                self._rpi_auto_motion_valid_L or self._rpi_auto_motion_valid_R
            )
            self._rpi_power_ratio = 0.5 * (self._rpi_power_ratio_L + self._rpi_power_ratio_R)
            self._rpi_pos_per_s = self._rpi_pos_per_s_L + self._rpi_pos_per_s_R
            self._rpi_neg_per_s = self._rpi_neg_per_s_L + self._rpi_neg_per_s_R
            self._rpi_best_delay_ms = 0.5 * (self._rpi_best_delay_ms_L + self._rpi_best_delay_ms_R)

            self._rpi_last_rx_time = time.time()
            self._rpi_online = True
            prev_nn = getattr(self, '_prev_rpi_nn_type', -1)
            force_rl_status_refresh = False
            if not self._rpi_status_valid or rpi_blob[3] != prev_nn:
                self._rpi_status_valid = True
                self._prev_rpi_nn_type = rpi_blob[3]
                self._update_rl_panel_for_nn_type()
                force_rl_status_refresh = True
            self._maybe_update_rl_filter_state_label(now=now, force=force_rl_status_refresh)

        # === Update UI ===
        self.lbl_imu.setText(f"IMU: {'OK' if imu_ok_flag else 'FAIL'}")
        self.lbl_maxt.setText(f"MaxT: {maxT_rx:.1f} Nm")

        # SD badge
        if sd_ok:
            self.badge_sd.setText("SD: OK")
            self.badge_sd.set_color(C.green)
        else:
            self.badge_sd.setText("SD: FAIL")
            self.badge_sd.set_color(C.red)

        # IMU status
        labels = ["L", "R", "1", "2", "3", "4"]
        imu_angles = [L_angle, R_angle, tx1, tx2, tx3, tx4]
        self._imu34_counter += 1
        update_34 = (self._imu34_counter % 7 == 0)

        for i in range(6):
            ok = (imu_bits >> i) & 0x01
            ok_bool = bool(ok)
            if self._imu_ok_state_cache[i] != ok_bool:
                if ok_bool:
                    self.lbl_imus[i].setStyleSheet(
                        f"color:{C.green}; font-weight:600; font-size:11px; background:transparent;")
                else:
                    self.lbl_imus[i].setStyleSheet(
                        f"color:{C.red}; font-weight:500; font-size:11px; background:transparent;")
                self._imu_ok_state_cache[i] = ok_bool
            if ok:
                if i < 2 or update_34:
                    txt = f"{labels[i]}:{imu_angles[i]:.1f}"
                    if self.lbl_imus[i].text() != txt:
                        self.lbl_imus[i].setText(txt)
            else:
                txt = f"{labels[i]}:-"
                if self.lbl_imus[i].text() != txt:
                    self.lbl_imus[i].setText(txt)

        # IMU battery labels (payload[49..54] => BATT L/R/1/2/3/4).
        if hasattr(self, "lbl_imu_batts"):
            for i in range(6):
                b = imu_batt_raw[i]
                valid_batt = (0 <= b <= 100)
                if valid_batt:
                    if b >= 60:
                        color = C.green
                        weight = "600"
                    elif b >= 30:
                        color = C.orange
                        weight = "600"
                    else:
                        color = C.red
                        weight = "700"
                    txt = f"{labels[i]}:{b}%"
                else:
                    color = C.text2
                    weight = "500"
                    txt = f"{labels[i]}:--"

                style_sig = (color, weight)
                if self._imu_batt_style_cache[i] != style_sig:
                    self.lbl_imu_batts[i].setStyleSheet(
                        f"color:{color}; font-size:11px; font-weight:{weight}; background:transparent;"
                    )
                    self._imu_batt_style_cache[i] = style_sig

                if self._imu_batt_cache[i] != txt:
                    self.lbl_imu_batts[i].setText(txt)
                    self._imu_batt_cache[i] = txt

        # Brand badge
        brand_names = {0: "-", 1: "SIG", 2: "TMOTOR"}
        bname = brand_names.get(brand_id, '?')
        self._current_brand = brand_id
        self._update_brand_apply_label()
        self.badge_brand.setText(f"Current:{bname}")
        if brand_id in (1, 2):
            if self._brand_pending in (1, 2) and self._brand_pending != brand_id:
                self.badge_brand.set_color(C.orange)
            else:
                self.badge_brand.set_color(C.green)
        else:
            self.badge_brand.set_color(C.fill)
        if brand_id == 2 and temp_L:
            self.lbl_temp_L.setText(f"L:{temp_L}C")
            self.lbl_temp_R.setText(f"R:{temp_R}C" if temp_R else "")
        else:
            self.lbl_temp_L.setText("")
            self.lbl_temp_R.setText("")

        # Algorithm badge
        algo_name = ALGO_NAMES.get(active_algo, "?")
        if active_algo == self._algo_select:
            self.badge_algo.setText(algo_name)
            self.badge_algo.set_color(C.green)
        else:
            pending = ALGO_NAMES.get(self._algo_pending, "?")
            self.badge_algo.setText(f"{algo_name} -> {pending}")
            self.badge_algo.set_color(C.orange)

        # Status
        (L_angle_disp, R_angle_disp, L_vel_disp, R_vel_disp,
         L_cmd_disp, R_cmd_disp, L_pwr_disp, R_pwr_disp,
         signal_active, power_active) = self._resolve_live_sources(active_algo)
        self.lbl_status.setText(
            f"t={t:.2f}s  gait={gait_freq:.2f}Hz  algo={algo_name}"
            + (f"  tag='{tag_char}'" if tag_valid else "")
            + f"  vel={'IMU' if has_imu_vel else 'DER'}"
            + f"  src={signal_active}/{power_active}")

        self._append_data_point(
            t,
            L_angle_disp, R_angle_disp,
            L_tau, R_tau,
            L_cmd_disp, R_cmd_disp,
            L_vel_disp, R_vel_disp,
            gait_freq,
            log_csv=True,
            power_override=(L_pwr_disp, R_pwr_disp),
        )


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')  # Cross-platform clean base
    # Antialiasing is expensive for continuously updated curves.
    pg.setConfigOptions(antialias=False)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
