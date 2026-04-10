import serial
from serial.tools import list_ports
import struct
import time
import csv
from math import *
from collections import deque
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
ALGO_NAMES = {ALGO_EG: "EG", ALGO_SAMSUNG: "Samsung", ALGO_RL: "RL", ALGO_TEST: "Test"}

# RPi passthrough payload format (40 bytes, packed into BLE payload[58:98])
RPI_PT_MAGIC0 = 0x52  # 'R'
RPI_PT_MAGIC1 = 0x4C  # 'L'
RPI_PT_VERSION = 0x01            # downlink (GUI → RPi) passthrough version
RPI_STATUS_VERSION_LEGACY = 0x02  # uplink status v2: single-leg float32 metrics
RPI_STATUS_VERSION_PER_LEG = 0x03  # uplink status v3: per-leg int16-packed metrics

# Filter type codes (keep in sync with RL_controller_torch.py)
RL_FILTER_TYPES = [
    ("Butterworth", 1),
    ("Bessel", 2),
    ("Chebyshev2", 3),
]

CONN_TIMEOUT_S = 2.0

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
    ports = [p.device for p in list_ports.comports()]
    # Keep stable order so the selection does not jump around.
    return sorted(ports)


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
        self._update_counter = 0
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
        self._brand_pending = 2  # 1=SIG, 2=TMOTOR (default: TMOTOR)
        self._current_brand = 0
        self._algo_select = ALGO_EG
        self._algo_pending = ALGO_EG
        self._current_tag = ""
        self._last_rx_tag_char = ""
        self._tag_started_at = None
        self._last_gait_freq = None
        self._pwr_band_scale_right = 5.0
        self._pwr_band_scale_left = 5.0
        self._rl_cfg_tx_seq = 0
        self._rl_last_tx_ts = 0.0

        # RPi status (received via BLE uplink passthrough)
        self._rpi_nn_type = -1          # -1=unknown, 0=dnn, 1=lstm, 2=lstm_leg_dcp, 3=lstm_pd
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

        # Connection health
        self._last_rx_time = 0.0
        self._conn_healthy = False
        self._imu34_counter = 0
        self._rx_buf = bytearray()
        self._last_port_scan = 0.0

        # Screenshot & recording
        self._capture_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")
        self._recording = False
        self._record_tmp_dir = None
        self._record_frame_idx = 0
        self._record_timer = None

        # Collect updatable themed widgets
        self._cards = []
        self._segmented_ctrls = []
        self._combo_boxes = []

        self._build_layout()
        self._update_rl_filter_state_label()
        self._update_brand_apply_label()
        self._build_plots()
        self._apply_plot_visibility()
        self._build_value_displays()

        self._maxT_before_off = 12.0
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
            'L_cmd_Nm', 'R_cmd_Nm', 'L_est_Nm', 'R_est_Nm',
            'L_vel_dps', 'R_vel_dps', 'L_pwr_W', 'R_pwr_W',
            'gait_freq_Hz',
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
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setStyleSheet("QScrollArea { background: transparent; border: none; }")
        left_widget = QWidget()
        left_widget.setStyleSheet("background: transparent;")
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
        self._setup_combo(self.cmb_port)
        conn_lay.addWidget(self.cmb_port)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setFixedHeight(32)
        self.btn_connect.setCursor(Qt.PointingHandCursor)
        self.btn_connect.clicked.connect(self._connect_clicked)
        conn_lay.addWidget(self.btn_connect)

        conn_lay.addStretch(1)

        # Eco toggle
        conn_lay.addWidget(QLabel("Eco"))
        self.sw_eco = ToggleSwitch(checked=False, on_color=C.green)
        conn_lay.addWidget(self.sw_eco)

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

        self.seg_algo = SegmentedControl(["EG", "Samsung", "RL", "Test"])
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

        # Max Torque row
        mt_row = QHBoxLayout()
        mt_row.addWidget(QLabel("Max Torque (Nm)"))
        mt_row.addStretch(1)

        def make_dspin(val=0.0, mn=-20, mx=20, step=0.01, dec=2, tip=""):
            sb = QDoubleSpinBox()
            sb.setDecimals(dec); sb.setSingleStep(step)
            sb.setRange(mn, mx); sb.setValue(val)
            sb.setToolTip(tip)
            sb.valueChanged.connect(self._tx_params)
            return sb

        self.sb_max_torque_cfg = make_dspin(12.0, 0.0, 30.0, 0.1, 1)
        self.sb_max_torque_cfg.setFixedWidth(90)
        mt_row.addWidget(self.sb_max_torque_cfg)
        param_lay.addLayout(mt_row)

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
        self.sb_Flex_Assist_gain  = make_dspin(1.0, -2, 2, 0.01, 2)
        self.sb_Ext_Assist_gain   = make_dspin(1.0, -2, 2, 0.01, 2)
        self.sb_gate_k            = make_dspin(1.0, 0, 10, 0.1, 2)
        self.sb_gate_p_on         = make_dspin(8, 0, 50, 1, 2)
        self.sb_scale_all         = make_dspin(0.20, -1.0, 1.0, 0.01, 2)
        self.sb_ext_phase_frac_L  = make_dspin(0.300, 0.0, 0.500, 0.001, 3)
        self.sb_ext_phase_frac_R  = make_dspin(0.300, 0.0, 0.500, 0.001, 3)
        self.sb_ext_gain          = make_dspin(0.50, -3.0, 3.0, 0.01, 2)
        self.sb_Assist_delay_gain = QSpinBox()
        self.sb_Assist_delay_gain.setRange(0, 99); self.sb_Assist_delay_gain.setValue(40)
        self.sb_Assist_delay_gain.setToolTip(
            "Assist_delay_gain phase index (0..99), auto-scaled by gait frequency (not fixed ms)"
        )
        self.sb_Assist_delay_gain.valueChanged.connect(self._tx_params)

        eg_labels = ["R Gain", "L Gain", "gate_k", "gate_p_on",
                     "scale_all", "ext_frac_L", "ext_frac_R", "ext_gain", "Delay idx(phase)"]
        eg_spins  = [self.sb_Flex_Assist_gain, self.sb_Ext_Assist_gain,
                     self.sb_gate_k, self.sb_gate_p_on, self.sb_scale_all,
                     self.sb_ext_phase_frac_L, self.sb_ext_phase_frac_R,
                     self.sb_ext_gain, self.sb_Assist_delay_gain]
        for i, (lb, sb) in enumerate(zip(eg_labels, eg_spins)):
            r, c = divmod(i, 3)
            lbl = QLabel(lb)
            lbl.setStyleSheet(f"color:{C.text2}; font-size:11px; background:transparent;")
            eg_grid.addWidget(lbl, r*2, c)
            eg_grid.addWidget(sb, r*2+1, c)
        self.algo_stack.addWidget(eg_panel)

        # -- Samsung panel --
        sam_panel = QWidget()
        sam_panel.setStyleSheet("background:transparent;")
        sam_grid = QGridLayout(sam_panel)
        sam_grid.setSpacing(4)
        self.sb_sam_kappa = make_dspin(3.0, 0, 20, 0.1, 1)
        self.sb_sam_delay = make_dspin(250, 0, 1500, 10, 0)
        sam_grid.addWidget(QLabel("Kappa"), 0, 0)
        sam_grid.addWidget(self.sb_sam_kappa, 0, 1)
        sam_grid.addWidget(QLabel("Delay (ms)"), 1, 0)
        sam_grid.addWidget(self.sb_sam_delay, 1, 1)
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
        rl_lay.addWidget(QLabel("RL Scale (L/R)"), 2, 0)
        rl_lay.addWidget(self.sb_rl_scale, 2, 1)

        # Row 3: Delay (step=10ms, staged)
        self.sb_rl_torque_delay = make_dspin(0.0, 0.0, 1000.0, 10.0, 0,
                                             "RL torque delay absolute (ms), 0~1000")
        self.sb_rl_torque_delay.valueChanged.disconnect(self._tx_params)
        rl_lay.addWidget(QLabel("Torque Delay (ms)"), 3, 0)
        rl_lay.addWidget(self.sb_rl_torque_delay, 3, 1)

        # Row 4: Filter Type
        self.cmb_rl_filter_type = QComboBox()
        self.cmb_rl_filter_type.addItems([x[0] for x in RL_FILTER_TYPES])
        self._setup_combo(self.cmb_rl_filter_type)
        self.lbl_rl_filter_type = QLabel("Filter Type")
        rl_lay.addWidget(self.lbl_rl_filter_type, 4, 0)
        rl_lay.addWidget(self.cmb_rl_filter_type, 4, 1)

        # Row 5: Filter Cutoff (default 5Hz, staged)
        self.sb_rl_cutoff_hz = make_dspin(5.0, 0.5, 30.0, 0.5, 1,
                                          "RL runtime filter cutoff (Hz)")
        self.sb_rl_cutoff_hz.valueChanged.disconnect(self._tx_params)
        self.lbl_rl_cutoff = QLabel("Filter Cutoff (Hz)")
        rl_lay.addWidget(self.lbl_rl_cutoff, 5, 0)
        rl_lay.addWidget(self.sb_rl_cutoff_hz, 5, 1)

        # Row 6: Filter Enable checkboxes (DNN: Vel+Ref + Torque; LSTM: only Torque)
        filt_row = QHBoxLayout()
        self.chk_rl_vr_filter = QCheckBox("Vel+Ref")
        self.chk_rl_torque_filter = QCheckBox("Torque")
        self.chk_rl_auto_delay = QCheckBox("Auto Delay")
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
        rl_lay.addWidget(self.lbl_rl_filter_en, 6, 0)
        rl_lay.addLayout(filt_row, 6, 1)

        # Row 7: Apply RL button (staged send, not realtime)
        self.btn_apply_rl = QPushButton("Apply RL Settings")
        self.btn_apply_rl.setStyleSheet(
            f"background-color:{C.blue}; color:white; font-weight:600; "
            f"border-radius:8px; padding:8px 16px; font-size:13px;")
        self.btn_apply_rl.clicked.connect(self._on_apply_rl_clicked)
        rl_lay.addWidget(self.btn_apply_rl, 7, 0, 1, 2)

        # Row 8: Status label
        self.lbl_rl_filter_state = QLabel("")
        self.lbl_rl_filter_state.setWordWrap(True)
        self.lbl_rl_filter_state.setStyleSheet(
            f"color:{C.text2}; font-size:11px; background:transparent; padding-top:2px;"
        )
        rl_lay.addWidget(self.lbl_rl_filter_state, 8, 0, 1, 2)

        # Row 9: Auto-delay telemetry (from RPi status uplink)
        self.lbl_rl_auto_state = QLabel("Auto Delay: waiting for RPi status...")
        self.lbl_rl_auto_state.setWordWrap(True)
        self.lbl_rl_auto_state.setStyleSheet(
            f"color:{C.purple}; font-size:11px; background:transparent; padding-top:1px;"
        )
        rl_lay.addWidget(self.lbl_rl_auto_state, 9, 0, 1, 2)
        self.algo_stack.addWidget(rl_panel)

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
        send_row.addWidget(self.btn_auto_mile)
        self._mile_values = ['sit-to-stand', 'walk', 'run', 'squat']
        self._mile_index = 0

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

        vsep1 = QFrame(); vsep1.setFrameShape(QFrame.VLine)
        vsep1.setStyleSheet(f"background-color:{C.separator}; max-width:1px; border:none;")
        vsep1.setFixedHeight(20)
        row1.addWidget(vsep1)

        self.btn_clear_buf = QPushButton("Clear")
        self.btn_clear_buf.setCursor(Qt.PointingHandCursor)
        self.btn_clear_buf.setFixedHeight(24)
        self.btn_clear_buf.clicked.connect(self._clear_buffers)
        row1.addWidget(self.btn_clear_buf)

        lbl_w = QLabel("Win:")
        lbl_w.setStyleSheet("font-size:12px; background:transparent;")
        row1.addWidget(lbl_w)
        self.sb_win_size = QSpinBox()
        self.sb_win_size.setRange(50, 2000)
        self.sb_win_size.setValue(self.win_size)
        self.sb_win_size.setFixedWidth(70)
        self.sb_win_size.setFixedHeight(24)
        self.sb_win_size.valueChanged.connect(self._on_win_size_changed)
        row1.addWidget(self.sb_win_size)

        lbl_a = QLabel("Auto")
        lbl_a.setStyleSheet("font-size:12px; background:transparent;")
        row1.addWidget(lbl_a)
        self.sw_auto_scroll = ToggleSwitch(checked=True, on_color=C.blue)
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
        self.btn_toggle_L = _dir_btn("L+", "Real L direction")
        self.btn_toggle_R = _dir_btn("R+", "Real R direction")
        self.btn_toggle_L.clicked.connect(self._toggle_left_dir)
        self.btn_toggle_R.clicked.connect(self._toggle_right_dir)
        row2.addWidget(self.btn_toggle_L)
        row2.addWidget(self.btn_toggle_R)

        lbl_vis = QLabel("Visual:")
        lbl_vis.setStyleSheet("font-size:12px; background:transparent;")
        row2.addWidget(lbl_vis)
        self.btn_visual_L = _dir_btn("VL+", "Visual L (plot only)")
        self.btn_visual_R = _dir_btn("VR+", "Visual R (plot only)")
        self.btn_visual_L.clicked.connect(self._toggle_visual_L)
        self.btn_visual_R.clicked.connect(self._toggle_visual_R)
        row2.addWidget(self.btn_visual_L)
        row2.addWidget(self.btn_visual_R)

        self.badge_original = PillBadge("Original", C.green)
        row2.addWidget(self.badge_original)

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
        self.btn_screenshot.clicked.connect(self._take_screenshot)
        row2.addWidget(self.btn_screenshot)

        self.btn_record = QPushButton("Record")
        self.btn_record.setCursor(Qt.PointingHandCursor)
        self.btn_record.setFixedHeight(26)
        self._apply_record_idle_style()
        self.btn_record.clicked.connect(self._toggle_recording)
        row2.addWidget(self.btn_record)

        row2.addStretch(1)

        plot_card_vlay.addLayout(row2)

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

        # Motor row
        motor_row = QHBoxLayout()
        self.btn_motor_init = QPushButton("Motor Init")
        self.btn_motor_init.setCursor(Qt.PointingHandCursor)
        self.btn_motor_init.setFixedHeight(28)
        self.btn_motor_init.clicked.connect(self._on_click_motor_init)
        motor_row.addWidget(self.btn_motor_init)

        motor_row.addWidget(QLabel("Brand:"))
        self.cmb_motor_brand = QComboBox()
        self.cmb_motor_brand.addItems(["SIG", "TMOTOR"])
        self.cmb_motor_brand.setFixedWidth(100)
        self._setup_combo(self.cmb_motor_brand)
        self.cmb_motor_brand.currentIndexChanged.connect(self._on_brand_changed)
        motor_row.addWidget(self.cmb_motor_brand)

        self.btn_brand_apply = QPushButton("Apply SIG")
        self.btn_brand_apply.setCursor(Qt.PointingHandCursor)
        self.btn_brand_apply.setFixedHeight(28)
        self.btn_brand_apply.clicked.connect(self._on_brand_confirm)
        motor_row.addWidget(self.btn_brand_apply)

        self.badge_brand = PillBadge("-", C.fill)
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
        ports = find_available_ports()
        selected = self.cmb_port.currentText()
        self.cmb_port.blockSignals(True)
        self.cmb_port.clear()
        if ports:
            self.cmb_port.addItems(ports)
            idx = self.cmb_port.findText(selected)
            self.cmb_port.setCurrentIndex(idx if idx >= 0 else 0)
        self.cmb_port.blockSignals(False)
        self._last_port_scan = time.time()

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
            auto_flags |= 0x01
        pt[20] = auto_flags & 0xFF
        return pt

    def _on_apply_rl_clicked(self):
        """Apply RL button: send staged RL params to RPi via BLE passthrough."""
        if not (self.connected and self.ser):
            return
        self._tx_params()
        self._update_rl_filter_state_label()
        # Re-affirm motor direction after RPi params are sent
        QTimer.singleShot(150, self._tx_params)

    def _on_rl_auto_delay_toggled(self, _state):
        self._update_rl_delay_input_mode()
        self._update_rl_filter_state_label()

    def _set_rl_delay_spinbox_value(self, delay_ms: float):
        if not hasattr(self, "sb_rl_torque_delay"):
            return
        delay = max(0.0, min(1000.0, float(delay_ms)))
        if abs(float(self.sb_rl_torque_delay.value()) - delay) < 0.05:
            return
        self.sb_rl_torque_delay.blockSignals(True)
        self.sb_rl_torque_delay.setValue(delay)
        self.sb_rl_torque_delay.blockSignals(False)

    def _update_rl_delay_input_mode(self):
        """Auto Delay ON: lock delay input and mirror active delay from RPi status."""
        if not hasattr(self, "sb_rl_torque_delay") or not hasattr(self, "chk_rl_auto_delay"):
            return
        auto_on = self.chk_rl_auto_delay.isChecked()
        self.sb_rl_torque_delay.setEnabled(not auto_on)
        if auto_on:
            self.sb_rl_torque_delay.setToolTip("Auto Delay ON: follows RPi active runtime delay.")
            if self._rpi_status_valid:
                self._set_rl_delay_spinbox_value(self._rpi_delay_ms)
        else:
            self.sb_rl_torque_delay.setToolTip("RL torque delay absolute (ms), 0~1000")

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
        # nn=0: DNN     → scale=0.50, delay=200ms, cutoff=2.5Hz, Butterworth, Vel+Ref+Torque ON
        # nn=1: LSTM    → scale=1.00, delay=0ms,   cutoff=5.0Hz, Butterworth, Torque ON
        # nn=2: LegDcp  → scale=1.00, delay=100ms, cutoff=5.0Hz, Butterworth, Torque ON
        # nn=3: LSTM-PD → scale=1.00, delay=200ms, cutoff=2.5Hz, Butterworth, Torque ON
        _presets = {
            0: dict(scale=0.50, delay=200.0, cutoff=2.5,  vr=True,  torque=True),
            1: dict(scale=1.00, delay=0.0,   cutoff=5.0,  vr=False, torque=True),
            2: dict(scale=1.00, delay=100.0, cutoff=5.0,  vr=False, torque=True),
            3: dict(scale=1.00, delay=200.0, cutoff=2.5,  vr=False, torque=True),
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
        self.lbl_rpi_nn_type.setText("RPi: Offline")
        self.lbl_rpi_nn_type.setStyleSheet(
            f"color:{C.red}; font-size:13px; font-weight:600; background:transparent;")
        self.lbl_rpi_current_filter.setText("RPi Active: no connection")
        if hasattr(self, "lbl_rl_auto_state"):
            self.lbl_rl_auto_state.setText("Auto Delay: no connection")
        self._update_power_strip_titles()

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

            # 非 auto 模式下 L/R 同步; auto 模式下可能分离 → 显示 "L/R" 对
            if abs(self._rpi_delay_ms_L - self._rpi_delay_ms_R) < 0.5:
                delay_str = f"Delay={self._rpi_delay_ms_L:.0f}ms"
            else:
                delay_str = f"Delay L/R={self._rpi_delay_ms_L:.0f}/{self._rpi_delay_ms_R:.0f}ms"
            if self._rpi_nn_type == 0:  # DNN
                current_str = (
                    f"[{src}] {ftype} {self._rpi_cutoff_hz:.1f}Hz order={self._rpi_filter_order} | "
                    f"Vel={en_v} Ref={en_r} Torque={en_t} | "
                    f"Scale={self._rpi_scale:.2f} {delay_str} | Auto={auto_en}"
                )
            else:  # LSTM
                current_str = (
                    f"[{src}] Torque Filter={en_t}"
                )
                if en_t == "ON" and self._rpi_filter_type_code > 0:
                    current_str += f" {ftype} {self._rpi_cutoff_hz:.1f}Hz order={self._rpi_filter_order}"
                current_str += f" | Scale={self._rpi_scale:.2f} {delay_str} | Auto={auto_en}"
            self.lbl_rpi_current_filter.setText(f"RPi Active: {current_str}")
        else:
            self.lbl_rpi_current_filter.setText("RPi Active: waiting for status...")

        # --- GUI pending/sent display ---
        idx = max(0, min(self.cmb_rl_filter_type.currentIndex(), len(RL_FILTER_TYPES) - 1))
        filt_name = RL_FILTER_TYPES[idx][0]
        vr = "ON" if self.chk_rl_vr_filter.isChecked() else "OFF"
        tq = "ON" if self.chk_rl_torque_filter.isChecked() else "OFF"
        ad = "ON" if self.chk_rl_auto_delay.isChecked() else "OFF"
        cutoff = float(self.sb_rl_cutoff_hz.value())
        delay = float(self.sb_rl_torque_delay.value())
        scale = float(self.sb_rl_scale.value())
        if self._algo_select == ALGO_RL:
            if self.connected and self.ser:
                state = (
                    f"GUI Sent(seq={self._rl_cfg_tx_seq}): "
                    f"{filt_name} {cutoff:.1f}Hz, Vel+Ref={vr}, Torque={tq}, "
                    f"Scale={scale:.2f}, Delay={delay:.0f}ms, Auto={ad}"
                )
            else:
                state = (
                    f"GUI Pending: "
                    f"{filt_name} {cutoff:.1f}Hz, Vel+Ref={vr}, Torque={tq}, "
                    f"Scale={scale:.2f}, Delay={delay:.0f}ms, Auto={ad}"
                )
        else:
            state = f"Algo={ALGO_NAMES.get(self._algo_select, '?')}. Override sent only when RL active."
        self.lbl_rl_filter_state.setText(state)

        if hasattr(self, "lbl_rl_auto_state"):
            if self._rpi_status_valid:
                auto_state = "ON" if self._rpi_auto_delay_enable else "OFF"
                mL = "VALID" if self._rpi_auto_motion_valid_L else "HOLD"
                mR = "VALID" if self._rpi_auto_motion_valid_R else "HOLD"
                self.lbl_rl_auto_state.setText(
                    f"Auto Delay(RPi): {auto_state}\n"
                    f"  L: motion={mL} | ratio={self._rpi_power_ratio_L:.3f} | "
                    f"+P={self._rpi_pos_per_s_L:+.2f}W | -P={self._rpi_neg_per_s_L:+.2f}W | "
                    f"delay={self._rpi_delay_ms_L:.0f}ms | best={self._rpi_best_delay_ms_L:.0f}ms\n"
                    f"  R: motion={mR} | ratio={self._rpi_power_ratio_R:.3f} | "
                    f"+P={self._rpi_pos_per_s_R:+.2f}W | -P={self._rpi_neg_per_s_R:+.2f}W | "
                    f"delay={self._rpi_delay_ms_R:.0f}ms | best={self._rpi_best_delay_ms_R:.0f}ms"
                )
            else:
                self.lbl_rl_auto_state.setText("Auto Delay: waiting for RPi status...")
        self._update_power_strip_titles()

    # ================================================================ Test waveform
    def _on_test_waveform_changed(self, index):
        is_sin = (index == 1)
        self.lbl_test_freq.setVisible(is_sin)
        self.sb_test_freq.setVisible(is_sin)
        self._tx_params()

    # ================================================================ Algorithm
    def _on_algo_selected(self, index):
        self._algo_pending = index
        self.algo_stack.setCurrentIndex(index)
        self._update_rl_filter_state_label()

    def _on_algo_confirm(self):
        self._algo_select = self._algo_pending
        print(f"[GUI] Algorithm CONFIRMED: {ALGO_NAMES.get(self._algo_select, '?')}")
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
        self.lbl_status.setText(f"Brand selected: {target}. Click Apply Brand.")

    def _update_brand_apply_label(self):
        if not hasattr(self, "btn_brand_apply"):
            return
        target = "SIG" if self._brand_pending == 1 else "TMOTOR"
        if self._current_brand in (1, 2) and self._current_brand == self._brand_pending:
            self.btn_brand_apply.setText(f"{target} Active")
        else:
            self.btn_brand_apply.setText(f"Apply {target}")

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

    def _on_click_imu_init(self):
        if not (self.connected and self.ser): return
        self._imu_init_request = True
        self._tx_params()
        self.ser.flush()
        QTimer.singleShot(100, lambda: setattr(self, "_imu_init_request", False))

    def _on_click_motor_init(self):
        if not (self.connected and self.ser): return
        self._motor_init_request = True
        self._tx_params()
        self.ser.flush()
        QTimer.singleShot(100, lambda: setattr(self, "_motor_init_request", False))

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
            val = self._maxT_before_off if self._maxT_before_off > 0.0 else 12.0
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

    def _clear_buffers(self):
        self._init_buffers()
        self._prev_L_angle = 0.0
        self._prev_R_angle = 0.0
        self._prev_wall_time = 0.0

    def _on_win_size_changed(self, val):
        self.win_size = val
        for attr in ['t_buffer', 'L_IMU_buf', 'R_IMU_buf', 'L_tau_buf', 'R_tau_buf',
                     'L_tau_d_buf', 'R_tau_d_buf', 'L_vel_buf', 'R_vel_buf',
                     'L_pwr_buf', 'R_pwr_buf']:
            old = getattr(self, attr)
            setattr(self, attr, deque(old, maxlen=val))

    # ================================================================ Theme
    def _on_theme_toggled(self, checked):
        self._dark_mode = checked
        self.lbl_theme_icon.setText("Dark" if checked else "Light")
        self._apply_theme()

    def _apply_theme(self):
        global C
        C = AppleColors.Dark if self._dark_mode else AppleColors.Light

        qss = _build_qss(C)
        self.setStyleSheet(qss)

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
        self._update_power_strip_titles()
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
        self.sw_eco._on_color = C.green
        self.sw_eco._off_color = C.fill
        self.sw_eco.update()
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
        if hasattr(self, "lbl_rl_filter_state"):
            self.lbl_rl_filter_state.setStyleSheet(
                f"color:{C.text2}; font-size:11px; background:transparent; padding-top:2px;"
            )
            self._update_rl_filter_state_label()

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

    def _start_recording(self):
        os.makedirs(self._capture_dir, exist_ok=True)
        self._record_tmp_dir = tempfile.mkdtemp(prefix="gui_rec_")
        self._record_frame_idx = 0
        self._recording = True
        self._record_start_time = time.time()
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
        self._record_timer.setInterval(66)  # ~15 fps
        self._record_timer.timeout.connect(self._capture_frame)
        self._record_timer.start()
        self.lbl_status.setText("Recording...")

    def _capture_frame(self):
        if not self._recording or not self._record_tmp_dir:
            return
        pixmap = self.grab()
        path = os.path.join(self._record_tmp_dir, f"frame_{self._record_frame_idx:06d}.png")
        pixmap.save(path, "PNG")
        self._record_frame_idx += 1
        elapsed = time.time() - self._record_start_time
        mm = int(elapsed // 60)
        ss = elapsed - mm * 60
        self.lbl_status.setText(f"Recording... {mm:02d}:{ss:04.1f}  ({self._record_frame_idx} frames)")

    def _stop_recording(self):
        self._recording = False
        if self._record_timer:
            self._record_timer.stop()
            self._record_timer = None
        self.btn_record.setText("Record")
        self._apply_record_idle_style()

        if self._record_frame_idx == 0:
            if self._record_tmp_dir:
                shutil.rmtree(self._record_tmp_dir, ignore_errors=True)
            self.lbl_status.setText("Recording cancelled (no frames)")
            return

        ts = datetime.now().strftime("recording_%Y%m%d_%H%M%S")
        mp4_path = os.path.join(self._capture_dir, ts + ".mp4")
        frame_pattern = os.path.join(self._record_tmp_dir, "frame_%06d.png")
        elapsed = time.time() - self._record_start_time
        dur_str = f"{int(elapsed // 60):02d}:{elapsed % 60:04.1f}"

        try:
            subprocess.run(
                ["ffmpeg", "-y", "-framerate", "15",
                 "-i", frame_pattern,
                 "-c:v", "libx264", "-pix_fmt", "yuv420p",
                 "-crf", "18", mp4_path],
                check=True, capture_output=True, timeout=120)
            shutil.rmtree(self._record_tmp_dir, ignore_errors=True)
            msg = f"Recording saved!\n\nFile: {ts}.mp4\nDuration: {dur_str}\nFrames: {self._record_frame_idx}"
            self.lbl_status.setText(f"Recording saved: {ts}.mp4 ({dur_str})")
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
        except (FileNotFoundError, subprocess.SubprocessError):
            fallback = os.path.join(self._capture_dir, ts)
            shutil.move(self._record_tmp_dir, fallback)
            msg = (f"ffmpeg not found, frames saved as images.\n\n"
                   f"Folder: {ts}/\nFrames: {self._record_frame_idx}\nDuration: {dur_str}")
            self.lbl_status.setText(f"Frames saved: {ts}/ ({self._record_frame_idx} frames)")
            QMessageBox.warning(self, "Recording Saved (frames)", msg)

        self._record_tmp_dir = None

    def _reset_record_btn(self):
        self.btn_record.setText("Record")
        self._apply_record_idle_style()

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
        self._update_power_strip_titles()

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

    def _update_power_strip_titles(self):
        """Keep strip titles static, and show per-leg ratio/power via in-plot overlay."""
        if not hasattr(self, 'pwr_strip_right') or not hasattr(self, 'pwr_strip_left'):
            return
        self.pwr_strip_right.setTitle("Right Leg Power Sign", color=C.text2, size='10pt')
        self.pwr_strip_left.setTitle("Left Leg Power Sign", color=C.text2, size='10pt')

        def _format_leg_overlay(side_label, ratio, pos_w, neg_w, delay_ms,
                                motion_valid, valid):
            if not valid:
                line1 = f"+Ratio --.-% | WAIT"
                line2 = f"+P --.-- W  -P --.-- W"
                line3 = f"delay --.- ms"
                return line1, line2, line3
            ratio_clamped = max(0.0, min(1.0, float(ratio)))
            ratio_pct = ratio_clamped * 100.0
            auto_txt = "ON" if self._rpi_auto_delay_enable else "OFF"
            motion_txt = "VALID" if motion_valid else "HOLD"
            line1 = f"+Ratio {ratio_pct:.1f}% | {auto_txt}/{motion_txt}"
            line2 = f"+P {float(pos_w):+.2f} W  -P {float(neg_w):+.2f} W"
            line3 = f"delay {float(delay_ms):.1f} ms"
            return line1, line2, line3

        valid = bool(self._rpi_status_valid)
        l1_L, l2_L, l3_L = _format_leg_overlay(
            "L", self._rpi_power_ratio_L, self._rpi_pos_per_s_L,
            self._rpi_neg_per_s_L, self._rpi_delay_ms_L,
            self._rpi_auto_motion_valid_L, valid,
        )
        l1_R, l2_R, l3_R = _format_leg_overlay(
            "R", self._rpi_power_ratio_R, self._rpi_pos_per_s_R,
            self._rpi_neg_per_s_R, self._rpi_delay_ms_R,
            self._rpi_auto_motion_valid_R, valid,
        )

        overlay_tmpl = (
            f"<div style='color:{{col}}; font-size:15pt; font-weight:600; "
            f"background:rgba(0,0,0,0); text-align:right;'>"
            f"{{l1}}<br/>{{l2}}<br/>{{l3}}</div>"
        )
        html_L = overlay_tmpl.format(col=C.purple, l1=l1_L, l2=l2_L, l3=l3_L)
        html_R = overlay_tmpl.format(col=C.purple, l1=l1_R, l2=l2_R, l3=l3_R)

        if hasattr(self, 'pwr_strip_left_overlay'):
            self.pwr_strip_left_overlay.setHtml(html_L)
            self._position_power_ratio_overlay(self.pwr_strip_left, self.pwr_strip_left_overlay)
        if hasattr(self, 'pwr_strip_right_overlay'):
            self.pwr_strip_right_overlay.setHtml(html_R)
            self._position_power_ratio_overlay(self.pwr_strip_right, self.pwr_strip_right_overlay)

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
        y = y1 - 0.02 * dy
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
    def _connect_clicked(self):
        port = self.cmb_port.currentText()
        if not port:
            QMessageBox.warning(self, "Port", "No serial port found. Waiting for auto refresh.")
            return
        try:
            self.ser = serial.Serial(port, 115200, timeout=0)
            self.connected = True
            self._rx_buf.clear()
            self._last_rx_time = time.time()
            self._conn_healthy = True
            # Reset RPi state on new connection
            self._rpi_status_valid = False
            self._rpi_online = False
            self._rpi_last_rx_time = 0.0
            self._rpi_nn_type = -1
            self._rpi_status_version = 0
            self._rpi_auto_delay_enable = False
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
        except serial.SerialException:
            QMessageBox.critical(self, "Error", f"Cannot open {port}")

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
        elif algo == ALGO_SAMSUNG:
            put_s16(5, float(self.sb_sam_kappa.value()))
            put_s16(7, float(self.sb_sam_delay.value()), 1)
        elif algo == ALGO_RL:
            payload[58:98] = self._build_rl_passthrough()
        elif algo == ALGO_TEST:
            put_s16(5, float(self.sb_test_amplitude.value()))
            payload[7] = self.cmb_test_waveform.currentIndex() & 0xFF
            put_s16(8, float(self.sb_test_freq.value()))

        header = struct.pack('<BBB', 0xA5, 0x5A, BLE_FRAME_LEN)
        self.ser.write(header + payload)
        if algo == ALGO_RL:
            self._rl_cfg_tx_seq += 1
            self._rl_last_tx_ts = time.time()
        self._update_rl_filter_state_label()

    def _send_logtag(self):
        if not (self.connected and self.ser): return
        tag = self.edt_label.text().encode('ascii', 'ignore')[:10]
        tag_txt = tag.decode('ascii', 'ignore').strip()
        flags = 0x01 if self.sw_persist.isChecked() else 0x00
        payload = bytearray(BLE_PAYLOAD_LEN)
        payload[0] = ord('L')
        payload[1] = ord('G')
        payload[2] = len(tag)
        payload[13] = flags
        if len(tag):
            payload[3:3+len(tag)] = tag
        header = struct.pack('<BBB', 0xA5, 0x5A, BLE_FRAME_LEN)
        self.ser.write(header + payload)
        self._current_tag = tag_txt
        self._tag_started_at = time.time() if tag_txt else None
        self._update_tag_panel(self._tag_started_at if self._tag_started_at else time.time())

    # ================================================================ Main loop
    def _update_everything(self):
        now = time.time()
        self._update_tag_panel(now)
        # Refresh port list periodically while disconnected.
        if not self.connected:
            if now - self._last_port_scan > 1.0:
                self._refresh_ports()
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

    # ================================================================ RX
    def _read_serial(self):
        try:
            avail = self.ser.in_waiting
        except (serial.SerialException, OSError):
            self.connected = False
            self._conn_healthy = False
            self._rx_buf.clear()
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
                self._rpi_delay_ms_L = struct.unpack_from('<h', rpi_blob, 16)[0] / 10.0
                self._rpi_delay_ms_R = struct.unpack_from('<h', rpi_blob, 18)[0] / 10.0
                auto_flags = rpi_blob[20]
                self._rpi_auto_delay_enable = bool(auto_flags & 0x01)
                self._rpi_auto_motion_valid_L = bool(auto_flags & 0x02)
                self._rpi_auto_motion_valid_R = bool(auto_flags & 0x04)
                self._rpi_power_ratio_L = struct.unpack_from('<h', rpi_blob, 24)[0] / 10000.0
                self._rpi_power_ratio_R = struct.unpack_from('<h', rpi_blob, 26)[0] / 10000.0
                self._rpi_pos_per_s_L = struct.unpack_from('<h', rpi_blob, 28)[0] / 100.0
                self._rpi_pos_per_s_R = struct.unpack_from('<h', rpi_blob, 30)[0] / 100.0
                self._rpi_neg_per_s_L = struct.unpack_from('<h', rpi_blob, 32)[0] / 100.0
                self._rpi_neg_per_s_R = struct.unpack_from('<h', rpi_blob, 34)[0] / 100.0
                self._rpi_best_delay_ms_L = struct.unpack_from('<h', rpi_blob, 36)[0] / 10.0
                self._rpi_best_delay_ms_R = struct.unpack_from('<h', rpi_blob, 38)[0] / 10.0
            else:
                # v2 legacy: single-leg float32 — mirror to both L/R for display.
                delay_ms = struct.unpack_from('<f', rpi_blob, 16)[0]
                auto_flags = rpi_blob[20] if len(rpi_blob) > 20 else 0
                self._rpi_auto_delay_enable = bool(auto_flags & 0x01)
                motion_valid = bool(auto_flags & 0x02)
                self._rpi_auto_motion_valid_L = motion_valid
                self._rpi_auto_motion_valid_R = motion_valid
                self._rpi_delay_ms_L = delay_ms
                self._rpi_delay_ms_R = delay_ms
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
            if not self._rpi_status_valid or rpi_blob[3] != prev_nn:
                self._rpi_status_valid = True
                self._prev_rpi_nn_type = rpi_blob[3]
                self._update_rl_panel_for_nn_type()
            self._update_rl_filter_state_label()

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
            if ok:
                self.lbl_imus[i].setStyleSheet(
                    f"color:{C.green}; font-weight:600; font-size:11px; background:transparent;")
                if i < 2 or update_34:
                    self.lbl_imus[i].setText(f"{labels[i]}:{imu_angles[i]:.1f}")
            else:
                self.lbl_imus[i].setStyleSheet(
                    f"color:{C.red}; font-weight:500; font-size:11px; background:transparent;")
                self.lbl_imus[i].setText(f"{labels[i]}:-")

        # Brand badge
        brand_names = {0: "-", 1: "SIG", 2: "TMOTOR"}
        bname = brand_names.get(brand_id, '?')
        self._current_brand = brand_id
        self._update_brand_apply_label()
        self.badge_brand.setText(bname)
        self.badge_brand.set_color(C.blue if brand_id > 0 else C.fill)
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
        self.lbl_status.setText(
            f"t={t:.2f}s  gait={gait_freq:.2f}Hz  algo={algo_name}"
            + (f"  tag='{tag_char}'" if tag_valid else "")
            + f"  vel={'IMU' if has_imu_vel else 'DER'}")

        # === Buffers ===
        vL, vR = self._visual_sign_L, self._visual_sign_R
        self.t_buffer.append(t)
        self.L_IMU_buf.append(L_angle * vL)
        self.R_IMU_buf.append(R_angle * vR)
        self.L_tau_buf.append(L_tau * vL)
        self.R_tau_buf.append(R_tau * vR)
        self.L_tau_d_buf.append(L_tau_d * vL)
        self.R_tau_d_buf.append(R_tau_d * vR)
        self.L_vel_buf.append(L_vel * vL)
        self.R_vel_buf.append(R_vel * vR)
        # Power = vel (deg/s) * torque (Nm) * pi/180 -> Watts.
        # Use command torque for all brands for consistent sign and behavior.
        L_tau_for_pwr = L_tau_d
        R_tau_for_pwr = R_tau_d
        L_pwr = L_vel * L_tau_for_pwr * (pi / 180.0)
        R_pwr = R_vel * R_tau_for_pwr * (pi / 180.0)
        self.L_pwr_buf.append(L_pwr * vL)
        self.R_pwr_buf.append(R_pwr * vR)

        # === CSV logging (always on) ===
        if hasattr(self, '_csv_writer'):
            self._csv_writer.writerow([
                f'{t:.3f}',
                f'{L_angle:.3f}', f'{R_angle:.3f}',
                f'{L_tau_d:.4f}', f'{R_tau_d:.4f}',
                f'{L_tau:.4f}',   f'{R_tau:.4f}',
                f'{L_vel:.3f}',   f'{R_vel:.3f}',
                f'{L_pwr:.4f}',   f'{R_pwr:.4f}',
                f'{gait_freq:.3f}',
            ])
            now_wall = time.time()
            if now_wall - self._csv_last_flush > 1.0:
                self._csv_file.flush()
                self._csv_last_flush = now_wall

        self._update_counter += 1
        eco = self.sw_eco.isChecked()

        if not eco or self._update_counter % 2 == 0:
            tl = list(self.t_buffer)
            self.L_angle_line.setData(tl, list(self.R_IMU_buf))
            self.L_tau_line.setData(tl, list(self.R_tau_buf))
            self.L_tau_d_line.setData(tl, list(self.R_tau_d_buf))
            self.L_vel_line.setData(tl, list(self.R_vel_buf))
            self.L_pwr_line.setData(tl, list(self.R_pwr_buf))
            self.R_angle_line.setData(tl, list(self.L_IMU_buf))
            self.R_tau_line.setData(tl, list(self.L_tau_buf))
            self.R_tau_d_line.setData(tl, list(self.L_tau_d_buf))
            self.R_vel_line.setData(tl, list(self.L_vel_buf))
            self.R_pwr_line.setData(tl, list(self.L_pwr_buf))
            self._update_power_strip(tl, list(self.R_pwr_buf), 'right')
            self._update_power_strip(tl, list(self.L_pwr_buf), 'left')

            if self.sw_auto_scroll.isChecked() and len(tl) > 1:
                for pw in [self.plot_left, self.plot_right]:
                    pw.getViewBox().setXRange(tl[0], tl[-1], padding=0.02)

        if not eco or self._update_counter % 3 == 0:
            self.lbl_Lang.setText(f"L: {L_angle:.1f} deg")
            self.lbl_Rang.setText(f"R: {R_angle:.1f} deg")
            self.lbl_Lcmd.setText(f"L cmd: {L_tau_d:.1f} Nm")
            self.lbl_Rcmd.setText(f"R cmd: {R_tau_d:.1f} Nm")
            self.lbl_Ltau.setText(f"L est: {L_tau:.1f} Nm")
            self.lbl_Rtau.setText(f"R est: {R_tau:.1f} Nm")
            self.lbl_Lpwr.setText(f"L pwr: {L_pwr:.2f} W")
            self.lbl_Rpwr.setText(f"R pwr: {R_pwr:.2f} W")


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')  # Cross-platform clean base
    pg.setConfigOptions(antialias=True)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
