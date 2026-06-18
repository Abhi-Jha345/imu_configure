#!/usr/bin/env python3
"""
SBG Ellipse-D — graphical dashboard (PySide6).

Live debug + configuration GUI. It launches the C `sbg-bridge` helper as a
subprocess (which does all the serial I/O and sbgECom protocol decode) and
talks to it over line-delimited JSON (stdout) / text commands (stdin).

Run:
    python3 dashboard.py [SERIAL_DEVICE] [BAUDRATE]
Defaults: /dev/ttyUSB0 921600  (also editable in the toolbar before connecting)
"""

import json
import math
import os
import sys
from collections import deque

from PySide6.QtCore import Qt, QProcess, QPointF, QRectF, QTimer
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QFont, QPolygonF
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QComboBox, QLineEdit,
    QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox, QFrame, QSizePolicy,
    QDoubleSpinBox, QSpinBox, QCheckBox, QFormLayout, QMessageBox,
    QTabWidget, QScrollArea, QFileDialog,
)

BRIDGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bridge", "build", "sbg-bridge")

GNSS_FIX = {
    0: "NO SOLUTION", 1: "UNKNOWN", 2: "SINGLE", 3: "PSRDIFF/DGPS", 4: "SBAS",
    5: "OMNISTAR", 6: "RTK FLOAT", 7: "RTK FIXED", 8: "PPP FLOAT", 9: "PPP FIXED",
    10: "FIXED POS",
}
SOL_MODE = {
    0: "UNINITIALIZED", 1: "VERTICAL GYRO", 2: "AHRS", 3: "NAV VELOCITY", 4: "NAV POSITION",
}
MOTION_PROFILES = [
    ("General purpose", 1), ("Automotive", 2), ("Marine", 3), ("Airplane", 4),
    ("Helicopter", 5), ("Pedestrian", 6), ("UAV (rotary)", 7), ("Heavy machinery", 8),
    ("Static", 9), ("Truck", 10),
]
AXIS_DIRS = [("Forward", 0), ("Backward", 1), ("Left", 2), ("Right", 3), ("Up", 4), ("Down", 5)]
REJECTION = [("Never accept", 0), ("Automatic", 1), ("Always accept", 2)]
GNSS_MODELS = [("Internal (Ellipse-D)", 101), ("NMEA external", 102), ("u-blox external", 104),
               ("Novatel external", 106), ("Septentrio external", 109)]
MAG_MODELS = [("Internal normal", 201), ("Internal (v2 compat)", 202), ("ECOM external", 203)]
PORT_ASSIGN = [("PORT_A", 0), ("PORT_B", 1), ("PORT_C", 2), ("PORT_E", 4), ("Internal", 5), ("Disabled", 255)]
SYNC_ASSIGN = [("Internal", 5), ("Disabled", 255)]
TIME_REF = [("Disabled", 0), ("Sync In A", 1), ("UTC / GPS1", 2)]
SYNC_IN_SENS = [("Disabled", 0), ("Falling edge", 1), ("Rising edge", 2), ("Both edges", 3)]
SYNC_OUT_FUNC = [("Disabled", 0), ("Main loop 200Hz", 1), ("100Hz", 2), ("50Hz", 4),
                 ("25Hz", 8), ("20Hz", 10), ("10Hz", 20), ("5Hz", 40), ("2Hz", 100)]
SYNC_OUT_POL = [("Falling edge", 0), ("Rising edge", 1), ("Toggle", 2)]
CAN_BITRATE = [("Disabled", 0), ("250 kbit/s", 250), ("500 kbit/s", 500), ("1 Mbit/s", 1000),
               ("125 kbit/s", 125), ("100 kbit/s", 100), ("50 kbit/s", 50)]
CAN_MODE = [("Normal", 1), ("Spy (listen-only)", 2)]
UART_PORT = [("COM_A", 0), ("COM_B", 1), ("COM_C", 2), ("COM_D", 3), ("COM_E", 4)]
UART_MODE = [("Off", 0), ("RS-232", 1), ("RS-422", 2)]
OUTPUT_PORT = [("PORT_A", 0), ("PORT_C", 2), ("PORT_D", 3), ("PORT_E", 4)]
OUTPUT_MODE = [("Disabled", 0), ("200 Hz", 1), ("100 Hz", 2), ("50 Hz", 4), ("25 Hz", 8),
               ("20 Hz", 10), ("10 Hz", 20), ("5 Hz", 40), ("1 Hz", 200)]
LOG_IDS = [("STATUS", 1), ("UTC_TIME", 2), ("IMU_DATA", 3), ("MAG", 4), ("EKF_EULER", 6),
           ("EKF_NAV", 8), ("GPS1_VEL", 13), ("GPS1_POS", 14), ("GPS1_HDT", 15)]

# EKF solution status bits
SOL_ATT, SOL_HDG, SOL_VEL, SOL_POS = 1 << 4, 1 << 5, 1 << 6, 1 << 7
# General status bits
G_MAIN, G_IMU, G_GPS, G_SET, G_TEMP = 1, 1 << 1, 1 << 2, 1 << 3, 1 << 4

BG = "#1b1f27"
PANEL = "#262b36"
ACCENT = "#4aa3ff"


# ----------------------------------------------------------------------------
#  Custom-painted instruments
# ----------------------------------------------------------------------------
class AttitudeIndicator(QWidget):
    """Artificial horizon driven by roll & pitch (degrees)."""

    def __init__(self):
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_attitude(self, roll, pitch):
        self.roll, self.pitch = roll, pitch
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 6
        cx, cy = w / 2, h / 2

        p.setPen(Qt.NoPen)
        p.setBrush(QColor(BG))
        p.drawRect(0, 0, w, h)

        p.setClipRect(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        p.translate(cx, cy)
        p.rotate(-self.roll)
        pix_per_deg = r / 45.0
        offset = self.pitch * pix_per_deg

        # sky / ground
        p.setBrush(QColor("#2d6fb5"))
        p.drawRect(QRectF(-2 * r, -2 * r + offset, 4 * r, 2 * r))
        p.setBrush(QColor("#7a5230"))
        p.drawRect(QRectF(-2 * r, offset, 4 * r, 2 * r))
        p.setPen(QPen(QColor("white"), 2))
        p.drawLine(QPointF(-2 * r, offset), QPointF(2 * r, offset))

        # pitch ladder
        p.setPen(QPen(QColor("white"), 1))
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = offset - deg * pix_per_deg
            p.drawLine(QPointF(-r / 3, y), QPointF(r / 3, y))
        p.resetTransform()

        # fixed aircraft reference
        p.translate(cx, cy)
        p.setPen(QPen(QColor("#ffcc00"), 3))
        p.drawLine(QPointF(-r / 2, 0), QPointF(-r / 6, 0))
        p.drawLine(QPointF(r / 6, 0), QPointF(r / 2, 0))
        p.drawEllipse(QPointF(0, 0), 3, 3)
        p.setClipping(False)
        p.setPen(QPen(QColor("#444"), 2))
        p.drawEllipse(QPointF(0, 0), r, r)


class Compass(QWidget):
    """Heading dial (degrees)."""

    def __init__(self):
        super().__init__()
        self.heading = 0.0
        self.setMinimumSize(200, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_heading(self, hdg):
        self.heading = hdg
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        r = min(w, h) / 2 - 6
        cx, cy = w / 2, h / 2
        p.translate(cx, cy)

        p.setBrush(QColor("#11141a"))
        p.setPen(QPen(QColor("#444"), 2))
        p.drawEllipse(QPointF(0, 0), r, r)

        p.save()
        p.rotate(-self.heading)
        p.setPen(QPen(QColor("#9aa4b2"), 1))
        f = QFont("Sans", max(7, int(r / 12)))
        p.setFont(f)
        for ang, lbl in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            p.save()
            p.rotate(ang)
            p.setPen(QPen(QColor("#e0e6ef" if lbl == "N" else "#9aa4b2"), 2))
            p.drawLine(QPointF(0, -r), QPointF(0, -r + 12))
            p.drawText(QRectF(-12, -r + 12, 24, 18), Qt.AlignCenter,
                       lbl if lbl == "N" else lbl)
            p.restore()
        p.restore()

        # fixed needle (points up = current heading)
        needle = QPolygonF([QPointF(0, -r + 14), QPointF(-8, 12), QPointF(0, 4), QPointF(8, 12)])
        p.setBrush(QColor("#ff5252"))
        p.setPen(Qt.NoPen)
        p.drawPolygon(needle)
        p.setPen(QColor("#e0e6ef"))
        p.setFont(QFont("Sans", max(9, int(r / 9)), QFont.Bold))
        p.drawText(QRectF(-r, r - 26, 2 * r, 22), Qt.AlignCenter, f"{self.heading:06.2f}°")


class StripChart(QWidget):
    """Rolling multi-trace plot (e.g. 3-axis gyro/accel)."""

    def __init__(self, labels, colors, span=300, yrange=20.0):
        super().__init__()
        self.labels, self.colors = labels, colors
        self.span, self.yrange = span, yrange
        self.data = [deque([0.0] * span, maxlen=span) for _ in labels]
        self.setMinimumHeight(120)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def push(self, values):
        for d, v in zip(self.data, values):
            d.append(v)
        # auto-scale a bit
        peak = max((abs(v) for d in self.data for v in d), default=1.0)
        self.yrange = max(self.yrange * 0.95, peak * 1.1, 1.0)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#11141a"))
        p.setPen(QPen(QColor("#333"), 1))
        p.drawLine(0, h / 2, w, h / 2)

        def y(v):
            return h / 2 - (v / self.yrange) * (h / 2 - 4)

        for trace, color in zip(self.data, self.colors):
            p.setPen(QPen(QColor(color), 1.5))
            poly = QPolygonF()
            for i, v in enumerate(trace):
                poly.append(QPointF(i / self.span * w, y(v)))
            p.drawPolyline(poly)

        p.setFont(QFont("Sans", 8))
        for i, (lbl, color) in enumerate(zip(self.labels, self.colors)):
            p.setPen(QColor(color))
            p.drawText(8 + i * 46, 14, lbl)


class LED(QWidget):
    def __init__(self, text):
        super().__init__()
        self.on = False
        self.text = text
        self.setFixedSize(120, 26)

    def set(self, on):
        self.on = on
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setBrush(QColor("#3ad17a") if self.on else QColor("#d14a4a"))
        p.setPen(Qt.NoPen)
        p.drawEllipse(4, 6, 14, 14)
        p.setPen(QColor("#cdd3dc"))
        p.setFont(QFont("Sans", 9))
        p.drawText(26, 18, self.text)


# ----------------------------------------------------------------------------
#  Value card
# ----------------------------------------------------------------------------
class Card(QFrame):
    def __init__(self, title):
        super().__init__()
        self.setStyleSheet(f"background:{PANEL}; border-radius:8px;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 8, 12, 10)
        t = QLabel(title)
        t.setStyleSheet("color:#8a93a3; font-size:11px; font-weight:bold; border:none;")
        self.value = QLabel("—")
        self.value.setStyleSheet("color:#e8edf4; font-size:20px; font-weight:bold; border:none;")
        lay.addWidget(t)
        lay.addWidget(self.value)

    def set(self, text, color="#e8edf4"):
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{color}; font-size:20px; font-weight:bold; border:none;")


# ----------------------------------------------------------------------------
#  Main window
# ----------------------------------------------------------------------------
class Dashboard(QWidget):
    def __init__(self, port, baud):
        super().__init__()
        self.setWindowTitle("SBG Ellipse-D — Dashboard")
        self.resize(1100, 720)
        self.setStyleSheet(f"background:{BG}; color:#e8edf4; font-family:Sans;")
        self.proc = None
        self._buf = b""
        self.counts = {}
        self.rates = {}

        self._build_ui(port, baud)

        self.rate_timer = QTimer(self)
        self.rate_timer.timeout.connect(self._update_rates)
        self.rate_timer.start(1000)
        self._last_counts = {}

    # ----- UI construction -------------------------------------------------
    def _build_ui(self, port, baud):
        root = QVBoxLayout(self)

        # toolbar
        bar = QHBoxLayout()
        self.port_edit = QLineEdit(port)
        self.port_edit.setFixedWidth(140)
        self.baud_edit = QLineEdit(baud)
        self.baud_edit.setFixedWidth(90)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connect)
        self.status_lbl = QLabel("Disconnected")
        self.status_lbl.setStyleSheet("color:#d14a4a; font-weight:bold;")
        for w in (QLabel("Port:"), self.port_edit, QLabel("Baud:"), self.baud_edit,
                  self.connect_btn, self.status_lbl):
            bar.addWidget(w)
        bar.addStretch()
        self.dev_lbl = QLabel("")
        self.dev_lbl.setStyleSheet("color:#8a93a3;")
        bar.addWidget(self.dev_lbl)
        root.addLayout(bar)

        # solution status banner
        self.sol_lbl = QLabel("EKF: —")
        self.sol_lbl.setStyleSheet("font-size:15px; font-weight:bold; padding:6px;")
        root.addWidget(self.sol_lbl)

        mid = QHBoxLayout()
        root.addLayout(mid, 1)

        # left: instruments
        instruments = QVBoxLayout()
        self.attitude = AttitudeIndicator()
        self.compass = Compass()
        for title, wdg in (("Attitude (roll / pitch)", self.attitude), ("Heading", self.compass)):
            box = QGroupBox(title)
            box.setStyleSheet("QGroupBox{color:#8a93a3;font-weight:bold;border:1px solid #333;border-radius:6px;margin-top:8px;padding-top:10px;} QGroupBox::title{left:10px;}")
            bl = QVBoxLayout(box)
            bl.addWidget(wdg)
            instruments.addWidget(box)
        mid.addLayout(instruments, 2)

        # center: value cards
        grid = QGridLayout()
        self.c_roll = Card("ROLL")
        self.c_pitch = Card("PITCH")
        self.c_yaw = Card("YAW")
        self.c_lat = Card("LATITUDE")
        self.c_lon = Card("LONGITUDE")
        self.c_alt = Card("ALTITUDE")
        self.c_vel = Card("VELOCITY N/E/D (m/s)")
        self.c_fix = Card("GNSS FIX")
        self.c_hdt = Card("DUAL-ANT HEADING")
        cards = [self.c_roll, self.c_pitch, self.c_yaw, self.c_lat, self.c_lon,
                 self.c_alt, self.c_vel, self.c_fix, self.c_hdt]
        for i, c in enumerate(cards):
            grid.addWidget(c, i // 3, i % 3)
        cg = QVBoxLayout()
        cg.addLayout(grid)

        # IMU plots
        self.acc_plot = StripChart(["aX", "aY", "aZ"], ["#ff6b6b", "#6bff95", "#6b9bff"], yrange=12)
        self.gyr_plot = StripChart(["gX", "gY", "gZ"], ["#ffd166", "#06d6a0", "#ef476f"], yrange=20)
        for title, wdg in (("Accelerometers (m/s²)", self.acc_plot), ("Gyroscopes (°/s)", self.gyr_plot)):
            box = QGroupBox(title)
            box.setStyleSheet("QGroupBox{color:#8a93a3;font-weight:bold;border:1px solid #333;border-radius:6px;margin-top:8px;padding-top:10px;}")
            bl = QVBoxLayout(box)
            bl.addWidget(wdg)
            cg.addWidget(box)
        mid.addLayout(cg, 3)

        # right: health + config
        right = QVBoxLayout()
        health = QGroupBox("Health")
        health.setStyleSheet("QGroupBox{color:#8a93a3;font-weight:bold;border:1px solid #333;border-radius:6px;margin-top:8px;padding-top:10px;}")
        hl = QVBoxLayout(health)
        self.led_main = LED("Main power")
        self.led_imu = LED("IMU")
        self.led_gps = LED("GPS power")
        self.led_set = LED("Settings")
        self.led_temp = LED("Temperature")
        self.led_att = LED("Attitude valid")
        self.led_hdg = LED("Heading valid")
        self.led_pos = LED("Position valid")
        for led in (self.led_main, self.led_imu, self.led_gps, self.led_set, self.led_temp,
                    self.led_att, self.led_hdg, self.led_pos):
            hl.addWidget(led)
        right.addWidget(health)

        self.rate_lbl = QLabel("rates: —")
        self.rate_lbl.setStyleSheet("color:#8a93a3; font-size:11px;")
        self.rate_lbl.setWordWrap(True)
        right.addWidget(self.rate_lbl)

        right.addWidget(self._build_config_tabs(), 1)
        mid.addLayout(right, 3)

    # ----- configuration tabs ---------------------------------------------
    def _combo(self, options, default_index=0):
        c = QComboBox()
        for name, val in options:
            c.addItem(name, val)
        c.setCurrentIndex(default_index)
        return c

    def _spin(self, lo, hi, step=0.01, dec=3, suffix="", val=0.0):
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setSingleStep(step)
        s.setDecimals(dec)
        if suffix:
            s.setSuffix(suffix)
        s.setValue(val)
        return s

    def _ispin(self, lo, hi, val=0):
        s = QSpinBox()
        s.setRange(lo, hi)
        s.setValue(val)
        return s

    def _apply_btn(self, text, fn):
        b = QPushButton(text)
        b.clicked.connect(fn)
        return b

    def _build_config_tabs(self):
        tabs = QTabWidget()
        tabs.setStyleSheet(
            "QTabWidget::pane{border:1px solid #333;border-radius:6px;}"
            "QTabBar::tab{background:#262b36;color:#cdd3dc;padding:5px 9px;}"
            "QTabBar::tab:selected{background:#34506e;color:#fff;}")

        def page():
            w = QWidget()
            f = QFormLayout(w)
            f.setLabelAlignment(Qt.AlignRight)
            return w, f

        def scroll(w):
            sa = QScrollArea()
            sa.setWidgetResizable(True)
            sa.setWidget(w)
            return sa

        # --- SENSOR ------------------------------------------------------
        w, f = page()
        self.motion = self._combo(MOTION_PROFILES)
        f.addRow("Motion profile", self.motion)
        f.addRow(self._apply_btn("Set motion profile", lambda: self.send(f"motion {self.motion.currentData()}")))
        self.ax_x = self._combo(AXIS_DIRS, 0)   # Forward
        self.ax_y = self._combo(AXIS_DIRS, 3)   # Right
        f.addRow("IMU X axis", self.ax_x)
        f.addRow("IMU Y axis", self.ax_y)
        self.mis = [self._spin(-180, 180, 0.1, 2, " °") for _ in range(3)]
        for lbl, s in zip(("Misalign roll", "Misalign pitch", "Misalign yaw"), self.mis):
            f.addRow(lbl, s)
        self.imu_la = [self._spin(-50, 50, 0.01, 3, " m") for _ in range(3)]
        for lbl, s in zip(("Lever arm X", "Lever arm Y", "Lever arm Z"), self.imu_la):
            f.addRow(lbl, s)
        f.addRow(self._apply_btn("Set alignment + lever arm", self._send_align))
        self.init_lat = self._spin(-90, 90, 0.0001, 7, " °", 48.0)
        self.init_lon = self._spin(-180, 180, 0.0001, 7, " °", 11.0)
        self.init_alt = self._spin(-1000, 10000, 1, 2, " m", 0.0)
        self.init_date = QLineEdit("2026 1 1")
        for lbl, wdg in (("Init latitude", self.init_lat), ("Init longitude", self.init_lon),
                         ("Init altitude", self.init_alt), ("Init Y M D", self.init_date)):
            f.addRow(lbl, wdg)
        f.addRow(self._apply_btn("Set initial conditions", self._send_init))
        tabs.addTab(scroll(w), "Sensor")

        # --- GNSS --------------------------------------------------------
        w, f = page()
        self.la_spins = [self._spin(-50, 50, 0.01, 3, " m") for _ in range(6)]
        for lbl, s in zip(("Primary X", "Primary Y", "Primary Z",
                           "Secondary X", "Secondary Y", "Secondary Z"), self.la_spins):
            f.addRow(lbl, s)
        f.addRow(self._apply_btn("Set dual-antenna lever arms", self._send_dual))
        self.gnss_model = self._combo(GNSS_MODELS)
        f.addRow("GNSS model", self.gnss_model)
        f.addRow(self._apply_btn("Set GNSS model", lambda: self.send(f"gnssmodel {self.gnss_model.currentData()}")))
        self.gr_pos = self._combo(REJECTION, 1)
        self.gr_vel = self._combo(REJECTION, 1)
        self.gr_hdt = self._combo(REJECTION, 1)
        f.addRow("Reject position", self.gr_pos)
        f.addRow("Reject velocity", self.gr_vel)
        f.addRow("Reject heading", self.gr_hdt)
        f.addRow(self._apply_btn("Set GNSS rejection", lambda: self.send(
            f"gnssreject {self.gr_pos.currentData()} {self.gr_vel.currentData()} {self.gr_hdt.currentData()}")))
        tabs.addTab(scroll(w), "GNSS")

        # --- MAG / ODO ---------------------------------------------------
        w, f = page()
        self.mag_model = self._combo(MAG_MODELS)
        f.addRow("Mag model", self.mag_model)
        f.addRow(self._apply_btn("Set mag model", lambda: self.send(f"magmodel {self.mag_model.currentData()}")))
        self.mag_rej = self._combo(REJECTION, 1)
        f.addRow("Mag rejection", self.mag_rej)
        f.addRow(self._apply_btn("Set mag rejection", lambda: self.send(f"magreject {self.mag_rej.currentData()}")))
        self.odo_gain = self._spin(0, 1e6, 1, 2, " p/m", 1000.0)
        self.odo_err = self._ispin(0, 100, 1)
        self.odo_rev = QCheckBox("Reverse mode")
        f.addRow("Odo gain", self.odo_gain)
        f.addRow("Odo gain error %", self.odo_err)
        f.addRow("", self.odo_rev)
        f.addRow(self._apply_btn("Set odometer config", lambda: self.send(
            f"odo {self.odo_gain.value():.3f} {self.odo_err.value()} {1 if self.odo_rev.isChecked() else 0}")))
        self.odo_la = [self._spin(-50, 50, 0.01, 3, " m") for _ in range(3)]
        for lbl, s in zip(("Odo lever X", "Odo lever Y", "Odo lever Z"), self.odo_la):
            f.addRow(lbl, s)
        f.addRow(self._apply_btn("Set odometer lever arm", lambda: self.send(
            "odolever " + " ".join(f"{s.value():.3f}" for s in self.odo_la))))
        self.odo_rej = self._combo(REJECTION, 1)
        f.addRow("Odo rejection", self.odo_rej)
        f.addRow(self._apply_btn("Set odometer rejection", lambda: self.send(f"odoreject {self.odo_rej.currentData()}")))
        tabs.addTab(scroll(w), "Mag/Odo")

        # --- SYNC / INTERFACES ------------------------------------------
        w, f = page()
        self.si_id = self._ispin(0, 3, 0)
        self.si_sens = self._combo(SYNC_IN_SENS, 2)
        self.si_delay = self._ispin(-1000000, 1000000, 0)
        f.addRow("Sync IN id (0=A..3=D)", self.si_id)
        f.addRow("Sync IN sensitivity", self.si_sens)
        f.addRow("Sync IN delay (ns)", self.si_delay)
        f.addRow(self._apply_btn("Set Sync IN", lambda: self.send(
            f"syncin {self.si_id.value()} {self.si_sens.currentData()} {self.si_delay.value()}")))
        self.so_id = self._ispin(0, 1, 0)
        self.so_func = self._combo(SYNC_OUT_FUNC, 0)
        self.so_pol = self._combo(SYNC_OUT_POL, 1)
        self.so_dur = self._ispin(0, 100000000, 100000)
        f.addRow("Sync OUT id (0=A,1=B)", self.so_id)
        f.addRow("Sync OUT function", self.so_func)
        f.addRow("Sync OUT polarity", self.so_pol)
        f.addRow("Sync OUT pulse (ns)", self.so_dur)
        f.addRow(self._apply_btn("Set Sync OUT", lambda: self.send(
            f"syncout {self.so_id.value()} {self.so_func.currentData()} {self.so_pol.currentData()} {self.so_dur.value()}")))
        self.uart_port = self._combo(UART_PORT, 0)
        self.uart_baud = self._ispin(4800, 4000000, 921600)
        self.uart_mode = self._combo(UART_MODE, 1)
        f.addRow("UART port", self.uart_port)
        f.addRow("UART baud", self.uart_baud)
        f.addRow("UART mode", self.uart_mode)
        f.addRow(self._apply_btn("Set UART", lambda: self.send(
            f"uart {self.uart_port.currentData()} {self.uart_baud.value()} {self.uart_mode.currentData()}")))
        self.can_br = self._combo(CAN_BITRATE, 2)
        self.can_mode = self._combo(CAN_MODE, 0)
        f.addRow("CAN bitrate", self.can_br)
        f.addRow("CAN mode", self.can_mode)
        f.addRow(self._apply_btn("Set CAN", lambda: self.send(
            f"can {self.can_br.currentData()} {self.can_mode.currentData()}")))
        tabs.addTab(scroll(w), "Sync/IF")

        # --- OUTPUTS / AIDING -------------------------------------------
        w, f = page()
        f.addRow(self._apply_btn("Enable default outputs (PORT_A)", lambda: self.send("outputs")))
        self.out_port = self._combo(OUTPUT_PORT, 0)
        self.out_log = self._combo(LOG_IDS, 4)   # EKF_EULER
        self.out_mode = self._combo(OUTPUT_MODE, 4)  # 25 Hz
        f.addRow("Output port", self.out_port)
        f.addRow("Log", self.out_log)
        f.addRow("Rate", self.out_mode)
        f.addRow(self._apply_btn("Set single log output", lambda: self.send(
            f"log {self.out_port.currentData()} 0 {self.out_log.currentData()} {self.out_mode.currentData()}")))
        self.nmea_port = self._combo(OUTPUT_PORT, 0)
        self.nmea_id = QLineEdit("GP")
        f.addRow("NMEA port", self.nmea_port)
        f.addRow("NMEA talker id", self.nmea_id)
        f.addRow(self._apply_btn("Set NMEA talker id", lambda: self.send(
            f"nmeatalker {self.nmea_port.currentData()} {self.nmea_id.text().strip() or 'GP'}")))
        self.aid_gps = self._combo(PORT_ASSIGN, 4)   # Internal
        self.aid_sync = self._combo(SYNC_ASSIGN, 0)
        self.aid_rtcm = self._combo(PORT_ASSIGN, 5)  # Disabled
        self.aid_air = self._combo(PORT_ASSIGN, 5)
        self.aid_dvl = self._combo(PORT_ASSIGN, 5)
        self.aid_odo = self._ispin(0, 255, 255)
        f.addRow("GNSS port", self.aid_gps)
        f.addRow("GNSS sync", self.aid_sync)
        f.addRow("RTCM port", self.aid_rtcm)
        f.addRow("Air-data port", self.aid_air)
        f.addRow("DVL port", self.aid_dvl)
        f.addRow("Odometer pins", self.aid_odo)
        f.addRow(self._apply_btn("Set aiding assignment", lambda: self.send(
            f"aiding {self.aid_gps.currentData()} {self.aid_sync.currentData()} {self.aid_rtcm.currentData()} "
            f"{self.aid_air.currentData()} {self.aid_dvl.currentData()} {self.aid_odo.value()}")))
        tabs.addTab(scroll(w), "Outputs")

        # --- ADVANCED ----------------------------------------------------
        w, f = page()
        self.time_ref = self._combo(TIME_REF, 2)
        self.gnss_opt = self._ispin(0, 2**31 - 1, 0)
        self.nmea_opt = self._ispin(0, 2**31 - 1, 0)
        f.addRow("Time reference", self.time_ref)
        f.addRow("GNSS options bits", self.gnss_opt)
        f.addRow("NMEA options bits", self.nmea_opt)
        f.addRow(self._apply_btn("Set advanced", lambda: self.send(
            f"advanced {self.time_ref.currentData()} {self.gnss_opt.value()} {self.nmea_opt.value()}")))
        self.th_pos = self._spin(0, 1000, 0.1, 2, " m", 1.0)
        self.th_vel = self._spin(0, 1000, 0.1, 2, " m/s", 0.5)
        self.th_att = self._spin(0, 90, 0.1, 2, " °", 1.0)
        self.th_hdg = self._spin(0, 180, 0.1, 2, " °", 2.0)
        for lbl, s in (("Position threshold", self.th_pos), ("Velocity threshold", self.th_vel),
                       ("Attitude threshold", self.th_att), ("Heading threshold", self.th_hdg)):
            f.addRow(lbl, s)
        f.addRow(self._apply_btn("Set validity thresholds", lambda: self.send(
            f"thresholds {self.th_pos.value():.3f} {self.th_vel.value():.3f} "
            f"{self.th_att.value():.3f} {self.th_hdg.value():.3f}")))
        tabs.addTab(scroll(w), "Advanced")

        # --- BACKUP / SETTINGS ------------------------------------------
        w, f = page()
        f.addRow(self._apply_btn("Export settings to file…", self._export))
        f.addRow(self._apply_btn("Import settings from file…", self._import))
        save_btn = self._apply_btn("SAVE settings + reboot", self._save)
        save_btn.setStyleSheet("background:#2e7d32; font-weight:bold;")
        f.addRow(save_btn)
        f.addRow(self._apply_btn("Reboot device", lambda: self.send("reboot")))
        restore_btn = self._apply_btn("Restore factory defaults", self._restore)
        restore_btn.setStyleSheet("background:#8a3030;")
        f.addRow(restore_btn)
        tabs.addTab(scroll(w), "Backup")

        return tabs

    # ----- bridge process --------------------------------------------------
    def toggle_connect(self):
        if self.proc and self.proc.state() != QProcess.NotRunning:
            self.proc.kill()
            return
        if not os.path.exists(BRIDGE):
            QMessageBox.critical(self, "Bridge missing",
                                 f"Bridge binary not found:\n{BRIDGE}\n\nBuild it first (see README).")
            return
        self.proc = QProcess(self)
        self.proc.readyReadStandardOutput.connect(self._on_stdout)
        self.proc.readyReadStandardError.connect(self._on_stderr)
        self.proc.finished.connect(self._on_finished)
        self.proc.start(BRIDGE, [self.port_edit.text(), self.baud_edit.text()])
        self.connect_btn.setText("Disconnect")
        self.status_lbl.setText("Connecting…")
        self.status_lbl.setStyleSheet("color:#ffb74d; font-weight:bold;")

    def _on_finished(self, *_):
        self.connect_btn.setText("Connect")
        self.status_lbl.setText("Disconnected")
        self.status_lbl.setStyleSheet("color:#d14a4a; font-weight:bold;")

    def send(self, cmd):
        if self.proc and self.proc.state() == QProcess.Running:
            self.proc.write((cmd + "\n").encode())

    def _send_dual(self):
        vals = " ".join(f"{s.value():.3f}" for s in self.la_spins)
        self.send(f"dual {vals}")

    def _send_align(self):
        mis = " ".join(f"{s.value():.3f}" for s in self.mis)
        la = " ".join(f"{s.value():.3f}" for s in self.imu_la)
        self.send(f"align {self.ax_x.currentData()} {self.ax_y.currentData()} {mis} {la}")

    def _send_init(self):
        try:
            y, m, d = (int(x) for x in self.init_date.text().split())
        except ValueError:
            QMessageBox.warning(self, "Init", "Date must be 'YEAR MONTH DAY', e.g. 2026 6 18")
            return
        self.send(f"init {self.init_lat.value():.7f} {self.init_lon.value():.7f} "
                  f"{self.init_alt.value():.3f} {y} {m} {d}")

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export settings", "ellipse-d-settings.bin",
                                              "Settings (*.bin);;All files (*)")
        if path:
            self.send(f"export {path}")

    def _import(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import settings", "",
                                              "Settings (*.bin);;All files (*)")
        if path and QMessageBox.question(self, "Import",
                "Import this settings file to the device? It will be applied immediately.") == QMessageBox.Yes:
            self.send(f"import {path}")

    def _save(self):
        if QMessageBox.question(self, "Save", "Save settings to flash and reboot the device?") == QMessageBox.Yes:
            self.send("save")

    def _restore(self):
        if QMessageBox.question(self, "Restore", "Restore FACTORY DEFAULTS and reboot? This erases config.") == QMessageBox.Yes:
            self.send("restore")

    def _on_stderr(self):
        sys.stderr.write(bytes(self.proc.readAllStandardError()).decode(errors="replace"))

    def _on_stdout(self):
        self._buf += bytes(self.proc.readAllStandardOutput())
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            line = line.strip()
            if line:
                try:
                    self._handle(json.loads(line.decode()))
                except (ValueError, KeyError):
                    pass

    # ----- telemetry handling ---------------------------------------------
    def _bump(self, t):
        self.counts[t] = self.counts.get(t, 0) + 1

    def _update_rates(self):
        parts = []
        for t in ("imu", "euler", "nav", "gnss", "hdt", "status"):
            c = self.counts.get(t, 0)
            r = c - self._last_counts.get(t, 0)
            self._last_counts[t] = c
            self.rates[t] = r
            parts.append(f"{t} {r}Hz")
        self.rate_lbl.setText("rates:  " + "   ".join(parts))

    def _handle(self, m):
        t = m.get("t")
        self._bump(t)

        if t == "info":
            self.status_lbl.setText("Connected")
            self.status_lbl.setStyleSheet("color:#3ad17a; font-weight:bold;")
            self.dev_lbl.setText(f"{m['product']}   SN {m['sn']}   FW {m['fw']}")

        elif t == "error":
            self.status_lbl.setText("Error")
            self.status_lbl.setStyleSheet("color:#d14a4a; font-weight:bold;")
            QMessageBox.warning(self, "Bridge error", m.get("msg", "unknown error"))

        elif t == "ack":
            ok = m.get("ok")
            self.status_lbl.setText(f"{m.get('cmd')}: {'OK' if ok else 'FAILED'}")
            self.status_lbl.setStyleSheet(f"color:{'#3ad17a' if ok else '#d14a4a'}; font-weight:bold;")

        elif t == "euler":
            self.attitude.set_attitude(m["roll"], m["pitch"])
            self.c_roll.set(f"{m['roll']:+.2f}°")
            self.c_pitch.set(f"{m['pitch']:+.2f}°")
            self.c_yaw.set(f"{m['yaw']:+.2f}°")
            status = m.get("status", 0)
            self.led_att.set(bool(status & SOL_ATT))
            self.led_hdg.set(bool(status & SOL_HDG))
            self.led_pos.set(bool(status & SOL_POS))
            mode = m.get("mode", 0)
            color = "#3ad17a" if mode >= 3 else "#ffb74d" if mode >= 2 else "#d14a4a"
            self.sol_lbl.setText(f"EKF SOLUTION:  {SOL_MODE.get(mode, '?')}")
            self.sol_lbl.setStyleSheet(f"font-size:15px;font-weight:bold;padding:6px;color:{color};")

        elif t == "nav":
            self.c_lat.set(f"{m['lat']:.7f}°")
            self.c_lon.set(f"{m['lon']:.7f}°")
            self.c_alt.set(f"{m['alt']:.2f} m")
            self.c_vel.set(f"{m['vN']:+.2f} {m['vE']:+.2f} {m['vD']:+.2f}")

        elif t == "gnss":
            fix = m.get("fix", 0)
            label = GNSS_FIX.get(fix, "?")
            color = "#3ad17a" if fix == 7 else "#ffb74d" if fix == 6 else "#e8edf4" if fix >= 2 else "#d14a4a"
            self.c_fix.set(f"{label}  ({m.get('sv', 0)} SV)", color)

        elif t == "hdt":
            self.compass.set_heading(m["heading"])
            ok = m.get("status") == 0
            self.c_hdt.set(f"{m['heading']:.2f}°  ±{m['headingAcc']:.2f}",
                           "#3ad17a" if ok else "#d14a4a")

        elif t == "imu":
            self.acc_plot.push([m["ax"], m["ay"], m["az"]])
            self.gyr_plot.push([m["gx"], m["gy"], m["gz"]])

        elif t == "status":
            g = m.get("general", 0)
            self.led_main.set(bool(g & G_MAIN))
            self.led_imu.set(bool(g & G_IMU))
            self.led_gps.set(bool(g & G_GPS))
            self.led_set.set(bool(g & G_SET))
            self.led_temp.set(bool(g & G_TEMP))

    def closeEvent(self, e):
        if self.proc and self.proc.state() != QProcess.NotRunning:
            self.proc.kill()
            self.proc.waitForFinished(1000)
        e.accept()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baud = sys.argv[2] if len(sys.argv) > 2 else "921600"
    app = QApplication(sys.argv)
    win = Dashboard(port, baud)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
