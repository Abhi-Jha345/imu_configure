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
    QDoubleSpinBox, QFormLayout, QMessageBox,
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

        cfg = QGroupBox("Configure")
        cfg.setStyleSheet("QGroupBox{color:#8a93a3;font-weight:bold;border:1px solid #333;border-radius:6px;margin-top:8px;padding-top:10px;}")
        form = QVBoxLayout(cfg)

        self.motion = QComboBox()
        for name, mid_ in MOTION_PROFILES:
            self.motion.addItem(name, mid_)
        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Motion:"))
        mrow.addWidget(self.motion)
        apply_motion = QPushButton("Set")
        apply_motion.clicked.connect(lambda: self.send(f"motion {self.motion.currentData()}"))
        mrow.addWidget(apply_motion)
        form.addLayout(mrow)

        # dual antenna lever arms
        la = QFormLayout()
        self.la_spins = []
        for lbl in ("Pri X", "Pri Y", "Pri Z", "Sec X", "Sec Y", "Sec Z"):
            s = QDoubleSpinBox()
            s.setRange(-50, 50)
            s.setSingleStep(0.01)
            s.setDecimals(3)
            s.setSuffix(" m")
            self.la_spins.append(s)
            la.addRow(lbl, s)
        form.addLayout(la)
        apply_dual = QPushButton("Set dual-antenna lever arms")
        apply_dual.clicked.connect(self._send_dual)
        form.addWidget(apply_dual)

        out_btn = QPushButton("Enable default outputs (PORT_A)")
        out_btn.clicked.connect(lambda: self.send("outputs"))
        form.addWidget(out_btn)

        save_btn = QPushButton("SAVE settings + reboot")
        save_btn.setStyleSheet("background:#2e7d32; font-weight:bold;")
        save_btn.clicked.connect(self._save)
        form.addWidget(save_btn)

        restore_btn = QPushButton("Restore factory defaults")
        restore_btn.setStyleSheet("background:#8a3030;")
        restore_btn.clicked.connect(self._restore)
        form.addWidget(restore_btn)

        right.addWidget(cfg)
        right.addStretch()
        mid.addLayout(right, 2)

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
