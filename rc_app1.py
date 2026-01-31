import sys
import time
import math
import pygame
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QSlider,
    QComboBox, QVBoxLayout, QHBoxLayout, QGroupBox, QFrame
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPen, QPainterPath, QBrush
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView

# ===================== CRSF =====================
CRSF_ADDRESS = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS = 0x16

def map_range(x, a1, a2, b1, b2):
    return int(b1 + (x - a1) * (b2 - b1) / (a2 - a1))

def pack_crsf_channels(ch):
    buf = 0
    bits = 0
    out = []
    for c in ch:
        buf |= (c & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            out.append(buf & 0xFF)
            buf >>= 8
            bits -= 8
    if bits:
        out.append(buf & 0xFF)
    payload = bytes(out)
    frame = bytes([CRSF_ADDRESS, len(payload) + 2, CRSF_FRAMETYPE_RC_CHANNELS]) + payload
    crc = 0
    for b in frame[2:]:
        crc = (crc + b) & 0xFF
    return frame + bytes([crc])

# ===================== APP =====================
class RCApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC Vehicle Control System")
        self.resize(1400, 820)

        self.serial_out = None
        self.serial_in = None
        self.in_buffer = ""

        self.sending_enabled = True
        self.steer = 0.0
        self.throttle = 0.0
        self.yaw = 0.0
        self.pitch = 0.0

        self.yaw_offset = 0.0
        self.pitch_offset = 0.0

        self.head_source = "arduino"

        self.joy = None

        self.last_steer = 0.0
        self.last_throttle = 0.0
        self.last_yaw = 0.0
        self.last_pitch = 0.0
        self.last_time = time.time()

        self.default_L2_axis = 4

        self.init_ui()

        self.init_pygame()

        self.init_timers()

    # ---------- PYGAME / JOYSTICK ----------
    def init_pygame(self):
        try:
            pygame.init()
            pygame.joystick.init()
        except Exception:
            pass
        self.reinit_joystick()
        self.joy_reinit_timer = QTimer()
        self.joy_reinit_timer.timeout.connect(self.reinit_joystick)
        self.joy_reinit_timer.start(1000)

    def reinit_joystick(self):
        try:
            count = pygame.joystick.get_count()
        except Exception:
            count = 0

        if count > 0:
            try:
                if not self.joy:
                    self.joy = pygame.joystick.Joystick(0)
                    self.joy.init()
                else:
                    _ = self.joy.get_name()
            except Exception:
                self.joy = None
        else:
            self.joy = None

        self.update_joystick_label()

    def update_joystick_label(self):
        if not hasattr(self, 'lbl_status') or self.lbl_status is None:
            return
        try:
            if self.joy:
                try:
                    name = self.joy.get_name()
                except Exception:
                    name = "Controller"
                try:
                    nbtns = self.joy.get_numbuttons()
                except Exception:
                    nbtns = "?"
                self.lbl_status.setText(f"🟢 Controller: {name} ({nbtns} buttons)")
            else:
                inp = self.cb_in.currentText() if hasattr(self, 'cb_in') else ""
                out = self.cb_out.currentText() if hasattr(self, 'cb_out') else ""
                txt = "⚪ No controller / Test mode"
                if inp:
                    txt = f"⚪ No controller — Input: {inp}"
                if out:
                    txt += f"  Output: {out}"
                self.lbl_status.setText(txt)
        except Exception:
            pass

    # ---------- UI ----------
    def init_ui(self):
        font_title = QFont("Segoe UI", 20, QFont.Bold)
        font = QFont("Segoe UI", 16)

        central = QWidget()
        self.setCentralWidget(central)
        main = QHBoxLayout(central)
        main.setSpacing(12)

        # ===== LEFT PANEL =====
        left = QVBoxLayout()
        left.setSpacing(10)
        left.setContentsMargins(6,6,6,6)

        title = QLabel("RC CONTROL")
        title.setFont(font_title)
        left.addWidget(title)
        left.addSpacing(8)

        box_conn = QGroupBox("Connections")
        box_conn.setFont(font)
        v = QVBoxLayout(box_conn)
        v.setSpacing(8)

        self.cb_out = QComboBox()

        self.btn_refresh_ports = QPushButton("🔄 Refresh COM ports")
        self.btn_refresh_ports.clicked.connect(self.refresh_ports)

        self.btn_connect_out = QPushButton("Connect Output")
        self.btn_connect_out.clicked.connect(self.connect_serial_out)

        v.addWidget(QLabel("CRSF Output (Ranger Micro)"))
        v.addWidget(self.cb_out)
        v.addWidget(self.btn_connect_out)

        v.addSpacing(6)

        self.cb_in = QComboBox()
        self.btn_connect_in = QPushButton("Connect Head-Tracking Input")
        self.btn_connect_in.clicked.connect(self.connect_serial_in)

        v.addWidget(QLabel("Head-tracking Input (Arduino)"))
        v.addWidget(self.cb_in)
        v.addWidget(self.btn_connect_in)

        v.addSpacing(8)
        v.addWidget(self.btn_refresh_ports)

        v.addSpacing(8)

        self.lbl_status = QLabel("⚪ No controller / Test mode")
        self.lbl_status.setFont(QFont("Segoe UI", 13))
        v.addWidget(self.lbl_status)

        box_conn.setLayout(v)
        left.addWidget(box_conn)

        self.telemetry_frame = QFrame()
        tf_layout = QVBoxLayout(self.telemetry_frame)
        self.lbl_telemetry = QLabel("S: 0  T: 0\nYaw: 0  Pitch: 0")
        self.lbl_telemetry.setFont(QFont("Consolas", 16))
        tf_layout.addWidget(self.lbl_telemetry)
        self.telemetry_frame.setStyleSheet("background: rgba(0,0,0,0.45); padding:8px; border-radius:6px;")
        left.addWidget(self.telemetry_frame)

        left.addStretch()
        main.addLayout(left, 1)

        # ===== CENTER PANEL =====
        center = QVBoxLayout()
        center.setSpacing(10)
        center.setContentsMargins(0,0,0,0)

        self.scene = QGraphicsScene(-320, -320, 640, 640)
        self.view = QGraphicsView(self.scene)
        self.view.setFixedSize(680, 420)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.view.setFrameStyle(QFrame.NoFrame)
        center.addWidget(self.view, alignment=Qt.AlignCenter)

        pen = QPen(QColor("#cccccc"), 2)
        car_path = QPainterPath()
        self.car_width = 110
        self.car_height = 210
        car_path.addRoundedRect(-self.car_width/2, -self.car_height/2, self.car_width, self.car_height, 14, 14)
        self.car_item = self.scene.addPath(car_path, pen)
        self.car_item.setBrush(QColor("#1db954"))

        wheel_pen = QPen(QColor("#aaaaaa"), 1)
        self.scene.addRect(-self.car_width/2 - 30, -self.car_height/2 + 12, 24, 52, wheel_pen)
        self.scene.addRect(self.car_width/2 + 6, -self.car_height/2 + 12, 24, 52, wheel_pen)
        self.scene.addRect(-self.car_width/2 - 30, self.car_height/2 - 68, 24, 52, wheel_pen)
        self.scene.addRect(self.car_width/2 + 6, self.car_height/2 - 68, 24, 52, wheel_pen)

        self.path_item = self.scene.addPath(QPainterPath(), QPen(QColor("#00ff88"), 6))

        self.lbl_reverse = QLabel("")
        self.lbl_reverse.setFont(QFont("Segoe UI", 16, QFont.Bold))
        self.lbl_reverse.setStyleSheet("color:#ff4444;")
        center.addWidget(self.lbl_reverse, alignment=Qt.AlignCenter)

        self.marker = self.scene.addRect(-18, 120, 36, 18, QPen(Qt.black), QBrush(QColor("#eeeeee")))

        self.head_scene = QGraphicsScene(-180, -180, 360, 360)
        self.head_view = QGraphicsView(self.head_scene)
        self.head_view.setFixedSize(420, 300)
        self.head_view.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.head_view.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.head_view.setFrameStyle(QFrame.NoFrame)

        g_pen = QPen(QColor("#bdbdbd"), 2)
        g_brush = QBrush(QColor("#222222"))
        frame_item = self.head_scene.addRect(-95, -38, 190, 76, g_pen, g_brush)
        left_lens = self.head_scene.addEllipse(-64, -28, 56, 56, QPen(QColor("#888")), QBrush(QColor("#0f4c81")))
        right_lens = self.head_scene.addEllipse(10, -28, 56, 56, QPen(QColor("#888")), QBrush(QColor("#0f4c81")))
        bridge = self.head_scene.addRect(-8, -8, 16, 16, QPen(QColor("#bdbdbd")), QBrush(QColor("#bdbdbd")))
        strap = self.head_scene.addRect(-150, -8, 40, 16, QPen(QColor("#444")), QBrush(QColor("#444")))
        strap2 = self.head_scene.addRect(110, -8, 40, 16, QPen(QColor("#444")), QBrush(QColor("#444")))
        self.goggles_group = self.head_scene.createItemGroup([frame_item, left_lens, right_lens, bridge, strap, strap2])
        self.goggles_group.setTransformOriginPoint(0, 0)

        self.gaze_line = self.head_scene.addLine(0, 0, 0, -100, QPen(QColor("#ffd366"), 4, Qt.SolidLine, Qt.RoundCap))
        self.pitch_bar = self.head_scene.addRect(-12, 80, 24, 0, QPen(Qt.NoPen), QBrush(QColor("#9b59ff")))

        center.addWidget(self.head_view, alignment=Qt.AlignCenter)

        main.addLayout(center, 3)

        # ===== RIGHT PANEL =====
        right = QVBoxLayout()
        right.setSpacing(10)
        right.setContentsMargins(6,6,6,6)

        box_prof = QGroupBox("Profile")
        box_prof.setFont(font)
        pv = QVBoxLayout(box_prof)
        self.profile_box = QComboBox()
        self.profile_box.addItems(["Custom", "Beginner", "Sport", "Race"])
        self.profile_box.currentTextChanged.connect(self.load_profile)
        pv.addWidget(QLabel("Driving profile"))
        pv.addWidget(self.profile_box)
        right.addWidget(box_prof)

        box_tune = QGroupBox("Tuning")
        box_tune.setFont(font)
        t = QVBoxLayout(box_tune)

        self.sl_steer = QSlider(Qt.Horizontal)
        self.sl_steer.setRange(50, 400)
        self.sl_steer.setValue(200)
        self.sl_throttle = QSlider(Qt.Horizontal)
        self.sl_throttle.setRange(50, 400)
        self.sl_throttle.setValue(200)
        self.deadzone_slider = QSlider(Qt.Horizontal)
        self.deadzone_slider.setRange(0, 20)
        self.deadzone_slider.setValue(5)

        t.addWidget(QLabel("Steering response (rate)"))
        t.addWidget(self.sl_steer)
        t.addWidget(QLabel("Throttle response (rate)"))
        t.addWidget(self.sl_throttle)
        t.addWidget(QLabel("Steering deadzone (%)"))
        t.addWidget(self.deadzone_slider)

        btn_reset = QPushButton("Reset tuning")
        btn_reset.clicked.connect(self.reset_tuning)
        t.addWidget(btn_reset)

        btn_pause = QPushButton("⏸ Pause / Resume sending")
        btn_pause.clicked.connect(self.toggle_sending)
        t.addWidget(btn_pause)

        right.addWidget(box_tune)

        box_head = QGroupBox("Head Tracking")
        box_head.setFont(font)
        hv = QVBoxLayout(box_head)

        self.cb_head_source = QComboBox()
        self.cb_head_source.addItems(["Arduino", "Joystick"])
        self.cb_head_source.currentIndexChanged.connect(self.change_head_source)
        hv.addWidget(QLabel("Source"))
        hv.addWidget(self.cb_head_source)

        self.btn_calibrate = QPushButton("🎯 Calibrate Head Center")
        self.btn_calibrate.clicked.connect(self.calibrate_head)
        hv.addWidget(self.btn_calibrate)

        self.head_deadzone_slider = QSlider(Qt.Horizontal)
        self.head_deadzone_slider.setRange(0, 30)
        self.head_deadzone_slider.setValue(5)
        hv.addWidget(QLabel("Head deadzone (joystick %)"))
        hv.addWidget(self.head_deadzone_slider)

        right.addWidget(box_head)

        right.addStretch()
        main.addLayout(right, 1)

        self.setStyleSheet("""
            QWidget { background:#0b0f14; color:#e6e6e6; }
            QGroupBox { border:1px solid #2a2f36; border-radius:8px; margin-top:10px; padding:12px; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 6px; color:#9fd3ff; font-weight:bold; }
            QPushButton { background:#161b22; border:1px solid #30363d; padding:8px; border-radius:6px; }
            QPushButton:hover { background:#1f2630; border-color:#6cb6ff; }
            QLabel { font-size: 16px; }
            QSlider::groove:horizontal { height:8px; background:#30363d; border-radius:4px; }
            QSlider::handle:horizontal { width:18px; background:#ffd366; margin:-7px 0; border-radius:9px; }
            QComboBox { background:#0f1720; padding:6px; color:#e8eef6; font-size:15px; }
        """)

        self.load_profile("Beginner")
        self.refresh_ports()
        self.update_joystick_label()

    # ---------- TIMERS ----------
    def init_timers(self):
        self.loop_timer = QTimer()
        self.loop_timer.timeout.connect(self.update_logic)
        self.loop_timer.start(20) 

        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(200)

    # ---------- SERIAL / COMS ----------
    def refresh_ports(self):
        self.cb_out.clear()
        self.cb_in.clear()
        for p in serial.tools.list_ports.comports():
            text = f"{p.device} – {p.description}"
            self.cb_out.addItem(text, p.device)
            self.cb_in.addItem(text, p.device)
        self.reinit_joystick()
        self.update_joystick_label()

    def connect_serial_out(self):
        try:
            port = self.cb_out.currentData()
            if port:
                self.serial_out = serial.Serial(port, 115200, timeout=0)
                self.lbl_status.setText(f"🟢 Output connected: {port}")
            else:
                self.lbl_status.setText("🔴 No output selected")
        except Exception:
            self.serial_out = None
            self.lbl_status.setText("🔴 Out connect failed")

    def connect_serial_in(self):
        try:
            port = self.cb_in.currentData()
            if port:
                self.serial_in = serial.Serial(port, 115200, timeout=0)
                self.lbl_status.setText(f"🟢 Input connected: {port}")
                self.in_buffer = ""
            else:
                self.lbl_status.setText("🔴 No input selected")
        except Exception:
            self.serial_in = None
            self.lbl_status.setText("🔴 In connect failed")

    def toggle_sending(self):
        self.sending_enabled = not self.sending_enabled
        self.lbl_status.setText("⏸ Sending paused" if not self.sending_enabled else "🟢 Sending active")

    def reset_tuning(self):
        self.sl_steer.setValue(200)
        self.sl_throttle.setValue(200)
        self.deadzone_slider.setValue(5)
        self.profile_box.setCurrentText("Custom")

    def load_profile(self, name):
        if name == "Beginner":
            self.sl_steer.setValue(120)
            self.sl_throttle.setValue(120)
            self.deadzone_slider.setValue(10)
        elif name == "Sport":
            self.sl_steer.setValue(250)
            self.sl_throttle.setValue(250)
            self.deadzone_slider.setValue(5)
        elif name == "Race":
            self.sl_steer.setValue(380)
            self.sl_throttle.setValue(380)
            self.deadzone_slider.setValue(0)
        else:
            pass

    def change_head_source(self):
        self.head_source = "arduino" if self.cb_head_source.currentIndex() == 0 else "joystick"
        self.lbl_status.setText(f"Head source: {self.head_source}")

    def calibrate_head(self):
        self.yaw_offset = self.yaw
        self.pitch_offset = self.pitch
        self.lbl_status.setText("🎯 Head calibrated")

    # ---------- LOGIC ----------
    def update_logic(self):
        try:
            pygame.event.pump()
        except Exception:
            pass

        if self.joy:
            try:
                raw_steer = self.joy.get_axis(0)
                raw_thr = (self.joy.get_axis(5) + 1) / 2 if self.joy.get_numaxes() > 5 else 0
            except Exception:
                raw_steer = 0
                raw_thr = 0
        else:
            raw_steer = 0
            raw_thr = 0

        deadzone = self.deadzone_slider.value() / 100.0
        if abs(raw_steer) < deadzone:
            raw_steer = 0.0
        else:
            raw_steer = math.copysign((abs(raw_steer) - deadzone) / (1.0 - deadzone), raw_steer)

        now = time.time()
        dt = now - self.last_time if self.last_time else 0.02
        self.last_time = now

        self.last_steer = self.steer
        self.last_throttle = self.throttle

        self.steer += (raw_steer - self.steer) * (self.sl_steer.value() / 1000.0)
        self.throttle += (raw_thr - self.throttle) * (self.sl_throttle.value() / 1000.0)

        if self.head_source == "arduino":
            if self.serial_in:
                try:
                    n = self.serial_in.in_waiting
                    if n:
                        data = self.serial_in.read(n).decode(errors='ignore')
                        self.in_buffer += data
                        lines = self.in_buffer.splitlines(True)
                        complete = []
                        rem = ""
                        for ln in lines:
                            if ln.endswith("\n") or ln.endswith("\r"):
                                complete.append(ln.strip())
                            else:
                                rem = ln
                        self.in_buffer = rem
                        for ln in complete:
                            try:
                                parts = ln.replace(" ", "").split(",")
                                yawv = None
                                pitchv = None
                                for p in parts:
                                    if p.upper().startswith("YAW:"):
                                        yawv = float(p.split(":")[1])
                                    if p.upper().startswith("PITCH:"):
                                        pitchv = float(p.split(":")[1])
                                if yawv is not None:
                                    self.yaw = yawv
                                if pitchv is not None:
                                    self.pitch = pitchv
                            except Exception:
                                pass
                except Exception:
                    try:
                        self.serial_in.close()
                    except:
                        pass
                    self.serial_in = None
                    self.lbl_status.setText("🔴 Input disconnected")
        else:
            if self.joy:
                try:
                    rx = self.joy.get_axis(2) if self.joy.get_numaxes() > 2 else 0.0
                    ry = self.joy.get_axis(3) if self.joy.get_numaxes() > 3 else 0.0

                    head_dead = self.head_deadzone_slider.value() / 100.0
                    if abs(rx) < head_dead:
                        rx = 0.0
                    else:
                        rx = math.copysign((abs(rx) - head_dead) / (1.0 - head_dead), rx)
                    if abs(ry) < head_dead:
                        ry = 0.0
                    else:
                        ry = math.copysign((abs(ry) - head_dead) / (1.0 - head_dead), ry)

                    self.yaw = -rx * 90.0
                    self.pitch = -ry * 45.0
                except Exception:
                    pass

        adj_yaw = self.yaw - self.yaw_offset
        adj_pitch = self.pitch - self.pitch_offset
        if adj_yaw > 180: adj_yaw -= 360
        if adj_yaw < -180: adj_yaw += 360
        adj_pitch = max(-45, min(45, adj_pitch))

        l2_val = 0.0
        try:
            if self.joy and self.joy.get_numaxes() > self.default_L2_axis:
                l2_raw = self.joy.get_axis(self.default_L2_axis)
                l2_val = (l2_raw + 1.0) / 2.0
            else:
                l2_val = 0.0
        except Exception:
            l2_val = 0.0

        if l2_val > 0.5:
            self.lbl_reverse.setText("REVERSE")
        else:
            self.lbl_reverse.setText("")

        ch = [1024] * 16
        ch[0] = map_range(self.steer, -1, 1, 172, 1811)
        ch[1] = map_range(self.throttle, 0, 1, 172, 1811)
        ch[2] = map_range(adj_yaw, -180, 180, 172, 1811)
        ch[3] = map_range(adj_pitch, -45, 45, 172, 1811)
        ch[4] = map_range(l2_val, 0, 1, 172, 1811)

        if self.serial_out and self.sending_enabled:
            try:
                self.serial_out.write(pack_crsf_channels(ch))
            except Exception:
                try:
                    self.serial_out.close()
                except:
                    pass
                self.serial_out = None
                self.lbl_status.setText("🔴 Output disconnected")

        if self.throttle < 0.3:
            col = QColor("#00ff88")
        elif self.throttle < 0.7:
            col = QColor("#ffaa00")
        else:
            col = QColor("#ff3333")
        self.car_item.setBrush(col)

        p = QPainterPath()
        center_x = 0
        center_y = -self.car_height/2
        p.moveTo(center_x, center_y)

        max_angle = math.radians(45)
        angle = self.steer * max_angle
        length = 320

        end_x = center_x + math.sin(angle) * length
        end_y = center_y - math.cos(angle) * length

        c1x = center_x + math.sin(angle) * (length * 0.35)
        c1y = center_y - math.cos(angle) * (length * 0.2)
        c2x = center_x + math.sin(angle) * (length * 0.7)
        c2y = center_y - math.cos(angle) * (length * 0.6)

        p.cubicTo(c1x, c1y, c2x, c2y, end_x, end_y)
        hue = int(((self.steer + 1) / 2.0) * 200)
        curve_col = QColor.fromHsv(hue % 360, 200, 255)
        self.path_item.setPath(p)
        self.path_item.setPen(QPen(curve_col, 6, Qt.SolidLine, Qt.RoundCap))

        marker_x = self.steer * 100
        marker_y = (self.throttle - 0.5) * 160
        self.marker.setRect(marker_x - 18, 120 - marker_y, 36, 18)

        try:
            display_yaw = -adj_yaw
            self.goggles_group.setRotation((display_yaw / 180.0) * 45.0)
        except Exception:
            pass

        pitch_norm = adj_pitch / 45.0
        bar_h = abs(pitch_norm) * 80
        if pitch_norm >= 0:
            self.pitch_bar.setRect(-12, 80 - bar_h, 24, bar_h)
        else:
            self.pitch_bar.setRect(-12, 80, 24, -bar_h)

        try:
            pct = (pitch_norm + 1) / 2
            r = int(15 + pct * (15))
            g = int(76 + pct * (120))
            b = int(129 + pct * (126))
            lens_color = QColor(r, g, b)
            for it in self.goggles_group.childItems():
                br = it.boundingRect()
                if abs(br.width() - 56) < 12 and abs(br.height() - 56) < 12:
                    it.setBrush(QBrush(lens_color))
        except Exception:
            pass

    def update_gui(self):
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.2
        self.last_time = now

        adj_yaw = self.yaw - self.yaw_offset
        adj_pitch = self.pitch - self.pitch_offset
        adj_pitch = max(-45, min(45, adj_pitch))

        try:
            self.lbl_telemetry.setText(
                f"S: {int(map_range(self.steer, -1,1,1000,2000))}  T: {int(map_range(self.throttle,0,1,1000,2000))}\n"
                f"Yaw: {adj_yaw:.1f}°  Pitch: {adj_pitch:.1f}°"
            )
        except Exception:
            self.lbl_telemetry.setText(f"S:{self.steer:.3f} T:{self.throttle:.3f}\nYaw:{adj_yaw:.1f} Pitch:{adj_pitch:.1f}")

        self.update_joystick_label()

    def closeEvent(self, event):
        try:
            if self.serial_out:
                self.serial_out.close()
        except:
            pass
        try:
            if self.serial_in:
                self.serial_in.close()
        except:
            pass
        try:
            if hasattr(self, 'joy_reinit_timer'):
                self.joy_reinit_timer.stop()
        except:
            pass
        event.accept()

# ===================== RUN =====================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setFont(QFont("Segoe UI", 16))
    win = RCApp()
    win.show()
    sys.exit(app.exec_())
