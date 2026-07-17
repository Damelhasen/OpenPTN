"""
IMU 3D Orientation Dashboard for MATEKSYS8743 Slim (via MAVLink) — PyVista + Qt edition
-----------------------------------------------------------------------------------------
GPU-accelerated (VTK) 3D flying-wing model that rotates live from MAVLink ATTITUDE
data, with a control panel (port select/refresh/connect, lock-yaw anti-drift,
zero-IMU) and a telemetry/servo readout panel, styled after a typical scale-drone
ground-station dashboard.

Requirements:
    pip install pyvista pyvistaqt PyQt5 pymavlink numpy pyserial

Usage:
    python imu_dashboard.py

Everything (port selection, baud, connect/disconnect) is driven from the GUI —
no command-line arguments needed.

Smoothness / rate notes:
    - Requests ATTITUDE @ 50Hz and GPS_RAW_INT @ 5Hz from the FC via
      MAV_CMD_SET_MESSAGE_INTERVAL (falls back to REQUEST_DATA_STREAM for
      older firmware).
    - Dead-reckons orientation between MAVLink messages using the gyro rates
      included in every ATTITUDE message, so the render loop (60fps, driven
      by Qt's QTimer) always has a fresh angle regardless of telemetry rate.
    - Rendering is VTK/GPU accelerated via PyVista, not redrawn CPU-side like
      matplotlib, so it holds a smooth frame rate.
"""

import sys
import threading
import time

import numpy as np

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink not installed. Run: pip install pymavlink pyserial")
    sys.exit(1)

try:
    import serial.tools.list_ports
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

try:
    import pyvista as pv
    from pyvistaqt import QtInteractor
except ImportError:
    print("pyvista/pyvistaqt not installed. Run: pip install pyvista pyvistaqt")
    sys.exit(1)

try:
    from PyQt5 import QtWidgets, QtCore, QtGui
except ImportError:
    print("PyQt5 not installed. Run: pip install PyQt5")
    sys.exit(1)


# ---------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------
ATTITUDE_RATE_HZ = 50
GPS_RATE_HZ = 5
MAX_EXTRAPOLATION_SEC = 0.35
RENDER_FPS = 60

MAVLINK_MSG_ID_ATTITUDE = 30
MAVLINK_MSG_ID_GPS_RAW_INT = 24

FIX_TYPE_LABELS = {
    0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix",
    4: "DGPS", 5: "RTK Float", 6: "RTK Fixed",
}


# ---------------------------------------------------------------------
# Shared telemetry state (written by reader thread, read by GUI timer)
# ---------------------------------------------------------------------
class AttitudeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.rollspeed = 0.0
        self.pitchspeed = 0.0
        self.yawspeed = 0.0
        self.sample_time = 0.0

        self.msg_rate_hz = 0.0
        self._rate_window_start = time.time()
        self._rate_window_count = 0

        self.satellites_visible = 0
        self.fix_type = 0
        self.gps_time = 0.0

        self.connected = False
        self.yaw_offset = 0.0  # for "Lock Yaw (Anti-Drift)"

    def update_attitude(self, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed):
        now = time.time()
        with self.lock:
            self.roll, self.pitch, self.yaw = roll, pitch, yaw
            self.rollspeed, self.pitchspeed, self.yawspeed = rollspeed, pitchspeed, yawspeed
            self.sample_time = now
            self._rate_window_count += 1
            elapsed = now - self._rate_window_start
            if elapsed >= 1.0:
                self.msg_rate_hz = self._rate_window_count / elapsed
                self._rate_window_count = 0
                self._rate_window_start = now

    def update_gps(self, satellites_visible, fix_type):
        with self.lock:
            self.satellites_visible = satellites_visible
            self.fix_type = fix_type
            self.gps_time = time.time()

    def zero_yaw_here(self):
        """Used by both 'Lock Yaw' and 'Zero IMU' — sets current yaw as the new zero."""
        with self.lock:
            self.yaw_offset = self.yaw + self.yawspeed * (time.time() - self.sample_time)

    def get_extrapolated(self):
        with self.lock:
            roll, pitch, yaw = self.roll, self.pitch, self.yaw
            rs, ps, ys = self.rollspeed, self.pitchspeed, self.yawspeed
            sample_time = self.sample_time
            rate_hz = self.msg_rate_hz
            sats = self.satellites_visible
            fix_type = self.fix_type
            connected = self.connected
            gps_time = self.gps_time
            yaw_offset = self.yaw_offset

        if sample_time == 0.0:
            return 0.0, 0.0, 0.0, rate_hz, sats, fix_type, connected, gps_time, False

        dt = time.time() - sample_time
        stale = dt > MAX_EXTRAPOLATION_SEC
        dt = min(dt, MAX_EXTRAPOLATION_SEC)

        roll += rs * dt
        pitch += ps * dt
        yaw += ys * dt
        return roll, pitch, yaw - yaw_offset, rate_hz, sats, fix_type, connected, gps_time, stale


def request_high_rate_streams(conn):
    def set_interval(msg_id, hz):
        interval_us = int(1_000_000 / hz)
        conn.mav.command_long_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0,
        )
    try:
        set_interval(MAVLINK_MSG_ID_ATTITUDE, ATTITUDE_RATE_HZ)
        set_interval(MAVLINK_MSG_ID_GPS_RAW_INT, GPS_RATE_HZ)
    except Exception:
        pass
    try:
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, ATTITUDE_RATE_HZ, 1,
        )
        conn.mav.request_data_stream_send(
            conn.target_system, conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, GPS_RATE_HZ, 1,
        )
    except Exception:
        pass


def mavlink_reader(port, baud, state: AttitudeState, stop_event, status_callback):
    try:
        status_callback(f"Connecting to {port} @ {baud}...")
        conn = mavutil.mavlink_connection(port, baud=baud)
    except Exception as e:
        status_callback(f"Failed to open {port}: {e}")
        return

    status_callback("Waiting for heartbeat...")
    hb = conn.wait_heartbeat(timeout=10)
    if hb is None:
        status_callback("No heartbeat received — check port/wiring")
        return

    status_callback(f"Connected: sys {conn.target_system} comp {conn.target_component}")
    state.connected = True
    request_high_rate_streams(conn)
    last_refresh = time.time()

    while not stop_event.is_set():
        msg = conn.recv_match(blocking=True, timeout=1)
        if msg is None:
            if time.time() - last_refresh > 5:
                request_high_rate_streams(conn)
                last_refresh = time.time()
            continue
        msg_type = msg.get_type()
        if msg_type == "ATTITUDE":
            state.update_attitude(msg.roll, msg.pitch, msg.yaw,
                                   msg.rollspeed, msg.pitchspeed, msg.yawspeed)
        elif msg_type == "GPS_RAW_INT":
            state.update_gps(msg.satellites_visible, msg.fix_type)

    state.connected = False
    status_callback("Disconnected")


# ---------------------------------------------------------------------
# Rotation math: aerospace body frame (X=nose/roll, Y=right/pitch, Z=down/yaw)
# remapped into a Z-up plotting world (X=right, Y=forward/nose, Z=up) via a
# proper (det=+1) change of basis, same idea as NED->ENU conversion.
# ---------------------------------------------------------------------
def rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return R_yaw @ R_pitch @ R_roll


_M = np.array([
    [0, 1, 0],   # aerospace X (nose)  -> world Y (forward)
    [1, 0, 0],   # aerospace Y (right) -> world X (right)
    [0, 0, -1],  # aerospace Z (down)  -> world Z (up)
])


def world_rotation_4x4(roll, pitch, yaw):
    R_aero = rotation_matrix(roll, pitch, yaw)
    R_world = _M @ R_aero @ _M.T
    T = np.eye(4)
    T[:3, :3] = R_world
    return T


# ---------------------------------------------------------------------
# Flying-wing mesh: hexagonal planform extruded for thickness, plus two
# red canted wingtip fins. Built directly in the Z-up world frame with
# nose pointing +Y, so it sits correctly before any rotation is applied.
# ---------------------------------------------------------------------
def build_flying_wing():
    t = 0.10  # half-thickness
    hexagon = np.array([
        [0.0, 1.8],    # nose tip
        [0.15, 1.0],   # right shoulder
        [1.4, -0.6],   # right wingtip
        [0.0, -0.9],   # tail center
        [-1.4, -0.6],  # left wingtip
        [-0.15, 1.0],  # left shoulder
    ])
    top = np.column_stack([hexagon, np.full(6, t)])
    bottom = np.column_stack([hexagon, np.full(6, -t)])
    pts = np.vstack([top, bottom])

    faces = [6, 0, 1, 2, 3, 4, 5]              # top hexagon
    faces += [6, 11, 10, 9, 8, 7, 6]            # bottom hexagon (reversed winding)
    for i in range(6):
        j = (i + 1) % 6
        faces += [4, i, j, 6 + j, 6 + i]        # side quads
    body = pv.PolyData(pts, faces)

    # Canted wingtip fins (red), one per side, matching the reference image
    def fin(tip_xy, sign):
        x, y = tip_xy
        base_front = [x, y + 0.15, 0.0]
        base_back = [x, y - 0.35, 0.0]
        apex = [x + sign * 0.35, y - 0.1, 0.55]
        pts_fin = np.array([base_front, base_back, apex])
        faces_fin = [3, 0, 1, 2]
        return pv.PolyData(pts_fin, faces_fin)

    right_fin = fin((1.4, -0.6), +1)
    left_fin = fin((-1.4, -0.6), -1)

    return body, right_fin, left_fin


# ---------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------
class Dashboard(QtWidgets.QMainWindow):
    status_signal = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("LIVE — MATEKSYS8743 Slim IMU Dashboard")
        self.resize(1200, 750)

        self.state = AttitudeState()
        self.stop_event = threading.Event()
        self.reader_thread = None
        self.status_signal.connect(self._set_status)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QHBoxLayout(central)

        # ---------------- Left control panel ----------------
        left_panel = QtWidgets.QVBoxLayout()
        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setFixedWidth(230)

        title = QtWidgets.QLabel("Scale Drone")
        title.setStyleSheet("font-size: 16px; font-weight: bold;")
        left_panel.addWidget(title)

        self.port_label = QtWidgets.QLabel("TARGET PORT: —")
        left_panel.addWidget(self.port_label)

        self.port_combo = QtWidgets.QComboBox()
        left_panel.addWidget(self.port_combo)

        self.baud_combo = QtWidgets.QComboBox()
        self.baud_combo.addItems(["115200", "57600", "921600", "460800"])
        left_panel.addWidget(self.baud_combo)

        refresh_btn = QtWidgets.QPushButton("REFRESH PORTS")
        refresh_btn.clicked.connect(self.refresh_ports)
        left_panel.addWidget(refresh_btn)

        self.connect_btn = QtWidgets.QPushButton("CONNECT")
        self.connect_btn.clicked.connect(self.toggle_connect)
        left_panel.addWidget(self.connect_btn)

        left_panel.addSpacing(15)

        self.lock_yaw_cb = QtWidgets.QCheckBox("LOCK YAW (Anti-Drift)")
        self.lock_yaw_cb.stateChanged.connect(self.on_lock_yaw)
        left_panel.addWidget(self.lock_yaw_cb)

        zero_btn = QtWidgets.QPushButton("ZERO IMU (C)")
        zero_btn.clicked.connect(self.zero_imu)
        zero_btn.setShortcut(QtGui.QKeySequence("C"))
        left_panel.addWidget(zero_btn)

        left_panel.addStretch()

        self.status_label = QtWidgets.QLabel("Status: idle")
        self.status_label.setWordWrap(True)
        left_panel.addWidget(self.status_label)

        layout.addWidget(left_widget)

        # ---------------- Center: PyVista 3D view ----------------
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter.interactor, stretch=1)

        body, right_fin, left_fin = build_flying_wing()
        self.body_actor = self.plotter.add_mesh(body, color="#3fa9a4", smooth_shading=True)
        self.right_fin_actor = self.plotter.add_mesh(right_fin, color="red")
        self.left_fin_actor = self.plotter.add_mesh(left_fin, color="red")
        self.plotter.show_grid()
        self.plotter.view_isometric()
        self.plotter.set_background("#f5f0e6")

        # ---------------- Right telemetry panel ----------------
        right_panel = QtWidgets.QVBoxLayout()
        right_widget = QtWidgets.QWidget()
        right_widget.setLayout(right_panel)
        right_widget.setFixedWidth(220)

        tel_title = QtWidgets.QLabel("--- TELEMETRY ---")
        tel_title.setStyleSheet("font-weight: bold;")
        right_panel.addWidget(tel_title)
        self.yaw_label = QtWidgets.QLabel("Yaw:    0.0")
        self.roll_label = QtWidgets.QLabel("Roll:   0.0")
        self.pitch_label = QtWidgets.QLabel("Pitch:  0.0")
        for w in (self.yaw_label, self.roll_label, self.pitch_label):
            w.setStyleSheet("font-family: monospace; font-size: 13px;")
            right_panel.addWidget(w)

        right_panel.addSpacing(10)
        gps_title = QtWidgets.QLabel("--- GPS ---")
        gps_title.setStyleSheet("font-weight: bold;")
        right_panel.addWidget(gps_title)
        self.gps_label = QtWidgets.QLabel("Sats: 0  |  No Fix")
        self.gps_label.setStyleSheet("font-family: monospace; font-size: 13px;")
        right_panel.addWidget(self.gps_label)

        right_panel.addSpacing(10)
        servo_title = QtWidgets.QLabel("--- SERVOS ---")
        servo_title.setStyleSheet("font-weight: bold;")
        right_panel.addWidget(servo_title)
        self.servo_left_label = QtWidgets.QLabel("Left:   0.0")
        self.servo_right_label = QtWidgets.QLabel("Right:  0.0")
        for w in (self.servo_left_label, self.servo_right_label):
            w.setStyleSheet("font-family: monospace; font-size: 13px;")
            right_panel.addWidget(w)

        right_panel.addSpacing(10)
        self.rate_label = QtWidgets.QLabel("Rate: 0.0 Hz")
        right_panel.addWidget(self.rate_label)

        right_panel.addStretch()
        layout.addWidget(right_widget)

        self.refresh_ports()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / RENDER_FPS))

    # ---------------- Port handling ----------------
    def refresh_ports(self):
        self.port_combo.clear()
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            label = f"{p.device} - {p.description}"
            self.port_combo.addItem(label, p.device)
        if ports:
            self.port_label.setText(f"TARGET PORT: {ports[0].device}")

    def toggle_connect(self):
        if self.state.connected:
            self.stop_event.set()
            self.connect_btn.setText("CONNECT")
            return

        if self.port_combo.currentData() is None:
            self._set_status("No port selected")
            return

        port = self.port_combo.currentData()
        baud = int(self.baud_combo.currentText())
        self.port_label.setText(f"TARGET PORT: {port}")
        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(
            target=mavlink_reader,
            args=(port, baud, self.state, self.stop_event, self.status_signal.emit),
            daemon=True,
        )
        self.reader_thread.start()
        self.connect_btn.setText("DISCONNECT")

    def _set_status(self, text):
        self.status_label.setText(f"Status: {text}")

    # ---------------- Yaw lock / zero IMU ----------------
    def on_lock_yaw(self, checked):
        if checked:
            self.state.zero_yaw_here()

    def zero_imu(self):
        self.state.zero_yaw_here()
        self._set_status("IMU zeroed (yaw reference reset)")

    # ---------------- Render loop ----------------
    def update_frame(self):
        roll, pitch, yaw, rate_hz, sats, fix_type, connected, gps_time, stale = \
            self.state.get_extrapolated()

        T = world_rotation_4x4(roll, pitch, yaw)
        self.body_actor.user_matrix = T
        self.right_fin_actor.user_matrix = T
        self.left_fin_actor.user_matrix = T

        self.yaw_label.setText(f"Yaw:    {np.degrees(yaw):6.1f}")
        self.roll_label.setText(f"Roll:   {np.degrees(roll):6.1f}")
        self.pitch_label.setText(f"Pitch:  {np.degrees(pitch):6.1f}")
        self.rate_label.setText(f"Rate: {rate_hz:4.1f} Hz")

        fix_label = FIX_TYPE_LABELS.get(fix_type, f"Unknown ({fix_type})")
        self.gps_label.setText(f"Sats: {sats}  |  {fix_label}")

        # Simple elevon mixing just to populate the servo readout —
        # adjust the mix/scale to match your actual control surface setup.
        pitch_deg = np.degrees(pitch)
        roll_deg = np.degrees(roll)
        servo_left = pitch_deg - roll_deg
        servo_right = pitch_deg + roll_deg
        self.servo_left_label.setText(f"Left:   {servo_left:6.1f}")
        self.servo_right_label.setText(f"Right:  {servo_right:6.1f}")

        self.plotter.render()

    def closeEvent(self, event):
        self.stop_event.set()
        super().closeEvent(event)


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Dashboard()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
