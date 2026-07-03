"""
telemetry_tab.py  —  Tab 3: Telemetry & Monitoring
Four panels:
  A) Vision Pose (/mavros/vision_pose/pose)
  B) Mission Script Output (mirrors mission console)
  C) Helper Nodes Output (yellow border + aruco)
  D) MAVROS Telemetry (battery, flow, position, velocity, height)
"""

import math
import re

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

from styles import COLORS, TAG_COLORS
from workers import MavrosTelemetryWorker, TopicEchoWorker

MONITORED_TOPICS = [
    "/mavros/imu/data_raw",
    "/camera/infra1/image_rect_raw",
    "/camera/infra2/image_rect_raw",
    "/camera/color/image_raw",
]


def _escape_html(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


class TelemetryTab(QWidget):
    """Tab 3 — Telemetry & Monitoring."""

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self._telem_worker: MavrosTelemetryWorker | None = None
        self._vision_worker: TopicEchoWorker | None = None
        self._connected = False
        self._build_ui()

    # ─── UI Build ────────────────────────────────────────────────
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(10)

        # Start/stop monitoring controls
        ctrl_row = QHBoxLayout()
        self._start_telem_btn = QPushButton("▶  Start Telemetry Monitoring")
        self._start_telem_btn.setObjectName("btn_green")
        self._start_telem_btn.setEnabled(False)
        self._start_telem_btn.clicked.connect(self._start_monitoring)

        self._stop_telem_btn = QPushButton("■  Stop")
        self._stop_telem_btn.setObjectName("btn_flat")
        self._stop_telem_btn.setEnabled(False)
        self._stop_telem_btn.clicked.connect(self._stop_monitoring)

        self._telem_status_lbl = QLabel("Not monitoring")
        self._telem_status_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 12px;")

        ctrl_row.addWidget(self._start_telem_btn)
        ctrl_row.addWidget(self._stop_telem_btn)
        ctrl_row.addWidget(self._telem_status_lbl)
        ctrl_row.addStretch()
        outer.addLayout(ctrl_row)

        # Hz Panel
        self._hz_panel = _HzPanel(MONITORED_TOPICS)
        outer.addWidget(self._hz_panel)

        # ── Top row: Vision Pose | MAVROS Telemetry ───────────────
        top_splitter = QSplitter(Qt.Horizontal)
        top_splitter.setChildrenCollapsible(False)

        # Panel A — Vision Pose
        self._vision_group = QGroupBox("Vision Pose  /mavros/vision_pose/pose")
        vp_layout = QVBoxLayout(self._vision_group)
        vp_layout.setSpacing(6)

        self._vp_fields = {}
        fields_a = [
            ("Position X", "pos_x"),
            ("Position Y", "pos_y"),
            ("Position Z", "pos_z"),
            ("Yaw (°)", "yaw"),
        ]
        for label, key in fields_a:
            row = _TelemetryRow(label)
            vp_layout.addWidget(row)
            self._vp_fields[key] = row

        vp_layout.addStretch()
        top_splitter.addWidget(self._vision_group)

        # Panel D — MAVROS Telemetry
        self._mavros_group = QGroupBox("MAVROS Telemetry")
        mv_layout = QVBoxLayout(self._mavros_group)
        mv_layout.setSpacing(6)

        self._mv_fields = {}
        fields_d = [
            ("🔋 Battery %",        "battery_pct"),
            ("🔋 Battery V",        "battery_v"),
            ("🌊 Optical Flow Q",   "flow_quality"),
            ("📍 Local X (NED)",    "pos_x"),
            ("📍 Local Y (NED)",    "pos_y"),
            ("↕️ Height Z",          "pos_z"),
            ("➡️ Velocity X",        "vel_x"),
            ("⬆️ Velocity Y",        "vel_y"),
        ]
        for label, key in fields_d:
            row = _TelemetryRow(label)
            mv_layout.addWidget(row)
            self._mv_fields[key] = row

        mv_layout.addStretch()
        top_splitter.addWidget(self._mavros_group)
        top_splitter.setSizes([400, 400])

        outer.addWidget(top_splitter)

        # ── Bottom row: Mission Output | Helper Nodes ─────────────
        bot_splitter = QSplitter(Qt.Horizontal)
        bot_splitter.setChildrenCollapsible(False)

        # Panel B — Mission Script Output
        mission_group = QGroupBox("Mission Script Output")
        ml = QVBoxLayout(mission_group)
        ml.setContentsMargins(6, 8, 6, 6)
        self._mission_console = _MiniConsole(tag_filter=["MISSION"])
        ml.addWidget(self._mission_console)
        bot_splitter.addWidget(mission_group)

        # Panel C — Helper Nodes Output
        helper_group = QGroupBox("Helper Nodes (Yellow Border + ArUco + Seeds)")
        hl = QVBoxLayout(helper_group)
        hl.setContentsMargins(6, 8, 6, 6)
        self._helper_console = _MiniConsole(tag_filter=["YELLOW", "ARUCO", "SEEDS"])
        hl.addWidget(self._helper_console)
        bot_splitter.addWidget(helper_group)

        bot_splitter.setSizes([500, 500])
        outer.addWidget(bot_splitter, stretch=1)

    # ─── External API ─────────────────────────────────────────────
    def on_ssh_connected(self):
        self._connected = True
        self._start_telem_btn.setEnabled(True)

    def on_ssh_disconnected(self):
        self._connected = False
        self._stop_monitoring()
        self._start_telem_btn.setEnabled(False)

    def pipe_console_line(self, tag: str, line: str):
        """
        Called by MissionTab to mirror mission/helper output here.
        Routed based on tag.
        """
        if tag in ("MISSION",):
            self._mission_console.append_line(tag, line)
        elif tag in ("YELLOW", "ARUCO", "SEEDS"):
            self._helper_console.append_line(tag, line)

    # ─── Monitoring start/stop ────────────────────────────────────
    def _start_monitoring(self):
        if not self.ssh_mgr.is_alive():
            return

        self._telem_status_lbl.setText("● Monitoring active")
        self._telem_status_lbl.setStyleSheet(f"color: {COLORS['green']}; font-size: 12px;")
        self._start_telem_btn.setEnabled(False)
        self._stop_telem_btn.setEnabled(True)

        # MAVROS telemetry worker
        self._telem_worker = MavrosTelemetryWorker(self.ssh_mgr)
        self._telem_worker.telemetry_updated.connect(self._on_telem_update)
        self._telem_worker.error_signal.connect(
            lambda m: self._telem_status_lbl.setText(f"⚠️ {m}")
        )
        self._telem_worker.start()

        # Vision pose worker
        self._vision_worker = TopicEchoWorker(self.ssh_mgr, "/mavros/vision_pose/pose")
        self._vision_worker.data_received.connect(self._on_vision_pose)
        self._vision_worker.error_signal.connect(
            lambda t, m: self._set_vp_error(m)
        )
        self._vision_worker.start()

    def _stop_monitoring(self):
        if self._telem_worker:
            self._telem_worker.stop()
            self._telem_worker.wait(2000)
            self._telem_worker = None
        if self._vision_worker:
            self._vision_worker.stop()
            self._vision_worker.wait(2000)
            self._vision_worker = None

        self._telem_status_lbl.setText("Not monitoring")
        self._telem_status_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 12px;")
        self._start_telem_btn.setEnabled(self._connected)
        self._stop_telem_btn.setEnabled(False)
        self._hz_panel.reset_all()

    # ─── Data handlers ─────────────────────────────────────────────
    def _on_telem_update(self, data: dict):
        # Battery
        if "battery_pct" in data:
            pct_str = data["battery_pct"]
            try:
                pct = float(pct_str)
                color = COLORS["green"] if pct > 0.3 else (
                    COLORS["amber"] if pct > 0.15 else COLORS["red"]
                )
                self._mv_fields["battery_pct"].set_value(f"{pct*100:.0f}%", color)
            except ValueError:
                self._mv_fields["battery_pct"].set_value(pct_str)
        if "battery_v" in data:
            self._mv_fields["battery_v"].set_value(f"{data['battery_v']} V")

        # Position
        for key in ("pos_x", "pos_y", "pos_z"):
            if key in data:
                self._mv_fields[key].set_value(f"{data[key]} m")

        # Velocity
        for key in ("vel_x", "vel_y"):
            if key in data:
                self._mv_fields[key].set_value(f"{data[key]} m/s")

        # Optical flow
        if "flow_quality" in data:
            q_str = data["flow_quality"]
            try:
                q = int(q_str)
                color = COLORS["green"] if q > 150 else (
                    COLORS["amber"] if q > 80 else COLORS["red"]
                )
                self._mv_fields["flow_quality"].set_value(str(q), color)
            except ValueError:
                self._mv_fields["flow_quality"].set_value(q_str)

    def _on_vision_pose(self, topic: str, raw_text: str):
        """Parse vision pose message block."""
        data = _parse_pose_block(raw_text)
        x = data.get("x", "—")
        y = data.get("y", "—")
        z = data.get("z", "—")
        qx = data.get("qx", None)
        qy = data.get("qy", None)
        qz = data.get("qz", None)
        qw = data.get("qw", None)

        self._vp_fields["pos_x"].set_value(f"{x} m" if x != "—" else "—")
        self._vp_fields["pos_y"].set_value(f"{y} m" if y != "—" else "—")
        self._vp_fields["pos_z"].set_value(f"{z} m" if z != "—" else "—")

        if all(v is not None for v in (qx, qy, qz, qw)):
            try:
                yaw = math.degrees(math.atan2(
                    2.0 * (float(qw) * float(qz) + float(qx) * float(qy)),
                    1.0 - 2.0 * (float(qy) ** 2 + float(qz) ** 2),
                ))
                self._vp_fields["yaw"].set_value(f"{yaw:.1f}°")
            except Exception:
                self._vp_fields["yaw"].set_value("—")

    def _set_vp_error(self, msg: str):
        for f in self._vp_fields.values():
            f.set_value("ERR", COLORS["red"])


# ─── Helpers ─────────────────────────────────────────────────────
def _parse_pose_block(text: str) -> dict:
    """Extract position and orientation from a rostopic echo pose block."""
    out = {}
    lines = text.splitlines()
    in_pos = in_orient = False
    for line in lines:
        s = line.strip()
        if s == "position:":
            in_pos = True; in_orient = False; continue
        if s == "orientation:":
            in_orient = True; in_pos = False; continue
        if s.startswith("header:") or s.startswith("pose:") or s.startswith("---"):
            in_pos = in_orient = False
            continue
        if in_pos:
            m = re.match(r"([xyz]):\s*([-\d.eE+]+)", s)
            if m:
                out[m.group(1)] = m.group(2)
        if in_orient:
            m = re.match(r"([xyzw]):\s*([-\d.eE+]+)", s)
            if m:
                out["q" + m.group(1)] = m.group(2)
    return out


# ─── Telemetry Row ────────────────────────────────────────────────
class _TelemetryRow(QWidget):
    """A label + value pair with colored value display."""

    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 3, 6, 3)
        layout.setSpacing(10)

        self.setStyleSheet(
            f"background-color: {COLORS['bg_input']}; "
            f"border: 1px solid {COLORS['border']}; border-radius: 4px;"
        )

        lbl = QLabel(label)
        lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 12px; font-weight: 500;")
        lbl.setMinimumWidth(150)

        self._val = QLabel("—")
        self._val.setStyleSheet(
            f"color: {COLORS['text']}; font-size: 14px; font-weight: 700; font-family: monospace;"
        )
        self._val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(lbl)
        layout.addStretch()
        layout.addWidget(self._val)
        self.setFixedHeight(34)

    def set_value(self, text: str, color: str = None):
        self._val.setText(text)
        c = color or COLORS["text"]
        self._val.setStyleSheet(
            f"color: {c}; font-size: 14px; font-weight: 700; font-family: monospace;"
        )


# ─── Mini Console ─────────────────────────────────────────────────
class _MiniConsole(QTextEdit):
    """Compact scrollable console filtered by tags."""

    MAX_LINES = 500

    def __init__(self, tag_filter: list[str] | None = None, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self._tag_filter = set(tag_filter) if tag_filter else None
        self._line_count = 0

    def append_line(self, tag: str, line: str):
        if self._tag_filter and tag not in self._tag_filter:
            return
        color = TAG_COLORS.get(tag, COLORS["text"])
        html = (
            f'<span style="color:{color}; font-family:monospace; font-size:12px;">'
            f'{_escape_html(str(line))}'
            f'</span>'
        )
        self.append(html)
        self._line_count += 1
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())

    def append(self, html: str):
        super().append(html)
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())


# Human-readable labels for monitored topics
_TOPIC_LABELS = {
    "/mavros/imu/data_raw":            ("IMU",    "imu/data_raw"),
    "/camera/infra1/image_rect_raw":   ("Infra 1", "infra1/image_rect_raw"),
    "/camera/infra2/image_rect_raw":   ("Infra 2", "infra2/image_rect_raw"),
    "/camera/color/image_raw":         ("Color",   "color/image_raw"),
}


class _HzPanel(QWidget):
    """Grid of topic → Hz rate indicators with full readable labels."""

    def __init__(self, topics: list[str], parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(8)
        self._labels: dict[str, QLabel] = {}   # topic → hz value label
        for topic in topics:
            category, short_path = _TOPIC_LABELS.get(topic, (topic.split("/")[-1], topic))
            card = QWidget()
            card.setStyleSheet(
                f"background-color: {COLORS['bg_input']}; "
                f"border: 1px solid {COLORS['border']}; border-radius: 5px;"
            )
            cl = QVBoxLayout(card)
            cl.setContentsMargins(10, 6, 10, 6)
            cl.setSpacing(1)

            # Main category label (e.g. "IMU", "Infra 1", "Color")
            cat_lbl = QLabel(category)
            cat_lbl.setStyleSheet(
                f"color: {COLORS['text']}; font-size: 13px; font-weight: 700;"
            )
            cat_lbl.setAlignment(Qt.AlignCenter)

            # Full path sub-label (e.g. "imu/data_raw")
            path_lbl = QLabel(short_path)
            path_lbl.setStyleSheet(
                f"color: {COLORS['text_muted']}; font-size: 10px;"
                f" font-family: monospace;"
            )
            path_lbl.setAlignment(Qt.AlignCenter)

            # Hz value
            hz_lbl = QLabel("— Hz")
            hz_lbl.setAlignment(Qt.AlignCenter)
            hz_lbl.setStyleSheet(
                f"color: {COLORS['text_muted']}; font-size: 15px; font-weight: 700;"
            )

            cl.addWidget(cat_lbl)
            cl.addWidget(path_lbl)
            cl.addWidget(hz_lbl)
            layout.addWidget(card)
            self._labels[topic] = hz_lbl

    def update_hz(self, topic: str, hz: float):
        if topic in self._labels:
            lbl = self._labels[topic]
            lbl.setText(f"{hz:.1f} Hz")
            color = COLORS["green"] if hz > 0.5 else COLORS["red"]
            lbl.setStyleSheet(
                f"color: {color}; font-size: 15px; font-weight: 700;"
            )

    def set_dead(self, topic: str):
        if topic in self._labels:
            lbl = self._labels[topic]
            lbl.setText("NO DATA")
            lbl.setStyleSheet(
                f"color: {COLORS['red']}; font-size: 12px; font-weight: 700;"
            )

    def reset_all(self):
        for topic, lbl in self._labels.items():
            lbl.setText("— Hz")
            lbl.setStyleSheet(
                f"color: {COLORS['text_muted']}; font-size: 15px; font-weight: 700;"
            )
