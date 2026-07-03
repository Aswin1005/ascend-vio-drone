"""
mission_tab.py  —  Tab 2: Mission Control
Handles the full mission sequence:
  1. Launch full pipeline (roslaunch)
  2. Set MAVROS stream rate
  3. Monitor topic rates
  4. Wait for VIO init (vision_pose/pose with non-zero x/y)
  5. Launch all 4 mission processes simultaneously
  + Emergency stop controls
  + Color-coded console log
"""

import json
import os
import time
import shutil

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSizePolicy, QSplitter, QTextEdit, QVBoxLayout, QWidget,
    QGridLayout,
)

from styles import COLORS, TAG_COLORS
from workers import AllTopicsHzWorker, MavrosTelemetryWorker, OpenVinsWatcher, StreamWorker, SeedTransferWorker

ROS_SOURCE = "source /opt/ros/melodic/setup.bash && source ~/catkin_ws/devel/setup.bash"

MONITORED_TOPICS = [
    "/mavros/imu/data_raw",
    "/camera/infra1/image_rect_raw",
    "/camera/infra2/image_rect_raw",
    "/camera/color/image_raw",
]

COLOR_CAMERA_TOPIC = "/camera/color/image_raw"


def _escape_html(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def load_config() -> dict:
    try:
        with open(os.path.expanduser("~/.ascend_gcs_config.json")) as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(data: dict):
    try:
        path = os.path.expanduser("~/.ascend_gcs_config.json")
        existing = {}
        try:
            with open(path) as f:
                existing = json.load(f)
        except Exception:
            pass
        existing.update(data)
        with open(path, "w") as f:
            json.dump(existing, f, indent=2)
    except Exception:
        pass


class MissionTab(QWidget):
    """Tab 2 — Mission Control."""

    transfer_completed = pyqtSignal(str)
    mission_completed_signal = pyqtSignal()

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr

        # Workers
        self._pipeline_worker: StreamWorker | None = None
        self._stream_rate_worker: StreamWorker | None = None   # kept alive to prevent GC crash
        self._telem_worker: MavrosTelemetryWorker | None = None
        self._hz_worker: AllTopicsHzWorker | None = None
        self._openvins_watcher: OpenVinsWatcher | None = None
        self._mission_workers: dict[str, StreamWorker] = {}
        self._telem_tab = None

        # State
        self._pipeline_running = False
        self._stream_rate_set = False
        self._openvins_ready = False
        self._mission_running = False
        self._countdown_remaining = 0

        # Color camera: only warn once if it goes dead, never auto-restart
        self._color_cam_warned = False

        # Countdown timer
        self._countdown_timer = QTimer(self)
        self._countdown_timer.setInterval(1000)
        self._countdown_timer.timeout.connect(self._countdown_tick)

        # Connection alive checker
        self._alive_timer = QTimer(self)
        self._alive_timer.setInterval(3000)
        self._alive_timer.timeout.connect(self._check_connection)

        self._build_ui()
        self._load_config()

    def set_telem_tab(self, telem_tab):
        self._telem_tab = telem_tab

    # ─── UI ──────────────────────────────────────────────────────
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(12, 12, 12, 12)
        outer.setSpacing(10)

        # ── Top: sequence + emergency row ────────────────────────
        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        # Sequence Group
        seq_group = QGroupBox("Mission Sequence")
        seq_layout = QVBoxLayout(seq_group)
        seq_layout.setSpacing(8)

        # Step 1 — Pipeline
        s1_row = QHBoxLayout()
        self._step1_indicator = _StepIndicator("1")
        self._launch_btn = QPushButton("▶  Launch Full Pipeline")
        self._launch_btn.setObjectName("btn_green")
        self._launch_btn.setEnabled(False)
        self._launch_btn.clicked.connect(self._on_launch_pipeline)
        s1_row.addWidget(self._step1_indicator)
        s1_row.addWidget(self._launch_btn)
        s1_row.addStretch()
        seq_layout.addLayout(s1_row)

        # Step 2 — Stream rate
        s2_row = QHBoxLayout()
        self._step2_indicator = _StepIndicator("2")
        self._stream_rate_btn = QPushButton("📡  Set Stream Rate (200 Hz)")
        self._stream_rate_btn.setEnabled(False)
        self._stream_rate_btn.clicked.connect(self._on_set_stream_rate)
        s2_row.addWidget(self._step2_indicator)
        s2_row.addWidget(self._stream_rate_btn)
        s2_row.addStretch()
        seq_layout.addLayout(s2_row)

        # Step 3 — MAVROS Telemetry Panel
        s3_row = QHBoxLayout()
        self._step3_indicator = _StepIndicator("3")
        s3_lbl = QLabel("MAVROS Telemetry:")
        s3_lbl.setObjectName("label_dim")
        s3_row.addWidget(self._step3_indicator)
        s3_row.addWidget(s3_lbl)
        s3_row.addStretch()
        seq_layout.addLayout(s3_row)

        self._telem_panel = _MavrosTelemPanel()
        seq_layout.addWidget(self._telem_panel)

        # Step 4 — OpenVINS
        s4_row = QHBoxLayout()
        self._step4_indicator = _StepIndicator("4")
        self._openvins_label = QLabel("⏳  OpenVINS: Waiting for launch...")
        self._openvins_label.setStyleSheet(f"color: {COLORS['text_dim']};")
        s4_row.addWidget(self._step4_indicator)
        s4_row.addWidget(self._openvins_label)
        s4_row.addStretch()
        seq_layout.addLayout(s4_row)

        # Step 5 — Mission
        s5_row = QHBoxLayout()
        self._step5_indicator = _StepIndicator("5")

        # Script dropdown
        self._script_combo = QComboBox()
        self._script_combo.setMinimumWidth(200)
        self._script_combo.setPlaceholderText("Select mission script...")
        self._script_combo.currentTextChanged.connect(self._on_script_changed)

        self._refresh_scripts_btn = QPushButton("🔄")
        self._refresh_scripts_btn.setObjectName("btn_flat")
        self._refresh_scripts_btn.setMaximumWidth(36)
        self._refresh_scripts_btn.setToolTip("Refresh scripts from Jetson")
        self._refresh_scripts_btn.setEnabled(False)
        self._refresh_scripts_btn.clicked.connect(self._on_refresh_scripts)

        self._mission_btn = QPushButton("🚀  Start Mission")
        self._mission_btn.setObjectName("btn_green")
        self._mission_btn.setMinimumWidth(140)
        self._mission_btn.setEnabled(False)
        self._mission_btn.clicked.connect(self._on_start_mission)

        s5_row.addWidget(self._step5_indicator)
        s5_row.addWidget(QLabel("Script:"))
        s5_row.addWidget(self._script_combo)
        s5_row.addWidget(self._refresh_scripts_btn)
        s5_row.addWidget(self._mission_btn)
        s5_row.addStretch()
        seq_layout.addLayout(s5_row)

        top_row.addWidget(seq_group, stretch=3)

        # Emergency Stop Group
        emer_group = QGroupBox("⚡ Emergency Controls")
        emer_layout = QVBoxLayout(emer_group)
        emer_layout.setSpacing(10)
        emer_layout.setAlignment(Qt.AlignTop)

        # Land button
        self._land_btn = QPushButton("🟡  Land\n(send 'l' key)")
        self._land_btn.setObjectName("btn_amber")
        self._land_btn.setMinimumHeight(70)
        self._land_btn.setEnabled(False)
        self._land_btn.clicked.connect(self._on_land)

        # Kill all button
        self._kill_btn = QPushButton("🔴  KILL ALL\nProcesses")
        self._kill_btn.setObjectName("btn_red")
        self._kill_btn.setMinimumHeight(70)
        self._kill_btn.setEnabled(False)
        self._kill_btn.clicked.connect(self._on_kill_all)

        emer_layout.addWidget(self._land_btn)
        emer_layout.addWidget(self._kill_btn)
        emer_layout.addStretch()

        top_row.addWidget(emer_group, stretch=1)
        outer.addLayout(top_row)

        # ── Console Log ───────────────────────────────────────────
        console_group = QGroupBox("Mission Console Log")
        console_layout = QVBoxLayout(console_group)
        console_layout.setContentsMargins(6, 8, 6, 6)

        console_header = QHBoxLayout()
        self._console_status_lbl = QLabel("Idle")
        self._console_status_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 12px;")
        clear_btn = QPushButton("Clear")
        clear_btn.setObjectName("btn_flat")
        clear_btn.setMaximumWidth(60)
        clear_btn.setMaximumHeight(26)
        clear_btn.clicked.connect(lambda: self._console.clear())
        console_header.addWidget(self._console_status_lbl)
        console_header.addStretch()
        console_header.addWidget(clear_btn)
        console_layout.addLayout(console_header)

        self._console = MultiConsoleWidget()
        self._console.setMinimumHeight(450)
        console_layout.addWidget(self._console)
        outer.addWidget(console_group, stretch=2)

    # ─── External API ────────────────────────────────────────────
    def on_ssh_connected(self):
        """Called when SSH connection is established."""
        self._launch_btn.setEnabled(True)
        self._refresh_scripts_btn.setEnabled(True)
        self._kill_btn.setEnabled(True)
        self._alive_timer.start()
        self._on_refresh_scripts()
        self._console.append("SYSTEM", "SSH connected — ready to launch.")
        # Start telemetry monitor immediately on connection
        self._start_telem_monitors()

    def on_ssh_disconnected(self):
        """Called when SSH disconnects."""
        self._alive_timer.stop()
        self._launch_btn.setEnabled(False)
        self._refresh_scripts_btn.setEnabled(False)
        self._mission_btn.setEnabled(False)
        self._land_btn.setEnabled(False)
        self._kill_btn.setEnabled(False)
        self._stop_all_workers()
        self._console.append("SYSTEM", "SSH disconnected.")

    # ─── Config ──────────────────────────────────────────────────
    def _load_config(self):
        cfg = load_config()
        if "last_script" in cfg:
            idx = self._script_combo.findText(cfg["last_script"])
            if idx >= 0:
                self._script_combo.setCurrentIndex(idx)

    def _on_script_changed(self, text: str):
        if text:
            save_config({"last_script": text})
        self._update_mission_btn_state()

    def _update_mission_btn_state(self):
        if not self._openvins_ready:
            self._mission_btn.setEnabled(False)
            self._mission_btn.setText("🚀  Start Mission")
            return

        if self._countdown_remaining > 0:
            self._mission_btn.setEnabled(False)
            self._mission_btn.setText(f"🚀  Start Mission ({self._countdown_remaining}s)")
            return

        if self._mission_running:
            self._mission_btn.setEnabled(False)
            self._mission_btn.setText("🚀  Mission Running")
            return

        has_script = bool(self._script_combo.currentText())
        self._mission_btn.setEnabled(has_script)
        self._mission_btn.setText("🚀  Start Mission")

    # ─── Script refresh ──────────────────────────────────────────
    def _on_refresh_scripts(self):
        if not self.ssh_mgr.is_alive():
            return
        self._console.append("SYSTEM", "Fetching scripts from Jetson...")
        scripts = self.ssh_mgr.sftp_list_remote_scripts(
            "~/catkin_ws/src/vio_bridge/scripts"
        )
        # Exclude helper nodes that shouldn't be the "main" script
        EXCLUDED = {
            "vio_bridge_node.py",
            "closer_yellow_border_node.py",
            "yellow_border_node.py",
            "check_xy_offsets.py",
        }
        scripts = [s for s in scripts if s not in EXCLUDED]
        prev = self._script_combo.currentText()
        self._script_combo.blockSignals(True)
        self._script_combo.clear()
        for s in scripts:
            self._script_combo.addItem(s)
        if prev:
            idx = self._script_combo.findText(prev)
            if idx >= 0:
                self._script_combo.setCurrentIndex(idx)
        self._script_combo.blockSignals(False)
        self._console.append("SYSTEM", f"Found {len(scripts)} mission scripts.")
        self._update_mission_btn_state()

    # ─── Step 1: Launch Pipeline ──────────────────────────────────
    def _on_launch_pipeline(self):
        if not self.ssh_mgr.is_alive():
            self._console.append("ERROR", "Not connected to Jetson.")
            return

        self._stop_pipeline()
        self._console.append("PIPELINE", "Launching full pipeline...")
        self._step1_indicator.set_state("running")

        cmd = f"bash -c '{self.ssh_mgr.ros_env_prefix}{ROS_SOURCE} && roslaunch vio_bridge full_pipeline.launch'"
        self._pipeline_worker = StreamWorker(self.ssh_mgr, cmd, "PIPELINE", use_pty=True)
        self._pipeline_worker.line_received.connect(self._on_console_line)
        self._pipeline_worker.finished.connect(self._on_pipeline_finished)
        self._pipeline_worker.error.connect(
            lambda tag, msg: self._console.append("ERROR", f"Pipeline error: {msg}")
        )
        self._pipeline_worker.start()

        self._pipeline_running = True
        self._launch_btn.setText("🔄  Restart Pipeline")
        self._stream_rate_btn.setEnabled(True)
        self._step2_indicator.set_state("pending")

        # Start Hz monitors
        self._start_hz_monitors()

    def _stop_pipeline(self):
        if self._pipeline_worker:
            self._pipeline_worker.stop()
            self._pipeline_worker.wait(2000)
            self._pipeline_worker = None
        self._pipeline_running = False
        self._stop_hz_worker()
        self._color_cam_warned = False

    def _on_pipeline_finished(self, tag: str, exit_code: int):
        self._pipeline_running = False
        self._step1_indicator.set_state("idle")
        self._console.append("PIPELINE", f"Pipeline exited (code {exit_code}).")

    # ─── Step 2: Stream Rate ──────────────────────────────────────
    def _on_set_stream_rate(self):
        if not self.ssh_mgr.is_alive():
            return
        self._console.append("SYSTEM", "Setting MAVROS stream rate to 200 Hz...")
        self._step2_indicator.set_state("running")

        cmd = f"bash -c '{self.ssh_mgr.ros_env_prefix}{ROS_SOURCE} && rosservice call /mavros/set_stream_rate 0 200 1'"
        # IMPORTANT: store on self — local variables get GC'd while QThread still runs, causing a crash
        self._stream_rate_worker = StreamWorker(self.ssh_mgr, cmd, "SYSTEM", use_pty=True)
        self._stream_rate_worker.line_received.connect(self._on_console_line)
        self._stream_rate_worker.finished.connect(self._on_stream_rate_done)
        self._stream_rate_worker.start()
        self._stream_rate_btn.setEnabled(False)

    def _on_stream_rate_done(self, tag: str, exit_code: int):
        if exit_code == 0:
            self._step2_indicator.set_state("done")
            self._stream_rate_set = True
            self._console.append("INFO", "Stream rate set to 200 Hz ✅")
            self._step3_indicator.set_state("running")
            # Start telemetry monitors now that stream rate is 200 Hz
            self._console.append("SYSTEM", "Starting MAVROS telemetry monitor...")
            self._start_telem_monitors()
            self._start_hz_monitors()
            # Start OpenVINS watcher
            self._start_openvins_watcher()
        else:
            self._step2_indicator.set_state("error")
            self._console.append("ERROR", "Failed to set stream rate. Is MAVROS connected?")
            self._stream_rate_btn.setEnabled(True)

    # ─── Step 3: Telemetry & Hz Monitors ───────────────────────────
    def _start_telem_monitors(self):
        """Start MavrosTelemetryWorker to show real-time telemetry."""
        self._stop_telem_worker()
        w = MavrosTelemetryWorker(self.ssh_mgr)
        w.telemetry_updated.connect(self._telem_panel.update_data)
        w.error_signal.connect(
            lambda m: self._console.append("ERROR", f"Telem monitor: {m}")
        )
        w.start()
        self._telem_worker = w

    def _stop_telem_worker(self):
        if self._telem_worker:
            self._telem_worker.stop()
            self._telem_worker.wait(1000)
            self._telem_worker = None
        if hasattr(self, "_telem_panel"):
            self._telem_panel.reset_all()

    def _start_hz_monitors(self):
        """Start AllTopicsHzWorker in background and link to TelemetryTab's Hz panel."""
        self._stop_hz_worker()
        w = AllTopicsHzWorker(self.ssh_mgr, MONITORED_TOPICS)
        if hasattr(self, "_telem_tab") and self._telem_tab:
            w.rate_updated.connect(self._telem_tab._hz_panel.update_hz)
            w.no_messages.connect(self._telem_tab._hz_panel.set_dead)
        w.start()
        self._hz_worker = w

    def _stop_hz_worker(self):
        if self._hz_worker:
            self._hz_worker.stop()
            self._hz_worker.wait(1000)
            self._hz_worker = None
        if hasattr(self, "_telem_tab") and self._telem_tab:
            self._telem_tab._hz_panel.reset_all()

    # ─── Step 4: OpenVINS ────────────────────────────────────────
    def _start_openvins_watcher(self):
        if self._openvins_watcher:
            self._openvins_watcher.stop()
            self._openvins_watcher.wait(1000)
        self._openvins_watcher = OpenVinsWatcher(self.ssh_mgr)
        self._openvins_watcher.status_update.connect(self._on_openvins_status)
        self._openvins_watcher.initialized.connect(self._on_openvins_initialized)
        self._openvins_watcher.start()
        self._step4_indicator.set_state("running")

    def _on_openvins_status(self, msg: str):
        self._openvins_label.setText(msg)
        if "Waiting" in msg:
            self._openvins_label.setStyleSheet(f"color: {COLORS['amber']};")
        else:
            self._openvins_label.setStyleSheet(f"color: {COLORS['green']};")

    def _on_openvins_initialized(self):
        self._openvins_ready = True
        self._step4_indicator.set_state("done")
        self._step5_indicator.set_state("pending")
        self._console.append("INFO", "OpenVINS initialized! Starting 5-second countdown...")

        # 5-second countdown before enabling mission button
        self._countdown_remaining = 5
        self._countdown_timer.start()
        self._update_mission_btn_state()

    def _countdown_tick(self):
        self._countdown_remaining -= 1
        if self._countdown_remaining <= 0:
            self._countdown_timer.stop()
            self._console.append("INFO", "✅ Ready to start mission!")
        self._update_mission_btn_state()

    # ─── Step 5: Mission ─────────────────────────────────────────
    def _on_start_mission(self):
        script_name = self._script_combo.currentText()
        if not script_name:
            self._console.append("ERROR", "No mission script selected!")
            return
        if not self.ssh_mgr.is_alive():
            self._console.append("ERROR", "Not connected!")
            return

        self._mission_running = True
        self._land_btn.setEnabled(True)
        self._step5_indicator.set_state("running")
        self._console.append("INFO", f"🚀 Starting mission: {script_name}")
        self._update_mission_btn_state()
        self._transferring_seeds = False
        self._pending_script_name = script_name

        # ── Sequential launch with delays ──────────────────────────
        # 1. Yellow border node — start immediately
        self._start_process(
            "YELLOW",
            f"bash -c '{self.ssh_mgr.ros_env_prefix}{ROS_SOURCE} && "
            f"python3 ~/catkin_ws/src/vio_bridge/scripts/closer_yellow_border_node.py'",
        )
        self._console.append("INFO", "🟡 Yellow border node started. Main script launches in 5 s...")

        # 2. Main mission script — start after 5 s
        QTimer.singleShot(5000, self._start_mission_script_delayed)

        # 3. Seed tracker — start 10 s after mission script (15 s total)
        QTimer.singleShot(15000, self._start_seeds_delayed)

        self._console_status_lbl.setText(f"● Mission running — {script_name}")
        self._console_status_lbl.setStyleSheet(f"color: {COLORS['green']};")

    def _start_mission_script_delayed(self):
        """Called 5 s after YELLOW starts."""
        script_name = getattr(self, '_pending_script_name', '')
        if not script_name:
            return
        self._console.append("INFO", "🚀 Starting main mission script...")
        self._start_process(
            "MISSION",
            f"bash -c '{self.ssh_mgr.ros_env_prefix}{ROS_SOURCE} && "
            f"python3 ~/catkin_ws/src/vio_bridge/scripts/{script_name}'",
        )
        # Send hover-mode selection inputs
        QTimer.singleShot(1500, lambda: self._send_mission_input("2\n"))
        QTimer.singleShot(3000, lambda: self._send_mission_input("1\n"))

    def _start_seeds_delayed(self):
        """Called 15 s after mission start (10 s after MISSION)."""
        self._console.append("INFO", "🌱 Starting seed tracker...")
        self._start_process(
            "SEEDS",
            f"bash -c '{self.ssh_mgr.ros_env_prefix}{ROS_SOURCE} && "
            f"python3 -m seed_tracker.src.texture_main'",
        )

    def _start_process(self, tag: str, cmd: str):
        w = StreamWorker(self.ssh_mgr, cmd, tag, use_pty=True)
        w.line_received.connect(self._on_console_line)
        w.finished.connect(self._on_process_finished)
        w.error.connect(
            lambda t, m: self._console.append("ERROR", f"[{t}] error: {m}")
        )
        w.start()
        self._mission_workers[tag] = w
        self._console.append("SYSTEM", f"Started process [{tag}]")

    def _send_mission_input(self, text: str):
        worker = self._mission_workers.get("MISSION")
        if worker:
            worker.send_stdin(text)
            self._console.append("SYSTEM", f"Sent input: {text.strip()}")

    def _on_process_finished(self, tag: str, exit_code: int):
        self._console.append(
            "SYSTEM", f"[{tag}] process exited (code {exit_code})."
        )
        self._mission_workers.pop(tag, None)
        if not self._mission_workers:
            self._mission_running = False
            self._land_btn.setEnabled(False)
            self._step5_indicator.set_state("done")
            self._console_status_lbl.setText("All processes finished.")
            self._console_status_lbl.setStyleSheet(f"color: {COLORS['text_dim']};")
            self._update_mission_btn_state()

    # ─── Console line handler ─────────────────────────────────────
    def _on_console_line(self, tag: str, line: str):
        self._console.append(tag, line)
        # Only trigger on the final ARENA NAV banner, e.g.:
        #   "ARENA NAV LOITER — COMPLETE" or "ARENA NAV GUIDED — COMPLETE"
        # NOT on intermediate lines like "All loops complete."
        line_up = line.upper()
        if tag == "MISSION" and "ARENA NAV" in line_up and "COMPLETE" in line_up:
            self._on_mission_complete_triggered()


    def _on_mission_complete_triggered(self):
        if hasattr(self, "_transferring_seeds") and self._transferring_seeds:
            return
        self._transferring_seeds = True

        self.mission_completed_signal.emit()

        self._console.append("SYSTEM", "Mission complete — transferring captured seeds to GCS...")
        self._console_status_lbl.setText("Transferring captured seeds...")
        self._console_status_lbl.setStyleSheet(f"color: {COLORS['amber']}; font-weight: bold;")
        
        # Stop all local mission workers and run remote pkill command to clean up the Jetson
        self._console.append("SYSTEM", "🧹 Cleaning up mission processes on Jetson...")
        for tag in list(self._mission_workers.keys()):
            w = self._mission_workers.pop(tag, None)
            if w:
                w.stop()
                w.wait(1000)

        kill_cmd = (
            "pkill -f arena_nav; pkill -f loiter_sweep; "
            "pkill -f vio_mission; pkill -f gps_mission; "
            "pkill -f closer_yellow_border; pkill -f yellow_border; "
            "pkill -f check_xy_offsets; pkill -f texture_main"
        )
        try:
            self.ssh_mgr.exec(kill_cmd, timeout=5)
        except Exception as e:
            self._console.append("ERROR", f"Mission cleanup pkill failed: {e}")

        # Update mission running state and UI controls
        self._mission_running = False
        self._land_btn.setEnabled(False)
        self._step5_indicator.set_state("done")
        self._update_mission_btn_state()

        # Local dir to store captured seeds
        local_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "transferred_seeds")
        if os.path.exists(local_dir):
            try:
                shutil.rmtree(local_dir)
            except Exception:
                pass
        os.makedirs(local_dir, exist_ok=True)
        
        # Run SeedTransferWorker
        remote_dir = f"/home/{self.ssh_mgr.username}/seed_tracker/detections"
        self._transfer_worker = SeedTransferWorker(self.ssh_mgr, remote_dir, local_dir)
        self._transfer_worker.progress.connect(lambda msg: self._console.append("SYSTEM", f"[TRANSFER] {msg}"))
        self._transfer_worker.finished.connect(self._on_transfer_finished)
        self._transfer_worker.start()

    def _on_transfer_finished(self, success: bool, msg: str):
        self._transferring_seeds = False
        if success:
            self._console.append("SYSTEM", "✅ Seeds transferred — opening Seed Verification...")
            local_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "transferred_seeds")
            # Shutdown Jetson for charging while user reviews seeds
            self._shutdown_jetson()
            self.transfer_completed.emit(local_dir)
        else:
            self._console.append("ERROR", f"Failed to transfer captured seeds: {msg}")

    def _shutdown_jetson(self):
        """Send sudo shutdown to Jetson in a background thread (connection drops immediately)."""
        import threading
        def _do_shutdown():
            try:
                self.ssh_mgr.exec("echo 'isro@123' | sudo -S shutdown now", timeout=8)
            except Exception:
                pass  # Expected — SSH drops when Jetson shuts down
        threading.Thread(target=_do_shutdown, daemon=True).start()
        self._console.append("SYSTEM", "🔌 Jetson shutdown initiated — powering off for charging...")

    # ─── Emergency controls ───────────────────────────────────────
    def _on_land(self):
        """Send 'l' to mission script stdin."""
        if "MISSION" in self._mission_workers:
            self._mission_workers["MISSION"].send_stdin("l")
            self._console.append("SYSTEM", "🟡 Sent 'l' (land) to mission script.")
            self._console.append("SYSTEM", "⏳ Land signal sent. Mission will auto-terminate and reset in 20 seconds...")
            QTimer.singleShot(20000, self._auto_terminate_mission)
        else:
            self._console.append("ERROR", "Mission script not running.")

    def _auto_terminate_mission(self):
        if not self._mission_running:
            return

        self._console.append("SYSTEM", "⏳ 20 seconds elapsed since land request. Auto-terminating mission processes...")

        # 1. Stop all local mission workers
        for tag in list(self._mission_workers.keys()):
            w = self._mission_workers.pop(tag, None)
            if w:
                w.stop()
                w.wait(1000)

        # 2. Run remote pkill command for mission scripts
        kill_cmd = (
            "pkill -f arena_nav; pkill -f loiter_sweep; "
            "pkill -f vio_mission; pkill -f gps_mission; "
            "pkill -f closer_yellow_border; pkill -f yellow_border; "
            "pkill -f check_xy_offsets; pkill -f texture_main"
        )
        try:
            self.ssh_mgr.exec(kill_cmd, timeout=5)
        except Exception as e:
            self._console.append("ERROR", f"Auto-terminate pkill failed: {e}")

        # 3. Reset UI state
        self._mission_running = False
        self._land_btn.setEnabled(False)
        self._step5_indicator.set_state("done")
        self._console_status_lbl.setText("Mission terminated after landing.")
        self._console_status_lbl.setStyleSheet(f"color: {COLORS['text_dim']};")
        self._update_mission_btn_state()
        self._console.append("SYSTEM", "✅ Mission reset complete. You can launch a new mission script now.")

    def _on_kill_all(self):
        """Kill all remote processes."""
        self._console.append("ERROR", "🔴 KILL ALL — terminating all processes on Jetson...")
        kill_cmd = (
            "pkill -f roslaunch; pkill -f roscore; "
            "pkill -f arena_nav; pkill -f loiter_sweep; "
            "pkill -f vio_mission; pkill -f gps_mission; "
            "pkill -f closer_yellow_border; pkill -f yellow_border; "
            "pkill -f check_xy_offsets; pkill -f texture_main; "
            "pkill -f vio_bridge_node; echo 'Kill done'"
        )
        self._stop_all_workers()
        try:
            _, out, _ = self.ssh_mgr.exec(kill_cmd, timeout=10)
            self._console.append("SYSTEM", f"Kill result: {out.strip()}")
        except Exception as e:
            self._console.append("ERROR", f"Kill failed: {e}")

        self._pipeline_running = False
        self._mission_running = False
        self._step1_indicator.set_state("idle")
        self._step5_indicator.set_state("idle")
        self._land_btn.setEnabled(False)
        self._launch_btn.setText("▶  Launch Full Pipeline")
        self._console_status_lbl.setText("All processes killed.")
        self._console_status_lbl.setStyleSheet(f"color: {COLORS['red']};")
        self._update_mission_btn_state()

    def _stop_all_workers(self):
        if self._pipeline_worker:
            self._pipeline_worker.stop()
            self._pipeline_worker.wait(1000)
            self._pipeline_worker = None
        self._stop_telem_worker()
        self._stop_hz_worker()
        if self._openvins_watcher:
            self._openvins_watcher.stop()
            self._openvins_watcher.wait(1000)
            self._openvins_watcher = None
        for w in list(self._mission_workers.values()):
            w.stop()
            w.wait(1000)
        self._mission_workers.clear()

    # ─── Misc ─────────────────────────────────────────────────────
    def _check_connection(self):
        if not self.ssh_mgr.is_alive() and self._pipeline_running:
            self._console.append("ERROR", "SSH connection lost!")
            self._stop_all_workers()

    def get_mission_workers(self) -> dict:
        """Expose mission workers for the Telemetry tab to mirror output."""
        return self._mission_workers


class _MavrosTelemPanel(QWidget):
    """Row of card widgets showing real-time MAVROS telemetry."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 4, 0, 4)
        layout.setSpacing(8)

        self.battery_val = self._create_card(layout, "🔋 Voltage", "— V")
        self.pose_x_val = self._create_card(layout, "📍 Position X", "— m")
        self.pose_y_val = self._create_card(layout, "📍 Position Y", "— m")
        self.pose_z_val = self._create_card(layout, "↕️ Position Z", "— m")
        self.vel_x_val = self._create_card(layout, "➡️ Velocity X", "— m/s")
        self.vel_y_val = self._create_card(layout, "⬆️ Velocity Y", "— m/s")

    def _create_card(self, parent_layout: QHBoxLayout, title: str, init_val: str) -> QLabel:
        card = QWidget()
        card.setStyleSheet(
            f"background-color: {COLORS['bg_input']}; "
            f"border: 1px solid {COLORS['border']}; border-radius: 5px;"
        )
        cl = QVBoxLayout(card)
        cl.setContentsMargins(10, 6, 10, 6)
        cl.setSpacing(1)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px; font-weight: 600;")
        title_lbl.setAlignment(Qt.AlignCenter)

        val_lbl = QLabel(init_val)
        val_lbl.setStyleSheet(f"color: {COLORS['text']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        val_lbl.setAlignment(Qt.AlignCenter)

        cl.addWidget(title_lbl)
        cl.addWidget(val_lbl)
        parent_layout.addWidget(card)
        return val_lbl

    def update_data(self, data: dict):
        # --- Battery voltage ---
        bv = data.get("battery_v", "—")
        try:
            bval = float(bv)
            color = COLORS["green"] if bval >= 14.0 else (COLORS["btn_amber"] if bval >= 13.0 else COLORS["btn_red"])
            self.battery_val.setText(f"{bval:.2f} V")
            self.battery_val.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

        # --- Position X, Y, Z (keep last good value on poll miss) ---
        px = data.get("pos_x", "—")
        py = data.get("pos_y", "—")
        pz = data.get("pos_z", "—")

        try:
            self.pose_x_val.setText(f"{float(px):+.3f} m")
            self.pose_x_val.setStyleSheet(f"color: {COLORS['blue']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

        try:
            self.pose_y_val.setText(f"{float(py):+.3f} m")
            self.pose_y_val.setStyleSheet(f"color: {COLORS['blue']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

        try:
            self.pose_z_val.setText(f"{float(pz):+.3f} m")
            self.pose_z_val.setStyleSheet(f"color: {COLORS['blue']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

        # --- Velocity X, Y (keep last good value on poll miss) ---
        vx = data.get("vel_x", "—")
        vy = data.get("vel_y", "—")

        try:
            self.vel_x_val.setText(f"{float(vx):+.2f} m/s")
            self.vel_x_val.setStyleSheet(f"color: {COLORS['purple']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

        try:
            self.vel_y_val.setText(f"{float(vy):+.2f} m/s")
            self.vel_y_val.setStyleSheet(f"color: {COLORS['purple']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        except (ValueError, TypeError):
            pass  # keep last known value

    def reset_all(self):
        self.battery_val.setText("— V")
        self.battery_val.setStyleSheet(f"color: {COLORS['text_muted']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        for lbl in (self.pose_x_val, self.pose_y_val, self.pose_z_val):
            lbl.setText("— m")
            lbl.setStyleSheet(f"color: {COLORS['text_muted']}; font-size: 14px; font-weight: 700; font-family: monospace;")
        for lbl in (self.vel_x_val, self.vel_y_val):
            lbl.setText("— m/s")
            lbl.setStyleSheet(f"color: {COLORS['text_muted']}; font-size: 14px; font-weight: 700; font-family: monospace;")


# ─── Step Indicator ──────────────────────────────────────────────
class _StepIndicator(QLabel):
    """Small colored circle with step number."""

    _STYLES = {
        "idle":    (COLORS["bg_input"],  COLORS["text_muted"]),
        "pending": (COLORS["btn_amber"], COLORS["text"]),
        "running": (COLORS["accent"],    COLORS["text"]),
        "done":    (COLORS["btn_green"], COLORS["text"]),
        "error":   (COLORS["btn_red"],   COLORS["text"]),
    }

    def __init__(self, number: str, parent=None):
        super().__init__(number, parent)
        self.setFixedSize(26, 26)
        self.setAlignment(Qt.AlignCenter)
        self.set_state("idle")

    def set_state(self, state: str):
        bg, fg = self._STYLES.get(state, self._STYLES["idle"])
        self.setStyleSheet(
            f"background-color: {bg}; color: {fg}; border-radius: 13px; "
            f"font-weight: 700; font-size: 12px;"
        )


# ─── Console Widget ───────────────────────────────────────────────
class ConsoleWidget(QTextEdit):
    """
    Scrollable, color-coded console widget.
    Each line is tagged with a source name and rendered in its color.
    """

    MAX_LINES = 1000

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self._line_count = 0

    def append(self, tag_or_html: str, line: str = None):
        """
        If called with (tag, line): format as colored log entry.
        If called with just (html_str): pass directly to QTextEdit.append.
        This handles both our custom calls and any internal Qt calls.
        """
        if line is not None:
            # Our custom call: append(tag, line)
            tag = tag_or_html
            color = TAG_COLORS.get(tag, COLORS["text"])
            html = (
                f'<span style="color:{COLORS["text_muted"]}; font-size:11px;">'
                f'[{tag:8s}]</span> '
                f'<span style="color:{color}; font-size:12px;">'
                f'{_escape_html(str(line))}'
                f'</span>'
            )
            # Trim excess lines
            if self._line_count > self.MAX_LINES:
                from PyQt5.QtGui import QTextCursor
                cursor = self.textCursor()
                cursor.movePosition(QTextCursor.Start)
                cursor.select(QTextCursor.LineUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
                self._line_count -= 1
            super().append(html)
            self._line_count += 1
        else:
            # Plain QTextEdit.append call (html string)
            super().append(tag_or_html)
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())


class MultiConsoleWidget(QWidget):
    """
    4-panel console layout (2x2 grid):
      Top-left:     Pipeline & System Log
      Top-right:    Main Navigation Script
      Bottom-left:  Yellow Border Tracking
      Bottom-right: Seed Tracking  (full width — no ArUco split)
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.console_pipeline = ConsoleWidget()
        self.console_mission  = ConsoleWidget()
        self.console_yellow   = ConsoleWidget()
        self.console_seeds    = ConsoleWidget()

        grid = QGridLayout(self)
        grid.setSpacing(10)
        grid.setContentsMargins(0, 0, 0, 0)

        grid.addWidget(self._wrap_console("🛠️  Pipeline & System Log",    self.console_pipeline, TAG_COLORS["PIPELINE"]), 0, 0)
        grid.addWidget(self._wrap_console("🚀  Main Navigation Script",    self.console_mission,  TAG_COLORS["MISSION"]),  0, 1)
        grid.addWidget(self._wrap_console("🟡  Yellow Border Tracking",    self.console_yellow,   TAG_COLORS["YELLOW"]),   1, 0)
        grid.addWidget(self._wrap_console("🌱  Seed Tracking",             self.console_seeds,    TAG_COLORS["SEEDS"]),    1, 1)

    def _wrap_console(self, title: str, console: ConsoleWidget, color: str) -> QWidget:
        wrapper = QWidget()
        layout = QVBoxLayout(wrapper)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        header = QLabel(title)
        header.setStyleSheet(
            f"color: {color}; font-weight: bold; font-size: 11px; "
            f"padding: 4px 6px; background-color: rgba(255,255,255,0.03); "
            f"border-left: 3px solid {color}; border-radius: 2px;"
        )
        layout.addWidget(header)
        layout.addWidget(console)
        return wrapper

    def append(self, tag_or_html: str, line: str = None):
        if line is not None:
            t_upper = tag_or_html.upper()
            if t_upper in ("PIPELINE", "SYSTEM", "INFO", "ERROR"):
                self.console_pipeline.append(tag_or_html, line)
            elif t_upper == "MISSION":
                self.console_mission.append(tag_or_html, line)
            elif t_upper == "YELLOW":
                self.console_yellow.append(tag_or_html, line)
            elif t_upper == "SEEDS":
                self.console_seeds.append(tag_or_html, line)
            else:
                self.console_pipeline.append(tag_or_html, line)
        else:
            self.console_pipeline.append(tag_or_html)

    def clear(self):
        self.console_pipeline.clear()
        self.console_mission.clear()
        self.console_yellow.clear()
        self.console_seeds.clear()
