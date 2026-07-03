"""
ascend_gcs.py  —  Ascend Ground Control Station
Main entry point for the PyQt5 GUI.

Run with:
    python3 ascend_gcs.py
"""

import sys
import os

# Add gcs directory to path so tabs can import siblings
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QIcon
from PyQt5.QtWidgets import (
    QApplication, QLabel, QMainWindow, QStatusBar,
    QTabWidget, QVBoxLayout, QWidget,
)

from ssh_manager import SSHManager
from styles import COLORS, STYLESHEET
from tabs.connection_tab import ConnectionTab
from tabs.mission_tab import MissionTab
from tabs.telemetry_tab import TelemetryTab
from tabs.seed_viewer_tab import SeedViewerTab
from tabs.charging_tab import ChargingTab


class AscendGCS(QMainWindow):
    """Main application window."""

    TITLE = "Ascend GCS — Drone Ground Control Station"
    MIN_WIDTH = 1100
    MIN_HEIGHT = 750

    def __init__(self):
        super().__init__()
        self.ssh_mgr = SSHManager()
        self._build_window()
        self._build_tabs()
        self._wire_signals()
        self._setup_status_bar()
        self._setup_connection_checker()

    # ─── Window Setup ─────────────────────────────────────────────
    def _build_window(self):
        self.setWindowTitle(self.TITLE)
        self.setMinimumSize(self.MIN_WIDTH, self.MIN_HEIGHT)
        self.resize(1280, 820)
        self.setStyleSheet(STYLESHEET)

        # Header bar
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        header = self._build_header()
        main_layout.addWidget(header)

        self._content_area = QWidget()
        content_layout = QVBoxLayout(self._content_area)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(0)
        main_layout.addWidget(self._content_area, stretch=1)

        self._main_layout = content_layout

    def _build_header(self) -> QWidget:
        header = QWidget()
        header.setFixedHeight(52)
        header.setStyleSheet(
            f"background-color: {COLORS['bg_panel']}; "
            f"border-bottom: 2px solid {COLORS['accent']};"
        )
        layout = QVBoxLayout(header)
        layout.setContentsMargins(18, 0, 18, 0)
        layout.setAlignment(Qt.AlignVCenter)

        title_row = QWidget()
        title_layout = __import__("PyQt5.QtWidgets", fromlist=["QHBoxLayout"]).QHBoxLayout(title_row)
        title_layout.setContentsMargins(0, 0, 0, 0)
        title_layout.setSpacing(16)

        # Logo / title
        logo_lbl = QLabel("🛸")
        logo_lbl.setStyleSheet("font-size: 24px;")
        title_lbl = QLabel("Ascend GCS")
        title_lbl.setStyleSheet(
            f"color: {COLORS['text']}; font-size: 20px; font-weight: 700; letter-spacing: 1px;"
        )
        subtitle_lbl = QLabel("Drone Ground Control Station  •  ISRO")
        subtitle_lbl.setStyleSheet(
            f"color: {COLORS['text_muted']}; font-size: 12px;"
        )

        self._conn_indicator = QLabel()
        self._conn_indicator.setStyleSheet(
            f"color: {COLORS['text_muted']}; font-size: 12px; font-weight: 500;"
        )
        self._update_conn_indicator(False)

        title_layout.addWidget(logo_lbl)
        title_layout.addWidget(title_lbl)
        title_layout.addWidget(subtitle_lbl)
        title_layout.addStretch()
        title_layout.addWidget(self._conn_indicator)

        layout.addWidget(title_row)
        return header

    def _update_conn_indicator(self, connected: bool):
        dot = "●"
        if connected:
            self._conn_indicator.setText(
                f'<span style="color:{COLORS["green"]}; font-size:14px;">{dot}</span>'
                f'  Jetson Connected'
            )
            self._conn_indicator.setTextFormat(Qt.RichText)
        else:
            self._conn_indicator.setText(
                f'<span style="color:{COLORS["text_muted"]}; font-size:14px;">{dot}</span>'
                f'  Not Connected'
            )
            self._conn_indicator.setTextFormat(Qt.RichText)

    # ─── Tabs ─────────────────────────────────────────────────────
    def _build_tabs(self):
        self._tabs = QTabWidget()
        self._tabs.setDocumentMode(True)

        self._conn_tab = ConnectionTab(self.ssh_mgr)
        self._mission_tab = MissionTab(self.ssh_mgr)
        self._telem_tab = TelemetryTab(self.ssh_mgr)
        self._seed_viewer_tab = SeedViewerTab(self.ssh_mgr)
        self._charging_tab = ChargingTab()

        self._tabs.addTab(self._conn_tab, "🔌  Connection & Setup")
        self._tabs.addTab(self._mission_tab, "🚀  Mission Control")
        self._tabs.addTab(self._telem_tab, "📡  Telemetry & Monitoring")
        self._tabs.addTab(self._seed_viewer_tab, "🌱  Seed Viewer")
        self._tabs.addTab(self._charging_tab, "⚡  Charging")

        self._main_layout.addWidget(self._tabs)

    # ─── Signal Wiring ────────────────────────────────────────────
    def _wire_signals(self):
        # Connection tab signals
        self._conn_tab.connected_signal.connect(self._on_connected)
        self._conn_tab.disconnected_signal.connect(self._on_disconnected)

        # Wire mission console output → telemetry tab
        self._mission_tab._console  # ensure it's built
        self._mission_tab.set_telem_tab(self._telem_tab)

        # Patch StreamWorker line output to also feed telemetry tab
        original_on_console_line = self._mission_tab._on_console_line

        def patched_on_console_line(tag: str, line: str):
            original_on_console_line(tag, line)
            self._telem_tab.pipe_console_line(tag, line)

        self._mission_tab._on_console_line = patched_on_console_line

        # Wire mission complete -> switch tab & load seeds
        self._mission_tab.transfer_completed.connect(self._on_transfer_completed)
        self._mission_tab.mission_completed_signal.connect(self._charging_tab.start_listening)

    def _on_transfer_completed(self, local_dir: str):
        # Switch tab to Seed Viewer (index 3)
        self._tabs.setCurrentIndex(3)
        self._seed_viewer_tab.load_seeds(local_dir)

    def _on_connected(self, ssh_mgr):
        self._update_conn_indicator(True)
        self._mission_tab.on_ssh_connected()
        self._telem_tab.on_ssh_connected()
        self._status_bar.showMessage(
            f"✅  Connected to {self.ssh_mgr.DEFAULT_HOST}  |  Ready", 5000
        )
        # Auto-switch to Mission tab
        self._tabs.setCurrentWidget(self._mission_tab)

    def _on_disconnected(self):
        self._update_conn_indicator(False)
        self._mission_tab.on_ssh_disconnected()
        self._telem_tab.on_ssh_disconnected()
        self._status_bar.showMessage("Disconnected from Jetson.", 3000)

    # ─── Status Bar ───────────────────────────────────────────────
    def _setup_status_bar(self):
        self._status_bar = QStatusBar()
        self._status_bar.setStyleSheet(
            f"background-color: {COLORS['bg_panel']}; "
            f"color: {COLORS['text_dim']}; font-size: 12px; "
            f"border-top: 1px solid {COLORS['border']};"
        )
        self.setStatusBar(self._status_bar)
        self._status_bar.showMessage("Ready.  Connect to Jetson to begin.")

    # ─── Connection alive check ───────────────────────────────────
    def _setup_connection_checker(self):
        self._alive_timer = QTimer(self)
        self._alive_timer.setInterval(5000)
        self._alive_timer.timeout.connect(self._check_alive)
        self._alive_timer.start()

    def _check_alive(self):
        was_alive = self.ssh_mgr.connected
        is_alive = self.ssh_mgr.is_alive()
        if was_alive and not is_alive:
            self.ssh_mgr.connected = False
            self._on_disconnected()
            self._status_bar.showMessage("⚠️  SSH connection lost!", 0)

    def closeEvent(self, event):
        """Clean up on close."""
        self._charging_tab.stop_listening()
        self.ssh_mgr.disconnect()
        event.accept()


# ─── Entry Point ─────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Ascend GCS")
    app.setOrganizationName("ISRO")

    # Set app-wide font
    font = QFont("Segoe UI", 10)
    font.setStyleHint(QFont.SansSerif)
    app.setFont(font)

    # Apply stylesheet
    app.setStyleSheet(STYLESHEET)

    window = AscendGCS()
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
