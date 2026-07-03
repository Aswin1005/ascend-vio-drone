"""
charging_tab.py  —  Tab 5: Base Station LiPo Charging Status Monitor
Listens to UDP broadcast messages on port 12345 from the charging microcontroller setup.
"""

import re
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit, QFrame
)
from styles import COLORS
from workers import ChargingWorker


class ChargingTab(QWidget):
    """Tab 5 — Base Station LiPo Charging Monitor."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._worker = None
        self._start_voltage = None
        self._target_voltage = 15.5  # Default target from charging.ino
        self._current_voltage = 0.0

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        # ── Status Bar / Header ──
        self._status_frame = QFrame()
        self._status_frame.setObjectName("status_frame")
        self._status_frame.setStyleSheet(
            f"QFrame#status_frame {{ "
            f"  background-color: {COLORS['bg_card']}; "
            f"  border: 1px solid {COLORS['border']}; "
            f"  border-radius: 8px; "
            f"}}"
        )
        status_layout = QHBoxLayout(self._status_frame)
        status_layout.setContentsMargins(16, 12, 16, 12)

        status_lbl_title = QLabel("🔌 CHARGING STATUS:")
        status_lbl_title.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 14px; font-weight: 600;")

        self._status_val = QLabel("OFFLINE (Waiting for mission...)")
        self._status_val.setStyleSheet(f"color: {COLORS['text_muted']}; font-size: 16px; font-weight: 700;")

        status_layout.addWidget(status_lbl_title)
        status_layout.addWidget(self._status_val)
        status_layout.addStretch()
        layout.addWidget(self._status_frame)

        # ── Metrics Cards Row ──
        cards_layout = QHBoxLayout()
        cards_layout.setSpacing(10)

        self._voltage_card = self._create_card(cards_layout, "🔋 Current Voltage", "— V", COLORS["blue"])
        self._start_card = self._create_card(cards_layout, "📍 Start Voltage", "— V", COLORS["text_dim"])
        self._target_card = self._create_card(cards_layout, "🎯 Target Voltage", "15.500 V", COLORS["amber"])
        self._rise_card = self._create_card(cards_layout, "📈 Voltage Rise", "— V", COLORS["green"])
        self._time_card = self._create_card(cards_layout, "⏱️ Elapsed Time", "—", COLORS["purple"])

        layout.addLayout(cards_layout)



        # ── Scrolling Log Console ──
        log_container = QWidget()
        log_container.setStyleSheet(
            f"background-color: {COLORS['bg_card']}; "
            f"border: 1px solid {COLORS['border']}; border-radius: 8px;"
        )
        log_layout = QVBoxLayout(log_container)
        log_layout.setContentsMargins(10, 10, 10, 10)
        log_layout.setSpacing(6)

        log_title = QLabel("Microcontroller Log Stream")
        log_title.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 12px; font-weight: 600;")
        log_layout.addWidget(log_title)

        self._console = QTextEdit()
        self._console.setReadOnly(True)
        self._console.setFontFamily("Consolas")
        self._console.setFontPointSize(10)
        self._console.setStyleSheet(
            f"background-color: {COLORS['bg_dark']}; "
            f"border: 1px solid {COLORS['border']}; "
            f"color: {COLORS['text']}; "
            f"border-radius: 4px;"
        )
        log_layout.addWidget(self._console)
        layout.addWidget(log_container, stretch=1)

    def _create_card(self, parent_layout: QHBoxLayout, title: str, init_val: str, color_hex: str) -> QLabel:
        card = QWidget()
        card.setStyleSheet(
            f"background-color: {COLORS['bg_card']}; "
            f"border: 1px solid {COLORS['border']}; border-radius: 8px;"
        )
        cl = QVBoxLayout(card)
        cl.setContentsMargins(12, 10, 12, 10)
        cl.setSpacing(4)

        title_lbl = QLabel(title)
        title_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px; font-weight: 600;")
        title_lbl.setAlignment(Qt.AlignCenter)

        val_lbl = QLabel(init_val)
        val_lbl.setStyleSheet(f"color: {color_hex}; font-size: 18px; font-weight: 700; font-family: monospace;")
        val_lbl.setAlignment(Qt.AlignCenter)

        cl.addWidget(title_lbl)
        cl.addWidget(val_lbl)
        parent_layout.addWidget(card)
        return val_lbl

    # ─── Public Control API ───
    def start_listening(self):
        """Starts the local UDP receiver worker."""
        if self._worker and self._worker.isRunning():
            return

        self._console.append("<span style='color:#8b949e;'>[SYSTEM] Starting background UDP listener...</span>")
        
        self._worker = ChargingWorker()
        self._worker.log_received.connect(self._append_log)
        self._worker.status_changed.connect(self._on_status_changed)
        self._worker.metrics_updated.connect(self._on_metrics_updated)
        self._worker.start()

    def stop_listening(self):
        """Stops the UDP receiver."""
        if self._worker:
            self._worker.stop()
            self._worker.wait(1000)
            self._worker = None

    # ─── Data Slots ───
    def _on_status_changed(self, status: str):
        self._status_val.setText(status.upper())
        if "EMERGENCY" in status.upper() or "FAULT" in status.upper():
            self._status_val.setStyleSheet(f"color: {COLORS['red']}; font-size: 16px; font-weight: 700;")
            self._status_frame.setStyleSheet(
                f"QFrame#status_frame {{ background-color: #3b1818; border: 1px solid {COLORS['red']}; border-radius: 8px; }}"
            )
        elif "CHARGING" in status.upper():
            self._status_val.setStyleSheet(f"color: {COLORS['blue']}; font-size: 16px; font-weight: 700;")
            self._status_frame.setStyleSheet(
                f"QFrame#status_frame {{ background-color: {COLORS['bg_card']}; border: 1px solid {COLORS['blue']}; border-radius: 8px; }}"
            )
        elif "COMPLETE" in status.upper() or "FINISHED" in status.upper():
            self._status_val.setStyleSheet(f"color: {COLORS['green']}; font-size: 16px; font-weight: 700;")
            self._status_frame.setStyleSheet(
                f"QFrame#status_frame {{ background-color: #1b3b22; border: 1px solid {COLORS['green']}; border-radius: 8px; }}"
            )
        else:
            self._status_val.setStyleSheet(f"color: {COLORS['amber']}; font-size: 16px; font-weight: 700;")
            self._status_frame.setStyleSheet(
                f"QFrame#status_frame {{ background-color: {COLORS['bg_card']}; border: 1px solid {COLORS['border']}; border-radius: 8px; }}"
            )

    def _on_metrics_updated(self, metrics: dict):
        # Current Voltage
        if "voltage" in metrics:
            self._current_voltage = metrics["voltage"]
            self._voltage_card.setText(f"{self._current_voltage:.3f} V")

        # Start Voltage
        if "start_voltage" in metrics:
            self._start_voltage = metrics["start_voltage"]
            self._start_card.setText(f"{self._start_voltage:.3f} V")

        # Target Voltage
        if "target_voltage" in metrics:
            self._target_voltage = metrics["target_voltage"]
            self._target_card.setText(f"{self._target_voltage:.3f} V")

        # Rise
        if "rise" in metrics:
            self._rise_card.setText(f"+{metrics['rise']:.3f} V")
        elif self._start_voltage is not None:
            rise = self._current_voltage - self._start_voltage
            self._rise_card.setText(f"+{rise:.3f} V")

        # Time
        if "time_str" in metrics:
            self._time_card.setText(metrics["time_str"])



    def _append_log(self, text: str):
        # Classify color and format html
        text_up = text.upper()
        
        # Determine color
        if "EMERGENCY" in text_up or "FATAL" in text_up or "FAULT" in text_up or "SSR CUT" in text_up:
            color = COLORS["red"]
            weight = "bold"
        elif "WARNING" in text_up or "ERROR" in text_up or "SAFETY" in text_up:
            color = COLORS["amber"]
            weight = "normal"
        elif "OK" in text_up or "CONFIRMED" in text_up or "STARTED" in text_up or "CONNECTED" in text_up:
            color = COLORS["green"]
            weight = "bold"
        elif "[LIVE]" in text_up:
            color = COLORS["blue"]
            weight = "normal"
        elif "[AVG]" in text_up:
            color = COLORS["purple"]
            weight = "normal"
        elif text.startswith("===") or text.startswith("---"):
            color = COLORS["text_muted"]
            weight = "normal"
        else:
            color = COLORS["text"]
            weight = "normal"

        html = f"<span style='color:{color}; font-weight:{weight};'>{self._escape_html(text)}</span>"
        self._console.append(html)

    def _escape_html(self, text: str) -> str:
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")

    def closeEvent(self, event):
        self.stop_listening()
        event.accept()
