"""
connection_tab.py  —  Tab 1: SSH Connection + Seed Image Transfer
"""

import json
import os
import threading

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QFileDialog, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QListWidget, QListWidgetItem, QProgressBar, QPushButton,
    QScrollArea, QSizePolicy, QVBoxLayout, QWidget,
)
from styles import COLORS, status_dot_html

CONFIG_PATH = os.path.expanduser("~/.ascend_gcs_config.json")


def load_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}


def save_config(data: dict):
    try:
        existing = load_config()
        existing.update(data)
        with open(CONFIG_PATH, "w") as f:
            json.dump(existing, f, indent=2)
    except Exception:
        pass


# ─── SSH Connect Worker ──────────────────────────────────────────
class ConnectWorker(QThread):
    success = pyqtSignal()
    failed = pyqtSignal(str)

    def __init__(self, ssh_mgr, host, user, password, port):
        super().__init__()
        self.ssh_mgr = ssh_mgr
        self.host, self.user, self.password, self.port = host, user, password, port

    def run(self):
        ok, err = self.ssh_mgr.connect(self.host, self.user, self.password, self.port)
        if ok:
            self.success.emit()
        else:
            self.failed.emit(err)


# ─── SFTP Transfer Worker ────────────────────────────────────────
class TransferWorker(QThread):
    progress_msg = pyqtSignal(str)
    progress_val = pyqtSignal(int, int)   # done, total
    finished = pyqtSignal(bool, str)      # success, message

    REMOTE_SEED_DIR = "~/seed_tracker/seeds/ascend_seeds"

    def __init__(self, ssh_mgr, local_folders: list[str]):
        super().__init__()
        self.ssh_mgr = ssh_mgr
        self.local_folders = local_folders

    def run(self):
        try:
            self.progress_msg.emit("🧹 Clearing remote seed directory...")
            # Expand ~ on remote
            code, out, _ = self.ssh_mgr.exec(
                f"echo {self.REMOTE_SEED_DIR}", timeout=5
            )
            remote_dir = out.strip()
            # Ensure dir exists
            self.ssh_mgr.exec(f"mkdir -p {remote_dir}", timeout=5)
            # Clear contents
            self.ssh_mgr.sftp_clear_remote_dir(
                remote_dir,
                progress_cb=lambda m: self.progress_msg.emit(f"  {m}"),
            )
            self.progress_msg.emit("✅ Remote directory cleared.")

            self.progress_msg.emit("📤 Uploading seed folders...")
            self.ssh_mgr.sftp_upload_folders(
                self.local_folders,
                remote_dir,
                progress_cb=lambda m: self.progress_msg.emit(f"  {m}"),
                total_progress_cb=lambda d, t: self.progress_val.emit(d, t),
            )
            self.finished.emit(True, "✅ All seeds uploaded successfully!")
        except Exception as e:
            self.finished.emit(False, f"❌ Transfer failed: {e}")


# ─── Connection Tab ──────────────────────────────────────────────
class ConnectionTab(QWidget):
    """
    Tab 1 — SSH connection management + seed image transfer.
    Emits connected_signal(ssh_mgr) when SSH is established.
    """

    connected_signal = pyqtSignal(object)    # ssh_mgr
    disconnected_signal = pyqtSignal()

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self._connect_worker = None
        self._transfer_worker = None
        self._selected_folders: list[str] = []
        self._build_ui()
        self._load_saved_settings()

    # ─── UI Build ────────────────────────────────────────────────
    def _build_ui(self):
        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 16, 16, 16)
        outer.setSpacing(14)

        # ── SSH Connection Panel ──────────────────────────────────
        conn_group = QGroupBox("SSH Connection")
        conn_layout = QVBoxLayout(conn_group)
        conn_layout.setSpacing(10)

        # Credentials row
        cred_row = QHBoxLayout()
        cred_row.setSpacing(12)

        self._host_edit = QLineEdit("isro.local")
        self._host_edit.setPlaceholderText("Hostname / IP")
        self._host_edit.setMinimumWidth(160)

        self._user_edit = QLineEdit("isro")
        self._user_edit.setPlaceholderText("Username")
        self._user_edit.setMaximumWidth(120)

        self._pass_edit = QLineEdit("isro@123")
        self._pass_edit.setPlaceholderText("Password")
        self._pass_edit.setEchoMode(QLineEdit.Password)
        self._pass_edit.setMaximumWidth(140)

        self._port_edit = QLineEdit("22")
        self._port_edit.setPlaceholderText("Port")
        self._port_edit.setMaximumWidth(60)

        for label_text, widget in [
            ("Host:", self._host_edit),
            ("User:", self._user_edit),
            ("Pass:", self._pass_edit),
            ("Port:", self._port_edit),
        ]:
            lbl = QLabel(label_text)
            lbl.setObjectName("label_dim")
            cred_row.addWidget(lbl)
            cred_row.addWidget(widget)
        cred_row.addStretch()

        conn_layout.addLayout(cred_row)

        # Status + buttons row
        status_row = QHBoxLayout()
        status_row.setSpacing(12)

        self._status_label = QLabel()
        self._status_label.setTextFormat(Qt.RichText)
        self._set_status("disconnected")

        self._connect_btn = QPushButton("🔌 Connect")
        self._connect_btn.setObjectName("btn_green")
        self._connect_btn.setMinimumWidth(110)
        self._connect_btn.clicked.connect(self._on_connect)

        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.setObjectName("btn_flat")
        self._disconnect_btn.setMinimumWidth(110)
        self._disconnect_btn.setEnabled(False)
        self._disconnect_btn.clicked.connect(self._on_disconnect)

        status_row.addWidget(self._status_label)
        status_row.addStretch()
        status_row.addWidget(self._connect_btn)
        status_row.addWidget(self._disconnect_btn)

        conn_layout.addLayout(status_row)
        outer.addWidget(conn_group)

        # ── Seed Image Transfer Panel ─────────────────────────────
        seed_group = QGroupBox("Seed Image Transfer → ~/seed_tracker/seeds/ascend_seeds/")
        seed_layout = QVBoxLayout(seed_group)
        seed_layout.setSpacing(10)

        # Folder selection
        folder_row = QHBoxLayout()
        self._select_folders_btn = QPushButton("📁 Select Folders")
        self._select_folders_btn.setObjectName("btn_flat")
        self._select_folders_btn.setEnabled(False)
        self._select_folders_btn.clicked.connect(self._on_select_folders)
        self._clear_folders_btn = QPushButton("✕ Clear")
        self._clear_folders_btn.setObjectName("btn_flat")
        self._clear_folders_btn.setEnabled(False)
        self._clear_folders_btn.clicked.connect(self._on_clear_folders)
        folder_count_lbl = QLabel("Selected folders:")
        folder_count_lbl.setObjectName("label_dim")
        folder_row.addWidget(folder_count_lbl)
        folder_row.addStretch()
        folder_row.addWidget(self._select_folders_btn)
        folder_row.addWidget(self._clear_folders_btn)
        seed_layout.addLayout(folder_row)

        # Folder list
        self._folder_list = QListWidget()
        self._folder_list.setMaximumHeight(110)
        self._folder_list.setSelectionMode(QListWidget.NoSelection)
        seed_layout.addWidget(self._folder_list)

        # Info label
        info_lbl = QLabel(
            "⚠️  Sending will CLEAR ascend_seeds on the Jetson first, then upload all selected folders."
        )
        info_lbl.setWordWrap(True)
        info_lbl.setStyleSheet(f"color: {COLORS['amber']}; font-size: 12px;")
        seed_layout.addWidget(info_lbl)

        # Send button + progress
        send_row = QHBoxLayout()
        self._send_btn = QPushButton("🚀 Send Seeds to Jetson")
        self._send_btn.setObjectName("btn_green")
        self._send_btn.setEnabled(False)
        self._send_btn.clicked.connect(self._on_send_seeds)
        send_row.addWidget(self._send_btn)
        send_row.addStretch()
        seed_layout.addLayout(send_row)

        self._progress_bar = QProgressBar()
        self._progress_bar.setValue(0)
        self._progress_bar.setVisible(False)
        seed_layout.addWidget(self._progress_bar)

        self._transfer_log = _ConsoleWidget(max_lines=80)
        self._transfer_log.setMaximumHeight(150)
        seed_layout.addWidget(self._transfer_log)

        outer.addWidget(seed_group)
        outer.addStretch()

    # ─── Settings ────────────────────────────────────────────────
    def _load_saved_settings(self):
        cfg = load_config()
        if "ssh_host" in cfg:
            self._host_edit.setText(cfg["ssh_host"])
        if "ssh_user" in cfg:
            self._user_edit.setText(cfg["ssh_user"])
        if "ssh_pass" in cfg:
            self._pass_edit.setText(cfg["ssh_pass"])
        if "ssh_port" in cfg:
            self._port_edit.setText(str(cfg["ssh_port"]))

    def _save_settings(self):
        save_config({
            "ssh_host": self._host_edit.text().strip(),
            "ssh_user": self._user_edit.text().strip(),
            "ssh_pass": self._pass_edit.text(),
            "ssh_port": self._port_edit.text().strip(),
        })

    # ─── Connection ───────────────────────────────────────────────
    def _on_connect(self):
        host = self._host_edit.text().strip()
        user = self._user_edit.text().strip()
        password = self._pass_edit.text()
        try:
            port = int(self._port_edit.text().strip())
        except ValueError:
            port = 22

        self._set_status("connecting")
        self._connect_btn.setEnabled(False)

        self._connect_worker = ConnectWorker(self.ssh_mgr, host, user, password, port)
        self._connect_worker.success.connect(self._on_connected)
        self._connect_worker.failed.connect(self._on_connect_failed)
        self._connect_worker.start()

    def _on_connected(self):
        self._set_status("connected")
        self._connect_btn.setEnabled(False)
        self._disconnect_btn.setEnabled(True)
        self._select_folders_btn.setEnabled(True)
        self._clear_folders_btn.setEnabled(True)
        self._save_settings()
        self.connected_signal.emit(self.ssh_mgr)

    def _on_connect_failed(self, msg: str):
        self._set_status("error", msg)
        self._connect_btn.setEnabled(True)

    def _on_disconnect(self):
        self.ssh_mgr.disconnect()
        self._set_status("disconnected")
        self._connect_btn.setEnabled(True)
        self._disconnect_btn.setEnabled(False)
        self._select_folders_btn.setEnabled(False)
        self._clear_folders_btn.setEnabled(False)
        self._send_btn.setEnabled(False)
        self.disconnected_signal.emit()

    def _set_status(self, state: str, msg: str = ""):
        if state == "connected":
            self._status_label.setText(
                status_dot_html(COLORS["green"], "Connected to Jetson Nano")
            )
        elif state == "connecting":
            self._status_label.setText(
                status_dot_html(COLORS["amber"], "Connecting...")
            )
        elif state == "error":
            self._status_label.setText(
                status_dot_html(COLORS["red"], f"Connection failed: {msg}")
            )
        else:
            self._status_label.setText(
                status_dot_html(COLORS["text_muted"], "Not connected")
            )

    # ─── Seed Transfer ────────────────────────────────────────────
    def _on_select_folders(self):
        folders = []
        while True:
            folder = QFileDialog.getExistingDirectory(
                self, "Select Seed Folder", os.path.expanduser("~"),
                QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
            )
            if not folder:
                break
            if folder not in self._selected_folders:
                self._selected_folders.append(folder)
            # Ask if they want to add more
            from PyQt5.QtWidgets import QMessageBox
            reply = QMessageBox.question(
                self, "Add Another?",
                "Add another folder?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply == QMessageBox.No:
                break
        self._refresh_folder_list()

    def _on_clear_folders(self):
        self._selected_folders.clear()
        self._refresh_folder_list()

    def _refresh_folder_list(self):
        self._folder_list.clear()
        for f in self._selected_folders:
            item = QListWidgetItem(f"📂  {os.path.basename(f)}   ({f})")
            self._folder_list.addItem(item)
        self._send_btn.setEnabled(
            bool(self._selected_folders) and self.ssh_mgr.is_alive()
        )

    def _on_send_seeds(self):
        if not self._selected_folders:
            return
        self._send_btn.setEnabled(False)
        self._progress_bar.setValue(0)
        self._progress_bar.setVisible(True)
        self._transfer_log.clear()
        self._transfer_log.append_line("SYSTEM", "Starting seed transfer...")

        self._transfer_worker = TransferWorker(self.ssh_mgr, list(self._selected_folders))
        self._transfer_worker.progress_msg.connect(
            lambda m: self._transfer_log.append_line("SEEDS", m)
        )
        self._transfer_worker.progress_val.connect(self._on_transfer_progress)
        self._transfer_worker.finished.connect(self._on_transfer_done)
        self._transfer_worker.start()

    def _on_transfer_progress(self, done: int, total: int):
        if total > 0:
            pct = int(done * 100 / total)
            self._progress_bar.setMaximum(total)
            self._progress_bar.setValue(done)
            self._progress_bar.setFormat(f"{done}/{total} files ({pct}%)")

    def _on_transfer_done(self, success: bool, message: str):
        tag = "INFO" if success else "ERROR"
        self._transfer_log.append_line(tag, message)
        self._progress_bar.setVisible(False)
        self._send_btn.setEnabled(True)


# ─── Simple console widget ───────────────────────────────────────
class _ConsoleWidget(QWidget):
    """Minimal scrollable console used inside the connection tab."""

    TAG_COLORS = {
        "SEEDS":  COLORS["blue"],
        "INFO":   COLORS["green"],
        "ERROR":  COLORS["red"],
        "SYSTEM": COLORS["text_dim"],
    }

    def __init__(self, max_lines=200, parent=None):
        super().__init__(parent)
        from PyQt5.QtWidgets import QTextEdit
        self._te = __import__("PyQt5.QtWidgets", fromlist=["QTextEdit"]).QTextEdit()
        self._te.setReadOnly(True)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._te)
        self._max_lines = max_lines

    def setMaximumHeight(self, h):
        self._te.setMaximumHeight(h)

    def clear(self):
        self._te.clear()

    def append_line(self, tag: str, line: str):
        color = self.TAG_COLORS.get(tag, COLORS["text"])
        html = (
            f'<span style="color:{color}; font-family:monospace; font-size:12px;">'
            f'[{tag}] {_escape_html(line)}'
            f'</span>'
        )
        self._te.append(html)
        sb = self._te.verticalScrollBar()
        sb.setValue(sb.maximum())


def _escape_html(text: str) -> str:
    return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
