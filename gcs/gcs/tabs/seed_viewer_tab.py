"""
seed_viewer_tab.py  —  GCS tab for displaying captured seeds and verifying them
"""

import os
import csv
import shutil
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSplitter,
    QListWidget, QListWidgetItem, QGroupBox, QScrollArea, QFrame,
    QProgressBar, QSizePolicy
)

from styles import COLORS
from workers import BatchVerifyWorker


class SeedItemWidget(QWidget):
    """Custom widget for seed list items to render image preview and metadata."""
    def __init__(self, img_path: str, filename: str, coords: str, is_verified: bool = False, feature_tag: str = "", parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(10)

        # Thumbnail
        self.thumb_lbl = QLabel()
        self.thumb_lbl.setFixedSize(90, 68)
        self.thumb_lbl.setStyleSheet(
            f"border: 1px solid {COLORS['border']}; border-radius: 4px; background-color: {COLORS['bg_dark']};"
        )
        
        pixmap = QPixmap(img_path)
        if not pixmap.isNull():
            self.thumb_lbl.setPixmap(pixmap.scaled(90, 68, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        layout.addWidget(self.thumb_lbl)

        # Info container
        info_layout = QVBoxLayout()
        info_layout.setSpacing(3)

        name_layout = QHBoxLayout()
        self.name_lbl = QLabel(filename)
        self.name_lbl.setStyleSheet("font-weight: bold; font-size: 13px; color: #e6edf3;")
        name_layout.addWidget(self.name_lbl)

        # Verified badge
        if is_verified:
            self.badge_lbl = QLabel(f"  {feature_tag} ✅  ")
            self.badge_lbl.setStyleSheet(
                f"background-color: {COLORS['btn_green']}; color: white; "
                "font-size: 10px; font-weight: bold; border-radius: 3px; padding: 2px;"
            )
            self.badge_lbl.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            name_layout.addWidget(self.badge_lbl)
        name_layout.addStretch()

        self.coords_lbl = QLabel(coords)
        self.coords_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-size: 11px; font-family: monospace;")

        info_layout.addLayout(name_layout)
        info_layout.addWidget(self.coords_lbl)
        layout.addLayout(info_layout)
        layout.addStretch()


class SeedViewerTab(QWidget):
    """Tab to view and verify seeds using batch_verify.py."""

    def __init__(self, ssh_mgr, parent=None):
        super().__init__(parent)
        self.ssh_mgr = ssh_mgr
        self.data_dir = None
        self.raw_seeds = []      # list of dict: {filename, path, coords}
        self.verified_seeds = {} # dict of filename -> {feature, seed, coords, ssim, orb, img_path}
        self.filter_verified = False

        self._build_ui()

    def _build_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(10)

        # ─── Top Control Bar ──────────────────────────────────────────
        top_bar = QHBoxLayout()
        top_bar.setSpacing(10)

        self.verify_btn = QPushButton("🔍  Verify Seed Images")
        self.verify_btn.setObjectName("btn_green")
        self.verify_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLORS['btn_green']};
                color: white;
                font-weight: bold;
                border-radius: 4px;
                padding: 8px 16px;
            }}
            QPushButton:hover {{
                background-color: {COLORS['btn_green_h']};
            }}
            QPushButton:disabled {{
                background-color: {COLORS['border']};
                color: {COLORS['text_muted']};
            }}
        """)
        self.verify_btn.setEnabled(False)
        self.verify_btn.clicked.connect(self._run_verification)
        top_bar.addWidget(self.verify_btn)

        # Filter buttons
        self.show_all_btn = QPushButton("All Captured")
        self.show_all_btn.setCheckable(True)
        self.show_all_btn.setChecked(True)
        self.show_all_btn.setStyleSheet(self._filter_btn_style(True))
        self.show_all_btn.clicked.connect(lambda: self._set_filter(False))

        self.show_verified_btn = QPushButton("Verified Only")
        self.show_verified_btn.setCheckable(True)
        self.show_verified_btn.setStyleSheet(self._filter_btn_style(False))
        self.show_verified_btn.clicked.connect(lambda: self._set_filter(True))

        top_bar.addWidget(self.show_all_btn)
        top_bar.addWidget(self.show_verified_btn)

        # Status label
        self.status_lbl = QLabel("No flight data loaded. Capture seeds during flight.")
        self.status_lbl.setStyleSheet(f"color: {COLORS['text_dim']}; font-style: italic; margin-left: 10px;")
        top_bar.addWidget(self.status_lbl)
        top_bar.addStretch()

        main_layout.addLayout(top_bar)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid {COLORS['border']};
                border-radius: 4px;
                text-align: center;
                background-color: {COLORS['bg_dark']};
                height: 16px;
            }}
            QProgressBar::chunk {{
                background-color: {COLORS['green']};
                border-radius: 3px;
            }}
        """)
        main_layout.addWidget(self.progress_bar)

        # ─── Splitter Layout ──────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.setStyleSheet(f"QSplitter::handle {{ background-color: {COLORS['border']}; width: 1px; }}")

        # Left panel: Image List
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        self.list_group = QGroupBox("Captured Seed List")
        list_group_layout = QVBoxLayout(self.list_group)
        list_group_layout.setContentsMargins(4, 8, 4, 4)

        self.list_widget = QListWidget()
        self.list_widget.setStyleSheet(f"""
            QListWidget {{
                background-color: {COLORS['bg_panel']};
                border: 1px solid {COLORS['border']};
                border-radius: 4px;
            }}
            QListWidget::item {{
                border-bottom: 1px solid {COLORS['border']};
            }}
            QListWidget::item:selected {{
                background-color: {COLORS['bg_card']};
                border-left: 3px solid {COLORS['accent']};
            }}
            QListWidget::item:hover:!selected {{
                background-color: {COLORS['bg_input']};
            }}
        """)
        self.list_widget.itemSelectionChanged.connect(self._on_item_selected)
        list_group_layout.addWidget(self.list_widget)
        left_layout.addWidget(self.list_group)

        # Right panel: Details Panel
        right_scroll = QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setFrameShape(QFrame.NoFrame)
        right_scroll.setStyleSheet(f"background-color: {COLORS['bg_dark']};")

        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 0, 10, 0)
        right_layout.setSpacing(12)

        # Image Viewer
        self.image_group = QGroupBox("Seed Image Detail")
        img_group_layout = QVBoxLayout(self.image_group)
        img_group_layout.setContentsMargins(6, 12, 6, 6)

        self.large_img_lbl = QLabel("Select an image from the list to view detail")
        self.large_img_lbl.setAlignment(Qt.AlignCenter)
        self.large_img_lbl.setMinimumSize(480, 270)
        self.large_img_lbl.setStyleSheet(
            f"border: 1px solid {COLORS['border']}; border-radius: 4px; "
            f"background-color: {COLORS['bg_dark']}; color: {COLORS['text_dim']};"
        )
        img_group_layout.addWidget(self.large_img_lbl)
        right_layout.addWidget(self.image_group)

        # Metadata Box
        self.meta_group = QGroupBox("Metadata & Metrics")
        meta_layout = QVBoxLayout(self.meta_group)
        meta_layout.setSpacing(8)

        self.lbl_filename = QLabel("Filename: —")
        self.lbl_coords = QLabel("Coordinates: —")
        self.lbl_status = QLabel("Verification: Not verified")
        self.lbl_matched = QLabel("Matched Seed: —")
        self.lbl_ssim = QLabel("SSIM Score: —")
        self.lbl_orb = QLabel("ORB Score: —")

        for lbl in [self.lbl_filename, self.lbl_coords, self.lbl_status, self.lbl_matched, self.lbl_ssim, self.lbl_orb]:
            lbl.setStyleSheet(f"font-size: 13px; color: {COLORS['text']};")
            meta_layout.addWidget(lbl)

        right_layout.addWidget(self.meta_group)
        right_layout.addStretch()

        right_scroll.setWidget(right_widget)

        splitter.addWidget(left_widget)
        splitter.addWidget(right_scroll)
        splitter.setSizes([450, 650])

        main_layout.addWidget(splitter)

    def _filter_btn_style(self, checked: bool) -> str:
        bg = COLORS['bg_card'] if not checked else COLORS['accent']
        fg = COLORS['text_dim'] if not checked else 'white'
        border = COLORS['border'] if not checked else COLORS['accent']
        return f"""
            QPushButton {{
                background-color: {bg};
                color: {fg};
                border: 1px solid {border};
                border-radius: 4px;
                padding: 6px 12px;
                font-weight: 500;
            }}
            QPushButton:hover {{
                background-color: {COLORS['bg_input'] if not checked else COLORS['accent_hover']};
            }}
        """

    def _set_filter(self, verified_only: bool):
        self.filter_verified = verified_only
        self.show_all_btn.setChecked(not verified_only)
        self.show_all_btn.setStyleSheet(self._filter_btn_style(not verified_only))
        self.show_verified_btn.setChecked(verified_only)
        self.show_verified_btn.setStyleSheet(self._filter_btn_style(verified_only))
        self._refresh_list()

    def load_seeds(self, data_dir: str):
        """Called automatically after seed images folder transfer completes."""
        self.data_dir = data_dir
        self.raw_seeds.clear()
        self.verified_seeds.clear()
        self.status_lbl.setText("Flight data loaded. Verification pending.")
        self.status_lbl.setStyleSheet(f"color: {COLORS['amber']}; font-weight: bold;")
        self.verify_btn.setEnabled(True)

        # Parse positions.csv to map coordinates to images
        csv_path = os.path.join(data_dir, "positions.csv")
        coords_map = {}
        if os.path.exists(csv_path):
            try:
                with open(csv_path, 'r') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        # Normalize keys (handles frame_file, filename, pos_x, pos_x_east_m)
                        fname = row.get('frame_file') or row.get('filename')
                        x = row.get('pos_x_east_m') or row.get('pos_x') or row.get('x') or '0.0'
                        y = row.get('pos_y_north_m') or row.get('pos_y') or row.get('y') or '0.0'
                        z = row.get('pos_z_up_m') or row.get('pos_z') or row.get('z') or '0.0'
                        if fname:
                            coords_map[fname] = f"X: {float(x):.3f}, Y: {float(y):.3f}, Z: {float(z):.3f}"
            except Exception as e:
                print(f"Error parsing positions.csv: {e}")

        # Scan for images in data_dir
        for f in os.listdir(data_dir):
            if f.lower().endswith(('.png', '.jpg', '.jpeg')) and not f.endswith('_boxed.jpg'):
                path = os.path.join(data_dir, f)
                coords = coords_map.get(f, "X: 0.000, Y: 0.000, Z: 0.000")
                self.raw_seeds.append({
                    'filename': f,
                    'path': path,
                    'coords': coords
                })

        # Sort raw seeds by name/number
        self.raw_seeds.sort(key=lambda s: s['filename'])
        self._refresh_list()

    def _refresh_list(self):
        self.list_widget.clear()
        
        display_list = []
        if self.filter_verified:
            # Only show raw seeds that are in verified_seeds
            for s in self.raw_seeds:
                if s['filename'] in self.verified_seeds:
                    display_list.append(s)
        else:
            display_list = self.raw_seeds

        for s in display_list:
            is_v = s['filename'] in self.verified_seeds
            feature_tag = self.verified_seeds[s['filename']]['feature'] if is_v else ""
            
            # Use original transferred image for visualization
            widget = SeedItemWidget(
                s['path'],
                s['filename'],
                s['coords'],
                is_verified=is_v,
                feature_tag=feature_tag
            )

            item = QListWidgetItem(self.list_widget)
            item.setSizeHint(widget.sizeHint())
            item.setData(Qt.UserRole, s)
            self.list_widget.addItem(item)
            self.list_widget.setItemWidget(item, widget)

        self.list_group.setTitle(f"Captured Seed List ({self.list_widget.count()} frames)")

        # Clear preview
        self.large_img_lbl.setPixmap(QPixmap())
        self.large_img_lbl.setText("Select an image from the list to view detail")
        self._clear_metadata()

    def _clear_metadata(self):
        self.lbl_filename.setText("Filename: —")
        self.lbl_coords.setText("Coordinates: —")
        self.lbl_status.setText("Verification: —")
        self.lbl_matched.setText("Matched Seed: —")
        self.lbl_ssim.setText("SSIM Score: —")
        self.lbl_orb.setText("ORB Score: —")

    def _on_item_selected(self):
        selected = self.list_widget.selectedItems()
        if not selected:
            return
        
        item_data = selected[0].data(Qt.UserRole)
        filename = item_data['filename']
        coords = item_data['coords']
        
        # Display image
        is_verified = filename in self.verified_seeds
        if is_verified:
            img_path = self.verified_seeds[filename]['img_path']
        else:
            img_path = item_data['path']

        pixmap = QPixmap(img_path)
        if not pixmap.isNull():
            # Scale down large HD images to fit nicely in viewer
            self.large_img_lbl.setPixmap(pixmap.scaled(self.large_img_lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        else:
            self.large_img_lbl.setText(f"Error loading image:\n{img_path}")

        # Load metadata
        self.lbl_filename.setText(f"Filename: {filename}")
        self.lbl_coords.setText(f"Coordinates: {coords}")
        
        if is_verified:
            m = self.verified_seeds[filename]
            self.lbl_status.setText("Verification: VERIFIED ✅")
            self.lbl_status.setStyleSheet(f"font-weight: bold; color: {COLORS['green']};")
            self.lbl_matched.setText(f"Matched Seed: {m['seed']} ({m['feature']})")
            self.lbl_ssim.setText(f"SSIM Score: {m['ssim']:.3f}")
            self.lbl_orb.setText(f"ORB Score: {m['orb']}")
        else:
            self.lbl_status.setText("Verification: Not verified")
            self.lbl_status.setStyleSheet(f"color: {COLORS['text_dim']};")
            self.lbl_matched.setText("Matched Seed: —")
            self.lbl_ssim.setText("SSIM Score: —")
            self.lbl_orb.setText("ORB Score: —")

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Rescale current selected image on tab resize
        self._on_item_selected()

    # ─── Verification Handler ──────────────────────────────────────
    def _run_verification(self):
        if not self.data_dir:
            return
        
        self.verify_btn.setEnabled(False)
        self.status_lbl.setText("Verifying captured seeds...")
        self.status_lbl.setStyleSheet(f"color: {COLORS['amber']}; font-weight: bold;")
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(True)

        self.worker = BatchVerifyWorker(self.data_dir)
        self.worker.progress.connect(lambda msg: self.status_lbl.setText(msg))
        self.worker.finished.connect(self._on_verification_finished)
        self.worker.start()

    def _on_verification_finished(self, success: bool, output: str):
        self.progress_bar.setVisible(False)
        self.verify_btn.setEnabled(True)

        if not success:
            self.status_lbl.setText(f"Verification failed: {output}")
            self.status_lbl.setStyleSheet(f"color: {COLORS['red']}; font-weight: bold;")
            return

        # Parse match_logs/verification_report.txt
        gcs_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        report_path = os.path.join(gcs_dir, "match_logs", "verification_report.txt")
        
        self.verified_seeds.clear()
        if os.path.exists(report_path):
            try:
                with open(report_path, 'r') as f:
                    content = f.read()

                # Split by 30 dashes
                blocks = content.split("-" * 30 + "\n")
                for block in blocks:
                    if "Feature:" not in block:
                        continue
                    
                    match_data = {}
                    for line in block.splitlines():
                        if ":" not in line:
                            continue
                        key, val = line.split(":", 1)
                        key = key.strip().lower()
                        val = val.strip()
                        if key == "feature":
                            match_data["feature"] = val
                        elif key == "best frame":
                            match_data["image"] = val
                        elif key == "matched seed":
                            match_data["seed"] = val
                        elif key == "coordinates":
                            match_data["coords"] = val.replace(" (wrt base station)", "")
                        elif key == "match scores":
                            # Match Scores: SSIM = 0.850, ORB = 12
                            parts = val.split(",")
                            for p in parts:
                                if "ssim" in p.lower():
                                    match_data["ssim"] = float(p.split("=")[1].strip())
                                elif "orb" in p.lower():
                                    match_data["orb"] = int(p.split("=")[1].strip())
                    
                    if "image" in match_data:
                        # Reconstruct verification image path
                        save_name = f"{match_data['feature']}_{match_data['image']}"
                        match_data["img_path"] = os.path.join(gcs_dir, "match_logs", save_name)
                        
                        # Save mapped by the original filename
                        self.verified_seeds[match_data['image']] = match_data
            except Exception as e:
                print(f"Error parsing verification report: {e}")

        # Update labels and refresh UI
        num_v = len(self.verified_seeds)
        self.status_lbl.setText(f"Verification complete. Found {num_v} matching seeds!")
        self.status_lbl.setStyleSheet(f"color: {COLORS['green']}; font-weight: bold;")
        
        # Select first tab, switch filter, refresh list
        self._refresh_list()
