"""
styles.py  —  Dark theme stylesheet and color palette for Ascend GCS
"""

# ─── Color Palette ──────────────────────────────────────────────
COLORS = {
    "bg_dark":      "#0d1117",
    "bg_panel":     "#161b22",
    "bg_card":      "#1c2128",
    "bg_input":     "#21262d",
    "border":       "#30363d",
    "border_light": "#444c56",
    "text":         "#e6edf3",
    "text_dim":     "#8b949e",
    "text_muted":   "#6e7681",

    "green":        "#3fb950",
    "green_dim":    "#2ea043",
    "red":          "#f85149",
    "amber":        "#e3b341",
    "blue":         "#58a6ff",
    "cyan":         "#39d0d8",
    "purple":       "#bc8cff",
    "orange":       "#f0883e",
    "magenta":      "#ff7b72",

    "accent":       "#1f6feb",
    "accent_hover": "#388bfd",
    "accent_press": "#1158c7",

    "btn_green":    "#238636",
    "btn_green_h":  "#2ea043",
    "btn_red":      "#b62324",
    "btn_red_h":    "#da3633",
    "btn_amber":    "#9e6a03",
    "btn_amber_h":  "#bb8009",
}

# ─── Console tag colors (HTML) ───────────────────────────────────
TAG_COLORS = {
    "PIPELINE": "#39d0d8",   # cyan
    "MISSION":  "#e3b341",   # amber/yellow
    "YELLOW":   "#f0883e",   # orange
    "ARUCO":    "#bc8cff",   # purple
    "SEEDS":    "#58a6ff",   # blue
    "SYSTEM":   "#8b949e",   # dim
    "ERROR":    "#f85149",   # red
    "INFO":     "#3fb950",   # green
}

# ─── Main stylesheet ─────────────────────────────────────────────
STYLESHEET = f"""
/* ── Global ── */
QMainWindow, QWidget, QDialog {{
    background-color: {COLORS["bg_dark"]};
    color: {COLORS["text"]};
    font-family: "Segoe UI", "Inter", "Ubuntu", sans-serif;
    font-size: 13px;
}}

/* ── Tab Bar ── */
QTabWidget::pane {{
    border: 1px solid {COLORS["border"]};
    background-color: {COLORS["bg_panel"]};
    border-radius: 6px;
}}
QTabBar::tab {{
    background-color: {COLORS["bg_dark"]};
    color: {COLORS["text_dim"]};
    border: 1px solid {COLORS["border"]};
    border-bottom: none;
    padding: 10px 22px;
    margin-right: 3px;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    font-size: 13px;
    font-weight: 500;
}}
QTabBar::tab:selected {{
    background-color: {COLORS["bg_panel"]};
    color: {COLORS["text"]};
    border-color: {COLORS["accent"]};
    border-bottom: 2px solid {COLORS["accent"]};
}}
QTabBar::tab:hover:!selected {{
    background-color: {COLORS["bg_card"]};
    color: {COLORS["text"]};
}}

/* ── GroupBox ── */
QGroupBox {{
    background-color: {COLORS["bg_card"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 8px;
    margin-top: 14px;
    padding: 10px 12px 10px 12px;
    font-weight: 600;
    color: {COLORS["text"]};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    top: 2px;
    color: {COLORS["blue"]};
    background-color: {COLORS["bg_card"]};
    padding: 0 6px;
    font-size: 12px;
    font-weight: 700;
    letter-spacing: 0.5px;
}}

/* ── Buttons ── */
QPushButton {{
    background-color: {COLORS["accent"]};
    color: {COLORS["text"]};
    border: none;
    border-radius: 6px;
    padding: 8px 18px;
    font-weight: 600;
    font-size: 13px;
}}
QPushButton:hover {{
    background-color: {COLORS["accent_hover"]};
}}
QPushButton:pressed {{
    background-color: {COLORS["accent_press"]};
}}
QPushButton:disabled {{
    background-color: {COLORS["bg_input"]};
    color: {COLORS["text_muted"]};
}}

QPushButton#btn_green {{
    background-color: {COLORS["btn_green"]};
}}
QPushButton#btn_green:hover {{
    background-color: {COLORS["btn_green_h"]};
}}

QPushButton#btn_red {{
    background-color: {COLORS["btn_red"]};
}}
QPushButton#btn_red:hover {{
    background-color: {COLORS["btn_red_h"]};
}}

QPushButton#btn_amber {{
    background-color: {COLORS["btn_amber"]};
}}
QPushButton#btn_amber:hover {{
    background-color: {COLORS["btn_amber_h"]};
}}

QPushButton#btn_flat {{
    background-color: {COLORS["bg_input"]};
    color: {COLORS["text_dim"]};
    border: 1px solid {COLORS["border"]};
}}
QPushButton#btn_flat:hover {{
    background-color: {COLORS["bg_card"]};
    color: {COLORS["text"]};
    border-color: {COLORS["border_light"]};
}}

/* ── Line Edit / Combo / Spin ── */
QLineEdit, QComboBox, QSpinBox {{
    background-color: {COLORS["bg_input"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 5px;
    color: {COLORS["text"]};
    padding: 6px 10px;
    font-size: 13px;
    selection-background-color: {COLORS["accent"]};
}}
QLineEdit:focus, QComboBox:focus {{
    border-color: {COLORS["accent"]};
}}
QComboBox::drop-down {{
    border: none;
    width: 24px;
}}
QComboBox::down-arrow {{
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 5px solid {COLORS["text_dim"]};
    margin-right: 6px;
}}
QComboBox QAbstractItemView {{
    background-color: {COLORS["bg_input"]};
    border: 1px solid {COLORS["border"]};
    color: {COLORS["text"]};
    selection-background-color: {COLORS["accent"]};
    outline: none;
}}

/* ── Text Edit (console) ── */
QTextEdit, QPlainTextEdit {{
    background-color: {COLORS["bg_dark"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 6px;
    color: {COLORS["text"]};
    font-family: "JetBrains Mono", "Fira Code", "Courier New", monospace;
    font-size: 12px;
    padding: 6px;
    selection-background-color: {COLORS["accent"]};
}}

/* ── List Widget ── */
QListWidget {{
    background-color: {COLORS["bg_input"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 5px;
    color: {COLORS["text"]};
    font-size: 12px;
}}
QListWidget::item {{
    padding: 4px 8px;
}}
QListWidget::item:selected {{
    background-color: {COLORS["accent"]};
    color: white;
}}
QListWidget::item:hover {{
    background-color: {COLORS["bg_card"]};
}}

/* ── Labels ── */
QLabel {{
    color: {COLORS["text"]};
}}
QLabel#label_dim {{
    color: {COLORS["text_dim"]};
    font-size: 12px;
}}
QLabel#label_green {{
    color: {COLORS["green"]};
    font-weight: 700;
}}
QLabel#label_red {{
    color: {COLORS["red"]};
    font-weight: 700;
}}
QLabel#label_amber {{
    color: {COLORS["amber"]};
    font-weight: 700;
}}
QLabel#label_blue {{
    color: {COLORS["blue"]};
    font-weight: 700;
}}

/* ── Progress Bar ── */
QProgressBar {{
    background-color: {COLORS["bg_input"]};
    border: 1px solid {COLORS["border"]};
    border-radius: 4px;
    color: {COLORS["text"]};
    text-align: center;
    font-size: 12px;
    height: 18px;
}}
QProgressBar::chunk {{
    background-color: {COLORS["accent"]};
    border-radius: 4px;
}}

/* ── Scrollbar ── */
QScrollBar:vertical {{
    background: {COLORS["bg_dark"]};
    width: 8px;
    border-radius: 4px;
}}
QScrollBar::handle:vertical {{
    background: {COLORS["border_light"]};
    border-radius: 4px;
    min-height: 20px;
}}
QScrollBar::handle:vertical:hover {{
    background: {COLORS["text_muted"]};
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}
QScrollBar:horizontal {{
    background: {COLORS["bg_dark"]};
    height: 8px;
    border-radius: 4px;
}}
QScrollBar::handle:horizontal {{
    background: {COLORS["border_light"]};
    border-radius: 4px;
    min-width: 20px;
}}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
    width: 0;
}}

/* ── Splitter ── */
QSplitter::handle {{
    background-color: {COLORS["border"]};
}}

/* ── Tool tip ── */
QToolTip {{
    background-color: {COLORS["bg_card"]};
    border: 1px solid {COLORS["border"]};
    color: {COLORS["text"]};
    padding: 4px 8px;
    border-radius: 4px;
}}

/* ── Status bar ── */
QStatusBar {{
    background-color: {COLORS["bg_dark"]};
    color: {COLORS["text_dim"]};
    border-top: 1px solid {COLORS["border"]};
    font-size: 12px;
}}
"""


def status_dot_html(color: str, label: str) -> str:
    """Return HTML for a colored status dot + label."""
    return (
        f'<span style="color:{color}; font-size:16px;">●</span>'
        f'<span style="color:{COLORS["text"]}; margin-left:6px;">{label}</span>'
    )
