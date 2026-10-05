#!/usr/bin/env python3
"""Emit Qt stylesheets from resources/tokens.json. No network."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOKENS = ROOT / "resources" / "tokens.json"
OUT = ROOT / "resources" / "theme"

TEMPLATE = """
* {{
  font-family: {font_ui};
  color: {text_1};
  font-size: 12px;
}}
QMainWindow, QDialog {{
  background: {bg_app};
}}
QMenuBar, QStatusBar, #chrome {{
  background: {bg_chrome};
  color: {text_2};
  border: none;
}}
QStatusBar {{
  border-top: 1px solid {line_soft};
  font-size: 11px;
}}
QMenuBar::item:selected, QMenu::item:selected {{
  background: {bg_active};
  color: {text_1};
}}
QMenu {{
  background: {bg_panel};
  border: 1px solid {line};
  padding: 4px;
}}
QDockWidget {{
  color: {text_2};
  titlebar-close-icon: none;
}}
QDockWidget::title {{
  background: {bg_panel};
  padding: 6px 8px;
  border-bottom: 1px solid {line_soft};
  text-align: left;
}}
QTreeWidget, QTableWidget, QListWidget, QPlainTextEdit, QTextEdit {{
  background: {bg_panel};
  alternate-background-color: {bg_raised};
  border: none;
  selection-background-color: {bg_active};
  selection-color: {text_1};
}}
QHeaderView::section {{
  background: {bg_chrome};
  color: {text_3};
  border: none;
  border-bottom: 1px solid {line_soft};
  padding: 4px 6px;
}}
QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {{
  background: {bg_input};
  border: 1px solid {line};
  border-radius: 6px;
  padding: 3px 6px;
  min-height: 20px;
  selection-background-color: {accent};
  selection-color: {on_accent};
}}
QPushButton {{
  background: {bg_raised};
  border: 1px solid {line_strong};
  border-radius: 7px;
  padding: 4px 10px;
  min-height: 22px;
}}
QPushButton:hover {{
  background: {bg_hover};
}}
QPushButton:checked, QToolButton:checked {{
  background: {bg_active};
  border: 1px solid {accent};
  color: {text_1};
}}
QToolButton {{
  background: transparent;
  border: none;
  border-radius: 7px;
  color: {text_2};
  padding: 4px;
}}
QToolButton:hover {{
  background: {bg_hover};
  color: {text_1};
}}
QScrollBar:vertical {{
  background: {bg_panel};
  width: 10px;
}}
QScrollBar::handle:vertical {{
  background: {line_strong};
  border-radius: 4px;
  min-height: 24px;
}}
#modeButton {{
  color: {text_2};
  background: transparent;
  border-radius: 7px;
  padding: 4px 10px;
}}
#modeButton:checked {{
  background: {bg_active};
  color: {text_1};
  border: 1px solid {accent};
}}
#commandHint, #commandPalette, #overlayGlass {{
  background: {bg_panel};
  border: 1px solid {line};
  border-radius: 12px;
}}
#intentBar {{
  background: {bg_panel};
  border: 1px solid {ai_start};
  border-radius: 12px;
}}
#aiRibbon {{
  background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {ai_start}, stop:1 {ai_end});
  border: none;
  min-height: 4px;
  max-height: 4px;
  border-radius: 2px;
}}
#accentButton {{
  background: {accent};
  color: {on_accent};
  border: none;
  font-weight: 600;
}}
#accentButton:hover {{
  background: {accent};
  color: {on_accent};
  border: 1px solid {accent_hover};
}}
#accentButton:pressed {{
  background: {accent_pressed};
  color: {on_accent};
  border: none;
}}
"""


def flat(theme: dict) -> dict:
    out = {}
    for key, node in theme.items():
        value = node["$value"] if isinstance(node, dict) else node
        out[key.replace(".", "_")] = value
    return out


def main() -> None:
    data = json.loads(TOKENS.read_text(encoding="utf-8"))
    OUT.mkdir(parents=True, exist_ok=True)
    font = data["font"]["ui"]
    for name in ("dark", "light"):
        tokens = flat(data["semantic"][name])
        tokens["font_ui"] = font
        text = TEMPLATE.format(**tokens)
        (OUT / f"{name}.qss").write_text(text, encoding="utf-8")
        print(f"wrote {OUT / f'{name}.qss'}")


if __name__ == "__main__":
    main()
