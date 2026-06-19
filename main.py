"""ControlShowcase entry point."""

from __future__ import annotations

import sys

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

from controlshowcase.main_window import MainWindow


def apply_dark_theme(app: QApplication) -> None:
    """A dark Fusion palette so the window chrome matches the dark plot.

    Lifted from the sibling CsvPlotter project so the two tools look like family.
    """
    app.setStyle("Fusion")

    bg = QColor("#1e2228")
    base = QColor("#262b33")
    text = QColor("#c8d0d8")
    accent = QColor("#3b6ea8")

    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, bg)
    palette.setColor(QPalette.ColorRole.WindowText, text)
    palette.setColor(QPalette.ColorRole.Base, base)
    palette.setColor(QPalette.ColorRole.AlternateBase, bg)
    palette.setColor(QPalette.ColorRole.Text, text)
    palette.setColor(QPalette.ColorRole.Button, base)
    palette.setColor(QPalette.ColorRole.ButtonText, text)
    palette.setColor(QPalette.ColorRole.ToolTipBase, base)
    palette.setColor(QPalette.ColorRole.ToolTipText, text)
    palette.setColor(QPalette.ColorRole.Highlight, accent)
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#ffffff"))
    palette.setColor(QPalette.ColorRole.PlaceholderText, QColor("#7a828c"))
    disabled = QColor("#6b7280")
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, disabled)
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, disabled)
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, disabled)
    app.setPalette(palette)


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("ControlShowcase")
    apply_dark_theme(app)

    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
