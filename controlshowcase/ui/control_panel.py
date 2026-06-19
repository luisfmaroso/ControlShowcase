"""The control panel.

A tabbed widget — one tab per control mode (Manual, PID, MPC, Calibration). Each
tab exposes the parameters for its mode and emits signals when the user changes
them; the MainWindow listens and swaps/retunes the active controller. The panel
never touches the plant or the simulator directly.

Phase 0: the four tabs exist as labelled placeholders so the layout is real. The
controls and signals are filled in per phase (Manual in Phase 2, PID in Phase 3,
MPC in Phase 4, Calibration in Phase 5).
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QLabel, QTabWidget, QVBoxLayout, QWidget


def _placeholder(text: str) -> QWidget:
    """A simple centred-label tab, used until each mode's controls are built."""
    page = QWidget()
    layout = QVBoxLayout(page)
    layout.setAlignment(Qt.AlignmentFlag.AlignTop)
    label = QLabel(text)
    label.setWordWrap(True)
    layout.addWidget(label)
    layout.addStretch(1)
    return page


class ControlPanel(QTabWidget):
    """Mode + parameter controls, one tab per controller."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.addTab(_placeholder("Manual PWM control — coming in Phase 2."), "Manual")
        self.addTab(_placeholder("PID with live P / I / D — coming in Phase 3."), "PID")
        self.addTab(_placeholder("Model-predictive control — coming in Phase 4."), "MPC")
        self.addTab(_placeholder("Auto-calibration — coming in Phase 5."), "Calibration")
