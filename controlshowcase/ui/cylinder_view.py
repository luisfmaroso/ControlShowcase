"""The cylinder animation.

A custom-painted widget that draws the hydraulic cylinder side-on: a barrel, a
piston inside it, and the rod sticking out. The piston slides with the simulated
position so you can watch the actuator move alongside the plot.

Phase 0: paints a static cylinder at mid-stroke to establish the layout. The
``set_state`` hook is already here; Phase 2 wires the simulation loop to it.
"""

from __future__ import annotations

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QBrush, QColor, QPainter, QPen
from PySide6.QtWidgets import QSizePolicy, QWidget

from . import theme


class CylinderView(QWidget):
    """Side-on view of the cylinder; the piston tracks ``position``."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(140)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        # Normalised stroke fraction in [0, 1]; 0 = fully retracted.
        self._position = 0.5
        # Setpoint as a stroke fraction, or None when no target is set.
        self._setpoint: float | None = None
        # Load as a 0..1 fraction; drives the size of the weight on the rod.
        self._load = 0.0

    def set_state(
        self, position: float, setpoint: float | None = None, load: float = 0.0
    ) -> None:
        """Update the drawn piston/target/load. ``position`` is a 0..1 stroke fraction."""
        self._position = max(0.0, min(1.0, position))
        self._setpoint = setpoint
        self._load = max(0.0, min(1.0, load))
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802 (Qt naming)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        margin = 16.0
        barrel = QRectF(margin, h * 0.3, w * 0.62, h * 0.4)

        # Barrel.
        painter.setPen(QPen(QColor(theme.FOREGROUND), 1.5))
        painter.setBrush(QBrush(QColor(theme.COLOR_BARREL)))
        painter.drawRect(barrel)

        # Piston: a block that slides within the barrel.
        piston_w = barrel.width() * 0.12
        travel = barrel.width() - piston_w
        piston_x = barrel.left() + self._position * travel
        piston = QRectF(piston_x, barrel.top(), piston_w, barrel.height())
        painter.setBrush(QBrush(QColor(theme.COLOR_PISTON)))
        painter.drawRect(piston)

        # Rod: from the piston out through the right cap.
        rod_h = barrel.height() * 0.18
        rod_y = barrel.center().y() - rod_h / 2
        rod = QRectF(piston.right(), rod_y, (w - margin) - piston.right(), rod_h)
        painter.setBrush(QBrush(QColor(theme.COLOR_ROD)))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(rod)

        # Load: a weight hanging from the rod tip, sized by the load fraction. It
        # opposes extension, so it is the disturbance the controller has to fight.
        if self._load > 0.0:
            tip_x = rod.right()
            tip_y = rod.center().y()
            side = 12.0 + 30.0 * self._load
            cable = 10.0
            box = QRectF(tip_x - side / 2, tip_y + cable, side, side)
            painter.setPen(QPen(QColor(theme.FOREGROUND), 1.0))
            painter.drawLine(int(tip_x), int(tip_y), int(tip_x), int(tip_y + cable))
            painter.setBrush(QBrush(QColor(theme.COLOR_LOAD)))
            painter.drawRect(box)

        # Setpoint marker: a vertical line at the target stroke fraction.
        if self._setpoint is not None:
            sp_x = barrel.left() + max(0.0, min(1.0, self._setpoint)) * travel + piston_w / 2
            painter.setPen(QPen(QColor(theme.COLOR_SETPOINT), 1.5, Qt.PenStyle.DashLine))
            painter.drawLine(int(sp_x), int(barrel.top() - 8), int(sp_x), int(barrel.bottom() + 8))

        painter.end()
