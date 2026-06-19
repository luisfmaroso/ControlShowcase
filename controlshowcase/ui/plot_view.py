"""The plot.

A thin wrapper around a pyqtgraph PlotWidget that shows the control loop over a
rolling time window: the setpoint and the cylinder's actual position, with the
valve PWM and the tracking error available as optional traces.

Phase 0: just a themed, empty plot with axis labels and a legend. Live curves and
the rolling buffer arrive in Phase 2 once the simulator exists.
"""

from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtWidgets import QVBoxLayout, QWidget

from . import theme

# Dark plot theme, matching the window chrome and the docs' code blocks.
pg.setConfigOption("background", theme.BACKGROUND)
pg.setConfigOption("foreground", theme.FOREGROUND)
pg.setConfigOptions(antialias=True)


class PlotView(QWidget):
    """Time-series view of the control loop (setpoint vs. position)."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.plot_widget = pg.PlotWidget()
        self._plot_item = self.plot_widget.getPlotItem()
        self._plot_item.setMenuEnabled(False)
        self._plot_item.showGrid(x=True, y=True, alpha=theme.GRID_ALPHA)
        self._plot_item.setLabel("bottom", "Time", units="s")
        self._plot_item.setLabel("left", "Position", units="mm")
        self._plot_item.addLegend(offset=(-10, 10))

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot_widget)
