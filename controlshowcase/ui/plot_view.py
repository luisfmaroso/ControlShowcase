"""The plot.

A pyqtgraph PlotWidget showing the control loop over a rolling time window:

  * **setpoint** (dashed orange) and the **measured position** (blue) on the left axis
    (mm) — "measured" because, with sensor noise enabled, this is the noisy reading the
    controller actually chases (it equals the true position when noise is off);
  * the valve **PWM command** (green) on a secondary right axis (%), since it lives on
    a different scale — this is what exposes a controller's effort and its saturation.

The tracking error isn't drawn as its own trace: it is simply the visible gap between
the setpoint and the position curves. The true (noise-free) position is shown by the
cylinder animation.

The MainWindow pushes the simulator's history here once per UI tick via
:meth:`update_data`; this widget owns no state beyond the curves.
"""

from __future__ import annotations

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QVBoxLayout, QWidget

from ..sim.simulator import DEFAULT_WINDOW_S
from . import theme

# Dark plot theme, matching the window chrome and the docs' code blocks.
pg.setConfigOption("background", theme.BACKGROUND)
pg.setConfigOption("foreground", theme.FOREGROUND)
pg.setConfigOptions(antialias=True)


class PlotView(QWidget):
    """Rolling time-series view of the control loop (setpoint, position, PWM)."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self.plot_widget = pg.PlotWidget()
        self._plot_item = self.plot_widget.getPlotItem()
        self._plot_item.setMenuEnabled(False)
        self._plot_item.showGrid(x=True, y=True, alpha=theme.GRID_ALPHA)
        self._plot_item.setLabel("bottom", "Time", units="s")
        self._plot_item.setLabel("left", "Position", units="mm")
        self._legend = self._plot_item.addLegend(offset=(-10, 10))
        # We drive the X/Y ranges ourselves (rolling X, fixed Y), so no auto-range.
        self._plot_item.disableAutoRange()

        self._setpoint_curve = self._plot_item.plot(
            [], [], name="Setpoint",
            pen=pg.mkPen(theme.COLOR_SETPOINT, width=2, style=Qt.PenStyle.DashLine),
        )
        self._position_curve = self._plot_item.plot(
            [], [], name="Position",
            pen=pg.mkPen(theme.COLOR_POSITION, width=2),
        )

        # Secondary right axis for the PWM command (a different unit: %).
        self._pwm_vb = pg.ViewBox()
        self._plot_item.scene().addItem(self._pwm_vb)
        right_axis = self._plot_item.getAxis("right")
        right_axis.linkToView(self._pwm_vb)
        right_axis.setLabel("PWM command", units="%")
        self._pwm_vb.setXLink(self._plot_item)
        self._pwm_vb.setYRange(-105.0, 105.0, padding=0)
        self._pwm_curve = pg.PlotDataItem(pen=pg.mkPen(theme.COLOR_PWM, width=1.5))
        self._pwm_vb.addItem(self._pwm_curve)
        self._legend.addItem(self._pwm_curve, "PWM")
        self._plot_item.vb.sigResized.connect(self._sync_pwm_vb)
        self.set_pwm_visible(True)

        self._window_s = DEFAULT_WINDOW_S

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot_widget)

    def _sync_pwm_vb(self) -> None:
        """Keep the secondary viewbox glued to the main plot's geometry."""
        self._pwm_vb.setGeometry(self._plot_item.vb.sceneBoundingRect())
        self._pwm_vb.linkedViewChanged(self._plot_item.vb, self._pwm_vb.XAxis)

    def set_stroke(self, stroke: float) -> None:
        """Fix the left Y axis to the cylinder's travel, with a little headroom."""
        self._plot_item.setYRange(-0.05 * stroke, 1.05 * stroke, padding=0)

    def set_pwm_visible(self, visible: bool) -> None:
        """Show/hide the PWM trace and its right axis."""
        self._pwm_curve.setVisible(visible)
        self._plot_item.showAxis("right", visible)

    def update_data(self, hist: dict) -> None:
        """Redraw from the simulator history and roll the X window to 'now'."""
        t = hist["t"]
        self._setpoint_curve.setData(t, hist["setpoint"])
        self._position_curve.setData(t, hist["measured"])
        self._pwm_curve.setData(t, hist["pwm"])
        if t.size:
            t_now = float(t[-1])
            self._plot_item.setXRange(
                max(0.0, t_now - self._window_s),
                max(self._window_s, t_now),
                padding=0,
            )
