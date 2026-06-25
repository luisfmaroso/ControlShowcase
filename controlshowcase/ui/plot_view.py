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

A **hover tooltip** shows the X (time) and Y value of the nearest signal under the
cursor. Because the PWM trace lives on its own right-axis viewbox, each candidate point
is mapped through its own viewbox for the pixel-distance test, but the marker/readout
is placed back in the main viewbox so a single overlay serves all three signals.

The MainWindow pushes the simulator's history here once per UI tick via
:meth:`update_data`; this widget owns no state beyond the curves and the last history.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QPointF, Qt
from PySide6.QtWidgets import QVBoxLayout, QWidget

from ..sim.simulator import DEFAULT_WINDOW_S
from . import theme

# Dark plot theme, matching the window chrome and the docs' code blocks.
pg.setConfigOption("background", theme.BACKGROUND)
pg.setConfigOption("foreground", theme.FOREGROUND)
pg.setConfigOptions(antialias=True)

# How close (in pixels) the cursor must be to a signal for the tooltip to show.
HOVER_THRESHOLD_PX = 20.0


def _readout_html(name: str, x: float, y: float, unit: str, color: str) -> str:
    return (
        "<div style='font-family:monospace; color:#e6edf3'>"
        f"<span style='color:{color}'>{name}</span><br>"
        f"x = {x:.3f} s<br>y = {y:.2f} {unit}</div>"
    )


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
        self._hist: dict | None = None

        # Each hoverable signal: label, curve, its viewbox, unit, colour, history key.
        self._channels = [
            ("Setpoint", self._setpoint_curve, self._plot_item.vb, "mm",
             theme.COLOR_SETPOINT, "setpoint"),
            ("Position", self._position_curve, self._plot_item.vb, "mm",
             theme.COLOR_POSITION, "measured"),
            ("PWM", self._pwm_curve, self._pwm_vb, "%",
             theme.COLOR_PWM, "pwm"),
        ]

        self._build_hover()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.plot_widget)

    # --- hover tooltip --------------------------------------------------
    def _build_hover(self) -> None:
        self._marker = pg.ScatterPlotItem(size=11, pen=pg.mkPen("#ffffff", width=1.5))
        self._readout = pg.TextItem(anchor=(0, 1), fill=pg.mkBrush(30, 34, 40, 220))
        self._marker.setZValue(100)
        self._readout.setZValue(100)
        self._plot_item.addItem(self._marker, ignoreBounds=True)
        self._plot_item.addItem(self._readout, ignoreBounds=True)
        self._hover_enabled = True
        self._set_hover_visible(False)

        # Rate-limited mouse tracking over the plot scene.
        self._proxy = pg.SignalProxy(
            self.plot_widget.scene().sigMouseMoved,
            rateLimit=60,
            slot=self._on_mouse_moved,
        )

    def _set_hover_visible(self, visible: bool) -> None:
        self._marker.setVisible(visible)
        self._readout.setVisible(visible)

    def set_hover_enabled(self, enabled: bool) -> None:
        self._hover_enabled = enabled
        if not enabled:
            self._set_hover_visible(False)

    def _on_mouse_moved(self, event) -> None:
        pos = event[0]  # SignalProxy wraps args in a tuple
        hist = self._hist
        if not self._hover_enabled or hist is None or hist["t"].size == 0:
            return
        if not self._plot_item.sceneBoundingRect().contains(pos):
            self._set_hover_visible(False)
            return

        main_vb = self._plot_item.vb
        t = hist["t"]
        cursor_t = main_vb.mapSceneToView(pos).x()
        idx = int(np.searchsorted(t, cursor_t))
        if idx > 0 and (idx >= t.size or abs(t[idx - 1] - cursor_t) <= abs(t[idx] - cursor_t)):
            idx -= 1
        idx = max(0, min(t.size - 1, idx))

        best = None
        best_dist = HOVER_THRESHOLD_PX
        for name, curve, vb, unit, color, key in self._channels:
            if not curve.isVisible():
                continue
            x = float(t[idx])
            y = float(hist[key][idx])
            scene_pt = vb.mapViewToScene(QPointF(x, y))
            dist = float(np.hypot(scene_pt.x() - pos.x(), scene_pt.y() - pos.y()))
            if dist < best_dist:
                best_dist = dist
                best = (name, x, y, unit, color, scene_pt)

        if best is None:
            self._set_hover_visible(False)
            return

        name, x, y, unit, color, scene_pt = best
        # Place the single overlay in the main viewbox, at the hit point's location.
        main_pt = main_vb.mapSceneToView(scene_pt)
        self._marker.setData([main_pt.x()], [main_pt.y()], brush=pg.mkBrush(color))
        self._readout.setHtml(_readout_html(name, x, y, unit, color))
        self._readout.setPos(main_pt)
        self._set_hover_visible(True)

    # --- ranges / data --------------------------------------------------
    def _sync_pwm_vb(self) -> None:
        """Keep the secondary viewbox glued to the main plot's geometry."""
        self._pwm_vb.setGeometry(self._plot_item.vb.sceneBoundingRect())
        self._pwm_vb.linkedViewChanged(self._plot_item.vb, self._pwm_vb.XAxis)

    def set_stroke(self, stroke: float) -> None:
        """Fix the left Y axis to the cylinder's travel, with a little headroom."""
        self._plot_item.setYRange(-0.05 * stroke, 1.05 * stroke, padding=0)

    def set_pwm_visible(self, visible: bool) -> None:
        """Show/hide the PWM trace and its right axis (also drops it from the hover)."""
        self._pwm_curve.setVisible(visible)
        self._plot_item.showAxis("right", visible)

    def update_data(self, hist: dict) -> None:
        """Redraw from the simulator history and roll the X window to 'now'."""
        self._hist = hist
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
