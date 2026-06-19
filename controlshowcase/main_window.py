"""MainWindow — layout and wiring (the glue).

The window has three zones, mirroring the brief:

  * a **control panel** (left dock) — a shared setpoint/run/reset header plus tabs
    to pick the mode and tune parameters;
  * the **cylinder view** (top of the central area) — the animated actuator;
  * the **plot** (below it) — setpoint vs. actual position over time, plus PWM.

Phase 3: closed-loop control. MainWindow owns one :class:`Controller` per mode and
the active one is chosen by the control panel's current tab. Each fixed sim step the
active controller computes the PWM from the loop state — for Manual that is the
slider value; for PID it is the P/I/D law — and the simulator integrates it.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QDockWidget,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QWidget,
)

from . import __version__
from .control.base import Controller
from .control.manual import ManualController
from .control.pid import PIDController, PIDGains
from .sim.simulator import Simulator
from .ui.control_panel import ControlPanel
from .ui.cylinder_view import CylinderView
from .ui.plot_view import PlotView

UI_INTERVAL_MS = 20  # ~50 Hz refresh; the sim runs several steps per tick


class MainWindow(QMainWindow):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowTitle("ControlShowcase")
        self.resize(1100, 720)

        # The plant + loop, the single source of truth.
        self.sim = Simulator()
        self._steps_per_tick = max(1, round((UI_INTERVAL_MS / 1000.0) / self.sim.dt))

        # One controller per mode; the active one is picked by the panel's tab.
        self._manual = ManualController()
        self._pid = PIDController(PIDGains())
        self._controllers: dict[str, Controller] = {"Manual": self._manual, "PID": self._pid}
        self._controller: Controller | None = self._manual
        self._last_pwm = 0.0

        # Central area: cylinder on top, plot below, in a vertical splitter.
        self.cylinder_view = CylinderView(self)
        self.plot_view = PlotView(self)
        self.plot_view.set_stroke(self.sim.plant.params.stroke)

        splitter = QSplitter(Qt.Orientation.Vertical, self)
        splitter.addWidget(self.cylinder_view)
        splitter.addWidget(self.plot_view)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        self.setCentralWidget(splitter)

        # Left dock: the tabbed control panel.
        self.control_panel = ControlPanel(self.sim.plant.params.stroke, self._pid.gains, self)
        self.control_panel.setpoint_changed.connect(self.sim.set_setpoint)
        self.control_panel.noise_changed.connect(self.sim.set_noise_std)
        self.control_panel.manual_pwm_changed.connect(self._manual.set_command)
        self.control_panel.pid_gains_changed.connect(self._pid.set_gains)
        self.control_panel.mode_changed.connect(self._on_mode_changed)
        self.control_panel.run_toggled.connect(self._on_run_toggled)
        self.control_panel.reset_requested.connect(self._on_reset)

        self._control_dock = QDockWidget("Control", self)
        self._control_dock.setWidget(self.control_panel)
        self._control_dock.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea
        )
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self._control_dock)
        self.resizeDocks([self._control_dock], [340], Qt.Orientation.Horizontal)

        self._build_menus()

        # Start the loop with the panel's initial setpoint.
        self.sim.set_setpoint(self.control_panel.setpoint)
        self._timer = QTimer(self)
        self._timer.setInterval(UI_INTERVAL_MS)
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()
        self._refresh_views()

    def _build_menus(self) -> None:
        menu = self.menuBar()

        file_menu = menu.addMenu("&File")
        quit_action = QAction("E&xit", self)
        quit_action.setShortcut(QKeySequence.StandardKey.Quit)
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        view_menu = menu.addMenu("&View")
        pwm_action = QAction("Show &PWM trace", self)
        pwm_action.setCheckable(True)
        pwm_action.setChecked(True)
        pwm_action.toggled.connect(self.plot_view.set_pwm_visible)
        view_menu.addAction(pwm_action)
        view_menu.addSeparator()
        view_menu.addAction(self._control_dock.toggleViewAction())

        help_menu = menu.addMenu("&Help")
        about_action = QAction("&About", self)
        about_action.triggered.connect(self._on_about)
        help_menu.addAction(about_action)

    # --- the loop -------------------------------------------------------
    def _on_tick(self) -> None:
        sp = self.sim.setpoint
        for _ in range(self._steps_per_tick):
            measurement = self.sim.measure()  # noisy sensor reading of the position
            if self._controller is None:
                pwm = 0.0  # an unimplemented mode just holds the valve shut
            else:
                pwm = self._controller.compute(sp, measurement, self.sim.dt)
            self.sim.step(pwm)
            self._last_pwm = pwm
        self._refresh_views()

    def _refresh_views(self) -> None:
        self.plot_view.update_data(self.sim.history())
        self.cylinder_view.set_state(self.sim.position_fraction, self.sim.setpoint_fraction)
        error = self.sim.setpoint - self.sim.position
        self.control_panel.pid_tab.update_readout(error, self._last_pwm)
        self.statusBar().showMessage(
            f"position {self.sim.position:6.1f} mm     "
            f"setpoint {self.sim.setpoint:6.1f} mm     "
            f"error {error:+6.1f} mm     PWM {self._last_pwm:+6.1f} %"
        )

    # --- slots ----------------------------------------------------------
    def _on_mode_changed(self, mode: str) -> None:
        self._controller = self._controllers.get(mode)
        if self._controller is not None:
            self._controller.reset()
        # Manual's command must follow the slider's current position on activation.
        if self._controller is self._manual:
            self._manual.set_command(self.control_panel.manual_tab.command)

    def _on_run_toggled(self, running: bool) -> None:
        if running:
            self._timer.start()
        else:
            self._timer.stop()

    def _on_reset(self) -> None:
        setpoint = self.sim.setpoint
        self.sim.reset(0.0)
        self.sim.set_setpoint(setpoint)
        if self._controller is not None:
            self._controller.reset()
        self.control_panel.reset()
        self._last_pwm = 0.0
        self._refresh_views()

    def _on_about(self) -> None:
        QMessageBox.about(
            self,
            "About ControlShowcase",
            f"<b>ControlShowcase</b> v{__version__}<br>"
            "A teaching simulator for closed-loop position control of a "
            "hydraulic cylinder.<br>Built with PySide6 + pyqtgraph.",
        )
