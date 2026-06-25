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
from .control.calibration import CalibrationController
from .control.manual import ManualController
from .control.mpc import MPCController, MPCModel, MPCParams
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

        # 100% load = 75% of the cylinder's stall force: heavy enough for a big droop
        # under P-only, but still holdable with integral action / enough PWM.
        self._max_load = 0.75 * self.sim.plant.extend_stall_force
        self._load_fraction = 0.0

        # One controller per mode; the active one is picked by the panel's tab.
        self._manual = ManualController()
        self._pid = PIDController(PIDGains())
        p = self.sim.plant.params
        mpc_model = MPCModel(m=p.mass, k_hyd=p.k_hyd, b=p.b_visc, vmax=p.vmax_extend)
        self._mpc = MPCController(mpc_model, MPCParams())
        self._calib = CalibrationController(stroke=p.stroke)
        self._controllers: dict[str, Controller] = {
            "Manual": self._manual, "PID": self._pid, "MPC": self._mpc,
            "Calibration": self._calib,
        }
        self._controller: Controller | None = self._manual
        self._last_pwm = 0.0
        self._calib_reported = True  # have we shown the current run's result yet?
        # While calibrating, the load is temporarily lifted (so the experiment works at
        # any load setting); this holds the force to restore afterwards, or None.
        self._load_before_calib: float | None = None

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
        self.control_panel = ControlPanel(
            self.sim.plant.params.stroke, self._pid.gains, self._mpc.params, self
        )
        self.control_panel.setpoint_changed.connect(self.sim.set_setpoint)
        self.control_panel.noise_changed.connect(self.sim.set_noise_std)
        self.control_panel.load_changed.connect(self._on_load_changed)
        self.control_panel.manual_pwm_changed.connect(self._manual.set_command)
        self.control_panel.pid_gains_changed.connect(self._pid.set_gains)
        self.control_panel.mpc_weights_changed.connect(self._mpc.set_weights)
        self.control_panel.mpc_horizon_changed.connect(self._mpc.set_horizon)
        self.control_panel.calibration_requested.connect(self._on_calibration_requested)
        self.control_panel.calibration_apply_requested.connect(self._on_calibration_apply)
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

        hover_action = QAction("&Hover tooltip", self)
        hover_action.setCheckable(True)
        hover_action.setChecked(True)
        hover_action.setShortcut("Ctrl+H")
        hover_action.toggled.connect(self.plot_view.set_hover_enabled)
        view_menu.addAction(hover_action)
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
            if self._controller is None:
                pwm = 0.0  # an unimplemented mode just holds the valve shut
            else:
                # Controllers act on the true position; sensor noise is display-only.
                pwm = self._controller.compute(sp, self.sim.position, self.sim.dt)
            self.sim.step(pwm)
            self._last_pwm = pwm
        if self._controller is self._calib:
            self._report_calibration()
        self._refresh_views()

    def _refresh_views(self) -> None:
        self.plot_view.update_data(self.sim.history())
        # The weight is drawn lifted (0) while it's lifted for calibration.
        load_frac = 0.0 if self._load_before_calib is not None else self._load_fraction
        self.cylinder_view.set_state(
            self.sim.position_fraction, self.sim.setpoint_fraction, load_frac
        )
        error = self.sim.setpoint - self.sim.position
        self.control_panel.pid_tab.update_readout(error, self._last_pwm)
        self.statusBar().showMessage(
            f"position {self.sim.position:6.1f} mm     "
            f"setpoint {self.sim.setpoint:6.1f} mm     "
            f"error {error:+6.1f} mm     PWM {self._last_pwm:+6.1f} %"
        )

    # --- slots ----------------------------------------------------------
    def _on_mode_changed(self, mode: str) -> None:
        # Leaving a calibration run (e.g. user switches tabs mid-test) restores the load.
        self._restore_calib_load()
        self._controller = self._controllers.get(mode)
        if self._controller is not None:
            self._controller.reset()
        # Manual's command must follow the slider's current position on activation.
        if self._controller is self._manual:
            self._manual.set_command(self.control_panel.manual_tab.command)

    def _on_calibration_requested(self, tau_c: float) -> None:
        # The Calibration tab is active, so self._calib is the live controller.
        # Lift the load for the duration — the deadband and gain are load-independent,
        # so the experiment identifies them cleanly and works at any load setting.
        self._load_before_calib = self.sim.plant.external_load
        self.sim.plant.set_load(0.0)
        self._calib.start(tau_c)
        self._calib_reported = False

    def _restore_calib_load(self) -> None:
        if self._load_before_calib is not None:
            self.sim.plant.set_load(self._load_before_calib)
            self._load_before_calib = None

    def _report_calibration(self) -> None:
        tab = self.control_panel.calibration_tab
        tab.set_progress(self._calib.progress)
        if self._calib.done and not self._calib_reported and self._calib.result is not None:
            tab.set_result(self._calib.result)
            self._restore_calib_load()
            self._calib_reported = True

    def _on_calibration_apply(self, kp: float, ki: float, kd: float) -> None:
        # Push the gains into the PID tab (which updates the controller), then show it.
        self.control_panel.pid_tab.set_gains(kp, ki, kd)
        self.control_panel.tabs.setCurrentWidget(self.control_panel.pid_tab)

    def _on_load_changed(self, percent: float) -> None:
        self._load_fraction = percent / 100.0
        force = self._load_fraction * self._max_load
        if self._load_before_calib is not None:
            # Mid-calibration the load is lifted; remember the new value to restore.
            self._load_before_calib = force
        else:
            self.sim.plant.set_load(force)
        # The MPC is told the load (its known-disturbance feed-forward); PID is not —
        # PID must reject it as an unknown disturbance, which is the comparison.
        self._mpc.set_load(force)

    def _on_run_toggled(self, running: bool) -> None:
        if running:
            self._timer.start()
        else:
            self._timer.stop()

    def _on_reset(self) -> None:
        self._restore_calib_load()
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
