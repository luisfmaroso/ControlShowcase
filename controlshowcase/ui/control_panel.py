"""The control panel.

A shared header (setpoint + Run/Pause + Reset, which apply in every mode) sits above
a tab widget with one tab per control mode. Each tab owns the parameters for its mode
and emits signals when the user changes them; the MainWindow listens, swaps the active
controller on a tab change, and retunes it live. The panel never touches the plant or
the simulator directly.

Phase 3: **Manual** and **PID** are live. MPC / Calibration are still placeholders.
"""

from __future__ import annotations

from PySide6.QtCore import QLocale, Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..control.calibration import CalibrationResult
from ..control.mpc import MPCParams
from ..control.pid import PIDGains

# Effectively "no limit" for the gains, so extreme tunings can be explored.
GAIN_MAX = 1.0e6


class DecimalSpinBox(QDoubleSpinBox):
    """A QDoubleSpinBox that is independent of the system locale's decimal separator.

    The default spin box uses the system locale's separator, so under a comma-locale
    (e.g. pt-BR) typing ``1.5`` is read as the thousands group ``15`` and silently
    rejected. This forces ``.`` as the separator for display and accepts both ``.``
    and ``,`` on input, so typing a decimal works the same everywhere.
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setLocale(QLocale(QLocale.Language.C))

    def validate(self, text: str, pos: int):  # noqa: N802 (Qt naming)
        return super().validate(text.replace(",", "."), pos)

    def valueFromText(self, text: str) -> float:  # noqa: N802 (Qt naming)
        cleaned = text.replace(self.suffix(), "").replace(",", ".").strip()
        if cleaned in ("", "+", "-", ".", "+.", "-."):
            return 0.0
        try:
            return float(cleaned)
        except ValueError:
            return 0.0


def _placeholder(text: str) -> QWidget:
    """A simple top-aligned label tab, used until a mode's controls are built."""
    page = QWidget()
    layout = QVBoxLayout(page)
    layout.setAlignment(Qt.AlignmentFlag.AlignTop)
    label = QLabel(text)
    label.setWordWrap(True)
    layout.addWidget(label)
    return page


class ManualTab(QWidget):
    """Drive the valve PWM by hand with a slider; learn the open-loop plant."""

    pwm_changed = Signal(float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._slider = QSlider(Qt.Orientation.Horizontal)
        self._slider.setRange(-100, 100)
        self._slider.setValue(0)
        self._slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._slider.setTickInterval(25)
        self._slider.valueChanged.connect(self._on_slider)

        self._readout = QLabel("PWM: +0 %")

        zero_btn = QPushButton("Return to 0")
        zero_btn.clicked.connect(lambda: self._slider.setValue(0))

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.addWidget(QLabel("Valve PWM command (retract − → + extend)"))
        layout.addWidget(self._slider)
        layout.addWidget(self._readout)
        layout.addWidget(zero_btn)
        layout.addStretch(1)

    def _on_slider(self, value: int) -> None:
        self._readout.setText(f"PWM: {value:+d} %")
        self.pwm_changed.emit(float(value))

    @property
    def command(self) -> float:
        return float(self._slider.value())

    def reset(self) -> None:
        self._slider.setValue(0)


class PIDTab(QWidget):
    """Live P / I / D tuning plus an error / output readout."""

    gains_changed = Signal(float, float, float)

    def __init__(self, gains: PIDGains, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._kp = self._spin(0.05, 3, gains.kp)
        self._ki = self._spin(0.01, 3, gains.ki)
        self._kd = self._spin(0.005, 4, gains.kd)
        for spin in (self._kp, self._ki, self._kd):
            spin.valueChanged.connect(self._emit)

        self._readout = QLabel("error —    output —")

        form = QFormLayout(self)
        form.addRow("P (kp, %/mm):", self._kp)
        form.addRow("I (ki, %/mm·s):", self._ki)
        form.addRow("D (kd, %/mm/s):", self._kd)
        form.addRow(self._readout)

    @staticmethod
    def _spin(step: float, decimals: int, value: float) -> DecimalSpinBox:
        spin = DecimalSpinBox()
        spin.setRange(0.0, GAIN_MAX)  # no practical upper limit
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setValue(value)
        return spin

    def _emit(self) -> None:
        self.gains_changed.emit(self._kp.value(), self._ki.value(), self._kd.value())

    def update_readout(self, error: float, output: float) -> None:
        self._readout.setText(f"error {error:+7.1f} mm    output {output:+6.1f} %")

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """Set all three gains at once (used by Apply from calibration)."""
        for spin, value in ((self._kp, kp), (self._ki, ki), (self._kd, kd)):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)
        self._emit()  # one combined update to the controller


class MPCTab(QWidget):
    """Live MPC tuning: horizon N, tracking Q, move-suppression R."""

    weights_changed = Signal(float, float)  # (q, r)
    horizon_changed = Signal(int)

    def __init__(self, params: MPCParams, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._n = QSpinBox()
        self._n.setRange(2, 200)
        self._n.setValue(params.horizon)
        self._n.valueChanged.connect(lambda v: self.horizon_changed.emit(int(v)))

        self._q = self._spin(0.05, 3, params.q)
        self._r = self._spin(0.1, 3, params.r)
        for spin in (self._q, self._r):
            spin.valueChanged.connect(self._emit_weights)

        note = QLabel(
            f"Preview horizon = N × {int(params.ts * 1000)} ms. "
            "Q = tracking, R = move-suppression (smoothness). "
            "The load is fed forward when set."
        )
        note.setWordWrap(True)

        form = QFormLayout(self)
        form.addRow("Horizon N:", self._n)
        form.addRow("Tracking Q:", self._q)
        form.addRow("Effort R (move):", self._r)
        form.addRow(note)

    @staticmethod
    def _spin(step: float, decimals: int, value: float) -> DecimalSpinBox:
        spin = DecimalSpinBox()
        spin.setRange(0.0, GAIN_MAX)
        spin.setSingleStep(step)
        spin.setDecimals(decimals)
        spin.setValue(value)
        return spin

    def _emit_weights(self) -> None:
        self.weights_changed.emit(self._q.value(), self._r.value())


class CalibrationTab(QWidget):
    """Safe, model-based auto-tuning: identify the plant with gentle bounded moves,
    then propose PID gains. Pick an aggressiveness, Run, then Apply to PID."""

    calibration_requested = Signal(float)  # tau_c (the aggressiveness knob)
    apply_requested = Signal(float, float, float)

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        self._aggr = QComboBox()
        self._aggr.addItem("Conservative (gentle)", 0.5)
        self._aggr.addItem("Balanced", 0.3)
        self._aggr.addItem("Aggressive (fast)", 0.15)

        self._run_btn = QPushButton("Run calibration")
        self._run_btn.clicked.connect(self._on_run)

        self._progress = QLabel("Idle.")
        self._progress.setWordWrap(True)
        self._results = QLabel("—")
        self._results.setWordWrap(True)
        self._results.setTextFormat(Qt.TextFormat.RichText)

        self._apply_btn = QPushButton("Apply to PID")
        self._apply_btn.setEnabled(False)
        self._apply_btn.clicked.connect(self._on_apply)

        note = QLabel(
            "Runs a few gentle, bounded open-loop moves to find the deadband and a "
            "plant model, then computes PID gains — no oscillation, no instability. "
            "The load is lifted for the test, so it works at any load setting; best with "
            "low sensor noise."
        )
        note.setWordWrap(True)

        self._kp = self._ki = self._kd = 0.0

        form = QFormLayout()
        form.addRow("Aggressiveness:", self._aggr)

        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        layout.addLayout(form)
        layout.addWidget(self._run_btn)
        layout.addWidget(self._progress)
        layout.addWidget(self._results)
        layout.addWidget(self._apply_btn)
        layout.addWidget(note)

    def _on_run(self) -> None:
        self._run_btn.setEnabled(False)
        self._apply_btn.setEnabled(False)
        self._results.setText("—")
        self._progress.setText("Starting…")
        self.calibration_requested.emit(float(self._aggr.currentData()))

    def _on_apply(self) -> None:
        self.apply_requested.emit(self._kp, self._ki, self._kd)

    def set_progress(self, text: str) -> None:
        self._progress.setText(text)

    def set_result(self, result: CalibrationResult) -> None:
        self._run_btn.setEnabled(True)
        if result.success:
            self._kp, self._ki, self._kd = result.kp, result.ki, result.kd
            self._results.setText(
                f"Deadband: <b>{result.deadband:.1f}%</b><br>"
                f"Velocity gain: {result.kv:.2f} mm/s per %<br>"
                f"Lag τ: {result.tau * 1000:.0f} ms<br>"
                f"Proposed PID: kp={result.kp:.3f}, ki={result.ki:.3f}, kd={result.kd:.3f}"
            )
            self._apply_btn.setEnabled(True)
        else:
            self._results.setText(f"<b>Failed:</b> {result.message}")
            self._apply_btn.setEnabled(False)


class ControlPanel(QWidget):
    """Shared setpoint/run/reset header + one tab per controller."""

    setpoint_changed = Signal(float)
    noise_changed = Signal(float)
    load_changed = Signal(float)  # load as a percentage, 0..100
    manual_pwm_changed = Signal(float)
    pid_gains_changed = Signal(float, float, float)
    mpc_weights_changed = Signal(float, float)
    mpc_horizon_changed = Signal(int)
    calibration_requested = Signal(float)
    calibration_apply_requested = Signal(float, float, float)
    mode_changed = Signal(str)
    run_toggled = Signal(bool)
    reset_requested = Signal()

    def __init__(
        self,
        stroke: float,
        gains: PIDGains,
        mpc_params: MPCParams,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)

        # --- shared header ---------------------------------------------
        self._setpoint = DecimalSpinBox()
        self._setpoint.setRange(0.0, stroke)
        self._setpoint.setSuffix(" mm")
        self._setpoint.setDecimals(1)
        self._setpoint.setSingleStep(10.0)
        self._setpoint.setValue(stroke / 2.0)
        self._setpoint.valueChanged.connect(self.setpoint_changed)

        # Sensor noise: std-dev (mm) of Gaussian noise added to the measured position.
        self._noise = DecimalSpinBox()
        self._noise.setRange(0.0, GAIN_MAX)  # no practical limit
        self._noise.setSuffix(" mm")
        self._noise.setDecimals(2)
        self._noise.setSingleStep(0.5)
        self._noise.setValue(0.0)
        self._noise.valueChanged.connect(self.noise_changed)

        # Load: a weight on the rod opposing extension, 0..100% of the cylinder's hold.
        self._load = QSlider(Qt.Orientation.Horizontal)
        self._load.setRange(0, 100)
        self._load.setValue(0)
        self._load.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._load.setTickInterval(25)
        self._load.valueChanged.connect(self._on_load)
        self._load_readout = QLabel("0 %")
        load_row = QWidget()
        load_layout = QHBoxLayout(load_row)
        load_layout.setContentsMargins(0, 0, 0, 0)
        load_layout.addWidget(self._load, stretch=1)
        load_layout.addWidget(self._load_readout)

        form = QFormLayout()
        form.addRow("Setpoint:", self._setpoint)
        form.addRow("Sensor noise σ:", self._noise)
        form.addRow("Load:", load_row)

        self._run_btn = QPushButton("Running")
        self._run_btn.setCheckable(True)
        self._run_btn.setChecked(True)
        self._run_btn.toggled.connect(self._on_run_toggled)

        reset_btn = QPushButton("Reset")
        reset_btn.clicked.connect(self.reset_requested)

        buttons = QHBoxLayout()
        buttons.addWidget(self._run_btn)
        buttons.addWidget(reset_btn)

        # --- tabs ------------------------------------------------------
        self.tabs = QTabWidget()
        self.manual_tab = ManualTab()
        self.manual_tab.pwm_changed.connect(self.manual_pwm_changed)
        self.pid_tab = PIDTab(gains)
        self.pid_tab.gains_changed.connect(self.pid_gains_changed)
        self.mpc_tab = MPCTab(mpc_params)
        self.mpc_tab.weights_changed.connect(self.mpc_weights_changed)
        self.mpc_tab.horizon_changed.connect(self.mpc_horizon_changed)
        self.calibration_tab = CalibrationTab()
        self.calibration_tab.calibration_requested.connect(self.calibration_requested)
        self.calibration_tab.apply_requested.connect(self.calibration_apply_requested)
        self.tabs.addTab(self.manual_tab, "Manual")
        self.tabs.addTab(self.pid_tab, "PID")
        self.tabs.addTab(self.mpc_tab, "MPC")
        self.tabs.addTab(self.calibration_tab, "Calibration")
        self.tabs.currentChanged.connect(self._on_tab)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addLayout(buttons)
        layout.addWidget(self.tabs, stretch=1)

    @property
    def setpoint(self) -> float:
        return self._setpoint.value()

    @property
    def mode(self) -> str:
        return self.tabs.tabText(self.tabs.currentIndex())

    def _on_tab(self, index: int) -> None:
        self.mode_changed.emit(self.tabs.tabText(index))

    def _on_load(self, value: int) -> None:
        self._load_readout.setText(f"{value} %")
        self.load_changed.emit(float(value))

    def _on_run_toggled(self, running: bool) -> None:
        self._run_btn.setText("Running" if running else "Paused")
        self.run_toggled.emit(running)

    def reset(self) -> None:
        """Return the manual command to zero (called on a sim reset)."""
        self.manual_tab.reset()
