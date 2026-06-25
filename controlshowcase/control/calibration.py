"""Auto-calibration: a safe, model-based PID tuner.

Rather than the classic Ziegler-Nichols or relay methods — which deliberately push the
closed loop to (or into) sustained oscillation to find the stability limit, dangerous on
a heavy hydraulic axis — this identifies the plant with a few **gentle, bounded,
open-loop** moves and then computes the gains analytically. Nothing ever approaches
instability.

The experiment runs as a :class:`Controller` state machine (so the existing loop drives
it live and the user watches the moves):

    PARK      -> ease to a safe start position with a PWM-capped proportional approach
    DEADBAND  -> ramp PWM slowly from 0 until motion begins; that PWM is the deadband
    SETTLE    -> command 0 and let it stop
    STEP      -> one bounded constant-PWM step; measure steady velocity and the lag
    RETURN    -> ease back toward the park position, then finish

Throughout, PWM is capped and the position is kept inside a safe window (abort if it
escapes). From the step we get an integrating-plus-lag model

    P(s) = Kv / (s (tau s + 1))     Kv in mm/s per %,  tau the velocity lag

and tune it with **SIMC / lambda** rules whose single knob is the desired closed-loop
time constant ``tau_c`` (larger = gentler, the "aggressiveness" control). See
:func:`simc_pid`.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np

from .base import Controller, clamp_pwm


@dataclass
class CalibrationParams:
    ramp_rate: float = 8.0        # %/s — deadband ramp speed (gentle)
    ramp_pwm_cap: float = 35.0    # % — give up the deadband search beyond this
    step_pwm: float = 40.0        # % — the bounded step-test command
    step_time: float = 1.4        # s — how long to hold the step
    v_fit_lo: float = 2.0         # mm/s — lower edge of the rising band used for the fit
    v_fit_hi: float = 12.0        # mm/s — stop ramping once clearly moving
    tau_v: float = 0.05           # s — velocity-estimate low-pass
    park_frac: float = 0.20       # safe start, as a fraction of stroke
    park_tol: float = 20.0        # mm — "close enough" to park
    safe_lo_frac: float = 0.08    # abort if position leaves [lo, hi] * stroke
    safe_hi_frac: float = 0.85
    park_pwm_cap: float = 25.0    # % — gentle approach PWM cap
    park_kp: float = 0.5          # %/mm — proportional approach gain
    settle_v: float = 2.0         # mm/s — "stopped" threshold
    tau_c: float = 0.5            # s — closed-loop time constant (aggressiveness knob)
    phase_timeout: float = 14.0   # s — per-phase watchdog


@dataclass
class CalibrationResult:
    success: bool
    message: str
    deadband: float = 0.0  # %
    kv: float = 0.0        # mm/s per %
    tau: float = 0.0       # s
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


def simc_pid(kv: float, tau: float, tau_c: float) -> tuple[float, float, float]:
    """SIMC / lambda tuning for an integrating-plus-lag process ``Kv/(s(tau*s+1))``.

    ``tau_c`` is the desired closed-loop time constant (larger = gentler). The lag is
    treated as an effective delay ``theta = tau``; ``tau_c`` is clamped to ``>= theta``
    for robustness. Returns PID gains in this app's units (kp %/mm, ki %/(mm·s),
    kd %/(mm/s)).
    """
    theta = max(tau, 1.0e-3)
    tc = max(tau_c, theta)
    kv = max(kv, 1.0e-6)
    kc = 1.0 / (kv * (tc + theta))
    tau_i = 4.0 * (tc + theta)
    tau_d = tau
    return kc, kc / tau_i, kc * tau_d


class _Phase(Enum):
    IDLE = auto()
    PARK = auto()
    DEADBAND = auto()
    SETTLE = auto()
    STEP = auto()
    RETURN = auto()
    DONE = auto()
    ABORT = auto()


class CalibrationController(Controller):
    """Drives the bounded identification experiment and produces PID gains."""

    def __init__(self, stroke: float, params: CalibrationParams | None = None) -> None:
        self.stroke = stroke
        self.params = params or CalibrationParams()
        self.reset()

    # --- public API ----------------------------------------------------
    def start(self, tau_c: float | None = None) -> None:
        """Arm and begin the experiment (aggressiveness via ``tau_c``)."""
        self.reset()
        if tau_c is not None:
            self.params.tau_c = tau_c
        self._phase = _Phase.PARK
        self.progress = "Parking at safe start…"

    @property
    def done(self) -> bool:
        return self._phase in (_Phase.DONE, _Phase.ABORT)

    @property
    def running(self) -> bool:
        return self._phase not in (_Phase.IDLE, _Phase.DONE, _Phase.ABORT)

    def reset(self) -> None:
        self._phase = _Phase.IDLE
        self._t_phase = 0.0
        self._v_est = 0.0
        self._meas_prev: float | None = None
        self._ramp = 0.0
        self._x_ramp0 = 0.0
        self._x_step0 = 0.0
        self._deadband = 0.0
        self._db_pwm: list[float] = []
        self._db_v: list[float] = []
        self._step_t: list[float] = []
        self._step_v: list[float] = []
        self.result: CalibrationResult | None = None
        self.progress = "Idle"

    # --- the controller interface --------------------------------------
    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        # setpoint is ignored — the calibration scripts its own motion.
        p = self.params
        x = measurement
        v = self._update_velocity(measurement, dt)
        self._t_phase += dt

        if not self.running:
            return 0.0

        if x < -1.0 or x > self.stroke + 1.0:  # hard end-stop guard
            self._abort("position out of range")
            return 0.0

        park = p.park_frac * self.stroke

        if self._phase is _Phase.PARK:
            if abs(x - park) < p.park_tol and abs(v) < p.settle_v:
                self._enter(_Phase.DEADBAND)
                self._ramp = 0.0
                self._x_ramp0 = x
                return 0.0
            if self._t_phase > p.phase_timeout:
                self._abort("could not reach park position")
                return 0.0
            return self._approach(park, x)

        if self._phase is _Phase.DEADBAND:
            self._ramp += p.ramp_rate * dt
            self.progress = f"Finding deadband… PWM {self._ramp:.1f}%"
            self._db_pwm.append(self._ramp)
            self._db_v.append(v)
            if v > p.v_fit_hi:  # clearly moving — fit the rise back to zero velocity
                self._deadband = self._estimate_deadband()
                self._enter(_Phase.SETTLE)
                return 0.0
            if self._ramp > p.ramp_pwm_cap:
                self._abort("no motion detected within PWM cap")
                return 0.0
            if not self._in_window(x):
                self._abort("left safe window during deadband ramp")
                return 0.0
            return self._ramp

        if self._phase is _Phase.SETTLE:
            self.progress = "Settling…"
            if abs(v) < p.settle_v or self._t_phase > 2.0:
                self._enter(_Phase.STEP)
                self._x_step0 = x
                self._step_t = []
                self._step_v = []
            return 0.0

        if self._phase is _Phase.STEP:
            self.progress = f"Step test… v ≈ {v:5.0f} mm/s"
            self._step_t.append(self._t_phase)
            self._step_v.append(v)
            if not self._in_window(x):
                self._finish_step() if self._t_phase > 0.4 else self._abort(
                    "left safe window during step"
                )
                return 0.0
            if self._t_phase >= p.step_time:
                self._finish_step()
                return 0.0
            return p.step_pwm

        if self._phase is _Phase.RETURN:
            self.progress = "Returning to park…"
            if (abs(x - park) < p.park_tol and abs(v) < p.settle_v) or \
                    self._t_phase > p.phase_timeout:
                self._phase = _Phase.DONE
                self.progress = "Done."
                return 0.0
            return self._approach(park, x)

        return 0.0

    # --- helpers -------------------------------------------------------
    def _update_velocity(self, measurement: float, dt: float) -> float:
        if self._meas_prev is None:
            raw = 0.0
        else:
            raw = (measurement - self._meas_prev) / dt
        self._meas_prev = measurement
        alpha = dt / (self.params.tau_v + dt)
        self._v_est += (raw - self._v_est) * alpha
        return self._v_est

    def _approach(self, target: float, x: float) -> float:
        p = self.params
        u = p.park_kp * (target - x)
        return clamp_pwm(max(-p.park_pwm_cap, min(p.park_pwm_cap, u)))

    def _in_window(self, x: float) -> bool:
        return (self.params.safe_lo_frac * self.stroke
                <= x <= self.params.safe_hi_frac * self.stroke)

    def _estimate_deadband(self) -> float:
        """Deadband = the PWM where velocity extrapolates to zero. Fitting the rising
        (PWM, velocity) points and taking the x-intercept removes the bias that a plain
        onset threshold suffers from valve lag and the ramp speed."""
        pwm = np.asarray(self._db_pwm)
        v = np.asarray(self._db_v)
        band = (v >= self.params.v_fit_lo) & (v <= self.params.v_fit_hi)
        if band.sum() >= 2:
            slope, intercept = np.polyfit(pwm[band], v[band], 1)
            if slope > 1.0e-6:
                return float(min(max(-intercept / slope, 0.0), self.params.ramp_pwm_cap))
        moving = np.flatnonzero(v >= self.params.v_fit_lo)
        return float(pwm[moving[0]]) if moving.size else self._ramp

    def _enter(self, phase: _Phase) -> None:
        self._phase = phase
        self._t_phase = 0.0

    def _abort(self, msg: str) -> None:
        self._phase = _Phase.ABORT
        self.result = CalibrationResult(False, msg)
        self.progress = f"Aborted: {msg}"

    def _finish_step(self) -> None:
        p = self.params
        t = np.asarray(self._step_t)
        v = np.asarray(self._step_v)
        if t.size < 5:
            self._abort("not enough step data")
            return
        n_tail = max(3, int(0.3 * t.size))
        v_ss = float(np.mean(v[-n_tail:]))  # steady velocity over the step's tail
        if v_ss < 1.0:
            self._abort("no measurable velocity in step")
            return
        # Lag tau = time to reach 63% of the steady velocity.
        target = 0.63 * v_ss
        reached = np.flatnonzero(v >= target)
        tau = float(t[reached[0]]) if reached.size else p.tau_v
        tau = min(max(tau, 0.02), 0.5)
        u_eff = max(p.step_pwm - self._deadband, 1.0)
        kv = v_ss / u_eff
        kp, ki, kd = simc_pid(kv, tau, p.tau_c)
        self.result = CalibrationResult(
            True, "Calibration complete.",
            deadband=self._deadband, kv=kv, tau=tau, kp=kp, ki=ki, kd=kd,
        )
        self._enter(_Phase.RETURN)
