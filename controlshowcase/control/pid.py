"""PID position controller.

A textbook PID with the practical bits that make it usable on this plant:

  * **output clamping** to the +/-100 PWM limit;
  * **anti-windup by conditional integration** — the integral only accumulates when
    doing so wouldn't push an already-saturated output further into saturation, so
    sitting far from the setpoint doesn't wind the integral up into a big overshoot;
  * **derivative on the measurement, low-pass filtered** — differentiating position
    instead of error avoids a "derivative kick" when the setpoint steps, and the
    filter stops the D term amplifying noise.

Gains can be changed live (that is the whole point of the PID tab); the internal
state (integral, filtered derivative) carries across edits so tuning feels smooth.

A note on units: error is in mm, output is in PWM %, so ``kp`` has units %/mm. With
this plant a full +/-100% command gives roughly +/-150-200 mm/s, so gains on the
order of kp~0.5, ki~0.2, kd~0.05 are a sensible starting point.
"""

from __future__ import annotations

from dataclasses import dataclass

from .base import PWM_LIMIT, Controller, clamp_pwm


@dataclass
class PIDGains:
    kp: float = 0.5  # %/mm
    ki: float = 0.2  # %/(mm·s)
    kd: float = 0.05  # %/(mm/s)


class PIDController(Controller):
    def __init__(self, gains: PIDGains | None = None, tau_d: float = 0.05) -> None:
        self.gains = gains or PIDGains()
        self.tau_d = tau_d  # derivative low-pass time constant (s)
        self.reset()

    def reset(self) -> None:
        self._integral = 0.0
        self._meas_prev: float | None = None
        self._deriv = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        self.gains = PIDGains(kp, ki, kd)

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        g = self.gains
        error = setpoint - measurement

        # Derivative on measurement (note the sign), low-pass filtered.
        if self._meas_prev is None:
            raw_deriv = 0.0
        else:
            raw_deriv = (measurement - self._meas_prev) / dt
        self._meas_prev = measurement
        alpha = dt / (self.tau_d + dt)
        self._deriv += (raw_deriv - self._deriv) * alpha
        d_term = -g.kd * self._deriv

        p_term = g.kp * error

        # Tentative output with the integral as it stands.
        output = p_term + g.ki * self._integral + d_term

        # Conditional integration: skip it only when it would deepen saturation.
        winding_up = (output >= PWM_LIMIT and error > 0) or (output <= -PWM_LIMIT and error < 0)
        if not winding_up:
            self._integral += error * dt
            output = p_term + g.ki * self._integral + d_term

        return clamp_pwm(output)
