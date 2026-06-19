"""Manual passthrough controller.

Ignores the setpoint and the measurement and just returns whatever PWM the user
dialled in on the Manual tab. Wrapping manual mode as a :class:`Controller` lets the
simulation loop treat every mode the same way.
"""

from __future__ import annotations

from .base import Controller, clamp_pwm


class ManualController(Controller):
    def __init__(self) -> None:
        self._command = 0.0

    def set_command(self, pwm: float) -> None:
        self._command = clamp_pwm(pwm)

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        return self._command

    def reset(self) -> None:
        self._command = 0.0
