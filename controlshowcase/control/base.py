"""The controller interface.

Every control mode implements the same tiny contract: given the current setpoint,
the measured position, and the time step ``dt``, return a PWM command in
[-100, +100]. Controllers know nothing about Qt or the plant — they only see numbers
— which keeps them easy to test and to swap at runtime, and makes the simulation
loop uniform (every mode is just "compute a PWM from the loop state").
"""

from __future__ import annotations

from abc import ABC, abstractmethod

PWM_LIMIT = 100.0


def clamp_pwm(pwm: float) -> float:
    """Clamp a command to the valve's PWM range."""
    return max(-PWM_LIMIT, min(PWM_LIMIT, pwm))


class Controller(ABC):
    @abstractmethod
    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        """Return the PWM command for this step, in [-100, +100]."""

    def reset(self) -> None:
        """Drop any internal state. Called when the mode becomes active."""
