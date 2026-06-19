"""The PWM proportional valve.

Turns a PWM command in [-100, +100] into a signed spool position. Three effects,
applied in order:

  1. **saturation** — the command is clamped to +/-100;
  2. **deadband** — below a threshold the spool stays shut (no flow); past it the
     command is re-scaled so motion starts at the deadband edge and reaches full
     opening at +/-100;
  3. **first-order lag** — the spool cannot snap instantly; it chases the
     post-deadband command with time constant ``tau`` (seconds).

The returned spool position is itself a signed percentage in [-100, +100]; the
plant turns that into flow and motion.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ValveParams:
    deadband: float = 8.0  # % PWM with no flow on either side of zero
    tau: float = 0.04      # spool lag time constant (s)


def _sign(v: float) -> float:
    return 1.0 if v > 0 else (-1.0 if v < 0 else 0.0)


def apply_deadband(pwm: float, deadband: float) -> float:
    """Zero inside +/-deadband; linearly re-scaled to reach +/-100 past it."""
    mag = abs(pwm)
    if mag <= deadband:
        return 0.0
    span = 100.0 - deadband
    return _sign(pwm) * (mag - deadband) / span * 100.0


class Valve:
    """Stateful PWM proportional valve. Hold one per simulation."""

    def __init__(self, params: ValveParams | None = None) -> None:
        self.params = params or ValveParams()
        self.spool = 0.0  # current spool position, signed %

    def reset(self) -> None:
        self.spool = 0.0

    def update(self, pwm: float, dt: float) -> float:
        """Advance the spool one step toward the (saturated, deadbanded) command."""
        pwm = max(-100.0, min(100.0, pwm))  # saturation
        target = apply_deadband(pwm, self.params.deadband)  # deadband
        # Exact discrete first-order lag (backward Euler) — stable for any dt.
        alpha = dt / (self.params.tau + dt)
        self.spool += (target - self.spool) * alpha
        return self.spool
