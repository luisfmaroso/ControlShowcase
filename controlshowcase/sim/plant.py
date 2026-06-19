"""The hydraulic cylinder (the plant).

A bidirectional cylinder modelled as a **second-order system** — state is position
``x`` (mm) and velocity ``v`` (mm/s) — driven by the valve spool position. The chain
each step:

    spool (%)   ->  commanded velocity  v_cmd = (spool/100) * vmax(direction)
    v_cmd, v    ->  hydraulic drive force  F = k_hyd * (v_cmd - v)
    force balance:  m * dv/dt = F_drive - F_viscous - F_coulomb - load
    integrate:      v += a*dt ;  x += v*dt ;  clamp to [0, stroke]

The drive term ``k_hyd * (v_cmd - v)`` models a flow source behind a compressible
fluid: the further actual velocity lags the commanded velocity, the more pressure
(force) builds to close the gap. That gives a stable first-order velocity response
and hence a second-order position response.

Nonlinearities — the reason a single fixed PID can't be perfect everywhere, and why
MPC's preview can help:

  * **area asymmetry** — extend and retract use different piston areas, so the same
    spool gives different top speeds in each direction;
  * **Coulomb friction** — a velocity-independent drag (smoothed with ``tanh`` so it
    doesn't chatter at v ~= 0), which leaves a steady-state error under plain P;
  * **end-stops** — the rod cannot pass 0 or the full stroke.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PlantParams:
    stroke: float = 500.0        # mm, usable travel [0, stroke]
    mass: float = 1.0            # effective moving mass (abstract, consistent units)
    vmax_extend: float = 180.0   # mm/s at full spool, extending (x increasing)
    vmax_retract: float = 240.0  # mm/s at full spool, retracting (smaller annulus -> faster)
    k_hyd: float = 12.0          # hydraulic drive gain (force per mm/s of velocity error)
    b_visc: float = 2.0          # viscous friction (force per mm/s)
    f_coulomb: float = 6.0       # Coulomb friction force magnitude
    v_eps: float = 5.0           # mm/s; smooths Coulomb friction around zero velocity
    load: float = 0.0            # constant external force; positive opposes extending


class Plant:
    """Stateful second-order cylinder. Hold one per simulation."""

    def __init__(self, params: PlantParams | None = None, x0: float = 0.0) -> None:
        self.params = params or PlantParams()
        self.x = x0
        self.v = 0.0

    def reset(self, x0: float = 0.0) -> None:
        self.x = x0
        self.v = 0.0

    def step(self, spool: float, dt: float) -> float:
        """Advance position/velocity one step given the valve spool position (%)."""
        p = self.params

        # Commanded velocity from flow, asymmetric by direction of travel.
        vmax = p.vmax_extend if spool >= 0 else p.vmax_retract
        v_cmd = (spool / 100.0) * vmax

        f_drive = p.k_hyd * (v_cmd - self.v)
        f_visc = p.b_visc * self.v
        f_coulomb = p.f_coulomb * math.tanh(self.v / p.v_eps)
        f_net = f_drive - f_visc - f_coulomb - p.load

        a = f_net / p.mass
        self.v += a * dt
        self.x += self.v * dt

        # End-stops: clamp the stroke and kill any velocity pushing into the stop.
        if self.x <= 0.0:
            self.x = 0.0
            if self.v < 0.0:
                self.v = 0.0
        elif self.x >= p.stroke:
            self.x = p.stroke
            if self.v > 0.0:
                self.v = 0.0
        return self.x

    @property
    def position_fraction(self) -> float:
        """Position as a 0..1 stroke fraction (for the cylinder view)."""
        return self.x / self.params.stroke
