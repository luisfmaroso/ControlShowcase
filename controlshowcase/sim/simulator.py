"""The simulator — the fixed-step loop and the single source of truth.

Owns the valve, the plant, the current setpoint, and a rolling history of the loop.
Each :meth:`step` takes the PWM the controller asked for, advances the valve and the
plant by one fixed time step, records the sample, and returns it. The UI reads this
object (position, history); it never writes to the plant directly.
"""

from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass

import numpy as np

from .plant import Plant, PlantParams
from .valve import Valve, ValveParams

DEFAULT_DT = 1.0 / 200.0  # 200 Hz fixed simulation step
DEFAULT_WINDOW_S = 20.0   # rolling history length kept for the plot

_CHANNELS = ("t", "setpoint", "position", "measured", "pwm", "error")


@dataclass
class SimSample:
    t: float
    setpoint: float
    position: float   # true cylinder position (drives the animation)
    measured: float   # the noisy sensor reading the controller acts on
    pwm: float
    error: float      # setpoint - measured (the error the controller sees)


class Simulator:
    def __init__(
        self,
        dt: float = DEFAULT_DT,
        window_s: float = DEFAULT_WINDOW_S,
        valve_params: ValveParams | None = None,
        plant_params: PlantParams | None = None,
    ) -> None:
        self.dt = dt
        self.valve = Valve(valve_params)
        self.plant = Plant(plant_params)
        self.setpoint = 0.0
        self.t = 0.0
        self.noise_std = 0.0          # sensor noise std-dev (mm); 0 = perfect sensor
        self._last_measured: float | None = None
        maxlen = max(2, int(round(window_s / dt)))
        self._hist: deque[SimSample] = deque(maxlen=maxlen)

    # --- control --------------------------------------------------------
    def reset(self, x0: float = 0.0) -> None:
        self.valve.reset()
        self.plant.reset(x0)
        self.setpoint = x0
        self.t = 0.0
        self._last_measured = None
        self._hist.clear()

    def set_setpoint(self, mm: float) -> None:
        self.setpoint = max(0.0, min(self.plant.params.stroke, mm))

    def set_noise_std(self, mm: float) -> None:
        self.noise_std = max(0.0, mm)

    def measure(self) -> float:
        """A sensor reading of the current position: the true position plus Gaussian
        noise. The controller acts on this, not on the true state."""
        noise = random.gauss(0.0, self.noise_std) if self.noise_std > 0.0 else 0.0
        self._last_measured = self.plant.x + noise
        return self._last_measured

    def step(self, pwm: float) -> SimSample:
        # Textbook discrete control: at time t we have state x(t), its measurement
        # y(t), and the command u(t); we record that sample, then apply u(t) to
        # advance to x(t+dt). Recording before advancing keeps the measurement and the
        # true position aligned at the same instant (so with no noise they are equal).
        measured = self._last_measured if self._last_measured is not None else self.plant.x
        self._last_measured = None

        sample = SimSample(
            t=self.t,
            setpoint=self.setpoint,
            position=self.plant.x,
            measured=measured,
            pwm=pwm,
            error=self.setpoint - measured,
        )
        self._hist.append(sample)

        spool = self.valve.update(pwm, self.dt)
        self.plant.step(spool, self.dt)
        self.t += self.dt
        return sample

    # --- views for the UI ----------------------------------------------
    @property
    def position(self) -> float:
        return self.plant.x

    @property
    def position_fraction(self) -> float:
        return self.plant.position_fraction

    @property
    def setpoint_fraction(self) -> float:
        return self.setpoint / self.plant.params.stroke

    def history(self) -> dict[str, np.ndarray]:
        """Rolling history as numpy arrays keyed by channel (for plotting)."""
        n = len(self._hist)
        if n == 0:
            empty = np.empty(0)
            return {k: empty for k in _CHANNELS}
        return {
            "t": np.fromiter((s.t for s in self._hist), float, n),
            "setpoint": np.fromiter((s.setpoint for s in self._hist), float, n),
            "position": np.fromiter((s.position for s in self._hist), float, n),
            "measured": np.fromiter((s.measured for s in self._hist), float, n),
            "pwm": np.fromiter((s.pwm for s in self._hist), float, n),
            "error": np.fromiter((s.error for s in self._hist), float, n),
        }


# --- headless sanity checks --------------------------------------------------
# Run `python -m controlshowcase.sim.simulator` to exercise the plant physics
# without the GUI. Doubles as the Phase 1 verification.
if __name__ == "__main__":
    def _run(pwm: float, seconds: float, sim: Simulator | None = None) -> Simulator:
        sim = sim or Simulator()
        for _ in range(int(round(seconds / sim.dt))):
            sim.step(pwm)
        return sim

    # 1) Deadband: a PWM inside the deadband produces no motion.
    s = _run(5.0, 2.0)
    assert abs(s.position) < 1e-6, f"deadband leak: {s.position}"

    # 2) Open-loop ramp: a steady extend command drives the rod out (toward the stop).
    s = _run(60.0, 5.0)
    assert s.position > 50.0, s.position
    assert s.position <= s.plant.params.stroke + 1e-6

    # 3) Asymmetry: retract is faster than extend for an equal |PWM| over the same time.
    ext = Simulator()
    for _ in range(40):  # 0.2 s
        ext.step(50.0)
    ret = Simulator()
    ret.plant.reset(500.0)
    for _ in range(40):
        ret.step(-50.0)
    ext_speed = ext.plant.x / 0.2
    ret_speed = (500.0 - ret.plant.x) / 0.2
    assert ret_speed > ext_speed, (ext_speed, ret_speed)

    # 4) End-stop: a long full-extend command saturates exactly at the stroke, v = 0.
    s = _run(100.0, 10.0)
    assert abs(s.position - s.plant.params.stroke) < 1e-6, s.position
    assert abs(s.plant.v) < 1e-9, s.plant.v

    print("plant sanity checks passed\n")
    print(f"asymmetry: extend ~{ext_speed:.0f} mm/s vs retract ~{ret_speed:.0f} mm/s")

    print("\nopen-loop step response to PWM=70 (extend):")
    sim = Simulator()
    sim.set_setpoint(300.0)
    every = int(round(0.5 / sim.dt))
    n = int(round(3.0 / sim.dt))
    for i in range(n):
        sim.step(70.0)
        if i % every == 0:
            print(f"  t={sim.t:4.2f}s  x={sim.position:6.1f} mm  v={sim.plant.v:7.1f} mm/s")
