"""Model-predictive control (MPC), hand-rolled in numpy.

Standard linear MPC: predict the cylinder's motion over a short horizon with a linear
model, choose the PWM sequence that minimises a tracking + effort cost subject to the
+/-100 PWM limit, apply only the first move, then re-solve next step (receding horizon).

The prediction model is the *linear core* of the plant — position/velocity driven by
PWM — with the nonlinearities (deadband, asymmetry, Coulomb friction) deliberately left
out: MPC, like any real controller, works from an approximate model and relies on
re-solving every step to stay on track. The **external load is modelled explicitly**,
so a known load is fed forward — the optimiser already knows it must hold against the
weight and picks the PWM that does so. That is the headline contrast with PID, which
sees the load only as a disturbance to be mopped up after the fact.

Model (state [x, v], input u = PWM %, load d as a known constant force):

    dx/dt = v
    dv/dt = (k_hyd*(Kv*u - v) - b*v - d) / m,    Kv = vmax / 100

discretised exactly (zero-order hold) over the MPC sample time Ts — the structure is an
integrator plus a first-order lag, so the ZOH matrices have a clean closed form (no
matrix exponential needed).

The condensed QP

    min_U  Q*||P(U) - r||^2 + R*||dU||^2     s.t.  -100 <= U <= 100

(P = predicted positions over the horizon, r = setpoint, dU = the step-to-step change
in PWM, with dU[0] = u_0 - u_prev) is solved with a projected, FISTA-accelerated
gradient method — small and transparent, no external solver. Penalising the input
*rate* dU rather than its absolute value (move suppression) is what lets the controller
hold a large steady command "for free" — essential for the known-load feed-forward to
actually hold position rather than droop.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .base import PWM_LIMIT, Controller, clamp_pwm


@dataclass
class MPCParams:
    horizon: int = 40    # N steps of preview
    ts: float = 0.02     # MPC sample time (s) — re-solve at 50 Hz
    q: float = 1.0       # tracking weight
    r: float = 2.0       # move-suppression weight (penalises step-to-step PWM change)
    tau_v: float = 0.05  # velocity-estimate low-pass time constant (s)
    iters: int = 80      # projected-gradient iterations per solve


@dataclass(frozen=True)
class MPCModel:
    """The plant's linear core, as the controller believes it to be."""

    m: float      # effective mass
    k_hyd: float  # hydraulic drive gain
    b: float      # viscous friction
    vmax: float   # mm/s at full PWM (nominal; the extend value)


class MPCController(Controller):
    def __init__(self, model: MPCModel, params: MPCParams | None = None) -> None:
        self.model = model
        self.params = params or MPCParams()
        self._load = 0.0
        self._build()
        self.reset()

    # --- configuration (live) ------------------------------------------
    def set_load(self, force: float) -> None:
        """The known external load fed forward in the prediction."""
        self._load = force

    def set_weights(self, q: float, r: float) -> None:
        self.params.q = max(0.0, q)
        self.params.r = max(0.0, r)
        self._build()

    def set_horizon(self, n: int) -> None:
        self.params.horizon = max(1, int(n))
        self._build()
        self._warm = np.zeros(self.params.horizon)

    def reset(self) -> None:
        self._meas_prev: float | None = None
        self._v_est = 0.0
        self._u_hold = 0.0
        self._since = 1.0e9  # force a solve on the first call
        self._warm = np.zeros(self.params.horizon)

    # --- model discretisation + QP build -------------------------------
    def _discretize(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        mo = self.model
        a = (mo.k_hyd + mo.b) / mo.m
        kv = mo.vmax / 100.0
        bu = mo.k_hyd * kv / mo.m  # dv/dt coefficient on u
        bd = -1.0 / mo.m           # dv/dt coefficient on the load d
        ts = self.params.ts
        phi = np.exp(-a * ts)
        s = ts - (1.0 - phi) / a
        ad = np.array([[1.0, (1.0 - phi) / a],
                       [0.0, phi]])
        bd_vec = np.array([(bu / a) * s, (bu / a) * (1.0 - phi)])
        ed_vec = np.array([(bd / a) * s, (bd / a) * (1.0 - phi)])
        return ad, bd_vec, ed_vec

    def _build(self) -> None:
        n = self.params.horizon
        ad, bd, ed = self._discretize()
        c = np.array([1.0, 0.0])

        # C A^m B and C A^m E for m = 0..n-1 (the impulse responses).
        cab = np.empty(n)
        cae = np.empty(n)
        am = np.eye(2)
        for m in range(n):
            cab[m] = c @ (am @ bd)
            cae[m] = c @ (am @ ed)
            am = ad @ am

        sx = np.zeros((n, 2))   # free response from the initial state
        su = np.zeros((n, n))   # forced response from U (lower-triangular Toeplitz)
        sd = np.zeros(n)        # forced response per unit of (constant) load
        ak = np.eye(2)
        gsum = 0.0
        for k in range(1, n + 1):
            ak = ad @ ak
            sx[k - 1, :] = c @ ak
            su[k - 1, :k] = cab[k - 1::-1]   # [C A^{k-1}B, ..., C A^0 B]
            gsum += cae[k - 1]               # sum_{m=0}^{k-1} C A^m E
            sd[k - 1] = gsum

        # First-difference matrix D so that dU = D @ U - u_prev*e0 (move suppression).
        d_mat = np.eye(n) - np.eye(n, k=-1)

        self._sx, self._su, self._sd = sx, su, sd
        q, r = self.params.q, self.params.r
        self._hess = 2.0 * (q * su.T @ su + r * (d_mat.T @ d_mat))
        self._step = 1.0 / float(np.linalg.eigvalsh(self._hess)[-1])  # 1/Lipschitz
        self._q, self._r = q, r

    # --- solve ----------------------------------------------------------
    def _solve(self, x0: np.ndarray, r_ref: float, u_prev: float) -> float:
        # Free-response tracking error: where the positions go with U = 0.
        c = self._sx @ x0 + self._sd * self._load - r_ref
        f = 2.0 * self._q * (self._su.T @ c)
        # Move-suppression couples the first move to the previously applied command:
        # the gradient of R*||D U - u_prev*e0||^2 contributes -2*R*u_prev to f[0].
        f[0] -= 2.0 * self._r * u_prev

        u = self._warm
        y = u.copy()
        t = 1.0
        for _ in range(self.params.iters):
            grad = self._hess @ y + f
            u_new = np.clip(y - self._step * grad, -PWM_LIMIT, PWM_LIMIT)
            t_new = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * t * t))
            y = u_new + ((t - 1.0) / t_new) * (u_new - u)
            u, t = u_new, t_new

        self._warm = u
        return float(u[0])

    def compute(self, setpoint: float, measurement: float, dt: float) -> float:
        # Estimate velocity from the measurement (low-pass filtered) — the model
        # state is [position, velocity] but only position is measured.
        if self._meas_prev is None:
            raw_v = 0.0
        else:
            raw_v = (measurement - self._meas_prev) / dt
        self._meas_prev = measurement
        alpha = dt / (self.params.tau_v + dt)
        self._v_est += (raw_v - self._v_est) * alpha

        # Re-solve at the MPC rate; hold the last command in between.
        self._since += dt
        if self._since >= self.params.ts:
            self._since = 0.0
            x0 = np.array([measurement, self._v_est])
            self._u_hold = self._solve(x0, setpoint, self._u_hold)
        return clamp_pwm(self._u_hold)
