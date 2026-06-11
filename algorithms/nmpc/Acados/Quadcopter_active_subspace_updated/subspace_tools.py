"""Projectors, horizon cost and terminal feedback for the updated
active-subspace NMPC (self-contained: uses this folder's Quadcopter model).

Why this exists
---------------
The first version built T1 with ``generate_block_identity(N*nu, nv)``: that is
a *corner* identity, not a block-diagonal projector.  With nv=3 the active
variable v only touched motors 1-3 at stage 0 and every later stage was frozen
to ``mu * inactive`` -- one scalar scaling the whole tail of the horizon.  The
reduced problem structurally cannot represent good solutions (suboptimality)
and the solver wastes iterations fighting it (slow convergence).

Provided here:

* ``build_hessian_projector``  -- dense T1 from the leading eigenvectors of
  d^2 J / dU^2 at hover: directions where the horizon cost actually varies,
  spread over all stages.  Deterministic, no sampling needed.
* ``build_block_diagonal_projector`` -- a *true* per-stage block-diagonal T1
  (what the corner identity was probably meant to be): every stage gets the
  same nv_per_stage local input directions.
* ``build_corner_identity_projector`` -- the old behaviour, kept for A/B
  comparison.
* ``HorizonCost`` -- exact J(x0, U) evaluator (RK4), used by the fallback rule
  of Algorithm 1 and by the Hessian projector.
* ``TerminalFeedback`` -- clipped LQR kappa(x) about hover for the shift
  append (Algorithm 1, line 9), instead of duplicating the last input.
"""

import casadi as ca
import numpy as np
from scipy.linalg import expm, null_space, solve_discrete_are

from Quadcopter import (
    Q_diag,
    R_weight,
    nu_phy,
    nx,
    quadcopter_dynamics,
    u_hover_per_motor,
)
from utils import reference_trajectory


# ---------------------------------------------------------------------------
# Exact horizon cost J(x0, U)
# ---------------------------------------------------------------------------

def build_horizon_cost_function(n_horizon, ts, rk4_substeps=10):
    """CasADi Function J(x0, U_stack, ref_stack) matching the acados cost."""
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu_phy)
    f = ca.Function("f", [x, u], [quadcopter_dynamics(x, u)])

    h = ts / rk4_substeps

    def rk4_step(xk, uk):
        k1 = f(xk, uk)
        k2 = f(xk + 0.5 * h * k1, uk)
        k3 = f(xk + 0.5 * h * k2, uk)
        k4 = f(xk + h * k3, uk)
        return xk + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    q = ca.DM(np.diag(Q_diag))
    r = ca.DM(R_weight * np.eye(nu_phy))
    u_hover = ca.DM(np.full(nu_phy, u_hover_per_motor))

    x0 = ca.SX.sym("x0", nx)
    u_stack = ca.SX.sym("u_stack", n_horizon * nu_phy)
    ref_stack = ca.SX.sym("ref_stack", (n_horizon + 1) * nx)

    cost = ca.SX(0)
    xk = x0
    for j in range(n_horizon):
        uj = u_stack[j * nu_phy:(j + 1) * nu_phy]
        refj = ref_stack[j * nx:(j + 1) * nx]
        ex = xk - refj
        eu = uj - u_hover
        cost = cost + 0.5 * (ex.T @ q @ ex + eu.T @ r @ eu)
        for _ in range(rk4_substeps):
            xk = rk4_step(xk, uj)
    refN = ref_stack[n_horizon * nx:(n_horizon + 1) * nx]
    exN = xk - refN
    cost = cost + exN.T @ q @ exN

    return ca.Function("J_horizon", [x0, u_stack, ref_stack], [cost])


def make_reference_stack(t0, n_horizon, ts):
    return np.concatenate(
        [reference_trajectory(t0 + j * ts) for j in range(n_horizon + 1)]
    )


class HorizonCost:
    def __init__(self, n_horizon, ts, rk4_substeps=10):
        self.n_horizon = n_horizon
        self.ts = ts
        self.fun = build_horizon_cost_function(n_horizon, ts, rk4_substeps)

    def __call__(self, x0, u_stack, t0):
        ref_stack = make_reference_stack(t0, self.n_horizon, self.ts)
        return float(self.fun(x0, u_stack, ref_stack))


# ---------------------------------------------------------------------------
# Nominal program P(x0)  (Algorithm 1, line 1)
# ---------------------------------------------------------------------------

def solve_nominal_ocp(horizon_cost_fun, x0, t0, n_horizon, ts, lb_u, ub_u, u_init=None):
    """Solve the nominal single-shooting OCP P(x0) for the initial feasible
    candidate u~_0 (thesis eq. (2.3), Algorithm 1 line 1) with IPOPT."""
    n_stack = n_horizon * nu_phy
    U = ca.SX.sym("U", n_stack)
    ref_stack = make_reference_stack(t0, n_horizon, ts)
    nlp = {"x": U, "f": horizon_cost_fun(ca.DM(np.asarray(x0, float)), U, ca.DM(ref_stack))}
    solver = ca.nlpsol("nominal_ocp", "ipopt", nlp, {
        "print_time": 0, "ipopt": {"print_level": 0, "sb": "yes", "max_iter": 300},
    })
    if u_init is None:
        u_init = np.full(n_stack, u_hover_per_motor)
    sol = solver(x0=u_init, lbx=np.tile(lb_u, n_horizon), ubx=np.tile(ub_u, n_horizon))
    return np.array(sol["x"]).flatten()


# ---------------------------------------------------------------------------
# Projectors:  T1 (n_stacked x nv, orthonormal columns), T2 = complement
# ---------------------------------------------------------------------------

def _finish_projector(T1, kind):
    q, _ = np.linalg.qr(T1)
    T1 = q[:, : T1.shape[1]]
    T2 = null_space(T1.T)
    print(
        f"[projector={kind}] T1 {T1.shape}, "
        f"||T1^T T1 - I|| = {np.linalg.norm(T1.T @ T1 - np.eye(T1.shape[1])):.2e}, "
        f"||T1^T T2|| = {np.linalg.norm(T1.T @ T2):.2e}"
    )
    return T1, T2


def build_corner_identity_projector(n_stacked, nv):
    """Old behaviour (first nv canonical directions). Kept for comparison."""
    T1 = np.zeros((n_stacked, nv))
    T1[:nv, :nv] = np.eye(nv)
    return _finish_projector(T1, "corner-identity")


def build_block_diagonal_projector(n_horizon, nv_per_stage):
    """True block-diagonal T1: the same nv_per_stage input directions at
    every stage, so every stage of the horizon keeps direct control authority.

    Local (per-stage) directions, orthonormal in R^4:
      collective  [1, 1, 1, 1]/2          -> total thrust
      roll-ish    [-1, -1, 1, 1]/2        -> Mx
      pitch-ish   [1, -1, -1, 1]/2        -> My
      yaw-ish     [-1, 1, -1, 1]/2        -> Mz
    """
    local = np.array(
        [
            [1.0, 1.0, 1.0, 1.0],
            [-1.0, -1.0, 1.0, 1.0],
            [1.0, -1.0, -1.0, 1.0],
            [-1.0, 1.0, -1.0, 1.0],
        ]
    ).T / 2.0
    if not 1 <= nv_per_stage <= nu_phy:
        raise ValueError(f"nv_per_stage must be in [1, {nu_phy}]")
    block = local[:, :nv_per_stage]
    T1 = np.kron(np.eye(n_horizon), block)
    return _finish_projector(T1, f"block-diagonal({nv_per_stage}/stage)")


def build_hessian_projector(horizon_cost_fun, n_horizon, ts, nv, x0, t0=0.0):
    """Dense T1 from the leading eigenvectors of d^2 J / dU^2 at hover."""
    n_stacked = n_horizon * nu_phy
    u_sym = ca.SX.sym("U", n_stacked)
    ref0 = make_reference_stack(t0, n_horizon, ts)
    cost = horizon_cost_fun(ca.DM(np.asarray(x0, dtype=float)), u_sym, ca.DM(ref0))
    hess_fun = ca.Function("H", [u_sym], [ca.hessian(cost, u_sym)[0]])
    H = np.array(hess_fun(np.full(n_stacked, u_hover_per_motor)))
    H = 0.5 * (H + H.T)
    eigvals, eigvecs = np.linalg.eigh(H)
    order = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]
    captured = float(np.abs(eigvals[:nv]).sum() / np.abs(eigvals).sum())
    print(f"[projector=hessian] captured curvature fraction (nv={nv}): {captured:.4f}")
    return _finish_projector(eigvecs[:, :nv], "hessian")


# ---------------------------------------------------------------------------
# Terminal feedback kappa(x) for the candidate shift (Algorithm 1, line 9)
# ---------------------------------------------------------------------------

class TerminalFeedback:
    """Clipped discrete LQR about hover; appended input stays feasible and
    steers the predicted terminal state to the reference."""

    # qw is uncontrollable at hover (unit-quaternion constraint), so the LQR
    # uses the standard 12-dim error state [pos, qx, qy, qz, vel, omega].
    _ERR_IDX = [0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12]

    def __init__(self, ts, lb_u, ub_u):
        self.lb_u = np.asarray(lb_u, dtype=float)
        self.ub_u = np.asarray(ub_u, dtype=float)
        self.u_eq = np.full(nu_phy, u_hover_per_motor)

        x = ca.SX.sym("x", nx)
        u = ca.SX.sym("u", nu_phy)
        f = quadcopter_dynamics(x, u)
        a_fun = ca.Function("A", [x, u], [ca.jacobian(f, x)])
        b_fun = ca.Function("B", [x, u], [ca.jacobian(f, u)])
        x_eq = np.zeros(nx)
        x_eq[3] = 1.0  # hover attitude: unit quaternion qw = 1
        idx = np.array(self._ERR_IDX)
        A = np.array(a_fun(x_eq, self.u_eq))[np.ix_(idx, idx)]
        B = np.array(b_fun(x_eq, self.u_eq))[idx, :]

        n_err = len(idx)
        M = np.zeros((n_err + nu_phy, n_err + nu_phy))
        M[:n_err, :n_err] = A
        M[:n_err, n_err:] = B
        Md = expm(M * ts)
        Ad, Bd = Md[:n_err, :n_err], Md[:n_err, n_err:]

        q = np.diag(np.asarray(Q_diag, dtype=float)[idx])
        r = R_weight * np.eye(nu_phy)
        P = solve_discrete_are(Ad, Bd, q, r)
        self.K = np.linalg.solve(r + Bd.T @ P @ Bd, Bd.T @ P @ Ad)

    def __call__(self, x, x_ref):
        err = (np.asarray(x) - np.asarray(x_ref))[self._ERR_IDX]
        u = self.u_eq - self.K @ err
        return np.clip(u, self.lb_u, self.ub_u)
