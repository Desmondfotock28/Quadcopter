"""Tools backing the active-subspace NMPC algorithm (Algorithm 1 in
`NMPC_active_Subspace.pdf`).

This module provides the three pieces required to match the PDF algorithm
beyond the bare reduced-OCP solve:

* an exact horizon-cost evaluator ``J(x_k, U)`` (Algorithm 1, lines 3-6 -- the
  fallback rule compares two such costs),
* a terminal state-feedback law ``kappa(x)`` (Algorithm 1, line 9 -- the
  recursive-feasibility / stability append), and
* projector generators ``T1, T2`` (PDF eqs. (2.8), (2.10)-(2.11)): ``identity``,
  ``pca`` and ``hessian``.

All cost / dynamics quantities are taken from ``Quadcopter.py`` so the Python
rollout reproduces the acados EXTERNAL cost exactly.
"""

import numpy as np
import casadi as ca
from scipy.linalg import expm, solve_discrete_are, null_space

from Quadcopter import (
    NX_QUAD,
    NU_PHYSICAL,
    Q_DIAG,
    R_WEIGHT,
    U_HOVER_COST,
    U_EQUILIBRIUM,
    quadcopter_continuous_dynamics,
)
from utils import reference_trajectory


# ---------------------------------------------------------------------------
# Exact horizon cost  J(x_k, U)   (PDF Algorithm 1, lines 3, 6; cost J of P)
# ---------------------------------------------------------------------------
#
# J(x_k, U) = sum_{j=0}^{N-1} 0.5 * ( ||x_j - ref_j||_Q^2 + ||u_j - u_hover||_R^2 )
#             + ||x_N - ref_N||_Q^2
#
# This is exactly the acados EXTERNAL stage + terminal cost (see Quadcopter.py),
# *excluding* the small `1e-3 (mu-1)^2` regulariser, which is an artefact of the
# augmented reduced parametrisation and is not part of the physical objective
# J(x, U).  Dropping it lets the reduced candidate and the shifted candidate be
# compared on the same physical cost, as Algorithm 1 requires.
#
# The trajectory x_0..x_N is produced from x_k by an explicit fixed-step RK4
# integration of the disturbance-free prediction dynamics (the same model
# acados integrates internally), so both candidates are scored identically.


def build_horizon_cost_function(n_horizon, ts, rk4_substeps=10):
    """Return a CasADi ``Function`` J(x0, U, ref_stack) -> scalar cost.

    * ``x0``        : initial state (NX_QUAD,)
    * ``U``         : stacked physical inputs (n_horizon * NU_PHYSICAL,)
    * ``ref_stack`` : stacked references ((n_horizon + 1) * NX_QUAD,)
    """
    x = ca.SX.sym("x", NX_QUAD)
    u = ca.SX.sym("u", NU_PHYSICAL)
    f = ca.Function("f", [x, u], [quadcopter_continuous_dynamics(x, u)])

    h = ts / rk4_substeps

    def rk4_step(xk, uk):
        k1 = f(xk, uk)
        k2 = f(xk + 0.5 * h * k1, uk)
        k3 = f(xk + 0.5 * h * k2, uk)
        k4 = f(xk + h * k3, uk)
        return xk + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    q = ca.DM(np.diag(Q_DIAG))
    r = ca.DM(R_WEIGHT * np.eye(NU_PHYSICAL))
    u_hover = ca.DM(np.full(NU_PHYSICAL, U_HOVER_COST))

    x0 = ca.SX.sym("x0", NX_QUAD)
    u_stack = ca.SX.sym("u_stack", n_horizon * NU_PHYSICAL)
    ref_stack = ca.SX.sym("ref_stack", (n_horizon + 1) * NX_QUAD)

    cost = ca.SX(0)
    xk = x0
    for j in range(n_horizon):
        uj = u_stack[j * NU_PHYSICAL:(j + 1) * NU_PHYSICAL]
        refj = ref_stack[j * NX_QUAD:(j + 1) * NX_QUAD]
        ex = xk - refj
        eu = uj - u_hover
        cost = cost + 0.5 * (ex.T @ q @ ex + eu.T @ r @ eu)
        for _ in range(rk4_substeps):
            xk = rk4_step(xk, uj)
    refN = ref_stack[n_horizon * NX_QUAD:(n_horizon + 1) * NX_QUAD]
    exN = xk - refN
    cost = cost + exN.T @ q @ exN

    return ca.Function("J_horizon", [x0, u_stack, ref_stack], [cost])


def make_reference_stack(t0, n_horizon, ts):
    """Stacked references ref_0..ref_N for prediction starting at ``t0``."""
    return np.concatenate(
        [reference_trajectory(t0 + j * ts) for j in range(n_horizon + 1)]
    )


class HorizonCost:
    """Convenience wrapper evaluating J(x0, U) at closed-loop time ``t0``."""

    def __init__(self, n_horizon, ts, rk4_substeps=10):
        self.n_horizon = n_horizon
        self.ts = ts
        self.fun = build_horizon_cost_function(n_horizon, ts, rk4_substeps)

    def __call__(self, x0, u_stack, t0):
        ref_stack = make_reference_stack(t0, self.n_horizon, self.ts)
        return float(self.fun(x0, u_stack, ref_stack))


# ---------------------------------------------------------------------------
# Terminal state feedback  kappa(x)   (PDF Algorithm 1, line 9)
# ---------------------------------------------------------------------------
#
# kappa is a discrete-time LQR law obtained by linearising the disturbance-free
# dynamics about the physical hover equilibrium (x* = 0, u* = U_EQUILIBRIUM) and
# discretising over the sample time Ts.  It is applied as a tracking feedback
#
#     kappa(x) = clip( u_eq - K (x - x_ref),  LB_U, UB_U ),
#
# which keeps the appended terminal input feasible (within the physical motor
# bounds) and steers the predicted terminal state toward the reference, giving
# the recursive-feasibility / stability guarantee of Algorithm 1.


class TerminalFeedback:
    def __init__(self, ts, lb_u, ub_u):
        self.ts = ts
        self.lb_u = np.asarray(lb_u, dtype=float)
        self.ub_u = np.asarray(ub_u, dtype=float)
        self.u_eq = float(U_EQUILIBRIUM)

        # Continuous-time linearisation A = df/dx, B = df/du at (0, u_eq).
        x = ca.SX.sym("x", NX_QUAD)
        u = ca.SX.sym("u", NU_PHYSICAL)
        f = quadcopter_continuous_dynamics(x, u)
        a_fun = ca.Function("A", [x, u], [ca.jacobian(f, x)])
        b_fun = ca.Function("B", [x, u], [ca.jacobian(f, u)])
        x_eq = np.zeros(NX_QUAD)
        u_eq_vec = np.full(NU_PHYSICAL, self.u_eq)
        A = np.array(a_fun(x_eq, u_eq_vec))
        B = np.array(b_fun(x_eq, u_eq_vec))

        # Exact zero-order-hold discretisation via the augmented matrix exp.
        n, m = NX_QUAD, NU_PHYSICAL
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A
        M[:n, n:] = B
        Md = expm(M * ts)
        Ad = Md[:n, :n]
        Bd = Md[:n, n:]

        q = np.diag(Q_DIAG)
        r = R_WEIGHT * np.eye(m)
        P = solve_discrete_are(Ad, Bd, q, r)
        self.K = np.linalg.solve(r + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
        self.A_disc = Ad
        self.B_disc = Bd

    def __call__(self, x, x_ref):
        u = self.u_eq - self.K @ (np.asarray(x) - np.asarray(x_ref))
        return np.clip(u, self.lb_u, self.ub_u)


# ---------------------------------------------------------------------------
# Projector generation  T1, T2   (PDF eqs. (2.8), (2.10)-(2.11))
# ---------------------------------------------------------------------------
#
# In every case T1 in R^{(N*nu) x nv} has orthonormal columns (T1^T T1 = I) and
# T2 = null_space(T1^T) spans the orthogonal complement, so [T1 T2] is
# orthonormal exactly as required by T T^T = I in eq. (2.8).


def _block_identity(n_rows, n_cols):
    matrix = np.zeros((n_rows, n_cols))
    k = min(n_rows, n_cols)
    matrix[:k, :k] = np.eye(k)
    return matrix


def _finish_projector(T1, kind, diagnostics):
    """Orthonormalise T1 columns, build T2 and verify the structure."""
    # QR keeps col(T1) but guarantees exact orthonormal columns.
    q, _ = np.linalg.qr(T1)
    T1 = q[:, : T1.shape[1]]
    T2 = null_space(T1.T)

    ortho_residual = float(np.linalg.norm(T1.T @ T1 - np.eye(T1.shape[1])))
    complement_residual = float(np.linalg.norm(T1.T @ T2)) if T2.size else 0.0
    diagnostics = dict(diagnostics)
    diagnostics.update(
        {
            "projector": kind,
            "n_stacked": int(T1.shape[0]),
            "nv_active": int(T1.shape[1]),
            "nw_inactive": int(T2.shape[1]),
            "orthonormality_residual_T1tT1_minus_I": ortho_residual,
            "complement_residual_T1tT2": complement_residual,
        }
    )
    return T1, T2, diagnostics


def build_identity_projector(n_stacked, nv):
    """Baseline: first ``nv`` canonical directions of the stacked input."""
    T1 = _block_identity(n_stacked, nv)
    return _finish_projector(T1, "identity", {})


def _covariance_eigendecomposition(C):
    """C = T Sigma T^T with descending eigenvalues (PDF eq. (2.11))."""
    eigvals, eigvecs = np.linalg.eigh(C)  # ascending, orthonormal
    order = np.argsort(eigvals)[::-1]
    return eigvals[order], eigvecs[:, order]


def build_pca_projector(n_stacked, nv, feature_samples):
    """PCA / active-subspace projector from nominal feature samples.

    ``feature_samples`` is an (M, n_stacked) array of feature vectors S(x_i)
    (here optimal nominal input stacks U*(x_i)).  Following PDF eq. (2.10) we
    form the covariance C = (1/M) sum_i S_i S_i^T, eigendecompose it
    (eq. (2.11)) and take the leading ``nv`` eigenvectors as the active subspace.
    """
    S = np.asarray(feature_samples, dtype=float)
    if S.ndim != 2 or S.shape[1] != n_stacked:
        raise ValueError(f"feature_samples must be (M, {n_stacked}), got {S.shape}")
    C = (S.T @ S) / S.shape[0]
    eigvals, eigvecs = _covariance_eigendecomposition(C)
    T1 = eigvecs[:, :nv]
    diagnostics = {
        "n_samples": int(S.shape[0]),
        "eigenvalues": [float(v) for v in eigvals],
        "captured_energy_fraction": float(eigvals[:nv].sum() / eigvals.sum())
        if eigvals.sum() > 0
        else 0.0,
    }
    return _finish_projector(T1, "pca", diagnostics)


def build_hessian_projector(n_stacked, nv, cost_hessian):
    """Curvature projector from the cost Hessian d^2 J / dU^2.

    The leading ``nv`` eigenvectors of the (symmetric) Hessian are the
    directions in which the cost varies most strongly -- the directions worth
    keeping as active under reduction (a sensitivity / cost-curvature feature,
    PDF "Computation of active subspaces via sensitivities analysis").
    """
    H = np.asarray(cost_hessian, dtype=float)
    H = 0.5 * (H + H.T)
    eigvals, eigvecs = _covariance_eigendecomposition(H)
    T1 = eigvecs[:, :nv]
    diagnostics = {
        "eigenvalues": [float(v) for v in eigvals],
        "captured_curvature_fraction": float(
            np.abs(eigvals[:nv]).sum() / np.abs(eigvals).sum()
        )
        if np.abs(eigvals).sum() > 0
        else 0.0,
    }
    return _finish_projector(T1, "hessian", diagnostics)


# ---------------------------------------------------------------------------
# Nominal data generation for the PCA / Hessian projectors (non-RL)
# ---------------------------------------------------------------------------


def solve_nominal_ocp(cost_fun, x0, ref_stack, n_horizon, lb_u, ub_u, u_init):
    """Solve the nominal single-shooting OCP P(x_k) (PDF eq. (2.3)).

    Minimises J(x0, U) over the stacked input U subject to the physical motor
    bounds, with IPOPT.  Returns the optimal stacked input U* (the feature
    vector S(x0)).
    """
    nu_stack = n_horizon * NU_PHYSICAL
    U = ca.SX.sym("U", nu_stack)
    p_x0 = ca.SX.sym("p_x0", NX_QUAD)
    p_ref = ca.SX.sym("p_ref", (n_horizon + 1) * NX_QUAD)
    nlp = {
        "x": U,
        "f": cost_fun(p_x0, U, p_ref),
        "p": ca.vertcat(p_x0, p_ref),
    }
    solver = ca.nlpsol(
        "nominal_ocp",
        "ipopt",
        nlp,
        {"print_time": 0, "ipopt": {"print_level": 0, "sb": "yes", "max_iter": 200}},
    )
    lbx = np.tile(lb_u, n_horizon)
    ubx = np.tile(ub_u, n_horizon)
    sol = solver(
        x0=u_init,
        lbx=lbx,
        ubx=ubx,
        p=np.concatenate([np.asarray(x0, dtype=float), np.asarray(ref_stack, dtype=float)]),
    )
    return np.array(sol["x"]).flatten()


def sample_initial_conditions(x0_base, n_samples, seed=0):
    """Deterministic set X0 of initial conditions around the nominal start.

    Position / velocity / attitude states are perturbed with small zero-mean
    noise so the covariance C of the optimal input stacks reflects the local
    operating region used by the closed loop.
    """
    rng = np.random.default_rng(seed)
    scales = np.array(
        [0.5, 0.5, 0.5, 0.15, 0.15, 0.15, 0.3, 0.3, 0.3, 0.2, 0.2, 0.2]
    )
    samples = [np.asarray(x0_base, dtype=float)]
    for _ in range(n_samples - 1):
        samples.append(np.asarray(x0_base, dtype=float) + scales * rng.standard_normal(NX_QUAD))
    return np.array(samples)


def generate_nominal_input_data(
    cost_fun,
    x0_base,
    n_horizon,
    ts,
    lb_u,
    ub_u,
    u_init,
    n_samples=24,
    seed=0,
):
    """Collect optimal nominal input stacks U*(x_i) over sampled X0 (feature S)."""
    x0_samples = sample_initial_conditions(x0_base, n_samples, seed=seed)
    feats = []
    for xi in x0_samples:
        ref_stack = make_reference_stack(0.0, n_horizon, ts)
        u_star = solve_nominal_ocp(cost_fun, xi, ref_stack, n_horizon, lb_u, ub_u, u_init)
        feats.append(u_star)
    return np.array(feats)
