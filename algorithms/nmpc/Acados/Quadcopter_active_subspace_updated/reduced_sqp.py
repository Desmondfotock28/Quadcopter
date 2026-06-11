"""Custom dense Gauss-Newton SQP for the active-subspace NMPC -- the solver
the thesis actually calls for.

The thesis (sec. 2.1.2) solves the OCP by *single shooting*: states are
eliminated and the dense NLP has only the input variables. The active-subspace
reduction then shrinks that dense NLP from N*nu = 40 variables to nv + 1 = 11.
Stage-structured solvers (acados/HPIPM) cannot see this reduction -- they see
13 + nv + 1 states per stage instead -- which is why the acados encoding is
slower than full NMPC. This module implements the reduction natively:

  variables   z = [v; mu]  (11)
  cost        J(x0, T1 v + mu w)        rollout via RK4, Gauss-Newton on the
                                        residual stack (CasADi, JIT-compiled)
  constraints 0 <= T1 v + mu w <= umax  LINEAR in z -> exact in the QP
              mu in [0, 1.5]
  QP solver   DAQP (dense dual active-set, built for exactly this size)

A 40-variable full single-shooting SQP (box constraints only) is included so
the reduction benefit can be measured within the SAME solver.

Run:  python reduced_sqp.py            # closed loop, reduced vs full, timings
"""

from pathlib import Path
import time

import casadi as ca
import daqp
import numpy as np

from Quadcopter import (
    Q_diag,
    R_weight,
    m,
    g,
    nu_phy,
    nx,
    quadcopter_dynamics,
    max_force_per_motor,
)
from subspace_tools import (
    HorizonCost,
    TerminalFeedback,
    build_hessian_projector,
    make_reference_stack,
    solve_nominal_ocp,
)
from utils import reference_trajectory, split_stage_vector

N_horizon = 10
T_horizon = 1.0
Ts = T_horizon / N_horizon
N_stack = N_horizon * nu_phy
NV = 10
NZ = NV + 1

X0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
U_MIN = np.zeros(nu_phy)
U_MAX = np.full(nu_phy, max_force_per_motor)
U_HOVER = np.full(nu_phy, m * g / 4.0)
MU_MIN, MU_MAX = 0.0, 1.5
BIG = 1e20

HERE = Path(__file__).resolve().parent

def jit_opts(flags="-O1"):
    return {"jit": True, "compiler": "shell",
            "jit_options": {"flags": [flags], "verbose": False}}


def _rollout_residuals(x0, U, ref_stack, rk4_substeps=3):
    """Residual stack r with J = r^T r matching the acados EXTERNAL cost."""
    x = ca.SX.sym("xf", nx)
    u = ca.SX.sym("uf", nu_phy)
    f = ca.Function("f", [x, u], [quadcopter_dynamics(x, u)])
    h = Ts / rk4_substeps

    def rk4(xk, uk):
        k1 = f(xk, uk)
        k2 = f(xk + 0.5 * h * k1, uk)
        k3 = f(xk + 0.5 * h * k2, uk)
        k4 = f(xk + h * k3, uk)
        return xk + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    sq = np.sqrt(0.5 * np.asarray(Q_diag))
    sr = np.sqrt(0.5 * R_weight) * np.ones(nu_phy)
    sqN = np.sqrt(np.asarray(Q_diag))

    res = []
    xk = x0
    for j in range(N_horizon):
        uj = U[j * nu_phy:(j + 1) * nu_phy]
        refj = ref_stack[j * nx:(j + 1) * nx]
        res.append(ca.DM(sq) * (xk - refj))
        res.append(ca.DM(sr) * (uj - ca.DM(U_HOVER)))
        for _ in range(rk4_substeps):
            xk = rk4(xk, uj)
    refN = ref_stack[N_horizon * nx:]
    res.append(ca.DM(sqN) * (xk - refN))
    return ca.vertcat(*res)


def build_reduced_kernel(T1):
    """CasADi JIT function: (z, x0, ref, w) -> (f, g, H_gn) for J(x0, T1 v + mu w)."""
    z = ca.SX.sym("z", NZ)
    x0 = ca.SX.sym("x0", nx)
    ref = ca.SX.sym("ref", (N_horizon + 1) * nx)
    w = ca.SX.sym("w", N_stack)

    U = ca.DM(T1) @ z[:NV] + z[NV] * w
    r = _rollout_residuals(x0, U, ref)
    Jac = ca.jacobian(r, z)
    fval = ca.dot(r, r)
    grad = 2 * Jac.T @ r
    hess = 2 * (Jac.T @ Jac)
    fval, grad, hess = ca.cse(ca.vertcat(fval)), ca.cse(grad), ca.cse(hess)
    # same -O1 as the full kernel so the reduced-vs-full comparison is flag-neutral
    return ca.Function("red_fgH", [z, x0, ref, w], [fval, grad, hess], jit_opts("-O1"))


def build_full_kernel():
    """CasADi JIT function: (U, x0, ref) -> (f, g, H_gn) for the full problem."""
    U = ca.SX.sym("U", N_stack)
    x0 = ca.SX.sym("x0", nx)
    ref = ca.SX.sym("ref", (N_horizon + 1) * nx)
    r = _rollout_residuals(x0, U, ref)
    Jac = ca.jacobian(r, U)
    fval = ca.dot(r, r)
    grad = 2 * Jac.T @ r
    hess = 2 * (Jac.T @ Jac)
    fval, grad, hess = ca.cse(ca.vertcat(fval)), ca.cse(grad), ca.cse(hess)
    return ca.Function("full_fgH", [U, x0, ref], [fval, grad, hess], jit_opts("-O1"))


def solve_reduced_sqp(kernel, z0, x0, ref, w, A_cons, max_iter=15, tol=1e-6):
    """Dense GN-SQP: QP subproblems solved exactly by DAQP.

    DAQP convention: the first NZ rows of (blower, bupper) are simple bounds
    on the QP variable d, the remaining rows belong to A_cons @ d.
    """
    z = z0.copy()
    sense = np.zeros(NZ + N_stack, dtype=np.int32)
    iters = 0
    for _ in range(max_iter):
        fval, grad, hess = kernel(z, x0, ref, w)
        H = np.array(hess) + 1e-8 * np.eye(NZ)
        gvec = np.array(grad).reshape(-1)
        Uc = A_cons @ z

        blower = np.concatenate((
            np.full(NV, -BIG), [MU_MIN - z[NV]],    # simple bounds on d
            np.tile(U_MIN, N_horizon) - Uc,         # general rows: A d
        ))
        bupper = np.concatenate((
            np.full(NV, BIG), [MU_MAX - z[NV]],
            np.tile(U_MAX, N_horizon) - Uc,
        ))
        d, _, exitflag, _ = daqp.solve(H, gvec, A_cons, bupper, blower, sense)
        if exitflag != 1:
            raise RuntimeError(f"DAQP exitflag {exitflag}")
        z = z + d
        iters += 1
        if np.max(np.abs(d)) < tol:
            break
    return z, iters


def solve_full_sqp(kernel, U0, x0, ref, max_iter=15, tol=1e-6):
    """Full 40-var single-shooting GN-SQP, box constraints only."""
    U = U0.copy()
    sense = np.zeros(N_stack, dtype=np.int32)
    A_empty = np.zeros((0, N_stack))
    lb = np.tile(U_MIN, N_horizon)
    ub = np.tile(U_MAX, N_horizon)
    iters = 0
    for _ in range(max_iter):
        fval, grad, hess = kernel(U, x0, ref)
        H = np.array(hess) + 1e-8 * np.eye(N_stack)
        gvec = np.array(grad).reshape(-1)
        d, _, exitflag, _ = daqp.solve(H, gvec, A_empty, ub - U, lb - U, sense)
        if exitflag != 1:
            raise RuntimeError(f"DAQP exitflag {exitflag}")
        U = U + d
        iters += 1
        if np.max(np.abs(d)) < tol:
            break
    return U, iters


def make_plant(substeps=30):
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu_phy)
    f = ca.Function("f", [x, u], [quadcopter_dynamics(x, u)])
    h = Ts / substeps
    xk = x
    for _ in range(substeps):
        k1 = f(xk, u)
        k2 = f(xk + 0.5 * h * k1, u)
        k3 = f(xk + 0.5 * h * k2, u)
        k4 = f(xk + h * k3, u)
        xk = xk + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return ca.Function("plant", [x, u], [xk])


def run_closed_loop(mode, nsim=400):
    """mode: 'reduced' (11-var custom SQP) or 'full' (40-var custom SQP)."""
    hc = HorizonCost(N_horizon, Ts)
    # Fallback evaluator at the SAME discretization as the SQP kernels (and
    # the C port), so the rule compares costs the solver actually optimizes.
    hc_fallback = HorizonCost(N_horizon, Ts, rk4_substeps=3)
    tfb = TerminalFeedback(Ts, U_MIN, U_MAX)
    plant = make_plant()

    # Algorithm 1, line 1: solve nominal P(x0) for the initial candidate.
    u_tilde = solve_nominal_ocp(hc.fun, X0, 0.0, N_horizon, Ts, U_MIN, U_MAX)
    xcurrent = X0.copy()
    t0 = 0.0

    if mode == "reduced":
        T1, T2 = build_hessian_projector(hc.fun, N_horizon, Ts, NV, X0)
        kernel = build_reduced_kernel(T1)
        A_cons = np.hstack([T1, np.zeros((N_stack, 1))])  # mu column set per step
        inactive = T2 @ (T2.T @ u_tilde)
    else:
        kernel = build_full_kernel()
        U_guess = u_tilde.copy()

    solve_times, iters_log, pos_err = [], [], []
    n_fallback = 0
    jcl = 0.0
    q = np.diag(Q_diag)

    for i in range(nsim):
        ref = make_reference_stack(t0, N_horizon, Ts)

        tic = time.perf_counter()
        if mode == "reduced":
            A_cons[:, NV] = inactive
            z0 = np.concatenate([T1.T @ u_tilde, [1.0]])
            z_opt, it = solve_reduced_sqp(kernel, z0, xcurrent, ref, inactive, A_cons)
            U_reduced = T1 @ z_opt[:NV] + z_opt[NV] * inactive
            solve_times.append(time.perf_counter() - tic)

            # fallback rule (Algorithm 1, lines 3-7), not timed as solver work
            j_red = hc_fallback(xcurrent, U_reduced, t0)
            j_cand = hc_fallback(xcurrent, u_tilde, t0)
            if j_red <= j_cand:
                U_apply = U_reduced
            else:
                U_apply = u_tilde
                n_fallback += 1
        else:
            U_apply, it = solve_full_sqp(kernel, U_guess, xcurrent, ref)
            solve_times.append(time.perf_counter() - tic)
        iters_log.append(it)

        u0 = np.clip(split_stage_vector(U_apply, 0, nu_phy), U_MIN, U_MAX)
        ex = xcurrent - reference_trajectory(t0)
        eu = u0 - U_HOVER
        jcl += 0.5 * (ex @ q @ ex + eu @ (R_weight * np.eye(nu_phy)) @ eu)
        xcurrent = np.array(plant(xcurrent, u0)).flatten()
        t0 += Ts
        pos_err.append(np.linalg.norm(xcurrent[:3] - reference_trajectory(t0)[:3]))

        # shift + terminal append
        xk = xcurrent.copy()
        for jstage in range(1, N_horizon):
            xk = np.array(plant(xk, split_stage_vector(U_apply, jstage, nu_phy))).flatten()
        term_u = tfb(xk, reference_trajectory(t0 + (N_horizon - 1) * Ts))
        if mode == "reduced":
            u_tilde = np.concatenate([U_apply[nu_phy:], term_u])
            inactive = T2 @ (T2.T @ u_tilde)
        else:
            U_guess = np.concatenate([U_apply[nu_phy:], term_u])

    solve_times = np.array(solve_times)
    pos_err = np.array(pos_err)
    label = f"custom-{mode}" + (f" ({NZ} vars)" if mode == "reduced" else f" ({N_stack} vars)")
    print(f"\n=== {label} ===")
    print(f"sqp iters        : mean {np.mean(iters_log):5.1f}  max {np.max(iters_log)}")
    print(f"solve time [ms]  : mean {1e3 * solve_times.mean():7.3f}  p95 {1e3 * np.percentile(solve_times, 95):7.3f}  max {1e3 * solve_times.max():7.3f}")
    print(f"pos error [m]    : mean {pos_err.mean():.4f}  max {pos_err.max():.4f}  final {pos_err[-1]:.4f}")
    print(f"closed-loop cost : {jcl:.2f}")
    if mode == "reduced":
        print(f"fallbacks        : {n_fallback}/{nsim}")
    return {"label": label, "solve_times": solve_times, "err": pos_err.mean(), "jcl": jcl}


if __name__ == "__main__":
    res_full = run_closed_loop("full")
    res_red = run_closed_loop("reduced")
    speedup = res_full["solve_times"].mean() / res_red["solve_times"].mean()
    print(f"\nreduction speedup within the custom dense SQP: {speedup:.2f}x")
    np.savez(HERE / "results" / "custom_sqp_timing.npz",
             reduced_times=res_red["solve_times"], full_times=res_full["solve_times"])
