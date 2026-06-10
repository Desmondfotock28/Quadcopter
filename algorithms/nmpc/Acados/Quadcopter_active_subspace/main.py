from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
import argparse
import json
import numpy as np
from pathlib import Path
import time

import casadi as ca

from Quadcopter import (
    NX_QUAD,
    NU_PHYSICAL,
    U_EQUILIBRIUM,
    export_active_subspace_quadcopter_model,
    export_quadcopter_realplant_model,
)
from subspace_tools import (
    HorizonCost,
    TerminalFeedback,
    build_hessian_projector,
    build_identity_projector,
    build_pca_projector,
    generate_nominal_input_data,
    make_reference_stack,
    solve_nominal_ocp,
)
from utils import (
    plot_3d_trajectory,
    plot_xyz_subplots,
    reconstruct_input_stack,
    reference_trajectory,
    shift_input_stack,
    split_stage_matrix,
    split_stage_vector,
)

N_HORIZON = 10
T_HORIZON = 1.0
TS = T_HORIZON / N_HORIZON
NV_ACTIVE = 3
N_STACKED_U = N_HORIZON * NU_PHYSICAL

X0_QUAD = np.zeros(NX_QUAD)
U_HOVER = np.array([5.75, 5.75, 5.75, 5.75])
LB_U = np.array([0.5, 0.5, 0.5, 0.5])
UB_U = np.array([11.0, 11.0, 11.0, 11.0])


def build_projector(kind, horizon_cost, n_samples=24, seed=0):
    """Construct (T1, T2, diagnostics) for the requested projector.

    * ``identity``: first NV_ACTIVE canonical directions of the stacked input.
    * ``pca``:      eigenvectors of the covariance C of optimal nominal input
                    stacks U*(x) over sampled initial conditions (PDF (2.10)).
    * ``hessian``:  leading eigenvectors of the cost Hessian d^2 J / dU^2.
    """
    if kind == "identity":
        return build_identity_projector(N_STACKED_U, NV_ACTIVE)

    if kind == "pca":
        u_init = np.full(N_STACKED_U, U_EQUILIBRIUM)
        feats = generate_nominal_input_data(
            horizon_cost.fun,
            X0_QUAD,
            N_HORIZON,
            TS,
            LB_U,
            UB_U,
            u_init,
            n_samples=n_samples,
            seed=seed,
        )
        return build_pca_projector(N_STACKED_U, NV_ACTIVE, feats)

    if kind == "hessian":
        ref0 = make_reference_stack(0.0, N_HORIZON, TS)
        u_sym = ca.SX.sym("U", N_STACKED_U)
        cost = horizon_cost.fun(ca.DM(X0_QUAD), u_sym, ca.DM(ref0))
        hess_fun = ca.Function("H", [u_sym], [ca.hessian(cost, u_sym)[0]])
        hessian = np.array(hess_fun(np.full(N_STACKED_U, U_EQUILIBRIUM)))
        return build_hessian_projector(N_STACKED_U, NV_ACTIVE, hessian)

    raise ValueError(f"unknown projector '{kind}'")


def make_stage_parameter(t1, stage, t0, inactive_stack):
    t1_stage = split_stage_matrix(t1, stage, NU_PHYSICAL)
    inactive_stage = split_stage_vector(inactive_stack, stage, NU_PHYSICAL)
    return np.concatenate(
        (
            reference_trajectory(t0 + stage * TS),
            t1_stage.reshape(-1, order="F"),
            inactive_stage,
        )
    )


def create_ocp_solver_description() -> AcadosOcp:
    ocp = AcadosOcp()
    model = export_active_subspace_quadcopter_model(NV_ACTIVE)
    ocp.model = model
    ocp.solver_options.N_horizon = N_HORIZON

    mu_idx = NX_QUAD + NV_ACTIVE

    ocp.parameter_values = np.zeros(NX_QUAD + NU_PHYSICAL * NV_ACTIVE + NU_PHYSICAL)

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.constraints.idxbx_0 = np.arange(NX_QUAD)
    ocp.constraints.lbx_0 = X0_QUAD
    ocp.constraints.ubx_0 = X0_QUAD

    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.lbu = np.array([0.0])
    ocp.constraints.ubu = np.array([0.0])

    ocp.constraints.lh = LB_U
    ocp.constraints.uh = UB_U

    #ocp.constraints.idxbx = np.array([mu_idx])
    #ocp.constraints.lbx = np.array([0.0])
    #ocp.constraints.ubx = np.array([1.5])
    #ocp.constraints.idxbx_e = np.array([mu_idx])
    #ocp.constraints.lbx_e = np.array([0.0])
    #ocp.constraints.ubx_e = np.array([1.5])

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tf = T_HORIZON

    return ocp


def create_sim_solver_description() -> AcadosSim:
    sim = AcadosSim()
    sim.model = export_quadcopter_realplant_model()
    sim.solver_options.integrator_type = "IRK"
    sim.solver_options.T = TS
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 3
    return sim


def initialize_active_variables(solver, t1, xcurrent, u_tilde):
    # Warm start: v0 = T1^T u~ with mu = 1 reconstructs the full feasible
    # candidate, since T1 T1^T u~ + mu T2 T2^T u~ = u~. This is a feasible guess
    # for any (dense) projector, unlike v0 = 0.
    v0 = t1.T @ u_tilde
    x_aug = np.zeros(NX_QUAD + NV_ACTIVE + 1)
    x_aug[:NX_QUAD] = xcurrent
    x_aug[NX_QUAD:NX_QUAD + NV_ACTIVE] = v0
    x_aug[NX_QUAD + NV_ACTIVE] = 1.0

    for stage in range(N_HORIZON + 1):
        solver.set(stage, "x", x_aug)
    for stage in range(N_HORIZON):
        solver.set(stage, "u", np.array([0.0]))


def make_terminal_parameter(t0):
    return np.concatenate(
        (
            reference_trajectory(t0 + N_HORIZON * TS),
            np.zeros(NU_PHYSICAL * NV_ACTIVE),
            np.zeros(NU_PHYSICAL),
        )
    )


def set_stage_parameters(solver, t1, t0, inactive_stack):
    for stage in range(N_HORIZON):
        solver.set(stage, "p", make_stage_parameter(t1, stage, t0, inactive_stack))
    solver.set(N_HORIZON, "p", make_terminal_parameter(t0))


def compute_metrics(t, sim_x, sim_u, solve_times, solver_statuses, fallback_log, projector_diag):
    refs = np.array([reference_trajectory(ti) for ti in t])
    pos_error = sim_x[:, :3] - refs[:, :3]
    pos_error_norm = np.linalg.norm(pos_error, axis=1)
    status_values, status_counts = np.unique(solver_statuses, return_counts=True)
    status_count_map = {str(int(status)): int(count) for status, count in zip(status_values, status_counts)}

    applied_reduced = np.asarray(fallback_log["applied_reduced"], dtype=bool)
    j_reduced = np.asarray(fallback_log["j_reduced"], dtype=float)
    j_candidate = np.asarray(fallback_log["j_candidate"], dtype=float)
    n_reduced = int(applied_reduced.sum())
    n_fallback = int((~applied_reduced).sum())

    return {
        "projector": projector_diag["projector"],
        "nsim": int(sim_u.shape[0]),
        "horizon_steps": int(N_HORIZON),
        "horizon_seconds": float(T_HORIZON),
        "sample_time": float(TS),
        "active_dimension": int(NV_ACTIVE),
        "stacked_input_dimension": int(N_STACKED_U),
        "mean_solve_time_s": float(np.mean(solve_times)),
        "max_solve_time_s": float(np.max(solve_times)),
        "mean_position_error_m": float(np.mean(pos_error_norm)),
        "final_position_error_m": float(pos_error_norm[-1]),
        "max_position_error_m": float(np.max(pos_error_norm)),
        "min_motor_command": float(np.min(sim_u)),
        "max_motor_command": float(np.max(sim_u)),
        "solver_status_counts": status_count_map,
        "fallback": {
            "reduced_applied": n_reduced,
            "candidate_applied": n_fallback,
            "reduced_fraction": float(n_reduced / sim_u.shape[0]),
            "mean_j_reduced": float(np.mean(j_reduced)),
            "mean_j_candidate": float(np.mean(j_candidate)),
            "mean_j_gap_candidate_minus_reduced": float(np.mean(j_candidate - j_reduced)),
        },
        "projector_diagnostics": projector_diag,
    }


def save_results(output_dir, t, sim_x, sim_u, solve_times, solver_statuses, fallback_log, metrics, projector_diag, make_plots=True):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    np.savez(
        output_dir / "active_subspace_results.npz",
        t=t,
        sim_x=sim_x,
        sim_u=sim_u,
        solve_times=solve_times,
        solver_statuses=solver_statuses,
        j_reduced=np.asarray(fallback_log["j_reduced"], dtype=float),
        j_candidate=np.asarray(fallback_log["j_candidate"], dtype=float),
        applied_reduced=np.asarray(fallback_log["applied_reduced"], dtype=bool),
    )
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as metrics_file:
        json.dump(metrics, metrics_file, indent=2)
    with open(output_dir / "projector_info.json", "w", encoding="utf-8") as proj_file:
        json.dump(projector_diag, proj_file, indent=2)

    if make_plots:
        plot_3d_trajectory(t, sim_x, output_path=output_dir / "trajectory_3d.png", show=False)
        plot_xyz_subplots(t, sim_x, output_path=output_dir / "position_tracking.png", show=False)


def solve_active_subspace_closed_loop(
    nsim=401,
    output_dir="results",
    projector="identity",
    pca_samples=24,
    seed=0,
    make_plots=True,
):
    # Horizon-cost evaluator J(x_k, U) -- backs both the fallback rule and the
    # data/curvature based projector construction.
    horizon_cost = HorizonCost(N_HORIZON, TS)
    terminal_feedback = TerminalFeedback(TS, LB_U, UB_U)

    t1, t2, projector_diag = build_projector(
        projector, horizon_cost, n_samples=pca_samples, seed=seed
    )
    print(
        f"[projector={projector}] T1{t1.shape}  T1^T T1 - I = "
        f"{projector_diag['orthonormality_residual_T1tT1_minus_I']:.2e}  "
        f"T1^T T2 = {projector_diag['complement_residual_T1tT2']:.2e}"
    )

    ocp = create_ocp_solver_description()
    solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")

    sim = create_sim_solver_description()
    integrator = AcadosSimSolver(sim, json_file="acados_sim_" + sim.model.name + ".json")

    sim_x = np.zeros((nsim + 1, NX_QUAD))
    sim_u = np.zeros((nsim, NU_PHYSICAL))
    solve_times = []
    solver_statuses = []
    fallback_log = {"j_reduced": [], "j_candidate": [], "applied_reduced": []}

    xcurrent = X0_QUAD.copy()
    sim_x[0, :] = xcurrent
    t0 = 0.0

    # Algorithm 1, line 1: solve the nominal OCP P(x0) (PDF eq. (2.3)) for the
    # initial feasible candidate u~_0, then set w~_0 = T2^T u~_0
    # (-> inactive_stack = T2 w~_0).  A hover stack is the IPOPT initial guess.
    ref0 = make_reference_stack(t0, N_HORIZON, TS)
    u_tilde = solve_nominal_ocp(
        horizon_cost.fun, xcurrent, ref0, N_HORIZON, LB_U, UB_U,
        u_init=np.tile(U_HOVER, N_HORIZON),
    )
    inactive_stack = t2 @ (t2.T @ u_tilde)

    initialize_active_variables(solver, t1, xcurrent, u_tilde)

    for i in range(nsim):
        # Algorithm 1, line 2: solve the reduced OCP P(x_k, w~_k) for (v*, mu*).
        solver.set(0, "lbx", xcurrent)
        solver.set(0, "ubx", xcurrent)
        set_stage_parameters(solver, t1, t0, inactive_stack)

        start_time = time.time()
        status = solver.solve()
        solve_times.append(time.time() - start_time)
        solver_statuses.append(status)

        if status not in [0, 2]:
            solver.print_statistics()
            raise Exception(f"acados returned status {status} at closed-loop step {i}")

        x_aug0 = solver.get(0, "x")
        v_active = x_aug0[NX_QUAD:NX_QUAD + NV_ACTIVE]
        mu = x_aug0[NX_QUAD + NV_ACTIVE]

        # Reconstruct the reduced stacked input  U = T1 v* + mu* T2 w~.
        u_reduced_stack = reconstruct_input_stack(t1, v_active, mu, inactive_stack)

        # Algorithm 1, line 3: fallback rule -- compare exact costs of the
        # reduced solution and of the shifted candidate u~_k.
        j_reduced = horizon_cost(xcurrent, u_reduced_stack, t0)
        j_candidate = horizon_cost(xcurrent, u_tilde, t0)

        if j_reduced <= j_candidate:           # line 4: apply reduced input
            chosen_stack = u_reduced_stack
            applied_reduced = True
        else:                                  # line 6: fall back to u~_k
            chosen_stack = u_tilde
            applied_reduced = False

        fallback_log["j_reduced"].append(j_reduced)
        fallback_log["j_candidate"].append(j_candidate)
        fallback_log["applied_reduced"].append(applied_reduced)

        # Algorithm 1, line 8: apply the first input of the chosen sequence.
        u0 = np.clip(split_stage_vector(chosen_stack, 0, NU_PHYSICAL), LB_U, UB_U)
        sim_u[i, :] = u0

        integrator.set("x", xcurrent)
        integrator.set("u", u0)
        sim_status = integrator.solve()
        if sim_status != 0:
            raise Exception(f"acados integrator returned status {sim_status} at closed-loop step {i}")

        xcurrent = integrator.get("x")
        sim_x[i + 1, :] = xcurrent

        # Algorithm 1, line 9: shift the chosen sequence and append the terminal
        # state-feedback kappa(x_{k+N-1|k-1}) evaluated at the predicted terminal
        # state, then update w~_k = T2^T u~_k.
        x_terminal = solver.get(N_HORIZON, "x")[:NX_QUAD]
        x_ref_terminal = reference_trajectory(t0 + N_HORIZON * TS)
        terminal_u = terminal_feedback(x_terminal, x_ref_terminal)
        u_tilde = shift_input_stack(chosen_stack, terminal_u, NU_PHYSICAL)
        inactive_stack = t2 @ (t2.T @ u_tilde)
        t0 += TS

    t = np.arange(nsim + 1) * TS
    solve_times = np.array(solve_times)
    solver_statuses = np.array(solver_statuses)
    metrics = compute_metrics(t, sim_x, sim_u, solve_times, solver_statuses, fallback_log, projector_diag)
    save_results(
        output_dir, t, sim_x, sim_u, solve_times, solver_statuses, fallback_log, metrics, projector_diag, make_plots=make_plots
    )
    print(json.dumps(metrics, indent=2))

    return sim_x, sim_u, solve_times, metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run acados active-subspace quadcopter NMPC.")
    parser.add_argument("--nsim", type=int, default=400)
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--projector", choices=["identity", "pca", "hessian"], default="identity")
    parser.add_argument("--pca-samples", type=int, default=24, help="Initial-condition samples for the PCA covariance.")
    parser.add_argument("--seed", type=int, default=0, help="Seed for PCA initial-condition sampling.")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    solve_active_subspace_closed_loop(
        nsim=args.nsim,
        output_dir=args.output_dir,
        projector=args.projector,
        pca_samples=args.pca_samples,
        seed=args.seed,
        make_plots=not args.no_plots,
    )
