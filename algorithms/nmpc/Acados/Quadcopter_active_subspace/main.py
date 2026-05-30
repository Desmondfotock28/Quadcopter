from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver
from scipy.linalg import null_space
import argparse
import json
import numpy as np
from pathlib import Path
import time

from Quadcopter import NX_QUAD, NU_PHYSICAL, export_active_subspace_quadcopter_model, export_quadcopter_realplant_model
from utils import (
    generate_block_identity,
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
NV_ACTIVE = 10
N_STACKED_U = N_HORIZON * NU_PHYSICAL

X0_QUAD = np.zeros(NX_QUAD)
U_HOVER = np.array([5.75, 5.75, 5.75, 5.75])
LB_U = np.array([0.5, 0.5, 0.5, 0.5])
UB_U = np.array([11.0, 11.0, 11.0, 11.0])

T1 = generate_block_identity(N_STACKED_U, NV_ACTIVE)
T2 = null_space(T1.T)


def make_stage_parameter(stage, t0, inactive_stack):
    t1_stage = split_stage_matrix(T1, stage, NU_PHYSICAL)
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

    ocp.constraints.idxbx = np.array([mu_idx])
    ocp.constraints.lbx = np.array([0.0])
    ocp.constraints.ubx = np.array([1.5])
    ocp.constraints.idxbx_e = np.array([mu_idx])
    ocp.constraints.lbx_e = np.array([0.0])
    ocp.constraints.ubx_e = np.array([1.5])

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


def initialize_active_variables(solver, xcurrent, inactive_stack):
    v0 = T1.T @ inactive_stack
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


def set_stage_parameters(solver, t0, inactive_stack):
    for stage in range(N_HORIZON):
        solver.set(stage, "p", make_stage_parameter(stage, t0, inactive_stack))
    solver.set(N_HORIZON, "p", make_terminal_parameter(t0))


def compute_metrics(t, sim_x, sim_u, solve_times, solver_statuses):
    refs = np.array([reference_trajectory(ti) for ti in t])
    pos_error = sim_x[:, :3] - refs[:, :3]
    pos_error_norm = np.linalg.norm(pos_error, axis=1)
    status_values, status_counts = np.unique(solver_statuses, return_counts=True)
    status_count_map = {str(int(status)): int(count) for status, count in zip(status_values, status_counts)}
    return {
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
    }


def save_results(output_dir, t, sim_x, sim_u, solve_times, solver_statuses, metrics, make_plots=True):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    np.savez(
        output_dir / "active_subspace_results.npz",
        t=t,
        sim_x=sim_x,
        sim_u=sim_u,
        solve_times=solve_times,
        solver_statuses=solver_statuses,
    )
    with open(output_dir / "metrics.json", "w", encoding="utf-8") as metrics_file:
        json.dump(metrics, metrics_file, indent=2)

    if make_plots:
        plot_3d_trajectory(t, sim_x, output_path=output_dir / "trajectory_3d.png", show=False)
        plot_xyz_subplots(t, sim_x, output_path=output_dir / "position_tracking.png", show=False)


def solve_active_subspace_closed_loop(nsim=120, output_dir="results", make_plots=True):
    ocp = create_ocp_solver_description()
    solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")

    sim = create_sim_solver_description()
    integrator = AcadosSimSolver(sim, json_file="acados_sim_" + sim.model.name + ".json")

    sim_x = np.zeros((nsim + 1, NX_QUAD))
    sim_u = np.zeros((nsim, NU_PHYSICAL))
    solve_times = []
    solver_statuses = []

    xcurrent = X0_QUAD.copy()
    sim_x[0, :] = xcurrent
    t0 = 0.0

    u_tilde = np.tile(U_HOVER, N_HORIZON)
    inactive_stack = T2 @ (T2.T @ u_tilde)

    initialize_active_variables(solver, xcurrent, inactive_stack)

    for i in range(nsim):
        solver.set(0, "lbx", xcurrent)
        solver.set(0, "ubx", xcurrent)
        set_stage_parameters(solver, t0, inactive_stack)

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
        u_stack = reconstruct_input_stack(T1, v_active, mu, inactive_stack)
        u0 = np.clip(split_stage_vector(u_stack, 0, NU_PHYSICAL), LB_U, UB_U)

        sim_u[i, :] = u0
        integrator.set("x", xcurrent)
        integrator.set("u", u0)
        sim_status = integrator.solve()
        if sim_status != 0:
            raise Exception(f"acados integrator returned status {sim_status} at closed-loop step {i}")

        xcurrent = integrator.get("x")
        sim_x[i + 1, :] = xcurrent

        terminal_u = U_HOVER
        u_tilde = shift_input_stack(u_stack, terminal_u, NU_PHYSICAL)
        inactive_stack = T2 @ (T2.T @ u_tilde)
        t0 += TS

    t = np.arange(nsim + 1) * TS
    solve_times = np.array(solve_times)
    solver_statuses = np.array(solver_statuses)
    metrics = compute_metrics(t, sim_x, sim_u, solve_times, solver_statuses)
    save_results(output_dir, t, sim_x, sim_u, solve_times, solver_statuses, metrics, make_plots=make_plots)
    print(json.dumps(metrics, indent=2))

    return sim_x, sim_u, solve_times, metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run acados active-subspace quadcopter NMPC.")
    parser.add_argument("--nsim", type=int, default=120)
    parser.add_argument("--output-dir", default="results")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()

    solve_active_subspace_closed_loop(
        nsim=args.nsim,
        output_dir=args.output_dir,
        make_plots=not args.no_plots,
    )
