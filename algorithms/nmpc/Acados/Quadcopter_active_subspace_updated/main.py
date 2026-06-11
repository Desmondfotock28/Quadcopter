import argparse
from pathlib import Path
import time

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from Quadcopter import export_active_subspace_quadcopter_model, export_Quadcopter_ode_model
from subspace_tools import (
    HorizonCost,
    TerminalFeedback,
    build_block_diagonal_projector,
    build_corner_identity_projector,
    build_hessian_projector,
    solve_nominal_ocp,
)
from utils import (
    plot_3d_trajectory,
    plot_xyz_subplots,
    reference_trajectory,
    split_stage_matrix,
    split_stage_vector,
)


nu_phy = 4

nx = 13

N_horizon = 10

T_horizon = 1.0

Ts = T_horizon / N_horizon

N_stack_u = N_horizon * nu_phy

m = 2.0                                     # [kg] total mass

g = 9.8066

thrust_to_weight = 1.75

max_force_per_motor = (g * m / 4.0) * thrust_to_weight


X0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

u_min = np.array([0.0, 0.0, 0.0, 0.0])

u_max = np.array([max_force_per_motor, max_force_per_motor, max_force_per_motor, max_force_per_motor])

u_hover = np.array([m * g / 4.0, m * g / 4.0, m * g / 4.0, m * g / 4.0])

MU_MIN, MU_MAX = 0.0, 1.5

HERE = Path(__file__).resolve().parent


def make_stage_parameter(t1, stage, t0, inactive_stack, nv):
    t1_stage = split_stage_matrix(t1, stage, nu_phy)
    inactive_stage = split_stage_vector(inactive_stack, stage, nu_phy)
    return np.concatenate(
        (
            reference_trajectory(t0 + stage * Ts),
            t1_stage.reshape(-1, order="F"),
            inactive_stage,
        )
    )


def make_terminal_parameter(t0, nv):
    return np.concatenate(
        (
            reference_trajectory(t0 + N_horizon * Ts),
            np.zeros(nu_phy * nv),
            np.zeros(nu_phy),
        )
    )


def set_stage_parameters(solver, t1, t0, inactive_stack, nv):
    for stage in range(N_horizon):
        solver.set(stage, "p", make_stage_parameter(t1, stage, t0, inactive_stack, nv))
    solver.set(N_horizon, "p", make_terminal_parameter(t0, nv))


def create_ocp_solver_description(nv, legacy, integrator="IRK", qp_solver="PARTIAL_CONDENSING_HPIPM") -> AcadosOcp:

    ocp = AcadosOcp()

    # Legacy reproduces the first version exactly: input cost centred at max
    # thrust and no bounds on mu. The fixed version centres the cost at hover
    # and keeps mu in [0, 1.5] (it scales the whole inactive stack).
    if legacy:
        model = export_active_subspace_quadcopter_model(
            nv, u_cost_center=max_force_per_motor, name_suffix="_legacy"
        )
    else:
        model = export_active_subspace_quadcopter_model(nv)
    ocp.model = model
    ocp.solver_options.N_horizon = N_horizon

    mu_idx = nx + nv

    ocp.parameter_values = np.zeros(nx + nu_phy * nv + nu_phy)

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.constraints.lbx_0 = X0
    ocp.constraints.ubx_0 = X0

    ocp.constraints.idxbu = np.array([0])
    ocp.constraints.lbu = np.array([0.0])
    ocp.constraints.ubu = np.array([0.0])

    ocp.constraints.lh = u_min
    ocp.constraints.uh = u_max

    if not legacy:
        ocp.constraints.idxbx = np.array([mu_idx])
        ocp.constraints.lbx = np.array([MU_MIN])
        ocp.constraints.ubx = np.array([MU_MAX])
        ocp.constraints.idxbx_e = np.array([mu_idx])
        ocp.constraints.lbx_e = np.array([MU_MIN])
        ocp.constraints.ubx_e = np.array([MU_MAX])

    # The augmented states (v, mu) make every stage 13+nv+1 dimensional, so
    # stage-wise (partial-condensing) Riccati solvers pay for the reduction
    # instead of profiting from it. FULL_CONDENSING eliminates all states and
    # leaves only the genuinely free variables (v, mu, dummy u's) in the QP.
    ocp.solver_options.qp_solver = qp_solver
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = integrator
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tf = T_horizon

    # acados scales stage costs by the step size by default; disable so the
    # solver optimizes exactly sum(stage) + terminal -- the same J(x, U) the
    # fallback rule evaluates (otherwise the two disagree and the fallback
    # fires against solutions that are optimal for a different objective).
    ocp.solver_options.cost_scaling = np.ones(N_horizon + 1)

    return ocp


def create_sim_solver_description() -> AcadosSim:

    sim = AcadosSim()

    sim.model = export_Quadcopter_ode_model()

    sim.solver_options.integrator_type = "IRK"

    sim.solver_options.T = Ts

    sim.solver_options.num_stages = 4

    sim.solver_options.num_steps = 3

    return sim


def warm_start(solver, t1, xcurrent, u_tilde_k, nv, shift_states=False):
    """Re-warm-start every iteration: v = T1^T u~, mu = 1 reconstructs the
    shifted candidate exactly (T1 T1^T u~ + 1 * T2 T2^T u~ = u~), so the
    solver starts from a consistent, feasible guess under the NEW parameters.
    The previous (v, mu) left in the solver means a different physical input
    once the inactive stack changes -- never reuse it across iterations.
    """
    v0 = t1.T @ u_tilde_k
    if shift_states:
        # Shift the previous state trajectory by one stage as the state guess.
        prev = [solver.get(stage, "x") for stage in range(N_horizon + 1)]
        guesses = prev[1:] + [prev[-1]]
    else:
        x_aug = np.zeros(nx + nv + 1)
        x_aug[:nx] = xcurrent
        guesses = [x_aug.copy() for _ in range(N_horizon + 1)]

    for stage, x_aug in enumerate(guesses):
        x_aug = np.array(x_aug)
        x_aug[nx:nx + nv] = v0
        x_aug[nx + nv] = 1.0
        if stage == 0:
            x_aug[:nx] = xcurrent
        solver.set(stage, "x", x_aug)
    for stage in range(N_horizon):
        solver.set(stage, "u", np.array([0.0]))


def build_projector(kind, horizon_cost, nv):
    if kind == "hessian":
        return build_hessian_projector(horizon_cost.fun, N_horizon, Ts, nv, X0)
    if kind == "block-diagonal":
        if nv % N_horizon != 0:
            raise ValueError("block-diagonal projector needs nv divisible by N_horizon")
        return build_block_diagonal_projector(N_horizon, nv // N_horizon)
    if kind == "corner-identity":
        return build_corner_identity_projector(N_stack_u, nv)
    raise ValueError(f"unknown projector '{kind}'")


def reconstruct_input_stack(t1, v_active, mu, inactive_stack):
    return t1 @ v_active + mu * inactive_stack


def solve_active_subspace_closed_loop(projector="hessian", nv=10, nsim=400, legacy=False, tag=None,
                                      integrator="IRK", qp_solver="PARTIAL_CONDENSING_HPIPM"):

    horizon_cost = HorizonCost(N_horizon, Ts)

    terminal_feedback = TerminalFeedback(Ts, u_min, u_max)

    T1, T2 = build_projector(projector, horizon_cost, nv)

    ocp = create_ocp_solver_description(nv, legacy, integrator, qp_solver)

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp_' + ocp.model.name + '.json')

    sim = create_sim_solver_description()
    acados_integrator = AcadosSimSolver(sim, json_file="acados_sim_" + sim.model.name + ".json")

    # Algorithm 1, line 1: solve the nominal P(x0) for the initial candidate u~_0.
    u_tilde_k = solve_nominal_ocp(horizon_cost.fun, X0, 0.0, N_horizon, Ts, u_min, u_max)

    simX = np.zeros((nsim + 1, nx))

    simU = np.zeros((nsim, nu_phy))

    solve_times = []

    sqp_iters = []

    xcurrent = X0.copy()

    simX[0, :] = xcurrent

    t0 = 0.0

    t = [t0]

    solver_statuses = []

    n_fallback = 0

    inactive_stack = T2 @ (T2.T @ u_tilde_k)

    warm_start(acados_ocp_solver, T1, xcurrent, u_tilde_k, nv, shift_states=False)

    for i in range(nsim):
        # Algorithm 1, line 2: solve the reduced OCP P(x_k, w~_k) for (v*, mu*).
        acados_ocp_solver.set(0, "lbx", xcurrent)

        acados_ocp_solver.set(0, "ubx", xcurrent)

        set_stage_parameters(acados_ocp_solver, T1, t0, inactive_stack, nv)

        start_time = time.time()
        status = acados_ocp_solver.solve()
        solve_times.append(time.time() - start_time)
        solver_statuses.append(status)
        sqp_iters.append(int(acados_ocp_solver.get_stats("sqp_iter")))

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            raise Exception(f"acados returned status {status} at closed-loop step {i}")

        x_aug0 = acados_ocp_solver.get(0, "x")

        v_active_opt = x_aug0[nx:nx + nv]

        mu_opt = x_aug0[nx + nv]

        # Reconstruct the reduced stacked input  U = T1 v* + mu* T2 w~.
        u_reduced_stack = reconstruct_input_stack(T1, v_active_opt, mu_opt, inactive_stack)

        if legacy:
            chosen_stack = u_reduced_stack
        else:
            # Algorithm 1, lines 3-7: fallback rule. If the reduced solve came
            # out worse than the shifted candidate (it can: the reduced
            # feasible set is a subspace slice), apply the candidate instead.
            j_reduced = horizon_cost(xcurrent, u_reduced_stack, t0)
            j_candidate = horizon_cost(xcurrent, u_tilde_k, t0)
            if j_reduced <= j_candidate:
                chosen_stack = u_reduced_stack
            else:
                chosen_stack = u_tilde_k
                n_fallback += 1

        u0 = np.clip(split_stage_vector(chosen_stack, 0, nu_phy), u_min, u_max)

        simU[i, :] = u0

        acados_integrator.set("x", xcurrent)

        acados_integrator.set("u", u0)

        sim_status = acados_integrator.solve()

        if sim_status != 0:
            raise Exception(f"acados integrator returned status {sim_status} at closed-loop step {i}")

        xcurrent = acados_integrator.get("x")

        simX[i + 1, :] = xcurrent

        if legacy:
            # Original shift: duplicate the last stage input, stale warm start.
            u_tilde_k = np.concatenate([chosen_stack[nu_phy:], chosen_stack[(N_horizon - 1) * nu_phy:]])
        else:
            # Algorithm 1, line 9: shift the chosen sequence and append the
            # terminal feedback kappa at the terminal state predicted under
            # the CHOSEN sequence (the solver's own terminal state is only
            # valid when the reduced solution was applied, not on fallback).
            x_terminal = xcurrent.copy()
            for jstage in range(1, N_horizon):
                acados_integrator.set("x", x_terminal)
                acados_integrator.set("u", split_stage_vector(chosen_stack, jstage, nu_phy))
                if acados_integrator.solve() != 0:
                    raise Exception(f"terminal rollout failed at step {i}")
                x_terminal = acados_integrator.get("x")
            x_ref_terminal = reference_trajectory(t0 + N_horizon * Ts)
            terminal_u = terminal_feedback(x_terminal, x_ref_terminal)
            u_tilde_k = np.concatenate([chosen_stack[nu_phy:], terminal_u])

        inactive_stack = T2 @ (T2.T @ u_tilde_k)

        t0 += Ts
        t.append(t0)

        if not legacy:
            warm_start(acados_ocp_solver, T1, xcurrent, u_tilde_k, nv, shift_states=True)

    t = np.array(t)

    solve_times = np.array(solve_times)
    sqp_iters = np.array(sqp_iters)
    refs = np.array([reference_trajectory(ti) for ti in t])
    pos_err = np.linalg.norm(simX[:, :3] - refs[:, :3], axis=1)
    statuses, counts = np.unique(solver_statuses, return_counts=True)

    label = tag or (("legacy" if legacy else f"{projector}-nv{nv}")
                    + ("-erk" if integrator == "ERK" else "")
                    + ("-fullcond" if qp_solver.startswith("FULL_CONDENSING") else ""))
    print(f"\n=== acados closed loop [{label}] ===")
    print(f"solver statuses      : {dict(zip(statuses.tolist(), counts.tolist()))}")
    print(f"sqp iters            : mean {sqp_iters.mean():.1f}  max {sqp_iters.max()}")
    print(f"solve time [ms]      : mean {1e3 * solve_times.mean():.3f}  max {1e3 * solve_times.max():.3f}")
    print(f"position error [m]   : mean {pos_err.mean():.4f}  max {pos_err.max():.4f}  final {pos_err[-1]:.4f}")
    print(f"fallbacks applied    : {n_fallback}/{nsim}")
    print(f"motor cmd range [N]  : [{simU.min():.3f}, {simU.max():.3f}]")

    out_dir = HERE / "results"
    out_dir.mkdir(exist_ok=True)
    np.savez(out_dir / f"closed_loop_{label}.npz", t=t, simX=simX, simU=simU,
             solve_times=solve_times, sqp_iters=sqp_iters, statuses=np.array(solver_statuses))
    plot_3d_trajectory(t, simX, output_path=out_dir / f"trajectory_3d_{label}.png", show=False)
    plot_xyz_subplots(t, simX, output_path=out_dir / f"position_tracking_{label}.png", show=False)
    print(f"results saved to {out_dir} (label: {label})")

    return {
        "label": label,
        "solve_times": solve_times,
        "sqp_iters": sqp_iters,
        "pos_err": pos_err,
        "simX": simX,
        "simU": simU,
        "t": t,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Active-subspace NMPC closed loop (acados).")
    parser.add_argument("--projector", choices=["hessian", "block-diagonal", "corner-identity"], default="hessian")
    parser.add_argument("--nv", type=int, default=10)
    parser.add_argument("--nsim", type=int, default=400)
    parser.add_argument("--legacy", action="store_true",
                        help="Reproduce the first version: corner-identity projector, free mu, "
                             "max-thrust cost centre, no fallback, stale warm start.")
    parser.add_argument("--integrator", choices=["IRK", "ERK"], default="IRK")
    parser.add_argument("--full-condensing", action="store_true",
                        help="Use FULL_CONDENSING_HPIPM so the QP only contains the truly free variables.")
    args = parser.parse_args()

    qp = "FULL_CONDENSING_HPIPM" if args.full_condensing else "PARTIAL_CONDENSING_HPIPM"
    if args.legacy:
        solve_active_subspace_closed_loop(
            projector="corner-identity", nv=3, nsim=args.nsim, legacy=True,
            integrator=args.integrator, qp_solver=qp
        )
    else:
        solve_active_subspace_closed_loop(
            projector=args.projector, nv=args.nv, nsim=args.nsim, legacy=False,
            integrator=args.integrator, qp_solver=qp
        )
