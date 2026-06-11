"""Full NMPC baseline in acados: same model, cost, horizon and bounds as the
active-subspace version, but optimizing all 4 motor thrusts at every stage.
Used as the optimality / timing reference for the reduced solver."""

import argparse
from pathlib import Path
import time

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver, AcadosSim, AcadosSimSolver

from Quadcopter import (
    Q_diag,
    R_weight,
    nu_phy,
    nx,
    quadcopter_dynamics,
    u_hover_per_motor,
    max_force_per_motor,
)
from utils import (
    plot_3d_trajectory,
    reference_trajectory,
)

N_horizon = 10
T_horizon = 1.0
Ts = T_horizon / N_horizon

X0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
u_min = np.zeros(nu_phy)
u_max = np.full(nu_phy, max_force_per_motor)

INTEGRATOR = "IRK"  # overridden by --integrator

HERE = Path(__file__).resolve().parent


def export_full_model() -> AcadosModel:
    x = ca.SX.sym("x", nx)
    u = ca.SX.sym("u", nu_phy)
    xdot = ca.SX.sym("xdot", nx)
    p = ca.SX.sym("p", nx)  # stage reference

    f_expl = quadcopter_dynamics(x, u)

    q_mat = ca.DM(np.diag(Q_diag))
    r_mat = ca.DM(R_weight * np.eye(nu_phy))
    u_hover = ca.DM(np.full(nu_phy, u_hover_per_motor))

    ex = x - p
    eu = u - u_hover

    model = AcadosModel()
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.p = p
    model.cost_expr_ext_cost = 0.5 * (ex.T @ q_mat @ ex + eu.T @ r_mat @ eu)
    model.cost_expr_ext_cost_e = ex.T @ q_mat @ ex
    model.name = "Quadcopter_full_nmpc"
    return model


def create_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    ocp.model = export_full_model()
    ocp.solver_options.N_horizon = N_horizon
    ocp.parameter_values = np.zeros(nx)

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    ocp.constraints.idxbx_0 = np.arange(nx)
    ocp.constraints.lbx_0 = X0
    ocp.constraints.ubx_0 = X0

    ocp.constraints.idxbu = np.arange(nu_phy)
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = INTEGRATOR
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tf = T_horizon
    # Same objective convention as the reduced solver: unscaled stage costs.
    ocp.solver_options.cost_scaling = np.ones(N_horizon + 1)
    return ocp


def create_sim() -> AcadosSim:
    from Quadcopter import export_Quadcopter_ode_model
    sim = AcadosSim()
    sim.model = export_Quadcopter_ode_model()
    sim.solver_options.integrator_type = "IRK"
    sim.solver_options.T = Ts
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 3
    return sim


def run(nsim=400):
    ocp = create_ocp()
    solver = AcadosOcpSolver(ocp, json_file="acados_ocp_" + ocp.model.name + ".json")
    sim = create_sim()
    integrator = AcadosSimSolver(sim, json_file="acados_sim_" + sim.model.name + "_full.json")

    simX = np.zeros((nsim + 1, nx))
    simU = np.zeros((nsim, nu_phy))
    solve_times, sqp_iters, statuses = [], [], []

    xcurrent = X0.copy()
    simX[0, :] = xcurrent
    t0 = 0.0
    t = [t0]

    u_hover = np.full(nu_phy, u_hover_per_motor)
    for stage in range(N_horizon + 1):
        solver.set(stage, "x", X0)
    for stage in range(N_horizon):
        solver.set(stage, "u", u_hover)

    for i in range(nsim):
        solver.set(0, "lbx", xcurrent)
        solver.set(0, "ubx", xcurrent)
        for stage in range(N_horizon + 1):
            solver.set(stage, "p", reference_trajectory(t0 + stage * Ts))

        tic = time.time()
        status = solver.solve()
        solve_times.append(time.time() - tic)
        statuses.append(status)
        sqp_iters.append(int(solver.get_stats("sqp_iter")))
        if status not in [0, 2]:
            solver.print_statistics()
            raise Exception(f"acados returned status {status} at step {i}")

        u0 = solver.get(0, "u")
        simU[i, :] = u0

        integrator.set("x", xcurrent)
        integrator.set("u", u0)
        if integrator.solve() != 0:
            raise Exception(f"integrator failed at step {i}")
        xcurrent = integrator.get("x")
        simX[i + 1, :] = xcurrent
        t0 += Ts
        t.append(t0)

    t = np.array(t)
    solve_times = np.array(solve_times)
    sqp_iters = np.array(sqp_iters)
    refs = np.array([reference_trajectory(ti) for ti in t])
    pos_err = np.linalg.norm(simX[:, :3] - refs[:, :3], axis=1)
    uniq, counts = np.unique(statuses, return_counts=True)

    print("\n=== acados closed loop [full-nmpc] ===")
    print(f"solver statuses      : {dict(zip(uniq.tolist(), counts.tolist()))}")
    print(f"sqp iters            : mean {sqp_iters.mean():.1f}  max {sqp_iters.max()}")
    print(f"solve time [ms]      : mean {1e3 * solve_times.mean():.3f}  max {1e3 * solve_times.max():.3f}")
    print(f"position error [m]   : mean {pos_err.mean():.4f}  max {pos_err.max():.4f}  final {pos_err[-1]:.4f}")

    out_dir = HERE / "results"
    out_dir.mkdir(exist_ok=True)
    np.savez(out_dir / "closed_loop_full-nmpc.npz", t=t, simX=simX, simU=simU,
             solve_times=solve_times, sqp_iters=sqp_iters, statuses=np.array(statuses))
    plot_3d_trajectory(t, simX, output_path=out_dir / "trajectory_3d_full-nmpc.png", show=False)
    return {"label": "full-nmpc", "solve_times": solve_times, "sqp_iters": sqp_iters, "pos_err": pos_err}


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--nsim", type=int, default=400)
    parser.add_argument("--integrator", choices=["IRK", "ERK"], default="IRK")
    args = parser.parse_args()
    INTEGRATOR = args.integrator
    run(nsim=args.nsim)
