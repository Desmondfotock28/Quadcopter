from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver , AcadosSim
from Quadcopter import export_active_subspace_quadcopter_model, export_Quadcopter_ode_model
from scipy.linalg import null_space
import numpy as np

import time

from utils import generate_block_identity,split_stage_matrix,reference_trajectory, split_stage_vector,plot_3d_trajectory




nu_phy = 4

nx = 13

N_horizon = 10

T_horizon = 1.0

Ts = T_horizon / N_horizon
nv = 3
N_stack_u = N_horizon * nu_phy

m = 2.0                                     # [kg] total mass

g = 9.8066 

thrust_to_weight = 1.75

max_force_per_motor = (g * m / 4.0) * thrust_to_weight


X0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

u_min = np.array([0.0, 0.0, 0.0, 0.0])

u_max = np.array([max_force_per_motor, max_force_per_motor, max_force_per_motor, max_force_per_motor])

u_hover = np.array([m*g/4.0, m*g/4.0, m*g/4.0, m*g/4.0])


def make_stage_parameter(t1, stage, t0, inactive_stack):
    t1_stage = split_stage_matrix(t1, stage, nu_phy)
    inactive_stage = split_stage_vector(inactive_stack, stage, nu_phy)
    return np.concatenate(
        (
            reference_trajectory(t0 + stage * Ts),
            t1_stage.reshape(-1, order="F"),
            inactive_stage,
        )
    )


def set_stage_parameters(solver, T1, t0, inactive_stack):
    for stage in range(N_horizon):
        solver.set(stage, "p", make_stage_parameter(T1, stage, t0, inactive_stack))
    solver.set(N_horizon, "p", make_terminal_parameter(t0))



def create_ocp_solver_description() -> AcadosOcp:

    ocp = AcadosOcp()

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

    #ocp.constraints.idxbx = np.array([mu_idx])
    #ocp.constraints.lbx = np.array([0.0])
    #ocp.constraints.ubx = np.array([1.0])
    #ocp.constraints.idxbx_e = np.array([mu_idx])
    #ocp.constraints.lbx_e = np.array([0.0])
    #ocp.constraints.ubx_e = np.array([1.0])

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "IRK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 3
    ocp.solver_options.print_level = 0
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tf = T_horizon

    return ocp

def create_sim_solver_description() -> AcadosSim:

    sim = AcadosSim()

    sim.model = export_Quadcopter_ode_model()

    sim.solver_options.integrator_type = "IRK"

    sim.solver_options.T = Ts

    sim.solver_options.num_stages = 4

    sim.solver_options.num_steps = 3

    return sim



def initialize_active_variables(solver, t1, xcurrent, u_tilde_k):
    # Warm start: v0 = T1^T u~ with mu = 1 reconstructs the full feasible
    # candidate, since T1 T1^T u~ + mu T2 T2^T u~ = u~. This is a feasible guess
    # for any (dense) projector, unlike v0 = 0.
    v0 = t1.T @ u_tilde_k
    x_aug = np.zeros(nx + nv + 1)
    x_aug[:nx] = xcurrent
    x_aug[nx:nx + nv] = v0
    x_aug[nx + nv] = 1.0

    for stage in range(N_horizon + 1):
        solver.set(stage, "x", x_aug)
    for stage in range(N_horizon):
        solver.set(stage, "u", np.array([0.0]))

def make_terminal_parameter(t0):
    return np.concatenate(
        (
            reference_trajectory(t0 + N_horizon * Ts),
            np.zeros(nu_phy * nv),
            np.zeros(nu_phy),
        )
    )

def reconstruct_input_stack(t1, v_active, mu, inactive_stack):
    return t1 @ v_active + mu * inactive_stack
    

def solve_active_subspace_closed_loop():


    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    sim = create_sim_solver_description()
    acados_integrator = AcadosSimSolver(sim, json_file="acados_sim_" + sim.model.name + ".json")

    u_tilde_k = np.load("/home/udeme/Quadcopter/algorithms/nmpc/Acados/Quadcopter_active_subspace_updated/open_loop_controls.npy").reshape(-1)  # shape (nu * N_horizon,)

    Nsim=400

    T1  = generate_block_identity(nu_phy*N_horizon, nv)

    T2  = null_space(T1.T)

    simX= np.zeros((Nsim + 1, nx))

    simU = np.zeros((Nsim, nu_phy))

    solve_times = []

    xcurrent = X0.copy()

    simX[0, :] = xcurrent

    t0 = 0.0

    t = [t0]

    solver_statuses = []

    inactive_stack = T2@(T2.T@u_tilde_k)

    initialize_active_variables(acados_ocp_solver, T1, xcurrent, u_tilde_k)

    for i in range(Nsim):
        # Algorithm 1, line 2: solve the reduced OCP P(x_k, w~_k) for (v*, mu*).
        acados_ocp_solver.set(0, "lbx", xcurrent)

        acados_ocp_solver.set(0, "ubx", xcurrent)

        set_stage_parameters(acados_ocp_solver, T1, t0, inactive_stack)

        start_time = time.time()
        status = acados_ocp_solver.solve()
        solve_times.append(time.time() - start_time)
        solver_statuses.append(status)
       

        acados_ocp_solver.print_statistics() # encapsulates: stat = acados_ocp_solver.get_stats("statistics")

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            raise Exception(f"acados returned status {status} at closed-loop step {i}")

        x_aug0 = acados_ocp_solver.get(0, "x")

        v_active_opt = x_aug0[nx:nx + nv]

        mu_opt = x_aug0[nx + nv]

        # Reconstruct the reduced stacked input  U = T1 v* + mu* T2 w~.
        u_reduced_stack = reconstruct_input_stack(T1, v_active_opt, mu_opt, inactive_stack)

        u0 = split_stage_vector(u_reduced_stack, 0, nu_phy) #need to remove clip 

        simU[i, :] = u0

        acados_integrator.set("x", xcurrent)

        acados_integrator.set("u", u0)

        sim_status = acados_integrator.solve()

        if sim_status != 0:
            raise Exception(f"acados integrator returned status {sim_status} at closed-loop step {i}")

        xcurrent = acados_integrator.get("x")

        simX[i + 1, :] = xcurrent

        u_tilde_k = np.concatenate([u_reduced_stack[nu_phy:], u_reduced_stack[(N_horizon-1)*nu_phy:]])

        inactive_stack = T2@(T2.T@u_tilde_k)

        t0 += Ts
        t.append(t0)

    t = np.array(t)

      # plot results
    solve_times = np.array(solve_times)

    plot_3d_trajectory(t, simX)

    print(np.mean(solve_times))


solve_active_subspace_closed_loop()