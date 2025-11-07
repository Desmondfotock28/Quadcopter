from acados_template import AcadosSim, AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from X_500 import export_x500_ode_model
from utils import plot_trajectory
import numpy as np




X0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
N_horizon = 100
T_horizon = 0.5
Ts = T_horizon /N_horizon
m = 2.0
g = 9.8066 
thrust_to_weight = 1.75
max_force_per_motor = (g * m / 4.0) * thrust_to_weight


# Input constraints
u_min = np.array([0.0, 0.0, 0.0, 0.0])
u_max = np.array([max_force_per_motor, max_force_per_motor, max_force_per_motor, max_force_per_motor])
u_hov = np.array([m*g/4.0, m*g/4.0, m*g/4.0, m*g/4.0])


def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = export_x500_ode_model()

    ocp.model = model

    # set dimensions
    ocp.dims.N = N_horizon

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu
    ny_e = nx

    # set cost

    W = np.diag([10, 10, 10,
             1e-1, 1e-1, 1e-1, 1e-1,
             1e-5, 1e-5, 1e-5,
             1e-5, 1e-5, 1e-5,
             6e-2, 6e-2, 6e-2, 6e-2])

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[-nu:, -nu:] = np.eye(nu)
    

    ocp.dims.N   = N_horizon
    # ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = W
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.W_e = W[:nx, :nx]
    ocp.cost.Vx_e = Vx[:nx, :nx]
    ocp.cost.yref = np.concatenate((X0, u_hov))
    ocp.cost.yref_e = X0
    
    # Constraints
    ocp.constraints.lbu = u_min
    ocp.constraints.ubu = u_max
    ocp.constraints.x0  = X0
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])


        # Solver parameters
    #ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    # ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    #ocp.solver_options.nlp_solver_type = 'SQP'

    #ocp.solver_options.print_level = 0     # Do not print out

     # set prediction horizon
    ocp.solver_options.tf = T_horizon
    ocp.solver_options.N_horizon = N_horizon

    return ocp


def solve_single_ocp():

    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nx = ocp.model.x.size()[0]
    us = []
    xs = []


    current_state = np.array([0.0, 0.0, 1.0, 
                          1.0, 0.0, 0.0, 0.0, 
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0,])
    
  

    # theta = np.linspace(0, 2.0*np.pi, ocp.dims.N+1)
# reference_trajectory = []
# for t in theta:
#     reference_trajectory.append([np.cos(t), np.sin(t), 1.0, 
#                                  1.0, 0.0, 0.0, 0.0, 
#                                  0.0, 0.0, 0.0,
#                                  0.0, 0.0, 0.0,
#                                  max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight])
    
# reference_trajectory = np.array(reference_trajectory)

    reference_trajectory = []
    for t in np.linspace(0, 1, ocp.dims.N+1):
        reference_trajectory.append([t, 0.0, 1.0, 
                                    1.0, 0.0, 0.0, 0.0, 
                                    0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0,
                                    max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight])
        
    reference_trajectory = np.array(reference_trajectory)

    # Fill initial state
    acados_ocp_solver.set(0, "lbx", current_state)
    acados_ocp_solver.set(0, "ubx", current_state)

    acados_ocp_solver.set(0, 'lbu', np.array([max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight]))
    acados_ocp_solver.set(0, 'ubu', np.array([max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight]))

     # initialize solver
    for stage in range(ocp.dims.N):
        acados_ocp_solver.set(stage, "y_ref", reference_trajectory[stage])

    acados_ocp_solver.set(ocp.dims.N, "y_ref", reference_trajectory[ocp.dims.N][:nx])
    status = acados_ocp_solver.solve()

    acados_ocp_solver.print_statistics() # encapsulates: stat = acados_ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

     # get solution
    for i in range(ocp.dims.N+1):
        xs.append(acados_ocp_solver.get(i, "x"))

    for i in range(ocp.dims.N):
        us.append(acados_ocp_solver.get(i, "u"))
        

    us = np.vstack(us)
    xs = np.vstack(xs)

    plot_trajectory(xs,reference_trajectory, us, ocp)
  
solve_single_ocp()

