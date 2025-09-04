from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from FBL_Quadcopter import export_feedback_lineraise_Quadcopter_ode_model
import numpy as np
import time
import scipy.linalg
from control import dare
from utils import plot_3d_trajectory, plot_xyz_subplots, reference_state, get_continous_time_matrices


w0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
d0 = np.array([0.0, 0.0, 0.0])
Z0 = np.concatenate([w0, d0])

nw = w0.shape[0]
nz = Z0.shape[0]


N_horizon = 10
T_horizon = 1.0
Ts = T_horizon /N_horizon
t0 = 0.0


# Input bounds for v (virtual controls)
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])

ub_v = np.array([ 1.0,  0.05,  0.05,  0.05])

nv = lb_v.shape[0]
nd = d0.shape[0]

A, B, _ = get_continous_time_matrices()


def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = export_feedback_lineraise_Quadcopter_ode_model()

    ocp.model = model

    # set dimensions
    ocp.dims.N = N_horizon


    # set cost

    # cost type: linear least squares over [z; v]

    # y = [z; v], W = block_diag(Q_z, R_v) 

    # state cost (x part)
    Q_w = np.diag([10,2,2,2, 10,2,2,2, 10,2,2,2, 10,10])  # (nw x nw)

     # penalty on virtual control v
    R_v = 0.01 * np.eye(nv)

    # Terminal cost (solution to Riccati equation)
    S, _, _ = dare(A, B, Q_w, R_v)

    # Stability Analysis of MPC
    S = np.array(S)

    # penalty on disturbance d 
    Q_d = np.diag([0.001, 0.001, 0.001])

    Q_aug = scipy.linalg.block_diag(Q_w, Q_d)

    S_aug = scipy.linalg.block_diag(S, Q_d)


    W_stage =  scipy.linalg.block_diag(Q_aug, R_v)


    # build Vx and Vu matrices mapping z and v to y

    Vx = np.vstack([np.eye(nz), np.zeros((nv, nz))])   # (z_dim + v_dim) x z_dim
    Vu = np.vstack([np.zeros((nz, nv)), np.eye(nv)])  # (z_dim+v_dim) x v_dim

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = W_stage
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.yref = np.zeros(nw + nv + nd)


     # terminal cost on z only:
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e = S_aug
    ocp.cost.Vx_e =   np.eye(nz)
    ocp.cost.yref_e = np.zeros(nz)


     # constraints: set bounds on v idxbu
    ocp.constraints.idxbu = np.arange(nv)   # indices of bounded inputs
    ocp.constraints.lbu = lb_v
    ocp.constraints.ubu = ub_v

    ocp.constraints.x0 = Z0


    # solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.print_level = 1

    ocp.solver_options.nlp_solver_max_iter = 100

     # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp




def solve_single_ocp():

    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nz = ocp.model.x.size()[0]
    nv = ocp.model.u.size()[0]
    ny = nz + nv
    simZ = np.ndarray((N_horizon+1, nz))
    simV = np.ndarray((N_horizon, nv))
      
    for k in range(N_horizon):
        acados_ocp_solver.set(k, "yref", reference_state(k*Ts, ny))
    acados_ocp_solver.set(N_horizon, "yref", reference_state(T_horizon, nz))  # only states at terminal

    start_time = time.time()
    status = acados_ocp_solver.solve()
    solver_time = time.time()-start_time

    acados_ocp_solver.print_statistics() # encapsulates: stat = acados_ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N_horizon):
        simZ[i,:] = acados_ocp_solver.get(i, "x")
        simV[i,:] = acados_ocp_solver.get(i, "u")
    simZ[N_horizon,:] = acados_ocp_solver.get(N_horizon, "x")

    plot_3d_trajectory(np.linspace(0, T_horizon, N_horizon+1),simZ)

    print(solver_time)

solve_single_ocp()