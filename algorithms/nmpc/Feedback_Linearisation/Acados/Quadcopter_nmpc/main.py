from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from Quadcopter import export_Quadcopter_ode_model
import numpy as np
import time
import scipy.linalg
from utils import plot_3d_trajectory, plot_xyz_subplots,  reference_trajectory

nx = 12
X0 = np.zeros(nx)
N_horizon = 10
T_horizon = 1.0
Ts = T_horizon /N_horizon


# Input constraints
lb_u = np.array([0.5, 0.5, 0.5, 0.5])
ub_u = np.array([11, 11 , 11, 11])
umax = 11

nu = lb_u.shape[0]



def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = export_Quadcopter_ode_model()

    ocp.model = model

    # set dimensions
    ocp.dims.N = N_horizon

    # set default parameter values (reference state)
    ocp.parameter_values = np.zeros(nx)


    # set cost

    Q_mat = np.diag([
        40,  # x (position)
        40,  # y (position)
        50,  # z (altitude)
        5,   # roll (phi)
        5,   # pitch (theta)
        5,   # yaw (psi)
        2,   # velocity in x (v_x)
        2,   # velocity in y (v_y)
        2,   # velocity in z (v_z)
        1,   # angular velocity roll (w_x)
        1,   # angular velocity pitch (w_y)
        1    # angular velocity yaw (w_z)
    ])

    R = 0.1
    R_mat = R*np.diag(np.ones(nu))
    

    # the 'EXTERNAL' cost type can be used to define general cost terms
    # NOTE: This leads to additional (exact) hessian contributions when using GAUSS_NEWTON hessian.
    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = (model.x- model.p).T @ Q_mat @ (model.x - model.p) + (model.u - umax).T @ R_mat @ (model.u - umax)
    ocp.model.cost_expr_ext_cost_e = (model.x - model.p).T @ Q_mat @ (model.x - model.p)
    

     # constraints: set bounds on u idxbu
    ocp.constraints.idxbu = np.arange(nu)   # indices of bounded inputs
    ocp.constraints.lbu = lb_u
    ocp.constraints.ubu = ub_u

    ocp.constraints.x0 = X0


    # solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.print_level = 1
    ocp.solver_options.nlp_solver_type = 'SQP' # SQP_RTI, SQP

    ocp.solver_options.nlp_solver_max_iter = 100

     # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp




def solve_single_ocp():

    t0 = 0.0

    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    simX = np.ndarray((N_horizon+1, nx))
    simU = np.ndarray((N_horizon, nu))
      
    for k in range(N_horizon):
        acados_ocp_solver.set(k, "p", reference_trajectory(t0 + k*Ts))
    acados_ocp_solver.set(N_horizon, "p", reference_trajectory(t0 + T_horizon))  # only states at terminal

    start_time = time.time()
    status = acados_ocp_solver.solve()
    solver_time = time.time()-start_time

    acados_ocp_solver.print_statistics() # encapsulates: stat = acados_ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

     # get solution
    for i in range(N_horizon):
        simX[i,:] = acados_ocp_solver.get(i, "x")
        simU[i,:] = acados_ocp_solver.get(i, "u")
    simX[N_horizon,:] = acados_ocp_solver.get(N_horizon, "x")

    plot_3d_trajectory(np.linspace(0, T_horizon, N_horizon+1),simX)

    print(solver_time)



def closed_loop_simulation():

    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

      # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')
   
    Nsim = 400


    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))

    xcurrent = X0
    simX[0,:] = xcurrent
    t0 = 0.0
    t = [t0]
    # closed loop
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

        # set reference trajectory 

        for k in range(N_horizon):
           
            acados_ocp_solver.set(k, "P", reference_trajectory(t0 + k*Ts) )
        acados_ocp_solver.set(N_horizon, "p", reference_trajectory(t0 + N_horizon*Ts))  # only states at terminal

       
        
        # initialize solver
        for stage in range(N_horizon+1):
            acados_ocp_solver.set(stage, 'x', xcurrent)

        #for stage in range(N_horizon):
            #acados_ocp_solver.set(stage, 'u', np.array([5.25, 5.25 , 5.25, 5.25]))

        # solve ocp
        status = acados_ocp_solver.solve()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            raise Exception(f'acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent=}')

        simU[i,:] = acados_ocp_solver.get(0, "u")
        u0= simU[i,:]


        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", simU[i,:])

        status = acados_integrator.solve()

        if status != 0:
            raise Exception(f'acados integrator returned status {status} in closed loop instance {i}')

       # update state
        xcurrent = acados_integrator.get("x")
        simX[i+1,:] = xcurrent
        t0 = t0  +  Ts

        t.append(t0)

    
    t = np.array(t)



    # plot results
    plot_3d_trajectory(t, simX)
    plot_xyz_subplots(t, simX)

closed_loop_simulation()