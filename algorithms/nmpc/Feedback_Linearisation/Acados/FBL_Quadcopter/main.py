from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from FBL_Quadcopter import export_feedback_lineraise_Quadcopter_ode_model
import numpy as np
import time
import scipy.linalg
from control import dare
from utils import plot_3d_trajectory, plot_xyz_subplots, reference_state, get_continous_time_matrices


w0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

nw = w0.shape[0]

N_horizon = 10
T_horizon = 1.0
Ts = T_horizon /N_horizon



# Input bounds for v (virtual controls)
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])

ub_v = np.array([ 1.0,  0.05,  0.05,  0.05])

nv = lb_v.shape[0]


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

    # cost type: linear least squares over [w; v]

    # y = [w; v], W = block_diag(Q_w, R_v) 

    # state cost (x part)
    Q_w = np.diag([
            10,  # w1 (x-position)
            2,  # w2 
            2,  # w3 
            2,   # w4 
            10,   # w5 (y-position)
            2,   # w6 
            2,   # w7 
            2,   # w8 
            10,   # w9 (altitude)
            10,   # w10 
            10,   # w11 
            10,    # w12 
            10,   # w13 (yaw )
            10    #w14
        ]) # (nw x nw)

     # penalty on virtual control v

    R_v = 0.01
    R_v = R_v*np.diag(np.ones(nv))

    W_stage =  scipy.linalg.block_diag(Q_w, R_v)


    # build Vx and Vu matrices mapping z and v to y

    Vx = np.vstack([np.eye(nw), np.zeros((nv, nw))])   # (w_dim + v_dim) x z_dim
    Vu = np.vstack([np.zeros((nw, nv)), np.eye(nv)])  # (w_dim+v_dim) x v_dim

    # set prediction horizon
    ocp.solver_options.tf = T_horizon

    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = W_stage
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.yref = np.zeros(nw + nv)


     # terminal cost on w only:
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e = 10*Q_w
    ocp.cost.Vx_e =   np.eye(nw)
    ocp.cost.yref_e = np.zeros(nw)


     # constraints: set bounds on v idxbu
    ocp.constraints.idxbu = np.arange(nv)   # indices of bounded inputs
    ocp.constraints.lbu = lb_v
    ocp.constraints.ubu = ub_v

    ocp.constraints.x0 = w0


    # solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.sim_method_num_stages = 1
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.print_level = 1

    ocp.solver_options.nlp_solver_max_iter = 100

     # set prediction horizon
    ocp.solver_options.tf = T_horizon

    return ocp




def solve_single_ocp():

    t0 = 0.0

    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nw = ocp.model.x.size()[0]
    nv = ocp.model.u.size()[0]
    ny = nw + nv
    simW = np.ndarray((N_horizon+1, nw))
    simV = np.ndarray((N_horizon, nv))
      
    for k in range(N_horizon):
        acados_ocp_solver.set(k, "yref", reference_state(t0 + k*Ts, ny))
    acados_ocp_solver.set(N_horizon, "yref", reference_state(t0 + T_horizon, nw))  # only states at terminal

    start_time = time.time()
    status = acados_ocp_solver.solve()
    solver_time = time.time()-start_time

    acados_ocp_solver.print_statistics() # encapsulates: stat = acados_ocp_solver.get_stats("statistics")

    if status != 0:
        raise Exception(f'acados returned status {status}.')

    # get solution
    for i in range(N_horizon):
        simW[i,:] = acados_ocp_solver.get(i, "x")
        simV[i,:] = acados_ocp_solver.get(i, "u")
    simW[N_horizon,:] = acados_ocp_solver.get(N_horizon, "x")

    plot_3d_trajectory(np.linspace(0, T_horizon, N_horizon+1),simW)

    print(solver_time)



def closed_loop_simulation():

    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

      # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')
   
    Nsim = 400

    nw = ocp.model.x.size()[0]
    nv = ocp.model.u.size()[0]
    ny = nw + nv
    simW = np.ndarray((Nsim+1, nw))
    simV = np.ndarray((Nsim, nv))
    wcurrent = w0

    simW[0,:] = wcurrent
    t0 = 0.0
    t = [t0]
    # closed loop
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", wcurrent)
        acados_ocp_solver.set(0, "ubx", wcurrent)

        # set reference trajectory 

        for k in range(N_horizon):
           
            acados_ocp_solver.set(k, "yref", reference_state(t0 + k*Ts, ny) )
        acados_ocp_solver.set(N_horizon, "yref", reference_state(t0 + N_horizon*Ts, nw))  # only states at terminal

       
        
        # initialize solver
        for stage in range(N_horizon+1):
            acados_ocp_solver.set(stage, 'x', wcurrent)

        #for stage in range(N_horizon):
            #acados_ocp_solver.set(stage, 'u', np.array([1.0, 0.05 , 0.05, 0.05]))

        # solve ocp
        status = acados_ocp_solver.solve()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()

        simV[i,:] = acados_ocp_solver.get(0, "u")
        v0= simV[i,:]
        # simulate system
     
        
        acados_integrator.set("x", wcurrent)
        acados_integrator.set("u", v0)

        status = acados_integrator.solve()

        if status != 0:
            raise Exception(f'acados integrator returned status {status} in closed loop instance {i}')

        # update state
        wcurrent = acados_integrator.get("x")
        simW[i+1,:] = wcurrent

        t0 = t0  +  Ts

        t.append(t0)

    
    t = np.array(t)



    # plot results
    plot_3d_trajectory(t, simW)
    plot_xyz_subplots(t, simW)

closed_loop_simulation()