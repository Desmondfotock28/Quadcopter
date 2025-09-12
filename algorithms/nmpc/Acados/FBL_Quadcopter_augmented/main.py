from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from FBL_Quadcopter_augmented import export_augmented_feedback_lineraise_Quadcopter_ode_model
import numpy as np
import time 
from control import dare 
import scipy.linalg
from utils import plot_3d_trajectory, plot_xyz_subplots, reference_state

w0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

nw = w0.shape[0]
d0 = np.array([0.0, 0.0, 0.0])
Z0 = np.concatenate([w0, d0])
nz = Z0.shape[0]
N_horizon = 10
T_horizon = 1.0
Ts = T_horizon /N_horizon
t0 = 0.0

# Input bounds for v (virtual controls)
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])

ub_v = np.array([ 1.0,  0.05,  0.05,  0.05])


# bounds on disturbance  
lb_d = np.array([-2.0, -2.0, -2.0])
ub_d = np.array([ 2.0,  2.0,  2.0])

nv = lb_v.shape[0]
nd = d0.shape[0]

lb_v_extended = np.hstack([lb_v, -2.0*np.ones(nd)])
ub_v_extended = np.hstack([ub_v,  2.0*np.ones(nd)])



def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = export_augmented_feedback_lineraise_Quadcopter_ode_model()

    ocp.model = model

    # set dimensions
    ocp.dims.N = N_horizon

    # set cost

    # cost type: linear least squares over [z; v]

    # y = [z; v], W = block_diag(Q_aug, R) where Q_aug = block_diag(Q_w, Q_d)

    # state cost (x part)
    Q_w = np.diag([10,2,2,2, 10,2,2,2, 10,2,2,2, 10,10])  # (nw x nw)

    # penalty on virtual control v
    R_v = 0.01 * np.eye(nv)
    
    #penalty on disturbance input 
    R_d = 0.0001*np.eye(nd)   

    # penalty on disturbance d 
    Q_d = np.diag([0, 0, 0])

    Q_aug = scipy.linalg.block_diag(Q_w, Q_d)                           # augmented state cost (nw+nd)

    R_aug = scipy.linalg.block_diag(R_v,  R_d) 

    W_stage =  scipy.linalg.block_diag(Q_aug, R_aug)                      # augmented input cost (nv+nd)
   
    S_aug = scipy.linalg.block_diag(10*Q_w, Q_d)  

    # build Vx and Vu matrices mapping Z and v to y

    Vx = np.vstack([np.eye(nz), np.zeros((nv+nd, nz))])   # (z_dim + v_dim) x z_dim
    Vu = np.vstack([np.zeros((nz, nv+nd)), np.eye(nv+nd)])  # (z_dim+v_dim) x v_dim
   


    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.W = W_stage
    ocp.cost.Vx = Vx
    ocp.cost.Vu = Vu
    ocp.cost.yref = np.zeros(nz + nv + nd)

      # terminal cost on z only:
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.cost.W_e = S_aug
    ocp.cost.Vx_e =   np.eye(nz)
    ocp.cost.yref_e = np.zeros(nz)

    # constraints: set bounds on v_tot (both v and delta) via idxbu
    ocp.constraints.idxbu = np.arange(nv+nd, dtype=int)
    ocp.constraints.lbu = lb_v_extended
    ocp.constraints.ubu = ub_v_extended

    # indices of d in the augmented state
    idx_d = np.array([nw, nw+1, nw+2])  # adjust if d is elsewhere

    # set hard bounds on d
    ocp.constraints.idxbx = idx_d
    ocp.constraints.lbx = lb_d  # lower bound
    ocp.constraints.ubx = ub_d  # upper bound

    ocp.constraints.x0 = Z0



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

    ocp = create_ocp_solver_description()
    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nz = ocp.model.x.size()[0]
    nv = ocp.model.u.size()[0]
    ny = nz + nv 
    simZ = np.ndarray((N_horizon+1, nz))
    simV = np.ndarray((N_horizon, nv))
      
    for k in range(N_horizon):
        acados_ocp_solver.set(k, "yref", reference_state(t0 + k*Ts, ny))
    acados_ocp_solver.set(N_horizon, "yref", reference_state(t0 + T_horizon, nz))  # only states at terminal

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



def closed_loop_simulation():

    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

      # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    Nsim = 400

    nz = ocp.model.x.size()[0]
    nv = ocp.model.u.size()[0]
    ny = nz + nv
    simZ = np.ndarray((Nsim+1, nz))
    simV = np.ndarray((Nsim, nv))
    zcurrent = Z0

    simZ[0,:] = zcurrent
    t0 = 0.0
    t = [t0]
    # closed loop
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", zcurrent)
        acados_ocp_solver.set(0, "ubx", zcurrent)

        # set reference trajectory 

        for k in range(N_horizon):

            acados_ocp_solver.set(k, "yref", reference_state(t0 + k*Ts, ny))

        acados_ocp_solver.set(N_horizon, "yref", reference_state(t0 + N_horizon*Ts, nz))  # only states at terminal

       
        
        # initialize solver
        for stage in range(N_horizon+1):
            acados_ocp_solver.set(stage, 'x', zcurrent)

        #for stage in range(N_horizon):
            #acados_ocp_solver.set(stage, 'u', np.array([1.0, 0.05 , 0.05, 0.05]))

        # solve ocp
        status = acados_ocp_solver.solve()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()

        simV[i,:] = acados_ocp_solver.get(0, "u")

        v0 = simV[i,:]
        
        # simulate system
        acados_integrator.set("x", zcurrent)
        acados_integrator.set("u", v0)

        status = acados_integrator.solve()

        if status != 0:
            raise Exception(f'acados integrator returned status {status} in closed loop instance {i}')

        # update state
        zcurrent = acados_integrator.get("x")
        simZ[i+1,:] = zcurrent

        t0 = t0  +  Ts

        t.append(t0)

    
    t = np.array(t)



    # plot results
    plot_3d_trajectory(t, simZ)
    plot_xyz_subplots(t, simZ)

closed_loop_simulation()
