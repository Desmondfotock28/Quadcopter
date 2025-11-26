from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from Quadcopter import export_Quadcopter_ode_model
import numpy as np
import time
from utils import generate_spiral_trajectory, plot_3d_trajectory_test,plot_xyz_subplots_test




X0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

N_horizon = 50

T_horizon = 0.5

m = 1.65

g = 9.8066 

Ts = T_horizon /N_horizon

thrust_to_weight = 2.85

max_force_per_motor = (g * m / 4.0) * thrust_to_weight

# Input constraints
u_min = np.array([0.0, 0.0, 0.0, 0.0])

u_max = np.array([max_force_per_motor, max_force_per_motor, max_force_per_motor, max_force_per_motor])

u_hov = np.array([m*g/4.0, m*g/4.0, m*g/4.0, m*g/4.0])

def create_ocp_solver_description() -> AcadosOcp:
    # create ocp object to formulate the OCP
    ocp = AcadosOcp()
 
    # set model
    model = export_Quadcopter_ode_model()

    ocp.model = model

    # set dimensions
    ocp.dims.N = N_horizon

    nx = model.x.size()[0]
    nu = model.u.size()[0]
    ny = nx + nu

    # set cost
    #W = np.diag([10.0, 10.0, 40.0,
       # 8.0,  8.0,  8.0, 8.0,
        #1e-5, 1e-5, 1e-5,
        #1e-5, 1e-5, 1e-5,
        #6e-2, 6e-2, 6e-2, 6e-2])
    
    W = np.diag([
    30.0, 30.0, 50.0,        # positions x, y, z (higher X/Y to reduce lateral deviation)
    8.0, 8.0, 8.0, 8.0,      # quaternions q0, q1, q2, q3 (higher to limit orientation swings)
    5.0, 5.0, 5.0,           # Linear velocities (vx,vy,vz)
    1.0, 1.0, 1.0,           # angular rates (wx,wy,wz)
    0.2, 0.2, 0.2, 0.2       # input forces (slightly higher to reduce aggressive thrust changes)
])





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

    ocp.solver_options.print_level = 0     # Do not print out

     # set prediction horizon
    ocp.solver_options.tf = T_horizon
    ocp.solver_options.N_horizon = N_horizon

    return ocp

starting_point = (0, 0, 3)
radius = 3
steps = 10000
height = 2
length = 10

generate_spiral_trajectory(starting_point, radius, steps, height)

ref_traj = np.loadtxt("/home/udeme/Quadcopter/nmpc_px4_ros2_ws/src/nmpc_px4_ros2/scripts/spiral.txt")


def solve_single_ocp():
     
    t0 = 0.0
    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

    nx = ocp.model.x.size()[0]

    nu = ocp.model.u.size()[0]

    simX = np.ndarray((N_horizon+1, nx))

    simU = np.ndarray((N_horizon, nu))

    current_state = X0

     # Fill initial state
    acados_ocp_solver.set(0, "lbx", current_state)
    acados_ocp_solver.set(0, "ubx", current_state)

    acados_ocp_solver.set(0, 'lbu', np.array([max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight]))
    acados_ocp_solver.set(0, 'ubu', np.array([max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight, max_force_per_motor/thrust_to_weight]))

     # initialize solver

   
    for k in range(N_horizon):
        traj_index = int((t0 / Ts) + k)
        traj_index = min(traj_index, len(ref_traj) - 1)  # prevent out of bounds
        acados_ocp_solver.set(k, "yref", ref_traj[traj_index, :nx+nu])

    traj_index = int((t0 / Ts) + N_horizon)
    traj_index = min(traj_index, len(ref_traj) - 1)
    acados_ocp_solver.set(N_horizon, "yref", ref_traj[traj_index, :nx])  # only states at terminal
     
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

    plot_3d_trajectory_test(simX, ref_traj)
    
    print(solver_time)

solve_single_ocp()



def closed_loop_simulation():

    ocp = create_ocp_solver_description()

    acados_ocp_solver = AcadosOcpSolver(ocp, json_file = 'acados_ocp_' + ocp.model.name + '.json')

      # create an integrator with the same settings as used in the OCP solver.
    acados_integrator = AcadosSimSolver(ocp, json_file = 'acados_sim_' + ocp.model.name + '.json')
   
    Nsim = 10000

    nx = ocp.model.x.size()[0]
    nu = ocp.model.u.size()[0]
    simX = np.ndarray((Nsim+1, nx))
    simU = np.ndarray((Nsim, nu))
   
    u0 = u_max 

    xcurrent = X0
    simX[0,:] = xcurrent
    t0 = 0.0
    t = [t0]
    solve_time_total =[]

    # initialize solver
    
    for stage in range(N_horizon):
            acados_ocp_solver.set(stage, 'u', u0)

    for stage in range(N_horizon+1):
            acados_ocp_solver.set(stage, 'x', xcurrent)

    # closed loop
    for i in range(Nsim):

        # set initial state constraint
        acados_ocp_solver.set(0, "lbx", xcurrent)
        acados_ocp_solver.set(0, "ubx", xcurrent)

            
        
        # set reference trajectory 

        for k in range(N_horizon):
            traj_index = int((t0 / Ts) + k)
            traj_index = min(traj_index, len(ref_traj) - 1)  # prevent out of bounds
            acados_ocp_solver.set(k, "yref", ref_traj[traj_index, :nx+nu])
        
        traj_index = int((t0 / Ts) + N_horizon)
        traj_index = min(traj_index, len(ref_traj) - 1)
        acados_ocp_solver.set(N_horizon, "yref", ref_traj[traj_index, :nx])

       # acados_ocp_solver.set(N_horizon, "yref", ref_traj[t0 + T_horizon][:nx])  # only states at terminal
        
        # solve ocp

        start_time = time.time()
        status = acados_ocp_solver.solve()
        solver_time = time.time()-start_time
        solve_time_total.append(solver_time)

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            raise Exception(f'acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent=}')

        simU[i,:] = acados_ocp_solver.get(0, "u")
        u0 = simU[i,:]


        # simulate system
        acados_integrator.set("x", xcurrent)
        acados_integrator.set("u", u0)

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
    solve_time_total = np.array(solve_time_total)

    plot_3d_trajectory_test(simX, ref_traj)
    plot_xyz_subplots_test(t, ref_traj, simX)

    print(np.mean(solve_time_total))

#closed_loop_simulation()





