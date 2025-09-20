import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import *
from control import dare
from scipy.linalg import block_diag 


#Feedback Linearisation for Quadcopter model with constant disturbance

def plot_3d_trajectory(t , w_pred):
    """
    Plot a 3D trajectory given predicted positions.

    Parameters:
    x_pred (numpy.ndarray): Predicted trajectory positions as an (N, 3) array, 
                            where each row represents [x, y, z].

    Returns:
    None
    """
    # Extract x, y, z values
    x_pred_vals = w_pred[:, 0]  # x values
    y_pred_vals = w_pred[:, 4]  # y values
    z_pred_vals = w_pred[:, 8]  # z values

    # Create a 3D figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the predicted trajectory
    t = np.array(t) 
    ax.plot(x_pred_vals, y_pred_vals, z_pred_vals, label="Predicted Trajectory", color='b', linestyle='--')
    xr =  np.sin(np.pi * t/10) 
    yr = np.cos(np.pi * t/10) + -1.0
    zr = np.sin(np.pi * t/10) + t

    ax.plot(xr, yr, zr, label="Reference Trajectory", color='r', linestyle='--')

    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()

def plot_xyz_subplots(t, x_pred):
    """
    Plot distances in X, Y, and Z as three separate subplots.

    Parameters:
    t (array-like): Time steps
    x_pred (numpy.ndarray): Predicted positions as an (N,3) array [x, y, z]

    Returns:
    None
    """
    # Extract x, y, z values
    x_vals = x_pred[:, 0]
    y_vals = x_pred[:, 4]
    z_vals = x_pred[:, 8]

    # Reference trajectory (example)
    t = np.array(t)
    xr = np.sin(np.pi * t / 10)
    yr = np.cos(np.pi * t / 10) - 1.0
    zr = np.sin(np.pi * t / 10) + t
    
    # Create subplots
    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    # Plot X
    axs[0].plot(t, x_vals, label="Predicted X", color='b')
    axs[0].plot(t, xr, label="Reference X", color='r', linestyle='--')
    axs[0].set_ylabel("X")
    axs[0].legend()
    axs[0].grid(True)

    # Plot Y
    axs[1].plot(t, y_vals, label="Predicted Y", color='b')
    axs[1].plot(t, yr, label="Reference Y", color='r', linestyle='--')
    axs[1].set_ylabel("Y")
    axs[1].legend()
    axs[1].grid(True)

    # Plot Z
    axs[2].plot(t, z_vals, label="Predicted Z", color='b')
    axs[2].plot(t, zr, label="Reference Z", color='r', linestyle='--')
    axs[2].set_ylabel("Z")
    axs[2].set_xlabel("Time")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()

def reference_trajectory(t, omega=np.pi, a=0.1):
    """
    Generate the reference trajectory for a given time array.

    Parameters:
    - t: np.ndarray or float, time (can be a single value or an array of values)
    - omega: float, angular frequency (default: 1.0)
    - a: float, slope of the z-direction trajectory (default: 0.1)

    Returns:
    - xr: np.ndarray, reference x-coordinate at time t
    - yr: np.ndarray, reference y-coordinate at time t
    - zr: np.ndarray, reference z-coordinate at time t
    """
    # Compute reference trajectory
        # Compute reference trajectory
    xr = np.sin(omega * t/10) 
    yr = np.cos(omega * t/10)-1.0
    zr = np.sin(omega * t/10) + t
    

    xref = vertcat(xr, np.zeros(3), yr, np.zeros(3), zr, np.zeros(5))

    return xref

def shift(T, t0, x0, u, d_est, d_cons , f):
    """
    Shift the state and time forward by one timestep.

    Args:
        T (float): The timestep.
        t0 (float): The current time.
        x0 (np.array): The current state.
        u (np.array): The control inputs.
        d (np.array): Disturnbance.
        f (Function): The system function.

    Returns:
        tuple: The updated time, state, and control inputs.
    """
    st = x0
    con = u[0, :]
    #f_value = f(st, con)
    #st = st + T * f_value
    st = f(st, con , d_est, d_cons)
    
    x0 = np.array(st.full()).flatten()

    t0 = t0 + T
    u0 = np.vstack([u[1:], u[-1, :]])

    return t0, x0, u0

def get_disturbance(t):
    dwx = 0.15 * np.sin(np.pi * t / 100) + 0.1 * np.sin(0.2 * t) + 0.03 * np.sin(t)
    dwy = 0.0
    dwz = 0.0
    return np.array([dwx, dwy, dwz])


# Continuous-time system matrices  W_dot = AW + BV
# important note  x = w1 , y=w5 , z=w9, psi = w13
A1 = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
    [0.0, 0.0, 0.0, 0.0],
])

A2 = np.array([
    [0.0, 1.0],
    [0.0, 0.0],
])

A = block_diag(A1, A1, A1, A2)

B = np.array([
    [0, 0, 0, 0],  # 1
    [0, 0, 0, 0],  # 2
    [0, 0, 0, 0],  # 3
    [1, 0, 0, 0],  # 4
    [0, 0, 0, 0],  # 5
    [0, 0, 0, 0],  # 6
    [0, 0, 0, 0],  # 7
    [0, 1, 0, 0],  # 8
    [0, 0, 0, 0],  # 9
    [0, 0, 0, 0],  # 10
    [0, 0, 0, 0],  # 11
    [0, 0, 1, 0],  # 12
    [0, 0, 0, 0],  # 13
    [0, 0, 0, 1],  # 14
], dtype=float)


# Discretization parameters
dt = 0.1

A_d = np.eye(A.shape[0]) + dt * A
B_d = dt * B

Ts = 0.1   #sampling time in [s]

N = 10   #prediction horizon

tf = 1

# State and input dimensions 
nw = A.shape[1]
nv = B.shape[1]

# Disturbance injection matrix: injects bias into rows 2, 6, 10
dist_indices = [1, 5, 9]
nd = len(dist_indices)


d_const = np.array([0.12, -0.08, 0.05])   # (nd,)

Bd_dist = np.zeros((nw, nd))
for j, idx in enumerate(dist_indices):
    Bd_dist[idx, j] = 1.0

Bd_dist = dt*Bd_dist
B_const_dist = Bd_dist  # for now 
# Define the CasADi system function using discrete-time matrices
w = SX.sym("w", nw)

v = SX.sym("v", nv)

d_est = SX.sym("d_est", nd)

d_cons = SX.sym('d_cons', nd)  # time-dependent disturbance

w_next = A_d @ w + B_d @ v + Bd_dist@d_est + B_const_dist @ d_cons  #need to confirm logic 

# Create the CasADi function
system = Function("sys", [w, v , d_est,  d_cons ], [w_next])

# Define initial state

w0 = np.zeros(nw) 

# Define cost function parameters

# Declear empty sys matrices
V = SX.sym('V',nv,N)               # Decision variables (controls)

#Parameters:initial state(x0)

P = SX.sym('P',nw + 1, 1) 

W= SX.sym('W',nw,(N+1)) # Decision variables (states)

D = SX.sym("D", nd, N+1)   # Disturbance trajectory

Q = np.diag([
        10,  # w1 (x-position)
        2,  # w2 
        2,  # w3 
        2,   # w4 
        10,   # w5 (y-position)
        2,   # w6 
        2,   # w7 
        2,   # w8 
        10,   # w9 (altitude)
        2,   # w10 
        2,   # w11 
        2,    # w12 
        10,   # w13 (yaw )
        10    #w14
    ])
R = 0.01
R = R*np.diag(np.ones(nv))

#Qd = 1e-6 * np.eye(nd)
#+ bilin(Qd ,d) +  , d

# Define the stage cost and terminal cost 
stage_cost =  0.5*(bilin(Q, w) +  bilin(R, v))

stage_cost_fcn = Function("cost", [w, v], [stage_cost])

# Terminal cost (solution to Riccati equation)
S, L, K = dare(A_d, B_d, Q, R)

# Stability Analysis of MPC
S = np.array(S)
K = -np.array(K)

# Closed-loop system
A_cl = A_d + B_d @ K

# Eigenvalues of close loop
#print(np.linalg.eig(A_cl)[0])
terminal_cost = bilin(S, w)
#terminal_cost = 100*mtimes(w.T, w)

terminal_cost_fcn = Function("T_cost", [w], [terminal_cost])

# Input constraints
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])    #need to check the bound for the transfrom system
ub_v = np.array([1.0, 0.05 , 0.05, 0.05])
lb_d= np.array([-2.0, -2.0, -2.0])
ub_d= np.array([2.0, 2.0, 2.0])

vmax = 1.0

Opt_Vars = vertcat(
    reshape(W, -1, 1),
    reshape(V, -1, 1),
    reshape(D, -1, 1)
)
lam_d = 1e-2          # smoothness weight: larger -> D varies less across horizon

def objective_cost():
    J = 0.0
    for i in range(N):
        dw = W[:, i+1]-reference_trajectory(P[nw:] + i*Ts)
        dv = V[:, i]      #-vmax
        dd = D[:, i+1] - D[:, i]

        J += stage_cost_fcn(dw, dv)
        J += 0.5 * lam_d * mtimes(dd.T, dd)  # scalar

    J += terminal_cost_fcn((W[:, -1]-reference_trajectory(P[nw:] + N*Ts)))      #+  bilin(Qd ,D[:, -1]) 
    return J

def equality_constraints():
    g = []  # Equality constraints initialization
    g.append(W[:, 0] - P[:nw])  # Initial state constraint
  
    for i in range(N):
        st = W[:, i]
        cons = V[:, i] 
        d_i = D[:, i]
        #st_next_euler = system(st,cons)
        st_next_model =  A_d @ st + B_d @ cons + Bd_dist @ d_i   # (need to clean this)
        st_next = W[:, i+1]
        g.append(st_next -  st_next_model)

    #terminal set constraints 
    #g.append(W[:, -1]-reference_trajectory(P[nw:] + N*Ts))
    return g

def inequality_constraints():
    
    hv = []   # Box constraints on virtual inputs 
    hd = []   # Box constraints on disturbance 
    for i in range(N):
        hv.append(lb_v - V[:, i]) 
        hv.append(V[:, i] - ub_v) 
        hd.append(lb_d - D[:, i]) 
        hd.append(D[:, i] - ub_d)
    hd.append(lb_d - D[:, -1])
    hd.append(D[:, -1]-ub_d)
    return  hv, hd

def Pi_opt_formulation():
    J = objective_cost()
    g = equality_constraints()
    G = vertcat(*g)
    hv, hd = inequality_constraints()
    Hv = vertcat(*hv)
    H_D = vertcat(*hd)
    G_vcsd = vertcat(*g, *hv, *hd)
    lbg = [0] * G.shape[0] + [-np.inf] * (Hv.shape[0] + H_D.shape[0])
    ubg = [0] * G.shape[0] + [0] * (Hv.shape[0] + H_D.shape[0])
    lbg_vcsd = vertcat(*lbg)
    ubg_vcsd = vertcat(*ubg)

    opts_setting = {
        "ipopt.max_iter": 500,
        "ipopt.print_level": 4,
        "print_time": 1,
        "ipopt.acceptable_tol": 1e-6,
        "ipopt.acceptable_obj_change_tol": 1e-6,
    }
    vnlp_prob = {
        "f": J,
        "x": Opt_Vars,
        "p": vertcat(P),
        "g": G_vcsd,
    }
    pisolver = nlpsol("vsolver", "ipopt", vnlp_prob)

    return lbg_vcsd, ubg_vcsd, G_vcsd, pisolver 

lbg_vcsd, ubg_vcsd, G_vcsd , pisolver = Pi_opt_formulation()




def run_open_loop_mpc(w0, t0 , v0 , solver ):
      # Initial control inputs and state
    d0 = np.zeros(nd)
    v_st_0 = np.tile(v0, (N, 1))
    w_st_0 = np.tile(w0, (N + 1, 1)).T
    d_st_0 = np.tile(d0, (N + 1, 1)).T
    
    args_p = np.concatenate([w0, t0 ])  # Ensure x0 and Tr are concatenated properly

    args_p= vertcat(*args_p)

    args_w0 = np.concatenate([w_st_0.T.reshape(-1), v_st_0.T.reshape(-1), d_st_0.T.reshape(-1)])
   # Solve the optimization problem
    sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    vsol = sol['x'][nw * (N+1):nw * (N+1) + nv*N]
    # Extract the control inputs from the solution
    v = np.array(sol['x'][nw * (N+1):nw * (N+1) + nv*N]).reshape((N, nv))

    #extract predicted state 
    w_pred = np.array(sol['x'][:nw * (N+1)]).reshape((N+1, nw))

    # Convert lists to numpy arrays for easier handling
    w_pred = np.array(w_pred) 
    v = np.array(v) 
    return w_pred, v , vsol

v0 =  np.array([1.0, 0.05 , 0.05, 0.05])

w0 = np.zeros(14)  

Tr =np.array([0.0])

w_pred, v_ol, vsol = run_open_loop_mpc(w0, Tr, v0 , pisolver)

t =  np.linspace(0, N*Ts, N+1)

plot_3d_trajectory(t, w_pred)


def run_closed_loop_mpc(w0, Tr, Ts, sim_time, solver):
   
    v0 =  np.array([1.0, 0.05 , 0.05, 0.05])
    d0 = np.zeros(nd)
    t0 = 0
    nw = w0.shape[0]
    t = [t0]
    w_ol = np.zeros((nw, int(sim_time / Ts) + 1))  # Open loop predicted states
    w_ol = [w0]
    mpc_i = 0
    w_cl = []    # Store predicted states in the closed loop
    v_cl = []    # Store control inputs in the closed loop

    v_st_0 = np.tile(v0, (N, 1))
    w_st_0 = np.tile(w0, (N + 1, 1)).T
    d_st_0 = np.tile(d0, (N + 1, 1)).T
    d_known = d_const

    args_p = np.concatenate([w0, Tr])  # Ensure x0 and Tr are concatenated properly

    args_p = vertcat(*args_p)
    cost = []
    time_full = []
    V_open_loop = []
    d_actual =[]
    d_predicted = []
  

    while  mpc_i < int(sim_time / Ts):
        args_p[:nw] = w0
        args_p[nw:] = np.array([t0])
        args_w0 = np.concatenate([w_st_0.T.reshape(-1), v_st_0.T.reshape(-1), d_st_0.T.reshape(-1)])
        start_time = time.time()
        sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full.append(solver_time)
        w_opt = sol['x']
        Dsol =  w_opt[nw * (N+1) + nv*N:]
        d_est = Dsol[:nd]
        wsol = np.array(w_opt[:nw*(N+1)]).reshape(N+1, nw)
        vsol = np.array(w_opt[nw*(N+1): nw*(N+1)+nv*N]).reshape(N, nv)
        dsol = np.array(w_opt[nw*(N+1)+nv*N:]).reshape(N+1, nd)
        V_open_loop.append(vsol)

        cost.append(sol['f'])
       
        w_cl.append(wsol)
        v_cl.append(vsol[0, :])
        
        #d_known =  get_disturbance(t0)
        d_actual.append(d_known[0])
        d_predicted.append(d_est[0])

        t0, w0, v0 =shift(Ts, t0, w0, vsol, d_est, d_known,  system)

        t.append(t0)
    
        w_ol.append(w0)
        w_st_0 = np.vstack([wsol[1:],  wsol[-1:]])
        v_st_0 = np.vstack([vsol[1:],  vsol[-1:]])
        d_st_0 = np.vstack([dsol[1:],  dsol[-1:]])

        mpc_i += 1

    w_ol = np.array(w_ol)
    v_cl = np.array(v_cl)
    d_actual = np.array(d_actual)
    d_predicted = np.array(d_predicted)

    return w_ol, v_cl, t, cost , time_full, V_open_loop, d_actual,d_predicted

# Run the closed-loop MPC for 10s

sim_time = 40
d0 = np.zeros(nd)

w_ol, v_cl, t, cost_n, time_full, V_open_loop,d_actual, d_predicted = run_closed_loop_mpc(w0, Tr,  Ts, sim_time, pisolver)

print(np.mean(time_full))

plot_3d_trajectory(t, w_ol)

plot_xyz_subplots(t, w_ol)


def plot_disturbance_x(t, d):
    """
    Plot actual and predicted disturbances in the x-direction over time.
    
    Args:
        t_hist (list or np.array): Time steps
        d_actual_hist (np.array): Actual disturbances (Nx3 or Nx1)
        d_pred_hist (np.array): Predicted disturbances (Nx3 or Nx1)
    """
    t = np.array(t)
    

    plt.figure(figsize=(8,4))
    plt.plot(t, d, 'r', label='Actual dx')
    plt.xlabel('Time [s]')
    plt.ylabel('Disturbance [dx]')
    plt.title('Disturbance in x-direction')
    plt.grid(True)
    plt.legend()
    plt.show()

plot_disturbance_x(t[:-1], d_actual)
plot_disturbance_x(t[:-1], d_predicted.flatten())

"""
cost function for reference traj:
 omega = np.pi
 xr = np.sin(omega * t/10) 
 yr = np.cos(omega * t/10)-1.0
 zr = np.sin(omega * t/10) + t

Q = np.diag([
        10,  # w1 (x-position)
        2,  # w2 
        2,  # w3 
        2,   # w4 
        10,   # w5 (y-position)
        2,   # w6 
        2,   # w7 
        2,   # w8 
        10,   # w9 (altitude)
        2,   # w10 
        2,   # w11 
        2,    # w12 
        10,   # w13 (yaw )
        10    #w14
    ])
R = 0.01
R = R*np.diag(np.ones(nv))

"""