import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from control import dare
from scipy.linalg import block_diag 





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



def build_prediction_mats(A, B, N):
    """
    Build condensed prediction matrices for horizon N
    without disturbance: W = Sx * w0 + Su * V
    
    Parameters:
    A : (nw, nw) ndarray
        State transition matrix
    B : (nw, nv) ndarray
        Input matrix
    N : int
        Horizon length
    
    Returns:
    Sx : ((N+1)*nw, nw) ndarray
    Su : ((N+1)*nw, N*nv) ndarray
    """
    nw, nv = B.shape
    # Precompute powers of A
    Apows = [np.eye(nw)]
    for k in range(1, N+1):
        Apows.append(Apows[-1] @ A)

    # Sx: stack A^k w0
    Sx = np.vstack(Apows)  # shape ((N+1)*nw, nw)

    # Su: block-Toeplitz matrix of A^(k-1-i) B
    Su = np.zeros(((N+1)*nw, N*nv))
    for k in range(1, N+1):  # row block
        for i in range(k):  # column block
            Su[k*nw:(k+1)*nw, i*nv:(i+1)*nv] = Apows[k-1-i] @ B

    return Sx, Su


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

def shift(T, t0, x0, u, f):
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
    st = f(st, con)
    
    x0 = np.array(st.full()).flatten()

    t0 = t0 + T
    u0 = np.vstack([u[1:], u[-1, :]])

    return t0, x0, u0


def build_blk_cost(Q, S, R, N):
    """
    Build block-diagonal cost matrices Qblk, Rblk
    
    Parameters:
    Q : (nw, nw) ndarray, stage cost on states
    S : (nw, nw) ndarray, terminal cost on state
    R : (nv, nv) ndarray, stage cost on inputs
    N : int, horizon length
    
    Returns:
    Qblk : ((N+1)*nw, (N+1)*nw) ndarray
    Rblk : (N*nv, N*nv) ndarray
    """
    nw = Q.shape[0]
    nv = R.shape[0]
    Qblk = np.zeros(((N+1)*nw, (N+1)*nw))
    Rblk = np.zeros((N*nv, N*nv))

    for k in range(N):
        Qblk[k*nw:(k+1)*nw, k*nw:(k+1)*nw] = Q
        Rblk[k*nv:(k+1)*nv, k*nv:(k+1)*nv] = R
    Qblk[N*nw:(N+1)*nw, N*nw:(N+1)*nw] = S

    return Qblk, Rblk


def stack_reference(ref_fun, t0, Ts, N):
    Ws = []
    for k in range(N+1):
        wk_ref = ref_fun(t0 + k*Ts)
        Ws.append(wk_ref)
    return vertcat(*Ws)


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
 
# Discrete-time system matrices using matrix exponential


# Discretization parameters
dt = 0.3

A_d = np.eye(A.shape[0]) + dt * A
B_d = dt * B

Ts = 0.3   #sampling time in [s]

N = 10    #prediction horizon

tf = 3

# State and input dimensions 
nw = A.shape[1]
nv = B.shape[1]

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

# Terminal cost (solution to Riccati equation)
S, L, K = dare(A_d, B_d, Q, R)

# Stability Analysis of MPC
S = np.array(S)
K = -np.array(K)

#build Hessian matrix 

Sx, Su = build_prediction_mats(A_d, B_d, N)


Qblk, Rblk = build_blk_cost(Q, S, R, N)

t0 = 0.0



v = SX.sym("v", N*nv)

w = SX.sym("w", nw)

vm = SX.sym("vm", nv)

Tr = SX.sym("Tr", 1)

Wref = stack_reference(reference_trajectory, Tr, Ts, N)

# Parameters: initial state 
P = SX.sym('P',nw + 1, 1) 


w_next = A_d @ w + B_d @ vm  #need to confirm logic 

# Create the CasADi function
system = Function("sys", [w, vm], [w_next])

H = Su.T @ Qblk @ Su + Rblk   # Hessian (constant wrt w)
h = Su.T @ Qblk @ (Sx @ w - Wref)   # gradient term (depends on w)



obj = 0.5 * mtimes([v.T, H, v]) + mtimes([h.T, v])  # scalar 

#obj = (Sx@w + Su@v -Wref).T@Qblk@(Sx@w + Su@v -Wref) + v.T@Rblk@v

# CasADi function: parametric in w0
objective = Function("J", [w, v, Tr], [obj])


# Input constraints
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])    #need to check the bound for the transfrom system
ub_v = np.array([1.0, 0.05 , 0.05, 0.05])

lbv = [lb_v]*N
ubv = [ub_v]*N
ubv = vertcat(*ubv)
lbv = vertcat (*lbv)


#defined decision variable 
V = SX.sym('V', nv*N)             

Opt_Vars = vertcat(V)

def objective_cost():
    J = objective(P[:nw],V,P[nw:]) 
    return J


def inequality_constraints():

    hv = []   # Box constraints on active inputs

    hv.append(lbv-V)
    hv.append(V-ubv)

    return hv

def Pi_opt_formulation():

    J = objective_cost()

    hv  = inequality_constraints()
    Hv = vertcat(*hv)
    G_vcsd = vertcat(*hv)
    lbg =  [-np.inf] * (Hv.shape[0])
    ubg =  [0] * (Hv.shape[0])
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
    #pisolver = nlpsol("vsolver", "ipopt", vnlp_prob)
   
    pisolver = qpsol('solver', 'qpoases',vnlp_prob )

    return lbg_vcsd, ubg_vcsd, G_vcsd, pisolver 

lbg_vcsd, ubg_vcsd, G_vcsd , pisolver = Pi_opt_formulation()

def run_open_loop_mpc(w0, v0 , solver ):
      # Initial control inputs and state
    w_pred = []
    v_st_0 = np.tile(v0, (N, 1))

    tr =np.array([0.0])
    
    args_p = np.concatenate([w0, tr])  # Ensure w0 and Tr are concatenated properly
    
    args_p= vertcat(*args_p)
    args_x0 = v_st_0.T.reshape(-1)
   # Solve the optimization problem
    sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    vsol = sol['x']
   # construct xsol 
    wsol = Sx@ w0 + Su @vsol
    print(wsol.shape)
    w_pred.append(wsol)
    # Reshape usol
    v = np.array(sol['x']).reshape((N, nv))
    #Reshape xsol 
    w_pred = vertcat(*w_pred)
    w_pred = np.array(w_pred).reshape((N+1, nw))
 

    return w_pred, v , vsol 

v0 =  np.array([1.0, 0.05 , 0.05, 0.05])

w0 = np.zeros(14)  

w_pred, v_ol, vsol  = run_open_loop_mpc(w0, v0 , pisolver)


t =  np.linspace(0, N*Ts, N+1)

plot_3d_trajectory(t, w_pred)


def run_closed_loop_mpc(w0, Ts, sim_time, solver):
   
    v0 =  np.array([1.0, 0.05 , 0.05, 0.05])
    t0 = 0.0
    nw = w0.shape[0]
    t = [t0]
    w_ol = np.zeros((nw, int(sim_time / Ts) + 1))  # Open loop predicted states
    w_ol = [w0]
    mpc_i = 0
    w_cl = []    # Store predicted states in the closed loop
    v_cl = []    # Store control inputs in the closed loop

    v_st_0 = np.tile(v0, (N, 1))
    tr = np.array([0.0])
    args_p = np.concatenate([w0, tr])  # Ensure w0 and Tr are concatenated properly
    args_p = vertcat(*args_p)

    while  mpc_i < int(sim_time / Ts):
        args_p[:nw] = w0
        args_p[nw:] = np.array([t0])
        args_w0 = v_st_0.T.reshape(-1)

        sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)

        v_opt = sol['x']

        v_seq = np.array(v_opt).reshape(N, nv)
        v_applied = v_seq[0, :]
        t0 = t0 + Ts
    
         #construct wsol
        wsol = Sx@ w0 + Su @v_opt
        
        wsol = np.array(wsol).reshape(N+1, nw)
        w0 = A_d@w0 + B_d@v_applied
        w_pred = np.array(wsol).reshape((N+1, nw))
  
        w_cl.append(w_pred)

        v_cl.append(v_applied)

        t.append(t0)

        w_ol.append(w0) 

        v_st_0 = np.vstack([v_seq[1:], v_seq[-1:]])

        mpc_i += 1

    w_ol = np.array(w_ol)
    v_cl = np.array(v_cl)

    return w_ol, v_cl, t


sim_time = 40


w_ol, v_cl, t = run_closed_loop_mpc(w0, Ts, sim_time, pisolver)

plot_3d_trajectory(t, w_ol)

plot_xyz_subplots(t, w_ol)
        