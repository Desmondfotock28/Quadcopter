import numpy as np
import time
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

    #xr = 0.5 + 0.2* np.cos(t)
    #yr = 0.5 + 0.2*np.sin(t) 
    #zr = 1.1 + 0.1*t

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

    #xr = 0.5 + 0.2* np.cos(t)
    #yr = 0.5 + 0.2*np.sin(t) 
    #zr = 1.1 + 0.1*t
    
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


def build_prediction_mats(A,B,Bd,N):
    """Build Sx, Su, Sd for single-shooting with constant disturbance d."""
    nw, nv = B.shape
    nd = Bd.shape[1]
    # Powers of A
    Apows = [np.eye(nw)]
    for k in range(1, N+1):
        Apows.append(Apows[-1] @ A)

    # Sx: stack A^k
    Sx = np.vstack(Apows)  # (nw*(N+1), nw)

    # Su: lower block Toeplitz of A^{k-1-i} B
    Su = np.zeros((nw*(N+1), nv*N))
    for k in range(1, N+1):
        for i in range(k):
            Su[k*nw:(k+1)*nw, i*nv:(i+1)*nv] = Apows[k-1-i] @ B

    Sd_var = np.zeros(((N+1)*nw, N*nd))
     # k = 0 -> w_0 has no disturbance contribution (row zeros)
    for k in range(1, N+1):            # w_k row
        row_start = k * nw
    # d_j for j=0..k-1 contributes Apows[k-1-j] @ Bd_dist
        for j in range(k):
           col_start = j * nd
           Sd_var[row_start:row_start+nw, col_start:col_start+nd] = Apows[k-1-j] @ Bd


    return Sx, Su, Sd_var

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

    #xr = 0.5 + 0.2* np.cos(t)
    #yr = 0.5 + 0.2*np.sin(t) 
    #zr = 1.1 + 0.1*t
    

    xref = vertcat(xr, np.zeros(3), yr, np.zeros(3), zr, np.zeros(5))
   

    return xref

def get_disturbance(t):

    dwx = 0.15 * np.sin(np.pi * t / 100) + 0.1 * np.sin(0.2 * t) + 0.03 * np.sin(t)
    dwy = 0.0
    dwz = 0.0
    return np.array([dwx, dwy, dwz])


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
dt = 0.1

A_d = np.eye(A.shape[0]) + dt * A
B_d = dt * B


Ts = 0.1   #sampling time in [s]

N =  10    #prediction horizon

tf = 1.0

# State and input dimensions 
nw = A.shape[1]
nv = B.shape[1]

# Disturbance injection matrix: injects bias into rows 2, 6, 9
dist_indices = [1, 5, 9]
nd = len(dist_indices)

Bd_dist = np.zeros((nw, nd))
for j, idx in enumerate(dist_indices):
    Bd_dist[idx, j] = 1.0

Bd_dist = dt*Bd_dist

B_const_dist = Bd_dist  # for now 

d_const = np.array([0.12, -0.08, 0.05])   # (nd,)
 
nd = Bd_dist.shape[1]

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

#computing block diagonal matrices 
Sx, Su, Sd = build_prediction_mats(A_d, B_d, Bd_dist, N)

#Hard coded matrices
Qblk, Rblk = build_blk_cost(Q,Q,R,N)

# H and h
H11 = Su.T @ Qblk @ Su + Rblk
H12 = Su.T @ Qblk @ Sd
H22 = Sd.T @ Qblk @ Sd
H = np.block([[H11, H12],
                  [H12.T, H22]])


v = SX.sym("v", N*nv)

d = SX.sym("d", N*nd)

z = vertcat(v, d)


w = SX.sym("w", nw)

vm = SX.sym("vm", nv)

Tr = SX.sym("Tr", 1)

P = SX.sym('P',nw + 1, 1) 

t0 = 0.0


Wref = stack_reference(reference_trajectory, Tr, Ts, N)


h1 = Su.T @ Qblk @ (Sx @ w - Wref)
h2 = Sd.T @ Qblk @ (Sx @ w - Wref)
h = vertcat(h1, h2)


# Define the stage cost and terminal cost 

obj = 0.5 * mtimes([z.T, H, z]) + mtimes([h.T, z])  # scalar 

# CasADi function: parametric in w0
objective = Function("J", [w, z, Tr], [obj])

# Input constraints
lb_v = np.array([-1.0, -0.05, -0.05, -0.05])    #need to check the bound for the transfrom system
ub_v = np.array([1.0, 0.05 , 0.05, 0.05])
lb_d= np.array([-2.0, -2.0, -2.0])
ub_d= np.array([2.0, 2.0, 2.0])

lbz = [lb_v]*N +[lb_d]*N
ubz =   [ub_v]*N +[ub_d]*N
ubz = vertcat(*ubz)
lbz = vertcat (*lbz)


V = SX.sym('V',nv, N)               # Decision variables (controls)

D = SX.sym("D", nd, N)   # Disturbance trajectory

Z = vertcat(
    reshape(V, -1, 1),
    reshape(D, -1, 1)
)


def objective_cost():
    J = objective(P[:nw], Z, P[nw:]) 
    return J

def inequality_constraints():

    hz = []   # Box constraints on active inputs

    hz.append(lbz-Z)
    hz.append(Z-ubz)

    return hz

def Pi_opt_formulation():

    J = objective_cost()

    hz  = inequality_constraints()
    Hz = vertcat(*hz)
    G_vcsd = vertcat(*hz)
    lbg =  [-np.inf] * (Hz.shape[0])
    ubg =  [0] * (Hz.shape[0])
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
        "x": Z,
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
    d0 = np.zeros(nd)
    v_st_0 = np.tile(v0, (N, 1))
    d_st_0 = np.tile(d0, (N, 1))
    tr =np.array([0.0])
    args_p = np.concatenate([w0, tr])  # Ensure w0 and Tr are concatenated properly
    
    args_p= vertcat(*args_p)
    args_z0 = np.concatenate([v_st_0.T.reshape(-1), d_st_0.T.reshape(-1)])
   # Solve the optimization problem
    sol = solver(x0=args_z0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    zsol = sol['x']
    #construct vsol 
    vsol = zsol[:N*nv]
    dsol = zsol[N*nv:]
   # construct xsol 
    wsol = Sx@ w0 + Su @vsol + Sd@dsol
    print(wsol.shape)
    w_pred.append(wsol)
    # Reshape usol
    v = np.array(vsol).reshape((N, nv))
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
    d0 = np.zeros(nd)
    t0 = 0.0
    nw = w0.shape[0]
    t = [t0]
    w_ol = []
    w_cl = [w0]
    mpc_i = 0

    v_st_0 = np.tile(v0, (N, 1))
    d_st_0 = np.tile(d0, (N, 1)).T
    tr = np.array([0.0])
    args_p = np.concatenate([w0, tr])  # Ensure w0 and Tr are concatenated properly
    args_p = vertcat(*args_p)
    time_full = []
    d_cons  = np.zeros(nd)

    while  mpc_i < int(sim_time / Ts):
        args_p[:nw] = w0
        args_p[nw:] = np.array([t0])
        
        args_w0 = np.concatenate([v_st_0.T.reshape(-1), d_st_0.T.reshape(-1)])
        start_time = time.time()
        sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full.append(solver_time)
        zsol = sol['x']
        #construct vsol 
        vsol = zsol[:N*nv]
        V_act = np.array(vsol).reshape((N, nv))
        dsol = zsol[N*nv:]
        # construct xsol 
        wsol = Sx@ w0 + Su @vsol + Sd@dsol
        d0 =dsol[:nd]
        w0  = A_d @ w0  + B_d @ V_act[0,:] + Bd_dist@ d0   +  B_const_dist @ d_cons
        w_cl.append(w0)
        w_pred = np.array(wsol).reshape((N+1, nw))
        t0 = t0 + Ts
        d_cons = get_disturbance(t0)
        t.append(t0)
        w_ol.append(w_pred)
        v_st_0 = np.vstack([vsol[nv:],  vsol[(N-1)*nv:]])
        d_st_0 = np.vstack([dsol[1:], dsol[-1:]])
        mpc_i += 1

    w_ol = np.array(w_ol)   
    w_cl = np.array(vertcat(*w_cl)).reshape((mpc_i +1, nw))
    return w_ol, w_cl, time_full,  t

sim_time = 40


w_ol, w_cl, time_full,  t = run_closed_loop_mpc(w0, Ts, sim_time, pisolver)

print(np.mean(time_full))

plot_3d_trajectory(t, w_cl)

plot_xyz_subplots(t, w_cl)
        