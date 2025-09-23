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

    # Sd: stack sum_{j=0}^{k-1} A^j Bd  (Sd[0] = 0)
    Sd = np.zeros((nw*(N+1), nd))
    cum = np.zeros((nw, nd))
    for k in range(1, N+1):
        cum = cum + Apows[k-1] @ Bd
        Sd[k*nw:(k+1)*nw, :] = cum

    return Sx, Su, Sd

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


def shift(T, t0, x0, u, d_cons, f):
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
    st = f(st, con ,  d_cons)
    
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


# Discrete-time system matrices using matrix exponential


# Discretization parameters
dt = 0.1

A_d = np.eye(A.shape[0]) + dt * A
B_d = dt * B


Ts = 0.1   #sampling time in [s]

N =  10    #prediction horizon

tf = 1

# State and input dimensions 
nw = A.shape[1]
nv = B.shape[1]

# Disturbance injection matrix: injects bias into rows 2, 6, 9
dist_indices = [1, 5, 9]
nd = len(dist_indices)

Bd_cons = np.zeros((nw, nd))
for j, idx in enumerate(dist_indices):
    Bd_cons[idx, j] = 1.0

Bd_dist = dt*Bd_cons


# Disturbance observer gain matrix 
L_n = Bd_cons.T

L_0 = L_n@Bd_cons

L_f = Bd_cons.T

d_const = np.array([0.12, -0.08, 0.05])   # (nd,)
 
nd = Bd_dist.shape[1]

Q = np.diag([
        40,  # w1 (x-position)
        2,  # w2 
        2,  # w3 
        2,   # w4 
        40,   # w5 (y-position)
        2,   # w6 
        2,   # w7 
        2,   # w8 
        50,   # w9 (altitude)
        2,   # w10 
        2,   # w11 
        2,    # w12 
        5,   # w13 (yaw )
        1    #w14
    ])
R = 0.01
R = R*np.diag(np.ones(nv))


#computing block diagonal matrices 
Sx, Su, Sd = build_prediction_mats(A_d, B_d, Bd_dist, N)

#Hard coded matrices
Qblk, Rblk = build_blk_cost(Q, 10*Q, R, N)

# H and h
H = Su.T @ Qblk @ Su + Rblk


v = SX.sym("v", N*nv)

w = SX.sym("w", nw)

vm = SX.sym("vm", nv)

Tr = SX.sym("Tr", 1)

d_cons = SX.sym('d_cons', nd) 

c = SX.sym('c', nd) 

P = SX.sym('P',nw + nd + 1, 1) 

t0 = 0.0


w_next = A_d @ w + B_d @ vm + Bd_dist @ d_cons  #need to confirm logic 

# Create the CasADi function
system = Function("sys", [w, vm ,d_cons], [w_next])

Wref = stack_reference(reference_trajectory, Tr, Ts, N)

h = Su.T @ Qblk @ (Sx @ w + Sd @ c - Wref)



# Define the stage cost and terminal cost 

obj = 0.5 * mtimes([v.T, H, v]) + mtimes([h.T, v])  # scalar 

# CasADi function: parametric in w0
objective = Function("J", [w, v, c , Tr], [obj])

# Input constraints
lb_v = np.array([ -537,  -537,  -537, -1675])    #right bound 
ub_v  = np.array([537, 537 , 537, 1675 ])


lbz = [lb_v]*N 
ubz =   [ub_v]*N 
ubz = vertcat(*ubz)
lbz = vertcat (*lbz)


V = SX.sym('V',nv, N)               # Decision variables (controls)


Z = vertcat(
    reshape(V, -1, 1)   
)


def objective_cost():
    J = objective(P[:nw], Z, P[nw:nw+nd], P[nw + nd:]) 
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
  
    d_hat = np.zeros(nd)

    v_st_0 = np.tile(v0, (N, 1))

    tr =np.array([0.0])

    args_p = np.concatenate([w0, d_hat , tr])  # Ensure w0 and Tr are concatenated properly
    
    args_p= vertcat(*args_p)

    args_z0 = v_st_0.T.reshape(-1)
   # Solve the optimization problem

    sol = solver(x0=args_z0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)

    vsol = sol['x']

   # construct xsol 
    wsol = Sx@ w0 + Su @vsol 

    w_pred.append(wsol)
    # Reshape usol

    v = np.array(vsol).reshape((N, nv))

    #Reshape xsol 
    w_pred = vertcat(*w_pred)
    w_pred = np.array(w_pred).reshape((N+1, nw))
 

    return w_pred, v , vsol 

v0 =  np.array([537, 537 , 537, 1675 ])

w0 = np.zeros(14)  

w_pred, v_ol, vsol  = run_open_loop_mpc(w0, v0 , pisolver)


t =  np.linspace(0, N*Ts, N+1)

plot_3d_trajectory(t, w_pred)



def run_closed_loop_mpc(w0, Ts, sim_time, solver):
   
    v0 =  np.array([537, 537 , 537, 1675 ])
   
    gamma = np.zeros(nd)        # gamma(k-1) at first iter

    d_hat = np.zeros(nd)

    t0 = 0.0
    nw = w0.shape[0]
    t = [t0]
    w_ol = []
    w_cl = [w0]
    mpc_i = 0

    v_st_0 = np.tile(v0, (N, 1))
  
    tr = np.array([0.0])
    args_p = np.concatenate([w0, d_hat, tr])  # Ensure w0 and Tr are concatenated properly
    args_p = vertcat(*args_p)
    time_full = []

    d_predicted = []
    d_actual = []


    while  mpc_i < int(sim_time / Ts):
        args_p[:nw] = w0
        args_p[nw + nd:] = np.array([t0])
        args_p[nw:nw + nd] = d_hat

        args_w0 = v_st_0.T.reshape(-1)
                                  

        start_time = time.time()
        sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time

        time_full.append(solver_time)

        vsol = sol['x']
        #construct vsol 
       
        V_act = np.array(vsol).reshape((N, nv))
        
        # construct xsol 
        wsol = Sx@ w0 + Su @vsol  +  Sd@d_hat

        
        d0 = np.array([0.0,0.0,0.0])     #get_disturbance(t0)
        d_actual.append(d0[0])

        #implement disturbance observer 
        gamma_prev = gamma
      
        gamma_dot = -L_0@(gamma_prev + L_f @ w0)-L_f@(A@w0 + B@ V_act[0,:])

        # Solve for gamma(k)

        gamma = Ts*gamma_dot + gamma_prev

       
        
        t0, w0, v0 =shift(Ts, t0, w0,  V_act , d0,  system)


        # disturbance estimate
        
        d_hat = gamma + L_f@w0

        d_predicted.append(d_hat[0])

        w_cl.append(w0)

        w_pred = np.array(wsol).reshape((N+1, nw))
      
        t.append(t0)

        w_ol.append(w_pred)

        v_st_0 = np.vstack([vsol[nv:],  vsol[(N-1)*nv:]])
       
        mpc_i += 1

    w_ol = np.array(w_ol)   
    w_cl = np.array(vertcat(*w_cl)).reshape((mpc_i +1, nw))
    d_actual = np.array(d_actual)
    d_predicted = np.array(d_predicted)

    return w_ol, w_cl, time_full,  t , d_actual, d_predicted

sim_time = 40


w_ol, w_cl, time_full,  t , d_actual, d_predicted = run_closed_loop_mpc(w0, Ts, sim_time, pisolver)

print(np.mean(time_full))

plot_3d_trajectory(t, w_cl)

plot_xyz_subplots(t, w_cl)


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


