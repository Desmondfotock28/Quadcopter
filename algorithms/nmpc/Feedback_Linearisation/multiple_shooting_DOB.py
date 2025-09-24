import numpy as np
import time
import matplotlib.pyplot as plt
from casadi import *
from scipy.linalg import block_diag
from scipy.optimize import  least_squares, root


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

def plot_motor_voltages(t, voltages):
    """
    Plot voltage drawn by each motor over time.

    Parameters:
    t : array-like
        Time vector (length N)
    voltages : numpy.ndarray
        Voltage values of shape (N, 4), each column corresponds to a motor [v1, v2, v3, v4]
    
    Returns:
    None
    """
    t = np.array(t)
    voltages = np.array(voltages)

    plt.figure(figsize=(10, 6))
    
    # Plot each motor voltage
    plt.plot(t, voltages[:, 0], label='Motor 1', linestyle='-', marker='o', markersize=4)
    plt.plot(t, voltages[:, 1], label='Motor 2', linestyle='--', marker='s', markersize=4)
    plt.plot(t, voltages[:, 2], label='Motor 3', linestyle='-.', marker='^', markersize=4)
    plt.plot(t, voltages[:, 3], label='Motor 4', linestyle=':', marker='d', markersize=4)
    
    plt.xlabel('Time [s]')
    plt.ylabel('Voltage [V]')
    plt.title('Motor Voltages over Time')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_controls_subplots(u_ol, t):
    """
    Plot the four control inputs (propellers) on separate subplots, excluding the last time step.
    
    Parameters
    ----------
    u_ol : np.ndarray
        Array of shape (N, 4), control inputs for 4 propellers.
    t : np.ndarray
        Time vector of shape (N,).
    """
   
    N, nu = u_ol.shape
    assert nu == 4, f"Expected 4 controls, got {nu}"
    assert len(t) == N, "Length of time vector must match number of control steps"
    
    fig, axs = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
    
    for i in range(4):
        axs[i].plot(t, u_ol[:, i], label=f'Propeller {i+1}', color=f'C{i}')
        axs[i].set_ylabel("Control input")
        axs[i].legend()
        axs[i].grid(True, linestyle="--", alpha=0.6)
    
    axs[-1].set_xlabel("Time [s]")
    fig.suptitle("Control inputs for 4 propellers (separate subplots)", fontsize=14)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
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

def shift(T, t0, x0, u, d_const , f):
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
    st = f(st, con , d_const)
    
    x0 = np.array(st.full()).flatten()

    t0 = t0 + T
    u0 = np.vstack([u[1:], u[-1, :]])

    return t0, x0, u0

def get_disturbance(t):
    dwx = 0.15 * np.sin(np.pi * t / 100) + 0.1 * np.sin(0.2 * t) + 0.03 * np.sin(t)
    dwy = 0.0
    dwz = 0.0
    return np.array([dwx, dwy, dwz])

# solve problem with bounds on theta 
def solve_nonlinear_system(w, x0):
    m = 2.0
    g= 9.81
    def nonlinear_system_eqs(x):

        phi, theta, psi, phi_dot, theta_dot, psi_dot, u1, u1_dot = x

        r1 = (u1/m) * (np.cos(psi)*np.sin(theta)*np.cos(phi)+ (np.sin(psi) * np.sin(phi))) - w[2]

        r2 = (u1_dot/m) * (np.cos(psi)*np.sin(theta)*np.cos(phi) + np.sin(psi)*np.sin(phi)) \
         + (u1/m) * ( 
             (-np.sin(psi)*psi_dot) * np.sin(theta) * np.cos(phi) 
             + np.cos(psi) * np.cos(theta) * theta_dot * np.cos(phi) 
             + np.cos(psi) * np.sin(theta) * (-np.sin(phi)*phi_dot) 
             + (np.cos(psi)*psi_dot) * np.sin(phi) 
             + np.sin(psi) * np.cos(phi) * phi_dot 
         ) - w[3]
        
        r3 = (u1/m)*(np.sin(psi)*np.sin(theta)*np.cos(phi)-np.cos(psi)*np.sin(phi)) + w[6]

        r4 = (u1_dot/m) * (np.sin(psi)*np.sin(theta)*np.cos(phi) - np.cos(psi)*np.sin(phi)) \
         + (u1/m) * (
             (np.cos(psi)*psi_dot)*np.sin(theta)*np.cos(phi) 
             + np.sin(psi)*np.cos(theta)*theta_dot*np.cos(phi)
             - np.sin(psi)*np.sin(theta)*np.sin(phi)*phi_dot
             + np.sin(psi)*psi_dot*np.sin(phi)
             - np.cos(psi)*np.cos(phi)*phi_dot
         )-w[7]

        r5 = (u1/m)*np.cos(phi)*np.cos(theta)-g-w[10]

        r6 = -phi_dot * (u1/m) * np.sin(phi) * np.cos(theta) - theta_dot * (u1/m) * np.cos(phi) * np.sin(theta) + (u1_dot/m) * np.cos(phi) * np.cos(theta) - w[11]
        
        r7 =  psi - w[12]

        r8 = psi_dot - w[13]

        return [r1, r2, r3, r4, r5, r6, r7, r8]
    
    sol = root(nonlinear_system_eqs, x0, method='hybr')  # or 'lm'
    return sol.x


def compute_alpha_beta(x):
    #Solution x = [x0=phi, x1=theta, x2=psi, x3=phi_dot, x4=theta_dot, x5=psi_dot, x6=U1, x7=U1_dot]
    m = 2.0
    I_x = 0.0035
    I_y = 0.0035
    I_z = 0.005

    # compute alpha vector
    alpha_1 = (2*x[7]/m)*(np.cos(x[1])*np.cos(x[0])*x[4] - np.sin(x[1])*np.sin(x[0])*x[3]) \
            + (x[6]/m)*(
           -np.sin(x[1])*np.cos(x[0])*(x[4]**2)
           -2*np.cos(x[1])*np.sin(x[0])*x[4]*x[3]
           -np.sin(x[1])*np.cos(x[0])*(x[3]**2)
       )       #change alpha_1 done


    alpha_2 = (x[6]/m) * np.sin(x[0]) * (x[3]**2) -2*np.cos(x[0])*(x[3]/m)* x[7]    #change alpha_2 done

    alpha_3 = (-(x[3]**2 + x[4]**2) * np.cos(x[0]) * np.cos(x[1]) * (x[6]/m)
               + 2 * x[3] * x[4] * np.sin(x[0]) * np.sin(x[1])* (x[6]/m) 
               - 2 * (x[3] * np.sin(x[0]) * np.cos(x[1]) * x[7]/m 
                      + x[4] * np.cos(x[0]) * np.sin(x[1]) * x[7]/m))              #remain same 

    alpha_4 = 0.0

    alpha = np.array([alpha_1, alpha_2, alpha_3, alpha_4])

    # compute beta matrix
    beta_11 = np.sin(x[1])*np.cos(x[0])/m      # change beta11 done     

    beta_12 = -(x[6]*np.sin(x[1])*np.sin(x[0]))/(m*I_x)   # change beta12 done 

    beta_13 = (x[6]/m)*( (np.cos(x[1])*np.cos(x[0])**2)/I_x - (np.sin(x[1])*np.sin(x[0])**2*np.tan(x[1]))/I_y) # change beta13 done

    beta_14 = (x[6]/m)*(-(np.cos(x[1])*np.cos(x[0])*np.sin(x[1]))/I_y - (np.sin(x[1])*np.sin(x[0])*np.cos(x[0])*np.tan(x[1]))/I_z) # change beta14 done

    beta_21 = -np.sin(x[0])/m   # change beta21 done

    beta_22 = -x[6]*np.cos(x[0])/(m*I_x)         # change beta22 done

    beta_23 = -x[6]*np.cos(x[0])*np.sin(x[0])*np.tan(x[1])/(m*I_y)    # change beta23 done
    
    beta_24 = -x[6]*np.cos(x[0])**2*np.tan(x[1])/(m*I_z)           # change beta24 done


    beta_31 = np.cos(x[0]) * np.cos(x[1]) / m            

    beta_32 = (-(np.sin(x[0]) * np.cos(x[1]) * np.cos(x[2]) 
                 + np.cos(x[0]) * np.sin(x[1]) * np.sin(x[2])) 
               * x[6] / (m * I_x))
    
    beta_33 = ((np.sin(x[0]) * np.cos(x[1]) * np.sin(x[2]) 
                - np.cos(x[0]) * np.sin(x[1]) * np.cos(x[2])) 
               * x[6] / (m * I_y))
    
    beta_34 = 0.0

    beta_41 = 0.0
    beta_42 = 0.0
    beta_43 = (np.sin(x[0])/np.cos(x[1]))*(1/I_y)
    beta_44 = (np.cos(x[0])/np.cos(x[1]))*(1/I_z)

    beta = np.array([
        [beta_11, beta_12, beta_13, beta_14],
        [beta_21, beta_22, beta_23, beta_24],
        [beta_31, beta_32, beta_33, beta_34],
        [beta_41, beta_42, beta_43, beta_44]
    ])

    return alpha, beta



def solve_controls(v, alpha, beta):
    """
    Solve for [U1_ddot, U2, U3, U4].

    Parameters
    ----------
    v : array_like, shape (4,)
        The vector [v1, v2, v3, v4].
    alpha : array_like, shape (4,)
        The vector [alpha1(X), alpha2(X), alpha3(X), alpha4(X)].
    beta : array_like, shape (4,4)
        The beta matrix.

    Returns
    -------
    u : ndarray, shape (4,)
        The solution [U1_ddot, U2, U3, U4].
    """
    v = np.asarray(v).reshape(4, 1)
    alpha = np.asarray(alpha).reshape(4, 1)
    beta = np.asarray(beta).reshape(4, 4)

    u = np.linalg.solve(beta, v - alpha)
    return u.flatten()

def motor_speed(u):
    k = 9.8e-6  #N.s^2
    b = 1.6e-6  #N.m.s^2
    l = 0.225    #m
    # mapping matrix from omega^2 to U
    M = np.array([
        [k,               k,               k,               k],
        [(np.sqrt(2)/2)*l*k, -(np.sqrt(2)/2)*l*k, -(np.sqrt(2)/2)*l*k, (np.sqrt(2)/2)*l*k],
        [(np.sqrt(2)/2)*l*k, (np.sqrt(2)/2)*l*k, -(np.sqrt(2)/2)*l*k, -(np.sqrt(2)/2)*l*k],
        [b,              -b,               b,              -b]
    ])

      #  solve for squared rotor speeds: M * omega_sq = u
    omega_sq = np.linalg.solve(M, u)

      # make sure no negative values due to numerical issues
    omega_sq = np.maximum(omega_sq, 0.0)

    return omega_sq

def motor_voltages(omega_sq):

    cm  = 10000      #v^-2s^-2

    voltage = np.sqrt(omega_sq/cm)

    return voltage

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

Bd_cons = np.zeros((nw, nd))
for j, idx in enumerate(dist_indices):
    Bd_cons[idx, j] = 1.0


Bd_dist = dt*Bd_cons

# Disturbance observer gain matrix 
L_n = Bd_cons.T

L_0 = L_n@Bd_cons

L_f = Bd_cons.T


# Define the CasADi system function using discrete-time matrices
w = SX.sym("w", nw)

v = SX.sym("v", nv) 

d_known = SX.sym("nd", nd) 


w_next = A_d @ w + B_d @ v +  Bd_dist@d_known   #need to confirm logic (reality)

# Create the CasADi function
system = Function("sys", [w, v, d_known ], [w_next])

# Define initial state

w0 = np.zeros(nw) 

# Define cost function parameters

# Declear empty sys matrices
V = SX.sym('V',nv,N)               # Decision variables (controls)

#Parameters:initial state(x0)

P = SX.sym('P', nw + nd + 1, 1) 



W= SX.sym('W',nw,(N+1)) # Decision variables (states)


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



# Define the stage cost and terminal cost 
stage_cost =  0.5*(bilin(Q, w) +  bilin(R, v))

stage_cost_fcn = Function("cost", [w, v], [stage_cost])


terminal_cost = 10*bilin(Q, w)


terminal_cost_fcn = Function("T_cost", [w], [terminal_cost])

# Input constraints

lb_v = np.array([ -537,  -537,  -537, -1675])    #right bound 
ub_v  = np.array([537, 537 , 537, 1675 ])

vmax = 1675
Opt_Vars = vertcat(
    reshape(W, -1, 1),
    reshape(V, -1, 1)
)


def objective_cost():
    J = 0.0
    for i in range(N):
        dw = W[:, i+1]-reference_trajectory(P[nd+nw:] + i*Ts)
        dv = V[:, i]- vmax      
        J += stage_cost_fcn(dw, dv)
    
    J += terminal_cost_fcn((W[:, -1]-reference_trajectory(P[nd + nw:] + N*Ts)))      #+  bilin(Qd ,D[:, -1]) 
    return J

def equality_constraints():
    g = []  # Equality constraints initialization
    g.append(W[:, 0] - P[:nw])  # Initial state constraint
  
    for i in range(N):
        st = W[:, i]
        cons = V[:, i] 
      
        st_next_model =  A_d @ st + B_d @ cons +  Bd_dist @(P[nw:nw+nd])    # (need to clean this)

        st_next = W[:, i+1]
        g.append(st_next -  st_next_model)
    
    #terminal set constraints 
    #g.append(W[:, -1]-reference_trajectory(P[nw:] + N*Ts))
    return g

def inequality_constraints():
    
    hv = []   # Box constraints on virtual inputs 
    for i in range(N):
        hv.append(lb_v - V[:, i]) 
        hv.append(V[:, i] - ub_v) 
    
    return  hv

def Pi_opt_formulation():
    J = objective_cost()
    g = equality_constraints()
    G = vertcat(*g)
    hv = inequality_constraints()
    Hv = vertcat(*hv)
    G_vcsd = vertcat(*g, *hv)
    lbg = [0] * G.shape[0] + [-np.inf] * (Hv.shape[0])
    ubg = [0] * G.shape[0] + [0] * (Hv.shape[0])
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

    d_hat = np.zeros(nd)
    v_st_0 = np.tile(v0, (N, 1))
    w_st_0 = np.tile(w0, (N + 1, 1)).T
    
    args_p = np.concatenate([w0, d_hat, t0 ])  # Ensure x0 and Tr are concatenated properly

    args_p= vertcat(*args_p)

    args_w0 = np.concatenate([w_st_0.T.reshape(-1), v_st_0.T.reshape(-1)])
   # Solve the optimization problem
    sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    vsol = sol['x'][nw * (N+1):]
    # Extract the control inputs from the solution
    v = np.array(sol['x'][nw * (N+1):]).reshape((N, nv))

    #extract predicted state 
    w_pred = np.array(sol['x'][:nw * (N+1)]).reshape((N+1, nw))

    # Convert lists to numpy arrays for easier handling
    w_pred = np.array(w_pred) 

    v = np.array(v) 

    return w_pred, v , vsol

v0 =  np.array([537, 537 , 537, 1675])

# Example usage:

w0 = np.zeros(14)  

Tr =np.array([0.0])

#w_pred, v_ol, vsol = run_open_loop_mpc(w0, Tr, v0 , pisolver)

#t =  np.linspace(0, N*Ts, N+1)

#plot_3d_trajectory(t, w_pred)


def run_closed_loop_mpc(w0, Tr, Ts, sim_time, solver):
   
    d_const = np.array([0.12, -0.08, 0.05])

    v0 = np.array([537, 537 , 537, 1675])

    d_hat = np.zeros(nd)

    # initialize gamma and d_hat
    gamma = np.zeros(nd)        # gamma(k-1) at first iter

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
   

    args_p = np.concatenate([w0, d_hat, Tr])  # Ensure x0 and Tr are concatenated properly
 
    args_p = vertcat(*args_p)
    cost = []
    time_full = []
    V_open_loop = []
    d_predicted = []
    d_actual = []

    while  mpc_i < int(sim_time / Ts):
        args_p[:nw] = w0
        args_p[nw +nd:] = np.array([t0])
        args_p[nw:nw+nd] = d_hat
       
        args_w0 = np.concatenate([w_st_0.T.reshape(-1), v_st_0.T.reshape(-1)])

        start_time = time.time()
        sol = solver(x0=args_w0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full.append(solver_time)
        w_opt = sol['x']
       
        wsol = np.array(w_opt[:nw*(N+1)]).reshape(N+1, nw)
        vsol = np.array(w_opt[nw*(N+1): ]).reshape(N, nv)
        
        V_open_loop.append(vsol)

        cost.append(sol['f'])
  
        w_cl.append(wsol)

        v_apply = vsol[0, :]

        
        d_actual.append(d_const[0])

        v_cl.append(vsol[0, :])

        gamma_prev = gamma

        #implement disturbance observer 
      
        gamma_dot = -L_0@(gamma_prev + L_f @ w0)-L_f@(A@w0 + B@v_apply)

        # Solve for gamma(k)

        gamma = Ts*gamma_dot + gamma_prev


    
        t0, w0, v0 =shift(Ts, t0, w0, vsol, d_const,  system)

        
        # disturbance estimate
        
        d_hat = gamma + L_f@w0

        
        d_predicted.append(d_hat[0])
       
        t.append(t0)
    
        w_ol.append(w0)

        w_st_0 = np.vstack([wsol[1:],  wsol[-1:]])
        v_st_0 = np.vstack([vsol[1:],  vsol[-1:]])
        

        mpc_i += 1

    w_ol = np.array(w_ol)
    v_cl = np.array(v_cl)
    d_actual = np.array(d_actual)
    d_predicted = np.array(d_predicted)
    
    return w_ol, v_cl, t, cost , time_full, V_open_loop, d_actual, d_predicted

# Run the closed-loop MPC for 10s

sim_time = 40

w_ol, v_cl, t, cost_n, time_full, V_open_loop, d_actual, d_predicted = run_closed_loop_mpc(w0, Tr,  Ts, sim_time, pisolver)

print(np.mean(time_full))

plot_3d_trajectory(t, w_ol)

plot_xyz_subplots(t, w_ol)


#test
m = 2.0
# Solve and store solutions
solutions = []
x0 = np.array([0, 0, 0, 0, 0, 0, np.sqrt(m*9.81), 0.0])

for w in w_ol:
    sol = solve_nonlinear_system(w, x0)
    solutions.append(sol)

solutions = np.array(solutions)  # shape (num_w, 8)

# Plot each component in its own subplot
labels = ['phi', 'theta', 'psi', 'phi_dot', 'theta_dot', 'psi_dot', 'u1', 'u1_dot']
fig, axes = plt.subplots(4, 2, figsize=(12, 10))
axes = axes.flatten()

for i in range(8):
    axes[i].plot(solutions[:, i])
    axes[i].set_title(labels[i])
    axes[i].set_ylabel('Value')
    axes[i].grid(True)

plt.tight_layout()
plt.show()





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




u_cl = []
omega_square_cl = []
voltage_cl =[]

x0 = np.array([0, 0, 0, 0, 0, 0, np.sqrt(m*9.81), 0.0])

for w_, v_ in zip(w_ol, v_cl):

    X = solve_nonlinear_system(w_, x0)

    alpha, beta = compute_alpha_beta(X)
  
    u = solve_controls(v_, alpha, beta)
    u[0] = X[6]
    omega_sq = motor_speed(u)

    voltage =motor_voltages(omega_sq)

    u_cl.append(u)

    voltage_cl.append(voltage)

    omega_square_cl.append(omega_sq)

u_cl = np.array(u_cl)

voltage_cl = np.array(voltage_cl)

omega_square_cl = np.array(omega_square_cl)

plot_motor_voltages(t[:-1], voltage_cl)

#plot_controls_subplots(voltage_cl, t[:-1])





