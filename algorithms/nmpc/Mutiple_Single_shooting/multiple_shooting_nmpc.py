import numpy as np
import time
from casadi import *
import matplotlib.pyplot as plt
from control import dare
from scipy.linalg import null_space
from mpl_toolkits.mplot3d import Axes3D



def plot_3d_trajectory(t , x_pred):
    """
    Plot a 3D trajectory given predicted positions.

    Parameters:
    x_pred (numpy.ndarray): Predicted trajectory positions as an (N, 3) array, 
                            where each row represents [x, y, z].

    Returns:
    None
    """
    # Extract x, y, z values
    x_pred_vals = x_pred[:, 0]  # x values
    y_pred_vals = x_pred[:, 1]  # y values
    z_pred_vals = x_pred[:, 2]  # z values

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
    y_vals = x_pred[:, 1]
    z_vals = x_pred[:, 2]

 
    # Reference trajectory (example)
    t = np.array(t)
    #xr = np.sin(np.pi * t / 10)
    #yr = np.cos(np.pi * t / 10) - 1.0
    #zr = np.sin(np.pi * t / 10) + t

    #xr = 0.5 + 0.2* np.cos(t)
    #yr = 0.5 + 0.2*np.sin(t) 
    #zr = 1.1 + 0.1*t

    xr = np.ones(t.shape[0])
    yr = np.ones(t.shape[0])
    zr = np.ones(t.shape[0])


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

def plot_angles(t, x):
    """
    Plot roll (x3), pitch (x4), and yaw (x5) over time.
    
    Parameters
    ----------
    t : array-like
        Time vector of shape (N,)
    x : array-like
        State trajectory of shape (N, 12)
    """
    roll  = x[:, 3]
    pitch = x[:, 4]
    yaw   = x[:, 5]

    fig, axs = plt.subplots(3, 1, figsize=(8, 8), sharex=True)

    axs[0].plot(t, roll, label="Roll (x3)", color="r")
    axs[0].set_ylabel("Roll [rad]")
    axs[0].grid(True)
    axs[0].legend()

    axs[1].plot(t, pitch, label="Pitch (x4)", color="g")
    axs[1].set_ylabel("Pitch [rad]")
    axs[1].grid(True)
    axs[1].legend()

    axs[2].plot(t, yaw, label="Yaw (x5)", color="b")
    axs[2].set_ylabel("Yaw [rad]")
    axs[2].set_xlabel("Time [s]")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

# States
# x=[x y z roll pitch yaw vx vy vz  wr wp wy];
#syms u1 u2 u3 u4

# Define system parameters  of F450 Quadcopter
#parameter obtain from N. P. Nguyen and S. K. Hong, "Sliding Mode Thau Observer for Actuator Fault Diagnosis of Quadcopter UAVs"
def Quadcopter_parameters() -> tuple:
    """Define system parameters and constraints."""
     # System parameters 
    g = 9.81        # m/s^2, gravitational acceleration
    m = 2.0         # kg, mass of the quadcopter
    k = 9.8e-6      # N·s^2/rad^2, thrust coefficient (relates rotor speed squared to thrust) T = kw^2
    l = 0.225       # m, distance from the center to each rotor (arm length)
    b = 1.6e-7        # N·m·s^2/rad^2, drag/torque coefficient (relates rotor speed squared to torque) tau_M= bw^2 + I_Mw_dot
    Ixx = 0.0035 # kg·m^2, moment of inertia around x-axis
    Iyy = 0.035 # kg·m^2, moment of inertia around y-axis
    Izz = 0.005 # kg·m^2, moment of inertia around z-axis
    cm = 10000     # v^-2·s^-2, motor constant (relates control input to rotor speed squared)  
    kd = 0.25       # kg/s, linear drag coefficient (damping due to air resistance)  : drag coefficient 

    
    # Create a dictionary to hold all parameters
    PAR = {
        'g': g,      
        'm': m,
        'k': k,
        'l': l,
        'b': b,
        'Ixx':Ixx,
        'Iyy':Iyy,
        'Izz':Izz,
        'cm':cm,
        'kd':kd
    }
    return PAR

def generate_random_states(num_samples=1):
    """
    Generate an array of random start positions where only the state x, y, z
    (indices 0, 1, 2) are sampled from a uniform distribution in [-10, 10].
    
    Parameters:
        num_samples (int): Number of random initial states to generate.
    
    Returns:
        np.ndarray: Array of shape (num_samples, 12) with randomized x, y, z.
    """
    states = np.zeros((num_samples, 12))  # Initialize with zeros
    states[:, :3] = np.random.uniform(0, 3, size=(num_samples, 3))  # Randomize x, y, z
    return states

def shift(T, t0, x0, u, f):
    """
    Shift the state and time forward by one timestep.

    Args:
        T (float): The timestep.
        t0 (float): The current time.
        x0 (np.array): The current state.
        u (np.array): The control inputs.
        f (Function): The system function.

    Returns:
        tuple: The updated time, state, and control inputs.
    """
    st = x0
    con = u[0, :]
    #f_value = f(st, con)
    #st = st + T * f_value
    st =  ERK4_no_param(f, st, con, T)

    x0 = np.array(st.full()).flatten()

    t0 = t0 + T
    u0 = np.vstack([u[1:], u[-1, :]])

    return t0, x0, u0


def ERK4_no_param(f, x, u, h):
    if isinstance(f, SX):
        f = Function("f", [x, u], [f], ["x", "u"], ["xf"])

    k1 = f(x, u)
    k2 = f(x + h / 2 * k1, u)
    k3 = f(x + h / 2 * k2, u)
    k4 = f(x + h * k3, u)
    xf = x + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    return xf

def Quadcopter_ode(x, u):
    """
    Compute the state derivatives for a Quadcopter system.

    Parameters:
    - x: State vector [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]
    - u: Input vector [u1, u2, u3, u4] (control inputs/voltages at four motors)

    Returns:
    - dx: State derivatives
    """
    # Extract parameters
    PAR = Quadcopter_parameters()
    g, m, k, l, b  = PAR["g"], PAR["m"], PAR["k"], PAR["l"], PAR["b"]
    Ixx, Iyy, Izz, cm, kd =  PAR["Ixx"], PAR["Iyy"], PAR["Izz"], PAR["cm"], PAR["kd"]
    
     # x=[x0=x x1=y x2=z x3=roll x4=pitch x5=yaw x6=vx x7=vy x8=vz  x9=wx x10=wy x11=wz];

     #adding disturbance 
     # Disturbances (set to 0 if not used)
    dwx, dwy, dwz = 0.12, -0.08, 0.05
   
     # Equations of motion
    dx0 = x[6]

    dx1 = x[7]

    dx2 = x[8]

    dx3 = x[9] + x[10]*(sin(x[3])*tan(x[4])) + x[11]*(cos(x[3])*tan(x[4]))

    dx4 = x[10]*(cos(x[3])) - x[11]*(sin(x[3]))

    dx5 = (sin(x[3])/cos(x[4]))*x[10] + (cos(x[3])/cos(x[4]))*x[11]

    dx6 =  (k*cm/m)*(sin(x[5])*sin(x[3])+cos(x[5])*cos(x[3])*sin(x[4]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwx   #(-kd/m)*x[6] ignore for now
    dx7 = (k*cm/m)*(cos(x[3])*sin(x[5])*sin(x[4])- cos(x[5])*sin(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwy   # (-kd/m)*x[7] +

    dx8 = -g + (k*cm/m)*(cos(x[4])*cos(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwz  #(-kd/m)*x[8] 

    dx9 = (l*k*cm/Ixx)*(u[0]**2-u[1]**2-u[2]**2 + u[3]**2)-((Iyy-Izz)/Ixx)*x[10]*x[11]

    dx10 = (l*k*cm/Iyy)*(u[0]**2+u[1]**2-u[2]**2 - u[3]**2)-((Izz-Ixx)/Iyy)*x[9]*x[11]

    dx11 = (b*cm/Izz)*(u[0]**2-u[1]**2+u[2]**2-u[3]**2)-((Ixx-Iyy)/Izz)*x[9]*x[10]
   
    dx = vertcat(dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11)

    return dx

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
    #xr =  np.sin(omega * t/10) 
    #yr = np.cos(omega * t/10) + -1.0
    #zr = np.sin(omega * t/10) + t

    #xr = 0.5 + 0.2* np.cos(t)
    #yr = 0.5 + 0.2*np.sin(t) 
    #zr = 1.1 + 0.1*t

    #linear trajecttory 
    xr = 1
    yr = 1
    zr = 1
    phi = np.pi/3
    xref = vertcat(xr, yr, zr, np.zeros(9))
    xref[5]=phi

    return xref



#Controller frequency and Prediction horizon
Ts = 0.1    #sampling time in [s]

N =  10      #prediction horizon

tf= 1

nx= 12       #state dimension 

nu= 4       #input dimension

# System states and controls 
x = SX.sym('x', nx);    # states of the system 

u = SX.sym('u', nu);    # control of the system

dx = Quadcopter_ode(x, u)
# Create the CasADi function
system = Function("sys", [x, u], [dx])

# Declear empty sys matrices
U = SX.sym('U',nu,N)               # Decision variables (controls)

#Parameters:initial state(x0)

P = SX.sym('P',nx + 1, 1) 

X = SX.sym('X',nx,(N+1)) # Decision variables (states)

# weighing matrices (states)
Q = np.diag([
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
R = R*np.diag(np.ones(nu))

# Define the stage cost and terminal cost 
stage_cost = 0.5 *( bilin(Q, x) +  bilin(R, u))
terminal_cost = 40*mtimes(x.T, x)

stage_cost_fcn = Function("cost", [x, u], [stage_cost])

terminal_cost_fcn = Function("T_cost", [x], [terminal_cost])

# Input constraints
lb_u = np.array([0.5, 0.5, 0.5, 0.5])
ub_u = np.array([11, 11 , 11, 11])
umax = 11
Opt_Vars = vertcat(
    reshape(X, -1, 1),
    reshape(U, -1, 1)
)

def objective_cost():
    J = 0.0
    for i in range(N):
        dx = X[:, i+1]-reference_trajectory(P[nx:] + i*Ts)
        du = U[:, i]-umax
        J += stage_cost_fcn(dx, du)
    J += terminal_cost_fcn((X[:, -1]-reference_trajectory(P[nx:] + N*Ts)))
    return J

def equality_constraints():
    g = []  # Equality constraints initialization
    g.append(X[:, 0] - P[:nx])  # Initial state constraint
  
    for i in range(N):
        st = X[:, i]
        cons = U[:, i] 
        #st_next_euler = st + (Ts*system(st,cons))
        st_next_euler = ERK4_no_param(system, st, cons, Ts)
        st_next = X[:, i+1]
        g.append(st_next -  st_next_euler)
    #terminal set constraints 
    #g.append(X[:, -1]-reference_trajectory(P[nx:] + N*Ts))
    return g

def inequality_constraints():
    
    hu = []   # Box constraints on active inputs 
    for i in range(N):
        hu.append(lb_u - U[:, i]) 
        hu.append(U[:, i] - ub_u)
    return  hu 

def Pi_opt_formulation():
    J = objective_cost()
    g = equality_constraints()
    G = vertcat(*g)
    hu = inequality_constraints()
    Hu = vertcat(*hu)
    G_vcsd = vertcat(*g, *hu)
    lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0])
    ubg = [0] * G.shape[0] + [0] * (Hu.shape[0])
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

def run_open_loop_mpc(x0, t0 , u0 , solver ):
      # Initial control inputs and state
    u_st_0 = np.tile(u0, (N, 1))
    x_st_0 = np.tile(x0, (N + 1, 1)).T
    args_p = np.concatenate([x0, t0])  # Ensure x0 and Tr are concatenated properly
    args_p= vertcat(*args_p)
    args_x0 = np.concatenate([x_st_0.T.reshape(-1), u_st_0.T.reshape(-1)])
   # Solve the optimization problem
    sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    usol = sol['x'][nx * (N+1):]
    # Extract the control inputs from the solution
    u = np.array(sol['x'][nx * (N+1):]).reshape((N, nu))

    #extract predicted state 
    x_pred = np.array(sol['x'][:nx * (N+1)]).reshape((N+1, nx))

    # Convert lists to numpy arrays for easier handling
    x_pred = np.array(x_pred) 
    u = np.array(u) 
    return x_pred, u , usol

u0 =  np.array([5.25, 5.25 , 5.25, 5.25])

# Example usage:
#random_states = generate_random_states(num_samples=5)
#x0 = random_states[0]

x0 = np.zeros(12)  
#x0[:1]= 1
#x0[:3] += np.random.uniform(low=-2.0, high= 2.0, size=3)
Tr =np.array([0.0])
x_pred, u_ol, usol = run_open_loop_mpc(x0, Tr, u0 , pisolver)

t =  np.linspace(0, N*Ts, N+1)

plot_xyz_subplots(t, x_pred)

def run_closed_loop_mpc(x0, Tr, Ts, sim_time, solver):
   
    u0 =  np.array([5.25, 5.25 , 5.25, 5.25])
    t0 = 0
    nx = x0.shape[0]
    t = [t0]
    x_ol = np.zeros((nx, int(sim_time / Ts) + 1))  # Open loop predicted states
    x_ol = [x0]
    mpc_i = 0
    x_cl = []    # Store predicted states in the closed loop
    u_cl = []    # Store control inputs in the closed loop

    u_st_0 = np.tile(u0, (N, 1))
    x_st_0 = np.tile(x0, (N + 1, 1)).T
    args_p = np.concatenate([x0, Tr])  # Ensure x0 and Tr are concatenated properly
    #args_p = np.array([x0])
    args_p = vertcat(*args_p)
    cost = []
    time_full = []
    U_open_loop = []
    while  mpc_i < int(sim_time / Ts):
        args_p[:nx] = x0
        args_p[nx:] = np.array([t0])
        args_x0 = np.concatenate([x_st_0.T.reshape(-1), u_st_0.T.reshape(-1)])
        start_time = time.time()
        sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full.append(solver_time)
        x_opt = sol['x']
        Xsol = x_opt[:nx * (N+1)]
        Usol = x_opt[nx * (N+1):]
        U_open_loop.append(Usol)
        cost.append(sol['f'])
        u = np.array(sol['x'][nx * (N+1):]).reshape((N, nu))
        x_pred = np.array(x_opt[:nx * (N+1)]).reshape((N+1, nx))
        x_cl.append(x_pred)
        u_cl.append(u[0, :])
        t.append(t0)
        t0, x0, u0 =shift(Ts, t0, x0, u, system)
    
        x_ol.append(x0)
        x_st_0 = np.vstack([Xsol[nx:],  Xsol[N*nx:]])
        u_st_0 = np.vstack([Usol[nu:],  Usol[(N-1)*nu:]])

        mpc_i += 1

    x_ol = np.array(x_ol)
    u_cl = np.array(u_cl)
    return x_ol, u_cl, t, cost , time_full, U_open_loop

# Run the closed-loop MPC for 10s
Ts = 0.1
sim_time = 20
x_ol, u_cl, t, cost_n, time_full, U_open_loop = run_closed_loop_mpc(x0, Tr, Ts, sim_time, pisolver)

print(np.mean(time_full))

plot_3d_trajectory(t, x_ol)

plot_xyz_subplots(t, x_ol)

plot_angles(t, x_ol)

min_len = min(len(t), u_cl.shape[0])
t = t[:min_len]

voltages = u_cl[:min_len, :]

plot_motor_voltages(t, voltages)