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
    xr = 0.2 * np.cos(t) + 0.5
    yr = 0.2 * np.sin(t) + 0.5
    zr = 0.1 * t + 1.1
    ax.plot(xr, yr, zr, label="Reference Trajectory", color='r', linestyle='--')

    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()

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
    states[:, :2] = np.random.uniform(0, 2, size=(num_samples, 2))  # Randomize x, y
    return states


# States
# x=[x y z roll pitch yaw vx vy vz  wr wp wy];
#syms u1 u2 u3 u4

# Define system parameters
def Quadcopter_parameters() -> tuple:
    """Define system parameters and constraints."""
    # System parameters
    g = 9.81   # m/s^2
    m = 1.0  # kg
    k = 5.3e-6 
    l = 0.225   # m
    b = 3e-7 
    Ixx = 0.0365651 # kg m^2
    Iyy = 0.0365651 # kg m^2
    Izz = 0.0290742 # kg m^2
    cm = 9281.8 # v^-2s^-2
    kd = 0.25  # kg/s

    
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
    - x: State vector [x, y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi]
    - u: Input vector [u1, u2, u3, u4] (control inputs)

    Returns:
    - dx: State derivatives
    """
    # Extract parameters
    PAR = Quadcopter_parameters()
    g, m, k, l, b  = PAR["g"], PAR["m"], PAR["k"], PAR["l"], PAR["b"]
    Ixx, Iyy, Izz, cm, kd =  PAR["Ixx"], PAR["Iyy"], PAR["Izz"], PAR["cm"], PAR["kd"]
    
     # x=[x0=x x1=y x2=z x3=roll x4=pitch x5=yaw x6=vx x7=vy x8=vz  x9=wx x10=wy x11=wz];
   
     # Equations of motion
    dx0 = x[6]

    dx1 = x[7]

    dx2 = x[8]

    dx3 = x[9] + x[10]*(sin(x[3])*tan(x[4])) + x[11]*(cos(x[3])*tan(x[4]))

    dx4 = x[10]*(cos(x[3])) - x[11]*(sin(x[3])*tan(x[4]))

    dx5 = (sin(x[3])/cos(x[4]))*x[10] + (cos(x[3])/cos(x[4]))*x[11]

    dx6 = (-kd/m)*x[6] + (k*cm/m)*(sin(x[5])*sin(x[3])+cos(x[5])*cos(x[3])*sin(x[4]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2)

    dx7 = (-kd/m)*x[7] + (k*cm/m)*(cos(x[3])*sin(x[5])*sin(x[4])- cos(x[5])*sin(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2)

    dx8 = (-kd/m)*x[8] -g + (k*cm/m)*(cos(x[4])*cos(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2)

    dx9 = (l*k*cm/Ixx)*(u[0]**2-u[2]**2)-((Iyy-Izz)/Ixx)*x[10]*x[11]

    dx10 = (l*k*cm/Iyy)*(u[1]**2-u[3]**2)-((Izz-Ixx)/Iyy)*x[9]*x[11]

    dx11 = (b*cm/Izz)*(u[0]**2-u[1]**2+u[2]**2-u[3]**2)-((Ixx-Iyy)/Izz)*x[9]*x[10]
   
    dx = vertcat(dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11)

    return dx

def reference_trajectory(t, omega=1.0, a=0.1):
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
    xr = 0.2 * np.cos(omega * t) + 0.5
    yr = 0.2 * np.sin(omega * t) + 0.5
    zr = a * t + 1.1
    xref = vertcat(xr, yr, zr, np.zeros(9))
    return xref

def g_vec(x0  , U , N , system ,Ts):
    g=[x0]
    U = reshape(U, nu,N)
    x_k = x0
    for i in range(1,N):
        x_k = ERK4_no_param(system, x_k,  U[:,i-1], Ts) 
        g.append(x_k)
    x_N = ERK4_no_param(system, x_k,  U[:,N-1], Ts)
    g.append(x_N)
    return g


def compute_block_matrix( Q_base, R_base, N):
    # Compute the solution to the discrete-time algebraic Riccati equation using control library
    I = np.eye(nx)
    # Construct the block diagonal matrix Q
    Q_blocks = [Q_base] * N  + 40*[I]

    Q_d = np.block([[Q_blocks[i] if i == j else np.zeros_like(Q_base) for j in range(N+1)] for i in range(N+1)])
    
    # Construct the block diagonal matrix R
    R_d = np.block([[R_base if i == j else np.zeros_like(R_base) for j in range(N)] for i in range(N)])

    return  Q_d ,  R_d

#Controller frequency and Prediction horizon
Ts = 0.3    #sampling time in [s]

N =  10      #prediction horizon

tf= 3

nx= 12       #state dimension 

nu= 4       #input dimension

# System states and controls 
x = SX.sym('x', nx);    # states of the system 

u = SX.sym('u', nu);    # control of the system

dx = Quadcopter_ode(x, u)

# Create the CasADi function
system = Function("sys", [x, u], [dx])

# Declear empty sys matrices
U = SX.sym('U', nu, N)               # Decision variables (controls)

#Parameters:initial state(x0,t0)

P = SX.sym('P',nx+1,1)

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
        1,   # angular velocity roll (omega_phi)
        1,   # angular velocity pitch (omega_theta)
        1    # angular velocity yaw (omega_psi)
    ])
    
R = 0.1
R = R*np.diag(np.ones(nu))

Q_d , R_d= compute_block_matrix( Q, R, N)

G = g_vec(P[:nx] , U , N , system ,Ts)

#symbolic variable for g_vec 
g_d = SX.sym("g_d", (N+1)*nx)

# symbolic varaible for control input vector 

u_c = SX.sym("u_x",N*nu)

obj = g_d.T @ Q_d @ g_d + u_c.T @ R_d@ u_c 

# Create the CasADi function
objective = Function("J", [g_d, u_c], [obj])

# Input constraints
lb_u = np.array([0.5, 0.5, 0.5, 0.5])
ub_u = np.array([11, 11 , 11, 11])
umax = [11]*nu

lbu = [lb_u]*N
ubu = [ub_u]*N
ubu = vertcat(*ubu)
lbu = vertcat (*lbu)

Opt_Vars = vertcat(reshape(U, -1,1))

#reference tracking:
X_r = []
U_r = []
for i in range(N):
    X_r.append(reference_trajectory(P[nx:] + i*Ts))
    U_r.append(umax)
X_r.append(reference_trajectory(P[nx:] + N*Ts))

G = vertcat(*G)
X_r  = vertcat(*X_r)
U_r  = vertcat (*U_r )
U = vertcat(reshape(U, -1,1))
def objective_cost():
    J = objective((G-X_r) , (U-U_r))
    return J

def inequality_constraints():
  
    hu = []   # Box constraints on active inputs
   
    hu.append(lbu-U )
    hu.append(U - ubu)
    return  hu

def Pi_opt_formulation():
    J = objective_cost()
    hu = inequality_constraints()
    Hu = vertcat(*hu)
    G_vcsd = vertcat(*hu)
    lbg =  [-np.inf] * ( Hu.shape[0])
    ubg =  [0] * (Hu.shape[0])
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

def run_open_loop_mpc(x0, Tr, u0 , solver ):
      # Initial control inputs and state
    u_st_0 = np.tile(u0, (N, 1))

    args_p = np.concatenate([x0, Tr])

    args_p= vertcat(*args_p)
    args_x0 = u_st_0.T.reshape(-1)
   # Solve the optimization problem
    sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
    usol = sol['x']

    # Extract the control inputs from the solution
    u = np.array(sol['x']).reshape((N , nu))
  
   # construct xsol 
    xsol = g_vec(x0 ,  usol , N, system,Ts)
 
    xsol = vertcat(*xsol)

    xsol = np.array(xsol).reshape(N+1 , nx)
 
    # Convert lists to numpy arrays for easier handling
    return xsol, u ,usol



Tr =np.array([0.0])

u0 =  np.array([5.25, 5.25 , 5.25, 5.25])

x0 = np.zeros(12)

#run open loop
x_pred, u_ol, usol = run_open_loop_mpc(x0, Tr, u0 , pisolver)

t =  np.linspace(0, N*Ts, N+1)

plot_3d_trajectory(t, x_pred)

#run closed -loop
def run_closed_loop_mpc(x0, Tr, Ts, sim_time, solver):
    
    u0 = np.array([5.25, 5.25 , 5.25, 5.25])
    t0 = 0
    nx = x0.shape[0]
    t = [t0]
    x_ol = np.zeros((nx, int(sim_time / Ts) + 1))  # Open loop predicted states
    x_ol = [x0]
    mpc_i = 0
    x_cl = [x0]   # Store predicted states in the closed loop
    u_cl = []    # Store control inputs in the closed loop
    #goal_tolerance = 0.01  # Define a goal tolerance
    u_st_0 = np.tile(u0, (N, 1))
    args_p  = vertcat( 
            reshape(x0, -1, 1), 
            reshape(Tr, -1, 1), 
        ) 
    cost_n = []
    U_open_loop = []
    
    while  mpc_i < int(sim_time / Ts):

        args_p[:nx] = x0 
        args_p[nx:] = np.array([t0])
        args_x0 = vertcat(*u_st_0) # vertcat(reshape(u_st_0, -1, 1)) 
        sol = solver(x0=args_x0, p=args_p, lbg=lbg_vcsd, ubg=ubg_vcsd)
        x_opt = sol['x']
        U_open_loop.append(x_opt)
        cost_n.append(sol['f'])
        u_0 = x_opt[:nu]
        # Extract the control inputs from the solution
        u = np.array(sol['x']).reshape(( N , nu))
        #construct x_pred 
        x_pred = g_vec(x0 , x_opt , N, system, Ts)
        x_pred = vertcat(*x_pred)
        x0 = x_pred[nx:2*nx]
        x_pred = np.array(x_pred).reshape((N+1, nx))
        x_cl.append(x_pred)
        u_cl.append(u_0)
        t.append(t0)
        t0 = t0 + Ts
        x_ol.append(x0)
        u_st_0 = np.vstack([x_opt[nu:], x_opt[(N-1)*nu:]])
        mpc_i += 1
       
    x_ol = vertcat(*x_ol)
    x_ol = np.array(x_ol).reshape(mpc_i + 1 , nx )
    u_cl = np.array(u_cl).reshape(mpc_i , nu)
  
    return x_ol, u_cl, t, cost_n, U_open_loop


Ts = 0.3
sim_time = 10
U_soln =  []

x_ol, u_cl, t, cost_nn, U_open_loop  = run_closed_loop_mpc(x0, Tr, Ts, sim_time, pisolver)

plot_3d_trajectory(t, x_ol)
