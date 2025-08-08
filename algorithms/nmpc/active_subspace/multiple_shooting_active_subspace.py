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

def generate_identity_matrix(N_m, N_v):
    """
    Generate an N_m by N_v identity-like matrix.

    Parameters:
    N_m (int): Number of rows.
    N_v (int): Number of columns.

    Returns:
    numpy.ndarray: An N_m x N_v identity-like matrix.
    """
    # Create a square identity matrix of size min(N_m, N_v)
    size = min(N_m, N_v)
    identity = np.eye(size)
    
    # Extend to desired shape (N_m x N_v)
    extended_identity = np.zeros((N_m, N_v))
    extended_identity[:size, :size] = identity
    
    return extended_identity


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
        1,   # angular velocity roll (omega_phi)
        1,   # angular velocity pitch (omega_theta)
        1    # angular velocity yaw (omega_psi)
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

x0 = np.zeros(12)  
Tr =np.array([0.0])
x_pred, u_ol, usol = run_open_loop_mpc(x0, Tr, u0 , pisolver)

t =  np.linspace(0, N*Ts, N+1)

plot_3d_trajectory(t, x_pred)

Ts = 0.3
sim_time = 10

#NMPC in active subspace using Multiple shooting 

N_m = 40  # Number of rows
nv = 15  # Number of columns
T1 = generate_identity_matrix(N_m, nv)
T2 = null_space(T1.T)

# Decision variables (states)
X_a = SX.sym('X_a',nx,(N+1)) 

# parameter for initial state and parameter t0 
nP = nx + 1  +  (N *nu - nv)
P_a = SX.sym("P_a", nP )

#Decision variables  for  input acive subspace vector 
V = SX.sym("V", nv, 1)

# penalty  variables for inactive subspace w 
mu = SX.sym("mu", 1, 1)

#generate input vector : 
U_a = T1@V + mu*T2 @ P_a[nx+1:]
U_aa = reshape(U_a, nu, N)
#optimisation variables
Opt_Vars_p = vertcat(
            reshape(X_a, -1, 1),
            reshape(V, -1, 1),
            reshape(mu, -1, 1),
        )

def objective_cost_p():
    J = 0.0
        # Compute stage costs
    for i in range(N):
        dx = X_a[:, i+1]-reference_trajectory(P_a[nx:nx+1] + i*Ts)
        du = U_aa[:, i]-umax 
        J += stage_cost_fcn(dx, du)
        
      # Compute terminal cost
    J += terminal_cost_fcn((X_a[:, -1]-reference_trajectory(P_a[nx:nx+1] + N*Ts)))
    return J
def equality_constraints_p():
    g = []  # Equality constraints initialization
    # Initial state constraint
    g.append(X_a[:, 0] - P_a[:nx])
  
    # Define dynamics constraints
    for i in range(N):
        st = X_a[:, i]
        cons = U_aa[:, i] 
        st_next_euler = ERK4_no_param(system, st, cons, Ts)
        #st_next_euler = st + (Ts*system(st,cons))
        st_next = X_a[:, i+1]
        g.append(st_next -  st_next_euler)
    return  g

def inequality_constraints_p():
    # Constraint list
    hu = []   # Box constraints on active inputs
    lbu =[]   # lower bound of u
    ubu =[]   #  upper bound of u
    for i in range(N):
        #input constraints 
        lbu.append(lb_u) 
        ubu.append(ub_u)
    lbu = vertcat(*lbu) 
    ubu = vertcat(*ubu)    
    hu.append(lbu - U_a)
    hu.append(U_a - ubu)
    return hu

def Pi_opt_formulation_p():
    """
    Formulate optimization cost and associated constraints
    - Cost uses stage and terminal cost functions, along with penalties

    """
        # Objective cost
    J = objective_cost_p()

    # Constraints for casadi and limits
    g = equality_constraints_p()
    G = vertcat(*g)
    hu = inequality_constraints_p()
    Hu = vertcat(*hu)
    G_vcsd = vertcat(*g,  *hu)
    lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0])
    ubg = [0] * G.shape[0] + [0] * ( Hu.shape[0])
    lbg_vcsd_p = vertcat(*lbg)
    ubg_vcsd_p = vertcat(*ubg)
    
    # NLP Problem for value function and policy approximation
    opts_setting = {
        "ipopt.max_iter": 500,
        "ipopt.print_level": 4,
        "print_time": 1,
        "ipopt.acceptable_tol": 1e-6,
        "ipopt.acceptable_obj_change_tol": 1e-6,
        }
   
        
    # NLP Problem for value function and policy approximation
    vnlp_prob = {
            "f": J,
            "x": Opt_Vars_p,
            "p": vertcat(P_a),
            "g": G_vcsd,
    }
    # Create the NLP solver instance 
    pisolver_p = nlpsol("vsolver", "ipopt", vnlp_prob)
    return lbg_vcsd_p, ubg_vcsd_p,  pisolver_p 

lbg_vcsd_p, ubg_vcsd_p,  pisolver_p =  Pi_opt_formulation_p()

def run_closed_loop_activesubspace_mpc(x0, u0, Tr, Ts, sim_time, solver):
    mu = 1
    u_tilda_k  = u_ol
    w_k  =  mtimes(T2.T, usol )
    t0 = 0
    nx = x0.shape[0]
    t = [t0]
    x_ol = np.zeros((nx, int(sim_time / Ts) + 1))  # Open loop predicted states
    x_ol = [x0]
    # Initialization
    mpc_i = 0
    x_cl = []    # Store predicted states in the closed loop
    u_cl = []    # Store control inputs in the closed loop
    u_st_0 = np.tile(u0, (N, 1))
    u_st_0  = u_st_0.reshape(-1,1)
    V_0 = T1.T@(u_st_0 -T2@w_k)
   # Initial control inputs
    x_st_0_p = np.tile(x0, (N+1, 1)).T  # initial states decision variables
    v_st_0 = np.tile(V_0 , (1, 1))
    mu_st_0 = np.tile(mu, (1, 1))
    
    # Reshape to column vectors if necessary
    x_st_0_p = x_st_0_p.reshape(-1, 1)
    v_st_0 = v_st_0.reshape(-1, 1)
    mu_st_0 = mu_st_0.reshape(-1, 1)
    # Concatenate all three into one array
    P_init  = vertcat( 
            reshape(x0, -1, 1), 
            reshape(Tr, -1, 1), 
            reshape(w_k, -1, 1)
        ) 
    time_full_a = []
    while  mpc_i < int(sim_time / Ts):
         
        P_init[:nx] = x0
        P_init[nx + 1:] = w_k
        P_init[nx:1+nx] = np.array([t0])
        Opt_Vars_init = np.concatenate(( x_st_0_p, v_st_0, mu_st_0), axis=0)

        # Solve the optimization problem
        start_time = time.time()
        sol = solver(x0=Opt_Vars_init, p=P_init, lbg=lbg_vcsd_p, ubg=ubg_vcsd_p)
        solver_time = time.time()-start_time
        time_full_a.append(solver_time)
        x_opt_p = sol['x']
         # Extract the solution trajectory
        Xsol_p = np.array(x_opt_p[:nx * (N+1)]).reshape((N+1, nx))
        Xsol = x_opt_p[:nx * (N+1)]
          #Extract varaible v
        Vsol = np.array(x_opt_p[nx * (N+1):nx * (N+1)+nv]).reshape((nv,1))
        musol = np.array(x_opt_p[nx * (N+1)+nv:]).reshape((1,1))
         #Reconstruct optimal control input 
        Usol = T1@Vsol + musol*T2 @w_k
        u = np.array(Usol).reshape((N , nu))
        # Store all the predictions
        x_pred = Xsol_p
        x_cl.append(x_pred)
        # Store the first control action
        u_cl.append(u[0, :])
        # Update the state and control for the next iteration
        t.append(t0 )      #store time 
        t0, x0, u0 =shift(Ts, t0, x0,  u, system)
    
        x_ol.append(x0) #store calculated state 
             
        u_tilda_k = np.vstack([Usol[nu:], Usol[(N-1)*nu:]]) 
      
        #update w_k 
        w_k = T2.T@(u_tilda_k)
        # Prepare the initial condition for the next iteration
        x_st_0_p = np.vstack([Xsol[nx:], Xsol[N*nx:]])
        v_st_0 = T1.T@(Usol - T2@w_k)
        mu_st_0 = musol
    #increment mpc counter 
        mpc_i += 1
    # Convert lists to numpy arrays for easier handling
    x_ol = np.array(x_ol) 
    u_cl = np.array(u_cl) 
    
    

    return x_ol ,  u_cl ,t , time_full_a

x_ol_p ,  u_cl_p , t_p, time_full_p  = run_closed_loop_activesubspace_mpc(x0, u0, Tr , Ts, sim_time, pisolver_p )

plot_3d_trajectory(t_p, x_ol_p)

