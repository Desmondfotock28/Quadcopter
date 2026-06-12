import numpy as np
import time
from casadi import *
from scipy.linalg import null_space
from utils import *
from Quadcopter import  quadcopter_dynamics




#Controller frequency and Prediction horizon
Ts = 0.1    #sampling time in [s]

N_horizon =  10      #prediction horizon

tf= 1

nx= 13       #state dimension 

nu= 4       #input dimension

nv = 3    # dimension of active input vector 

m = 2.0                                     # [kg] total mass

g = 9.8066                                  # [m/s^2] Gravity  

thrust_to_weight = 1.75

max_force_per_motor = (g * m / 4.0) * thrust_to_weight

u_max = vertcat(([max_force_per_motor] * nu))

N_m = N_horizon*nu  # Number of rows

# System states and controls 
x = SX.sym('x', nx);    # states of the system 

u = SX.sym('u', nu);    # control of the system

#system dynamics 
dx = quadcopter_dynamics(x, u)


# Create the CasADi function
system = Function("sys", [x, u], [dx])

# active subspace Projection matrix 

T1 = generate_identity_matrix(N_m, nv)   # active subspace projector matrix 

T2 = null_space(T1.T)                    # inactive subspace projector matrix 


# Decision variables (states)
X = SX.sym('X',nx,(N_horizon+1)) 

# parameter for initial state and parameter t0 
nP = nx + 1  +  (N_horizon *nu - nv)

P = SX.sym("P", nP )

#Decision variables  for  input acive subspace vector 
V = SX.sym("V", nv, 1)

# penalty  variables for inactive subspace w 
mu = SX.sym("mu", 1, 1)

#generate input vector : 
U_a  = T1@V + mu*T2 @ P[nx+1:]

U = reshape(U_a, nu, N_horizon)

# weighing matrices (states)

Q_weight = np.diag([40, 40, 50,
        1.0,  0.043, 0.043, 0.043,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0])



R_weight = 0.1
R_weight = R_weight*np.diag(np.ones(nu))


# Define the stage cost and terminal cost
stage_cost = 0.5 *(bilin(Q_weight, x) +  bilin(R_weight, u))

terminal_cost = bilin(Q_weight, x)

stage_cost_fcn = Function("cost", [x, u], [stage_cost])

terminal_cost_fcn = Function("T_cost", [x], [terminal_cost])

# Input constraints
lb_u = np.array([0.0, 0.0, 0.0, 0.0])

ub_u = np.array([max_force_per_motor, max_force_per_motor , max_force_per_motor, max_force_per_motor])


#optimisation variables
Opt_Vars = vertcat(
            reshape(X, -1, 1),
            reshape(V, -1, 1),
            reshape(mu, -1, 1),
        )


def objective_cost():
    J = 0.0
        # Compute stage costs
    for i in range(N_horizon):
        dx = X[:, i+1]-reference_trajectory(P[nx:nx+1] + i*Ts)
        du = U[:, i]-u_max
        J += stage_cost_fcn(dx, du)
        
      # Compute terminal cost
    J += terminal_cost_fcn((X[:, -1]-reference_trajectory(P[nx:nx+1] + N_horizon*Ts)))
    return J

def equality_constraints():
    g_eq = []  # Equality constraints initialization
    # Initial state constraint
    g_eq.append(X[:, 0] - P[:nx])
  
    # Define dynamics constraints
    for i in range(N_horizon):
        st = X[:, i]
        cons = U[:, i] 
        st_next_euler = ERK4_no_param(system, st, cons, Ts)
        st_next = X[:, i+1]
        g_eq.append(st_next -  st_next_euler)

    return  g_eq

def inequality_constraints():
    # Constraint list
    hu = []   # Box constraints on active inputs
    lbu =[]   # lower bound of u
    ubu =[]   #  upper bound of u

    for _ in range(N_horizon):
        #input constraints 
        lbu.append(lb_u) 
        ubu.append(ub_u)

    lbu = vertcat(*lbu) 
    ubu = vertcat(*ubu)    
    hu.append(lbu - U_a)
    hu.append(U_a - ubu)
    return hu

def Pi_opt_formulation():
    """
    Formulate optimization cost and associated constraints
    - Cost uses stage and terminal cost functions, along with penalties

    """
        # Objective cost
    J = objective_cost()

    # Constraints for casadi and limits
    g_eq = equality_constraints()
    G = vertcat(*g_eq)
    hu = inequality_constraints()
    Hu = vertcat(*hu)
    G_vcsd = vertcat(*g_eq,  *hu)
    lbg = [0] * G.shape[0] + [-np.inf] * (Hu.shape[0])
    ubg = [0] * G.shape[0] + [0] * ( Hu.shape[0])
    lbg_vcsd = vertcat(*lbg)
    ubg_vcsd = vertcat(*ubg)
    
    # NLP Problem for value function and policy approximation
    opts_setting = {
        "ipopt.max_iter": 500,
        "ipopt.print_level": 5,
        "print_time": 1,
        "ipopt.acceptable_tol": 1e-6,
        "ipopt.acceptable_obj_change_tol": 1e-6,
        "ipopt.hessian_approximation": "limited-memory",
        }
   
        
    # NLP Problem for value function and policy approximation
    vnlp_prob = {
            "f": J,
            "x": Opt_Vars,
            "p": vertcat(P),
            "g": G_vcsd,
    }
    # Create the NLP solver instance 
    pisolver = nlpsol("vsolver", "ipopt", vnlp_prob)

    return lbg_vcsd, ubg_vcsd,  pisolver 

lbg_vcsd, ubg_vcsd,  pisolver =  Pi_opt_formulation()



def run_closed_loop_activesubspace_mpc(x0, u_ol, Tr, Ts, sim_time, solver):

    mu = 1

    u_tilda_k  = u_ol

    w_k  =  mtimes(T2.T,u_tilda_k)

    t0 = 0

    t = [t0]

    x_cl = [x0]        # store closed loop states trajectories 

    x_ol = []          # open loop predicted trajectories 

    # Initialization
    mpc_i = 0

    u_cl = []    # Store control inputs in the closed loop

    V_0 = T1.T@u_tilda_k
  
    x_st_0 = np.tile(x0, (N_horizon+1, 1)).T  # initial states decision variables
    v_st_0 = np.tile(V_0 , (1, 1))           # initial active control          
    mu_st_0 = np.tile(mu, (1, 1))            # initial slack 
    
    # Reshape to column vectors if necessary
    x_st_0 = x_st_0.reshape(-1, 1)
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
        Opt_Vars_init = np.concatenate(( x_st_0, v_st_0, mu_st_0), axis=0)

        # Solve the optimization problem
        start_time = time.time()
        sol = solver(x0=Opt_Vars_init, p=P_init, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full_a.append(solver_time)
        x_opt = sol['x']

      # Extract the solution trajectory
        Xsol = x_opt[:nx * (N_horizon+1)]

        x_pred = np.array(Xsol).reshape((N_horizon+1, nx))
        
        
          #Extract varaible v
        Vsol = np.array(x_opt[nx * (N_horizon+1):nx * (N_horizon+1)+nv]).reshape((nv,1))
        musol = np.array(x_opt[nx * (N_horizon+1)+nv:]).reshape((1,1))

         #Reconstruct optimal control input 
        Usol = mtimes(T1,Vsol) + mtimes(musol*T2 , w_k)

        Usol = np.array(Usol)

        u = Usol.reshape((N_horizon , nu))

        # Store all the predictions
        x_ol.append(x_pred)
        
        # Store the first control action
        u_cl.append(u[0, :])

        # Update the state and control for the next iteration
        t.append(t0 )      #store time 
        t0, x0, u0 =shift(Ts, t0, x0,  u, system)
    
        x_cl.append(x0) #store calculated state 
        x_ol.append(x0)
             
        u_tilda_k = u0.reshape(-1, 1)
      
        #update w_k 
        w_k = mtimes(T2.T, u_tilda_k)

        # Prepare the initial condition for the next iteration
        x_st_0 = np.vstack([Xsol[nx:], Xsol[N_horizon*nx:]])

        v_st_0 = mtimes(T1.T, u_tilda_k )

        mu_st_0 = musol

    #increment mpc counter 
        mpc_i += 1
    # Convert lists to numpy arrays for easier handling
    x_cl = np.array(x_cl) 
    u_cl = np.array(u_cl) 

    return x_cl ,  u_cl , t , time_full_a, x_ol


#simulation data 
sim_time = 40

Tr =np.array([0.0])


x0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#u_ol = np.load("open_loop_controls.npy")

u_ol = np.load("/home/udeme/Quadcopter/algorithms/nmpc/active_subspace/Multiple_Shooting/open_loop_controls.npy").reshape(-1) 

x_cl ,  u_cl , t, time_full, x_ol  = run_closed_loop_activesubspace_mpc(x0, u_ol, Tr , Ts, sim_time, pisolver )

t_mean = np.mean(time_full)

print(t_mean)
plot_3d_trajectory(t, x_cl)
plot_xyz_subplots(t, x_cl)

