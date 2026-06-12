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

# System states and controls 
x = SX.sym('x', nx);    # states of the system 

u = SX.sym('u', nu);    # control of the system

#system dynamics 
dx = quadcopter_dynamics(x, u)


# Create the CasADi function
system = Function("sys", [x, u], [dx])

# active subspace Projection matrix 

N_m = N_horizon*nu  # Number of rows

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
U_a  = mtimes(T1,V) + mtimes(mu*T2 , P[nx+1:])

#U = reshape(U_a, nu, N_horizon)


# weighing matrices (states)

Q_weight = np.diag([40, 40, 50,
        1.0,  0.043, 0.043, 0.043,
        2.0, 2.0, 2.0,
        1.0, 1.0, 1.0])



R_weight = 0.1
R_weight = R_weight*np.diag([1, 1 , 1 , 1])

Q_d , R_d= compute_block_matrix( Q_weight, R_weight, N_horizon)

G = g_vec(P[:nx] , U_a  , N_horizon , system ,Ts)

#symbolic variable for g_vec 
g_d = SX.sym("g_d", (N_horizon+1)*nx)

# symbolic varaible for control input vector 

u_c = SX.sym("u_x",N_horizon*nu)

obj = g_d.T @ Q_d @ g_d + u_c.T @ R_d@ u_c 

# Create the CasADi function
objective = Function("J", [g_d, u_c], [obj])



# Input constraints
lb_u = np.array([0.0, 0.0, 0.0, 0.0])

ub_u = np.array([max_force_per_motor, max_force_per_motor , max_force_per_motor, max_force_per_motor])

umax = [max_force_per_motor]*nu

lbu = [lb_u]*N_horizon
ubu = [ub_u]*N_horizon
ubu = vertcat(*ubu)
lbu = vertcat (*lbu)


Opt_Vars = vertcat(
            reshape(V, -1, 1),
            reshape(mu, -1, 1),
        )

U_a = vertcat(reshape(U_a, -1,1))

G = vertcat(*G)

#reference tracking:
X_ref = []
U_ref = []


for i in range(N_horizon):
    X_ref.append(reference_trajectory(P[nx:nx+1] + i*Ts))
    U_ref.append(umax)
X_ref.append(reference_trajectory(P[nx:nx+1] + N_horizon*Ts))

X_ref  = vertcat(*X_ref)
U_ref  = vertcat (*U_ref )


def objective_cost():

    J = objective((G-X_ref) , (U_a-U_ref)) 

    return J

def inequality_constraints():

    # Constraint list
    hu = []   # Box constraints on active inputs

    hu.append(U_a - ubu)

    hu.append(lbu- U_a)

    return hu



def Pi_opt_formulation():

    J = objective_cost()

    hu = inequality_constraints()

    Hu = vertcat(*hu)

    G_vcsd = vertcat(*hu)

    lbg = [-np.inf] * (Hu.shape[0])

    ubg = [0] * (Hu.shape[0])

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

    return lbg_vcsd, ubg_vcsd,  pisolver

lbg_vcsd, ubg_vcsd,  pisolver = Pi_opt_formulation()




def run_closed_loop_activesubspace_mpc(x0, u_tilda_k, Tr, Ts, sim_time, solver ):

    mu = 1

    w_k  =  mtimes(T2.T, u_tilda_k )

    t0 = 0

    t = [t0]

    x_ol = []   # Open loop predicted trajectories 

    x_cl = [x0] # Store predicted states in the closed loop

    # Initialization
    mpc_i = 0
    
    u_cl = []    # Store control inputs in the closed loop

    cost_fn = []

    

   # Initial control inputs
    V_0 = T1.T@u_tilda_k

    v_st_0 = np.tile(V_0 , (1, 1))

    mu_st_0 = np.tile(mu, (1, 1))
    
    # Reshape to column vectors if necessary
    v_st_0 = v_st_0.reshape(-1, 1)
    
    mu_st_0 = mu_st_0.reshape(-1, 1)

    # Concatenate all three into one array
    P_init  = vertcat( 
            reshape(x0, -1, 1), 
            reshape(Tr, -1, 1), 
            reshape(w_k, -1, 1)
        ) 
    #open_loop 
   
    time_full = []

    while  mpc_i < int(sim_time / Ts):
         
        P_init[:nx] = x0
        P_init[1+nx:] = w_k
        P_init[nx:1+nx] = np.array([t0])
        Opt_Vars_init = np.concatenate(( v_st_0, mu_st_0), axis=0)

        # Solve the optimization problem
        start_time = time.time()
        sol = solver(x0=Opt_Vars_init, p=P_init, lbg=lbg_vcsd, ubg=ubg_vcsd)
        solver_time = time.time()-start_time
        time_full.append(solver_time)
        x_opt_p = sol['x']

         # Extract the solution trajectory
        Vsol = np.array(x_opt_p[:nv]).reshape((nv,1))
        musol = np.array(x_opt_p[nv:]).reshape((1,1))

         #Reconstruct optimal control input 
        Usol = mtimes(T1, Vsol) + mtimes(musol*T2, w_k)

        u = np.array(Usol).reshape((nu, N_horizon))

        # compute and store cost function
        J_n = sol['f']
        cost_fn.append(J_n)

        #  Store the first control action
        u_cl.append(u[0, :])

        x_pred = g_vec(x0 , Usol, N_horizon, system,Ts)
        x_pred = vertcat(*x_pred)

        x0 = x_pred[nx:2*nx]

        x_pred = np.array(x_pred).reshape((N_horizon+1, nx))
        x_ol.append(x_pred)

        #store time 
       
        t.append(t0)

        #update time 
        t0 = t0 + Ts

        # store closed loop state trajectoreis 
        x_cl.append(x0)

        #update u_tilda_k 
        u_tilda_k = np.vstack([Usol[nu:], Usol[(N_horizon-1)*nu:]])
      
        #update w_k 
        w_k = mtimes(T2.T, u_tilda_k)

        # Prepare the initial condition for the next iteration
        v_st_0 = mtimes(T1.T, u_tilda_k)

        mu_st_0 = musol

    #increment mpc counter 
        mpc_i += 1

    # Convert lists to numpy arrays for easier handling

    x_cl = np.array(vertcat(*x_cl)).reshape(mpc_i+1,nx)
    u_cl = np.array(u_cl) 

    #u_cl = np.array(vertcat(*u_cl)).reshape(mpc_i, nu)
    
    return x_cl ,  u_cl ,t, cost_fn , time_full


#simulation data 
sim_time = 40

Tr =np.array([0.0])


x0 = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

u_ol = np.load("/home/udeme/Quadcopter/algorithms/nmpc/active_subspace/Single_Shooting/open_loop_controls.npy").reshape(-1) 

x_cl ,  u_cl ,t , cost_n, time_full = run_closed_loop_activesubspace_mpc(x0, u_ol, Tr, Ts, sim_time, pisolver)

plot_3d_trajectory(t, x_cl)

plot_xyz_subplots(t, x_cl)

t_mean = np.mean(time_full)

print(t_mean)

