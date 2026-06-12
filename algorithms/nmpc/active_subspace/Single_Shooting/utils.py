import numpy as np
from casadi import Function, SX, sin, cos, vertcat
import matplotlib.pyplot as plt



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


def compute_block_matrix( Q_base, R_base, N):
    # Compute the solution to the discrete-time algebraic Riccati equation using control library
    #I = np.eye(Q_base.shape[0])
    # Construct the block diagonal matrix Q
    Q_blocks = [Q_base] * (N  + 1)

    Q_d = np.block([[Q_blocks[i] if i == j else np.zeros_like(Q_base) for j in range(N+1)] for i in range(N+1)])
    
    # Construct the block diagonal matrix R
    R_d = np.block([[R_base if i == j else np.zeros_like(R_base) for j in range(N)] for i in range(N)])

    return  Q_d ,  R_d

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


def reference_trajectory(t, omega=np.pi):
    xr = sin(omega * t / 10)
    yr = cos(omega * t / 10) - 1.0
    zr = sin(omega * t / 10) + t + 1.0
    
    xref = vertcat(xr, yr, zr, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

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
    zr = np.sin(np.pi * t/10) + t + 1.0
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
    xr = np.sin(np.pi * t / 10)
    yr = np.cos(np.pi * t / 10) - 1.0
    zr = np.sin(np.pi * t / 10) + t + 1.0

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