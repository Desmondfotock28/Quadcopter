
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as scipylinalg





def get_continous_time_matrices():
      
    A1 = np.array([[0., 1., 0., 0.],
                   [0., 0., 1., 0.],
                   [0., 0., 0., 1.],
                   [0., 0., 0., 0.]])

    A2 = np.array([[0., 1.],
                   [0., 0.]])

    A = scipylinalg.block_diag(A1, A1, A1, A2)

    B = np.array([
        [0,0,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [1,0,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [0,1,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [0,0,0,0],
        [0,0,1,0],
        [0,0,0,0],
        [0,0,0,1],
    ], dtype=float)


    return A, B


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
    y_pred_vals = x_pred[:, 4]  # y values
    z_pred_vals = x_pred[:, 8]  # z values

    # Create a 3D figure and axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the predicted trajectory
    t = np.array(t) 
    ax.plot(x_pred_vals, y_pred_vals, z_pred_vals, label="Predicted Trajectory", color='b', linestyle='--')
    xr = np.sin(np.pi * t/10) 
    yr = np.cos(np.pi * t/10) -1.0
    zr = np.sin(np.pi * t/10) + t

    ax.plot(xr, yr, zr, label="Reference Trajectory", color='r', linestyle='--')

    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()

def reference_state(t, ny):
    
    xr = np.sin(np.pi * t / 10.0)
    yr = np.cos(np.pi * t / 10.0) - 1.0
    zr = np.sin(np.pi * t / 10.0) + t

    yref = np.zeros(ny)
    yref[0] = xr   # x position
    yref[4] = yr   # y position
    yref[8] = zr   # z position
    # everything else stays zero → penalized against zero
    
    return yref


def get_disturbance(t):
    dwx = 0.15 * np.sin(np.pi * t / 100) + 0.1 * np.sin(0.2 * t) + 0.03 * np.sin(t)
    dwy = 0.0
    dwz = 0.0
    return np.array([dwx, dwy, dwz])
