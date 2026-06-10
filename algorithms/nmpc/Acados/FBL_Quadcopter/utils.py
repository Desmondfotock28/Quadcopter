
import matplotlib.pyplot as plt
import numpy as np
import scipy.linalg as scipylinalg
import os



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

     # Disturbance injection matrix: injects bias into rows 2, 6, 10
    dist_indices = [1, 5, 9]
    nd = len(dist_indices)

    Bd_cons = np.zeros((A.shape[0], nd))

    for j, idx in enumerate(dist_indices):
        Bd_cons[idx, j] = 1.0

    return A, B, Bd_cons


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

    # Plot the reference trajectory
    t = np.array(t) 
    ax.plot(x_pred_vals, y_pred_vals, z_pred_vals, label="Predicted Trajectory", color='b', linestyle='--')
    xr = np.sin(np.pi * t/10) 
    yr = np.cos(np.pi * t/10) -1.0
    zr = np.sin(np.pi * t/10) + t + 1.0

    ax.plot(xr, yr, zr, label="Reference Trajectory", color='r', linestyle='--')

    # Labels and legend
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    # Show the plot
    plt.show()


def _save(filename, trajectory):
    # Get the current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Save file directly in the same directory as this script
    filepath = os.path.join(script_dir, filename)

    # Write trajectory data
    with open(filepath, 'w') as file:
        for state in trajectory:
            file.write(' '.join(map(str, state)) + '\n')

    print(f" Trajectory saved to: {filepath}")


def reference_state(t, ny, terminal=True):
    
    xr = np.sin(np.pi * t / 10.0)
    yr = np.cos(np.pi * t / 10.0) - 1.0
    zr = np.sin(np.pi * t / 10.0) + t + 1.0

    yref = np.zeros(ny)
    yref[0] = xr   # x position
    yref[4] = yr   # y position
    yref[8] = zr   # z position

    if not terminal:
        yref[14] = 222   # v1
        yref[15] = 222   # v2
        yref[16] = 187   # v3
        yref[17] = 689   # v4
    # everything else stays zero → penalized against zero
    
    return yref

def generate_spiral_trajectory_two(starting_point, radius, steps, height=1.0):
    t_final = steps*0.1
    theta = np.linspace(0, t_final, steps+1)  # Two full circles

    xc, yc, zc = starting_point

    trajectory = []

    for t in theta:
       # sinusoidal circular trajectory
        x = xc + radius * np.sin(t / 10.0 * np.pi)       # sinusoidal x
        y = yc + radius * np.cos(t / 10.0 * np.pi) - 1.0 # sinusoidal y offset
        z = zc + np.sin(t / 10.0 * np.pi) + t * height + 1 # rising helix
        state = [x, 0, 0, 0, y, 0, 0, 0, z, 0, 0, 0, 0, 0, 6222, 222, 187, 689]
        trajectory.append(state)

    _save('spiral2.txt', trajectory)


def plot_3d_trajectory_test(simX, ref_traj):


    # Extract reference positions
    ref_x = ref_traj[:, 0]
    ref_y = ref_traj[:, 4]
    ref_z = ref_traj[:, 8]

    # Extract simulated positions
    sim_x = simX[:, 0]
    sim_y = simX[:, 4]
    sim_z = simX[:, 8]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ref_x, ref_y, ref_z, 'r--', label='Reference Spiral')
    ax.plot(sim_x, sim_y, sim_z, 'b', label='NMPC Trajectory')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Trajectory Tracking')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()



