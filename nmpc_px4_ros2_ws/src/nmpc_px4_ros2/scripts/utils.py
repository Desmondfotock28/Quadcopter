import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from casadi import vertcat
import os
import math



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
    zr = np.sin(np.pi * t/10) + t + 1

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
    xr = np.sin(np.pi * t / 10)
    yr = np.cos(np.pi * t / 10) - 1.0
    zr = np.sin(np.pi * t / 10) + t + 1

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



def generate_spiral_trajectory(starting_point, radius, steps, height):
    theta = np.linspace(0, 4.0*np.pi, steps+1)  # Two full circles

    xc, yc, zc = starting_point
    xc -= radius
    trajectory = []

    for t in theta:
        x = xc + radius * math.cos(t)
        y = yc + radius * math.sin(t)
        z = zc + (height / (2.0 * np.pi)) * t  # Linear increase in z
        state = [x, y, z, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0452, 4.0452, 4.0452, 4.0452]
        trajectory.append(state)

    _save('spiral.txt', trajectory)


def generate_spiral_trajectory_two(starting_point, radius, steps, height=1.0):
    t_final = steps*0.005
    theta = np.linspace(0, t_final, steps+1)  # Two full circles

    xc, yc, zc = starting_point

    trajectory = []

    for t in theta:
       # sinusoidal circular trajectory
        x = xc + radius * np.sin(t / 10.0 * np.pi)       # sinusoidal x
        y = yc + radius * np.cos(t / 10.0 * np.pi) - 1.0 # sinusoidal y offset
        z = zc + np.sin(t / 10.0 * np.pi) + t * height + 1 # rising helix
        state = [x, y, z, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0452, 4.0452, 4.0452, 4.0452]
        trajectory.append(state)

    _save('spiral2.txt', trajectory)


def plot_3d_trajectory_test(simX, ref_traj):
    """
    Plots the 3D position trajectory of the NMPC simulation vs reference.
    Assumes state structure: [x, y, z, q0, q1, q2, q3, vx, vy, vz, wx, wy, wz]
    """

    # Extract reference positions
    ref_x = ref_traj[:, 0]
    ref_y = ref_traj[:, 1]
    ref_z = ref_traj[:, 2]

    # Extract simulated positions
    sim_x = simX[:, 0]
    sim_y = simX[:, 1]
    sim_z = simX[:, 2]

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(ref_x, ref_y, ref_z, 'r--', label='Reference Spiral')
    ax.plot(sim_x, sim_y, sim_z, 'b--', label='NMPC Trajectory')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.set_title('3D Trajectory Tracking')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()
    plt.show()


def plot_xyz_subplots_test(t, ref_traj, x_pred):
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

    t = np.array(t)
    # Extract reference positions
    ref_x = ref_traj[:, 0]
    ref_y = ref_traj[:, 1]
    ref_z = ref_traj[:, 2]

    # Create subplots
    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

    # Plot X
    axs[0].plot(t, x_vals, label="Predicted X", color='b')
    axs[0].plot(t, ref_x, label="Reference X", color='r', linestyle='--')
    axs[0].set_ylabel("X")
    axs[0].legend()
    axs[0].grid(True)

    # Plot Y
    axs[1].plot(t, y_vals, label="Predicted Y", color='b')
    axs[1].plot(t, ref_y, label="Reference Y", color='r', linestyle='--')
    axs[1].set_ylabel("Y")
    axs[1].legend()
    axs[1].grid(True)

    # Plot Z
    axs[2].plot(t, z_vals, label="Predicted Z", color='b')
    axs[2].plot(t, ref_z, label="Reference Z", color='r', linestyle='--')
    axs[2].set_ylabel("Z")
    axs[2].set_xlabel("Time")
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()