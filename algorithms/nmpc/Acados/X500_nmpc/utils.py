
import matplotlib.pyplot as plt
import numpy as np



def plot_trajectory(xs, reference_trajectory, us, ocp):
    """
    Plots 3D trajectory, positions vs reference, velocities vs reference, and control signals.

    Parameters:
    xs : np.ndarray
        State trajectory array of shape (N+1, state_dim)
    reference_trajectory : np.ndarray
        Reference trajectory array of same shape as xs
    us : np.ndarray
        Control signal array of shape (N, control_dim)
    ocp : object
        Object with attribute dims.N specifying horizon length
    """

    # --- 3D trajectory plot ---
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(xs[:, 0], xs[:, 1], xs[:, 2], label='3D Line', color='b')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Automatically set limits based on data
    ax.set_xlim(np.min(xs[:, 0]), np.max(xs[:, 0]))
    ax.set_ylim(np.min(xs[:, 1]), np.max(xs[:, 1]))
    ax.set_zlim(np.min(xs[:, 2]), np.max(xs[:, 2]))

    ax.legend()
    plt.title('3D Trajectory')
    plt.show()

    # --- Time array for states ---
    t_state = np.linspace(0, ocp.dims.N, ocp.dims.N+1)


    # Positions vs reference
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t_state, xs[:, 0], label='x')
    ax.plot(t_state, xs[:, 1], label='y')
    ax.plot(t_state, xs[:, 2], label='z')
    ax.plot(t_state, reference_trajectory[:, 0], '--', label='x_ref')
    ax.plot(t_state, reference_trajectory[:, 1], '--', label='y_ref')
    ax.plot(t_state, reference_trajectory[:, 2], '--', label='z_ref')
    ax.set_xlabel('Time step')
    ax.set_ylabel('Position')
    ax.legend()
    plt.title('Positions vs Reference')
    plt.show()

    # Velocities vs reference
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(t_state, xs[:, 7], label='vx')
    ax.plot(t_state, xs[:, 8], label='vy')
    ax.plot(t_state, xs[:, 9], label='vz')
    ax.plot(t_state, reference_trajectory[:, 7], '--', label='vx_ref')
    ax.plot(t_state, reference_trajectory[:, 8], '--', label='vy_ref')
    ax.plot(t_state, reference_trajectory[:, 9], '--', label='vz_ref')
    ax.set_xlabel('Time step')
    ax.set_ylabel('Velocity')
    ax.legend()
    plt.title('Velocities vs Reference')
    plt.show()

    # --- Time array for controls ---
    t_control = np.linspace(0, ocp.dims.N-1, ocp.dims.N)

    # Control signals
    fig, ax = plt.subplots(figsize=(8, 6))
    for i in range(us.shape[1]):
        ax.plot(t_control, us[:, i], label=f'f{i+1}')
    ax.set_xlabel('Time step')
    ax.set_ylabel('Control input')
    ax.legend()
    plt.title('Control Signals')
    plt.show()




