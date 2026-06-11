import numpy as np
import matplotlib.pyplot as plt


def reference_trajectory(t, omega=np.pi):
    xref = np.zeros(12)
    xref[0] = np.sin(omega * t / 10)
    xref[1] = np.cos(omega * t / 10) - 1.0
    xref[2] = np.sin(omega * t / 10) + t
    return xref


def generate_block_identity(n_rows, n_cols):
    matrix = np.zeros((n_rows, n_cols))
    matrix[: min(n_rows, n_cols), : min(n_rows, n_cols)] = np.eye(min(n_rows, n_cols))
    return matrix


def split_stage_matrix(t1, stage, nu):
    return t1[stage * nu:(stage + 1) * nu, :]


def split_stage_vector(u_stack, stage, nu):
    return u_stack[stage * nu:(stage + 1) * nu]


def reconstruct_input_stack(t1, v_active, mu, inactive_stack):
    return t1 @ v_active + mu * inactive_stack


def shift_input_stack(u_stack, terminal_u, nu):
    return np.concatenate((u_stack[nu:], terminal_u))


def plot_xyz_subplots(t, x_pred, output_path=None, show=True):
    t = np.array(t)
    xr = np.sin(np.pi * t / 10)
    yr = np.cos(np.pi * t / 10) - 1.0
    zr = np.sin(np.pi * t / 10) + t

    fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    for axis, values, ref, label in [
        (axs[0], x_pred[:, 0], xr, "X"),
        (axs[1], x_pred[:, 1], yr, "Y"),
        (axs[2], x_pred[:, 2], zr, "Z"),
    ]:
        axis.plot(t, values, label=f"Predicted {label}")
        axis.plot(t, ref, label=f"Reference {label}", linestyle="--")
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend()
    axs[2].set_xlabel("Time")
    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def plot_3d_trajectory(t, x_pred, output_path=None, show=True):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    t = np.array(t)
    ax.plot(x_pred[:, 0], x_pred[:, 1], x_pred[:, 2], label="Predicted Trajectory", linestyle="--")
    ax.plot(np.sin(np.pi * t / 10), np.cos(np.pi * t / 10) - 1.0, np.sin(np.pi * t / 10) + t, label="Reference Trajectory", linestyle="--")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    if output_path:
        plt.savefig(output_path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)
