from acados_template import AcadosModel
from casadi import SX, cos, reshape, sin, tan, vertcat


NX_QUAD = 12
NU_PHYSICAL = 4


def _quadcopter_dynamics(x, u, disturbance=None):
    g = 9.81
    m = 2.0
    k = 9.8e-6
    l = 0.225
    b = 1.6e-7
    ixx = 0.0035
    iyy = 0.0035
    izz = 0.005
    cm = 10000

    thrust_sum = u[0] ** 2 + u[1] ** 2 + u[2] ** 2 + u[3] ** 2

    dx0 = x[6]
    dx1 = x[7]
    dx2 = x[8]
    dx3 = x[9] + x[10] * (sin(x[3]) * tan(x[4])) + x[11] * (cos(x[3]) * tan(x[4]))
    dx4 = x[10] * cos(x[3]) - x[11] * sin(x[3])
    dx5 = (sin(x[3]) / cos(x[4])) * x[10] + (cos(x[3]) / cos(x[4])) * x[11]
    dx6 = (k * cm / m) * (sin(x[5]) * sin(x[3]) + cos(x[5]) * cos(x[3]) * sin(x[4])) * thrust_sum
    dx7 = (k * cm / m) * (cos(x[3]) * sin(x[5]) * sin(x[4]) - cos(x[5]) * sin(x[3])) * thrust_sum
    dx8 = -g + (k * cm / m) * (cos(x[4]) * cos(x[3])) * thrust_sum
    dx9 = (l * k * cm / ixx) * (u[0] ** 2 - u[1] ** 2 - u[2] ** 2 + u[3] ** 2) - ((iyy - izz) / ixx) * x[10] * x[11]
    dx10 = (l * k * cm / iyy) * (u[0] ** 2 + u[1] ** 2 - u[2] ** 2 - u[3] ** 2) - ((izz - ixx) / iyy) * x[9] * x[11]
    dx11 = (b * cm / izz) * (u[0] ** 2 - u[1] ** 2 + u[2] ** 2 - u[3] ** 2) - ((ixx - iyy) / izz) * x[9] * x[10]

    if disturbance is not None:
        dx6 += disturbance[0]
        dx7 += disturbance[1]
        dx8 += disturbance[2]

    return vertcat(dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11)


def export_active_subspace_quadcopter_model(nv_active: int) -> AcadosModel:
    """Quadcopter model with horizon-level active-subspace variables.

    The augmented states are [quad_state, V, mu]. V and mu have zero dynamics,
    so acados optimizes one horizon-level active vector rather than 4 controls
    at every stage. The physical motor voltages are reconstructed per stage from
    stage parameters: u_real = T1_stage @ V + mu * inactive_stage.
    """

    model_name = f"Quadcopter_active_subspace_nv{nv_active}"

    nx_aug = NX_QUAD + nv_active + 1
    x = SX.sym("x", nx_aug)
    xdot = SX.sym("xdot", nx_aug)

    # A fixed dummy control keeps the OCP in the standard acados control form.
    u_dummy = SX.sym("u_dummy", 1)

    np_stage = NX_QUAD + NU_PHYSICAL * nv_active + NU_PHYSICAL
    p = SX.sym("p", np_stage)

    x_quad = x[:NX_QUAD]
    v_active = x[NX_QUAD:NX_QUAD + nv_active]
    mu = x[NX_QUAD + nv_active]

    ref = p[:NX_QUAD]
    t1_flat = p[NX_QUAD:NX_QUAD + NU_PHYSICAL * nv_active]
    inactive = p[NX_QUAD + NU_PHYSICAL * nv_active:]

    t1_stage = reshape(t1_flat, NU_PHYSICAL, nv_active)
    u_real = t1_stage @ v_active + mu * inactive

    f_quad = _quadcopter_dynamics(x_quad, u_real)
    f_expl = vertcat(f_quad, SX.zeros(nv_active), 0)
    f_impl = xdot - f_expl

    q_mat = SX.zeros(NX_QUAD, NX_QUAD)
    q_diag = [40, 40, 50, 5, 5, 5, 2, 2, 2, 1, 1, 1]
    for i, weight in enumerate(q_diag):
        q_mat[i, i] = weight

    r_mat = 0.1 * SX.eye(NU_PHYSICAL)
    u_hover = vertcat(5.75, 5.75, 5.75, 5.75)
    tracking_error = x_quad - ref
    input_error = u_real - u_hover

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u_dummy
    model.p = p
    model.con_h_expr = u_real
    model.cost_expr_ext_cost = 0.5 * (tracking_error.T @ q_mat @ tracking_error + input_error.T @ r_mat @ input_error + 1e-3 * (mu - 1) ** 2)
    model.cost_expr_ext_cost_e = tracking_error.T @ q_mat @ tracking_error
    model.name = model_name

    return model


def export_quadcopter_realplant_model() -> AcadosModel:
    model_name = "Quadcopter_realplant_active_subspace"

    x = SX.sym("x", NX_QUAD)
    u = SX.sym("u", NU_PHYSICAL)
    xdot = SX.sym("xdot", NX_QUAD)

    f_expl = _quadcopter_dynamics(x, u, disturbance=(0.12, -0.08, 0.05))

    model = AcadosModel()
    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.name = model_name

    return model
