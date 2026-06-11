from casadi import  SX, vertcat, horzcat, reshape


nx = 13

nu_phy= 4


# ---------------------------
# F450 PARAMETERS
# ---------------------------
m = 2.0                                     # [kg] total mass
g = 9.8066                                  # [m/s^2] Gravity   
jx, jy, jz = 0.0035, 0.0035, 0.005         # [kg.m^2] Inertia moment
# Yaw torque per unit thrust [N.m/N]. The mixer below works with motor FORCES,
# so this must be the torque-to-thrust ratio (Gazebo's momentConstant), not a
# rotor-speed-squared drag coefficient. Value matches this repo's F450 model
# (model/quad_f450_camera/model.sdf: momentConstant = 0.017); PX4's stock iris
# uses 0.016. The previous 1.6e-7 was a speed^2-domain coefficient applied to
# forces -- ~5 orders of magnitude too small, leaving yaw uncontrolled.
cd  = 0.017
dx = [0.225, 0.225, 0.225, 0.225]          # [m] Distance from center to rotors
dy = [0.225, 0.225, 0.225, 0.225]          # [m] Distance from center to rotors

thrust_to_weight = 1.75

max_force_per_motor = (g * m / 4.0) * thrust_to_weight

# Hover equilibrium thrust per motor: the input cost must be centred here,
# NOT at max_force_per_motor, otherwise the R term rewards flying at full
# throttle and biases every solution toward the upper input bound.
u_hover_per_motor = m * g / 4.0

#cost weights
Q_diag = [40, 40, 50, 1.0,  0.043, 0.043, 0.043,2.0, 2.0, 2.0, 1.0, 1.0, 1.0]

R_weight = 0.1


def quadcopter_dynamics(x, u):
     
   
    # ---------------------------
    # Model equations
    # ---------------------------

    # Position derivatives 
    dpx = x[7]
    dpy = x[8]
    dpz = x[9]

    # Quaternion kinematics
    dqw = 0.5 * (-x[10]*x[4] - x[11]*x[5] - x[12]*x[6])
    dqx = 0.5 * ( x[10]*x[3] + x[12]*x[5] - x[11]*x[6])
    dqy = 0.5 * ( x[11]*x[3] - x[12]*x[4] + x[10]*x[6])
    dqz = 0.5 * ( x[12]*x[3] + x[11]*x[4] - x[10]*x[5])

    # Rotation matrix from quaternion
    R = vertcat(
      horzcat(1 - 2*(x[5]**2 + x[6]**2), 2*(x[4]*x[5] - x[3]*x[6]), 2*(x[4]*x[6] + x[3]*x[5])),
      horzcat(2*(x[4]*x[5] + x[3]*x[6]), 1 - 2*(x[4]**2 + x[6]**2), 2*(x[5]*x[6] - x[3]*x[4])),
      horzcat(2*(x[4]*x[6] - x[3]*x[5]), 2*(x[5]*x[6] + x[3]*x[4]), 1 - 2*(x[4]**2 + x[5]**2)))
    
    # Linear  acceleration
    F_total = u[0] + u[1] + u[2] + u[3]
    force_body =  vertcat(0, 0, F_total)
    acc_inertial = (1/m) * R @ force_body - vertcat(0, 0, g)
    dvx, dvy, dvz = acc_inertial[0], acc_inertial[1], acc_inertial[2]

    # Angular acceleration

    Mx = -dx[0]*u[0] - dx[1]*u[1] + dx[2]*u[2] + dx[3]*u[3]
    My =  dy[0]*u[0] - dy[1]*u[1] - dy[2]*u[2] + dy[3]*u[3]
    Mz = -cd*u[0] + cd*u[1] - cd*u[2] + cd*u[3]

   # Gyroscopic effects
    dwx = (1/jx) * ( Mx - (x[11]*x[12]*(jz - jy)) )
    dwy = (1/jy) * ( My - (x[12]*x[10]*(jx - jz)) )
    dwz = (1/jz) * ( Mz - (x[10]*x[11]*(jy - jx)) )
   
 
    return vertcat(dpx, dpy, dpz, dqw, dqx, dqy, dqz, dvx, dvy, dvz, dwx, dwy, dwz)


def export_active_subspace_quadcopter_model(nv: int, u_cost_center: float = None, name_suffix: str = ""):

    """Quadcopter model with horizon-level active-subspace variables.

    The augmented states are [x, v, mu]. v and mu have zero dynamics,
    so acados optimizes one horizon-level active vector rather than 4 controls
    at every stage. The physical motor thrust are reconstructed per stage from
    stage parameters: u = T1_stage @ V + mu * T2_stage@w_tilda.
    """

    if u_cost_center is None:
        u_cost_center = u_hover_per_motor

    model_name = f"Quadcopter_active_subspace_nv{nv}{name_suffix}"

    nx_aug = nx + nv + 1
    x = SX.sym("x", nx_aug)
    xdot = SX.sym("xdot", nx_aug)

    # A fixed dummy control keeps the OCP in the standard acados control form.
    u_dummy = SX.sym("u_dummy", 1)

    np_stage = nx + nu_phy * nv + nu_phy
    p = SX.sym("p", np_stage)

    x_quad = x[:nx]

    v_active = x[nx:nx + nv]

    mu = x[nx + nv]

    ref = p[:nx]

    t1_flat = p[nx:nx + nu_phy * nv]
    
    inactive = p[nx + nu_phy * nv:]

    t1_stage = reshape(t1_flat, nu_phy, nv)
    u_real = t1_stage @ v_active + mu * inactive

    f_quad = quadcopter_dynamics(x_quad, u_real)

    f_expl = vertcat(f_quad, SX.zeros(nv), 0)

    f_impl = xdot - f_expl

    q_mat = SX.zeros(nx, nx)

    for i, weight in enumerate(Q_diag):
        q_mat[i, i] = weight

    r_mat = R_weight * SX.eye(nu_phy)

    u_hover = vertcat(*([u_cost_center] * nu_phy))

    tracking_error = x_quad - ref

    input_error = u_real - u_hover

    from acados_template import AcadosModel
    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = x

    model.xdot = xdot

    model.u = u_dummy

    model.p = p

    model.con_h_expr = u_real

    model.cost_expr_ext_cost = 0.5 * (tracking_error.T @ q_mat @ tracking_error + input_error.T @ r_mat @ input_error)

    model.cost_expr_ext_cost_e = tracking_error.T @ q_mat @ tracking_error

    model.name = model_name

    
    return model


def export_Quadcopter_ode_model():

    from acados_template import AcadosModel

    model_name = 'Quadcopter_ode'

    x = SX.sym("x", nx)
    u = SX.sym("u", nu_phy)
    xdot = SX.sym("xdot", nx)

    f_expl = quadcopter_dynamics(x, u)

    model = AcadosModel()

    model.f_impl_expr = xdot - f_expl

    model.f_expl_expr = f_expl

    model.x = x

    model.xdot = xdot
    
    model.u = u
    model.name = model_name

    return model



