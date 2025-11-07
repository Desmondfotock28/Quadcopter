from acados_template import AcadosModel
from casadi import  MX, vertcat


def export_x500_ode_model() -> AcadosModel:

    model_name = "nmpc_flight_mode"

    """Define system parameters and constraints."""

    #  X500 PARAMETERS
    g = 9.8066        # m/s^2, gravitational acceleration
    m = 2.0         # kg, mass of the quadcopter
    jx, jy, jz = 0.02166, 0.02166, 0.04   # [kg.m^2] Inertia moment
    cd  = 8.06428e-05                               # Rotor drag coef
    dx = [0.174, 0.174, 0.174, 0.174]       # [m] Distance from center to rotors
    dy = [0.174, 0.174, 0.174, 0.174]       # [m] Distance from center to rotors


    # Model definition

    # State variables
    px, py, pz = MX.sym('px', 1), MX.sym('py', 1), MX.sym('pz', 1)
    vx, vy, vz = MX.sym('vx', 1), MX.sym('vy', 1), MX.sym('vz', 1)
    qw, qx, qy, qz = MX.sym('qw',1), MX.sym('qx',1), MX.sym('qy',1), MX.sym('qz',1), 
    wx, wy, wz = MX.sym('wx', 1), MX.sym('wy', 1), MX.sym('wz', 1)

    # Time derivative of state variables
    dpx, dpy, dpz =  MX.sym('dpx', 1),  MX.sym('dpy', 1),  MX.sym('dpz', 1)
    dvx, dvy, dvz =  MX.sym('dvx', 1),  MX.sym('dvy', 1),  MX.sym('dvz', 1)
    dqw, dqx, dqy, dqz =  MX.sym('dqw',1), MX.sym('dqx',1),  MX.sym('dqy',1),  MX.sym('dqz',1), 
    dwx, dwy, dwz =  MX.sym('dwx', 1),  MX.sym('dwy', 1),  MX.sym('dwz', 1)

        # Control variables
    u0 = MX.sym('u0')
    u1 = MX.sym('u1')
    u2 = MX.sym('u2')
    u3 = MX.sym('u3')

    # Stacked state and control variables
    x =  vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)
    xdot =  vertcat(dpx, dpy, dpz, dqw, dqx, dqy, dqz, dvx, dvy, dvz, dwx, dwy, dwz)
    u =  vertcat(u0, u1, u2, u3)
    

    # Model equations
    dpx = vx
    dpy = vy
    dpz = vz
    dqw = 0.5 * (-wx*qx - wy*qy - wz*qz)
    dqx = 0.5 * ( wx*qw + wz*qy - wy*qz)
    dqy = 0.5 * ( wy*qw - wz*qx + wx*qz)
    dqz = 0.5 * ( wz*qw + wy*qx - wx*qy)
    dvx = (2 * (u0+u1+u2+u3) / m) * (qy*qw + qz*qx)
    dvy = (2 * (u0+u1+u2+u3) / m) * (qy*qz - qw*qx)
    dvz = ((u0+u1+u2+u3) / m) * (qw*qw - qx*qx - qy*qy +qz*qz) - g
    dwx = (1/jx) * (-dx[0]*u0 - dx[1]*u1 + dx[2]*u2 + dx[3]*u3 - wy*jz*wz + wz*jy*wy)
    dwy = (1/jy) * ( dy[0]*u0 - dy[1]*u1 - dy[2]*u2 + dy[3]*u3 - wz*jx*wx + wx*jz*wz)
    dwz = (1/jz) * (   -cd*u0 +    cd*u1 -    cd*u2 +    cd*u3 - wx*jy*wy + wy*jx*wx)
   
    f_expl = vertcat(dpx, dpy, dpz, dqw, dqx, dqy, dqz, dvx, dvy, dvz, dwx, dwy, dwz)
    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = x

    model.xdot = xdot

    model.u = u

    model.name = model_name

    return model

