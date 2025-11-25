from acados_template import AcadosModel
from casadi import  MX, vertcat, horzcat



def export_Quadcopter_ode_model() -> AcadosModel:
     
    model_name = "nmpc_flight_mode"

    """Define system parameters and constraints."""

    # ---------------------------
    # F450 PARAMETERS
    # ---------------------------
    m = 1.65                                    # [kg] total mass
    g = 9.8066                                  # [m/s^2] Gravity   
    jx, jy, jz = 0.014959, 0.022005, 0.028033         # [kg.m^2] Inertia moment
    cd  = 0.000806428                             # Rotor drag coef
    dx = [0.16, 0.16, 0.16, 0.16]          # [m] Distance from center to rotors
    dy = [0.16, 0.16, 0.16, 0.16]          # [m] Distance from center to rotors

    

    # ---------------------------
    # X500  PARAMETERS
    # ---------------------------
                        
    #m = 2.0                                     # [kg] total mass
    #g = 9.8066                                  # [m/s^2] Gravity   
    #jx, jy, jz = 0.02166, 0.02166, 0.04        # [kg.m^2] Inertia moment
    #cd  = 8.06428e-05                           # Rotor drag coef
    #dx = [0.174, 0.174, 0.174, 0.174]          # [m] Distance from center to rotors
    #dy = [0.174, 0.174, 0.174, 0.174]          # [m] Distance from center to rotors

     # Model definition

    # ---------------------------
    # State variables
    # ---------------------------
    px, py, pz = MX.sym('px', 1), MX.sym('py', 1), MX.sym('pz', 1)      #position x y z
    vx, vy, vz = MX.sym('vx', 1), MX.sym('vy', 1), MX.sym('vz', 1)      #velocity dx dy dz
    qw, qx, qy, qz = MX.sym('qw',1), MX.sym('qx',1), MX.sym('qy',1), MX.sym('qz',1) #orientation in quaternions q = qw + qxi + qyj + qzk
    wx, wy, wz = MX.sym('wx', 1), MX.sym('wy', 1), MX.sym('wz', 1)                # angular velocities wx, wy, wz

    # ---------------------------
    # Time derivative of state variables
    # ---------------------------
    dpx, dpy, dpz =  MX.sym('dpx', 1),  MX.sym('dpy', 1),  MX.sym('dpz', 1)
    dvx, dvy, dvz =  MX.sym('dvx', 1),  MX.sym('dvy', 1),  MX.sym('dvz', 1)
    dqw, dqx, dqy, dqz =  MX.sym('dqw',1), MX.sym('dqx',1),  MX.sym('dqy',1),  MX.sym('dqz',1), 
    dwx, dwy, dwz =  MX.sym('dwx', 1),  MX.sym('dwy', 1),  MX.sym('dwz', 1)

    # ---------------------------
    # Control variables
    # ---------------------------
    u0 = MX.sym('u0')
    u1 = MX.sym('u1')
    u2 = MX.sym('u2')
    u3 = MX.sym('u3')

    # ---------------------------
    # Stacked state and control
    # ---------------------------
    x =  vertcat(px, py, pz, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz)
    xdot =  vertcat(dpx, dpy, dpz, dqw, dqx, dqy, dqz, dvx, dvy, dvz, dwx, dwy, dwz)
    u =  vertcat(u0, u1, u2, u3)
    

    # ---------------------------
    # Model equations
    # ---------------------------

    # Position derivatives 
    dpx = vx
    dpy = vy
    dpz = vz

    # Quaternion kinematics
    dqw = 0.5 * (-wx*qx - wy*qy - wz*qz)
    dqx = 0.5 * ( wx*qw + wz*qy - wy*qz)
    dqy = 0.5 * ( wy*qw - wz*qx + wx*qz)
    dqz = 0.5 * ( wz*qw + wy*qx - wx*qy)

    # Rotation matrix from quaternion
    R = vertcat(
      horzcat(1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)),
      horzcat(2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)),
      horzcat(2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)))
    
    # Linear  acceleration
    F_total = u0 + u1 + u2 + u3
    force_body =  vertcat(0, 0, F_total)
    acc_inertial = (1/m) * R @ force_body - vertcat(0, 0, g)
    dvx, dvy, dvz = acc_inertial[0], acc_inertial[1], acc_inertial[2]

    # Angular acceleration

    Mx = -dx[0]*u0 - dx[1]*u1 + dx[2]*u2 + dx[3]*u3
    My =  dy[0]*u0 - dy[1]*u1 - dy[2]*u2 + dy[3]*u3
    Mz = -cd*u0 + cd*u1 - cd*u2 + cd*u3 

   # Gyroscopic effects
    dwx = (1/jx) * ( Mx - (wy*wz*(jz - jy)) )
    dwy = (1/jy) * ( My - (wz*wx*(jx - jz)) )
    dwz = (1/jz) * ( Mz - (wx*wy*(jy - jx)) )
   
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

