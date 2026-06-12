from acados_template import AcadosModel
from casadi import  vertcat, horzcat



def quadcopter_dynamics(x, u):

    # ---------------------------
    # F450 PARAMETERS
    # ---------------------------
    m = 2.0                                     # [kg] total mass
    g = 9.8066                                  # [m/s^2] Gravity   
    jx, jy, jz = 0.0035, 0.0035, 0.005         # [kg.m^2] Inertia moment
    cd  = 1.6e-7                               # Rotor drag coef
    dx = [0.225, 0.225, 0.225, 0.225]          # [m] Distance from center to rotors
    dy = [0.225, 0.225, 0.225, 0.225]          # [m] Distance from center to rotors
        
    
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