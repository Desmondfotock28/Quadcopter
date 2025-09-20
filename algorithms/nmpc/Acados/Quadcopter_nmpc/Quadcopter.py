
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, tan, Function



def export_Quadcopter_ode_model() -> AcadosModel:

    model_name = 'Quadcopter_ode'

    """Define system parameters and constraints."""

    # System parameters 
    g = 9.81        # m/s^2, gravitational acceleration
    m = 2.0         # kg, mass of the quadcopter
    k = 9.8e-6      # N·s^2/rad^2, thrust coefficient (relates rotor speed squared to thrust) T = kw^2
    l = 0.225       # m, distance from the center to each rotor (arm length)
    b = 1.6e-7      # N·m·s^2/rad^2, drag/torque coefficient (relates rotor speed squared to torque) tau_M= bw^2 + I_Mw_dot
    Ixx = 0.035    # kg·m^2, moment of inertia around x-axis
    Iyy = 0.035     # kg·m^2, moment of inertia around y-axis
    Izz = 0.005     # kg·m^2, moment of inertia around z-axis
    cm = 10000      # v^-2·s^-2, motor constant (relates control input to rotor speed squared)  
    kd = 0.25       # kg/s, linear drag coefficient (damping due to air resistance)  : drag coefficient 
    
    # set up states & controls
    #  Parameters:
    # - x: State vector [x, y, z, phi, theta, psi, dx, dy, dz, p, q, r]
    #- u: Input vector [u1, u2, u3, u4] (control inputs/voltages at four motors)

    #Returns:
    # - dx: State derivatives

     #adding disturbance 
     # Disturbances (set to 0 if not used)
    dwx, dwy, dwz = 0.0, 0.0, 0.0

    nx  = 12 
    nu  = 4
    x      = SX.sym('x', nx)

    u   = SX.sym('u', nu)

    p   = SX.sym('p', nx)
   
    # xdot
    xdot      = SX.sym('xdot', nx)

    # dynamics

    dx0 = x[6]

    dx1 = x[7]

    dx2 = x[8]

    dx3 = x[9] + x[10]*(sin(x[3])*tan(x[4])) + x[11]*(cos(x[3])*tan(x[4]))

    dx4 = x[10]*(cos(x[3])) - x[11]*(sin(x[3]))

    dx5 = (sin(x[3])/cos(x[4]))*x[10] + (cos(x[3])/cos(x[4]))*x[11]

    dx6 =  (k*cm/m)*(sin(x[5])*sin(x[3])+cos(x[5])*cos(x[3])*sin(x[4]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwx   #(-kd/m)*x[6] ignore for now
    
    dx7 = (k*cm/m)*(cos(x[3])*sin(x[5])*sin(x[4])- cos(x[5])*sin(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwy   # (-kd/m)*x[7] +

    dx8 = -g + (k*cm/m)*(cos(x[4])*cos(x[3]))*(u[0]**2 + u[1]**2 + u[2]**2+u[3]**2) + dwz  #(-kd/m)*x[8] 

    dx9 = (l*k*cm/Ixx)*(u[0]**2-u[1]**2-u[2]**2 + u[3]**2)-((Iyy-Izz)/Ixx)*x[10]*x[11]

    dx10 = (l*k*cm/Iyy)*(u[0]**2+u[1]**2-u[2]**2 - u[3]**2)-((Izz-Ixx)/Iyy)*x[9]*x[11]

    dx11 = (b*cm/Izz)*(u[0]**2-u[1]**2+u[2]**2-u[3]**2)-((Ixx-Iyy)/Izz)*x[9]*x[10]
   
    f_expl = vertcat( dx0, dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11)

    f_impl = xdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = x

    model.xdot = xdot

    model.u = u

    model.p = p
   
    model.name = model_name

    return model



def export_parametric_Quadcopter_ode_model() -> AcadosModel:
    # TASK: implement this function, you can copy/paste most from above.
    raise NotImplementedError()

    return model


