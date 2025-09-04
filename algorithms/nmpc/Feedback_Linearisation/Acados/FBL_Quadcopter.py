import numpy as np
from acados_template import AcadosModel,  AcadosSim
from casadi import SX, vertcat, sin, cos, Function
from utils import get_continous_time_matrices



def export_feedback_lineraise_Quadcopter_ode_model(dt = 0.1) -> AcadosModel:

    model_name = "FBL_Quadcopter_ode"

    #state : W = [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11,w12, w13]

    #disturbance : d =[d0, d1, d2]   use to model mismatch in the linear 
    
    #control: V =[v0 , v1, v2, v3]


    # --------------------------
    # System (your matrices)
    # --------------------------

    A, B, B_d = get_continous_time_matrices()

    nw = A.shape[0]
    nd = B_d.shape[1]
    nv = B.shape[1]

          # set up states & controls
    w = SX.sym('w', nw)          # plant 
    
    v = SX.sym('v', nv)          # plant virtual controls 

    d = SX.sym('d', nd)          # disturbance state (part of augmented state )

    #zdot
    wdot = SX.sym('wdot', nw) 
    ddot = SX.sym('ddot', nd)  

    zdot = vertcat(wdot, ddot)
     # augmented state: [w,d]
    
    z = vertcat(w, d) 

    f_expl = vertcat(A @ w + B @ v + B_d @ d, d)

    f_impl = zdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = z

    model.xdot = zdot

    model.u = v

    model.name = model_name


    return model



def export_realcontrolInput_FBL_Quadcopter_model() -> AcadosModel:
    # TASK: implement this function,
    raise NotImplementedError()

    return model

