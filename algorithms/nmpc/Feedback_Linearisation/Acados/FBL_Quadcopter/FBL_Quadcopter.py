import numpy as np
from acados_template import AcadosModel,  AcadosSim
from casadi import SX, vertcat, sin, cos, Function
from utils import get_continous_time_matrices



def export_feedback_lineraise_Quadcopter_ode_model(dt = 0.1) -> AcadosModel:

    model_name = "FBL_Quadcopter_ode"

    #state : W = [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11,w12, w13]

    #control: V =[v0 , v1, v2, v3]


    # --------------------------
    # System (your matrices)
    # --------------------------

    A, B = get_continous_time_matrices()

    nw = A.shape[0]
    nv = B.shape[1]

    # set up states & controls
    w = SX.sym('w', nw)          # plant 
    
    v = SX.sym('v', nv)          # plant virtual controls 

    #zdot
    wdot = SX.sym('wdot', nw) 
  
    

    f_expl = A @ w + B @ v 

    f_impl = wdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = w

    model.xdot = wdot

    model.u = v

    model.name = model_name


    return model



def export_realcontrolInput_FBL_Quadcopter_model() -> AcadosModel:
    # TASK: implement this function,
    raise NotImplementedError()

    return model

