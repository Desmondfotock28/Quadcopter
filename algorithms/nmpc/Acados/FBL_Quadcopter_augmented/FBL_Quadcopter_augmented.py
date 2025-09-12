import numpy as np
from acados_template import AcadosModel,  AcadosSim
from casadi import SX, vertcat, sin, cos, Function
from utils import get_continous_time_matrices



def export_augmented_feedback_lineraise_Quadcopter_ode_model() -> AcadosModel:

    model_name = "FBL_augmented_Quadcopter_ode"

    #state : W = [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11,w12, w13]

    #control: V =[v0 , v1, v2, v3]
    
    #disturbance state  : D = [d0, d1, d2]
    # disturbance input : D_v = [u0, u1, u2]

    # --------------------------
    # System (your matrices)
    # --------------------------

    A, B, Bd = get_continous_time_matrices()

    nw = A.shape[0]
    nv = B.shape[1]
    nd = Bd.shape[1]

    # set up states & controls
    w = SX.sym('w', nw)          # plant 
    
    v = SX.sym('v', nv)          # plant virtual controls 

    v_d = SX.sym('v_d', nd)      # input disturbance 
    
    d = SX.sym('d', nd)          # plant disturbance 

    z = vertcat(w, d)           #augmented state
    v_aug = vertcat(v, v_d)

    #derivative 
    wdot = SX.sym('wdot', nw) 
    ddot = SX.sym('ddot', nd)
    
    zdot = vertcat(wdot, ddot)

    f_expl = vertcat(A @ w + B @ v + Bd @ d, v_d)

    f_impl = zdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = z

    model.xdot = zdot

    model.u = v_aug

    model.name = model_name


    return model



def export_realcontrolInput_FBL_Quadcopter_model() -> AcadosModel:
    # TASK: implement this function,
    raise NotImplementedError()

    return model

