import numpy as np
from acados_template import AcadosModel,  AcadosSim
from casadi import SX
from utils import get_continous_time_matrices



def export_feedback_lineraise_Quadcopter_ode_model() -> AcadosModel:

    model_name = "FBL_Quadcopter_DOB_ode"

    #state : W = [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11,w12, w13]

    #control: V =[v0 , v1, v2, v3]


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

    p = SX.sym('p', nd)         #disturbance parameter 

    #zdot
    wdot = SX.sym('wdot', nw) 
  

    f_expl = A @ w + B @ v + Bd@p

    f_impl = wdot - f_expl

    model = AcadosModel()

    model.f_impl_expr = f_impl

    model.f_expl_expr = f_expl

    model.x = w

    model.xdot = wdot

    model.u = v

    model.p = p

    model.name = model_name


    return model


def export_quadcopter_realplant_model() -> AcadosModel:
    """
    Simulation model (true plant) of the quadcopter with constant disturbance.
    """

    model_name = "Quadcopter_realplant"

    # --------------------------
    # System matrices
    # --------------------------
    A, B, Bd = get_continous_time_matrices()   # <- ensure Bd is returned in utils
    nw = A.shape[0]
    nv = B.shape[1]

    # States & controls
    w = SX.sym('w', nw)          # plant state
    v = SX.sym('v', nv)          # plant control
    
    # Derivative
    wdot = SX.sym('wdot', nw)

    # Example: fixed constant disturbance vector
    d_known = np.array([0.12, -0.08, 0.05])  # could also be symbolic if estimated


    # Dynamics
    f_expl = A @ w + B @ v + Bd @ d_known

    f_impl = wdot - f_expl

    # Build model
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

