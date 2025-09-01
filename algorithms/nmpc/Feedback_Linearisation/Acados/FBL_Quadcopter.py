import numpy as np
import scipy.linalg as scipylinalg
from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos, Function


def export_feedback_lineraise_Quadcopter_ode_model() -> AcadosModel:
    model_name = "FBL_Quadcopter_ode"

    #state : W = [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11,w12, w13]

    #disturbance : d =[d0, d1, d2]   use to model mismatch in the linear 
    
    #control: V =[v0 , v1, v2, v3]

    # augmented control 

    #control v_aug = [V, d ] = [v0 , v1, v2, v3, d0, d1, d2]

    # --------------------------
    # System (your matrices)
    # --------------------------

    A1 = np.array([[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.],[0.,0.,0.,0.]])

    A2 = np.array([[0.,1.],[0.,0.]])

    A = scipylinalg.block_diag(A1, A1, A1, A2)

    B = np.array([
    [0,0,0,0],
    [0,0,0,0],
    [0,0,0,0],
    [1,0,0,0],
    [0,0,0,0],
    [0,0,0,0],
    [0,0,0,0],
    [0,1,0,0],
    [0,0,0,0],
    [0,0,0,0],
    [0,0,0,0],
    [0,0,1,0],
    [0,0,0,0],
    [0,0,0,1],
   ], dtype=float)
    
    B_dist = np.array([
    [0,0,0],  # row 0
    [1,0,0],  # row 1 <- d0
    [0,0,0],  # row 2
    [0,0,0],  # row 3
    [0,0,0],  # row 4
    [0,1,0],  # row 5 <- d1
    [0,0,0],  # row 6
    [0,0,0],  # row 7
    [0,0,0],  # row 8
    [0,0,1],  # row 9 <- d2)
    [0,0,0],  # row 10
    [0,0,0],  # row 11
    [0,0,0],  # row 12
    [0,0,0],  # row 13
], dtype=float)



