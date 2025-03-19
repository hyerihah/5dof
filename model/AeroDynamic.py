import numpy as np
from model.Aircraft_parm import Aircraft_parm

def AeroForce(Alpha, Vel, rho):
    F_Aero = np.array([0, 0])
    Cl = np.interp(Alpha, Aircraft_parm.AOA, Aircraft_parm.CL)
    Cd = np.interp(Alpha, Aircraft_parm.AOA, Aircraft_parm.CD)
    q = 0.5 * rho * Vel * Vel
    F_Aero[0] = q * Aircraft_parm.Sref * Cl
    F_Aero[1] = q * Aircraft_parm.Sref * Cd

    return F_Aero
