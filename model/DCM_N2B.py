import math
import numpy as np

def C_N2B(Alpha, Phi, Gamma, Psi):
    C2_Alpha = np.zeros((3, 3), dtype=np.float64)
    C1_Phi = np.zeros((3, 3), dtype=np.float64)
    C2_Gamma = np.zeros((3, 3), dtype=np.float64)
    C3_Psi = np.zeros((3, 3), dtype=np.float64)

    C2_Alpha[0, 0] = math.cos(Alpha)
    C2_Alpha[0, 2] = -math.sin(Alpha)
    C2_Alpha[1, 1] = 1
    C2_Alpha[2, 0] = math.sin(Alpha)
    C2_Alpha[2, 2] = math.cos(Alpha)

    C1_Phi[0, 0] = 1
    C1_Phi[1, 1] = math.cos(Phi)
    C1_Phi[1, 2] = math.sin(Phi)
    C1_Phi[2, 1] = -math.sin(Phi)
    C1_Phi[2, 2] = math.cos(Phi)

    C2_Gamma[0, 0] = math.cos(Gamma)
    C2_Gamma[0, 2] = -math.sin(Gamma)
    C2_Gamma[1, 1] = 1
    C2_Gamma[2, 0] = math.sin(Gamma)
    C2_Gamma[2, 2] = math.cos(Gamma)

    C3_Psi[0, 0] = math.cos(Psi)
    C3_Psi[0, 1] = math.sin(Psi)
    C3_Psi[1, 0] = -math.sin(Psi)
    C3_Psi[1, 1] = math.cos(Psi)
    C3_Psi[2, 2] = 1

    return C2_Alpha, C1_Phi, C2_Gamma, C3_Psi


