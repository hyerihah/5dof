import math

def CalNED(States):
    Vel = States[4]
    Gamma = States[5]
    Psi = States[6]

    Vel_N = Vel * math.cos(Gamma) * math.cos(Psi)
    Vel_E = Vel * math.cos(Gamma) * math.cos(Psi)
    Vel_D = -1 * Vel * math.sin(Gamma)

    return Vel_N, Vel_E, Vel_D
