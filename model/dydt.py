import math
import numpy as np

from model.Aircraft_parm import Aircraft_parm
from model.AeroDynamic import AeroForce
from model.Cal_Density import CalDensity
from model.Gravity_Force import GravityForce
from model.R_Earth import REarth


def RK4thOrder(init_states, u, dt):
    # curret_states = np.zeros([11,1], dtype=np.float64)
    curret_states = init_states
    k1 = dydt(init_states, u)
    init_states = curret_states + 0.5 * k1 * dt
    k2 = dydt(init_states, u)
    init_states = curret_states + 0.5 * k2 * dt
    k3 = dydt(init_states, u)
    init_states = curret_states + k3 * dt
    k4 = dydt(init_states, u)

    new_States = curret_states + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    if new_States[6] > math.pi:
        new_States[6] = new_States[6] - 2 * math.pi
    elif new_States[6] < -math.pi:
        new_States[6] = new_States[6] + 2 * math.pi

    return new_States

def dydt(states, u):
    states_dot = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
    rho = CalDensity(states[9])
    F_Aero = AeroForce(states[1], states[4], rho)
    F_grav = GravityForce(states[10])
    r_earth = REarth(states[7])

    # Alpha dot
    states_dot[0] = -2 * Aircraft_parm.zeta * Aircraft_parm.wn * states[0] - Aircraft_parm.wn * Aircraft_parm.wn * states[1] + Aircraft_parm.wn * Aircraft_parm.wn * u[1]
    # Alpha
    states_dot[1] = states[0]
    # Roll
    states_dot[2] = (u[2] - states[2]) / Aircraft_parm.tau_Roll
    # Thrust
    states_dot[3] = (u[0] - states[3]) / Aircraft_parm.tau_T
    # Velocity
    states_dot[4] = (states[3] * math.cos(states[1]) - F_Aero[1]) / states[10] - F_grav / states[10] * math.sin(states[5])
    # FlightPath
    states_dot[5] = (F_Aero[0] + states[3] * math.sin(states[1])) * math.cos(states[2]) / (
                states[10] * states[4]) - F_grav / states[10] / states[4] * math.cos(states[5])
    # Azimuth
    states_dot[6] = (F_Aero[0] + states[3] * math.sin(states[1])) * math.sin(states[2]) / (
                states[10] * states[4] * math.cos(states[5]))
    # Latitude
    states_dot[7] = states[4] * math.cos(states[5]) * math.cos(states[6]) / (r_earth[0] + states[9])
    # Longitude
    states_dot[8] = states[4] * math.cos(states[5]) * math.sin(states[6]) / (r_earth[1] + states[9]) / math.cos(states[7])
    # Altitude
    states_dot[9] = -1 * -1 * states[4] * math.sin(states[5])
    # Mass
    states_dot[10] = 0

    return states_dot
