import numpy as np

from model.Aircraft_parm import Aircraft_parm
from model.Cal_NED import CalNED

def Controller(states, Cmd, dt, Flit_Alt_Cmd, Flit_Psi_Cmd, Err_Alt_I):
    # Limiter_Rate_Vel = np.zeros([2,1], dtype=np.float64)
    # Limiter_Rate_Alt = np.zeros([2, 1], dtype=np.float64)
    # Limiter_Rate_Psi = np.zeros([2, 1], dtype=np.float64)

    Vel_Cmd = Cmd[0]
    Alt_Cmd = Cmd[1]
    Psi_Cmd = Cmd[2]

    Vel = states[4]
    Alt = states[9]
    Psi = states[6]

    Limiter_Rate_Vel = [2 * 0.8, -2 * 0.8]
    Limiter_Rate_Alt = [Aircraft_parm.Limit_dot_Alt[0] * 0.8, Aircraft_parm.Limit_dot_Alt[1] * 0.8]
    Limiter_Rate_Psi = [Aircraft_parm.Limit_dPsi[0] * 0.6, Aircraft_parm.Limit_dPsi[1] * 0.6]

    # Velocity-Thrust Controller
    if Vel_Cmd > Aircraft_parm.Limit_Vel_Cmd[0]:
        Vel_Cmd = Aircraft_parm.Limit_Vel_Cmd[0]
    elif Vel_Cmd < Aircraft_parm.Limit_Vel_Cmd[1]:
        Vel_Cmd = Aircraft_parm.Limit_Vel_Cmd[1]

    Aircraft_parm.Flit_Vel_Cmd = Vel
    dot_Vel = Vel_Cmd - Aircraft_parm.Flit_Vel_Cmd

    if dot_Vel > Limiter_Rate_Vel[0]:
        dot_Vel = Limiter_Rate_Vel[0]
    elif dot_Vel < Limiter_Rate_Vel[1]:
        dot_Vel = Limiter_Rate_Vel[1]

    Aircraft_parm.Flit_Vel_Cmd = Aircraft_parm.Flit_Vel_Cmd + dot_Vel * dt

    # Thrust Controller
    trim_thro = Aircraft_parm.Const_Thro[0] * Vel * Vel + Aircraft_parm.Const_Thro[1] * Vel + Aircraft_parm.Const_Thro[2]
    Err_Vel = Aircraft_parm.Flit_Vel_Cmd - Vel
    Aircraft_parm.Err_Vel_I = Aircraft_parm.Err_Vel_I + Err_Vel * dt

    Thro_Cmd = (trim_thro + Err_Vel * Aircraft_parm.Gain_Vel_P + Aircraft_parm.Err_Vel_I * Aircraft_parm.Gain_Vel_I) / Aircraft_parm.Factor_Thro

    if Thro_Cmd > Aircraft_parm.Limit_Thro[0]:
        Thro_Cmd = Aircraft_parm.Limit_Thro[0]
    elif Thro_Cmd < Aircraft_parm.Limit_Thro[1]:
        Thro_Cmd = Aircraft_parm.Limit_Thro[1]

    TCmd = Thro_Cmd * Aircraft_parm.Factor_Thro

    # Altitude-Alpha Controller ( longigudinal commad)
    dot_Alt = Alt_Cmd - Flit_Alt_Cmd
    if dot_Alt > Limiter_Rate_Alt[0]:
        dot_Alt = Limiter_Rate_Alt[0]
    elif dot_Alt < Limiter_Rate_Alt[1]:
        dot_Alt = Limiter_Rate_Alt[1]

    Flit_Alt_Cmd = Flit_Alt_Cmd + dot_Alt * dt

    Err_Alt = Flit_Alt_Cmd - Alt
    Err_Alt_I = Err_Alt_I + Err_Alt * dt

    Vel_N, Vel_E, Vel_D = CalNED(states)

    trim_alpha = Aircraft_parm.Const_Alpha[0] * Vel * Vel + Aircraft_parm.Const_Alpha[1] * Vel + Aircraft_parm.Const_Alpha[2]
    AlphaCmd = Err_Alt * Aircraft_parm.Gain_Alt_P + Err_Alt_I * Aircraft_parm.Gain_Alt_I + Vel_D * Aircraft_parm.Gain_Alt_D + trim_alpha * Aircraft_parm.D2R

    if AlphaCmd > Aircraft_parm.Limit_Alpha[0]:
        AlphaCmd = Aircraft_parm.Limit_Alpha[0]
    elif AlphaCmd < Aircraft_parm.Limit_Alpha[1]:
        AlphaCmd = Aircraft_parm.Limit_Alpha[1]

    # Psi_Phi Controller(lateral command)
    Psi_state = Flit_Psi_Cmd                                  # [Rad]
    Psi_state_dot = Psi_state + 180 * Aircraft_parm.D2R                     # [Rad]
    mod1, Psi_state_dot = divmod(Psi_state_dot, 360 * Aircraft_parm.D2R)
    cmd_dot = Psi_Cmd - Psi_state_dot

    # Short Turn Algorithm
    if Psi_state < 180 * Aircraft_parm.D2R and Psi_Cmd >= 180 * Aircraft_parm.D2R:
        if Psi_Cmd - Psi_state_dot >= 0:
            Direction = 0
        else:
            Direction = 1
    elif Psi_state < 180 * Aircraft_parm.D2R and Psi_Cmd < 180 * Aircraft_parm.D2R:
        if Psi_Cmd - Psi_state >= 0:
            Direction = 1
        else:
            Direction = 0
    elif Psi_state > 180 * Aircraft_parm.D2R and Psi_Cmd >= 180 * Aircraft_parm.D2R:
        if Psi_Cmd - Psi_state > 0:
            Direction = 1
        else:
            Direction = 0
    elif Psi_state > 180 * Aircraft_parm.D2R and Psi_Cmd < 180 * Aircraft_parm.D2R:
        if Psi_Cmd - Psi_state_dot >= 0:
            Direction = 0
        else:
            Direction = 1
    elif Psi_state == 180 * Aircraft_parm.D2R:
        if Psi_Cmd > 180 * Aircraft_parm.D2R:
            Direction = 1
        else:
            Direction = 0

    if Direction == 0:          # Left Turn
        if np.abs(cmd_dot) < 180 * Aircraft_parm.D2R:
            dot_Psi = - (180 * Aircraft_parm.D2R - cmd_dot)
        else:
            cmd_dot = np.abs(cmd_dot) - 180 * Aircraft_parm.D2R
            dot_Psi = - cmd_dot

    elif Direction == 1:            # Right Turn
        dot_Psi = Psi_Cmd - Flit_Psi_Cmd
        mod1, dot_Psi = divmod(dot_Psi, 360 * Aircraft_parm.D2R)

    if dot_Psi > Limiter_Rate_Psi[0]:
        dot_Psi = Limiter_Rate_Psi[0]
    elif dot_Psi < Limiter_Rate_Psi[1]:
        dot_Psi = Limiter_Rate_Psi[1]

    Flit_Psi_Cmd = Flit_Psi_Cmd + dot_Psi * dt
    mod1, Flit_Psi_Cmd = divmod(Flit_Psi_Cmd, 360 * Aircraft_parm.D2R)

    Err_Psi = Flit_Psi_Cmd - Psi
    mod1, Err_Psi = divmod(Err_Psi, 360 * Aircraft_parm.D2R)

    if Err_Psi > 180 * Aircraft_parm.D2R:
        Err_Psi = Err_Psi - 360 * Aircraft_parm.D2R

    PhiCmd = Err_Psi * Aircraft_parm.Gain_Psi_P * 14.76

    if PhiCmd > Aircraft_parm.Limit_Phi[0]:
        PhiCmd = Aircraft_parm.Limit_Phi[0]
    elif PhiCmd < Aircraft_parm.Limit_Phi[1]:
        PhiCmd = Aircraft_parm.Limit_Phi[1]

    return TCmd, AlphaCmd, PhiCmd, Flit_Alt_Cmd, Flit_Psi_Cmd, Err_Alt_I
