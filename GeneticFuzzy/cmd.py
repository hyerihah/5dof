import numpy as np

def limit(value, range_):
    return max(range_[0], min(value, range_[1]))

def lla_to_ned(target, base):
    Re = 6378137
    return np.array([
        Re * (target[0] - base[0]),
        Re * np.cos(base[0]) * (target[1] - base[1]),
        -(target[2] - base[2])
    ])

def ned_to_lla(target, base):
    Re = 6378137
    return np.array([
        target[0] / Re + base[0],
        target[1] / (Re * np.cos(base[0])) + base[1],
        -target[2] + base[2]
    ])

def line_tracking(p_des, p_prev, p_now):
    base = p_prev
    u5 = lla_to_ned(p_des, base)
    psi_u5 = np.arctan2(u5[1], u5[0])
    
    u6 = lla_to_ned(p_now, base)
    psi_u6 = np.arctan2(u6[1], u6[0])
    
    gamma_leg = np.arctan(-u5[2] / np.sqrt(u5[0]**2 + u5[1]**2))
    psi_leg = np.arctan2(u5[1], u5[0])
    
    d_trk = (-u6[2]) - (-u5[2])
    rac = np.sqrt(u6[0]**2 + u6[1]**2 + u6[2]**2) * np.sin(psi_u6 - psi_u5)
    
    k_r = -0.001
    k_psi = -0.001
    
    gamma_cmd = gamma_leg + limit(k_r * d_trk, [-5 * np.pi / 180, 5 * np.pi / 180])
    psi_cmd = psi_leg + limit(k_psi * (-rac), [-20 * np.pi / 180, 20 * np.pi / 180])
    
    return gamma_cmd, psi_cmd

def fcn(target_info, vel_ctrl_type, mode, my_info, velocity_des, velocity_corner, continuous_turn):
    global flag_emergency, des_phi
    if 'flag_emergency' not in globals():
        flag_emergency = 0
        des_phi = 0
    
    vel_cmd, gamma_cmd, psi_cmd = 0, 0, 0
    mode_lat = 0
    
    target_lla = target_info[:3]
    target_v_ned = target_info[3:6]
    
    y = lla_to_ned(target_lla, my_info)
    range_ = np.linalg.norm(y)
    
    if vel_ctrl_type == 0:
        vel_cmd = np.linalg.norm(target_v_ned)
    elif vel_ctrl_type == 1:
        temp = range_ * 0.05
        vel_cmd = np.linalg.norm(target_v_ned) + limit(temp, [-300, 300])
    elif vel_ctrl_type == 2:
        vel_cmd = velocity_des
    elif vel_ctrl_type == 3:
        vel_cmd = np.linalg.norm(my_info[3:6])
    elif vel_ctrl_type == 4:
        vel_cmd = velocity_corner
    elif vel_ctrl_type == 5:
        temp = range_ * 0.01
        vel_cmd = velocity_corner + limit(temp, [-50, 50])
    
    mode_floor = int(np.floor(mode))
    if mode_floor == 104:  # Low yoyo
        gamma_cmd = limit(np.arctan2(-y[2], np.sqrt(y[0]**2 + y[1]**2)), [-5 * np.pi / 180, 10 * np.pi / 180])
        psi_cmd = np.arctan2(y[1], y[0])
        vel_cmd = velocity_des + 30
    elif mode_floor == 105:  # Head on
        if mode == 105:
            p_prev = ned_to_lla(target_info[3:6] * 100, target_info[:3])
            gamma_cmd, psi_cmd = line_tracking(target_info[:3], p_prev, my_info[:3])
        else:
            gamma_cmd = limit(np.arctan2(-y[2], np.sqrt(y[0]**2 + y[1]**2)), [-5 * np.pi / 180, 10 * np.pi / 180])
            psi_cmd = np.arctan2(y[1], y[0])
    elif mode_floor == 201:  # Basic Defense
        gamma_cmd = limit(target_info[0], [-5 * np.pi / 180, 10 * np.pi / 180])
        psi_cmd = target_info[1]
        mode_lat = 1
    elif mode_floor == 202:  # Break Turn
        gamma_cmd = limit(target_info[0], [-5 * np.pi / 180, 10 * np.pi / 180])
        psi_cmd = target_info[1]
        mode_lat = 1
        vel_cmd = velocity_corner
    else:
        gamma_cmd = limit(np.arctan2(-y[2], np.sqrt(y[0]**2 + y[1]**2)), [-5 * np.pi / 180, 10 * np.pi / 180])
        psi_cmd = np.arctan2(y[1], y[0])
    
    if psi_cmd > 100 * np.pi / 180:
        a = 1  # 이 부분은 MATLAB 코드에서 의도를 정확히 파악하기 어려움
    
    return vel_cmd, gamma_cmd, psi_cmd, mode_lat
