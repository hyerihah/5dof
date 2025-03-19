import numpy as np

def AngleOff(Target_V, My_V):
    temp = np.dot(Target_V, My_V) / (np.linalg.norm(Target_V) * np.linalg.norm(My_V))
    return np.arccos(np.clip(temp, -1.0, 1.0))

def LLA2NED(target, base):
    Re = 6378137
    return np.array([
        Re * (target[0] - base[0]),
        Re * np.cos(base[0]) * (target[1] - base[1]),
        -(target[2] - base[2])
    ])

def NED2LLA(target, base):
    Re = 6378137
    return np.array([
        target[0] / Re + base[0],
        target[1] / (Re * np.cos(base[0])) + base[1],
        -target[2] + base[2]
    ])

def EulerTrans321(attitude):
    phi, theta, psi = attitude
    T1 = np.array([
        [1, 0, 0],
        [0, np.cos(phi), np.sin(phi)],
        [0, -np.sin(phi), np.cos(phi)]
    ])
    T2 = np.array([
        [np.cos(theta), 0, -np.sin(theta)],
        [0, 1, 0],
        [np.sin(theta), 0, np.cos(theta)]
    ])
    T3 = np.array([
        [np.cos(psi), np.sin(psi), 0],
        [-np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    return np.dot(T1, np.dot(T2, T3))


def call_Lag_Pursuit(Target_info, My_info, Weapon_Range_My):
    # case 101
    Target_LLA = Target_info[:3]
    Target_V_NED = Target_info[3:6]
    Target_length = Target_info[10]
    v_interval = -Target_V_NED / np.linalg.norm(Target_V_NED) * Target_length
    Position1 = NED2LLA(v_interval, Target_LLA)
    Y = LLA2NED(My_info, Position1)
    v_interval = Y / np.linalg.norm(Y) * Weapon_Range_My
    Position_t = NED2LLA(v_interval, Position1)
    Vel_ctrl_type = 1

    return Position_t, Vel_ctrl_type

def call_Pure_Pursuit(Target_info, My_info, Weapon_Range_My):
    # case 102
    Target_LLA = Target_info[:3]
    Y = LLA2NED(My_info, Target_LLA)
    v_interval = Y / np.linalg.norm(Y) * Weapon_Range_My
    Position_t = NED2LLA(v_interval, Target_LLA)
    Vel_ctrl_type = 1

    return Position_t, Vel_ctrl_type

def call_Lead_Pursuit(Target_info, My_info, Weapon_Range_My):
    # case 103
    Target_LLA = Target_info[:3]
    Target_V_NED = Target_info[3:6]
    Target_length = Target_info[10]
    K_radius = 0.1
    V_length = np.linalg.norm(Target_V_NED)
    v_interval = Target_V_NED / V_length * Target_length
    Position1 = NED2LLA(v_interval, Target_LLA)
    Y = LLA2NED(My_info, Position1)
    v_interval = Y / np.linalg.norm(Y) * Weapon_Range_My
    Position_t = NED2LLA(v_interval, Position1)
    Vel_ctrl_type = 1

    return Position_t, Vel_ctrl_type


def call_Low_yoyo(Target_info, My_info, Weapon_Range_My, flag_step, b_lead):
    # case 104
    dAlt = 500
    
    if flag_step == 0:
        temp = AngleOff(Target_info[3:6], My_info[3:6])
        Y = LLA2NED(Target_info[:3], My_info[:3])
        
        if np.linalg.norm(Y) <= Target_info[9] and temp < 10 * np.pi / 180:
            flag_step = 1
        
        if Target_info[2] < My_info[2]:
            flag_step = 1
        
        Position_t, _ = call_Lead_Pursuit(np.append(Target_info[:10], -b_lead), My_info[:3], Weapon_Range_My)
        Position_t[2] = Target_info[2] - dAlt
        Vel_ctrl_type = 2
    else:
        Position_t, _ = call_Lead_Pursuit(np.append(Target_info[:10], -b_lead), My_info[:3], Weapon_Range_My)
        Vel_ctrl_type = 1
    
    return Position_t, Vel_ctrl_type, flag_step


def call_HeadOn(Target_info, My_info, Weapon_Range_My, flag_step, V_Closing, b_lead):
    # case 105
    sep_distance = 700
    
    if flag_step == 0:
        Y = LLA2NED(My_info[:3], Target_info[:3])
        X = np.dot(EulerTrans321([0, 0, Target_info[13]]), Y)
        
        if abs(X[1]) > sep_distance:
            flag_step = 1
        
        X = np.array([0, sep_distance * np.sign(X[1]), 0])
        Y = np.dot(EulerTrans321([0, 0, Target_info[13]]).T, X)
        Position_t = NED2LLA(Y, Target_info[:3])
        Vel_ctrl_type = 2
    
    elif flag_step == 1:
        Y = LLA2NED(My_info[:3], Target_info[:3])
        X = np.dot(EulerTrans321([0, 0, Target_info[13]]), Y)
        
        if V_Closing > 0:
            flag_step = 2
        
        X = np.array([0, sep_distance * np.sign(X[1]), 0])
        Y = np.dot(EulerTrans321([0, 0, Target_info[13]]).T, X)
        Position_t = NED2LLA(Y, Target_info[:3])
        Vel_ctrl_type = 2
    
    else:
        Position_t, Vel_ctrl_type = call_Lead_Pursuit(np.append(Target_info[:10], b_lead), My_info[:3], Weapon_Range_My)
    
    return Position_t, Vel_ctrl_type, flag_step

def call_High_yoyo(Target_info, My_info, Weapon_Range_My, flag_step, V_Closing, para, b_lead):
    # case 106
    dAlt = para[0]
    gamma = 30 * np.pi / 180
    V_Closing_ref = 0
    
    if flag_step == 0:
        temp = AngleOff(Target_info[3:6], My_info[3:6])
        Y = LLA2NED(Target_info[:3], My_info[:3])
        
        if np.linalg.norm(Y) <= Target_info[9] and temp < 10 * np.pi / 180:
            flag_step = 2
        if V_Closing > V_Closing_ref:
            flag_step = 2
        if np.linalg.norm(My_info[3:6]) <= np.linalg.norm(Target_info[3:6]):
            flag_step = 1
        
        Position_t, _ = call_Lead_Pursuit(np.append(Target_info[:10], b_lead), My_info[:3], 0)
        dist = np.linalg.norm(Y[:2])
        Position_t[2] = Target_info[2] + dist * np.tan(gamma)
        Vel_ctrl_type = 0
    
    elif flag_step == 1:
        temp = AngleOff(Target_info[3:6], My_info[3:6])
        Y = LLA2NED(Target_info[:3], My_info[:3])
        
        if np.linalg.norm(Y) <= Target_info[9] and temp < 10 * np.pi / 180:
            flag_step = 2
        if V_Closing > V_Closing_ref:
            flag_step = 2
        
        Position_t, _ = call_Lead_Pursuit(np.append(Target_info[:10], b_lead), My_info[:3], 0)
        Position_t[2] = My_info[2]  # hold altitude
        Vel_ctrl_type = 0
    
    else:
        Position_t, Vel_ctrl_type = call_Lead_Pursuit(np.append(Target_info[:10], b_lead), My_info[:3], 0)
        Vel_ctrl_type = 5
    
    return Position_t, Vel_ctrl_type, flag_step

def call_Basic_Defense(Target_info, My_info, V_Closing, flag_step):
    # case 201
    global num_rand, hold_count
    
    if 'num_rand' not in globals():
        num_rand = np.random.rand(2)
        hold_count = 0
    
    if flag_step == 0:
        num_rand = np.random.rand(2)
        flag_step = 1
    
    Y = AngleOff(Target_info[3:6], My_info[3:6])
    if Y < 2 * np.pi / 180 and hold_count > 200:
        num_rand = np.random.rand(2)
        hold_count = 0
    hold_count += 1
    
    gamma_cmd = -5 * np.pi / 180 if num_rand[0] < 0.30 else 0 if num_rand[0] < 0.70 else 5 * np.pi / 180
    phi_cmd = -80 * np.pi / 180 if num_rand[1] < 0.40 else 0 if num_rand[1] < 0.60 else 80 * np.pi / 180
    
    Position_t = np.array([gamma_cmd, phi_cmd, 0])
    Vel_ctrl_type = 2
    
    return Position_t, Vel_ctrl_type, flag_step

def call_BreakTurn(Target_info, My_info, flag_step):
    # case 202
    global num_rand, hold_count
    
    if 'num_rand' not in globals():
        num_rand = np.random.rand()
        hold_count = 0
    
    if flag_step == 0:
        num_rand = np.random.rand()
        flag_step = 1
        hold_count = 0
    
    Y = AngleOff(Target_info[3:6], My_info[3:6])
    if Y < 2 * np.pi / 180 and hold_count > 200:
        num_rand = np.random.rand()
        hold_count = 0
    hold_count += 1
    
    phi_cmd = -np.arccos(1/8) if num_rand < 0.5 else np.arccos(1/8)
    gamma_cmd = 0
    
    Position_t = np.array([gamma_cmd, phi_cmd, 0])
    Vel_ctrl_type = 4
    
    return Position_t, Vel_ctrl_type, flag_step