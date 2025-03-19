import numpy as np
import math
import matplotlib.pyplot as plt 
import datetime
import os

# Import Parameters
from model.Aircraft_parm import Aircraft_parm

# Import function
from model.dydt import RK4thOrder
from GeneticFuzzy.maneuver import call_Lag_Pursuit
from GeneticFuzzy.maneuver import call_Pure_Pursuit
from GeneticFuzzy.maneuver import call_Lead_Pursuit
from GeneticFuzzy.maneuver import call_Low_yoyo
from GeneticFuzzy.maneuver import call_HeadOn
from GeneticFuzzy.maneuver import call_High_yoyo
from GeneticFuzzy.maneuver import call_Basic_Defense
from GeneticFuzzy.maneuver import call_BreakTurn

from GeneticFuzzy.cmd import cmd

from GeneticFuzzy.control import Controller


# Simulation Setting
dt = 0.1
T_sim = 10  # Simulation Time
timer = 0

# Set Init Blue Team States
StatesB = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
StatesB[0] = 0                                   # Alpha dot[Rad]
StatesB[1] = 1.131 * Aircraft_parm.D2R           # Alpha[Rad]
StatesB[2] = 0                                   # Phi[Rad]
StatesB[3] = 28460                               # Thrust[N]
StatesB[4] = 250                                 # Velocity[m/s]
StatesB[5] = 0                                   # Flight Path Angle(gamma)[Rad]
StatesB[6] = 0                                   # Psi[Rad]
StatesB[7] = 37 * Aircraft_parm.D2R              # Latitude[Rad]
StatesB[8] = 126 * Aircraft_parm.D2R             # Longitude[Rad]
StatesB[9] = 3000                                # Altitude[m]
StatesB[10] = 5240.6                             # Mass[Kg]

# Set Init Red Team States
StatesR = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
StatesR[0] = 0                                   # Alpha dot[Rad]
StatesR[1] = 1.131 * Aircraft_parm.D2R           # Alpha[Rad]
StatesR[2] = 0                                   # Phi[Rad]
StatesR[3] = 28460                               # Thrust[N]
StatesR[4] = 250                                 # Velocity[m/s]
StatesR[5] = 0                                   # Flight Path Angle(gamma)[Rad]
StatesR[6] = 0                                   # Psi[Rad]
StatesR[7] = 37.46 * Aircraft_parm.D2R           # Latitude[Rad]
StatesR[8] = 126.22 * Aircraft_parm.D2R          # Longitude[Rad]
StatesR[9] = 3000                                # Altitude[m]
StatesR[10] = 5240.6                             # Mass[Kg]


# cmd = [thrust alpha phi]
# u = np.array([30000, 5*Aircraft_parm.D2R, 10*Aircraft_parm.D2R], dtype=np.float64)

u = [StatesB[3], StatesB[1], StatesB[2]]

# 결과 저장 리스트
sB_history = [StatesB.copy()]  # 초기 상태 저장
sR_history = [StatesR.copy()]  # 초기 상태 저장

# Simulation Loop
while timer < T_sim:

    [Position_t, Vel_ctrl_type] = call_Lag_Pursuit(StatesR, StatesB, 600)
    [vel_cmd, gamma_cmd, psi_cmd, mode_lat] = cmd(StatesR, Vel_ctrl_type, 101, StatesB, StatesB[4], 330)
    cmd = [vel_cmd, gamma_cmd, psi_cmd]
    [TCmd, AlphaCmd, PhiCmd, Flit_Alt_Cmd, Flit_Psi_Cmd, Err_Alt_I] = Controller(StatesB, cmd, dt, Flit_Alt_Cmd, Flit_Psi_Cmd, Err_Alt_I)

    uB = [TCmd, AlphaCmd, PhiCmd]

    StatesB = RK4thOrder(StatesB, uB, dt)  # 업데이트된 상태
    sB_history.append(StatesB.copy())  # 리스트에 저장

    StatesR = RK4thOrder(StatesR, u, dt)  # 업데이트된 상태
    sR_history.append(StatesR.copy())  # 리스트에 저장

    timer += dt

# 리스트를 numpy 배열로 변환 (각 행이 시간에 따른 상태 벡터)
sB_history = np.array(sB_history)
sR_history = np.array(sR_history)

# 결과 플로팅
tB_vals = np.linspace(0, T_sim, len(sB_history))
tR_vals = np.linspace(0, T_sim, len(sR_history))


today_date = datetime.datetime.today().strftime('%Y-%m-%d')
save_dir = "png"
os.makedirs(save_dir, exist_ok=True)


##### plot

plt.figure(figsize=(12, 8))

plt.title('B CMD')
plt.subplot(3, 1, 1)
plt.plot(tB_vals, sB_history[:, 3], label='Thrust (N)', color='g')
plt.plot(tB_vals, [uB[0]] * len(tB_vals), label='Thrust Command (N)', linestyle='dashed', color='r')
plt.ylabel('Thrust (N)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 2)
plt.plot(tB_vals, sB_history[:, 1]*Aircraft_parm.R2D, label='AOA (deg)', color='g')
plt.plot(tB_vals, [uB[1] * Aircraft_parm.R2D] * len(tB_vals), label='AOA Command (deg)', linestyle='dashed', color='r')
plt.ylabel('AOA (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 3)
plt.plot(tB_vals, sB_history[:, 2]*Aircraft_parm.R2D, label='Phi (deg)', color='g')
plt.plot(tB_vals, [u[2] * Aircraft_parm.R2D]*len(tB_vals), label='Phi Command (deg)', linestyle='dashed', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Phi (deg)')
plt.grid()
plt.legend()

filename1 = os.path.join(save_dir, f"{today_date}_1.png")
plt.savefig(filename1, dpi=300)
print(f"Saved: {filename1}")
plt.close()

plt.figure(figsize=(12, 8))

plt.title('B LLA')
plt.subplot(3, 1, 1)
plt.plot(tB_vals, sB_history[:,7]*Aircraft_parm.R2D, label='Latitude (deg)', color='g')
plt.ylabel('Latitude (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 2)
plt.plot(tB_vals, sB_history[:, 8]*Aircraft_parm.R2D, label='Longitude (deg)', color='g')
plt.ylabel('Longitude (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 3)
plt.plot(tB_vals, sB_history[:, 9], label='Altitude (m)', color='g')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.grid()
plt.legend()

filename2 = os.path.join(save_dir, f"{today_date}_2.png")
plt.savefig(filename2, dpi=300)
print(f"Saved: {filename2}")
plt.close()

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(sB_history[:,7]*Aircraft_parm.R2D, sB_history[:,8]*Aircraft_parm.R2D, sB_history[:,9], label='Flight Path', color='b')
startB_lat = sB_history[0, 7] * Aircraft_parm.R2D
startB_lon = sB_history[0, 8] * Aircraft_parm.R2D
startB_alt = sB_history[0, 9]
ax.scatter(startB_lat, startB_lon, startB_alt, color='b', s=100, label="Start Point", edgecolors='black')

ax.plot(sR_history[:,7]*Aircraft_parm.R2D, sR_history[:,8]*Aircraft_parm.R2D, sR_history[:,9], label='Flight Path', color='r')
startR_lat = sR_history[0, 7] * Aircraft_parm.R2D
startR_lon = sR_history[0, 8] * Aircraft_parm.R2D
startR_alt = sR_history[0, 9]
ax.scatter(startR_lat, startR_lon, startR_alt, color='r', s=100, label="Start Point", edgecolors='black')

ax.set_xlabel('Latitude (deg)')
ax.set_ylabel('Longitude (deg)')
ax.set_zlabel('Altitude (m)')
ax.set_title('F16 call_Lag_Pursuit CMD Flight Trajectory')

plt.tight_layout()

filename3 = os.path.join(save_dir, f"{today_date}_3.png")
plt.savefig(filename3, dpi=300)
print(f"Saved: {filename3}")
plt.close()
