# Import Basic function
import numpy as np
import math
import matplotlib.pyplot as plt 
import datetime
import os

# Import Parameters
from model.Aircraft_parm import Aircraft_parm

# Import function
from model.dydt import RK4thOrder

# Simulation Setting
dt = 0.1
T_sim = 10  # Simulation Time
timer = 0

# Set Init States
States = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
States[0] = 0                                   # Alpha dot[Rad]
States[1] = 1.131 * Aircraft_parm.D2R           # Alpha[Rad]
States[2] = 0                                   # Phi[Rad]
States[3] = 28460                               # Thrust[N]
States[4] = 250                                 # Velocity[m/s]
States[5] = 0                                   # Flight Path Angle(gamma)[Rad]
States[6] = 0                                   # Psi[Rad]
States[7] = 37 * Aircraft_parm.D2R              # Latitude[Rad]
States[8] = 126 * Aircraft_parm.D2R             # Longitude[Rad]
States[9] = 3000                                # Altitude[m]
States[10] = 5240.6                             # Mass[Kg]

# cmd = [thrust alpha phi]
u = np.array([30000, 5*Aircraft_parm.D2R, 10*Aircraft_parm.D2R], dtype=np.float64)

# 결과 저장 리스트
sn_history = [States.copy()]  # 초기 상태 저장

# Simulation Loop
while timer < T_sim:
    States = RK4thOrder(States, u, dt)  # 업데이트된 상태
    sn_history.append(States.copy())  # 리스트에 저장
    timer += dt

# 리스트를 numpy 배열로 변환 (각 행이 시간에 따른 상태 벡터)
sn_history = np.array(sn_history)

# 결과 플로팅
t_vals = np.linspace(0, T_sim, len(sn_history))


today_date = datetime.datetime.today().strftime('%Y-%m-%d')
save_dir = "png"
os.makedirs(save_dir, exist_ok=True)


plt.figure(figsize=(12, 8))

plt.title('F16 5DOF')
plt.subplot(3, 1, 1)
plt.plot(t_vals, sn_history[:, 3], label='Thrust (N)', color='g')
plt.plot(t_vals, [u[0]] * len(t_vals), label='Thrust Command (N)', linestyle='dashed', color='r')
plt.ylabel('Thrust (N)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 2)
plt.plot(t_vals, sn_history[:, 1]*Aircraft_parm.R2D, label='AOA (deg)', color='g')
plt.plot(t_vals, [u[1] * Aircraft_parm.R2D] * len(t_vals), label='AOA Command (deg)', linestyle='dashed', color='r')
plt.ylabel('AOA (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 3)
plt.plot(t_vals, sn_history[:, 2]*Aircraft_parm.R2D, label='Phi (deg)', color='g')
plt.plot(t_vals, [u[2] * Aircraft_parm.R2D]*len(t_vals), label='Phi Command (deg)', linestyle='dashed', color='r')
plt.xlabel('Time (s)')
plt.ylabel('Phi (deg)')
plt.grid()
plt.legend()

filename1 = os.path.join(save_dir, f"{today_date}_1.png")
plt.savefig(filename1, dpi=300)
print(f"Saved: {filename1}")
plt.close()

plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(t_vals, sn_history[:,7]*Aircraft_parm.R2D, label='Latitude (deg)', color='g')
plt.ylabel('Latitude (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 2)
plt.plot(t_vals, sn_history[:, 8]*Aircraft_parm.R2D, label='Longitude (deg)', color='g')
plt.ylabel('Longitude (deg)')
plt.grid()
plt.legend()


plt.subplot(3, 1, 3)
plt.plot(t_vals, sn_history[:, 9], label='Altitude (m)', color='g')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.grid()
plt.legend()

filename2 = os.path.join(save_dir, f"{today_date}_2.png")
plt.savefig(filename2, dpi=300)
print(f"Saved: {filename2}")
plt.close()

# plt.subplot(3, 1, 1)
# plt.plot(t_vals, sn_history[7, :]*Aircraft_parm.R2D, label='Psi (deg)', color='g')
# plt.ylabel('Thrust (N)')
# plt.grid()
# plt.legend()


# plt.subplot(3, 1, 2)
# plt.plot(t_vals, sn_history[:, 8]*Aircraft_parm.R2D, label='v (deg)', color='g')
# plt.ylabel('AOA (deg)')
# plt.grid()
# plt.legend()


fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(sn_history[:,7]*Aircraft_parm.R2D, sn_history[:,8]*Aircraft_parm.R2D, sn_history[:,9], label='Flight Path', color='b')
start_lat = sn_history[0, 7] * Aircraft_parm.R2D
start_lon = sn_history[0, 8] * Aircraft_parm.R2D
start_alt = sn_history[0, 9]
ax.scatter(start_lat, start_lon, start_alt, color='r', s=100, label="Start Point", edgecolors='black')

ax.set_xlabel('Latitude (deg)')
ax.set_ylabel('Longitude (deg)')
ax.set_zlabel('Altitude (m)')
ax.set_title('F16 5DOF Flight Trajectory')

plt.tight_layout()

filename3 = os.path.join(save_dir, f"{today_date}_3.png")
plt.savefig(filename3, dpi=300)
print(f"Saved: {filename3}")
plt.close()
