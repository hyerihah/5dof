# Import Basic function
import numpy
import numpy as np
import math
import matplotlib.pyplot as plt 

# Import Parameters
from model.dydt import RK4thOrder

# Import function
from model.Aircraft_parm import Aircraft_parm

# Simulation Setting
dt = 0.1
T_sim = 10  # Simulation Time
timer = 0

# Set Init States A
StatesA = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
StatesA[0] = 0                                   # Alpha dot[Rad]
StatesA[1] = 1.131 * Aircraft_parm.D2R           # Alpha[Rad]
StatesA[2] = 0                                   # Phi[Rad]
StatesA[3] = 28460                               # Thrust[N]
StatesA[4] = 250                                 # Velocity[m/s]
StatesA[5] = 0                                   # Flight Path Angle(gamma)[Rad]
StatesA[6] = 0                                   # Psi[Rad]
StatesA[7] = 37 * Aircraft_parm.D2R              # Latitude[Rad]
StatesA[8] = 126 * Aircraft_parm.D2R             # Longitude[Rad]
StatesA[9] = 3000                                # Altitude[m]
StatesA[10] = 5240.6                             # Mass[Kg]

# Set Init States B
StatesB = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.float64)
StatesB[0] = 0                                   # Alpha dot[Rad]
StatesB[1] = 1.131 * Aircraft_parm.D2R           # Alpha[Rad]
StatesB[2] = 0                                   # Phi[Rad]
StatesB[3] = 28460                               # Thrust[N]
StatesB[4] = 250                                 # Velocity[m/s]
StatesB[5] = 0                                   # Flight Path Angle(gamma)[Rad]
StatesB[6] = 0                                   # Psi[Rad]
StatesB[7] = 37.46 * Aircraft_parm.D2R              # Latitude[Rad]
StatesB[8] = 126.22 * Aircraft_parm.D2R             # Longitude[Rad]
StatesB[9] = 3000                                # Altitude[m]
StatesB[10] = 5240.6                             # Mass[Kg]

class  combatgeometry(StatesA):
    def _focus_angle(self, agent_id, opp_id, norm=False):
            """
        Compute ATA angle based on vector angles of current heading direction and position of the two airplanes. 
        """
        x = np.clip((np.dot(np.array([cos( ((90-self.sim.get_unit.heading)%360)*(pi/180) ),sin( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) )]), np.array([self.sim.get_unit(opp_id).position.lon-self.sim.get_unit(agent_id).position.lon, self.sim.get_unit(opp_id).position.lat-self.sim.get_unit(agent_id).position.lat])))/(np.linalg.norm(np.array([cos( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) ),sin( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) )]))*np.linalg.norm(np.array([self.sim.get_unit(opp_id).position.lon-self.sim.get_unit(agent_id).position.lon, self.sim.get_unit(opp_id).position.lat-self.sim.get_unit(agent_id).position.lat]))+1e-10), -1, 1)
        if norm:
            return np.clip( (acos(x) * (180 / pi))/180, 0, 1)
        else:
            return acos(x) * (180 / pi)

    def _distance(self, agent_id, opp_id, norm=False):
        """
        Euclidian Distance between two aircrafts.
        """
        d = hypot(self.sim.get_unit(opp_id).position.lon - self.sim.get_unit(agent_id).position.lon, self.sim.get_unit(opp_id).position.lat - self.sim.get_unit(agent_id).position.lat)
        return self._shifted_range(d, 0, sqrt(2*self.map_size**2), 0, 1) if norm else d

    def _aspect_angle(self, agent_id, opp_id, norm=True):
        """
        Aspect angle: angle from agent_id tail to opp_id, regardless of heading of opp_id.
        """
        focus = self._focus_angle(agent_id, opp_id)
        return np.clip((180 - focus)/180,0,1) if norm else np.clip(180-focus,0,180)

    def _heading_diff(self, agent_id, opp_id, norm=True):
        """
        Angle between heading vectors.
        """
        x = np.clip((np.dot(np.array([cos( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) ), sin( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) )]), np.array([cos( ((90-self.sim.get_unit(opp_id).heading)%360)*(pi/180) ), sin( ((90-self.sim.get_unit(opp_id).heading)%360)*(pi/180) )])))/(np.linalg.norm(np.array([cos( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) ),sin( ((90-self.sim.get_unit(agent_id).heading)%360)*(pi/180) )]))*np.linalg.norm(np.array([cos( ((90-self.sim.get_unit(opp_id).heading)%360)*(pi/180) ), sin( ((90-self.sim.get_unit(opp_id).heading)%360)*(pi/180) )]))+1e-10), -1, 1)
        if norm:
            return np.clip( (acos(x) * (180 / pi))/180, 0, 1)
        else:
            return acos(x) * (180 / pi)
