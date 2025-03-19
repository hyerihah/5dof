from model.Aircraft_parm import Aircraft_parm
import numpy as np
import math

def REarth(Lat):
    r_earth = np.array([0, 0],dtype=np.float64)
    r_earth[0] = Aircraft_parm.Re*(1-Aircraft_parm.Ecc_Earth*Aircraft_parm.Ecc_Earth)/(1-Aircraft_parm.Ecc_Earth*Aircraft_parm.Ecc_Earth*math.sin(Lat)*math.sin(Lat))**1.5
    r_earth[1] = Aircraft_parm.Re/(1-Aircraft_parm.Ecc_Earth*Aircraft_parm.Ecc_Earth * math.sin(Lat)*math.sin(Lat))**0.5

    return r_earth
