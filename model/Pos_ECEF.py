from Aircraft_parm import Aircraft_parm
import numpy as np
import math
def PosECEF(Lat, Lng, Alt, R_Earth):
    ECEF = np.array([0, 0, 0],dtype=np.float64)
    ECEF[0] = (R_Earth[0] + Alt) * math.cos(Lat) * math.cos(Lng)
    ECEF[1] = (R_Earth[0] + Alt) * math.cos(Lat) * math.sin(Lng)
    ECEF[2] = ((1-Aircraft_parm.Ecc_Earth**2) * R_Earth[0] + Alt) * math.sin(Lat)

    return ECEF