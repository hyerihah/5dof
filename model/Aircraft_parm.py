import math
import numpy as np

class Aircraft_parm(object):
    zeta = 1
    wn = 16
    tau_Roll = 0.1
    tau_T = 0.1
    Re = 6378137
    Ecc_Earth = 0.0818191908426
    Const_Earth = 3986005e8
    Mass_Earth = 5.9733328e24
    Sref = 27.8709
    D2R = math.pi/180
    R2D = 180/math.pi
    

    AOA = [-0.3491, -0.1745, -0.0873, 0, 0.0873, 0.1745, 0.2618, 0.3491, 0.4363, 0.5236, 0.6109, 0.6981, 0.7854, 0.8727, 0.9599, 1.0472, 1.1345, 1.2217, 1.3090, 1.3963, 1.5708] #rad
    CL = [-0.9788, -0.7100, -0.3332, -0.0466, 0.4398, 0.8099, 1.1502, 1.4266, 1.5986, 1.8571, 1.9560, 1.9660, 1.8752, 1.5349, 1.4809, 1.2453, 1.0655, 0.8021, 0.5876, 0.2221, 0]
    CD = [0.2610, 0.1921, 0.1217, 0.0623, 0.0755, 0.1051, 0.1622, 0.3204, 0.4932, 0.7197, 0.9802, 1.1155, 1.2484, 1.3821, 1.4412, 1.5016, 1.6630, 1.7635, 1.8083, 1.8179, 1.9671]

    Gain_Vel_P = 6.2406e4
    Gain_Vel_I = 1000 * 0.8

    Err_Vel_I = 0
    Err_Alt_I = 0

    Gain_Alt_P = 4.9224e-4
    Gain_Alt_I = 4.9224e-5
    Gain_Alt_D = 3 * 4.9224e-4

    Trim_Alpha = 1.131 * D2R

    Gain_Psi_P = 1.3
    Err_Psi = 0

    Limit_Thro = [1, 0]
    Limit_dot_Alt = [14, -12]
    Limit_Theta = [50 * D2R, -50 * D2R]
    Limit_Alpha = [35 * D2R, -35 * D2R]
    Limit_dPsi = [5.83 * D2R, -5.83 * D2R]
    Limit_Dist_I = [1000, -1000]
    Limit_Phi = [65 * D2R, -65 * D2R]
    Limit_Gilde_Dist_I = [100, -100]
    Limit_Vel_Cmd = [640, 100]

    Factor_Thro = 76.3 * 10**3
    Const_Thro = [0.85, 0.16666667, 1390]
    Const_Alpha = [0.00012875, -0.05845, 7.915625]

    Flit_Vel_Cmd = 0
    Flit_Alt_Cmd = 0
    Flit_Psi_Cmd = 0





