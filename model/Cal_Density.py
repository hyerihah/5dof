import numpy as np

def CalDensity(States):
    Poly_Density = [2.6536e-9, -0.00010723, 1.2123]
    if States < 0:
        Alt = 0
    elif States > 15000:
        Alt = 15000
    else:
        Alt = States

    rho = np.polyval(Poly_Density, Alt)

    return rho
