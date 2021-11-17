import numpy as np
from get_avail_car_index import TIME_INTERVAL

def vehicle_dynamic_model(beta_0, vel, phi, r_0, PHI_0):
    m = 1500
    Jz = 2500
    a = 1.1
    b = 1.6
    C1 = 55000
    C2 = 60000
    delta_beta = 1 / m / vel * (
                -(C1 + C2) * beta_0 - (m * vel + 1 / vel * (a * C1 - b * C2)) * r_0 + C1 * phi) * TIME_INTERVAL / 1000
    delta_r = 1 / Jz * ((-a * C1 + b * C2) * beta_0 + 1 / vel * (
                -a ** 2 * C1 - b ** 2 * C2) * r_0 + a * C1 * phi) * TIME_INTERVAL / 1000
    beta_a = beta_0 + delta_beta
    r_a = r_0 + delta_r
    PHI_a = PHI_0 + r_0 * TIME_INTERVAL / 1000  # check this
    delta_X = (np.cos(PHI_0) * vel * np.cos(beta_0) - np.sin(PHI_0) * vel * np.sin(beta_0)) * TIME_INTERVAL / 1000
    delta_y = (np.sin(PHI_0) * vel * np.cos(beta_0) + np.cos(PHI_0) * vel * np.sin(beta_0)) * TIME_INTERVAL / 1000
    return delta_X, delta_y, r_a, beta_a, PHI_a