"""Sideslip residual for a right-multiplicative attitude error.

Error order [delta_position_world, delta_velocity_world, delta_theta_body].
MSCKF adapters must map this Jacobian into their own state/error convention.
"""
import numpy as np


def skew(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])


def linearize(velocity_world, rotation_world_body, beta_ref=0., wind_world=None, min_speed=5.):
    wind = np.zeros(3) if wind_world is None else np.asarray(wind_world)
    u = rotation_world_body.T @ (velocity_world-wind)
    x,y,z = u
    rho = np.hypot(x,z)
    speed2 = float(u @ u)
    if speed2 < min_speed**2 or rho < 1e-6:
        return None
    prediction = np.arctan2(y,rho)
    gradient = np.array([-y*x/(speed2*rho),rho/speed2,-y*z/(speed2*rho)])
    H = np.zeros((1,9))
    H[0,3:6] = gradient @ rotation_world_body.T
    H[0,6:9] = gradient @ skew(u)
    residual = np.arctan2(np.sin(beta_ref-prediction),np.cos(beta_ref-prediction))
    return float(residual),H
