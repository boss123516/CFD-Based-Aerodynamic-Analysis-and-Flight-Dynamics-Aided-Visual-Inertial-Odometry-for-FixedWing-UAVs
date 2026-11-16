"""Closed horizontal paths with arc-length phase progression and smooth references."""
import numpy as np


class PathFollower:
    def __init__(self, config, anchor):
        self.config = config
        self.anchor = np.asarray(anchor)
        self.phase = 0.
        self.bank = 0.

    def curve(self, phase):
        p = self.config
        kind = p['kind']
        a, b = p['north_radius_m'], p['east_radius_m']
        if kind in ['circle','ellipse']:
            return (np.array([a*np.sin(phase), b*(1-np.cos(phase))]),
                    np.array([a*np.cos(phase), b*np.sin(phase)]),
                    np.array([-a*np.sin(phase), b*np.cos(phase)]))
        if kind == 'figure_eight':
            return (np.array([a*np.sin(phase), b*np.sin(2*phase)]),
                    np.array([a*np.cos(phase), 2*b*np.cos(2*phase)]),
                    np.array([-a*np.sin(phase), -4*b*np.sin(2*phase)]))
        raise ValueError('Unknown closed trajectory kind')

    def command(self, t, position, velocity, airspeed, dt):
        cfg = self.config
        target, tangent, second = self.curve(self.phase)
        speed = cfg['speed_mps'] + cfg.get('speed_amplitude_mps',0)*np.sin(2*np.pi*t/cfg.get('speed_period_s',100))
        target += self.anchor
        norm = np.linalg.norm(tangent)
        unit = tangent/norm
        desired_ground = speed*unit + cfg.get('position_gain_s_inv',.09)*(target-position)
        actual_speed = max(np.linalg.norm(velocity),1)
        desired_course = np.arctan2(desired_ground[1],desired_ground[0])
        course = np.arctan2(velocity[1],velocity[0])
        course_error = np.arctan2(np.sin(desired_course-course),np.cos(desired_course-course))
        curvature = (tangent[0]*second[1]-tangent[1]*second[0])/norm**3
        bank = np.arctan((actual_speed**2*curvature + cfg.get('course_gain_s_inv',.7)*actual_speed*course_error)/9.80665)
        bank = np.clip(bank,-np.deg2rad(cfg['bank_limit_deg']),np.deg2rad(cfg['bank_limit_deg']))
        limit=np.deg2rad(cfg.get('bank_slew_deg_s',8))*dt
        bank=float(self.bank+np.clip(bank-self.bank,-limit,limit))
        self.bank=bank
        altitude = cfg['altitude_h_sl_m']+cfg.get('altitude_amplitude_m',0)*np.sin(2*np.pi*t/cfg.get('altitude_period_s',120))
        self.phase += max(1,float(np.dot(velocity,unit)))*dt/norm
        return bank, altitude, speed, float(np.linalg.norm(target-position))
