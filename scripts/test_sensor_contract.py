#!/usr/bin/env python3
"""Analytic cases independent of JSBSim and the exported flight."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from frames import camera_transform, kinematic_imu


class SensorContract(unittest.TestCase):
    def test_stationary_specific_force(self):
        rotation = np.tile(Rotation.from_euler('ZYX',[.2,.1,-.3]).as_matrix(),(401,1,1))
        gyro,force = kinematic_imu(np.zeros((401,3)),rotation,.005,9.80665)
        np.testing.assert_allclose(gyro,0,atol=1e-12)
        np.testing.assert_allclose(force, np.tile(rotation[0].T @ [0,0,-9.80665],(401,1)),atol=1e-12)

    def test_constant_acceleration(self):
        t = np.arange(401)*.005
        rotation = np.tile(np.eye(3),(401,1,1))
        _,force = kinematic_imu(t[:,None]*[2.,-1.,.5],rotation,.005,9.80665)
        np.testing.assert_allclose(force,np.tile([2.,-1.,.5-9.80665],(401,1)),atol=1e-10)

    def test_positive_yaw(self):
        t = np.arange(401)*.005
        rotation = Rotation.from_euler('z',.2*t).as_matrix()
        gyro,_ = kinematic_imu(np.zeros((401,3)),rotation,.005,9.80665)
        np.testing.assert_allclose(gyro,np.tile([0,0,.2],(401,1)),atol=1e-7)

    def test_camera_forward_down(self):
        T = camera_transform({'mount_pitch_deg':-45,'translation_body_m':[0,0,.45]})
        np.testing.assert_allclose(T[:3,:3] @ [0,0,1],[2**-.5,0,2**-.5],atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1)


if __name__ == '__main__':unittest.main()
