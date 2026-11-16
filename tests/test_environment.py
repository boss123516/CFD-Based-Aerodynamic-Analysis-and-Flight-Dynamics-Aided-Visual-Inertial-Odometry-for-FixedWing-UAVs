import contextlib
import hashlib
import io
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from environment import apply_imu_noise,gust_ned
from generate_flight import generate


class EnvironmentTests(unittest.TestCase):
    def test_gust_smooth_start_and_end(self):
        wind={'gusts':[dict(start_s=2,rise_s=1,hold_s=1,fall_s=1,ned_mps=[0,2,0])]}
        np.testing.assert_allclose(gust_ned(1,wind),0)
        np.testing.assert_allclose(gust_ned(2,wind),0)
        np.testing.assert_allclose(gust_ned(2.5,wind),[0,1,0])
        np.testing.assert_allclose(gust_ned(3.5,wind),[0,2,0])
        np.testing.assert_allclose(gust_ned(5,wind),0)
        self.assertLess(np.linalg.norm(gust_ned(2+1e-6,wind)),1e-10)

    def test_noise_bias_and_seed(self):
        noise={'gyro_bias_rad_s':[.01,0,0],'accel_bias_m_s2':[0,.02,0],
               'gyro_white_std_rad_s':.001,'accel_white_std_m_s2':.01,
               'gyro_bias_random_walk_rad_s_sqrt_s':.0001,'accel_bias_random_walk_m_s2_sqrt_s':.001}
        zeros=np.zeros((1000,3))
        a=apply_imu_noise(zeros,zeros,noise,2027,.01);b=apply_imu_noise(zeros,zeros,noise,2027,.01)
        for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
        np.testing.assert_array_equal(a[2][0],noise['gyro_bias_rad_s'])
        self.assertGreater(np.std(a[0]-a[2]),.0008)
        self.assertGreater(np.std(a[2][:,0]),0)

    def test_jsbsim_wind_turbulence_reproducible(self):
        config=yaml.safe_load((ROOT/'config/strasbourg.yaml').read_text())
        config.update(duration_s=1,settle_s=1)
        with tempfile.TemporaryDirectory() as temp,contextlib.redirect_stdout(io.StringIO()):
            a=Path(temp)/'a';b=Path(temp)/'b';generate(config,a);generate(config,b)
            for name in ['flight_debug.csv','imu.csv','imu_bias.csv']:
                self.assertEqual(hashlib.sha256((a/name).read_bytes()).digest(),hashlib.sha256((b/name).read_bytes()).digest())
            flight=np.genfromtxt(a/'flight_debug.csv',delimiter=',',names=True)
            self.assertGreater(np.std(flight['turb_n_mps']),0)
            self.assertGreater(np.mean(flight['mean_wind_n_mps']),1.9)
