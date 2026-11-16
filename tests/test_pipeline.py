import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from cfd.model import BetaLUT
from dynamics.constraint import linearize
from evaluation.metrics import evaluate


class DynamicsTests(unittest.TestCase):
    def test_jacobian_matches_finite_difference(self):
        rotation = Rotation.from_euler('ZYX',[.7,.1,.3]).as_matrix()
        velocity = rotation @ [40.,2.,3.]
        wind = np.array([1.,-.2,0.])
        residual,H = linearize(velocity,rotation,wind_world=wind)
        eps = 1e-6
        numerical = np.zeros(9)
        for j in range(9):
            delta = np.zeros(9);delta[j] = eps
            plus = linearize(velocity+delta[3:6],rotation @ Rotation.from_rotvec(delta[6:9]).as_matrix(),wind_world=wind)[0]
            minus = linearize(velocity-delta[3:6],rotation @ Rotation.from_rotvec(-delta[6:9]).as_matrix(),wind_world=wind)[0]
            numerical[j] = -(plus-minus)/(2*eps)
        np.testing.assert_allclose(H[0],numerical,atol=1e-8)

    def test_update_reduces_sideslip_and_skips_low_speed(self):
        velocity = np.array([40.,5.,0.])
        residual,H = linearize(velocity,np.eye(3))
        P = np.diag([0.]*3+[4.]*3+[0.]*3)
        K = P @ H.T / ((H @ P @ H.T).item()+.01)
        updated = velocity+(K[:,0]*residual)[3:6]
        self.assertLess(abs(linearize(updated,np.eye(3))[0]),abs(residual))
        self.assertIsNone(linearize(np.zeros(3),np.eye(3)))

    def test_cfd_provenance_and_domain(self):
        with self.assertRaises(ValueError):BetaLUT(ROOT/'cfd/demo_lut')
        model = BetaLUT(ROOT/'cfd/demo_lut',allow_demo=True)
        self.assertIsNone(model.query(1.,47.))
        self.assertGreaterEqual(model.query(0.,47.)[1],.02)


class EvaluationTests(unittest.TestCase):
    def make_gt(self):
        t = np.arange(41)*100_000_000
        p = np.column_stack((t/1e9*10,np.zeros(41),np.zeros(41)))
        q = np.tile([1.,0.,0.,0.],(41,1))
        return np.column_stack((t,p,q))

    def test_initial_yaw_alignment_no_scale_fit(self):
        gt = self.make_gt();estimate = gt.copy()
        r = Rotation.from_euler('z',.5)
        estimate[:,1:4] = r.inv().apply(gt[:,1:4]-[20,30,2])
        estimate[:,4:8] = np.roll(np.tile(r.inv().as_quat(),(len(gt),1)),1,axis=1)
        report,_ = evaluate(gt,estimate,gt[:,0])
        self.assertLess(report['ate_rmse_m'],1e-10)
        self.assertLess(report['rpe_rotation_rmse_deg'],1e-10)
        estimate[:,1:4]*=2
        report,_ = evaluate(gt,estimate,gt[:,0])
        self.assertGreater(report['ate_rmse_m'],10)

    def test_outages_reduce_coverage(self):
        gt = self.make_gt()
        report,_ = evaluate(gt,np.concatenate((gt[:10],gt[21:])),gt[:,0])
        self.assertEqual(report['valid_poses'],30)
        self.assertLess(report['valid_output_rate'],1)
        self.assertLess(report['rpe_pairs'],31)


class EndToEndTests(unittest.TestCase):
    def test_full_demo_shared_input_without_gt(self):
        with tempfile.TemporaryDirectory(prefix='kroc-pipeline-test-') as temp:
            directory = Path(temp)
            config = yaml.safe_load((ROOT/'config/straight_turn.yaml').read_text())
            config.update(duration_s=2,settle_s=1)
            flight = directory/'flight.yaml';flight.write_text(yaml.safe_dump(config))
            output = directory/'run'
            process = subprocess.run([sys.executable,str(ROOT/'scripts/pipeline.py'),'demo',
                '--flight-config',str(flight),'--output',str(output)],capture_output=True,text=True,timeout=120)
            self.assertEqual(process.returncode,0,process.stdout+process.stderr)
            report = json.loads((output/'comparison.json').read_text())
            self.assertFalse(report['scientific_result'])
            self.assertEqual(set(report['methods']),{'A','B','C'})
            for name in ['A','B','C']:
                self.assertEqual(report['methods'][name]['status'],'complete')
                self.assertEqual(report['methods'][name]['valid_output_rate'],1.)
            self.assertFalse((output/'sensor_input/groundtruth.csv').exists())
            self.assertFalse((output/'sensor_input/mav0/state_groundtruth_estimate0').exists())
            self.assertEqual(len(list((output/'sensor_input/mav0/cam0/data').glob('*.png'))),21)
            self.assertFalse((output/'.pipeline_incomplete').exists())
            # A has no model updates, B/C use the shared residual implementation.
            self.assertEqual(len((output/'A/dynamics.csv').read_text().splitlines()),1)
            self.assertGreater(len((output/'B/dynamics.csv').read_text().splitlines()),1)


if __name__=='__main__':unittest.main()
