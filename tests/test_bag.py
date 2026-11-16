"""Serialization, frame transforms and clock-offset contracts, with small test images."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))


@unittest.skipUnless(importlib.util.find_spec('rosbags'),'optional requirements-bag.lock.txt not installed')
class BagTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from generate_flight import generate
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.dataset=cls.root/'flight'
        cfg=yaml.safe_load((ROOT/'config/straight_turn.yaml').read_text())
        cfg.update(duration_s=1,settle_s=1);cfg['camera'].update(width=32,height=24)
        with contextlib.redirect_stdout(io.StringIO()):generate(cfg,cls.dataset)
        for t in range(0,1_000_000_001,100_000_000):
            Image.fromarray(np.arange(32*24,dtype=np.uint8).reshape(24,32)).save(cls.dataset/'mav0/cam0/data'/f'{t}.png')

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def exercise(self,version,mode):
        from export_bag import export,FRD_FLU
        from rosbags.highlevel import AnyReader
        # This fixture tests serialization only; it is explicitly not real-map acquisition.
        target=self.root/f'{version}_{mode}.bag' if version==1 else self.root/f'{version}_{mode}'
        with patch('export_bag.validate',return_value={'visual_dataset_ready':True}):
            report=export(self.dataset,target,version,camera_clock_offset_ns=12_000_000,
                imu_clock_offset_ns=-3_000_000,stamp_mode=mode,camera_latency_ns=25_000_000)
        self.assertTrue(report['readback_pass'])
        self.assertEqual(report['maximum_corrected_grid_alignment_error_ns'],0)
        self.assertEqual(report['timing']['timeshift_cam_imu_s'],-.015 if mode=='raw' else 0)
        self.assertEqual(set(report['counts']),{'/imu0','/cam0/image_raw','/cam0/camera_info'})
        imu0=np.loadtxt(self.dataset/'imu.csv',delimiter=',',skiprows=1)[0]
        seen=set()
        with AnyReader([target]) as reader:
            for connection,receipt,raw in reader.messages():
                if connection.topic in seen:continue
                seen.add(connection.topic);msg=reader.deserialize(raw,connection.msgtype)
                ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
                if connection.topic=='/imu0':
                    self.assertEqual(ns,997_000_000 if mode=='raw' else 1_000_000_000)
                    self.assertEqual(msg.orientation_covariance[0],-1)
                    np.testing.assert_allclose([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z],FRD_FLU@imu0[1:4])
                    np.testing.assert_allclose([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],FRD_FLU@imu0[4:7])
                else:
                    self.assertEqual(ns,1_012_000_000 if mode=='raw' else 1_000_000_000)
                    self.assertEqual(receipt,1_025_000_000)
                if connection.topic=='/cam0/image_raw':self.assertEqual(msg.encoding,'mono8')
        cam=yaml.safe_load((Path(report['calibration_directory'])/'camchain_ros.yaml').read_text())['cam0']
        T=np.array(cam['T_imu_cam']);np.testing.assert_allclose(T@np.array(cam['T_cam_imu']),np.eye(4),atol=1e-12)
        original=yaml.safe_load((self.dataset/'calibration.yaml').read_text())
        np.testing.assert_allclose(T[:3,:3],FRD_FLU@np.array(original['T_IC'])[:3,:3])

    def test_ros1_corrected(self):self.exercise(1,'corrected')
    def test_ros1_raw(self):self.exercise(1,'raw')
    def test_ros2_corrected(self):self.exercise(2,'corrected')
    def test_ros2_raw(self):self.exercise(2,'raw')

    def test_unverified_renderer_rejected(self):
        from export_bag import export
        with self.assertRaisesRegex(ValueError,'Verified real-map images'):
            export(self.dataset,self.root/'unverified.bag')


if __name__=='__main__':unittest.main()
