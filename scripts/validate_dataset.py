#!/usr/bin/env python3
"""Check the sensor contract and independent physical consistency invariants."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
from frames import matrix_wxyz


def read(path):
    return np.atleast_2d(np.loadtxt(path,delimiter=',',skiprows=1))


def validate(dataset, require_images=False):
    manifest = yaml.safe_load((dataset/'manifest.yaml').read_text())
    config = manifest['config']
    imu, gt, cam, physics = [read(dataset/name) for name in
                            ['imu.csv','groundtruth.csv','camera_poses.csv','physics_groundtruth.csv']]
    checks, metrics = {}, {}
    checks['no_incomplete_flight'] = not (dataset/'.flight_incomplete').exists()
    for name,data,rate in [('imu',imu,config['imu_hz']),('gt',gt,config['imu_hz']),
                           ('camera',cam,config['camera_hz']),('physics',physics,config['physics_hz'])]:
        checks[name+'_finite'] = bool(np.isfinite(data).all())
        checks[name+'_exact_grid'] = bool(data[0,0]==0 and np.all(np.diff(data[:,0])==1_000_000_000//rate)
                                             and data[-1,0]==round(config['duration_s']*1e9))
    checks['imu_gt_same_times'] = np.array_equal(imu[:,0],gt[:,0])
    checks['imu_covers_camera'] = bool(imu[0,0]<=cam[0,0] and imu[-1,0]>=cam[-1,0])
    for name,data in [('gt',gt),('camera',cam),('physics',physics)]:
        checks[name+'_unit_quaternion'] = bool(np.allclose(np.linalg.norm(data[:,4:8],axis=1),1,atol=1e-10))
    rotation = matrix_wxyz(gt[:,4:8])
    calibration = yaml.safe_load((dataset/'calibration.yaml').read_text())
    extrinsic = np.asarray(calibration['T_IC'])
    indices = np.searchsorted(gt[:,0],cam[:,0])
    expected_position = gt[indices,1:4]+np.einsum('nij,j->ni',rotation[indices],extrinsic[:3,3])
    metrics['camera_position_error_max_m'] = float(np.max(np.linalg.norm(expected_position-cam[:,1:4],axis=1)))
    camera_rotation = matrix_wxyz(cam[:,4:8])
    expected_rotation = rotation[indices] @ extrinsic[:3,:3]
    metrics['camera_rotation_error_max_rad'] = float(Rotation.from_matrix(np.swapaxes(expected_rotation,1,2) @ camera_rotation).magnitude().max())
    checks['camera_extrinsic_position'] = metrics['camera_position_error_max_m']<1e-8
    checks['camera_extrinsic_rotation'] = metrics['camera_rotation_error_max_rad']<1e-8
    dt = 1/config['imu_hz']
    biases = read(dataset/'imu_bias.csv')[:,1:] if (dataset/'imu_bias.csv').exists() else np.tile(
        [*config['noise']['gyro_bias_rad_s'],*config['noise']['accel_bias_m_s2']],(len(imu),1))
    force = imu[:,4:7]-biases[:,3:]
    gyro = imu[:,1:4]-biases[:,:3]
    acceleration = np.einsum('nij,nj->ni',rotation,force)+[0,0,config['gravity_mps2']]
    dv = np.diff(gt[:,8:11],axis=0)-dt*(acceleration[1:]+acceleration[:-1])/2
    gyro_gt = (Rotation.from_matrix(rotation[:-1]).inv()*Rotation.from_matrix(rotation[1:])).as_rotvec()/dt
    dw = gyro_gt-(gyro[:-1]+gyro[1:])/2
    metrics['velocity_increment_error_max_mps'] = float(np.linalg.norm(dv,axis=1).max())
    metrics['angular_rate_error_max_rad_s'] = float(np.linalg.norm(dw,axis=1).max())
    if config['noise']['accel_white_std_m_s2']==0:
        checks['specific_force_gravity_sign_and_time'] = metrics['velocity_increment_error_max_mps']<.001
    if config['noise']['gyro_white_std_rad_s']==0:
        checks['gyro_frame_and_time'] = metrics['angular_rate_error_max_rad_s']<.001
    physics_dt = 1/config['physics_hz']
    pv = np.gradient(physics[:,1:4],physics_dt,axis=0,edge_order=2)[2:-2]-physics[2:-2,8:11]
    metrics['position_velocity_error_max_mps'] = float(np.linalg.norm(pv,axis=1).max())
    checks['position_velocity_agree'] = metrics['position_velocity_error_max_mps']<.01
    checks['source_csv_hashes'] = all(hashlib.sha256((dataset/name).read_bytes()).hexdigest()==digest
                                        for name,digest in manifest['sha256'].items())
    expected_count = len(cam)
    images = list((dataset/'mav0/cam0/data').glob('*.png'))
    images_present = len(images)==expected_count
    if require_images:
        from PIL import Image
        checks['all_images_present'] = images_present
        checks['no_incomplete_render'] = not (dataset/'.render_incomplete').exists()
        checks['image_names_match_times'] = {p.stem for p in images}=={str(int(t)) for t in cam[:,0]}
        checks['image_dimensions'] = all(Image.open(p).size==(config['camera']['width'],config['camera']['height']) for p in images)
    renderer = manifest.get('renderer',{})
    visual_ready = (require_images and all(checks.values()) and images_present and
                    all(renderer.get(key,False) for key in ['tile_completion_verified','reprojection_verified','camera_pose_verified']))
    remaining = ['MSCKF baseline replay and initialization','CFD plant/model validation and A/B/C evaluation']
    if not visual_ready:
        remaining[:0] = ['UE/Cesium rendering and tile readiness','intrinsics/reprojection and rotation visual checks']
    report = {'flight_contract_pass':all(checks.values()),'checks':checks,'metrics':metrics,
              'image_count':len(images),'expected_images':expected_count,
              'visual_dataset_ready':bool(visual_ready),'remaining':remaining}
    (dataset/'validation_report.json').write_text(json.dumps(report,indent=2))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    parser.add_argument('--require-images',action='store_true')
    args = parser.parse_args()
    report = validate(args.dataset,args.require_images)
    print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['flight_contract_pass'] else 1)
