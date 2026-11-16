#!/usr/bin/env python3
"""Replay absolute camera poses through Colosseum/AirSim RPC in CV mode.

No renderer IMU/flight physics is used. This adapter needs live UE validation.
"""
import argparse
import csv
import io
import json
from pathlib import Path
import socket
import time

import msgpackrpc
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation
import yaml
from frames import matrix_wxyz, quat_wxyz


def rpc_pose(position, rotation):
    q = quat_wxyz(rotation)
    return {'position':dict(zip(['x_val','y_val','z_val'],map(float,position))),
            'orientation':dict(zip(['w_val','x_val','y_val','z_val'],map(float,q)))}


def pose_error(actual, position, rotation):
    p = np.array([actual['position'][k] for k in ['x_val','y_val','z_val']])
    q = np.array([actual['orientation'][k] for k in ['w_val','x_val','y_val','z_val']])
    return float(np.linalg.norm(p-position)), float(Rotation.from_matrix(rotation.T @ matrix_wxyz(q)).magnitude())


def render(args):
    # Fail before creating markers if no renderer is listening.
    with socket.create_connection((args.host,args.port),timeout=3):
        pass
    dataset = args.dataset.resolve()
    manifest = yaml.safe_load((dataset/'manifest.yaml').read_text())
    if (dataset/'.flight_incomplete').exists():
        raise RuntimeError('Incomplete JSBSim flight')
    destination = dataset/'mav0/cam0/data'
    if any(destination.iterdir()):
        raise RuntimeError('Image output is not empty; preserve previous run and use a fresh dataset')
    poses = np.genfromtxt(dataset/'camera_poses.csv',delimiter=',',names=True)
    camera = manifest['config']['camera']
    align = Rotation.from_euler('z',args.world_yaw_deg,degrees=True).as_matrix()
    offset = np.asarray(args.origin_in_airsim)
    # Camera optical -> FRD, which is the orientation used by AirSim pose RPCs.
    optical_to_frd = np.array([[0,0,1],[1,0,0],[0,1,0]])
    client = msgpackrpc.Client(msgpackrpc.Address(args.host,args.port),timeout=30,
                               pack_encoding='utf-8',unpack_encoding='utf-8')
    if not client.call('ping'):
        raise RuntimeError('RPC ping failed')
    remote_settings = json.loads(client.call('getSettingsString'))
    if remote_settings.get('SimMode') != 'ComputerVision':
        raise RuntimeError('Launch the renderer with config/airsim_render_only.json (ComputerVision mode)')
    saved_pause = client.call('simIsPaused')
    original_pose = client.call('simGetVehiclePose',args.vehicle)
    incomplete = dataset/'.render_incomplete'
    incomplete.touch()
    completed = False
    mapping = []
    try:
        client.call('simPause', True)
        client.call('simSetCameraPose',args.camera,rpc_pose([0,0,0],np.eye(3)),args.vehicle,False)
        client.call('simSetCameraFov',args.camera,float(camera['horizontal_fov_deg']),args.vehicle,False)
        request = [{'camera_name':args.camera,'image_type':0,'pixels_as_float':False,'compress':True}]
        calibration = yaml.safe_load((dataset/'calibration.yaml').read_text())
        for index,row in enumerate(np.atleast_1d(poses)):
            position = align @ np.array([row[k] for k in ['px','py','pz']]) + offset
            orientation = align @ matrix_wxyz([row[k] for k in ['qw','qx','qy','qz']]) @ optical_to_frd.T
            client.call('simSetVehiclePose',rpc_pose(position,orientation),True,args.vehicle)
            # Wall wait holds pose fixed; does not advance the dataset timestamp.
            # Tile completion needs separate UE/Cesium validation, not just sleep.
            time.sleep(args.settle_wall_s)
            response = None
            for _ in range(args.warmup_captures+1):
                response = client.call('simGetImages',request,args.vehicle,False)[0]
            error_m,error_rad = pose_error({'position':response['camera_position'],
                                           'orientation':response['camera_orientation']},position,orientation)
            if error_m > .02 or error_rad > np.deg2rad(.1):
                raise RuntimeError(f'Camera pose mismatch at {index}: {error_m} m, {error_rad} rad')
            raw = response['image_data_uint8']
            if isinstance(raw,str):
                raw = raw.encode('latin1')
            image = Image.open(io.BytesIO(bytes(raw)))
            if image.size != (camera['width'],camera['height']):
                raise RuntimeError(f'Image dimensions mismatch: {image.size}')
            gray = np.asarray(image.convert('L'))
            if gray.std()<1 or np.percentile(gray,95)<5:
                raise RuntimeError(f'Blank/dark image at {index}; check terrain and lighting')
            info = client.call('simGetCameraInfo',args.camera,args.vehicle,False)
            if not np.isclose(info['fov'],camera['horizontal_fov_deg'],atol=.01):
                raise RuntimeError('Runtime FOV mismatch')
            if index == 0:
                (dataset/'runtime_camera_info.json').write_text(json.dumps(info,indent=2))
                (dataset/'settings.renderer.snapshot.json').write_text(json.dumps(remote_settings,indent=2))
                calibration['intrinsics_status'] = 'runtime FOV and resolution checked; centered pinhole, zero distortion; reprojection validation pending'
            timestamp = int(row['t_ns'])
            (destination/f'{timestamp}.png').write_bytes(bytes(raw))
            mapping.append([timestamp,response['time_stamp'],time.time_ns(),error_m,error_rad,
                            float(gray.mean()),float(gray.std())])
            if index % 50 == 0:
                print(f'Rendered {index+1}/{len(poses)}',flush=True)
        (dataset/'calibration.yaml').write_text(yaml.safe_dump(calibration,sort_keys=False))
        manifest['status'] = 'images_rendered_visual_checks_pending'
        manifest['renderer'] = {'adapter':'AirSim RPC ComputerVision','origin_in_airsim_m':offset.tolist(),
                                'world_yaw_deg':args.world_yaw_deg,'settle_wall_s':args.settle_wall_s,
                                'warmup_captures':args.warmup_captures,
                                'tile_completion_verified':False,'reprojection_verified':False}
        (dataset/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
        completed = True
    finally:
        with (dataset/'render_mapping.csv').open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t_ns','renderer_time_ns','wall_time_ns','pose_error_m','pose_error_rad','mean_gray','std_gray'])
            writer.writerows(mapping)
        # Restore the invisible CV pawn; never arm, take off or send flight commands.
        try:
            client.call('simSetVehiclePose',original_pose,True,args.vehicle)
        finally:
            client.call('simPause',saved_pause)
            client.close()
        if completed:
            incomplete.unlink()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset',type=Path)
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=41451)
    parser.add_argument('--vehicle',default='Drone1')
    parser.add_argument('--camera',default='cam0')
    parser.add_argument('--origin-in-airsim',type=float,nargs=3,required=True,
                        metavar=('N','E','D'),help='AirSim NED position of the first exported IMU CG; measure this in the UE level')
    parser.add_argument('--world-yaw-deg',type=float,default=0)
    parser.add_argument('--settle-wall-s',type=float,default=.25)
    parser.add_argument('--warmup-captures',type=int,default=3)
    args = parser.parse_args()
    render(args)
