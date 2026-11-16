#!/usr/bin/env python3
"""Export verified images + IMU to ROS1/ROS2 and independently read timestamps back.

ROS topics use ENU / FLU, while source CSVs stay NED / FRD. Ground truth is
written as a separate TUM file, never as estimator input in the sensor bag.
Positive clock_offset means header stamp is ahead of physical capture time.
"""
import argparse
import csv
from collections import Counter
import heapq
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from scipy.spatial.transform import Rotation
from rosbags.typesys import Stores,get_typestore
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer as Writer1
from rosbags.rosbag2 import Writer as Writer2
from frames import matrix_wxyz
from validate_dataset import validate

NED_ENU=np.array([[0.,1,0],[1,0,0],[0,0,-1]])
FRD_FLU=np.diag([1.,-1,-1])
TOPICS={'imu':'/imu0','camera':'/cam0/image_raw','info':'/cam0/camera_info'}


def load_rows(path):
    with path.open() as stream:
        reader=csv.reader(stream);next(reader)
        return [(int(row[0]),np.asarray(row[1:],dtype=float)) for row in reader]


def export(dataset, output, version=1, camera_clock_offset_ns=0, imu_clock_offset_ns=0,
           stamp_mode='corrected', camera_latency_ns=0, imu_latency_ns=0, epoch_ns=1_000_000_000):
    dataset=Path(dataset).resolve();output=Path(output).resolve()
    if version not in (1,2) or stamp_mode not in ('raw','corrected'):
        raise ValueError('Invalid bag format / stamp mode')
    if min(camera_latency_ns,imu_latency_ns)<0 or epoch_ns+min(camera_clock_offset_ns,imu_clock_offset_ns)<0:
        raise ValueError('Invalid delivery latency / negative header time')
    validation=validate(dataset,require_images=True)
    if not validation['visual_dataset_ready']:
        raise ValueError('Verified real-map images are required before bag export')
    partial=output.with_name(output.stem+'.incomplete.bag' if version==1 else output.name+'.incomplete')
    if output.exists() or partial.exists():
        raise FileExistsError(output)
    calibration_dir=output.with_name(output.name+'.calibration')
    if calibration_dir.exists():raise FileExistsError(calibration_dir)
    output.parent.mkdir(parents=True,exist_ok=True)
    cfg=yaml.safe_load((dataset/'manifest.yaml').read_text())['config']
    calibration=yaml.safe_load((dataset/'calibration.yaml').read_text())
    imu=load_rows(dataset/'imu.csv')
    with (dataset/'mav0/cam0/data.csv').open() as stream:
        camera=[(int(r[0]),r[1]) for r in list(csv.reader(stream))[1:]]
    for _,filename in camera:
        if Path(filename).name!=filename or not (dataset/'mav0/cam0/data'/filename).is_file():
            raise ValueError('Invalid camera index')
    types=get_typestore(Stores.ROS2_HUMBLE)
    make=lambda name,*args:types.types[name](*args)
    def header(ns,frame):
        sec,nsec=divmod(int(ns),1_000_000_000)
        return make('std_msgs/msg/Header',make('builtin_interfaces/msg/Time',sec,nsec),frame)
    def vector(v):return make('geometry_msgs/msg/Vector3',*map(float,v))
    def stamp(t,offset):return int(t+epoch_ns+(offset if stamp_mode=='raw' else 0))
    width,height=calibration['resolution'];fx,fy,cx,cy=calibration['intrinsics']
    k=np.array([fx,0,cx,0,fy,cy,0,0,1.],dtype=np.float64)
    projection=np.array([fx,0,cx,0,0,fy,cy,0,0,0,1,0.],dtype=np.float64)
    noise=cfg['noise'];zero=np.zeros(9)
    gyro_cov=np.diag([noise['gyro_white_std_rad_s']**2]*3).ravel()
    accel_cov=np.diag([noise['accel_white_std_m_s2']**2]*3).ravel()
    events=heapq.merge(((t+epoch_ns+imu_latency_ns,0,i) for i,(t,_) in enumerate(imu)),
                      ((t+epoch_ns+camera_latency_ns,1,i) for i,(t,_) in enumerate(camera)))
    writer=Writer1(partial) if version==1 else Writer2(partial,version=8)
    if version==1:writer.set_compression(Writer1.CompressionFormat.LZ4)
    serialize=types.serialize_ros1 if version==1 else types.serialize_cdr
    expected=Counter();record_times={topic:[] for topic in TOPICS.values()};header_times={topic:[] for topic in TOPICS.values()}
    with writer:
        connections={key:writer.add_connection(TOPICS[key],msgtype,typestore=types) for key,msgtype in
            [('imu','sensor_msgs/msg/Imu'),('camera','sensor_msgs/msg/Image'),('info','sensor_msgs/msg/CameraInfo')]}
        for receipt,kind,index in events:
            if kind==0:
                t,values=imu[index];ns=stamp(t,imu_clock_offset_ns)
                unavailable=np.array([-1.,0,0,0,0,0,0,0,0])
                msg=make('sensor_msgs/msg/Imu',header(ns,'imu_flu'),
                    make('geometry_msgs/msg/Quaternion',0.,0.,0.,1.),unavailable,
                    vector(FRD_FLU@values[:3]),gyro_cov,vector(FRD_FLU@values[3:]),accel_cov)
                messages=[('imu',msg)]
            else:
                t,filename=camera[index];ns=stamp(t,camera_clock_offset_ns)
                with Image.open(dataset/'mav0/cam0/data'/filename) as image:
                    pixels=np.asarray(image.convert('L'),dtype=np.uint8)
                if pixels.shape!=(height,width):raise ValueError('Unexpected image dimensions')
                h=header(ns,'cam0_optical')
                image_msg=make('sensor_msgs/msg/Image',h,height,width,'mono8',0,width,pixels.ravel())
                roi=make('sensor_msgs/msg/RegionOfInterest',0,0,0,0,False)
                info=make('sensor_msgs/msg/CameraInfo',h,height,width,'plumb_bob',
                    np.array(calibration['distortion_coeffs'],dtype=np.float64),k,np.eye(3).ravel(),projection,0,0,roi)
                messages=[('camera',image_msg),('info',info)]
            for key,msg in messages:
                writer.write(connections[key],int(receipt),serialize(msg,msg.__msgtype__))
                topic=TOPICS[key];expected[topic]+=1
                record_times[topic].append(int(receipt));header_times[topic].append(ns)
    # Reader deserializes every header, checking every record and every grid.
    counts=Counter();last={};readback={topic:[] for topic in TOPICS.values()};receipts={topic:[] for topic in TOPICS.values()}
    with AnyReader([partial],default_typestore=types) as reader:
        for connection,receipt,raw in reader.messages():
            topic=connection.topic;msg=reader.deserialize(raw,connection.msgtype)
            ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if topic in last and ns<=last[topic]:raise ValueError('Nonmonotonic sensor header timestamps')
            last[topic]=ns;counts[topic]+=1;readback[topic].append(ns);receipts[topic].append(receipt)
    if counts!=expected or readback!=header_times or receipts!=record_times:
        raise ValueError('Bag readback differs from source timeline')
    cam_corrected=np.array(readback[TOPICS['camera']],dtype=np.int64)-(camera_clock_offset_ns if stamp_mode=='raw' else 0)
    imu_corrected=np.array(readback[TOPICS['imu']],dtype=np.int64)-(imu_clock_offset_ns if stamp_mode=='raw' else 0)
    if cam_corrected[0]<imu_corrected[0] or cam_corrected[-1]>imu_corrected[-1]:
        raise ValueError('IMU does not bracket corrected camera capture times')
    # Both sensor grids originate at capture time zero. At 200 / 20 Hz these match exactly.
    ids=np.searchsorted(imu_corrected,cam_corrected)
    alignment=int(np.max(np.abs(imu_corrected[ids]-cam_corrected)))
    timing={'epoch_ns':epoch_ns,'mode':stamp_mode,
        'definition':'raw_header_ns = epoch_ns + physical_capture_ns + sensor_clock_offset_ns; bag record = epoch + capture + delivery_latency',
        'camera_clock_offset_ns':camera_clock_offset_ns,'imu_clock_offset_ns':imu_clock_offset_ns,
        'camera_delivery_latency_ns':camera_latency_ns,'imu_delivery_latency_ns':imu_latency_ns,
        'timeshift_cam_imu_s':(imu_clock_offset_ns-camera_clock_offset_ns)/1e9 if stamp_mode=='raw' else 0.,
        'timeshift_definition':'t_imu_clock = t_cam_clock + timeshift_cam_imu_s for the same physical instant',
        'capture_model':'instantaneous global shutter, no rolling shutter / motion blur / exposure integration'}
    calibration_dir.mkdir()
    T=np.array(calibration['T_IC']);T[:3,:3]=FRD_FLU@T[:3,:3];T[:3,3]=FRD_FLU@T[:3,3]
    kalibr={'cam0':{'camera_model':'pinhole','intrinsics':calibration['intrinsics'],'distortion_model':'radtan',
        'distortion_coeffs':calibration['distortion_coeffs'],'resolution':[width,height],
        'T_imu_cam':T.tolist(),'T_cam_imu':np.linalg.inv(T).tolist(),
        'rostopic':TOPICS['camera'],'timeshift_cam_imu':timing['timeshift_cam_imu_s']}}
    (calibration_dir/'camchain_ros.yaml').write_text(yaml.safe_dump(kalibr,sort_keys=False))
    imu_cal={'rostopic':TOPICS['imu'],'update_rate':cfg['imu_hz'],
        'gyroscope_noise_density':float(noise['gyro_white_std_rad_s']/np.sqrt(cfg['imu_hz'])),
        'accelerometer_noise_density':float(noise['accel_white_std_m_s2']/np.sqrt(cfg['imu_hz'])),
        'gyroscope_random_walk':noise.get('gyro_bias_random_walk_rad_s_sqrt_s',0),
        'accelerometer_random_walk':noise.get('accel_bias_random_walk_m_s2_sqrt_s',0)}
    (calibration_dir/'imu_ros.yaml').write_text(yaml.safe_dump(imu_cal,sort_keys=False))
    gt=load_rows(dataset/'groundtruth.csv');bias=load_rows(dataset/'imu_bias.csv')
    first=None
    with (calibration_dir/'groundtruth_enu_flu.tum').open('w') as stream:
        stream.write('# timestamp tx ty tz qx qy qz qw (ENU world, FLU IMU; evaluation only)\n')
        for t,row in gt:
            position=NED_ENU@row[:3]
            rot=NED_ENU@matrix_wxyz(row[3:7])@FRD_FLU
            quat=Rotation.from_matrix(rot).as_quat()
            sec,ns=divmod(t+epoch_ns,1_000_000_000)
            stream.write(f'{sec}.{ns:09d} '+' '.join(f'{v:.12g}' for v in [*position,*quat])+'\n')
            if first is None:
                first={'physical_time_ns':t+epoch_ns,'position_enu_m':position.tolist(),
                    'orientation_flu_to_enu_xyzw':quat.tolist(),'velocity_enu_mps':(NED_ENU@row[7:10]).tolist(),
                    'gyro_bias_flu_rad_s':(FRD_FLU@bias[0][1][:3]).tolist(),
                    'accel_bias_flu_m_s2':(FRD_FLU@bias[0][1][3:]).tolist(),
                    'use':'evaluation-only initial-state oracle; disclose and apply equally if used; otherwise dynamic initialization required'}
    (calibration_dir/'initial_state_evaluation_only.yaml').write_text(yaml.safe_dump(first,sort_keys=False))
    partial.rename(output)
    report={'bag':str(output),'format':'ROS1' if version==1 else 'ROS2 sqlite3',
        'calibration_directory':str(calibration_dir),
        'counts':dict(counts),'readback_pass':True,'timing':timing,
        'maximum_corrected_grid_alignment_error_ns':alignment,
        'sensor_frames':{'imu':'FLU (x forward, y left, z up)','camera':'optical (right, down, forward)'},
        'groundtruth_frame':'ENU, FLU; sidecar only, no GT orientation/position topics in bag',
        'duration_s':cfg['duration_s'],'bytes':output.stat().st_size if output.is_file() else sum(p.stat().st_size for p in output.rglob('*') if p.is_file())}
    output.with_name(output.name+'.report.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--ros-version',type=int,choices=[1,2],default=1)
    p.add_argument('--stamp-mode',choices=['raw','corrected'],default='corrected')
    for option in ['camera-clock-offset-ns','imu-clock-offset-ns','camera-latency-ns','imu-latency-ns']:
        p.add_argument('--'+option,type=int,default=0)
    p.add_argument('--epoch-ns',type=int,default=1_000_000_000)
    args=p.parse_args()
    print(json.dumps(export(args.dataset,args.output,args.ros_version,args.camera_clock_offset_ns,args.imu_clock_offset_ns,
        args.stamp_mode,args.camera_latency_ns,args.imu_latency_ns,args.epoch_ns),indent=2))
