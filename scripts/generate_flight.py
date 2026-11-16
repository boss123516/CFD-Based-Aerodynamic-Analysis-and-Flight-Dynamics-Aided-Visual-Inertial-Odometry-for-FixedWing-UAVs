#!/usr/bin/env python3
"""JSBSim flight -> synchronized GT, ideal IMU, optical camera poses."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('JSBSIM_DEBUG', '0')
import jsbsim
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from frames import FT, camera_transform, ecef_to_ned, kinematic_imu, quat_wxyz
from environment import configure_wind, gust_ned, apply_imu_noise
from aircraft_model import build_model, approximate_trim
from flight_path import PathFollower

ROOT = Path(__file__).resolve().parents[1]


def write_csv(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as stream:
        out = csv.writer(stream)
        out.writerow(header.split(','))
        out.writerows(rows)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bank_command(t):
    # Continuous slope at roll entry/exit; integration flight, not evaluation set.
    ramp = lambda s: (1 - np.cos(np.pi * np.clip(s / 5, 0, 1))) / 2
    return np.deg2rad(20) * (ramp(t-15) - ramp(t-40))


def initialize(config, model_directory=None):
    f = jsbsim.FGFDMExec(None)
    f.set_debug_level(0)
    f.disable_output()
    custom = config.get('model_kind') == 'approximate_flying_wing'
    if custom:
        build_model(config,model_directory)
        f.set_aircraft_path(str(model_directory))
    if not f.load_model('wing' if custom else config['model']):
        raise RuntimeError('JSBSim model failed to load')
    f.set_dt(1 / config['physics_hz'])
    ic = config['initial']
    for key, value in {
        'ic/lat-geod-deg': ic['latitude_deg'],
        'ic/long-gc-deg': ic['longitude_deg'],
        'ic/h-sl-ft': ic['altitude_h_sl_m'] / FT,
        'ic/vc-kts': ic['calibrated_airspeed_kts'],
        'ic/psi-true-deg': ic['heading_deg'],
        'ic/gamma-deg': 0,
        'fcs/mixture-cmd-norm': 1,
        'atmosphere/turb-type': 0,
        'atmosphere/wind-north-fps': 0,
        'atmosphere/wind-east-fps': 0,
        'atmosphere/wind-down-fps': 0,
    }.items():
        f[key] = value
    if 'terrain_h_sl_m' in config:
        f['ic/terrain-elevation-ft'] = config['terrain_h_sl_m']/FT
    if custom:
        alpha,elevator,throttle=approximate_trim(config)
        f['ic/alpha-deg']=float(np.rad2deg(alpha))
        f['fcs/elevator-cmd-norm']=elevator
        f['control/throttle']=throttle
    configure_wind(f,config)
    # Solve equilibrium in the steady wind; enable random turbulence afterward.
    f['atmosphere/turb-type'] = 0
    if not f.run_ic():
        raise RuntimeError('JSBSim initial conditions failed')
    if not custom:
        f['propulsion/set-running'] = -1
        f.do_trim(0)
    configure_wind(f,config)
    if not np.isclose(f.get_delta_t(), 1/config['physics_hz'], atol=1e-12):
        raise RuntimeError('Physics dt mismatch')
    return f


def generate(config, output):
    output=Path(output).resolve()
    hz = config['physics_hz']
    if not isinstance(hz,int) or hz <= 0 or 1_000_000_000 % hz:
        raise ValueError('physics_hz must divide integer nanoseconds')
    for rate in (config['imu_hz'], config['camera_hz']):
        if hz % rate or 1_000_000_000 % rate:
            raise ValueError('Rates must divide physics_hz and integer nanoseconds')
    if config['duration_s'] <= 0 or config['settle_s'] * hz < 1:
        raise ValueError('Invalid duration')
    custom = config.get('model_kind') == 'approximate_flying_wing'
    if config['model'] != 'c172p' and not custom:
        raise ValueError('Current controller is validated only for c172p; add a model-specific controller first')
    if custom and output.is_relative_to(ROOT) and not any(output.is_relative_to(ROOT/p) for p in ['datasets','runs']):
        raise ValueError('Private aircraft outputs inside the repo must stay under ignored datasets/ or runs/')
    output.mkdir(parents=True, exist_ok=False,mode=0o700 if custom else 0o755)
    (output / '.flight_incomplete').touch()
    (output / 'config.snapshot.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    f = initialize(config,output/'private_model')
    trim_pitch = f['attitude/theta-rad']
    throttle_property='control/throttle' if custom else 'fcs/throttle-cmd-norm'
    trim_throttle = f[throttle_property]
    speed_ref = f['velocities/vtrue-fps'] * FT
    alt_ref = f['position/h-sl-ft'] * FT
    trim = {k: f[k] for k in ['fcs/pitch-trim-cmd-norm', 'fcs/elevator-cmd-norm',
                             throttle_property, 'attitude/theta-rad']}
    settle = round(config['settle_s'] * hz)
    count = round(config['duration_s'] * hz)
    if not np.isclose(count/hz, config['duration_s']):
        raise ValueError('Duration must be an integer physics tick')
    if count % (hz//config['camera_hz']):
        raise ValueError('Duration must end on a camera tick')
    p_ecef, r_ecef_body, v_ecef, debug = [], [], [], []
    follower=None
    anchor_ecef=None
    path_error=[]
    for step in range(settle + count + 3):
        t = (step-settle)/hz
        gust = gust_ned(t,config.get('wind',{}))
        for axis,value in zip(['north','east','down'],gust):f[f'atmosphere/gust-{axis}-fps']=float(value)/FT
        phi, theta = f['attitude/phi-rad'], f['attitude/theta-rad']
        p, q, r = (f[f'velocities/{s}-rad_sec'] for s in 'pqr')
        bank_ref = bank_command(t)
        altitude = f['position/h-sl-ft'] * FT
        speed = f['velocities/vtrue-fps'] * FT
        distance_error=0.
        if custom and 'trajectory' in config:
            # A fixed NED navigation frame is established at export start.
            gc=f['position/lat-gc-rad'];lon=f['position/long-gc-rad']
            radius=f['position/radius-to-vehicle-ft']*FT
            current_ecef=radius*np.array([np.cos(gc)*np.cos(lon),np.cos(gc)*np.sin(lon),np.sin(gc)])
            if step == settle:
                anchor_ecef=current_ecef
                nav_rotation=ecef_to_ned(f['position/lat-geod-rad'],lon)
                follower=PathFollower(config['trajectory'],[0,0])
            if follower is not None:
                nav_position=(nav_rotation@(current_ecef-anchor_ecef))[:2]
                nav_velocity=np.array([f['velocities/v-north-fps'],f['velocities/v-east-fps']])*FT
                bank_ref,alt_ref,speed_ref,distance_error=follower.command(t,nav_position,nav_velocity,speed,1/hz)
            else:
                bank_ref=0.
                alt_ref=config['initial']['altitude_h_sl_m']
        pitch_ref = trim_pitch + np.clip((.018 if custom else .006)*(alt_ref-altitude), -.18 if custom else -.08, .18 if custom else .08)
        pitch_ref += 0.10*(1/np.cos(phi)-1)
        f['fcs/aileron-cmd-norm'] = np.clip((.8 if custom else 1.5)*(bank_ref-phi)-(.14 if custom else .7)*p, -.6, .6)
        elevator_trim=approximate_trim(config)[1] if custom else 0.
        f['fcs/elevator-cmd-norm'] = np.clip(elevator_trim-(1.5 if custom else 2.)*(pitch_ref-theta)+(.18 if custom else .8)*q, -.5, .5)
        f['fcs/rudder-cmd-norm'] = np.clip(-1.0*f['aero/beta-rad'] + .5*(9.80665*np.tan(phi)/speed-r), -.3, .3)
        f['control/throttle' if custom else 'fcs/throttle-cmd-norm'] = np.clip(trim_throttle + (.10 if custom else .06)*(speed_ref-speed), .05, 1)
        if step >= settle-1:
            lat, lon = f['position/lat-geod-rad'], f['position/long-gc-rad']
            transform = ecef_to_ned(lat, lon).T
            # Use JSBSim's own geocentric radius for exact ECEF positions.
            gc = f['position/lat-gc-rad']
            radius = f['position/radius-to-vehicle-ft'] * FT
            p_ecef.append(radius*np.array([np.cos(gc)*np.cos(lon), np.cos(gc)*np.sin(lon), np.sin(gc)]))
            local_body = Rotation.from_euler('ZYX', [f['attitude/psi-rad'], theta, phi]).as_matrix()
            r_ecef_body.append(transform @ local_body)
            vel = np.array([f[f'velocities/v-{s}-fps'] for s in ['north','east','down']])*FT
            v_ecef.append(transform @ vel)
            debug.append([t, altitude, speed, phi, theta, f['attitude/psi-rad'],
                          f['aero/alpha-rad'], f['aero/beta-rad'], bank_ref,
                          f['fcs/aileron-cmd-norm'], f['fcs/elevator-cmd-norm'],
                          f['fcs/rudder-cmd-norm'], f['control/throttle' if custom else 'fcs/throttle-cmd-norm'],
                          lat, lon, f['position/geod-alt-ft']*FT, p,q,r,
                          *[f[f'atmosphere/total-wind-{s}-fps']*FT for s in ['north','east','down']],
                          *[f[f'atmosphere/wind-{s}-fps']*FT for s in ['north','east','down']],
                          *[f[f'atmosphere/turb-{s}-fps']*FT for s in ['north','east','down']],*gust])
            path_error.append(distance_error)
        if not f.run():
            raise RuntimeError(f'JSBSim stopped at {t}s')
    r0 = ecef_to_ned(debug[1][13], debug[1][14])
    # One sample on either side of exported times supports centered derivatives.
    p_ecef, v_ecef, r_ecef_body = map(np.asarray, (p_ecef, v_ecef, r_ecef_body))
    position = (p_ecef-p_ecef[1]) @ r0.T
    velocity = v_ecef @ r0.T
    rotations = np.einsum('ij,njk->nik', r0, r_ecef_body)
    gyro, force = kinematic_imu(velocity, rotations, 1/hz, config['gravity_mps2'])
    sl = slice(1, count+2)
    position, velocity, rotations, gyro, force = [a[sl] for a in (position,velocity,rotations,gyro,force)]
    debug = np.array(debug)[sl]
    if custom:
        a=config['aircraft']
        if debug[:,1].min()<config.get('terrain_h_sl_m',0)+30:
            raise RuntimeError('Surrogate flight descended below terrain reference +30m')
        if np.rad2deg(abs(debug[:,6])).max()>15:
            raise RuntimeError('Surrogate exceeded linear lift curve validity envelope (15deg alpha)')
        if debug[:,2].max()>a['simulation_speed_limit_mps']:
            raise RuntimeError('Surrogate exceeded configured simulation speed limit')
    for a in (position,velocity,rotations,gyro,force,debug):
        if not np.isfinite(a).all():
            raise RuntimeError('Nonfinite flight state')
    times = np.arange(count+1, dtype=np.int64) * (1_000_000_000//hz)
    quat = quat_wxyz(rotations)
    imu_idx = np.arange(0, count+1, hz//config['imu_hz'])
    cam_idx = np.arange(0, count+1, hz//config['camera_hz'])
    gt = np.column_stack((position,quat,velocity))
    rows = lambda indices, values: ([int(times[i]), *values[i]] for i in indices)
    write_csv(output/'groundtruth.csv', 't_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz', rows(imu_idx,gt))
    write_csv(output/'physics_groundtruth.csv', 't_ns,px,py,pz,qw,qx,qy,qz,vx,vy,vz', rows(range(count+1),gt))
    noise = config['noise']
    measured_gyro,measured_force,gyro_bias,accel_bias = apply_imu_noise(
        gyro[imu_idx],force[imu_idx],noise,config['seed'],1/config['imu_hz'])
    write_csv(output/'imu.csv','t_ns,wx,wy,wz,fx,fy,fz',
              ([int(times[i]),*w,*a] for i,w,a in zip(imu_idx, measured_gyro,measured_force)))
    T = camera_transform(config['camera'])
    cam_p = position + np.einsum('nij,j->ni', rotations, T[:3,3])
    cam_q = quat_wxyz(rotations @ T[:3,:3])
    write_csv(output/'camera_poses.csv','t_ns,px,py,pz,qw,qx,qy,qz',rows(cam_idx,np.column_stack((cam_p,cam_q))))
    write_csv(output/'flight_debug.csv',
              't_ns,alt_h_sl_m,airspeed_mps,roll_rad,pitch_rad,yaw_rad,alpha_rad,beta_rad,bank_command_rad,aileron_cmd,elevator_cmd,rudder_cmd,throttle_cmd,latitude_rad,longitude_rad,ellipsoid_height_m,p_body_ecef_rad_s,q_body_ecef_rad_s,r_body_ecef_rad_s,wind_n_mps,wind_e_mps,wind_d_mps,mean_wind_n_mps,mean_wind_e_mps,mean_wind_d_mps,turb_n_mps,turb_e_mps,turb_d_mps,gust_n_mps,gust_e_mps,gust_d_mps',
              rows(range(count+1),debug[:,1:]))
    # EuRoC-like schema; all world/body frame conventions stay NED/FRD.
    write_csv(output/'mav0/imu0/data.csv', '#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]',
              ([int(times[i]),*w,*a] for i,w,a in zip(imu_idx,measured_gyro,measured_force)))
    write_csv(output/'mav0/state_groundtruth_estimate0/data.csv',
              '#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,b_w_x,b_w_y,b_w_z,b_a_x,b_a_y,b_a_z',
              ([int(times[i]),*gt[i],*bg,*ba] for i,bg,ba in zip(imu_idx,gyro_bias,accel_bias)))
    write_csv(output/'imu_bias.csv','t_ns,bgx,bgy,bgz,bax,bay,baz',
              ([int(times[i]),*bg,*ba] for i,bg,ba in zip(imu_idx,gyro_bias,accel_bias)))
    write_csv(output/'mav0/cam0/data.csv', '#timestamp [ns],filename',
              ([int(times[i]),f'{times[i]}.png'] for i in cam_idx))
    (output/'mav0/cam0/data').mkdir()
    cam = config['camera']
    focal = cam['width']/(2*np.tan(np.deg2rad(cam['horizontal_fov_deg'])/2))
    calibration = {'camera_model':'pinhole','resolution':[cam['width'],cam['height']],
                   'intrinsics':[float(focal),float(focal),(cam['width']-1)/2,(cam['height']-1)/2],
                   'distortion_model':'radtan','distortion_coeffs':[0.,0.,0.,0.],
                   'intrinsics_status':'planned; renderer must verify runtime FOV and image dimensions',
                   'T_IC':T.tolist(), 'T_IC_definition':'optical camera to FRD IMU at CG',
                   'rigid_mount':True, 'sensor_rate_hz':config['camera_hz']}
    (output/'calibration.yaml').write_text(yaml.safe_dump(calibration,sort_keys=False))
    data_root = Path(f.get_root_dir())
    model_path = output/'private_model/wing/wing.xml' if custom else data_root/'aircraft'/config['model']/f"{config['model']}.xml"
    data_hash = hashlib.sha256()
    for path in sorted(data_root.rglob('*.xml')):
        data_hash.update(str(path.relative_to(data_root)).encode());data_hash.update(path.read_bytes())
    manifest = {'schema_version':1,'status':'flight_generated_images_pending',
                'research_ready':False,'CFD_model_applied':False,'model':config['model'],
                'model_role':config['model_role'],'jsbsim_version':jsbsim.__version__,
                'model_sha256':sha(model_path),'bundled_xml_sha256':data_hash.hexdigest(),
                'source_sha256':{name:sha(ROOT/'scripts'/name) for name in ['generate_flight.py','frames.py','environment.py','aircraft_model.py','flight_path.py']},
                'config':config,'trim':trim,'actual_dt_s':f.get_delta_t(),
                'jsbsim_export_epoch_s':settle/hz,
                'frames':{'world':'fixed tangent NED at first exported CG position',
                          'imu':'FRD at CG','camera':'optical right-down-forward',
                          'quaternion':'wxyz, active body/camera to fixed world'},
                'origin':{'latitude_rad':float(debug[0,13]),'longitude_rad':float(debug[0,14]),
                          'ellipsoid_height_m':float(debug[0,15])},
                'imu_model':'kinematic flat-local: central d(v_world)/dt and R^T dR/dt at 200Hz; f=R^T(a-[0,0,g]); no Earth-rate sensing',
                'imu_origin':'CG; no IMU lever arm',
                'time':'integer ns since first exported state, not wall/render time',
                'altitude_note':'h-sl initial condition and actual ellipsoid height logged separately; terrain AGL not measured',
                'counts':{'imu':len(imu_idx),'camera_poses':len(cam_idx),'physics':count+1},
                'online_inputs':['mav0/cam0','mav0/imu0'],
                'evaluation_only':['groundtruth.csv','physics_groundtruth.csv','flight_debug.csv','imu_bias.csv','mav0/state_groundtruth_estimate0'],
                'sha256':{p.name:sha(p) for p in output.glob('*.csv')}}
    if custom:
        manifest['aircraft_validation']='unvalidated flying-wing surrogate; published dimensions plus assumed mass/inertia/coefficients; no paid CAD assembly inference or CFD'
        manifest['controller']={'trajectory':config.get('trajectory'),
            'maximum_path_error_m':float(np.max(np.array(path_error)[sl]))}
        manifest['aircraft_private']=True
    (output/'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))
    (output/'.flight_incomplete').unlink()
    print(json.dumps({'output':str(output),'counts':manifest['counts'],
                      'altitude_range_m':[float(debug[:,1].min()),float(debug[:,1].max())],
                      'speed_range_mps':[float(debug[:,2].min()),float(debug[:,2].max())],
                      'max_roll_deg':float(np.rad2deg(abs(debug[:,3]).max())),
                      'max_beta_deg':float(np.rad2deg(abs(debug[:,7]).max()))},indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/straight_turn.yaml')
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    generate(yaml.safe_load(args.config.read_text()), args.output.resolve())
