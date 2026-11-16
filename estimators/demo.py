"""IMU-only plumbing demo with optional sideslip updates. NOT a VIO/MSCKF."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from cfd.model import BetaLUT
from dynamics.constraint import linearize,skew


def run(sensor_input,output,method,project_root):
    # Fixed shared initialization assumption, never read simulator GT here.
    imu = np.loadtxt(sensor_input/'mav0/imu0/data.csv',delimiter=',',skiprows=1)
    position = np.zeros(3)
    velocity = np.array([47.,0.,0.])
    rotation = Rotation.from_euler('ZYX',[0.,.023,0.]).as_matrix()
    P = np.diag([1.]*3+[4.]*3+[.01]*3)
    config = method['dynamics']
    lut = None
    if method['cfd']['enabled']:
        lut = BetaLUT(project_root/'cfd/demo_lut',allow_demo=True)
    trajectory,diagnostics = [],[]
    for index,row in enumerate(imu):
        t = int(row[0])
        if index:
            dt = (t-int(imu[index-1,0]))/1e9
            omega,force = (imu[index-1,1:4]+row[1:4])/2,(imu[index-1,4:7]+row[4:7])/2
            mid = rotation @ Rotation.from_rotvec(omega*dt/2).as_matrix()
            acceleration = mid @ force + [0,0,9.80665]
            position += velocity*dt+acceleration*dt*dt/2
            velocity += acceleration*dt
            F = np.eye(9)
            F[:3,3:6] = np.eye(3)*dt
            F[3:6,6:9] = -mid @ skew(force)*dt
            P = F @ P @ F.T + np.diag([1e-8]*3+[1e-4]*3+[1e-7]*3)*dt
            rotation = rotation @ Rotation.from_rotvec(omega*dt).as_matrix()
        if config['enabled'] and t % 100_000_000 == 0:
            mean,sigma,source = config['beta_ref_rad'],config['sigma_beta_rad'],'generic'
            if lut:
                bank = np.arctan2(rotation[2,1],rotation[2,2])
                entry = lut.query(bank,float(np.linalg.norm(velocity)))
                if entry:
                    mean,sigma = entry
                    source = 'toy_lut_not_CFD'
                else:
                    sigma *= 2
                    source = 'out_of_range_generic_fallback'
            # LUT conditions are frozen before this update; no GT wind/velocity.
            observation = linearize(velocity,rotation,mean,min_speed=config['min_speed_mps'])
            if observation:
                residual,H = observation
                variance = sigma**2
                S = float((H @ P @ H.T).item()+variance)
                nis = residual**2/S
                applied = nis<=9
                if applied:
                    K = (P @ H.T)/S
                    delta = K[:,0]*residual
                    position += delta[:3];velocity += delta[3:6]
                    rotation = rotation @ Rotation.from_rotvec(delta[6:9]).as_matrix()
                    J = np.eye(9)-K @ H
                    P = J @ P @ J.T + variance*(K @ K.T)
                    reset = np.eye(9);reset[6:9,6:9] -= .5*skew(delta[6:9])
                    P = reset @ P @ reset.T
                    P = (P+P.T)/2
                diagnostics.append([t,residual,variance,S,nis,int(applied),source])
        q = np.roll(Rotation.from_matrix(rotation).as_quat(),1)
        trajectory.append([t,*position,*q])
    with (output/'trajectory.csv').open('w',newline='') as stream:
        writer = csv.writer(stream);writer.writerow(['t_ns','px','py','pz','qw','qx','qy','qz']);writer.writerows(trajectory)
    with (output/'dynamics.csv').open('w',newline='') as stream:
        writer = csv.writer(stream);writer.writerow(['t_ns','residual','R','S','NIS','applied','source']);writer.writerows(diagnostics)
    (output/'backend.json').write_text(json.dumps({'backend':'IMU_only_plumbing_demo_NOT_VIO',
        'scientific_result':False,'initialization':'shared fixed p=0,v=[47,0,0],rpy=[0,0.023,0]; no GT read',
        'visual_frontend':False,'CFD_data':False,'LUT_condition_jacobian':'frozen before update'},indent=2))
