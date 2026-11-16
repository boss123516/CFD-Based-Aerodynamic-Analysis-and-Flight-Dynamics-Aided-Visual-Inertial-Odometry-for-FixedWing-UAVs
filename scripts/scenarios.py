#!/usr/bin/env python3
"""Generate private flight datasets for configurable >=10 minute VIO scenarios."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import numpy as np
import yaml
from generate_flight import generate
from validate_dataset import validate

ROOT=Path(__file__).resolve().parents[1]


def make_config(settings,profile,name):
    region=settings['scenarios'][name]
    config=deepcopy(settings);config.pop('scenarios');config.update(deepcopy(profile))
    location=settings['location']
    config['initial'].update(latitude_deg=location['latitude_deg'],longitude_deg=location['longitude_deg'])
    config['terrain_h_sl_m']=location['reference_terrain_height_m']
    config['trajectory']=deepcopy(region)
    path=config['trajectory']
    config['initial']['altitude_h_sl_m']=path['altitude_h_sl_m']
    config['initial']['calibrated_airspeed_kts']=path['speed_mps']/0.5144444444444445
    config['initial']['heading_deg']=float(np.rad2deg(np.arctan2(2*path['east_radius_m'],path['north_radius_m']))) if path['kind']=='figure_eight' else 0.
    return config


def generate_suite(settings, profile, output, smoke_seconds=None,resume=False):
    if settings['duration_s'] < 600 and smoke_seconds is None:
        raise ValueError('Benchmark scenarios must last at least 600s; use --smoke-seconds for tests')
    output.mkdir(parents=True,exist_ok=resume)
    report={'benchmark':smoke_seconds is None,'CFD':False,'aircraft_validated':False,
            'map_catalog':yaml.safe_load((ROOT/'config/maps.yaml').read_text()),'scenarios':{}}
    for name,region in settings['scenarios'].items():
        config=make_config(settings,profile,name)
        if smoke_seconds is not None:config['duration_s']=smoke_seconds
        dataset=output/name
        if dataset.exists() and resume:
            manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
            def physical(c):
                c=deepcopy(c);c.pop('map_id',None);c['trajectory'].pop('height_above_reference_ground_m',None)
                return c
            if physical(manifest['config'])!=physical(config):raise ValueError(f'{name}: existing flight differs from requested config')
            manifest['config']=config
            (dataset/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
            (dataset/'config.snapshot.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        else:generate(config,dataset)
        validation=validate(dataset)
        if not validation['flight_contract_pass']:
            raise RuntimeError(f'{name}: sensor contract failed: {validation["checks"]}')
        debug=np.genfromtxt(dataset/'flight_debug.csv',delimiter=',',names=True)
        gt=np.genfromtxt(dataset/'groundtruth.csv',delimiter=',',names=True)
        summary={'duration_s':config['duration_s'],'map_id':config.get('map_id','strasbourg_city'),
            'camera_frames':int(config['duration_s']*config['camera_hz'])+1,
            'imu_samples':int(config['duration_s']*config['imu_hz'])+1,
            'mean_airspeed_mps':float(debug['airspeed_mps'].mean()),
            'max_airspeed_mps':float(debug['airspeed_mps'].max()),
            'mean_groundspeed_mps':float(np.sqrt(gt['vx']**2+gt['vy']**2+gt['vz']**2).mean()),
            'altitude_h_sl_range_m':[float(debug['alt_h_sl_m'].min()),float(debug['alt_h_sl_m'].max())],
            'max_abs_roll_deg':float(np.rad2deg(abs(debug['roll_rad'])).max()),
            'max_abs_alpha_deg':float(np.rad2deg(abs(debug['alpha_rad'])).max()),
            'max_body_rate_deg_s':float(np.rad2deg(np.sqrt(sum(debug[k]**2 for k in ['p_body_ecef_rad_s','q_body_ecef_rad_s','r_body_ecef_rad_s']))).max()),
            'max_abs_vertical_speed_mps':float(abs(gt['vz']).max()),
            'distance_m':float(np.linalg.norm(np.diff(np.column_stack([gt['px'],gt['py'],gt['pz']]),axis=0),axis=1).sum()),
            'sensor_contract_pass':True,'status':'flight_generated_images_pending',
            'aircraft':config['aircraft'], 'size_bytes':sum(p.stat().st_size for p in dataset.rglob('*') if p.is_file()),
            'raw_mono8_bag_image_bytes':(int(config['duration_s']*config['camera_hz'])+1)*config['camera']['width']*config['camera']['height']}
        (dataset/'scenario_report.private.json').write_text(json.dumps(summary,indent=2))
        report['scenarios'][name]=summary
        (output/'suite_report.private.json').write_text(json.dumps(report,indent=2))
    plot_suite(output)
    return report


def plot_suite(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    paths=[output/name for name in ['easy','mid','hard'] if (output/name/'groundtruth.csv').exists()]
    fig,axes=plt.subplots(2,len(paths),figsize=(5*len(paths),7),squeeze=False)
    for i,path in enumerate(paths):
        gt=np.genfromtxt(path/'groundtruth.csv',delimiter=',',names=True)
        debug=np.genfromtxt(path/'flight_debug.csv',delimiter=',',names=True)
        ax=axes[0,i];ax.plot(gt['py'],gt['px']);ax.scatter([gt['py'][0]],[gt['px'][0]],s=25,c='green')
        ax.set(xlabel='East [m]',ylabel='North [m]',title=path.name);ax.axis('equal');ax.grid()
        axes[1,i].plot(debug['t_ns']/1e9,debug['alt_h_sl_m'])
        axes[1,i].set(xlabel='Time [s]',ylabel='Altitude h-sl [m]');axes[1,i].grid()
    fig.tight_layout();fig.savefig(output/'scenario_trajectories.private.png',dpi=130);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=ROOT/'config/scenarios.yaml')
    p.add_argument('--aircraft-profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--smoke-seconds',type=float)
    p.add_argument('--resume',action='store_true')
    args=p.parse_args()
    result=generate_suite(yaml.safe_load(args.config.read_text()),yaml.safe_load(args.aircraft_profile.read_text()),args.output.resolve(),args.smoke_seconds,args.resume)
    print(json.dumps({'output':str(args.output),'scenarios':list(result['scenarios'])}))
