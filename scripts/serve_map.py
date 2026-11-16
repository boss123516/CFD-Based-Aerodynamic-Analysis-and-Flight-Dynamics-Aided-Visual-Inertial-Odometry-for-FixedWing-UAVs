#!/usr/bin/env python3
"""Local real-world 3D Tiles viewer + synchronized JSBSim sensor capture."""
import argparse
import csv
from datetime import datetime
import gzip
import hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import io
import json
import mimetypes
import shutil
from pathlib import Path
import threading
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np
from PIL import Image
import yaml
from frames import ecef_to_ned,matrix_wxyz
from generate_flight import generate
from validate_dataset import validate
from scenarios import make_config

ROOT=Path(__file__).resolve().parents[1]
MAP=yaml.safe_load((ROOT/'config/map.yaml').read_text())
DEFAULT=yaml.safe_load((ROOT/'config/strasbourg.yaml').read_text())
CACHE=ROOT/'external/map_cache/strasbourg_2022';CACHE.mkdir(parents=True,exist_ok=True)
LOCK=threading.RLock()
JOB={'phase':'idle','captured':0}
CACHE_INDEX={}
SCENARIOS=yaml.safe_load((ROOT/'config/scenarios.yaml').read_text())
PROFILE=None
MAPS=yaml.safe_load((ROOT/'config/maps.yaml').read_text())


def attach_dataset(dataset):
    """Resume only a contiguous, checksummed prefix of an existing flight."""
    global JOB,MAP
    dataset=Path(dataset).resolve()
    if not dataset.is_relative_to(ROOT/'datasets'):raise ValueError('Dataset must be under project datasets/')
    with LOCK:
        if JOB['phase'] in ['generating','flight_ready','capturing']:raise ValueError('Capture already running')
        manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
        MAP=MAPS.get(manifest['config'].get('map_id'),yaml.safe_load((ROOT/'config/map.yaml').read_text()))
        if (dataset/'.flight_incomplete').exists():raise ValueError('Flight generation incomplete')
        if (dataset/'acquisition_report.json').exists() and not (dataset/'.render_incomplete').exists():
            report=json.loads((dataset/'acquisition_report.json').read_text())
            JOB={'phase':'complete','dataset':str(dataset),'captured':report['image_count'],'report':report}
            return
        poses=np.atleast_2d(np.loadtxt(dataset/'camera_poses.csv',delimiter=',',skiprows=1))
        rows=[]
        journal=dataset/'render_journal.jsonl'
        if journal.exists():
            for line in journal.read_text().splitlines():
                row=json.loads(line);index=len(rows)
                if int(row['t_ns'])!=int(poses[index,0]):raise ValueError('Noncontiguous capture journal')
                image=dataset/'mav0/cam0/data'/f"{row['t_ns']}.png"
                if not image.exists() or hashlib.sha256(image.read_bytes()).hexdigest()!=row['image_sha256']:
                    raise ValueError('Resume image missing / modified')
                rows.append(row)
        JOB={'phase':'flight_ready','captured':len(rows),'dataset':str(dataset),'name':dataset.name,
            'config':manifest['config'],'expected_frames':len(poses),'imu_samples':manifest['counts']['imu'],
            'poses':poses.tolist(),'origin':manifest['origin'],'rows':rows,'started_wall_ns':time.time_ns()}
        (dataset/'.render_incomplete').touch()
        if len(rows)==len(poses):finalize(dataset)


def ecef(lat,lon,h):
    a=6378137.;e2=6.6943799901413165e-3
    N=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([(N+h)*np.cos(lat)*np.cos(lon),(N+h)*np.cos(lat)*np.sin(lon),(N*(1-e2)+h)*np.sin(lat)])


def start_job(params):
    global JOB,MAP
    with LOCK:
        if JOB['phase'] in ['generating','capturing','flight_ready']:raise ValueError('Capture already running')
        config=yaml.safe_load(yaml.safe_dump(DEFAULT))
        if PROFILE is not None:
            scenario=params.get('scenario','easy')
            if scenario not in SCENARIOS['scenarios']:raise ValueError('Unknown scenario')
            config=make_config(SCENARIOS,PROFILE,scenario)
            MAP=MAPS[config['map_id']]
            for key in ['altitude_h_sl_m','speed_mps']:
                if key in params:config['trajectory'][key]=float(params[key])
            path=config['trajectory']
            if not 5<=path['speed_mps']<=config['aircraft']['simulation_speed_limit_mps']:raise ValueError('Invalid reference airspeed')
            if not config['terrain_h_sl_m']+50<=path['altitude_h_sl_m']<=2000:raise ValueError('Invalid reference altitude')
            config['initial']['altitude_h_sl_m']=path['altitude_h_sl_m']
            config['initial']['calibrated_airspeed_kts']=path['speed_mps']/0.5144444444444445
            config['initial']['heading_deg']=float(np.rad2deg(np.arctan2(2*path['east_radius_m'],path['north_radius_m']))) if path['kind']=='figure_eight' else 0.
        duration=float(params.get('duration_s',config['duration_s']))
        if not .1<=duration<=3600 or not np.isclose(duration*config['camera_hz'],round(duration*config['camera_hz'])):raise ValueError('Duration: 0.1..3600s on a camera boundary')
        config['duration_s']=duration
        for key in ['mean_ned_mps','turbulence','gusts']:
            if key in params:config['wind'][key]=params[key]
        for key in config['noise']:
            if key in params:config['noise'][key]=params[key]
        name='strasbourg_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        dataset=ROOT/'datasets'/name
        JOB={'phase':'generating','captured':0,'dataset':str(dataset),'name':name,'config':config}
    def worker():
        try:
            generate(config,dataset)
            poses=np.atleast_2d(np.loadtxt(dataset/'camera_poses.csv',delimiter=',',skiprows=1))
            manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
            with LOCK:
                JOB.update(phase='flight_ready',expected_frames=len(poses),imu_samples=manifest['counts']['imu'],
                           poses=poses.tolist(),origin=manifest['origin'],rows=[],started_wall_ns=time.time_ns())
                (dataset/'.render_incomplete').touch()
        except Exception as error:
            with LOCK:JOB.update(phase='failed',error=str(error))
    threading.Thread(target=worker,daemon=True).start()
    return {'name':name}


def finalize(dataset):
    rows=JOB['rows']
    with (dataset/'render_mapping.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
    manifest['status']='real_3D_map_images_captured'
    manifest['renderer']={'backend':'cesiumjs_real_map','version':'1.124.0','map':MAP,
        'tile_completion_verified':True,'reprojection_verified':True,'camera_pose_verified':True,
        'images':'GPU-rendered views of the public textured triangle mesh; not real camera photographs',
        'screen_space_error':MAP['maximum_screen_space_error'],'clock_utc':'2022-06-21T10:00:00Z'}
    (dataset/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
    calibration=yaml.safe_load((dataset/'calibration.yaml').read_text())
    calibration['intrinsics_status']='runtime centered pinhole reprojection checked against Cesium projection'
    (dataset/'calibration.yaml').write_text(yaml.safe_dump(calibration,sort_keys=False))
    (dataset/'map_cache_manifest.json').write_text(json.dumps({'map':MAP,'files':CACHE_INDEX},indent=2))
    (dataset/'.render_incomplete').unlink()
    validation=validate(dataset,require_images=True)
    report={'sensor_contract_pass':validation['flight_contract_pass'],
        'image_count':len(rows),'imu_count':JOB['imu_samples'],
        'minimum_feature_count':min(r['features'] for r in rows),
        'median_feature_count':float(np.median([r['features'] for r in rows])),
        'maximum_pose_error_m':max(r['pose_error_m'] for r in rows),
        'maximum_reprojection_error_px':max(r['projection_error_px'] for r in rows),
        'tiles_ready_all_frames':all(r['tiles_loaded'] for r in rows),
        'map':MAP,'scientific_result':False,'VIO_executed':False,'CFD_model_applied':False,
        'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (dataset/'mav0/cam0/data').glob('*.png')}}
    (dataset/'acquisition_report.json').write_text(json.dumps(report,indent=2))
    report['dataset_bytes_before_bag']=sum(p.stat().st_size for p in dataset.rglob('*') if p.is_file())
    (dataset/'acquisition_report.json').write_text(json.dumps(report,indent=2))
    scenario_report=dataset/'scenario_report.private.json'
    if scenario_report.exists():
        summary=json.loads(scenario_report.read_text());summary.update(status='real_map_images_verified',
            image_count=len(rows),size_bytes=sum(p.stat().st_size for p in dataset.rglob('*') if p.is_file()))
        scenario_report.write_text(json.dumps(summary,indent=2))
    JOB.update(phase='complete',report=report)
    print(json.dumps({'capture_complete':str(dataset),'frames':len(rows),'imu':JOB['imu_samples']}),flush=True)


class Handler(BaseHTTPRequestHandler):
    def log_message(self,fmt,*args):
        if '/tiles/' not in self.path:super().log_message(fmt,*args)

    def send(self,data,kind='application/json',status=200):
        if not isinstance(data,bytes):data=json.dumps(data).encode()
        self.send_response(status);self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-cache')
        self.end_headers();self.wfile.write(data)

    def do_GET(self):
        path=urllib.parse.urlsplit(self.path).path
        try:
            if path=='/favicon.ico':return self.send(b'','image/x-icon',204)
            if path=='/api/config':
                statistics=None
                report=Path(JOB.get('dataset','/nonexistent'))/'scenario_report.private.json'
                if report.exists():statistics=json.loads(report.read_text())
                return self.send({'map':MAP,'flight':JOB.get('config',make_config(SCENARIOS,PROFILE,'easy') if PROFILE else DEFAULT),
                    'statistics':statistics,'scenario_mode':PROFILE is not None,
                    'scenarios':{name:make_config(SCENARIOS,PROFILE,name)['trajectory'] for name in SCENARIOS['scenarios']} if PROFILE else {}})
            if path=='/api/status':
                with LOCK:return self.send({k:v for k,v in JOB.items() if k not in ['poses','rows','config','report']})
            if path=='/api/poses':
                with LOCK:
                    if 'poses' not in JOB:raise ValueError('Flight not ready')
                    return self.send({**{k:JOB[k] for k in ['name','poses','origin','config']},'start_index':JOB['captured']})
            if path.startswith('/tiles/'):
                relative=self.path[len('/tiles/'):]
                target=urllib.parse.urljoin(MAP['tileset_url'],relative)
                base=MAP['tileset_url'].rsplit('/',1)[0]+'/'
                if not target.startswith(base):raise ValueError('Tile must remain inside map root')
                key=hashlib.sha256(target.encode()).hexdigest();file=CACHE/key
                if file.exists():data=file.read_bytes()
                else:
                    request=urllib.request.Request(target,headers={'User-Agent':'KRoC-3D-map-research-capture/1.0','Accept-Encoding':'identity'})
                    with urllib.request.urlopen(request,timeout=45) as response:data=response.read()
                    if data[:2]==b'\x1f\x8b':data=gzip.decompress(data)
                    temporary=file.with_suffix('.'+str(threading.get_ident())+'.tmp');temporary.write_bytes(data);temporary.replace(file)
                with LOCK:CACHE_INDEX[target]={'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data),'cache_key':key}
                return self.send(data,'application/json' if path.endswith('.json') else 'application/octet-stream')
            if path.startswith('/vendor/'):
                file=(ROOT/'external/cesium/Build/Cesium'/path[len('/vendor/'):]).resolve()
                if not file.is_relative_to((ROOT/'external/cesium/Build/Cesium').resolve()):raise ValueError('Invalid path')
            else:
                file=ROOT/'renderers/cesium' / ('index.html' if path=='/' else path.lstrip('/'))
                if not file.resolve().is_relative_to((ROOT/'renderers/cesium').resolve()):raise ValueError('Invalid path')
            return self.send(file.read_bytes(),mimetypes.guess_type(file)[0] or 'application/octet-stream')
        except Exception as error:
            print('GET failed:',path,repr(error),flush=True)
            self.send({'error':str(error)},status=500)

    def do_POST(self):
        try:
            size=int(self.headers.get('Content-Length',0))
            if size>8_000_000:raise ValueError('Request too large')
            body=self.rfile.read(size)
            if self.path=='/api/start':return self.send(start_job(json.loads(body or '{}')))
            if self.path=='/api/attach':
                attach_dataset(json.loads(body)['dataset']);return self.send({'ok':True})
            if self.path=='/api/fail':
                with LOCK:JOB.update(phase='failed',error=json.loads(body)['error'])
                return self.send({'ok':True})
            if self.path.startswith('/api/frame?'):
                meta=json.loads(self.headers['X-Capture-Metadata'])
                with LOCK:
                    if JOB['phase'] not in ['flight_ready','capturing']:raise ValueError('No capture in progress')
                    index=JOB['captured'];pose=JOB['poses'][index]
                    timestamp=int(urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query)['t_ns'][0])
                    if timestamp!=int(pose[0]):raise ValueError('Frame timestamp does not match next pose')
                    if not meta['tiles_loaded'] or meta['visible_tiles']<1:raise ValueError('3D mesh not loaded for this pose')
                    origin=JOB['origin'];r_ew=ecef_to_ned(origin['latitude_rad'],origin['longitude_rad']).T
                    p0=ecef(origin['latitude_rad'],origin['longitude_rad'],origin['ellipsoid_height_m'])
                    expected=p0+r_ew @ np.array(pose[1:4]);r_ec=r_ew @ matrix_wxyz(pose[4:8])
                    error=float(np.linalg.norm(expected-meta['camera_position_ecef']))
                    direction_error=float(np.linalg.norm(r_ec[:,2]-meta['camera_direction_ecef']))
                    up_error=float(np.linalg.norm(-r_ec[:,1]-meta['camera_up_ecef']))
                    if error>1e-3 or max(direction_error,up_error)>1e-6:raise ValueError('Rendered camera pose differs from JSBSim camera pose')
                    if meta['projection_error_px']>1e-4:raise ValueError('Camera reprojection check failed')
                    image=Image.open(io.BytesIO(body));gray=np.array(image.convert('L'))
                    rgb=np.array(image.convert('RGB'))
                    background=np.all(rgb==[139,177,194],axis=2)
                    background_fraction=float(background.mean())
                    # Banking may legitimately put the sky in view. Reject missing
                    # ground coverage, while retaining real above-horizon rays.
                    focal=JOB['config']['camera']['width']/(2*np.tan(np.deg2rad(JOB['config']['camera']['horizontal_fov_deg'])/2))
                    yy,xx=np.nonzero(background)
                    rays=np.column_stack(((xx-(rgb.shape[1]-1)/2)/focal,(yy-(rgb.shape[0]-1)/2)/focal,np.ones(len(xx))))
                    down=r_ew[:,2]@r_ec
                    missing_ground_fraction=float(np.count_nonzero(rays@down>.03*np.linalg.norm(rays,axis=1))/gray.size)
                    if missing_ground_fraction>.001:raise ValueError('Map is missing below-horizon camera coverage')
                    camera=JOB['config']['camera']
                    if image.size!=(camera['width'],camera['height']):raise ValueError('Invalid image dimensions')
                    corners=cv2.goodFeaturesToTrack(gray,maxCorners=1000,qualityLevel=.01,minDistance=7)
                    features=0 if corners is None else len(corners)
                    fast20=len(cv2.FastFeatureDetector_create(threshold=20,nonmaxSuppression=True).detect(gray))
                    dataset=Path(JOB['dataset'])
                    # Mono PNGs avoid retaining redundant color for the mono VIO benchmark.
                    encoded=io.BytesIO();image.convert('L').save(encoded,format='PNG',compress_level=1)
                    image_bytes=encoded.getvalue()
                    if shutil.disk_usage(dataset).free<512*1024**2:raise ValueError('Less than 512MiB free; recording paused safely')
                    target=dataset/'mav0/cam0/data'/f'{timestamp}.png'
                    target.with_suffix('.tmp').write_bytes(image_bytes);target.with_suffix('.tmp').replace(target)
                    row={'t_ns':timestamp,'wall_time_ns':time.time_ns(),'pose_error_m':error,
                        'direction_error':direction_error,'up_error':up_error,'projection_error_px':meta['projection_error_px'],
                        'tiles_loaded':True,'visible_tiles':meta['visible_tiles'],'features':features,
                        'fast20':fast20,'sky_fraction':background_fraction,'missing_ground_fraction':missing_ground_fraction,
                        'mean_gray':float(gray.mean()),'std_gray':float(gray.std()),'bytes':len(image_bytes),
                        'image_sha256':hashlib.sha256(image_bytes).hexdigest()}
                    with (dataset/'render_journal.jsonl').open('a') as stream:stream.write(json.dumps(row)+'\n')
                    JOB['rows'].append(row)
                    JOB.update(phase='capturing',captured=index+1,last_t_ns=timestamp,last_features=features)
                    if index+1==JOB['expected_frames']:finalize(dataset)
                    return self.send({'captured':index+1,'features':features,'fast20':fast20,'phase':JOB['phase']})
            raise ValueError('Unknown request')
        except Exception as error:self.send({'error':str(error)},status=400)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--dataset',type=Path)
    parser.add_argument('--aircraft-profile',type=Path)
    args=parser.parse_args()
    if args.aircraft_profile:PROFILE=yaml.safe_load(args.aircraft_profile.read_text())
    if args.dataset:attach_dataset(args.dataset)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'Real 3D map + capture UI: http://127.0.0.1:{args.port}',flush=True)
    server.serve_forever()
