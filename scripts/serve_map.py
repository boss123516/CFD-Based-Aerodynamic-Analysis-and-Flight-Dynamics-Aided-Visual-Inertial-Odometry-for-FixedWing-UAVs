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

ROOT=Path(__file__).resolve().parents[1]
MAP=yaml.safe_load((ROOT/'config/map.yaml').read_text())
DEFAULT=yaml.safe_load((ROOT/'config/strasbourg.yaml').read_text())
CACHE=ROOT/'external/map_cache/strasbourg_2022';CACHE.mkdir(parents=True,exist_ok=True)
LOCK=threading.RLock()
JOB={'phase':'idle','captured':0}
CACHE_INDEX={}


def ecef(lat,lon,h):
    a=6378137.;e2=6.6943799901413165e-3
    N=a/np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([(N+h)*np.cos(lat)*np.cos(lon),(N+h)*np.cos(lat)*np.sin(lon),(N*(1-e2)+h)*np.sin(lat)])


def start_job(params):
    global JOB
    with LOCK:
        if JOB['phase'] in ['generating','capturing','flight_ready']:raise ValueError('Capture already running')
        config=yaml.safe_load(yaml.safe_dump(DEFAULT))
        duration=float(params.get('duration_s',config['duration_s']))
        if not .1<=duration<=120 or not np.isclose(duration*10,round(duration*10)):raise ValueError('Duration: 0.1..120s on a 10Hz boundary')
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
    manifest['renderer']={'backend':'cesiumjs_photogrammetry','version':'1.124.0','map':MAP,
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
            if path=='/api/config':return self.send({'map':MAP,'flight':DEFAULT})
            if path=='/api/status':
                with LOCK:return self.send({k:v for k,v in JOB.items() if k not in ['poses','rows','config']})
            if path=='/api/poses':
                with LOCK:
                    if 'poses' not in JOB:raise ValueError('Flight not ready')
                    return self.send({k:JOB[k] for k in ['name','poses','origin','config']})
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
        except Exception as error:self.send({'error':str(error)},status=500)

    def do_POST(self):
        try:
            size=int(self.headers.get('Content-Length',0))
            if size>8_000_000:raise ValueError('Request too large')
            body=self.rfile.read(size)
            if self.path=='/api/start':return self.send(start_job(json.loads(body or '{}')))
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
                    if image.size!=(640,480) or gray.std()<3:raise ValueError('Invalid image dimensions or texture')
                    corners=cv2.goodFeaturesToTrack(gray,maxCorners=1000,qualityLevel=.01,minDistance=7)
                    features=0 if corners is None else len(corners)
                    dataset=Path(JOB['dataset']);(dataset/'mav0/cam0/data'/f'{timestamp}.png').write_bytes(body)
                    JOB['rows'].append({'t_ns':timestamp,'wall_time_ns':time.time_ns(),'pose_error_m':error,
                        'direction_error':direction_error,'up_error':up_error,'projection_error_px':meta['projection_error_px'],
                        'tiles_loaded':True,'visible_tiles':meta['visible_tiles'],'features':features,
                        'mean_gray':float(gray.mean()),'std_gray':float(gray.std()),'bytes':len(body)})
                    JOB.update(phase='capturing',captured=index+1,last_t_ns=timestamp,last_features=features)
                    if index+1==JOB['expected_frames']:finalize(dataset)
                    return self.send({'captured':index+1,'features':features,'phase':JOB['phase']})
            raise ValueError('Unknown request')
        except Exception as error:self.send({'error':str(error)},status=400)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--port',type=int,default=8765)
    args=parser.parse_args()
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'Real 3D map + capture UI: http://127.0.0.1:{args.port}',flush=True)
    server.serve_forever()
