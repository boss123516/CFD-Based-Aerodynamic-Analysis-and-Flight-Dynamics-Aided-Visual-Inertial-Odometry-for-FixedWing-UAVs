#!/usr/bin/env python3
"""Make three real-time 30s GIF excerpts without waiting for complete acquisition.

Existing easy frames are reused; mid/hard views use the same JSBSim camera
poses and real Cesium mesh. Presentation frames never enter the sensor bag.
Only the GIFs and a small non-aircraft manifest go into the public assets.
Requires ffmpeg on PATH and the map renderer dependencies.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import urllib.request

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright
import yaml
from frames import ecef_to_ned

ROOT=Path(__file__).resolve().parents[1]


def post(base,path,data):
    request=urllib.request.Request(base+path,data=json.dumps(data).encode(),headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(request,timeout=30) as response:return json.load(response)


def active_window(dataset):
    debug=np.loadtxt(dataset/'flight_debug.csv',delimiter=',',skiprows=1,usecols=[0,3,4,16,17,18])[::20]
    best=(-np.inf,0)
    for start in range(10,int(debug[-1,0]/1e9)-30):
        window=debug[(debug[:,0]>=start*1e9)&(debug[:,0]<(start+30)*1e9)]
        score=np.ptp(window[:,1])+.5*np.ptp(window[:,2])+.8*np.sqrt(np.mean(np.sum(window[:,3:]**2,axis=1)))
        if score>best[0]:best=score,start
    return best[1]


def encode(frames,destination):
    # GIF previews retain a 30s wall duration and 1x flight speed. Palette and
    # resolution are kept small enough for three GitHub README images.
    for colors in (32,16):
        filters=f'[0:v]fps=6,scale=280:-2:flags=lanczos,split[a][b];[a]palettegen=max_colors={colors}:reserve_transparent=0[p];[b][p]paletteuse=dither=none'
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-filter_complex_threads','1','-framerate','10',
            '-i',str(frames/'%06d.png'),'-filter_complex',filters,'-loop','0',str(destination)],check=True)
        if destination.stat().st_size<8_000_000:break
    with Image.open(destination) as gif:
        if gif.info.get('loop')!=0:raise ValueError('Expected an infinitely looping GIF')
        total_ms=0
        for index in range(gif.n_frames):
            gif.seek(index);total_ms+=gif.info['duration']
        if abs(total_ms-30000)>10:raise ValueError('GIF does not have 30s duration')
        return {'duration_s':total_ms/1000,'frames':gif.n_frames,'resolution':list(gif.size),
                'bytes':destination.stat().st_size,'sha256':hashlib.sha256(destination.read_bytes()).hexdigest()}


def run(suite,port):
    output=ROOT/'assets/sim';output.mkdir(exist_ok=True)
    private=suite/'presentation_previews';private.mkdir(exist_ok=True)
    records={};base=f'http://127.0.0.1:{port}'
    with (private/'server.log').open('a') as log:
        server=subprocess.Popen([sys.executable,'-u',str(ROOT/'scripts/serve_map.py'),'--port',str(port)],stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
        try:
            for _ in range(100):
                if server.poll() is not None:raise RuntimeError('Preview server did not start')
                try:
                    with urllib.request.urlopen(base+'/api/config',timeout=1):break
                except OSError:time.sleep(.2)
            with sync_playwright() as playwright:
                browser=playwright.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,
                    args=['--no-sandbox','--enable-gpu','--ignore-gpu-blocklist','--enable-webgl',
                        '--disable-backgrounding-occluded-windows','--disable-renderer-backgrounding','--disable-background-timer-throttling'])
                page=browser.new_page(viewport={'width':1150,'height':920},device_scale_factor=1)
                page.set_default_navigation_timeout(120000)
                page.on('console',lambda message:print('BROWSER:',message.text,flush=True) if message.type=='error' else None)
                print('GPU renderer:',page.evaluate('''()=>{const g=document.createElement('canvas').getContext('webgl2');const e=g.getExtension('WEBGL_debug_renderer_info');return g.getParameter(e.UNMASKED_RENDERER_WEBGL);}'''),flush=True)
                for name in ['easy','mid','hard']:
                    dataset=suite/name
                    start=20 if name=='easy' else active_window(dataset)
                    manifest=yaml.safe_load((dataset/'manifest.yaml').read_text());camera=manifest['config']['camera']
                    poses=np.loadtxt(dataset/'camera_poses.csv',delimiter=',',skiprows=1)
                    chosen=poses[(poses[:,0]>=start*1e9)&(poses[:,0]<(start+30)*1e9)][::2]
                    if len(chosen)!=300 or not np.all(np.diff(chosen[:,0])==100_000_000):raise ValueError('Expected 300 frames at 10Hz')
                    folder=private/name;folder.mkdir(exist_ok=True)
                    qc_path=folder/'render_qc.private.jsonl'
                    previous={}
                    if qc_path.exists():previous={r['index']:r for r in map(json.loads,qc_path.read_text().splitlines())}
                    if name!='easy':
                        post(base,'/api/attach',{'dataset':str(dataset)})
                        page.goto(base+'/?preview=1',wait_until='domcontentloaded')
                        page.wait_for_function('window.mapReady===true',timeout=240000)
                    print(json.dumps({'scenario':name,'start_s':start,'end_s':start+30}),flush=True)
                    for index,row in enumerate(chosen):
                        file=folder/f'{index:06d}.png'
                        prior=previous.get(index)
                        if prior and prior['capture_t_ns']==int(row[0]) and file.exists() and prior['sha256']==hashlib.sha256(file.read_bytes()).hexdigest():continue
                        if name=='easy':
                            source=dataset/'mav0/cam0/data'/f'{int(row[0])}.png'
                            with Image.open(source) as image:image.convert('L').save(file)
                            qc={'source':'verified sensor capture PNG'}
                        else:
                            for attempt in range(4):
                                try:
                                    result=page.evaluate('''async args=>{
                                      window.captureDebug.capturing=true;
                                      const pose=setPose(args.row,args.origin,args.camera);await waitTiles();
                                      return {png:viewer.scene.canvas.toDataURL('image/png'),
                                        projection_error_px:reprojection(pose),tiles_loaded:tileset.tilesLoaded,
                                        position_ecef:[viewer.camera.positionWC.x,viewer.camera.positionWC.y,viewer.camera.positionWC.z]};
                                    }''',{'row':row.tolist(),'origin':manifest['origin'],'camera':camera})
                                    break
                                except Exception:
                                    if attempt==3:raise
                                    page.reload(wait_until='domcontentloaded');page.wait_for_function('window.mapReady===true',timeout=240000)
                            raw=base64.b64decode(result.pop('png').split(',')[1])
                            import io
                            with Image.open(io.BytesIO(raw)) as image:
                                if image.size!=(camera['width'],camera['height']):raise ValueError('Wrong render size')
                                image.convert('L').save(file)
                            origin=manifest['origin'];lat,lon=origin['latitude_rad'],origin['longitude_rad'];h=origin['ellipsoid_height_m']
                            a=6378137.;e2=6.6943799901413165e-3;n=a/np.sqrt(1-e2*np.sin(lat)**2)
                            p0=np.array([(n+h)*np.cos(lat)*np.cos(lon),(n+h)*np.cos(lat)*np.sin(lon),(n*(1-e2)+h)*np.sin(lat)])
                            expected=p0+ecef_to_ned(lat,lon).T@row[1:4]
                            error=float(np.linalg.norm(expected-result.pop('position_ecef')))
                            if error>1e-3 or not result['tiles_loaded'] or result['projection_error_px']>1e-4:raise ValueError('Preview rendering contract failed')
                            qc={**result,'position_error_m':error}
                        qc.update(index=index,capture_t_ns=int(row[0]),sha256=hashlib.sha256(file.read_bytes()).hexdigest())
                        with qc_path.open('a') as stream:stream.write(json.dumps(qc)+'\n')
                        if index%30==0:print(json.dumps({'scenario':name,'frames':index+1,'expected':300}),flush=True)
                    result=encode(folder,output/f'{name}-30s.gif')
                    records[name]={'start_s':start,'end_s':start+30,'playback_speed':1,**result}
                    print(json.dumps({'scenario':name,'gif':records[name]}),flush=True)
                    # Preview rendering does not record sensor frames or finish a dataset.
                    if name!='easy':post(base,'/api/fail',{'error':'Presentation excerpt finished; full acquisition remains separate'})
                browser.close()
        finally:
            server.terminate()
            try:server.wait(timeout=10)
            except subprocess.TimeoutExpired:server.kill();server.wait()
    (output/'previews.json').write_text(json.dumps({'description':'30-second excerpts from 600-second JSBSim scenarios, 1x playback; not VIO/CFD accuracy results',
        'map':'Strasbourg 2022 public photogrammetry','scenarios':records},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('suite',type=Path)
    parser.add_argument('--port',type=int,default=8768);args=parser.parse_args();run(args.suite.resolve(),args.port)
