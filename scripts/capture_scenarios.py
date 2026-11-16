#!/usr/bin/env python3
"""Resume real-map acquisition, then export verified ROS bags for a flight suite."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import urllib.request
from playwright.sync_api import sync_playwright
from export_bag import export
from scene_quality import measure

ROOT=Path(__file__).resolve().parents[1]


def request(base,path,body=None):
    req=urllib.request.Request(base+path,data=None if body is None else json.dumps(body).encode(),
                               headers={'Content-Type':'application/json'})
    with urllib.request.urlopen(req,timeout=30) as response:return json.load(response)


def capture_suite(suite,port=8766,version=1,headless=False,render_only=False,names=None):
    suite=suite.resolve()
    if not suite.is_relative_to(ROOT/'datasets'):raise ValueError('Suite must be under datasets/')
    base=f'http://127.0.0.1:{port}'
    log=(suite/'capture_server.log').open('a')
    server=subprocess.Popen([sys.executable,'-u',str(ROOT/'scripts/serve_map.py'),'--port',str(port)],stdout=log,stderr=subprocess.STDOUT,cwd=ROOT)
    summary={'status':'running','scenarios':{}}
    progress=suite/('capture_progress.'+'-'.join(names or ['easy','mid','hard'])+'.private.json')
    def save():progress.write_text(json.dumps(summary,indent=2))
    try:
        for _ in range(100):
            if server.poll() is not None:raise RuntimeError('Map server exited; see capture_server.log')
            try:request(base,'/api/config');break
            except OSError:time.sleep(.2)
        else:raise TimeoutError('Map server did not start')
        with sync_playwright() as p:
            browser=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=headless,
                args=['--no-sandbox','--ignore-gpu-blocklist','--enable-webgl','--window-size=1250,1000',f'--remote-debugging-port={port+458}',
                '--disable-backgrounding-occluded-windows','--disable-renderer-backgrounding','--disable-background-timer-throttling'])
            page=browser.new_page(viewport={'width':1150,'height':920},device_scale_factor=1)
            page.on('pageerror',lambda error:print('BROWSER ERROR',str(error),flush=True))
            for name in names or ['easy','mid','hard']:
                dataset=suite/name
                if not (dataset/'manifest.yaml').exists():continue
                bag=dataset/('sensors.bag' if version==1 else 'sensors_ros2')
                if bag.exists() and bag.with_name(bag.name+'.report.json').exists():
                    summary['scenarios'][name]={'status':'complete','bag':str(bag)};save();continue
                request(base,'/api/attach',{'dataset':str(dataset)})
                page.goto(base,wait_until='domcontentloaded')
                started=time.monotonic();last_print=0;retries=0
                while True:
                    page.wait_for_timeout(1500)
                    status=request(base,'/api/status')
                    public_status={k:v for k,v in status.items() if k!='report'}
                    summary['scenarios'][name]=public_status;save()
                    if time.monotonic()-last_print>30:
                        print('RENDER',page.evaluate('({ready:window.mapReady,tiles:window.tileset?.tilesLoaded,debug:window.captureDebug})'),flush=True)
                        print(json.dumps({'scenario':name,'phase':status['phase'],'frames':status.get('captured'),
                            'expected':status.get('expected_frames'),'wall_s':round(time.monotonic()-started)}),flush=True)
                        last_print=time.monotonic()
                    if status['phase']=='failed':
                        error=status.get('error','Capture failed')
                        if retries<3 and any(word in error.lower() for word in ['tile','timeout','failed to fetch']):
                            retries+=1;print(f'Resuming {name} after transient map failure ({retries}/3): {error}',flush=True)
                            request(base,'/api/attach',{'dataset':str(dataset)})
                            page.reload(wait_until='domcontentloaded');continue
                        raise RuntimeError(error)
                    if status['phase']=='complete':break
                    if time.monotonic()-started>24*3600:raise TimeoutError('Capture exceeded 24h; rerun to resume')
                if not render_only:
                    summary['scenarios'][name]={'status':'exporting_bag'};save()
                    bag_report=export(dataset,bag,version=version)
                    summary['scenarios'][name]={'status':'complete','bag':bag_report};save()
                    print(json.dumps({'scenario':name,'bag_verified':str(bag)}),flush=True)
                quality=measure(dataset)
                summary['scenarios'][name]['scene_quality']={k:v for k,v in quality.items() if k!='samples'}
                save()
            browser.close()
        summary['status']='complete';save()
    except Exception as error:
        summary['status']='failed';summary['error']=str(error);save();raise
    finally:
        server.terminate()
        try:server.wait(timeout=10)
        except subprocess.TimeoutExpired:server.kill();server.wait()
        log.close()
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite',type=Path);parser.add_argument('--port',type=int,default=8766)
    parser.add_argument('--ros-version',type=int,choices=[1,2],default=1)
    parser.add_argument('--headless',action='store_true');parser.add_argument('--render-only',action='store_true')
    parser.add_argument('--scenarios',nargs='+',choices=['easy','mid','hard'])
    args=parser.parse_args()
    print(json.dumps(capture_suite(args.suite,args.port,args.ros_version,args.headless,args.render_only,args.scenarios),indent=2))
