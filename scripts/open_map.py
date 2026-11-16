#!/usr/bin/env python3
"""Keep a visible GPU browser open on the local 3D acquisition UI."""
import argparse
import time
from pathlib import Path
from playwright.sync_api import sync_playwright

parser=argparse.ArgumentParser();parser.add_argument('--url',default='http://127.0.0.1:8765')
parser.add_argument('--record',action='store_true');args=parser.parse_args()
with sync_playwright() as p:
    browser=p.chromium.launch(executable_path='/usr/bin/google-chrome',headless=False,
        args=['--no-sandbox','--ignore-gpu-blocklist','--enable-webgl','--window-size=1250,900','--remote-debugging-port=9223',
              '--disable-backgrounding-occluded-windows','--disable-renderer-backgrounding','--disable-background-timer-throttling'])
    page=browser.new_page(viewport={'width':1150,'height':800},device_scale_factor=1)
    page.on('console',lambda msg:print('BROWSER',msg.type,msg.text,flush=True) if msg.type in ['error','warning'] else None)
    page.on('pageerror',lambda error:print('PAGE ERROR',error,flush=True))
    page.goto(args.url,wait_until='domcontentloaded')
    print('Visible real 3D map browser opened.',flush=True)
    page.wait_for_function('window.mapReady === true',timeout=240000)
    print('3D mesh loaded.',flush=True)
    if args.record:page.click('#record');print('Started JSBSim + real map capture.',flush=True)
    while browser.is_connected() and not page.is_closed():page.wait_for_timeout(1000)
