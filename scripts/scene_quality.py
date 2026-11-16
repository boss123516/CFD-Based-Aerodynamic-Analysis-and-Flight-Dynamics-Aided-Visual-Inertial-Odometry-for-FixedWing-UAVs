#!/usr/bin/env python3
"""Measure visual difficulty on adjacent real rendered frames with fixed settings."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import yaml


def measure(dataset,samples=120):
    with (dataset/'mav0/cam0/data.csv').open() as stream:
        rows=list(csv.reader(stream))[1:]
    available=[i for i in range(len(rows)-1) if all((dataset/'mav0/cam0/data'/rows[j][1]).is_file() for j in [i,i+1])]
    if not available:raise ValueError('No adjacent captured frames')
    indices=np.unique(np.linspace(0,len(available)-1,min(samples,len(available))).astype(int))
    fast=cv2.FastFeatureDetector_create(threshold=20,nonmaxSuppression=True)
    results=[]
    for index in indices:
        i=available[index]
        a=cv2.imread(str(dataset/'mav0/cam0/data'/rows[i][1]),cv2.IMREAD_GRAYSCALE)
        b=cv2.imread(str(dataset/'mav0/cam0/data'/rows[i+1][1]),cv2.IMREAD_GRAYSCALE)
        corners=cv2.goodFeaturesToTrack(a,maxCorners=1000,qualityLevel=.01,minDistance=7)
        points=0 if corners is None else len(corners)
        survival=None;covered=0
        if points:
            xy=corners[:,0,:];h,w=a.shape
            cells=np.floor(xy/[w/8,h/6]).astype(int)
            covered=len(set(map(tuple,cells)))/48
            forward,valid,_=cv2.calcOpticalFlowPyrLK(a,b,corners,None,winSize=(21,21),maxLevel=3)
            backward,valid_back,_=cv2.calcOpticalFlowPyrLK(b,a,forward,None,winSize=(21,21),maxLevel=3)
            inside=(forward[:,0,0]>=0)&(forward[:,0,0]<w)&(forward[:,0,1]>=0)&(forward[:,0,1]<h)
            good=(valid.ravel()>0)&(valid_back.ravel()>0)&inside&(np.linalg.norm(backward[:,0,:]-corners[:,0,:],axis=1)<1.)
            survival=float(good.mean())
        results.append({'capture_t_ns':int(rows[i][0]),'fast20_count':len(fast.detect(a)),
            'shi_tomasi_count_capped_1000':points,'grid_coverage_8x6':covered,
            'klt_forward_backward_survival':survival,'gray_std':float(a.std())})
    survival=[r['klt_forward_backward_survival'] for r in results if r['klt_forward_backward_survival'] is not None]
    manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
    report={'map_id':manifest['config'].get('map_id','strasbourg_city'),
        'sampled_adjacent_pairs':len(results),'complete_sequence_sampled':len(available)==len(rows)-1,
        'fast20_median':float(np.median([r['fast20_count'] for r in results])),
        'shi_tomasi_median_capped_1000':float(np.median([r['shi_tomasi_count_capped_1000'] for r in results])),
        'grid_coverage_median':float(np.median([r['grid_coverage_8x6'] for r in results])),
        'klt_survival_median':float(np.median(survival)) if survival else None,
        'fast20_under_50_fraction':float(np.mean([r['fast20_count']<50 for r in results])),
        'parameters':{'FAST_threshold':20,'ShiTomasi_qualityLevel':.01,'minDistance_px':7,
            'KLT_win_px':21,'KLT_levels':3,'forward_backward_error_px':1},
        'note':'Low feature count is expected and retained. This report is not an MSCKF/MSCEqF accuracy result.',
        'samples':results}
    (dataset/'scene_quality.json').write_text(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('datasets',nargs='+',type=Path)
    p.add_argument('--samples',type=int,default=120);args=p.parse_args()
    for path in args.datasets:
        r=measure(path,args.samples);print(json.dumps({k:v for k,v in r.items() if k!='samples'},indent=2))
