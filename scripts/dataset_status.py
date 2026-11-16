#!/usr/bin/env python3
"""Read acquisition progress from artifacts without assuming a process completed."""
import argparse
import json
from pathlib import Path
import yaml


def status(suite):
    result={}
    for name in ['easy','mid','hard']:
        dataset=suite/name
        if not (dataset/'manifest.yaml').exists():
            result[name]={'status':'flight_pending'};continue
        manifest=yaml.safe_load((dataset/'manifest.yaml').read_text())
        images=list((dataset/'mav0/cam0/data').glob('*.png'))
        acquisition=dataset/'acquisition_report.json'
        rendered=acquisition.exists() and not (dataset/'.render_incomplete').exists()
        bags=list(dataset.glob('sensors*.report.json'))
        verified=[]
        for report in bags:
            data=json.loads(report.read_text())
            if data.get('readback_pass') and Path(data['bag']).exists():verified.append(data)
        phase='complete' if verified else ('images_verified_bag_pending' if rendered else ('rendering' if images else 'images_pending'))
        result[name]={'status':phase,'duration_s':manifest['config']['duration_s'],
            'camera_frames':len(images),'expected_camera_frames':manifest['counts']['camera_poses'],
            'imu_samples':manifest['counts']['imu'],'model_validated':False,'CFD':False,
            'captured_through_s':max((int(p.stem)/1e9 for p in images),default=0),
            'bags':[{'path':r['bag'],'bytes':r['bytes'],'alignment_error_ns':r['maximum_corrected_grid_alignment_error_ns']} for r in verified]}
    for progress in suite.glob('capture_progress*.private.json'):
        try:d=json.loads(progress.read_text())
        except json.JSONDecodeError:continue
        if d.get('status')=='failed':
            for name,v in d.get('scenarios',{}).items():
                if result.get(name,{}).get('status')!='complete' and v.get('phase')!='complete':
                    result[name]['last_error']=d.get('error',v.get('error'))
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('suite',type=Path)
    args=parser.parse_args();print(json.dumps(status(args.suite.resolve()),indent=2))
