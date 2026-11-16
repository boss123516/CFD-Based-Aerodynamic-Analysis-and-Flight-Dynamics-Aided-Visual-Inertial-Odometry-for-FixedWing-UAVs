#!/usr/bin/env python3
"""Shared dataset -> A/B/C estimator runners -> metrics. Explicit demo mode."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from cfd.model import BetaLUT
from estimators import demo as demo_backend
from estimators.baseline import adapter
from evaluation.metrics import compare
from generate_flight import generate
from render_synthetic import render as render_synthetic
from validate_dataset import validate


def digest_tree(path):
    return {str(p.relative_to(path)):hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(path.rglob('*')) if p.is_file()}


def stage_input(dataset,destination):
    destination.mkdir()
    # Copy only online sensor inputs. GT/debug/poses/manifest never go to runners.
    for name in ['cam0','imu0']:
        shutil.copytree(dataset/'mav0'/name,destination/'mav0'/name)
    shutil.copy2(dataset/'calibration.yaml',destination/'calibration.yaml')


def run_methods(dataset,output,methods_file,demo=False):
    checks = validate(dataset,require_images=True)
    if not checks['flight_contract_pass']:
        raise ValueError('Dataset validation failed; see validation_report.json')
    manifest = yaml.safe_load((dataset/'manifest.yaml').read_text())
    if not demo and manifest.get('renderer',{}).get('backend','').startswith('synthetic'):
        raise ValueError('Synthetic point images require explicit demo mode')
    methods = yaml.safe_load(methods_file.read_text())
    if set(methods)!={'A','B','C'}:
        raise ValueError('Expected exactly A, B, C method configurations')
    expected = {'A':(False,False),'B':(True,False),'C':(True,True)}
    for name,method in methods.items():
        if (method['dynamics']['enabled'],method['cfd']['enabled'])!=expected[name]:
            raise ValueError(f'{name} flags do not match the A/B/C experiment')
    (output/'methods.snapshot.yaml').write_text(yaml.safe_dump(methods,sort_keys=False))
    shared = output/'sensor_input'
    stage_input(dataset,shared)
    hashes = digest_tree(shared)
    (output/'input.sha256.json').write_text(json.dumps(hashes,indent=2))
    statuses = {}
    for name in ['A','B','C']:
        method = dict(methods[name]);destination = output/name;destination.mkdir()
        statuses[name] = {'label':method['label'],'status':'running','demo_only':demo}
        try:
            if digest_tree(shared)!=hashes:
                raise ValueError('Shared input changed between methods')
            if method['cfd']['enabled'] and not demo:
                model = method['cfd'].get('model')
                if not model:
                    statuses[name]['status'] = 'blocked'
                    raise ValueError('C requires a calibrated CFD-informed LUT; set cfd.model')
                model = (methods_file.parent/Path(model)).resolve()
                BetaLUT(model)  # reject toy model for real experiment
                method['cfd'] = {**method['cfd'],'model':str(model)}
            (destination/'method.yaml').write_text(yaml.safe_dump(method,sort_keys=False))
            backend = demo_backend if demo else adapter
            backend.run(shared,destination,method,ROOT)
            if digest_tree(shared)!=hashes:
                raise ValueError('Estimator modified shared sensor input')
            statuses[name]['status'] = 'complete'
        except Exception as error:
            if statuses[name]['status']!='blocked':statuses[name]['status']='failed'
            statuses[name]['error'] = str(error)
        (destination/'status.json').write_text(json.dumps(statuses[name],indent=2))
        print(f"{name}: {statuses[name]['status']}",flush=True)
    report = compare(dataset,output,statuses,demo)
    (output/'.pipeline_incomplete').unlink()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['demo','run','evaluate'])
    parser.add_argument('--dataset',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--flight-config',type=Path,default=ROOT/'config/straight_turn.yaml')
    parser.add_argument('--methods',type=Path,default=ROOT/'config/methods.yaml')
    args = parser.parse_args()
    output = args.output.resolve()
    if args.mode=='evaluate':
        if args.dataset is None:parser.error('--dataset required')
        statuses = {name:json.loads((output/name/'status.json').read_text()) for name in ['A','B','C']}
        demo = all(status.get('demo_only') for status in statuses.values())
        report = compare(args.dataset.resolve(),output,statuses,demo)
    else:
        output.mkdir(parents=True,exist_ok=False)
        (output/'.pipeline_incomplete').touch()
        dataset = args.dataset.resolve() if args.dataset else output/'dataset'
        if args.mode=='demo' and args.dataset is None:
            generate(yaml.safe_load(args.flight_config.read_text()),dataset)
            render_synthetic(dataset)
        elif args.dataset is None:
            parser.error('run requires --dataset; generate/render once first')
        report = run_methods(dataset,output,args.methods.resolve(),demo=args.mode=='demo')
    print(json.dumps({'output':str(output),'scientific_result':report['scientific_result'],
                      'statuses':{k:v['status'] for k,v in report['methods'].items()}},indent=2))
    return 0 if all(m['status']=='complete' for m in report['methods'].values()) else 1


if __name__=='__main__':raise SystemExit(main())
