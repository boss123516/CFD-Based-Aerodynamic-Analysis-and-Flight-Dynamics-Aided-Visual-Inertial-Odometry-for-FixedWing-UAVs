"""Shared external estimator entry point for A/B/C; plug your MSCKF runner here."""
import os
from pathlib import Path
import subprocess
import yaml


def run(sensor_input, output, method, project_root):
    command = method.get('command')
    if not isinstance(command,list) or not command or not all(isinstance(t,str) for t in command):
        raise ValueError('Set command argv in config/methods.yaml to your estimator runner')
    config = output/'method.yaml'
    config.write_text(yaml.safe_dump(method,sort_keys=False))
    fields = {'input':str(sensor_input.resolve()),'output':str(output.resolve()),
              'config':str(config.resolve()),'root':str(project_root)}
    argv = [token.format(**fields) for token in command]
    env = {**os.environ,'KROC_INPUT':fields['input'],'KROC_OUTPUT':fields['output'],
           'KROC_METHOD_CONFIG':fields['config']}
    with (output/'runner.log').open('w') as log:
        subprocess.run(argv,cwd=output,env=env,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=3600)
    if not (output/'trajectory.csv').is_file():
        raise RuntimeError('Runner must write trajectory.csv; see docs/CONTRACTS.md')
