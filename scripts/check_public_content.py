#!/usr/bin/env python3
"""Prevent purchased CAD, manufacturing archives and private model folders in Git."""
import argparse
from pathlib import PurePosixPath
import subprocess

PROTECTED_SUFFIXES={'.step','.stp','.stl','.iges','.igs','.3mf','.obj','.ply','.glb','.gltf',
                    '.blend','.fcstd','.f3d','.zip','.7z','.rar','.tar','.tgz'}
PROTECTED_DIRECTORIES=('private/','aircraft_private/','cfd/private/','cfd/models/','datasets/','runs/')
AIRCRAFT_ALLOWED={'sim/aircraft/.gitkeep','sim/aircraft/README.md'}


def prohibited(path):
    normalized=path.lower()
    return (PurePosixPath(normalized).suffix in PROTECTED_SUFFIXES or
            '.private.' in normalized or '/private_model/' in normalized or
            PurePosixPath(normalized).suffix in {'.bag','.db3','.mcap'} or
            any(normalized.startswith(p) for p in PROTECTED_DIRECTORIES) or
            (normalized.startswith('sim/aircraft/') and path not in AIRCRAFT_ALLOWED))


def check(history=False):
    names=set(subprocess.check_output(['git','ls-files','-z']).decode().split('\0'))-{''}
    if history:
        for line in subprocess.check_output(['git','rev-list','--objects','--all']).decode().splitlines():
            if ' ' in line:names.add(line.split(' ',1)[1])
    blocked=sorted(p for p in names if prohibited(p))
    if blocked:
        raise SystemExit('Private aircraft content found in tracked files/history:\n'+'\n'.join(blocked))
    print('Public-content check passed: no CAD, manufacturing archives or private aircraft models tracked.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--history',action='store_true');args=parser.parse_args();check(args.history)
