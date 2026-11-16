"""Deterministic 3D point-scene images for plumbing; not Unreal imagery."""
import json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
import yaml
from frames import matrix_wxyz


def render(dataset):
    destination = dataset/'mav0/cam0/data'
    if any(destination.iterdir()):
        raise ValueError('Preserve existing images; choose a fresh dataset')
    marker = dataset/'.render_incomplete';marker.touch()
    poses = np.atleast_2d(np.loadtxt(dataset/'camera_poses.csv',delimiter=',',skiprows=1))
    calibration = yaml.safe_load((dataset/'calibration.yaml').read_text())
    width,height = calibration['resolution']
    fx,fy,cx,cy = calibration['intrinsics']
    rng = np.random.default_rng(2027)
    points = rng.uniform([-800,-800,270],[3500,3500,300],size=(18000,3))
    colors = rng.integers(70,250,size=(len(points),3))
    for row in poses:
        local = (points-row[1:4]) @ matrix_wxyz(row[4:8])
        valid = local[:,2]>1
        uv = local[valid,:2]/local[valid,2:3]
        uv = uv*[fx,fy]+[cx,cy]
        color = colors[valid]
        valid = (uv[:,0]>=2)&(uv[:,0]<width-2)&(uv[:,1]>=2)&(uv[:,1]<height-2)
        image = Image.new('RGB',(width,height),(25,30,25));draw = ImageDraw.Draw(image)
        for (u,v),c in zip(uv[valid],color[valid]):
            draw.rectangle([int(u)-1,int(v)-1,int(u)+1,int(v)+1],fill=tuple(map(int,c)))
        image.save(destination/f'{int(row[0])}.png')
    manifest = yaml.safe_load((dataset/'manifest.yaml').read_text())
    manifest['status'] = 'synthetic_point_images_plumbing_demo'
    manifest['renderer'] = {'backend':'synthetic_3D_points_NOT_UNREAL','seed':2027,'research_ready':False}
    (dataset/'manifest.yaml').write_text(yaml.safe_dump(manifest,sort_keys=False))
    marker.unlink()
