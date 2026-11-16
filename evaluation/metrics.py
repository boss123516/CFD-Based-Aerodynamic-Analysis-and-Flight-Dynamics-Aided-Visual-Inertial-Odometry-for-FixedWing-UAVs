"""One initial yaw/translation alignment; no scale fitting, no segment refits."""
import csv
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def read_trajectory(path):
    with Path(path).open() as stream:
        reader = csv.DictReader(stream)
        required = ['t_ns','px','py','pz','qw','qx','qy','qz']
        if not set(required)<=set(reader.fieldnames or []):
            raise ValueError(f'{path}: expected columns {required}')
        rows = [[float(row[k]) for k in required] for row in reader]
    a = np.asarray(rows,dtype=float).reshape(-1,8)
    if len(a)<2 or not np.isfinite(a[:,0]).all() or (np.diff(a[:,0])<=0).any():
        raise ValueError('Need >=2 strictly increasing timestamps')
    if not np.isfinite(a).all() or not np.allclose(np.linalg.norm(a[:,4:8],axis=1),1,atol=1e-3):
        raise ValueError('Nonfinite poses or non-unit quaternions')
    return a


def sample(trajectory,times,max_gap_ns=150_000_000):
    stamps = trajectory[:,0]
    right = np.searchsorted(stamps,times)
    exact = (right<len(stamps)) & (stamps[np.minimum(right,len(stamps)-1)]==times)
    hi = np.clip(right,1,len(stamps)-1);lo = hi-1
    valid = exact | ((times>=stamps[0]) & (times<=stamps[-1]) & ((stamps[hi]-stamps[lo])<=max_gap_ns))
    position = np.full((len(times),3),np.nan)
    rotation = np.tile(np.eye(3),(len(times),1,1))
    if valid.any():
        ts = times[valid]
        position[valid] = np.column_stack([np.interp(ts,stamps,trajectory[:,k]) for k in [1,2,3]])
        rotation[valid] = Slerp((stamps-stamps[0])/1e9,
                              Rotation.from_quat(trajectory[:,[5,6,7,4]]))((ts-stamps[0])/1e9).as_matrix()
    return position,rotation,valid


def evaluate(gt,estimate,times,rpe_interval_s=1.):
    gp,gr,gv = sample(gt,times)
    ep,er,ev = sample(estimate,times)
    valid = gv & ev
    result = {'valid_output_rate':float(valid.mean()),'valid_poses':int(valid.sum()),'expected_poses':len(times),
              'alignment':'one initial common pose: translation+yaw, no scale','rpe_interval_s':rpe_interval_s,
              'ate_rmse_m':None,'rpe_translation_rmse_m':None,'rpe_rotation_rmse_deg':None,
              'rpe_translation_p95_m':None,'rpe_rotation_p95_deg':None,'rpe_pairs':0}
    if not valid.any():
        return result,None
    first = np.flatnonzero(valid)[0]
    yaw = lambda r: np.arctan2(r[1,0],r[0,0])
    align = Rotation.from_euler('z',yaw(gr[first])-yaw(er[first])).as_matrix()
    ep = (ep-ep[first]) @ align.T + gp[first]
    er = align @ er
    result['alignment_t_ns'] = int(times[first])
    error = np.linalg.norm(ep[valid]-gp[valid],axis=1)
    result['ate_rmse_m'] = float(np.sqrt(np.mean(error**2)))
    pairs = []
    interval = round(rpe_interval_s*1e9)
    for i,t in enumerate(times):
        j = int(np.searchsorted(times,t+interval))
        if j<len(times) and times[j]==t+interval and valid[i] and valid[j]:
            # Avoid bridging a tracking outage even when both endpoints exist.
            if not valid[i:j+1].all():
                continue
            dp_g = gr[i].T @ (gp[j]-gp[i]);dp_e = er[i].T @ (ep[j]-ep[i])
            dr_g = gr[i].T @ gr[j];dr_e = er[i].T @ er[j]
            pairs.append([np.linalg.norm(dp_g-dp_e),np.rad2deg(Rotation.from_matrix(dr_g.T @ dr_e).magnitude())])
    if pairs:
        pairs = np.array(pairs)
        result.update(rpe_translation_rmse_m=float(np.sqrt(np.mean(pairs[:,0]**2))),
                      rpe_rotation_rmse_deg=float(np.sqrt(np.mean(pairs[:,1]**2))),
                      rpe_translation_p95_m=float(np.percentile(pairs[:,0],95)),
                      rpe_rotation_p95_deg=float(np.percentile(pairs[:,1],95)),rpe_pairs=len(pairs))
    return result,{'time_ns':times[valid],'position':ep[valid],'gt_position':gp[valid]}


def compare(dataset,run_root,statuses,demo=False):
    gt = read_trajectory(dataset/'groundtruth.csv')
    times = np.loadtxt(dataset/'camera_poses.csv',delimiter=',',skiprows=1,usecols=0).astype(np.int64)
    report = {'scientific_result':False if demo else None,'backend':'plumbing_demo_NOT_VIO' if demo else 'external',
              'shared_dataset':str(dataset),'methods':{}}
    curves = {}
    for name,status in statuses.items():
        entry = {**status,'valid_output_rate':0.,'valid_poses':0,'expected_poses':len(times)}
        if status['status']=='complete':
            try:
                metrics,curve = evaluate(gt,read_trajectory(run_root/name/'trajectory.csv'),times)
                entry.update(metrics)
                if curve is not None:curves[name] = curve
            except (ValueError,OSError) as error:
                entry.update(status='failed',error=str(error))
        report['methods'][name] = entry
    (run_root/'comparison.json').write_text(json.dumps(report,indent=2))
    keys = ['method','label','status','valid_output_rate','valid_poses','expected_poses',
            'ate_rmse_m','rpe_translation_rmse_m','rpe_rotation_rmse_deg','rpe_pairs','error']
    with (run_root/'comparison.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=keys,extrasaction='ignore');writer.writeheader()
        for name,entry in report['methods'].items():writer.writerow({'method':name,**entry})
    if curves:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,ax = plt.subplots(figsize=(7,5),layout='constrained')
        ax.plot(gt[:,2],gt[:,1],'k--',label='GT')
        for name,curve in curves.items():ax.plot(curve['position'][:,1],curve['position'][:,0],label=name)
        ax.set(xlabel='East [m]',ylabel='North [m]',title='Plumbing demo — NOT VIO/CFD results' if demo else 'A/B/C comparison')
        ax.axis('equal');ax.legend();ax.grid(alpha=.25)
        fig.savefig(run_root/'trajectories.png',dpi=120);plt.close(fig)
    return report
