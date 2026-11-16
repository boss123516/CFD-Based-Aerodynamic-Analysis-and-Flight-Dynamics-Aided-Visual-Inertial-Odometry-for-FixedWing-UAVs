"""JSBSim 1.3.3 wind inputs (SI at boundary) and discrete IMU noise."""
import numpy as np
from frames import FT


def configure_wind(f,config):
    wind = config.get('wind',{})
    mean = np.asarray(wind.get('mean_ned_mps',[0,0,0]),dtype=float)
    if mean.shape!=(3,) or not np.isfinite(mean).all():raise ValueError('Invalid mean_ned_mps')
    for axis,value in zip(['north','east','down'],mean):f[f'atmosphere/wind-{axis}-fps'] = float(value)/FT
    f['simulation/randomseed'] = config['seed']
    f['atmosphere/randomseed'] = wind.get('seed',config['seed']+1)
    turbulence = wind.get('turbulence',{})
    models = {'none':0,'dryden_milspec':3,'dryden_tustin':4}
    model = turbulence.get('model','none')
    if model not in models:raise ValueError('Unknown turbulence model')
    severity = int(turbulence.get('severity',0))
    if not 0<=severity<=7:raise ValueError('Dryden severity must be 0..7')
    f['atmosphere/turb-type'] = models[model]
    f['atmosphere/turbulence/milspec/severity'] = severity
    f['atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps'] = float(turbulence.get('windspeed_20ft_mps',0))/FT
    if model!='none' and np.linalg.norm(mean[:2])<1e-6:
        raise ValueError('Dryden directional test requires nonzero mean horizontal wind')


def gust_ned(t,wind):
    result = np.zeros(3)
    for gust in wind.get('gusts',[]):
        start,rise,hold,fall = [float(gust[k]) for k in ['start_s','rise_s','hold_s','fall_s']]
        if min(rise,fall)<=0 or hold<0:raise ValueError('Invalid gust timing')
        phase=t-start
        amplitude=0.
        if 0<=phase<rise:amplitude=.5*(1-np.cos(np.pi*phase/rise))
        elif rise<=phase<rise+hold:amplitude=1.
        elif rise+hold<=phase<rise+hold+fall:amplitude=.5*(1+np.cos(np.pi*(phase-rise-hold)/fall))
        result += amplitude*np.asarray(gust['ned_mps'],dtype=float)
    return result


def apply_imu_noise(gyro,force,noise,seed,dt):
    rng=np.random.default_rng(seed)
    biases=[]
    for field,walk in [('gyro_bias_rad_s','gyro_bias_random_walk_rad_s_sqrt_s'),
                       ('accel_bias_m_s2','accel_bias_random_walk_m_s2_sqrt_s')]:
        initial=np.asarray(noise.get(field,[0,0,0]),dtype=float)
        sigma=float(noise.get(walk,0))
        if sigma<0:raise ValueError('Random walk sigma must be nonnegative')
        increments=rng.normal(0,sigma*np.sqrt(dt),(len(gyro),3));increments[0]=0
        biases.append(initial+np.cumsum(increments,axis=0))
    gyro_std=float(noise['gyro_white_std_rad_s']);accel_std=float(noise['accel_white_std_m_s2'])
    if min(gyro_std,accel_std)<0:raise ValueError('Noise std must be nonnegative')
    return (gyro+biases[0]+rng.normal(0,gyro_std,gyro.shape),
            force+biases[1]+rng.normal(0,accel_std,force.shape),*biases)
