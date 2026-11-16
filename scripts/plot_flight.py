#!/usr/bin/env python3
import argparse
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('dataset',type=Path)
args = parser.parse_args()
d = np.genfromtxt(args.dataset/'flight_debug.csv',delimiter=',',names=True)
g = np.genfromtxt(args.dataset/'groundtruth.csv',delimiter=',',names=True)
t = d['t_ns']/1e9
fig,ax = plt.subplots(3,2,figsize=(12,10),layout='constrained')
ax[0,0].plot(g['py'],g['px']); ax[0,0].set(xlabel='East [m]',ylabel='North [m]',title='JSBSim trajectory')
ax[0,0].axis('equal')
for a,key,label in [(ax[0,1],'alt_h_sl_m','Altitude h-sl [m]'),(ax[1,0],'airspeed_mps','Airspeed [m/s]')]:
    a.plot(t,d[key]);a.set(xlabel='Simulation time [s]',ylabel=label)
ax[1,1].plot(t,np.rad2deg(d['roll_rad']),label='roll')
ax[1,1].plot(t,np.rad2deg(d['bank_command_rad']),'--',label='command')
ax[1,1].set(xlabel='Simulation time [s]',ylabel='Roll [deg]');ax[1,1].legend()
for key in ['pitch_rad','alpha_rad','beta_rad']:
    ax[2,0].plot(t,np.rad2deg(d[key]),label=key.removesuffix('_rad'))
ax[2,0].set(xlabel='Simulation time [s]',ylabel='Angle [deg]');ax[2,0].legend()
for key in ['aileron_cmd','elevator_cmd','rudder_cmd','throttle_cmd']:
    ax[2,1].plot(t,d[key],label=key.removesuffix('_cmd'))
ax[2,1].set(xlabel='Simulation time [s]',ylabel='Normalized control');ax[2,1].legend()
for a in ax.flat:a.grid(alpha=.25)
fig.suptitle('KRoC integration smoke: C172p, no CFD coefficients, no wind')
fig.savefig(args.dataset/'flight_overview.png',dpi=150)
print(args.dataset/'flight_overview.png')
