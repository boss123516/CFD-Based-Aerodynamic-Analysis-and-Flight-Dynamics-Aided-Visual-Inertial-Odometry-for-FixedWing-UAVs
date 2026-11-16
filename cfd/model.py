"""Aircraft-specific beta mean/uncertainty. Fit using calibration flights only."""
from pathlib import Path
import numpy as np
from scipy.interpolate import RegularGridInterpolator
import yaml


class BetaLUT:
    def __init__(self,directory,allow_demo=False):
        directory = Path(directory)
        self.metadata = yaml.safe_load((directory/'metadata.yaml').read_text())
        if self.metadata.get('demo_only',True) and not allow_demo:
            raise ValueError('Demo LUT cannot be used for a research run')
        for field in ['aircraft_id','provenance','calibration_split']:
            if not self.metadata.get(field):
                raise ValueError(f'Missing CFD model metadata: {field}')
        table = np.atleast_1d(np.genfromtxt(directory/'beta_lut.csv',delimiter=',',names=True))
        if table.dtype.names != ('bank_rad','airspeed_mps','beta_ref_rad','sigma_beta_rad'):
            raise ValueError('Unexpected beta LUT columns')
        bank,speed = np.unique(table['bank_rad']),np.unique(table['airspeed_mps'])
        if len(bank)<2 or len(speed)<2 or len(table)!=len(bank)*len(speed):
            raise ValueError('LUT requires a complete bank x speed grid')
        values = np.full((len(bank),len(speed),2),np.nan)
        for row in table:
            i,j = np.searchsorted(bank,row['bank_rad']),np.searchsorted(speed,row['airspeed_mps'])
            if np.isfinite(values[i,j]).any():
                raise ValueError('Duplicate LUT cell')
            values[i,j] = [row['beta_ref_rad'],row['sigma_beta_rad']]
        if not np.isfinite(values).all() or (values[:,:,1]<=0).any():
            raise ValueError('LUT values must be finite; sigma must be positive')
        self.interpolate = RegularGridInterpolator((bank,speed),values,bounds_error=False,fill_value=np.nan)
        self.floor = float(self.metadata.get('sigma_floor_rad',.01))
        if self.floor<=0:
            raise ValueError('sigma_floor_rad must be positive')

    def query(self,bank_rad,estimated_airspeed_mps):
        mean,sigma = self.interpolate([[bank_rad,estimated_airspeed_mps]])[0]
        if not np.isfinite([mean,sigma]).all():
            return None  # caller uses conservative B fallback
        return float(mean),max(float(sigma),self.floor)
