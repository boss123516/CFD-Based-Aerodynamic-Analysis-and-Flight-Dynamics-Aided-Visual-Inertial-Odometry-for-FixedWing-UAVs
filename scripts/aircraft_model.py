"""Build an explicitly unvalidated JSBSim surrogate from a private SI profile.

No paid geometry or aircraft-specific values are bundled with this module.
Forces are a linear lift curve and parabolic drag polar; moments include
static stability, rate damping and normalized ideal elevon commands.
"""
from pathlib import Path
import numpy as np


def build_model(profile, directory):
    a = profile['aircraft']
    for key in ['mass_kg', 'wingspan_m', 'wing_area_m2', 'chord_m', 'max_thrust_n']:
        if not np.isfinite(a[key]) or a[key] <= 0:
            raise ValueError(f'Invalid {key}')
    inertia = np.array(a['inertia_kg_m2'])
    if inertia.shape != (3,) or (inertia <= 0).any() or max(inertia) > sum(inertia)-max(inertia):
        raise ValueError('Inertia must be positive and satisfy triangle inequality')
    c = a['coefficients']
    v = lambda x: f'<value>{float(x):.14g}</value>'
    p = lambda x: f'<property>{x}</property>'
    product = lambda *x: '<product>'+''.join(x)+'</product>'
    total = lambda *x: '<sum>'+''.join(x)+'</sum>'
    term = lambda k, prop: product(v(c[k]), p(prop))
    cl = total(v(c['CL0']), term('CL_alpha', 'aero/alpha-rad'))
    cd = total(v(c['CD0']), product(v(c['k']), '<pow>'+cl+v(2)+'</pow>'))
    qS = [p('aero/qbar-psf'), p('metrics/Sw-sqft')]
    moments = {
        'ROLL': ('metrics/bw-ft', total(term('Cl_beta','aero/beta-rad'), term('Cl_p','aero/bi2vel'),
                  term('Cl_da','fcs/aileron-cmd-norm'))),
        'PITCH': ('metrics/cbarw-ft', total(v(c['Cm0']), term('Cm_alpha','aero/alpha-rad'),
                  product(v(c['Cm_q']),p('aero/ci2vel'),p('velocities/q-rad_sec')),
                  term('Cm_de','fcs/elevator-cmd-norm'))),
        'YAW': ('metrics/bw-ft', total(term('Cn_beta','aero/beta-rad'),
                product(v(c['Cn_r']),p('aero/bi2vel'),p('velocities/r-rad_sec')))),
    }
    # Roll damping needs p*b/(2V), not just the nondimensional length factor.
    moments['ROLL'] = ('metrics/bw-ft', total(term('Cl_beta','aero/beta-rad'),
        product(v(c['Cl_p']),p('aero/bi2vel'),p('velocities/p-rad_sec')),
        term('Cl_da','fcs/aileron-cmd-norm')))
    axes = {'LIFT': product(*qS,cl), 'DRAG': product(*qS,cd),
            'SIDE': product(*qS,term('CY_beta','aero/beta-rad'))}
    axes.update({axis:product(*qS,p(length),expr) for axis,(length,expr) in moments.items()})
    location = lambda name: f'<location name="{name}" unit="IN"><x>0</x><y>0</y><z>0</z></location>'
    xml = f'''<?xml version="1.0"?>
<fdm_config name="private_wing_surrogate" version="2.0" release="BETA">
 <fileheader><description>UNVALIDATED parameterized flying wing; no CFD; ideal instantaneous actuators and constant maximum thrust.</description></fileheader>
 <metrics><wingarea unit="FT2">{a['wing_area_m2']/0.3048**2}</wingarea>
 <wingspan unit="FT">{a['wingspan_m']/0.3048}</wingspan><chord unit="FT">{a['chord_m']/0.3048}</chord>
 {location('AERORP')}{location('EYEPOINT')}{location('VRP')}</metrics>
 <mass_balance><ixx unit="SLUG*FT2">{inertia[0]/1.3558179483314}</ixx>
 <iyy unit="SLUG*FT2">{inertia[1]/1.3558179483314}</iyy><izz unit="SLUG*FT2">{inertia[2]/1.3558179483314}</izz>
 <emptywt unit="LBS">{a['mass_kg']/0.45359237}</emptywt>{location('CG')}</mass_balance>
 <ground_reactions/>
 <external_reactions><property value="0">control/throttle</property>
 <force name="ideal_motor" frame="BODY"><function>{product(v(a['max_thrust_n']/4.4482216152605),p('control/throttle'))}</function>
 {location('THRUST')}<direction><x>1</x><y>0</y><z>0</z></direction></force></external_reactions>
 <propulsion/><flight_control name="ideal_normalized_elevon_channels"/>
 <aerodynamics><alphalimits unit="RAD"><min>-0.3</min><max>0.3</max></alphalimits>
 {''.join(f'<axis name="{name}"><function name="aero/surrogate/{name}">{expr}</function></axis>' for name,expr in axes.items())}
 </aerodynamics>
</fdm_config>'''
    directory = Path(directory)
    path = directory/'wing'/'wing.xml'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(xml)
    return path


def approximate_trim(config):
    a = config['aircraft']; c = a['coefficients']
    speed = config['initial']['calibrated_airspeed_kts'] * 0.5144444444444445
    rho = 1.225*np.exp(-config['initial']['altitude_h_sl_m']/8500)
    lift = a['mass_kg']*config['gravity_mps2']/(.5*rho*speed**2*a['wing_area_m2'])
    alpha = (lift-c['CL0'])/c['CL_alpha']
    elevator = -(c['Cm0']+c['Cm_alpha']*alpha)/c['Cm_de']
    throttle = .5*rho*speed**2*a['wing_area_m2']*(c['CD0']+c['k']*lift**2)/a['max_thrust_n']
    return alpha, elevator, throttle
