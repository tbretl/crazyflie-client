import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def load_hardware_data(filename):
    with open(Path(filename), 'r') as f:
        data = json.load(f)
        return data['drone'], data['mocap']
    

def resample_data_drone(raw_data, hz=1e2, t_min_offset=0., t_max_offset=0., only_in_flight=False):
    # copy data (FIXME: may be unnecessary?)
    data = {}
    for key, val in raw_data.items():
        data[key] = {
            'time': val['time'].copy(),
            'data': val['data'].copy(),
        }

    # convert lists to numpy arrays
    for key, val in data.items():
        val['data'] = np.array(val['data'], dtype=np.float64).copy()
        val['time'] = np.array(val['time'], dtype=np.float64).copy()

    # find time interval
    t_min = -np.inf
    t_max = np.inf
    for key, val in data.items():
        t_min = max(t_min, val['time'][0])
        t_max = min(t_max, val['time'][-1])
    if (t_min_offset < 0) or (t_min_offset > t_max - t_min):
        raise Exception(f't_min_offset = {t_min_offset:.4f} must be in [0, {t_max - t_min:.4f}]')
    if (t_max_offset < 0) or (t_max_offset > t_max - t_min):
        raise Exception(f't_max_offset = {t_max_offset:.4f} must be in [0, {t_max - t_min:.4f}]')
    t_min += t_min_offset
    t_max -= t_max_offset

    # find zero time
    t_zero = t_min

    # do time shift
    t_min -= t_zero
    t_max -= t_zero
    for key, val in data.items():
        val['time'] -= t_zero

    # create an array of times at which to subsample
    if np.round(hz) != hz:
        raise Exception(f'hz = {hz} must be an integer')
    nt = int(1 + np.floor((t_max - t_min) * hz))
    t = t_min + np.arange(0, nt) / hz
    
    # resample raw data with linear interpolation
    resampled_data = {'time': t}
    for key, val in data.items():
        resampled_data[key] = np.interp(t, val['time'], val['data'])
    
    # truncate to times when o_z_des is positive
    if only_in_flight:
        i = []
        for k in ['ae483log.o_z_des', 'ctrltarget.z']:
            if k in resampled_data.keys():
                j = np.argwhere(resampled_data[k] > 0).flatten()
                if len(j) > len(i):
                    i = j
        if len(i) < 2:
            raise Exception(
                'Failed to get "only_in_flight" data.\n' + \
                ' - Did you remember to log "ae483log.o_z_des" and was it ever positive?\n' + \
                ' - Did you remember to log "ctrltarget.z" and was it ever positive?\n'
            )
        for key in resampled_data.keys():
            resampled_data[key] = resampled_data[key][i[0]:i[-1]]
        
    # return the resampled data
    return resampled_data


def resample_data_mocap(raw_data, t, t_shift=0.):
    # copy data (FIXME: may be unnecessary?) and convert lists to numpy arrays
    data = {}
    for key, val in raw_data.items():
        data[key] = np.array(val, dtype=np.float64).copy()
    
    # find nan
    is_valid = np.ones_like(data['time']).astype('bool')
    for val in data.values():
        is_valid = np.bitwise_and(is_valid, ~np.isnan(val))
    i_is_valid = np.argwhere(is_valid).flatten()

    # remove nan
    for key, val in data.items():
        data[key] = val[i_is_valid]
    
    # do time shift
    data['time'] -= (data['time'][0] - t_shift)

    # resample raw data with linear interpolation
    resampled_data = {'time': t.copy()}
    for key, val in data.items():
        if key != 'time':
            resampled_data[key] = np.interp(t, data['time'], val)
    
    # return the resampled data
    return resampled_data


