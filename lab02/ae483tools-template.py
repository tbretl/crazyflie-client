import json
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation


def load_hardware_data(filename):
    with open(Path(filename), 'r') as f:
        data = json.load(f)
        return data['drone'], data['mocap']
    

def resample_data_drone(raw_data, hz=1e2, t_min_offset=0., t_max_offset=0.):
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


def only_in_flight(data_drone, data_mocap=None, t_interval=None):
    # Verify the desired z position was logged
    if ('ae483log.p_z_des' not in data_drone.keys()) and ('ctrltarget.z' not in data_drone.keys()):
        raise Exception('Neither "ae483log.p_z_des" or "ctrltarget.z" were logged.')
    
    # Find the indices at which the desired z position was positive
    i = []
    for k in ['ae483log.p_z_des', 'ctrltarget.z']:
        if k in data_drone.keys():
            j = np.argwhere(data_drone[k] > 0).flatten()
            if len(j) > len(i):
                i = j
    
    # Verify that there were indices at which the desired z position was positive
    if len(i) < 2:
        raise Exception('The desired z position was never positive.')
    
    # Get first and last index at which the desired z position was positive
    i_first = i[0]
    i_last = i[-1]

    # Adjust the first and last index, if desired, to get a subset of data with
    # a given length centered in the middle of the flight time (if desired)
    if t_interval is not None:
        # Get time step (assuming it is constant)
        dt = data_drone['time'][1] - data_drone['time'][0]

        # Get number of time steps that would correspond to desired time interval
        n_interval = int(np.ceil(t_interval / dt))
        n_flight = (i_last + 1) - i_first
        
        # Verify that we have at least that number of time steps
        if n_flight < n_interval:
            t_flight = n_flight * dt
            raise Exception(f'The requested time interval ({t_interval:.2f} s) is longer than the flight time ({t_flight:.2f} s).')
        
        # Get first and last index again
        i_first += int(np.floor((n_flight - n_interval) / 2))
        i_last = i_first + n_interval

    # Truncate and time-shift data_drone
    for k in data_drone.keys():
        data_drone[k] = data_drone[k][i_first:(i_last + 1)]
    data_drone['time'] -= data_drone['time'][0]
    
    # Truncate and time-shift data_mocap, if it exists
    if data_mocap is not None:
        # Verify mocap data actually exist
        if len(data_mocap.keys()) == 0:
            raise Exception('The dictionary "data_mocap" is empty.')

        # Truncate mocap data
        for k in data_mocap.keys():
            data_mocap[k] = data_mocap[k][i_first:(i_last + 1)]

        # Time-shift mocap data
        data_mocap['time'] -= data_mocap['time'][0]

        # Verify drone and mocap data are the same
        assert(np.allclose(data_drone['time'], data_mocap['time']))
