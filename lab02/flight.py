import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Specify the uri of the drone to which we want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/0/2M/E7E7E7E7E7'

# Specify the variables we want to log (all at 100 Hz)
variables = [
    # Sensor measurements
    # - IMU
    'gyro.x',
    'gyro.y',
    'gyro.z',
    'acc.x',
    'acc.y',
    'acc.z',
    # - Flow deck
    'motion.deltaX',
    'motion.deltaY',
    'range.zrange',
    # State estimates (from stock EKF)
    # - Position and orientation
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.roll',
    'stateEstimate.pitch',
    'stateEstimate.yaw',
    # - Linear and angular velocity
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    'kalman.statePX',
    'kalman.statePY',
    'kalman.statePZ',
    # Variables from the AE483 controller code (examples)
    'ae483log.num_tof',
    'ae483log.num_flow',
]


class SimpleClient:
    def __init__(self, uri, use_controller=False, use_observer=False):
        self.init_time = time.time()
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self.connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.connection_lost.add_callback(self.connection_lost)
        self.cf.disconnected.add_callback(self.disconnected)
        print(f'Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_connected = False
        self.data = {}

    def connected(self, uri):
        print(f'Connected to {uri}')
        self.is_connected = True

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self.log_data)
                logconf.error_cb.add_callback(self.log_error)
                logconf.start()
            except KeyError as e:
                print(f'Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

        # Reset the stock EKF
        self.cf.param.set_value('kalman.resetEstimation', 1)

        # Enable the controller (1 for stock controller, 4 for ae483 controller)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 4)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

    def connection_failed(self, uri, msg):
        print(f'Connection to {uri} failed: {msg}')

    def connection_lost(self, uri, msg):
        print(f'Connection to {uri} lost: {msg}')

    def disconnected(self, uri):
        print(f'Disconnected from {uri}')
        self.is_connected = False

    def log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp)
            self.data[v.name]['data'].append(data[v.name])

    def log_error(self, logconf, msg):
        print(f'Error when logging {logconf}: {msg}')

    def move(self, x, y, z, dt):
        print(f'Move to {x}, {y}, {z} for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, 0.0)
            time.sleep(0.1)

    def stop(self, dt):
        print(f'Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()

    def write_data(self, filename='logged_data.json'):
        with open(filename, 'w') as outfile:
            json.dump(self.data, outfile, indent=4, sort_keys=False)

if __name__ == '__main__':
    # Initialize everything
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    client = SimpleClient(uri, use_controller=False, use_observer=False)
    while not client.is_connected:
        print(f' ... connecting ...')
        time.sleep(1.0)

    # Leave a little time at the start to initialize
    client.stop(1.0)

    # Take off
    client.move(0.0, 0.0, 0.3, 2.0)

    # Fly in a square
    client.move(0.2, 0.0, 0.3, 2.0)
    client.move(0.2, 0.2, 0.3, 2.0)
    client.move(0.0, 0.2, 0.3, 2.0)
    client.move(0.0, 0.0, 0.3, 2.0)

    # Prepare to land
    client.move(0.0, 0.0, 0.15, 1.0)

    # Land
    client.stop(1.0)

    # Disconnect from drone
    client.disconnect()

    # Write data from flight
    client.write_data('logged_data.json')
