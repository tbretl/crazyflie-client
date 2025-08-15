###################################
# IMPORTS

# Imports for crazyflie (the drone)
import logging
import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Imports for qualisys (the motion capture system)
import copy
import asyncio
import xml.etree.cElementTree as ET
from threading import Thread
from multiprocessing import SimpleQueue
import qtm_rt as qtm
from scipy.spatial.transform import Rotation

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Initialize radio
cflib.crtp.init_drivers()


###################################
# CLIENT FOR CRAZYFLIE

class CrazyflieClient:
    def __init__(self, uri, use_controller=False, use_observer=False, marker_deck_ids=None, variables=[]):
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.marker_deck_ids = marker_deck_ids
        self.variables = variables
        self.cf = Crazyflie(rw_cache='./__cfcache__')
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')
    
    def _fully_connected(self, uri):
        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {self.marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', self.marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', self.marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', self.marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', self.marker_deck_ids[3])

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)
        
        # Reset the ae483 observer
        self.cf.param.set_value('ae483par.reset_observer', 1)

        # Enable the controller (1 for default, 6 for ae483)
        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 6)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)

        # Enable the observer (0 for disable, 1 for enable)
        if self.use_observer:
            self.cf.param.set_value('ae483par.use_observer', 1)
        else:
            self.cf.param.set_value('ae483par.use_observer', 0)

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in self.variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as e:
                print(f'CrazyflieClient: Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'CrazyflieClient: Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')
        
        print(f'CrazyflieClient: Fully connected to {uri}')
        self.is_fully_connected = True

    def _connection_failed(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} failed: {msg}')

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')

    def _disconnected(self, uri):
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False
    
    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'CrazyflieClient: Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)
    
    def stop(self, dt):
        print(f'CrazyflieClient: Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)
    
    def send_pose(self, pose):
        x, y, z, qx, qy, qz, qw = pose
        self.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)

    def disconnect(self):
        # Stop logging
        for logconf in self.logconfs:
            logconf.stop()
        time.sleep(0.1)

        # Close connection
        self.cf.close_link()


###################################
# HELPER FUNCTION TO HANDLE QUEUES

def queue_handler(queue, callback):
    while True:
        pose = queue.get()
        if pose == 'END':
            break
        callback(pose)


###################################
# CLIENT FOR QUALISYS

class QualisysClient(Thread):
    def __init__(self, bodies_to_track, ip_address='128.174.245.190', version='1.24'):
        """
        The argument
            
            bodies_to_track
        
        must be a list. Each element of this list must be a dictionary with two keys:

            'name': a string with the name of a rigid body to track
            'callback': either a function with one argument (pose) that you want to be called each time a
                        new pose of the rigid body is available, or None (no function will be called); the
                        'pose' argument will be a list with seven elements: [x, y, z, qx, qy, qz, qw]
        
        The position and orientation of each named rigid body in bodies_to_track will be logged at
        100 Hz as x, y, z, yaw, pitch, roll, along with a time stamp, regardless of whether or not
        a callback is provided.

        Examples:

        If you want to track 'marker_deck_90' with no callback (i.e., only logging position
        and orientation), then specify bodies_to_track as follows:
        
            [{'name': 'marker_deck_90', 'callback': None}]

        If you want to track 'marker_deck_90' and 'marker_deck_40', and if you want to send
        the position and orientation of marker_deck_40 to the drone that is carrying the deck
        with that name, then specify bodies_to_track as follows (assuming a drone_client has
        been created for that drone):

            [{'name': 'marker_deck_90', 'callback': None},
             {'name': 'marker_deck_40', 'callback': drone_client.send_pose}}]
        
        Callbacks are needed if you want a drone_client (for example) to access motion capture data
        in real-time because everything is running in parallel in separate threads. Callbacks ensure
        that data are passed correctly between threads.
        """
        Thread.__init__(self)
        self.ip_address = ip_address
        self.bodies_to_track = copy.deepcopy(bodies_to_track)
        self.version = version
        self.data = {}
        for body in self.bodies_to_track:
            if body['callback'] is None:
                body['queue'] = None
                body['thread'] = None
            else:
                body['queue'] = SimpleQueue()
                body['thread'] = Thread(target=queue_handler, args=[body['queue'], body['callback']])
                body['thread'].start()
            self.data[body['name']] = {
                'time': [],
                'x': [],
                'y': [],
                'z': [],
                'yaw': [],
                'pitch': [],
                'roll': [],
            }
        self.num_bodies = 0
        self.connection = None
        self._stay_open = True
        self.start()

    def close(self):
        for body in self.bodies_to_track:
            if body['queue'] is not None:
                body['queue'].put('END')
        self._stay_open = False
        self.join()
        for body in self.bodies_to_track:
            if body['thread'] is not None:
                body['thread'].join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while self._stay_open:
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        print('QualisysClient: Connect to motion capture system')
        self.connection = await qtm.connect(self.ip_address, version=self.version)
        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        names = [name.text.strip() for index, name in enumerate(xml.findall('*/Body/Name'))]

        self.num_bodies = len(names)
        if self.num_bodies == 0:
            print(f'QualisysClient: WARNING -- No rigid bodies were found in params')
            return
        for body in self.bodies_to_track:
            try:
                body['index'] = names.index(body['name'])
            except:
                body['index'] = None
                print(f'QualisysClient: WARNING -- Rigid body {body['name']} was not found in params')
        await self.connection.stream_frames(
            components=['6d'],
            on_packet=self._on_packet,
        )

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()
        
        if bodies is None:
            print(f'QualisysClient: ERROR -- No rigid bodies were found in packet')
            return
        
        if len(bodies) != self.num_bodies:
            print(f'QualisysClient: ERROR -- The wrong number of rigid bodies were found in packet')
            return
        
        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6
        
        for body in self.bodies_to_track:
            # FIXME
            # Consider wrapping all of this in a "try/except" ?

            # Only proceed if the body has an index
            if body['index'] is None:
                continue
            
            # Log the time
            self.data[body['name']]['time'].append(t)

            # Get the position and orientation
            position, orientation = bodies[body['index']]

            # Only proceed if the position and orientation are valid
            if not np.all(np.isfinite(position)):
                self.data[body['name']]['x'].append(np.nan)
                self.data[body['name']]['y'].append(np.nan)
                self.data[body['name']]['z'].append(np.nan)
                self.data[body['name']]['yaw'].append(np.nan)
                self.data[body['name']]['pitch'].append(np.nan)
                self.data[body['name']]['roll'].append(np.nan)
                continue
            
            # Express the position in units of meters
            x, y, z = np.array(position) / 1e3

            # Express the orientation as ZYX body-fixed Euler angles in units of radians
            R = Rotation.from_matrix(np.reshape(orientation.matrix, (3, -1), order='F'))
            yaw, pitch, roll = R.as_euler('ZYX', degrees=False)

            # Log the position and orientation
            self.data[body['name']]['x'].append(x)
            self.data[body['name']]['y'].append(y)
            self.data[body['name']]['z'].append(z)
            self.data[body['name']]['yaw'].append(yaw)
            self.data[body['name']]['pitch'].append(pitch)
            self.data[body['name']]['roll'].append(roll)

            # Only proceed if the body has a queue (i.e., if a callback was specified)
            if body['queue'] is None:
                continue

            # Express the orientation as a unit quaternion
            qx, qy, qz, qw = R.as_quat(scalar_first=False)

            # Collect position and orientation together as a pose
            pose = [x, y, z, qx, qy, qz, qw]

            # Only proceed if the pose is valid
            if not np.all(np.isfinite(pose)):
                continue

            # Only proceed if the queue is empty, so we never create a backlog of poses
            if not body['queue'].empty():
                continue

            # Put pose in queue
            body['queue'].put(pose)

    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()