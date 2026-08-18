###################################
# IMPORTS

# Imports for crazyflie (the drone)
import atexit
import logging
import time
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

# Imports for qualisys (the motion capture system)
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

# The supervisor subsystem (arming, crash recovery, emergency stop) requires
# this version of the CRTP protocol or later. The version is reported by the
# firmware on the drone (CRTP_PROTOCOL_VERSION in the firmware source), not by
# the radio and not by cflib.
MIN_PROTOCOL_VERSION_FOR_SUPERVISOR = 12


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

        # Everything below is new. These are initialized here (and not in
        # _fully_connected) so that they always exist, even if the connection
        # fails before _fully_connected is ever called.
        self.logconfs = []
        self.connection_error = None
        self._is_closed = False
        self._has_supervisor = False

        # Make a best effort to stop the motors and disarm even if the flight
        # script exits in an unexpected way. This is a backstop, not a
        # substitute for calling close() from a "finally" block.
        atexit.register(self.close)

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')

    def _fully_connected(self, uri):
        # NOTE: This function runs in a callback thread, not in the thread that
        # runs your flight script. Avoid long sleeps here — they delay the
        # handling of incoming packets. Anything slow belongs in
        # wait_until_ready(), which runs in the main thread.

        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {self.marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', self.marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', self.marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', self.marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', self.marker_deck_ids[3])

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

        # Note that what we call the "default observer" will run
        # regardless of whether or not use_observer is True - the
        # parameter use_observer is only used to tell the custom
        # controller whether or not to use the estimates from the
        # default observer.
        #
        # It is VERY IMPORTANT that the default observer be the
        # Extended Kalman Filter (what is called the "kalman" type
        # in the firmware code). Measurements are forwarded from
        # this observer to ae483_controller.c so, if some other type
        # of default observer is running, measurements will not be
        # forwarded. In particular, without the flow deck, the default
        # type of the default observer is "complementary" not "kalman."
        # This will be a problem for projects that don't use the flow
        # deck. So, we need to set the correct type of the default
        # observer here, whether or not a custom observer is used.
        #
        # CHANGED: This now happens BEFORE the estimator is reset. Switching
        # estimators re-initializes the new one, so resetting first would reset
        # the estimator we are about to stop using.
        self.cf.param.set_value('stabilizer.estimator', 2)

        # Reset the default observer
        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)

        # Reset the ae483 observer (controller_ae483.c sets this back to zero)
        self.cf.param.set_value('ae483par.reset_observer', 1)

        # Find out whether this drone's firmware supports the supervisor, which
        # is what provides arming, crash recovery, and emergency stop. Arming is
        # required by brushless drones and is harmless for brushed drones.
        protocol_version = self.cf.platform.get_protocol_version()
        self._has_supervisor = protocol_version >= MIN_PROTOCOL_VERSION_FOR_SUPERVISOR
        if not self._has_supervisor:
            print(
                f'CrazyflieClient: WARNING -- The firmware on this drone reports CRTP protocol '
                f'version {protocol_version}, but version {MIN_PROTOCOL_VERSION_FOR_SUPERVISOR} '
                f'or later is needed to arm the drone. Update the firmware if this is a '
                f'brushless drone.'
            )

        # Start logging (self.logconfs was initialized in __init__)
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
        # CHANGED: Record the failure so that wait_until_ready() can stop waiting
        # instead of looping forever.
        self.connection_error = f'Connection to {uri} failed: {msg}'

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')
        self.connection_error = f'Connection to {uri} lost: {msg}'

    def _disconnected(self, uri):
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False

    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    #
    # NEW: waiting for the drone to be ready
    #

    def wait_until_ready(self, timeout=10., settle_time=2.):
        """
        Wait until the drone is fully connected and its state estimate has had
        time to converge. Raises an exception if the connection fails or if the
        drone does not connect within "timeout" seconds, rather than waiting
        forever (the usual cause is a drone that is powered off or that is on a
        different radio channel).
        """
        start_time = time.time()
        while not self.is_fully_connected:
            if self.connection_error is not None:
                raise Exception(f'CrazyflieClient: {self.connection_error}')
            if time.time() - start_time > timeout:
                raise Exception(
                    f'CrazyflieClient: Could not connect within {timeout:.1f} seconds. '
                    f'Is the drone powered on? Is it on the radio channel in your uri?'
                )
            time.sleep(0.1)

        self._report_configuration()

        # Give the state estimate time to converge after being reset. Without
        # this pause, the first second or so of every flight is flown (and
        # logged) with a state estimate that is still settling.
        print(f'CrazyflieClient: Waiting {settle_time:.1f} seconds for the state estimate to converge')
        time.sleep(settle_time)

    def _report_configuration(self):
        """
        Print a short summary of how the drone is configured. This runs in the
        main thread because reading a parameter can block.
        """
        print(f'CrazyflieClient: CRTP protocol version is {self.cf.platform.get_protocol_version()}')

        # The value of idleThrust says which kind of drone the firmware was
        # built for. There is no need to tell this client which kind of drone
        # you have — it asks the drone.
        try:
            idle_thrust = int(self.cf.param.get_value('powerDist.idleThrust', timeout=5))
        except Exception as e:
            print(f'CrazyflieClient: WARNING -- Could not read powerDist.idleThrust ({e})')
            return

        if idle_thrust == 0:
            kind = 'a brushed drone'
        else:
            kind = 'a brushless drone'
        print(f'CrazyflieClient: powerDist.idleThrust is {idle_thrust}, which is consistent with {kind}')
        print(f'CrazyflieClient: If that is not the kind of drone you have, you have very likely')
        print(f'                 built and flashed firmware for the wrong platform. Stop and check')
        print(f'                 before you fly.')

    #
    # NEW: arming, disarming, and recovery
    #

    def arm(self, timeout=2.):
        """
        Arm the drone. Brushless drones will not spin their motors until they
        are armed. Brushed drones do not require arming, but arming them anyway
        is harmless — so flight code does not need to know which kind of drone
        it is talking to.
        """
        if not self._has_supervisor:
            print('CrazyflieClient: Skipping arming (firmware does not support it)')
            return

        supervisor = self.cf.supervisor

        # Nothing to do if the drone is already armed. Brushed drones arm
        # themselves at startup — their firmware is built without
        # CONFIG_MOTORS_REQUIRE_ARMING — so this is the normal case for them.
        #
        # This check has to come first. "Can be armed" means "can *become*
        # armed," so it is false for a drone that is armed already, and testing
        # it first would report a failure to arm a drone that is armed.
        if supervisor.is_armed:
            if supervisor.is_auto_armed:
                print('CrazyflieClient: Already armed (this drone arms itself)')
            else:
                print('CrazyflieClient: Already armed')
            return

        # A drone that has crashed refuses to arm until it is told to recover.
        if supervisor.is_crashed:
            print('CrazyflieClient: The drone is in a crashed state — requesting recovery')
            supervisor.send_crash_recovery_request()
            time.sleep(0.5)

        if supervisor.is_tumbled:
            raise Exception(
                'CrazyflieClient: The drone thinks it is upside down and will not arm. '
                'Place it level, right side up, and try again.'
            )

        if supervisor.is_locked:
            raise Exception(
                'CrazyflieClient: The drone is locked and will not arm. Power it off and on '
                'again (or run "uv run ae483-reboot" with the uri of your drone).'
            )

        if not supervisor.can_be_armed:
            raise Exception(
                'CrazyflieClient: The drone reports that it cannot be armed. Its state is: '
                f'{supervisor.read_state_list()}'
            )

        print('CrazyflieClient: Arming')
        supervisor.send_arming_request(True)

        start_time = time.time()
        while not supervisor.is_armed:
            if time.time() - start_time > timeout:
                raise Exception(
                    'CrazyflieClient: Failed to arm within '
                    f'{timeout:.1f} seconds. Its state is: {supervisor.read_state_list()}'
                )
            time.sleep(0.1)
        print('CrazyflieClient: Armed')

    def disarm(self):
        """
        Disarm the drone, so that its motors will not spin.
        """
        if not self._has_supervisor:
            return
        if self.cf.supervisor.is_auto_armed:
            # This drone arms itself, so a disarm request would be undone
            # immediately and saying "Disarming" would be misleading. What
            # actually stops the motors on such a drone is the stop setpoint,
            # which close() sends just before calling this.
            return
        print('CrazyflieClient: Disarming')
        self.cf.supervisor.send_arming_request(False)

    def recover(self):
        """
        Ask the drone to recover from a crashed state, so that it can be armed
        again. This is what the "recover" button in cfclient does. It does NOT
        help with a locked drone — power-cycle that one instead.
        """
        if not self._has_supervisor:
            print('CrazyflieClient: Cannot recover (firmware does not support it)')
            return
        print('CrazyflieClient: Requesting crash recovery')
        self.cf.supervisor.send_crash_recovery_request()

    def emergency_stop(self):
        """
        Immediately stop all motors. The drone will fall. Use this only when a
        drone is misbehaving and you want it to stop right now — for normal use,
        stop() and close() bring the drone down in a controlled way.
        """
        if not self._has_supervisor:
            print('CrazyflieClient: Cannot send emergency stop (firmware does not support it)')
            return
        print('CrazyflieClient: EMERGENCY STOP')
        self.cf.supervisor.send_emergency_stop()

    def state(self):
        """
        Return a list of everything the drone's supervisor currently believes
        about itself (e.g., 'Can be armed', 'Is flying', 'Is tumbled'). Useful
        when something will not arm and you want to know why.
        """
        if not self._has_supervisor:
            return []
        return self.cf.supervisor.read_state_list()

    #
    # Flight
    #

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
            try:
                logconf.stop()
            except Exception:
                pass
        self.logconfs = []
        time.sleep(0.1)

        # Close connection
        self.cf.close_link()

    def close(self):
        """
        Stop the motors, disarm, and disconnect. Safe to call more than once,
        and safe to call when the drone never connected in the first place.

        Call this from a "finally" block in your flight code so that it runs
        even if your script is interrupted with Ctrl-C or raises an error.
        """
        if self._is_closed:
            return
        self._is_closed = True

        # Each step is guarded so that a failure in one of them does not
        # prevent the others — in particular, we always want to disconnect.
        if self.is_fully_connected:
            try:
                self.cf.commander.send_stop_setpoint()
                self.cf.commander.send_notify_setpoint_stop()
            except Exception as e:
                print(f'CrazyflieClient: Could not stop the motors ({e})')
            try:
                self.disarm()
            except Exception as e:
                print(f'CrazyflieClient: Could not disarm ({e})')

        try:
            self.disconnect()
        except Exception as e:
            print(f'CrazyflieClient: Could not disconnect cleanly ({e})')


###################################
# HELPER FUNCTION TO HANDLE QUEUES

def queue_handler(queue, callback):
    while True:
        pose = queue.get()
        if pose == 'END':
            break
        # CHANGED: If the callback raises — which happens, for example, if the
        # drone has already been disconnected while the motion capture system is
        # still streaming — this thread used to die silently and stop forwarding
        # poses. Now it complains and keeps going, so that it is still listening
        # for 'END' and can always be shut down.
        try:
            callback(pose)
        except Exception as e:
            print(f'QualisysClient: Error in callback ({e})')


###################################
# CLIENT FOR QUALISYS

class QualisysClient(Thread):
    def __init__(self, bodies_to_track, ip_address='128.174.245.64', version='1.24'):
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
        # CHANGED: daemon=True. A non-daemon thread that gets stuck (for
        # example, inside the motion capture connection) prevents python from
        # exiting AND prevents atexit handlers from running — including the one
        # that disarms the drone. Making these threads daemons means a stuck
        # thread can never keep the drone armed or wedge your terminal.
        Thread.__init__(self, daemon=True)
        self.ip_address = ip_address
        self.bodies_to_track = bodies_to_track # <-- NOTE: This means that bodies_to_track will be
                                               #           modified in place. That should be ok, but
                                               #           you need to be careful in code that uses
                                               #           QualisysClient. You also need to remember
                                               #           this as a possible source of error if you
                                               #           have any problem down the line with threads.
        self.version = version
        self.data = {}
        for body in self.bodies_to_track:
            if body['callback'] is None:
                body['queue'] = None
                body['thread'] = None
            else:
                body['queue'] = SimpleQueue()
                body['thread'] = Thread(
                    target=queue_handler,
                    args=[body['queue'], body['callback']],
                    daemon=True,  # CHANGED: see the note in Thread.__init__ above
                )
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
        self._is_closed = False
        self.start()

    def close(self, timeout=5.):
        # CHANGED: made safe to call more than once, so that it can go in a
        # "finally" block alongside CrazyflieClient.close(). Also, every join
        # now has a timeout — this function must always return, even if a thread
        # is stuck somewhere we did not anticipate. The threads are daemons, so
        # anything still running when we give up cannot keep python alive.
        if self._is_closed:
            return
        self._is_closed = True

        for body in self.bodies_to_track:
            if body['queue'] is not None:
                body['queue'].put('END')

        self._stay_open = False

        self.join(timeout)
        if self.is_alive():
            print(
                f'QualisysClient: WARNING -- The motion capture thread did not stop within '
                f'{timeout:.1f} seconds. Flight data have still been collected. If this keeps '
                f'happening, tell us.'
            )

        for body in self.bodies_to_track:
            if body['thread'] is not None:
                body['thread'].join(timeout)
                if body['thread'].is_alive():
                    print(
                        f'QualisysClient: WARNING -- The callback thread for '
                        f'{body["name"]} did not stop within {timeout:.1f} seconds.'
                    )

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
        # CHANGED: guard against never having connected. If qtm.connect timed
        # out or failed, self.connection is None, and the AttributeError that
        # used to be raised here would escape asyncio.run() during shutdown.
        if self.connection is None:
            return
        try:
            await self.connection.stream_frames_stop()
        except Exception as e:
            print(f'QualisysClient: Error while stopping the stream ({e})')
        try:
            self.connection.disconnect()
        except Exception as e:
            print(f'QualisysClient: Error while disconnecting ({e})')
