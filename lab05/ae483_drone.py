import numpy as np
import pybullet
import time
import os
import json
from scipy import linalg
import importlib
import traceback

class Simulator:

    def __init__(
                    self,
                    display=True,
                    seed=None,
                    width=640,
                    height=480,
                ):

        # Create random number generator
        self.rng = np.random.default_rng(seed)

        # Size of display
        self.width = width
        self.height = height

        # Choose the time step
        self.dt = 0.01

        # Create empty list of drones
        self.drones = []
        self.max_num_drones = 40

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            pybullet.connect(pybullet.GUI, options=f'--width={width} --height={height}')
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        else:
            pybullet.connect(pybullet.DIRECT)
        pybullet.setGravity(0, 0, -9.81)
        pybullet.setPhysicsEngineParameter(fixedTimeStep=self.dt,
                                    numSubSteps=4,
                                    restitutionVelocityThreshold=0.05,
                                    enableFileCaching=0)

        # Load plane
        self.plane_id = pybullet.loadURDF(os.path.join('.', 'urdf', 'plane.urdf'),
                                        basePosition=np.array([0., 0., 0.]),
                                        baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                                        useFixedBase=1)

        # Load world frame
        self.world_frame_id = pybullet.loadURDF(os.path.join('.', 'urdf', 'frame.urdf'),
                                            basePosition=np.array([0., 0., 0.]),
                                            baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                                            useFixedBase=1,
                                            globalScaling=0.05,
                                            flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER))

        # Set contact parameters
        object_ids = [self.plane_id]
        for object_id in object_ids:
            pybullet.changeDynamics(object_id, -1,
                lateralFriction=1.0,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.5,
                contactDamping=-1,
                contactStiffness=-1)

        self.camera_target = np.array([0., 0., 0.3])
        self.camera_yaw = 0.
        self.camera_distance = 0.5
        self.camera_pitch = 0.

        self.needs_reset = True

        self.camera()

    def clear_drones(self):
        self.set_camera_target([0., 0., 0.3])
        for drone in self.drones:
            pybullet.removeBody(drone['id'])
            pybullet.removeBody(drone['setpoint_frame_id'])
        self.drones = []
        self.needs_reset = True

    def add_drone(self,
                  name,
                  RobotClient,
                  RobotController,
                  rgba=[0., 1., 1., 1.],
                  m=0.032,
                  J_x=1e-5,
                  J_y=1e-5,
                  J_z=2e-5,
                  g=9.81,
                  l=0.035,
                  k_F=2e-6,
                  k_M=1e-8):

        if self.get_drone_by_name(name) is not None:
            raise Exception(f'drone with name "{name}" already exists')

        # load urdf
        id = pybullet.loadURDF(os.path.join('.', 'urdf', 'drone.urdf'),
                       basePosition=np.array([0., 0., 0.1]),
                       baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                       useFixedBase=0,
                       flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                              pybullet.URDF_USE_INERTIA_FROM_FILE  ))

        # apply color
        pybullet.changeVisualShape(id, -1, rgbaColor=rgba)

        # set mass, moment of inertia, and contact parameters
        pybullet.changeDynamics(id, -1,
            mass=m,
            localInertiaDiagonal=np.array([J_x, J_y, J_z]),
            lateralFriction=1.0,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.5,
            contactDamping=-1,
            contactStiffness=-1)

        if np.isscalar(k_F):
            k_F = k_F * np.ones(4)

        if np.isscalar(k_M):
            k_M = k_M * np.ones(4)

        setpoint_frame_id = pybullet.loadURDF(os.path.join('.', 'urdf', 'frame.urdf'),
                                              basePosition=np.array([0., 0., 0.1]),
                                              baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                                              useFixedBase=0,
                                              globalScaling=0.05,
                                              flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER))
        pybullet.changeVisualShape(setpoint_frame_id, -1, rgbaColor=rgba)

        self.drones.append({
            'id': id,
            'setpoint_frame_id': setpoint_frame_id,
            'RobotController': RobotController,
            'RobotClient': RobotClient,
            'name': name,
            'P': np.array([[ -l * k_F[0], -l * k_F[1],  l * k_F[2],  l * k_F[3]  ],
                           [ -l * k_F[0], l * k_F[1],   l * k_F[2],  -l * k_F[3] ],
                           [ -k_M[0],     k_M[1],       -k_M[2],     k_M[3]      ],
                           [ k_F[0],      k_F[1],       k_F[2],      k_F[3]      ]]),
        })

        self.needs_reset = True

    def set_color(self, id, rgba):
        pybullet.changeVisualShape(id, -1, rgbaColor=rgba)
        for i in range(pybullet.getNumJoints(id)):
            pybullet.changeVisualShape(id, i, rgbaColor=rgba)

    def camera(self):
        if self.display:
            if isinstance(self.camera_target, str):
                drone = self.get_drone_by_name(self.camera_target)
                if drone is None:
                    raise Exception(f'drone "{self.camera_target}" does not exist')
                pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
                eul = pybullet.getEulerFromQuaternion(ori)
                pybullet.resetDebugVisualizerCamera(self.camera_distance, (eul[2] * 180 / np.pi) - 90 + self.camera_yaw, -self.camera_pitch, pos)
            else:
                pybullet.resetDebugVisualizerCamera(self.camera_distance, -90 + self.camera_yaw, -self.camera_pitch, self.camera_target)

            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = pybullet.getKeyboardEvents()

    def set_camera_yaw(self, yaw):
        self.camera_yaw = yaw
        self.camera()

    def set_camera_pitch(self, pitch):
        self.camera_pitch = pitch
        self.camera()

    def set_camera_distance(self, distance):
        self.camera_distance = distance
        self.camera()

    def camera_sideview(self):
        self.camera_pitch = 0.
        self.camera()

    def camera_topview(self):
        self.camera_pitch = 89. # <-- hack: camera can't be perfectly overhead
        self.camera()

    def set_camera_target(self, target):
        if isinstance(target, str):
            if self.get_drone_by_name(target) is None:
                raise Exception(f'drone "{target}" does not exist')
        self.camera_target = target
        self.camera()

    def disconnect(self):
        pybullet.disconnect()

    def reset(self):
        # Reset time
        self.max_time_steps = 0
        self.time_step = 0
        self.t = 0.

        # Do nothing else if there are no drones
        if len(self.drones) == 0:
            return

        for drone in self.drones:
            try:
                drone['controller'] = drone['RobotController']()
                drone['client'] = drone['RobotClient']()
            except Exception as err:
                print(f'Failed to initialize drone {drone["name"]} (turning it off):')
                print(f'\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

            drone['u'] = np.zeros(4)
            drone['running'] = True
            drone['data'] = {
                't': [],
                'power': {},
                'state': {},
                'setpoint': {},
            }

        self.needs_reset = False

        # Reset camera
        self.camera()

    def set_actuator_commands(self, m1, m2, m3, m4, drone):
        m = np.array([m1, m2, m3, m4])
        m_clipped = np.clip(m, 0, 65535)
        if not np.allclose(m, m_clipped):
            raise Exception(f'motor power commands are out of bounds:\n {m}')
        drone['u'] = drone['P'] @ np.around(m_clipped, decimals=0)

    def get_state(self, drone):
        pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
        rpy = pybullet.getEulerFromQuaternion(ori)
        vel = pybullet.getBaseVelocity(drone['id'])
        v_world = np.array(vel[0])
        w_world = np.array(vel[1])
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
        v_body = R_body_in_world.T @ v_world
        w_body = R_body_in_world.T @ w_world
        o = np.array(pos)
        rpy = np.array(rpy)
        v = v_body
        w = w_body
        state = {
            'o_x': o[0],
            'o_y': o[1],
            'o_z': o[2],
            'psi': rpy[2],
            'theta': rpy[1],
            'phi': rpy[0],
            'v_x': v[0],
            'v_y': v[1],
            'v_z': v[2],
            'w_x': w[0],
            'w_y': w[1],
            'w_z': w[2],
        }
        return state

    def set_state(self, name, state):
        drone = self.get_drone_by_name(name)
        if drone is None:
            raise Exception(f'drone {name} not found')

        # Position and orientation
        pos = np.array([state['o_x'], state['o_y'], state['o_z']])
        rpy = np.array([state['phi'], state['theta'], state['psi']])
        ori = pybullet.getQuaternionFromEuler(rpy)
        pybullet.resetBasePositionAndOrientation(drone['id'], pos, ori)
        pybullet.resetBasePositionAndOrientation(drone['setpoint_frame_id'], pos, ori)

        # Linear and angular velocity
        v_body = np.array([state['v_x'], state['v_y'], state['v_z']])
        w_body = np.array([state['w_x'], state['w_y'], state['w_z']])
        R_body_in_world = np.reshape(np.array(pybullet.getMatrixFromQuaternion(ori)), (3, 3))
        v_world = R_body_in_world @ v_body
        w_world = R_body_in_world @ w_body
        pybullet.resetBaseVelocity(drone['id'],
                            linearVelocity=v_world,
                            angularVelocity=w_world)

        # Reset camera
        self.camera()

        self.needs_reset = True

    def run(self, max_time=None, data_filename=None, video_filename=None):
        if self.needs_reset:
            self.reset()

        if max_time is None:
            self.max_time_steps = None
        else:
            self.max_time_steps = int((max_time + self.t) / self.dt)
        self.start_time = time.time() - self.t

        logged_start_time = time.time()
        logged_start_time_step = self.time_step

        if video_filename is not None:
            # Import imageio
            imageio = importlib.import_module('imageio')

            # Open video
            fps = int(1 / self.dt)
            print(f'Creating a video with name {video_filename} and fps {fps}')
            w = imageio.get_writer(video_filename,
                                   format='FFMPEG',
                                   mode='I',
                                   fps=fps)

            # Add first frame to video
            rgba = self.snapshot()
            w.append_data(rgba)

        while True:
            all_done = self.step()

            if video_filename is not None:
                if self.time_step % 100 == 0:
                    print(f' {self.time_step} / {self.max_time_steps}')

                # Add frame to video
                rgba = self.snapshot()
                w.append_data(rgba)

            if all_done:
                break

            if (self.max_time_steps is not None) and (self.time_step == self.max_time_steps):
                break

        if video_filename is not None:
            # Close video
            w.close()

        if data_filename is not None:
            data = {}
            for drone in self.drones:
                data[drone['name']] = drone['data']
            with open(data_filename, 'w') as f:
                json.dump(data, f)

        logged_stop_time = time.time()
        logged_stop_time_step = self.time_step

        elapsed_time = logged_stop_time - logged_start_time
        elapsed_time_steps = logged_stop_time_step - logged_start_time_step
        if elapsed_time > 0:
            print(f'Completed {elapsed_time_steps} time steps in {elapsed_time:.4f} seconds ({(elapsed_time_steps / elapsed_time):.4f} time steps per second)')


    def step(self):
        """
        does one step in the simulation
        """

        # current time
        self.t = self.time_step * self.dt

        all_done = True
        for index, drone in enumerate(self.drones):
            # ignore the drone if it is not still running
            if not drone['running']:
                continue

            # if the drone is still running, the simulation should continue
            all_done = False

            # get state
            state = self.get_state(drone)

            # get actuator commands
            try:
                setpoint = drone['client'].run(self.t)
                pos = np.array([setpoint['o_x'], setpoint['o_y'], setpoint['o_z']])
                rpy = np.array([0., 0., setpoint['psi']])
                ori = pybullet.getQuaternionFromEuler(rpy)
                pybullet.resetBasePositionAndOrientation(drone['setpoint_frame_id'], pos, ori)
                m1, m2, m3, m4 = drone['controller'].run(state, setpoint)
                self.set_actuator_commands(m1, m2, m3, m4, drone)
            except Exception as err:
                print(f'\n==========\nerror on run of drone {drone["name"]} (turning it off):\n==========\n{traceback.format_exc()}==========\n')
                drone['running'] = False
                continue

            # collect motor power commands
            power = {
                'm_1': m1,
                'm_2': m2,
                'm_3': m3,
                'm_4': m4,
            }

            # apply rotor forces
            pybullet.applyExternalForce(drone['id'], 0, np.array([0., 0., drone['u'][3]]), np.array([0., 0., 0.]), pybullet.LINK_FRAME)

            # apply rotor torques
            pybullet.applyExternalTorque(drone['id'], 0, np.array([drone['u'][0], drone['u'][1], drone['u'][2]]), pybullet.LINK_FRAME)

            # log data
            data = drone['data']
            data['t'].append(self.t)
            for (obj, name) in [(power, 'power'), (state, 'state'), (setpoint, 'setpoint')]:
                for key, val in obj.items():
                    if key in data[name].keys():
                        data[name][key].append(val)
                    else:
                        data[name][key] = [val]

        # try to stay real-time
        if self.display:
            t = self.start_time + (self.dt * (self.time_step + 1))
            time_to_wait = t - time.time()
            while time_to_wait > 0:
                time.sleep(0.9 * time_to_wait)
                time_to_wait = t - time.time()

        # take a simulation step
        pybullet.stepSimulation()

        # increment time step
        self.time_step += 1

        # update camera
        if isinstance(self.camera_target, str):
            self.camera()

        return all_done

    def get_drone_by_name(self, name):
        for drone in self.drones:
            if drone['name'] == name:
                return drone
        return None

    def snapshot(self):
        if isinstance(self.camera_target, str):
            drone = self.get_drone_by_name(self.camera_target)
            if drone is None:
                raise Exception(f'drone "{self.camera_target}" does not exist')
            pos, ori = pybullet.getBasePositionAndOrientation(drone['id'])
            eul = pybullet.getEulerFromQuaternion(ori)
            yaw = (eul[2] * 180 / np.pi) - 90 + self.camera_yaw
        else:
            pos = self.camera_target
            yaw = -90 + self.camera_yaw
        view_matrix = pybullet.computeViewMatrixFromYawPitchRoll(pos, self.camera_distance, yaw, -self.camera_pitch, 0., 2)
        projection_matrix = pybullet.computeProjectionMatrixFOV(fov=120, aspect=1.0, nearVal=0.01, farVal=100.0)
        im = pybullet.getCameraImage(self.width, self.height, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=1)
        rgba = im[2]
        return rgba
