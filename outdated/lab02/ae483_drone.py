import os
import time
import numpy as np
import pybullet

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

        # Connect to and configure pybullet
        self.display = display
        if self.display:
            pybullet.connect(pybullet.GUI, options=f'--width={width} --height={height}')
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        else:
            pybullet.connect(pybullet.DIRECT)

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

        # Load body frame
        self.body_frame_id = pybullet.loadURDF(os.path.join('.', 'urdf', 'frame.urdf'),
                                            basePosition=np.array([0., 0., 0.]),
                                            baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                                            useFixedBase=1,
                                            globalScaling=0.05,
                                            flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER))

        # Load drone
        self.drone_id = pybullet.loadURDF(os.path.join('.', 'urdf', 'drone.urdf'),
                           basePosition=np.array([0., 0., 0.]),
                           baseOrientation=pybullet.getQuaternionFromEuler([0., 0., 0.]),
                           useFixedBase=0,
                           globalScaling=0.25,
                           flags=(pybullet.URDF_USE_IMPLICIT_CYLINDER  |
                                  pybullet.URDF_USE_INERTIA_FROM_FILE  ))

        pybullet.resetDebugVisualizerCamera(0.35, -45, -45, [0., 0., 0.15])
        self.update_display()

    def set_pose(self, x, y, z, yaw, pitch, roll):
        pos = np.array([x, y, z])
        rpy = np.array([roll, pitch, yaw])
        ori = pybullet.getQuaternionFromEuler(rpy)
        pybullet.resetBasePositionAndOrientation(self.drone_id, pos, ori)
        pybullet.resetBasePositionAndOrientation(self.body_frame_id, pos, ori)
        self.update_display()

    def update_display(self):
        if self.display:
            # hack to get GUI to update on MacOS
            time.sleep(0.01)
            keys = pybullet.getKeyboardEvents()

    def snapshot(self):
        if self.display:
            # show whatever is in the GUI
            im = pybullet.getCameraImage(self.width, self.height, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=1)
            rgba = im[2]
            return rgba
        else:
            # show a default view
            p_eye = np.array([-0.14, -0.14, 0.375])
            p_target = np.array([0., 0., 0.15])
            v_up = np.array([0., 0., 1.])
            view_matrix = pybullet.computeViewMatrix(p_eye, p_target, v_up)
            projection_matrix = pybullet.computeProjectionMatrixFOV(fov=120, aspect=1.0, nearVal=0.01, farVal=100.0)
            im = pybullet.getCameraImage(self.width, self.height, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=pybullet.ER_BULLET_HARDWARE_OPENGL, shadow=1)
            rgba = im[2]
            return rgba
