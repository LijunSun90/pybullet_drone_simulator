import os
import time
import argparse

import numpy as np

import pybullet as p
import pybullet_data

from PIL import Image


def parse_args():

    parser = argparse.ArgumentParser()

    # Log.

    data_log_folder = os.path.join("data", "log")
    parser.add_argument("--data_log_folder", type=str, default=data_log_folder)

    parser.add_argument("--render_save", action="store_true", default=True)

    video_frame_folder = os.path.join(data_log_folder, "frames")
    parser.add_argument("--video_frame_folder", type=str, default=video_frame_folder)

    # Pybullet parameters.

    parser.add_argument("--timesteps_per_second", type=int, default=100)
    parser.add_argument("--n_timesteps", type=int, default=int(100))

    # Camera parameters.

    parser.add_argument("--use_customized_camera_parameters", action="store_true", default=False)

    # Full view, from top to down, rectangle.

    parser.add_argument("--camera_distance", type=int, default=1.5)
    parser.add_argument("--camera_yaw", type=int, default=0)
    parser.add_argument("--camera_pitch", type=int, default=0)
    parser.add_argument("--camera_roll", type=int, default=0)
    parser.add_argument("--camera_target_position", type=list, default=[0, 0, 0])

    # Video frame size.
    # 1920 x 1080
    # 1600 x 900
    # 1280 x 1024
    # 1152 x 864
    # 1024 x 768
    # 800 x 600
    # 720 x 400
    # 640 x 480

    parser.add_argument("--viewport_width", type=int, default=640)
    parser.add_argument("--viewport_height", type=int, default=480)

    return parser.parse_args()


class PybulletSimulator:

    def __init__(self, all_args):

        self.all_args = all_args

        # Share parameters.

        self.client_id = None

        self.timestep_counter = 0

        pass

    def warmup_simulator(self):

        os.makedirs(self.all_args.data_log_folder, exist_ok=True)

        print("data_log_folder:", self.all_args.data_log_folder)
        
        if self.all_args.render_save:
            
            os.makedirs(self.all_args.video_frame_folder, exist_ok=True)

            print("video_frame_folder:", self.all_args.video_frame_folder)

    def start_and_configure_pybullet(self):

        # Connect to the PyBullet server.

        self.client_id = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load plane.
        # Plane is important to prevent object from moving below z < 0.

        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        # Duck.

        p.loadURDF(fileName="duck_vhacd.urdf",
                   basePosition=[1, -1, 0.05],
                   baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                   useFixedBase=True,
                   globalScaling=10,
                   physicsClientId=self.client_id)

        # Camera.

        if self.all_args.use_customized_camera_parameters:

            p.resetDebugVisualizerCamera(cameraDistance=self.all_args.camera_distance,
                                         cameraYaw=self.all_args.camera_yaw,
                                         cameraPitch=self.all_args.camera_pitch,
                                         cameraTargetPosition=self.all_args.camera_target_position,
                                         physicsClientId=self.client_id)

    def close_pybullet(self):

        p.disconnect(physicsClientId=self.client_id)

    def run_pybullet_simulation(self):

        self.warmup_simulator()

        self.start_and_configure_pybullet()

        # Run the simulation.

        for self.timestep_counter in range(self.all_args.n_timesteps):

            if self.timestep_counter % 10 == 0:

                print("timestep_counter:", self.timestep_counter)

            if self.all_args.render_save:

                self.save_frame(self.timestep_counter)

            p.stepSimulation()

            # time.sleep(0.05)

        self.close_pybullet()

        pass

    def save_frame(self, frame_no):

        if self.all_args.use_customized_camera_parameters:

            camera_distance = self.all_args.camera_distance
            camera_yaw = self.all_args.camera_yaw
            camera_pitch = self.all_args.camera_pitch
            camera_roll = self.all_args.camera_roll
            camera_target_position = self.all_args.camera_target_position

            viewport_width =self.all_args.viewport_width
            viewport_height = self.all_args.viewport_height
            camera_aspect_ratio = viewport_width / viewport_height

        else:

            # camera_info:
            # Length of each returned element is: 1, 1, 16, 16, 3, 3, 3, 3, 1, 1, 1, 3.
            # 0 viewport width 1024
            # 1 viewport height 768
            # 2 view matrix (0.6427875757217407, -0.4393851161003113, 0.6275069117546082, 0.0, 0.766044557094574, 0.36868780851364136, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004, 0.0, 2.384185791015625e-07, -0.0, -5.0, 1.0)
            # 3 projection matrix (0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0)
            # 4 (0.0, 0.0, 1.0)
            # 5 target position: (-0.6275069117546082, 0.5265407562255859, -0.5735764503479004)
            # 6 (17141.001953125, 20427.853515625, -0.0)
            # 7 (-8787.701171875, 7373.755859375, 16383.041015625)
            # 8 yaw 50.0
            # 9 pitch -35.0
            # 10 camera distance 5.0
            # 11 (0.0, 0.0, 0.0)
            # Angles (cameraYaw, cameraPitch, and cameraRoll) are expressed in radians, not degrees.
            # To convert to degrees, you can use the math.degrees function in Python.

            camera_info = p.getDebugVisualizerCamera()

            viewport_width = camera_info[0]
            viewport_height = camera_info[1]
            camera_aspect_ratio = viewport_width / viewport_height

            camera_target_position = camera_info[5]

            camera_yaw = camera_info[8]
            camera_pitch = camera_info[9]
            camera_roll = 0

            camera_distance = camera_info[10]

        camera_view = p.computeViewMatrixFromYawPitchRoll(distance=camera_distance,
                                                          yaw=camera_yaw,
                                                          pitch=camera_pitch,
                                                          roll=camera_roll,
                                                          cameraTargetPosition=camera_target_position,
                                                          upAxisIndex=2,
                                                          physicsClientId=self.client_id)

        camera_projection = \
            p.computeProjectionMatrixFOV(fov=90.0,
                                         aspect=camera_aspect_ratio,
                                         nearVal=0.1,
                                         farVal=1000.0)

        [w, h, rgb, dep, seg] = p.getCameraImage(width=viewport_width,
                                                 height=viewport_height,
                                                 shadow=1,
                                                 viewMatrix=camera_view,
                                                 projectionMatrix=camera_projection,
                                                 renderer=p.ER_TINY_RENDERER,
                                                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                                                 physicsClientId=self.client_id)

        (Image.fromarray(np.reshape(rgb, (h, w, 4)), 'RGBA')).save(
            os.path.join(self.all_args.video_frame_folder, "frame_" + str(frame_no) + ".png"))

        pass


def main():

    all_args = parse_args()

    simulator = PybulletSimulator(all_args)

    simulator.run_pybullet_simulation()

    pass


if __name__ == "__main__":
    main()
    print("COMPLETE!")

