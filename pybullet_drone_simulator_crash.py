import os
import time
import argparse

import numpy as np

import pybullet as p
import pybullet_data

from PIL import Image

from proprocess_trajectory_data import get_trajectory_data


def parse_args():

    parser = argparse.ArgumentParser()

    # Log.

    data_log_folder = os.path.join("data", "log")
    parser.add_argument("--data_log_folder", type=str, default=data_log_folder)

    trajectory_data_folder = os.path.join("data", "trajectory_data")
    parser.add_argument("--trajectory_data_filename", type=str,
                        default=os.path.join(trajectory_data_folder,
                                             "trajectory_40_40_8_30_p7e3_adversarial.txt"))

    parser.add_argument("--render_save", action="store_true", default=True)

    video_frame_folder = os.path.join(data_log_folder, "frames")
    parser.add_argument("--video_frame_folder", type=str, default=video_frame_folder)

    parser.add_argument("--drone_model_evader", type=str,
                        default=os.path.join("data", "drone_model", "cf2x.urdf"))

    parser.add_argument("--drone_model_pursuer", type=str,
                        default=os.path.join("data", "drone_model", "cf2x.urdf"))

    # [red, green, blue,  alpha]

    parser.add_argument("--drone_color_evader", type=str, default=[0, 0, 1, 1])
    # Bright magenta: (1.0, 0.0, 1.0)
    parser.add_argument("--drone_color_evader_captured", type=str, default=[1, 0, 1, 1])
    parser.add_argument("--drone_color_pursuer", type=str, default=[1, 0, 0, 1])

    # Pybullet parameters.

    parser.add_argument("--gravity", type=float, default=-9.81)

    parser.add_argument("--timesteps_per_second", type=int, default=100)
    parser.add_argument("--n_timesteps", type=int, default=int(1e6))

    parser.add_argument("--use_terrain", action="store_false", default=True)

    # Camera.

    parser.add_argument("--use_customized_camera_parameters", action="store_true", default=True)

    # Full view, from top to down, rectangle.

    # parser.add_argument("--camera_distance", type=int, default=15)
    # parser.add_argument("--camera_yaw", type=int, default=0)
    # parser.add_argument("--camera_pitch", type=int, default=-89)
    # parser.add_argument("--camera_roll", type=int, default=0)
    # parser.add_argument("--camera_target_position", type=list, default=[0, 0, 0])

    # Side view.

    parser.add_argument("--camera_distance", type=int, default=10)
    parser.add_argument("--camera_yaw", type=int, default=135)
    parser.add_argument("--camera_pitch", type=int, default=-50)
    parser.add_argument("--camera_roll", type=int, default=0)
    parser.add_argument("--camera_target_position", type=list, default=[5, 5, 0])

    # Video frame size.
    # 1920 x 1080
    # 1600 x 900
    # 1280 x 1024
    # 1152 x 864
    # 1024 x 768, consider
    # 800 x 600
    # 720 x 400
    # 640 x 480, consider

    parser.add_argument("--viewport_width", type=int, default=1024)
    parser.add_argument("--viewport_height", type=int, default=768)

    return parser.parse_args()


class PybulletSimulator:

    def __init__(self, all_args):

        self.all_args = all_args

        # Share parameters.

        self.client_id = None

        self.timestep_counter = 0

        self.n_timesteps = self.all_args.n_timesteps

        self.n_evaders = 0

        self.n_pursuers = 0

        self.global_scale = 1.0

        pass

    def run_pybullet_simulation(self):

        self.warmup_simulator()

        self.start_and_configure_pybullet()

        drone_evaders, drone_pursuers = self.initialize_drone()

        # Run the simulation.

        for self.timestep_counter in range(self.n_timesteps):

            if self.timestep_counter % 50 == 0:

                print("timestep_counter / n_timesteps:", self.timestep_counter, "/", self.n_timesteps)

            if self.all_args.render_save:

                self.save_frame(self.timestep_counter)

            # Write before the position reset code. Otherwise, there are problems in the position update.

            p.stepSimulation()

            for idx_evader in range(self.n_evaders):

                if self.evader_capture_status[self.timestep_counter, idx_evader]:

                    drone_id = drone_evaders[idx_evader]

                    self.apply_physics(drone_id)

                    p.changeVisualShape(drone_id, -1, rgbaColor=self.all_args.drone_color_evader_captured)

                else:

                    p.resetBasePositionAndOrientation(drone_evaders[idx_evader],
                                                      self.evaders[self.timestep_counter, idx_evader],
                                                      [0, 0, 0, 1])

            for idx_pursuer in range(self.n_pursuers):

                p.resetBasePositionAndOrientation(drone_pursuers[idx_pursuer],
                                                  self.pursuers[self.timestep_counter, idx_pursuer],
                                                  [0, 0, 0, 1])

            # time.sleep(0.1)

        self.close_pybullet()

        pass

    def warmup_simulator(self):

        os.makedirs(self.all_args.data_log_folder, exist_ok=True)

        print("data_log_folder:", self.all_args.data_log_folder)

        if self.all_args.render_save:
            os.makedirs(self.all_args.video_frame_folder, exist_ok=True)

            print("video_frame_folder:", self.all_args.video_frame_folder)

    def start_and_configure_pybullet(self):

        self.client_id = p.connect(p.GUI)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.setGravity(gravX=0, gravY=0, gravZ=self.all_args.gravity)

        p.setRealTimeSimulation(enableRealTimeSimulation=0, physicsClientId=self.client_id)

        # Set the amount of time to proceed at each call to stepSimulation.
        # (unit is seconds, typically range is 0.01 or 0.001)

        p.setTimeStep(1 / self.all_args.timesteps_per_second, physicsClientId=self.client_id)

        # Load plane.
        # Plane is important to prevent object from moving below z < 0.

        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 1])

        # Load terrain.

        if self.all_args.use_terrain:

            p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0])

            terrain_shape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.1, .1, 1],
                                                   fileName="heightmaps/wm_height_out.png")

            texture_id = p.loadTexture("heightmaps/gimp_overlay_out.png")

            terrain_id = p.createMultiBody(0, terrain_shape)

            p.changeVisualShape(terrain_id, -1, textureUniqueId=texture_id)

            p.changeVisualShape(terrain_id, -1, rgbaColor=[1, 1, 1, 1])

        # Camera.

        if self.all_args.use_customized_camera_parameters:
            p.resetDebugVisualizerCamera(cameraDistance=self.all_args.camera_distance,
                                         cameraYaw=self.all_args.camera_yaw,
                                         cameraPitch=self.all_args.camera_pitch,
                                         cameraTargetPosition=self.all_args.camera_target_position,
                                         physicsClientId=self.client_id)

        pass

    def initialize_drone(self):

        # Get data.

        self.global_scale, self.evaders, self.pursuers, self.evader_capture_status = \
            get_trajectory_data(trajectory_data_filename=self.all_args.trajectory_data_filename)

        self.n_timesteps = len(self.evaders)

        self.n_evaders = len(self.evaders[0])

        self.n_pursuers = len(self.pursuers[0]) if self.pursuers.size > 0 else 0

        # Evader.

        drone_evaders = []

        for idx in range(self.n_evaders):

            # Do not useFixedBase=True, otherwise the drone cannot move by applying forces to it.

            drone_id = p.loadURDF(self.all_args.drone_model_evader,
                                  basePosition=[0, 0, 2],
                                  baseOrientation=[0., 0., 0., 1.],
                                  globalScaling=self.global_scale,
                                  physicsClientId=self.client_id)

            p.changeVisualShape(drone_id, -1, rgbaColor=self.all_args.drone_color_evader)

            drone_evaders.append(drone_id)

        # Pursuer.

        drone_pursuers = []

        for idx in range(self.n_pursuers):

            # Do not useFixedBase=True, otherwise the drone cannot move by applying forces to it.

            drone_id = p.loadURDF(self.all_args.drone_model_pursuer,
                                  basePosition=[0, 0, 2],
                                  baseOrientation=[0., 0., 0., 1.],
                                  globalScaling=self.global_scale,
                                  physicsClientId=self.client_id)

            p.changeVisualShape(drone_id, -1, rgbaColor=self.all_args.drone_color_pursuer)

            drone_pursuers.append(drone_id)

        return drone_evaders, drone_pursuers

    def apply_physics(self, drone_id):
        """
        Base PyBullet physics implementation.
        """

        # Crash force value.

        current_position, current_orientation = p.getBasePositionAndOrientation(drone_id)

        if current_position[2] < 2:

            forces = [0, 0, 0, 0]

        else:

            forces = [-10, 0, -10, 0]

        z_torque = 0

        for idx_motor in range(4):

            p.applyExternalForce(drone_id,
                                 idx_motor,
                                 forceObj=[0, 0, forces[idx_motor]],
                                 posObj=[0, 0, 0],
                                 flags=p.LINK_FRAME,
                                 physicsClientId=self.client_id)

        p.applyExternalTorque(drone_id,
                              4,
                              torqueObj=[0, 0, z_torque],
                              flags=p.LINK_FRAME,
                              physicsClientId=self.client_id)

        # Crash method 2.

        # Set the velocity of the drone to simulate a crash
        # p.resetBaseVelocity(drone_id, [0, 0, -10])

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

    def close_pybullet(self):

        p.disconnect(physicsClientId=self.client_id)


def main():

    all_args = parse_args()

    simulator = PybulletSimulator(all_args)

    simulator.run_pybullet_simulation()

    pass


if __name__ == "__main__":
    start_time = time.time()
    main()
    print("Time (s):", time.time() - start_time)
    print("COMPLETE!")

