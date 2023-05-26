import os
import time
import argparse

import numpy as np

import pybullet as p
import pybullet_data


def parse_args():

    parser = argparse.ArgumentParser()

    # Log.

    data_log_folder = os.path.join("data", "log")
    parser.add_argument("--data_log_folder", type=str, default=data_log_folder)

    parser.add_argument("--render_save", action="store_true", default=True)

    video_frame_folder = os.path.join(data_log_folder, "frames")
    parser.add_argument("--video_frame_folder", type=str, default=video_frame_folder)

    parser.add_argument("--drone_model", type=str,
                        default=os.path.join("data", "drone_model", "cf2x.urdf"))

    # [red, green, blue,  alpha]

    parser.add_argument("--drone_color", type=str, default=[1, 0, 0, 1])

    # Pybullet parameters.

    parser.add_argument("--gravity", type=float, default=-9.81)

    parser.add_argument("--timesteps_per_second", type=int, default=100)
    parser.add_argument("--n_timesteps", type=int, default=int(1e6))

    parser.add_argument("--use_terrain", action="store_false", default=True)

    # Camera.

    parser.add_argument("--use_customized_camera_parameters", action="store_true", default=False)

    # Full view, from top to down, rectangle.

    # parser.add_argument("--camera_distance", type=int, default=15)
    # parser.add_argument("--camera_yaw", type=int, default=0)
    # parser.add_argument("--camera_pitch", type=int, default=-89)
    # parser.add_argument("--camera_roll", type=int, default=0)
    # parser.add_argument("--camera_target_position", type=list, default=[0, 0, 0])

    # Side view.

    parser.add_argument("--camera_distance", type=int, default=15)
    parser.add_argument("--camera_yaw", type=int, default=45)
    parser.add_argument("--camera_pitch", type=int, default=-35)
    parser.add_argument("--camera_roll", type=int, default=0)
    parser.add_argument("--camera_target_position", type=list, default=[0, 0, 0])

    # Default.

    # parser.add_argument("--camera_distance", type=int, default=5)
    # parser.add_argument("--camera_yaw", type=int, default=50)
    # parser.add_argument("--camera_pitch", type=int, default=-35)
    # parser.add_argument("--camera_roll", type=int, default=0)
    # parser.add_argument("--camera_target_position", type=list,
    #                     default=[-0.6275069117546082, 0.5265407562255859, -0.5735764503479004])

    # Video frame size.
    # 1920 x 1080
    # 1600 x 900
    # 1280 x 1024
    # 1152 x 864
    # 1024 x 768, consider
    # 800 x 600
    # 720 x 400
    # 640 x 480, consider

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

    def run_pybullet_simulation(self):

        self.warmup_simulator()

        self.start_and_configure_pybullet()

        drone_id = self.initialize_drone()

        # Run the simulation.

        for self.timestep_counter in range(self.all_args.n_timesteps):

            # Write before the position reset code. Otherwise, there are problems in the position update.

            p.stepSimulation()

            ##################################################
            # Apply physical force.
            
            self.apply_physics(drone_id)

            ##################################################

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

        p.setTimeStep(1/self.all_args.timesteps_per_second, physicsClientId=self.client_id)

        # Load plane.
        # Plane is important to prevent object from moving below z < 0.

        plane_id = p.loadURDF("plane.urdf", physicsClientId=self.client_id)

        p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 1])

        # Load terrain.

        if self.all_args.use_terrain:

            p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0])

            terrain_shape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.1, .1, 10],
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

        # Do not useFixedBase=True, otherwise the drone cannot move by applying forces to it.

        drone_id = p.loadURDF(self.all_args.drone_model,
                              basePosition=[0, 0, 2],
                              baseOrientation=[0., 0., 0., 1.],
                              globalScaling=1 * 4,
                              physicsClientId=self.client_id)

        p.changeVisualShape(drone_id, -1, rgbaColor=self.all_args.drone_color)

        return drone_id

    def apply_physics(self, drone_id):
        """
        Base PyBullet physics implementation.
        """

        # Crash force value.

        forces = [0, 0, 0, 0]

        # Crash force value.

        # forces = [-10, 0, -10, 0]

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

