import os
import argparse

import numpy as np
import pandas as pd
import json


def parse_args():

    parser = argparse.ArgumentParser()

    trajectory_data_folder = os.path.join("data", "trajectory_data")

    parser.add_argument("--trajectory_data_filename", type=str,
                        default=os.path.join(trajectory_data_folder,
                                             "trajectory_40_40_8_30_p7e3_adversarial.txt"))

    return parser.parse_args()


def main():

    all_args = parse_args()

    # Get data.

    global_scale, evaders, pursuers, evader_capture_status = \
        get_trajectory_data(trajectory_data_filename=all_args.trajectory_data_filename)

    print("global_scale", global_scale)
    print("evaders", evaders.shape)
    print("pursuers", pursuers.shape)
    print("evader_capture_status", evader_capture_status.shape)

    pass


def get_trajectory_data(trajectory_data_filename):

    # Discrete world.

    evaders, pursuers, evader_capture_status = load_multiple_numpy_array_from_file(trajectory_data_filename)

    # Continuous world.

    global_scale, evaders, pursuers = \
        convert_discrete_2d_to_continuous_3d_coordinate(evaders, pursuers, evader_capture_status)

    return global_scale, evaders, pursuers, evader_capture_status


def load_multiple_numpy_array_from_file(trajectory_data_filename):

    data = pd.read_table(trajectory_data_filename)

    # print("data:\n", data)

    # KEY CODE:
    # Convert
    # s = "[[0, 1], [0, 3], [1, 1], [1, 3]]"
    # to list
    # json.loads(s) -> [[0, 0], [0, 2], [2, 0], [2, 2]]
    # and then to numpy array.

    # (n_timestep, swarm_size, 2).

    evaders = np.array(list(map(lambda x: np.array(json.loads(x)), data["evaders"])))
    pursuers = np.array(list(map(lambda x: np.array(json.loads(x)), data["pursuers"])))

    if "evader_capture_status" in data.columns:

        evader_capture_status = np.array(list(map(lambda x: np.array(json.loads(x)), data["evader_capture_status"])))

    else:

        evader_capture_status = np.ones((evaders.shape[0], evaders.shape[1])) * False

    # print("evaders:\n", data["evaders"])
    # print("pursuers:\n", pursuers.shape)
    # print("evader_capture_status:", evader_capture_status)

    return evaders, pursuers, evader_capture_status


def convert_discrete_2d_to_continuous_3d_coordinate(evaders, pursuers, evader_capture_status):
    """
    :param evaders: 2-D numpy array, where one row correspond to the trajectory of one type agent in one time step.
    :param pursuers: 2-D numpy array, where one row correspond to the trajectory of one type agent in one time step.
    :param evader_capture_status: 2-D numpy array,
                                  where one row correspond to the trajectory of one type agent in one time step.

    Correspondence between continuous world and discrete world.

    # quadrotor.

    Continuous world: [-10, 10] x [-10, 10].
    Discrete world if global_scale = 1: 30 x 30.

    Linear map from discrete coordinate to continuous coordinate: y = 2 / 3 * x - 10.

    # cf2x.

    Continuous world: [-10, 10] x [-10, 10].
    Discrete world if global_scale = 1: 166 x 166.

    Linear map from discrete coordinate to continuous coordinate: y = 20 / 166 * x - 10.

    Calculation detail.

    quadrotor.urdf
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder radius=".3" length=".1"/>
      </geometry>
    </collision>

    Quadrotor geometry:
        - Shape: cylinder.
        - Radius: 0.3.

    Grid world:
        - grid size in the continuous world: 20 x 20.
        - no. of quadrotors (global scale = 1) one-by-one align with one dimension: 16 * 2.
        - equivalent grid size: 32 x 32.
        - coordinate system range: [-9.6, 9.6] x [-9.6, 9.6].

    """

    # Inherent parameter.

    # base_grid_size = 30
    base_grid_size = 166.66

    # Discrete world.

    if pursuers.size > 0:

        grid_size = np.max([np.max(evaders), np.max(pursuers)])

    else:

        grid_size = np.max(evaders)

    global_scale = base_grid_size / grid_size

    # Convert to the continuous world: y = 20 / grid_size * x - 10.

    evaders = 20 / grid_size * evaders - 10

    pursuers = 20 / grid_size * pursuers - 10

    # 2-D to 3-D.

    height = 4

    evaders = np.pad(evaders, [(0, 0), (0, 0), (0, 1)], mode='constant', constant_values=height)

    for idx_time_step, swarm_capture_status in enumerate(evader_capture_status):

        for idx_evader, capture_status in enumerate(swarm_capture_status):

            if capture_status == 1:

                evaders[idx_time_step, idx_evader, 2] = 0

    if pursuers.size > 0:

        pursuers = np.pad(pursuers, [(0, 0), (0, 0), (0, 1)], mode='constant', constant_values=height)

    return global_scale, evaders, pursuers


if __name__ == "__main__":
    main()
    print("COMPLETE!")

