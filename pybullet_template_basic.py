import os
import time
import argparse

import pybullet as p


def parse_args():

    parser = argparse.ArgumentParser()

    parser.add_argument("--data_log_folder", type=str, default=os.path.join("data", "log"))

    return parser.parse_args()


class PybulletSimulator:

    def __init__(self, all_args):

        self.all_args = all_args

        # Share parameters.

        self.client_id = None

        pass

    def run_pybullet_simulation(self):

        self.warmup_simulator()

        self.start_and_configure_pybullet()

        # Run the simulation.

        n_timesteps = 1000

        for t in range(n_timesteps):
            p.stepSimulation()

            time.sleep(0.1)

        self.close_pybullet()

        pass

    def warmup_simulator(self):

        os.makedirs(self.all_args.data_log_folder, exist_ok=True)

        print("data_log_folder:", self.all_args.data_log_folder)

    def start_and_configure_pybullet(self):

        # Connect to the PyBullet server.

        self.client_id = p.connect(p.GUI)

    def close_pybullet(self):

        p.disconnect(physicsClientId=self.client_id)


def main():

    all_args = parse_args()

    simulator = PybulletSimulator(all_args)

    simulator.run_pybullet_simulation()

    pass


if __name__ == "__main__":
    main()
    print("COMPLETE!")

