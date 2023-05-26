import time
import pybullet as p


def pybullet_basic_use_example():

    # Connect to the PyBullet server.

    client_id = p.connect(p.GUI)

    # Run the simulation.

    n_timesteps = 1000

    for t in range(n_timesteps):

        p.stepSimulation()

        time.sleep(0.1)

    # Disconnect from the PyBullet server

    p.disconnect(physicsClientId=client_id)

    pass


def main():
    pybullet_basic_use_example()
    pass


if __name__ == "__main__":
    main()
    print("COMPLETE!")

