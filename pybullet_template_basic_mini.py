import time
import pybullet as p


def start_and_configure_pybullet():

    # Connect to the PyBullet server.

    client_id = p.connect(p.GUI)

    return client_id


def close_pybullet(client_id):

    p.disconnect(physicsClientId=client_id)


def run_pybullet_simulation(client_id):

    # Run the simulation.

    n_timesteps = 1000

    for t in range(n_timesteps):

        p.stepSimulation()

        time.sleep(0.1)

    pass


def main():

    client_id = start_and_configure_pybullet()

    run_pybullet_simulation(client_id)

    close_pybullet(client_id)

    pass


if __name__ == "__main__":
    main()
    print("COMPLETE!")

