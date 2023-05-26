import time
import pybullet as p
import pybullet_data


def start_and_configure_pybullet():

    # Connect to the PyBullet server.

    client_id = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load plane.
    # Plane is important to prevent object from moving below z < 0.

    plane_id = p.loadURDF("plane.urdf", physicsClientId=client_id)

    # Load drone.

    # Drone appearance 1.

    # quadrotor.urdf depends on quadrotor_base.obj

    drone_id = p.loadURDF("data/drone_model/quadrotor.urdf",
                          basePosition=[1, 1, 2],
                          baseOrientation=[0., 0., 0., 1.],
                          useMaximalCoordinates=0,
                          useFixedBase=True,
                          flags=0,
                          globalScaling=1,
                          physicsClientId=client_id)

    #  rgb_color [red, green, blue,  alpha]

    p.changeVisualShape(drone_id, -1, rgbaColor=[1, 0, 0, 1])

    # hb.urdf depends on quad.obj

    drone_id = p.loadURDF("data/drone_model/hb.urdf",
                          basePosition=[0, 0, 2],
                          baseOrientation=[0., 0., 0., 1.],
                          useMaximalCoordinates=0,
                          useFixedBase=True,
                          flags=0,
                          globalScaling=1,
                          physicsClientId=client_id)

    p.changeVisualShape(drone_id, -1, rgbaColor=[1, 1, 0, 1])

    # Drone appearance 2.

    # cf2x.urdf depends on cf2.dae.

    drone_id = p.loadURDF("data/drone_model/cf2x.urdf",
                          basePosition=[1, -1, 2],
                          baseOrientation=[0., 0., 0., 1.],
                          useMaximalCoordinates=0,
                          useFixedBase=True,
                          flags=0,
                          globalScaling=1*4,
                          physicsClientId=client_id)

    p.changeVisualShape(drone_id, -1, rgbaColor=[0, 1, 0, 1])

    # cf2p.urdf dependes on cf2.dae.

    drone_id = p.loadURDF("data/drone_model/cf2p.urdf",
                          basePosition=[-1, 1, 2],
                          baseOrientation=[0., 0., 0., 1.],
                          useMaximalCoordinates=0,
                          useFixedBase=True,
                          flags=0,
                          globalScaling=1 * 4,
                          physicsClientId=client_id)

    p.changeVisualShape(drone_id, -1, rgbaColor=[0, 0, 1, 1])

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

