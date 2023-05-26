import time
import pybullet as p
import pybullet_data


def start_and_configure_pybullet():

    client_id = p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    plane_id = p.loadURDF("plane.urdf", physicsClientId=client_id)

    # If useFixedBase=False, you can move the object in the rendering by mouse.

    # Teddy.

    p.loadURDF(fileName="teddy_large.urdf",
               basePosition=[0.5, 0.5, 0],
               useFixedBase=True,
               physicsClientId=client_id)

    # Duck.

    p.loadURDF(fileName="duck_vhacd.urdf",
               basePosition=[1, -1, 0.05],
               baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
               useFixedBase=True,
               globalScaling=10,
               physicsClientId=client_id)

    # Sphere.

    p.loadURDF(fileName="sphere2.urdf",
               basePosition=[-1, 1, 0.5],
               useFixedBase=True,
               physicsClientId=client_id)

    # Cube.

    p.loadURDF(fileName="cube_no_rotation.urdf",
               basePosition=[-1, -1, 0.5],
               useFixedBase=True,
               physicsClientId=client_id)

    # Wall 1.
    wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 2, 1])
    wall_id = p.createMultiBody(0, wall_collision_shape, -1,
                                basePosition=[-3, 0, 0],
                                baseOrientation=[0, 0, 0, 1],
                                baseInertialFramePosition=[0, 0, 0],
                                baseInertialFrameOrientation=[0, 0, 0, 1])

    return client_id


def close_pybullet(client_id):

    p.disconnect(physicsClientId=client_id)


def run_pybullet_simulation(client_id):

    # Run the simulation.

    n_timesteps = 1000

    for t in range(n_timesteps):

        p.stepSimulation()

        time.sleep(0.05)

    pass


def main():

    client_id = start_and_configure_pybullet()

    run_pybullet_simulation(client_id)

    close_pybullet(client_id)

    pass


if __name__ == "__main__":
    main()
    print("COMPLETE!")

