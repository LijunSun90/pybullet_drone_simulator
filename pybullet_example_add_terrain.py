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

    # Hide the color of plane by setting alpha = 0 when load the terrain.
    p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0])

    # Load terrain.

    terrain_shape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.1, .1, 10],
                                          fileName="heightmaps/wm_height_out.png")

    texture_id = p.loadTexture("heightmaps/gimp_overlay_out.png")

    terrain_id = p.createMultiBody(0, terrain_shape)

    p.changeVisualShape(terrain_id, -1, textureUniqueId=texture_id)

    p.changeVisualShape(terrain_id, -1, rgbaColor=[1, 1, 1, 1])

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

