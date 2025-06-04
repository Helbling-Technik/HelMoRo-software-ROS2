import os
from ament_index_python.packages import get_package_share_directory


class SimulationManager:
    def __init__(self):
        self.robots = {}
        self.simulation_status = "stopped"
        self.pkg_dir = get_package_share_directory("helmoro_cli")
        self.world = "empty"

    def spawn_robot(self, name, x, y, z, yaw):
        # Trigger a ROS 2 launch command to spawn the robot
        spawn = f"sudo NAMESPACE={name} X={x} Y={y} Z={z} YAW={yaw} \
            docker compose -p sim_{name} -f /home/ws/docker/docker-compose.yml up spawn_robot"

        ros_gz_bridge = f"sudo NAMESPACE={name} WORLD={self.world} \
            docker compose -p robot_{name} -f /home/ws/docker/docker-compose.yml up gz_ros_bridge --detach --wait"

        print(f"Spawn {name} in gazebo simulation.")
        os.system(spawn)
        print(f"Attach ros_gz_bridge to {name}")
        os.system(ros_gz_bridge)

    def delete_robot(self, name):
        "Deletes the robot from the simulation"
        os.system(
            f"sudo WORLD={self.world} ENTITY_NAME={name} \
            docker compose -p sim_{name} -f /home/ws/docker/docker-compose.yml up despawn_robot --detach --wait"
        )

    def get_status(self):
        return self.simulation_status

    def start_simulation(self, world):
        # Start the simulation environment
        print("Starting simulation environment...")
        simulation = f"sudo WORLD={world} docker compose -f /home/ws/docker/docker-compose.yml up simulation --detach --wait"
        print(f"Executing command: {simulation}")
        os.system(simulation)
        self.simulation_status = "running"

    def stop_simulation(self):
        # Stop the simulation environment
        print("Stopping simulation environment...")
        stop_simulation = f"sudo docker compose -f /home/ws/docker/docker-compose.yml down simulation --timeout 1"
        print(f"Executing command: {stop_simulation}")
        os.system(stop_simulation)
        self.simulation_status = "stopped"
