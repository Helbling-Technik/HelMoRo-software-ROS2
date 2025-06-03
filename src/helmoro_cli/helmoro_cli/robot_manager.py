import os
import re


class RobotManager:
    def __init__(self):
        self.robots = {}

    def spawn_robot(self, name, x, y, z, yaw):
        self.robots[name] = {"pos": (x, y, z), "yaw": yaw, "status": "idle"}
        # Here you would trigger a ROS 2 launch or service call
        spawn = f"sudo NAMESPACE={name} USE_SIM_TIME=True \
            docker compose -p robot_{name} -f /home/ws/.devcontainer/docker-compose.yml up robot_description --detach --wait"
        print(f"Start core nodes of {name}")
        print(f"Executing command: {spawn}")
        os.system(spawn)

    def delete_robot(self, name):
        "Stops all containers spawned by the robot"
        raw_output = (
            os.popen(
                f"sudo docker container ls --filter name={name} --format '{{{{.ID}}}} {{{{.Names}}}}'"
            )
            .read()
            .strip()
        )

        if not raw_output:
            print(f"No running containers found for robot: {name}")
            return

        containers = [line.strip().split() for line in raw_output.splitlines()]
        for container_id, container_name in containers:
            result = os.system(
                f"sudo docker container stop {container_id} --timeout 1 && sudo docker container rm {container_id} > /dev/null"
            )
            if result == 0:
                print(f"{container_name}  Stopped")

    def list_robots(self):
        "Print out all spawned robots"
        robots = self.get_robot_names()
        if not robots:
            print("No robots spawned.")
        for name in robots:
            print(name)

    def get_robot_names(self):
        "Return the list of robot names"
        # Run the Docker command and read the output
        stream = os.popen(
            'docker container ls --filter name=robot --format "{{.Names}}"'
        )
        output = stream.read()

        # Extract robot names using regex
        robot_names = set()
        pattern = re.compile(r"^robot_([^-\s]+)-")
        for line in output.strip().split("\n"):
            match = pattern.match(line)
            if match:
                robot_names.add(match.group(1))

        return sorted(robot_names)
