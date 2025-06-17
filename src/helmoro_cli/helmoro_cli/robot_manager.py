import os
import re


class RobotManager:
    def __init__(self):
        self.robots = {}

    def add_robot(self, name, x, y, z, yaw):
        self.robots[name] = {"pos": (x, y, z), "yaw": yaw, "status": "idle"}
        spawn = f"NAMESPACE={name} USE_SIM_TIME=true \
            docker compose -p robot_{name} -f /home/ws/docker/docker-compose.yml up robot_description --detach --wait"
        ros2_control = f"NAMESPACE={name} USE_SIM_TIME=true \
            docker compose -p robot_{name}_control -f /home/ws/docker/docker-compose.yml up ros2_control --detach --wait"
        print(f"Start core nodes of {name}")
        os.system(spawn)
        os.system(ros2_control)

    def delete_robot(self, name):
        "Stops all containers spawned by the robot"
        
        # To delete all robots, change identifier
        if name == "all": name = "robot"
        
        raw_output = (
            os.popen(
                f"docker container ls --filter name={name} --format '{{{{.ID}}}} {{{{.Names}}}}'"
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
                f"docker container stop {container_id} --timeout 0 && docker container rm {container_id} > /dev/null"
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
