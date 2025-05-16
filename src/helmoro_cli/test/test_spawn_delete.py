import subprocess
import time
import pytest
import os

from helmoro_cli.robot_manager import RobotManager


@pytest.fixture(scope="module")
def robot_manager():
    print("Creating RobotManager instance...")
    return RobotManager()


def test_spawn_delete_robot(robot_manager):
    print("Spawning robot...")
    name = "testbot"
    robot_manager.spawn_robot(name, 0, 0, 0, 0)

    print("Checking if robot has spawned...")
    start = time.time()
    topic_list = os.popen("ros2 topic list").read().strip().split("\n")
    while time.time() - start < 2.0 and f"/{name}/robot_description" not in topic_list:
        topic_list = os.popen("ros2 topic list").read().strip().split("\n")
        time.sleep(0.1)

    # Check for failure
    if f"/{name}/robot_description" not in topic_list:
        print("ros2 topic list output:")
        print(topic_list)
        assert False, f"Expected topic /{name}/robot_description not found"

    print("Robot spawned sucessfully: robot_description topic is getting advertised")
    print("Deleting robot...")
    robot_manager.delete_robot(name)

    print("Checking if robot is deleted...")
    start = time.time()
    robot_list = robot_manager.get_robot_names()
    while time.time() - start < 2.0 and name in robot_list:
        robot_list = robot_manager.get_robot_names()
        time.sleep(0.1)

    # Check for failure
    if name in robot_list:
        print("Currently existing robot containers:")
        print(robot_list)
        assert False, f"{name}'s container still exists, robot wasn't properly deleted"

    print(f"Robot deleted sucessfully: {name}'s container was stopped")
