import time
import pytest
import rclpy

from helmoro_cli.robot_manager import RobotManager
from helmoro_cli.simulation_manager import SimulationManager
from helmoro_utils.test_helpers import wait_for_topic, wait_for_node_with_namespace

@pytest.fixture(scope="module")
def robot_manager():
    """Fixture to initialize and return a RobotManager instance."""
    print("[Fixture] Creating RobotManager instance")
    return RobotManager()

@pytest.fixture(scope="module")
def simulation_manager():
    """Fixture to initialize and return a SimulationManager instance."""
    print("[Fixture] Creating SimulationManager instance")
    return SimulationManager()

def test_control(robot_manager, simulation_manager):
    """Integration test for spawning, controlling, and deleting a robot."""
    rclpy.init()
    node = rclpy.create_node("test_node")

    world = "empty"
    robot_name = "testbot"

    print("[Action] Starting simulation...")
    simulation_manager.start_simulation(world)

    print(f"[Action] Spawning robot: {robot_name}")
    robot_manager.add_robot(robot_name, 0, 0, 0, 0)
    simulation_manager.spawn_robot(robot_name, 0, 0, 0, 0)

    print("[Test] Checking for robot_description topic...")
    wait_for_topic(node, f"/{robot_name}/robot_description", timeout=10.0)
    print("[Result] Robot description topic is active")

    print("[Test] Checking for control nodes...")
    wait_for_node_with_namespace(node, "joint_state_broadcaster", f"/{robot_name}", timeout=10.0)
    wait_for_node_with_namespace(node, "diff_drive_controller", f"/{robot_name}", timeout=10.0)
    print("[Results] Control nodes are active")

    print(f"[Action] Deleting robot: {robot_name}")
    robot_manager.delete_robot(robot_name)

    print("[Test] Verifying robot deletion...")
    timeout = 2.0
    start_time = time.time()
    while time.time() - start_time < timeout:
        if robot_name not in robot_manager.get_robot_names():
            break
        time.sleep(0.1)

    remaining_robots = robot_manager.get_robot_names()
    assert robot_name not in remaining_robots, (
        f"Robot '{robot_name}' was not properly deleted. Remaining robots: {remaining_robots}"
    )
    print(f"[Result] Robot '{robot_name}' deleted successfully")

    rclpy.shutdown()
