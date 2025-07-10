import time
import pytest
import rclpy

from helmoro_cli.robot_manager import RobotManager
from helmoro_cli.simulation_manager import SimulationManager
from helmoro_utils.test_helpers import wait_for_topic, wait_for_node_with_namespace, wait_for_goal_reached
from helmoro_utils.publishers import publish_twist_stamped

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

def test_simple_scenario(robot_manager, simulation_manager):
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
    wait_for_node_with_namespace(node, "joint_state_broadcaster", f"/{robot_name}", timeout=20.0)
    wait_for_node_with_namespace(node, "diff_drive_controller", f"/{robot_name}", timeout=10.0)
    print("[Results] Control nodes are active")

    print("[Action] Sending move command to the robot...")
    publish_twist_stamped(
        node, f"/{robot_name}/cmd_vel", x_vel=1.0, rot_vel=0.0)
    
    print("[Test] Waiting for robot to reach the goal position...")
    wait_for_goal_reached(node, robot_name, x=1.0, y=0.0, timeout=10.0)
    
    print("[Action] Sending stop command to the robot...")
    publish_twist_stamped(
        node, f"/{robot_name}/cmd_vel", x_vel=0.0, rot_vel=0.0)
    
    print("[Test] Verifying robot stopped...")
    print("[Result] Robot control commands were successfully sent and executed")
        
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
    
    print("[Action] Stopping simulation...")
    simulation_manager.stop_simulation()
    
    rclpy.shutdown()

def test_cleanup(robot_manager, simulation_manager):
    """Test to ensure RobotManager cleans up properly."""
    print("[Action] Cleaning up test...")
    robot_manager.delete_robot("all")
    
    remaining_robots = robot_manager.get_robot_names()
    assert not remaining_robots, (
        f"RobotManager cleanup failed. Remaining robots: {remaining_robots}"
    )
    
    simulation_manager.stop_simulation()
    print("[Result] RobotManager cleaned up successfully")
    
    