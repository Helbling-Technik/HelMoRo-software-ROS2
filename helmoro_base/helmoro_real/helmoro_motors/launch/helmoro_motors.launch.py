import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='helmoro_motors',
            executable='helmoro_motors_node',
            name='helmoro_motors_node'),
  ])