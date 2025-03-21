import os, yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='example_robot_name',
                          description='Robot namespace'),
    DeclareLaunchArgument('run_in_simulation', default_value='true',
                          choices=['true', 'false'], description='Use sim time.'),
    DeclareLaunchArgument('rviz', default_value='true',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('world', default_value='empty',
                          description='Simulation World')
]

for pose_element in ['x', 'y', 'z', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))


def generate_launch_description():
    # Directories
    pkg_helmoro_bringup = get_package_share_directory(
        'helmoro_bringup')
    pkg_helmoro_simulator = get_package_share_directory(
        'helmoro_simulator')
    pkg_helmoro_visualization = get_package_share_directory(
        'helmoro_visualization')
    
    # Paths
    common_launch = PathJoinSubstitution(
        [pkg_helmoro_bringup, 'launch', 'common.launch.py'])
    gazebo_launch = PathJoinSubstitution(
        [pkg_helmoro_simulator, 'launch', 'simulator.launch.py'])
    visualization_launch = PathJoinSubstitution(
        [pkg_helmoro_visualization, 'launch', 'visualization.launch.py'])

    common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([common_launch]),
        launch_arguments=[
            ('run_in_simulation', LaunchConfiguration('run_in_simulation')),
            ('namespace', LaunchConfiguration('namespace'))
        ] 
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )
    
    spawn_robot = Node(
            package='ros_gz_sim',
            namespace=LaunchConfiguration('namespace'),
            executable='create',
            arguments=['-name', LaunchConfiguration('namespace'),
                       '-x', LaunchConfiguration('x'),
                       '-y', LaunchConfiguration('y'),
                       '-z', LaunchConfiguration('z'),
                       '-Y', LaunchConfiguration('yaw'),
                       '-topic', 'robot_description'],
            output='screen'
        )
    
    ros_gz_bridge = GroupAction([
        PushRosNamespace(LaunchConfiguration('namespace')),
        
        # Depth Camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='depth_camera_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo'],
                
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/depth_image' +
                '@sensor_msgs/msg/Image' + '[gz.msgs.Image'],
                
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/depth_image/points' +
                '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked']
            ],
            remappings=[
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/camera_info'],
                ['sensor/camera/depth/camera_info']),
                
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/depth_image'],
                ['sensor/camera/depth/image_raw']),
                
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/depth_camera/depth_image/points'],
                ['sensor/camera/depth/points']),
                ('/clock', 'clock')
            ]
        ),
        
        # RGB Camera
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rgb_camera_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rgb_camera/camera_info' +
                '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo'],
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rgb_camera/image' +
                '@sensor_msgs/msg/Image' + '[gz.msgs.Image']
            ],
            remappings=[
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rgb_camera/camera_info'],
                ['sensor/camera/color/camera_info']),
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rgb_camera/image'],
                ['sensor/camera/color/image_raw']),
                ('/clock', 'clock')
            ]
        ),
        
        # Lidar        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='rplidar_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rplidar/scan' +
                '@sensor_msgs/msg/LaserScan' + '[gz.msgs.LaserScan'],
                
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rplidar/scan/points' +
                '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked']
            ],
            remappings=[
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rplidar/scan'],
                ['sensor/lidar/scan']),
                
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/rplidar/scan/points'],
                ['sensor/lidar/scan/points']),
                ('/clock', 'clock')
            ]
        ),
        
        # IMU
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                ['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/imu_sensor/imu' +
                '@sensor_msgs/msg/Imu' + '[gz.msgs.IMU']
            ],
            remappings=[
                (['/world/', LaunchConfiguration('world'), '/model/', LaunchConfiguration('namespace'),
                '/link/base_link/sensor/imu_sensor/imu'],
                ['sensor/imu/imu']),
                ('/clock', 'clock')
            ]
        )
    ])
    
    
    
    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([visualization_launch])
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(common)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(visualization)
    return ld
