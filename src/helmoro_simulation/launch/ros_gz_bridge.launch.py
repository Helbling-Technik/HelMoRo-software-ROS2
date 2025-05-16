from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
    DeclareLaunchArgument(
        "namespace", default_value="empty_namespace", description="Robot namespace"
    ),
    DeclareLaunchArgument(
        "world", default_value="unspecified", description="Wolrd name"
    ),
]


def generate_launch_description():

    ros_gz_bridge = GroupAction(
        [
            PushRosNamespace(LaunchConfiguration("namespace")),
            # TODO: Update to ROS Kilted will add support for custom URDF frames making this file unnecessary
            # Depth Camera
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="depth_camera_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/depth_camera/camera_info"
                        + "@sensor_msgs/msg/CameraInfo"
                        + "[gz.msgs.CameraInfo",
                    ],
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/depth_camera/depth_image"
                        + "@sensor_msgs/msg/Image"
                        + "[gz.msgs.Image",
                    ],
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/depth_camera/depth_image/points"
                        + "@sensor_msgs/msg/PointCloud2"
                        + "[gz.msgs.PointCloudPacked",
                    ],
                ],
                remappings=[
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/depth_camera/camera_info",
                        ],
                        ["sensor/camera/depth/camera_info"],
                    ),
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/depth_camera/depth_image",
                        ],
                        ["sensor/camera/depth/image_raw"],
                    ),
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/depth_camera/depth_image/points",
                        ],
                        ["sensor/camera/depth/points"],
                    ),
                ],
            ),
            # RGB Camera
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rgb_camera_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rgb_camera/camera_info"
                        + "@sensor_msgs/msg/CameraInfo"
                        + "[gz.msgs.CameraInfo",
                    ],
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rgb_camera/image"
                        + "@sensor_msgs/msg/Image"
                        + "[gz.msgs.Image",
                    ],
                ],
                remappings=[
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rgb_camera/camera_info",
                        ],
                        ["sensor/camera/color/camera_info"],
                    ),
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rgb_camera/image",
                        ],
                        ["sensor/camera/color/image_raw"],
                    ),
                ],
            ),
            # Lidar
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="rplidar_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rplidar/scan"
                        + "@sensor_msgs/msg/LaserScan"
                        + "[gz.msgs.LaserScan",
                    ],
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/rplidar/scan/points"
                        + "@sensor_msgs/msg/PointCloud2"
                        + "[gz.msgs.PointCloudPacked",
                    ],
                ],
                remappings=[
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rplidar/scan",
                        ],
                        ["sensor/lidar/scan"],
                    ),
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/rplidar/scan/points",
                        ],
                        ["sensor/lidar/scan/points"],
                    ),
                ],
            ),
            # IMU
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="imu_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    [
                        "/world/",
                        LaunchConfiguration("world"),
                        "/model/",
                        LaunchConfiguration("namespace"),
                        "/link/",
                        "base_link/sensor/imu_sensor/imu"
                        + "@sensor_msgs/msg/Imu"
                        + "[gz.msgs.IMU",
                    ]
                ],
                remappings=[
                    (
                        [
                            "/world/",
                            LaunchConfiguration("world"),
                            "/model/",
                            LaunchConfiguration("namespace"),
                            "/link/",
                            "base_link/sensor/imu_sensor/imu",
                        ],
                        ["sensor/imu/imu"],
                    )
                ],
            ),
        ]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ros_gz_bridge)
    return ld
