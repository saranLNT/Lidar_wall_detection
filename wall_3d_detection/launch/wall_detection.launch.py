from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_frame_arg = DeclareLaunchArgument(
        "laser",
        default_value="laser",
        description="Lidar frame to transform from"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization"
    )

    config = PathJoinSubstitution(
        [FindPackageShare("wall_3d_detection"), "config", "wall_detection.yaml"]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wall_3d_detection"), "rviz", "wall_detection.rviz"]
    )

    cloud_accumulator_node = Node(
        package="wall_3d_detection",
        executable="cloud_accumulator",
        name="cloud_accumulator_wall_scan",
        output="screen",
        parameters=[
            config,
            {
                "fixed_frame": "odom",
                "lidar_frame": LaunchConfiguration("laser"),
            },
        ]
    )

    wall_scan_to_pointcloud_node = Node(
        package="wall_scan_to_pointcloud",
        executable="wall_scan_to_pointcloud_node",
        name="wall_scan_to_pointcloud",
        output="screen",
        parameters=[{
            "scan_topic": "/wall_scan",
            "cloud_topic": "/wall_scan_cloud",
            "target_frame": LaunchConfiguration("laser"),
            "publish_z": 0.0,
        }]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )

    return LaunchDescription([
        lidar_frame_arg,
        # use_rviz_arg,
        wall_scan_to_pointcloud_node,
        cloud_accumulator_node,
        # rviz_node
    ])