#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic', default='/wall_scan')
    cloud_topic = LaunchConfiguration('cloud_topic', default='/wall_scan_cloud')
    target_frame = LaunchConfiguration('target_frame', default='laser')
    publish_z = LaunchConfiguration('publish_z', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_topic',
            default_value=scan_topic,
            description='Input LaserScan topic'),
        DeclareLaunchArgument(
            'cloud_topic',
            default_value=cloud_topic,
            description='Output PointCloud2 topic'),
        DeclareLaunchArgument(
            'target_frame',
            default_value=target_frame,
            description='Frame for the published point cloud'),
        DeclareLaunchArgument(
            'publish_z',
            default_value=publish_z,
            description='Z value assigned to each point'),
        Node(
            package='wall_scan_to_pointcloud',
            executable='wall_scan_to_pointcloud_node',
            name='wall_scan_to_pointcloud',
            output='screen',
            parameters=[{
                'scan_topic': scan_topic,
                'cloud_topic': cloud_topic,
                'target_frame': target_frame,
                'publish_z': publish_z,
            }]),
    ])
