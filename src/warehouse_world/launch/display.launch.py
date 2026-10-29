"""Xem nhanh nha kho trong RViz (khong can Gazebo).

    ros2 launch warehouse_world display.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_world')
    xacro_file = os.path.join(pkg, 'urdf', 'warehouse.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' aisle_width:=', LaunchConfiguration('aisle_width'),
                 ' bays_per_row:=', LaunchConfiguration('bays_per_row')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('aisle_width', default_value='4.0'),
        DeclareLaunchArgument('bays_per_row', default_value='11'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='warehouse',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg, 'rviz', 'warehouse.rviz')],
        ),
    ])
