"""Kho 30m x 50m: bang tai doc, 3 hanh lang, lan robot canh loi di nguoi, 3 tram sac.

    ros2 launch warehouse_world kho_30x50.launch.py
    ros2 launch warehouse_world kho_30x50.launch.py rviz:=true
    ros2 launch warehouse_world kho_30x50.launch.py animated_workers:=true   # nguoi di lai

Dieu khien bang tai (m/s, duong = hang chay xuong dau thap, 0 = dung):
    ros2 topic pub --once /conveyor/speed std_msgs/msg/Float64 "{data: 0.2}"
"""
import math
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Phai khop voi conv_* trong kho_30x50.urdf.xacro
CONV_X, CONV_Y_LO, CONV_Y_HI, CONV_Z_LO, CONV_Z_HI, CONV_W = 12.6, 17.8, 24.3, 0.35, 1.3, 0.8
BELT_T = 0.1
CONV_TOPIC = '/model/conveyor/link/conveyor_belt/track_cmd_vel'

RUN, DROP = CONV_Y_HI - CONV_Y_LO, CONV_Z_HI - CONV_Z_LO
PITCH = math.atan2(DROP, RUN)
BELT_LEN = math.hypot(RUN, DROP)
TOP_CENTER = (CONV_X, (CONV_Y_LO + CONV_Y_HI) / 2, (CONV_Z_LO + CONV_Z_HI) / 2)
NORMAL = (0.0, -math.sin(PITCH), math.cos(PITCH))   # phap tuyen mat bang
UPHILL = (0.0, math.cos(PITCH), math.sin(PITCH))    # huong len doc

BOX = 0.4
BOX_COLORS = ['0.72 0.53 0.32 1', '0.58 0.40 0.22 1', '0.80 0.62 0.38 1']


def on_belt(s, lift):
    """Diem cach dau thap s (m) doc theo bang, nang len 'lift' theo phap tuyen."""
    low = [TOP_CENTER[i] - BELT_LEN / 2 * UPHILL[i] for i in range(3)]
    return [low[i] + s * UPHILL[i] + lift * NORMAL[i] for i in range(3)]


def spawn_node(name, urdf, xyz, rpy=(0.0, 0.0, 0.0)):
    return Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', name, '-string', urdf,
                   '-x', f'{xyz[0]:.4f}', '-y', f'{xyz[1]:.4f}', '-z', f'{xyz[2]:.4f}',
                   '-R', f'{rpy[0]:.4f}', '-P', f'{rpy[1]:.4f}', '-Y', f'{rpy[2]:.4f}'])


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_world')
    urdf_dir = os.path.join(pkg, 'urdf')
    animated = LaunchConfiguration('animated_workers')

    world = PythonExpression([
        "'", os.path.join(pkg, 'worlds', 'kho_30x50_actors.sdf'), "' if '", animated,
        "' == 'true' else '", os.path.join(pkg, 'worlds', 'kho_30x50.sdf'), "'"])
    static_walkers = PythonExpression(["'false' if '", animated, "' == 'true' else 'true'"])

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(urdf_dir, 'kho_30x50.urdf.xacro'),
                 ' static_walkers:=', static_walkers]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [
            PythonExpression(["'-s ' if '", LaunchConfiguration('headless'),
                              "' == 'true' else ''"]),
            '-r ', world]}.items(),
    )

    # Kho (tinh). Namespace /warehouse de dung chung rviz/warehouse.rviz
    state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='warehouse', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn_kho = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'kho', '-topic', '/warehouse/robot_description'],
    )

    # Mat bang tai doc: +X cua link huong xuong doc (ve phia Nam)
    belt_rpy = (0.0, PITCH, -math.pi / 2)
    conveyor_urdf = xacro.process_file(
        os.path.join(urdf_dir, 'conveyor.urdf.xacro'),
        mappings={'length': f'{BELT_LEN:.4f}', 'width': str(CONV_W)}).toxml()
    spawn_conveyor = spawn_node('conveyor', conveyor_urdf,
                                on_belt(BELT_LEN / 2, -BELT_T / 2), belt_rpy)

    # 5 thung hang tren bang, thung dau tien nam sat tam chan o dau thap
    box_xacro = os.path.join(urdf_dir, 'cargo_box.urdf.xacro')
    spawn_boxes = []
    for i, s in enumerate((0.25, 1.4, 2.6, 3.8, 5.0)):
        urdf = xacro.process_file(box_xacro, mappings={
            'size': str(BOX), 'color': BOX_COLORS[i % len(BOX_COLORS)]}).toxml()
        spawn_boxes.append(spawn_node(f'cargo_{i}', urdf, on_belt(s, BOX / 2 + 0.015), belt_rpy))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   f'{CONV_TOPIC}@std_msgs/msg/Float64]gz.msgs.Double'],
        remappings=[(CONV_TOPIC, '/conveyor/speed')],
    )

    start_conveyor = ExecuteProcess(
        cmd=['gz', 'topic', '-t', CONV_TOPIC, '-m', 'gz.msgs.Double',
             '-p', ['data: ', LaunchConfiguration('conveyor_speed')]],
        output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'warehouse.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('animated_workers', default_value='false',
                              description='Nhan vien di lai (tai mesh tu Gazebo Fuel lan dau)'),
        DeclareLaunchArgument('conveyor_speed', default_value='0.2',
                              description='Toc do bang tai luc khoi dong (m/s)'),
        DeclareLaunchArgument('rviz', default_value='false', description='Mo them RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Chi chay server Gazebo, khong mo giao dien'),
        gz_sim,
        state_pub,
        spawn_kho,
        bridge,
        TimerAction(period=3.0, actions=[spawn_conveyor]),
        TimerAction(period=6.0, actions=spawn_boxes),
        TimerAction(period=10.0, actions=[start_conveyor]),
        rviz,
    ])
