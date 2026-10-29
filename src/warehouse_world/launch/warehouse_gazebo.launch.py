"""Mo Gazebo Sim, spawn nha kho + bang tai + thung hang.

    ros2 launch warehouse_world warehouse_gazebo.launch.py
    ros2 launch warehouse_world warehouse_gazebo.launch.py rviz:=true conveyor_speed:=0.0

Dieu khien bang tai (m/s, duong = hang chay ve phia robot, 0 = dung):
    ros2 topic pub --once /conveyor/speed std_msgs/msg/Float64 "{data: 0.3}"
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Phai khop voi conv_* trong warehouse.urdf.xacro
CONV_X, CONV_Y_END, CONV_LEN, CONV_W, CONV_H = 23.0, 16.5, 8.0, 0.8, 0.8
CONV_TOPIC = '/model/conveyor/link/conveyor_belt/track_cmd_vel'

BOX = 0.4  # canh thung hang (m)
BOX_COLORS = ['0.72 0.53 0.32 1', '0.58 0.40 0.22 1', '0.80 0.62 0.38 1']
PILE_X, PILE_Y, PALLET_H = 19.8, 18.6, 0.15  # pallet dong hang (khop xacro)


def cargo_poses():
    """(ten, x, y, z) cho tung thung hang dong."""
    poses = []
    # 5 thung tren bang tai
    for i in range(5):
        poses.append((f'cargo_belt_{i}', CONV_X, CONV_Y_END + 0.8 + i * 1.4,
                      CONV_H + BOX / 2 + 0.02))
    # dong 2 x 2 x 2 thung tren pallet canh bang tai
    off = BOX / 2 + 0.005
    for k in range(2):
        for i, dx in enumerate((-off, off)):
            for j, dy in enumerate((-off, off)):
                poses.append((f'cargo_pile_{k}{i}{j}', PILE_X + dx, PILE_Y + dy,
                              PALLET_H + BOX / 2 + 0.01 + k * (BOX + 0.01)))
    return poses


def spawn_node(name, urdf, x, y, z, yaw=0.0):
    return Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', name, '-string', urdf,
                   '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)])


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_world')
    urdf_dir = os.path.join(pkg, 'urdf')
    world_file = os.path.join(pkg, 'worlds', 'warehouse.sdf')

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(urdf_dir, 'warehouse.urdf.xacro'),
                 ' aisle_width:=', LaunchConfiguration('aisle_width'),
                 ' bays_per_row:=', LaunchConfiguration('bays_per_row'),
                 ' with_roof:=', LaunchConfiguration('with_roof')]),
        value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': [PythonExpression(
            ["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
            f'-r {world_file}']}.items(),
    )

    # Nha kho (tinh) - namespace rieng de khong dung /robot_description cua robot
    state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='warehouse', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn_warehouse = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'warehouse', '-topic', '/warehouse/robot_description'],
    )

    # Mat bang tai: tam o giua chieu dai, quay -90 do de +X huong ve dau ra (Nam)
    conveyor_urdf = xacro.process_file(
        os.path.join(urdf_dir, 'conveyor.urdf.xacro'),
        mappings={'length': str(CONV_LEN), 'width': str(CONV_W)}).toxml()
    spawn_conveyor = spawn_node('conveyor', conveyor_urdf, CONV_X,
                                CONV_Y_END + CONV_LEN / 2, CONV_H - 0.05, -1.5708)

    # Thung hang: spawn sau khi nha kho va bang tai da co de khong roi xuong san
    box_xacro = os.path.join(urdf_dir, 'cargo_box.urdf.xacro')
    spawn_boxes = []
    for n, (name, x, y, z) in enumerate(cargo_poses()):
        urdf = xacro.process_file(box_xacro, mappings={
            'size': str(BOX), 'color': BOX_COLORS[n % len(BOX_COLORS)]}).toxml()
        spawn_boxes.append(spawn_node(name, urdf, x, y, z))

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   f'{CONV_TOPIC}@std_msgs/msg/Float64]gz.msgs.Double'],
        remappings=[(CONV_TOPIC, '/conveyor/speed')],
    )

    # Chay bang tai voi toc do ban dau (sau khi thung hang da nam yen)
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
        DeclareLaunchArgument('aisle_width', default_value='4.0',
                              description='Be rong hanh lang lay hang (m)'),
        DeclareLaunchArgument('bays_per_row', default_value='11',
                              description='So o ke moi day (moi o dai 2.7 m)'),
        DeclareLaunchArgument('with_roof', default_value='false',
                              description='Them mai nha kho'),
        DeclareLaunchArgument('conveyor_speed', default_value='0.3',
                              description='Toc do bang tai luc khoi dong (m/s)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Mo them RViz'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Chi chay server Gazebo, khong mo giao dien'),
        gz_sim,
        state_pub,
        spawn_warehouse,
        bridge,
        TimerAction(period=3.0, actions=[spawn_conveyor]),
        TimerAction(period=6.0, actions=spawn_boxes),
        TimerAction(period=10.0, actions=[start_conveyor]),
        rviz,
    ])
