"""KHO 30x50 + 4 ROBOT AMR (Nav2) + OPEN-RMF

    ros2 launch warehouse_world kho_rmf.launch.py
    ros2 launch warehouse_world kho_rmf.launch.py num_robots:=2 headless:=true show_rviz:=false

Sau khi moi thu len (~40 s), gui nhiem vu giao hang:
    ros2 run warehouse_world dispatch_delivery.py -d stack_2
    ros2 run warehouse_world dispatch_delivery.py --auto 40

Thanh phan:
  Gazebo (kho + bang tai)  <-- kho_30x50.launch.py
  Moi robot amrN: spawn, ros_gz_bridge, robot_state_publisher, Nav2 (AMCL + planner + RPP)
  RMF: rmf_traffic_schedule, rmf_task_dispatcher, full_control fleet adapter 'amr'
  amr_fleet_driver.py  : RMF <-> Nav2, pin mo phong
  kho_workcells.py     : dispenser bang tai + ingestor khu chong hang / xuat hang
"""
import os
import sys
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = get_package_share_directory('warehouse_world')
sys.path.insert(0, os.path.join(PKG, 'scripts'))
from make_rmf_nav_graph import convert  # noqa: E402

WORLD = 'kho'
FLEET = 'amr'
# ten, tram sac xuat phat, mau vong nhan dien
ROBOTS = [('amr1', 'charge_1', '0.95 0.45 0.05 1'),
          ('amr2', 'charge_2', '0.10 0.55 0.95 1'),
          ('amr3', 'charge_3', '0.20 0.75 0.25 1'),
          ('amr4', 'charge_4', '0.80 0.20 0.70 1')]


def robot_nodes(name, start, color, nav2_params, map_yaml):
    x, y, yaw = start
    urdf = xacro.process_file(os.path.join(PKG, 'urdf', 'amr.urdf.xacro'),
                              mappings={'name': name, 'color': color}).toxml()
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', WORLD, '-name', name, '-string', urdf,
                            '-x', str(x), '-y', str(y), '-z', '0.02', '-Y', str(yaw)])

    gz = f'/model/{name}'
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name=f'bridge_{name}',
        output='screen',
        arguments=[f'{gz}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                   f'{gz}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                   f'{gz}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   f'{gz}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   f'{gz}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[(f'{gz}/cmd_vel', f'/{name}/cmd_vel'),
                    (f'{gz}/odometry', f'/{name}/odom'),
                    (f'{gz}/tf', f'/{name}/tf'),
                    (f'{gz}/scan', f'/{name}/scan'),
                    (f'{gz}/joint_states', f'/{name}/joint_states')],
        parameters=[{'use_sim_time': True}],
    )

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               namespace=name, output='log',
               parameters=[{'robot_description': urdf, 'use_sim_time': True}],
               remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'namespace': name, 'use_namespace': 'True', 'slam': 'False',
                          'map': map_yaml, 'use_sim_time': 'True', 'params_file': nav2_params,
                          'autostart': 'True', 'use_composition': 'True'}.items())

    return [spawn, bridge, rsp], [nav2]


def setup(context):
    n = int(LaunchConfiguration('num_robots').perform(context))
    robots = ROBOTS[:max(1, min(n, len(ROBOTS)))]
    tmp = tempfile.mkdtemp(prefix='kho_rmf_')

    paths_file = os.path.join(PKG, 'config', 'kho_30x50_paths.yaml')
    with open(paths_file) as f:
        paths = yaml.safe_load(f)

    # nav graph cho RMF sinh tu cung file duong di ve tren san
    nav_graph = os.path.join(tmp, 'kho_nav_graph.yaml')
    with open(nav_graph, 'w') as f:
        yaml.safe_dump(convert(paths), f, default_flow_style=None, sort_keys=False)

    map_yaml = os.path.join(PKG, 'maps', 'kho_30x50.yaml')
    with open(os.path.join(PKG, 'config', 'nav2_amr.yaml')) as f:
        nav2_base = yaml.safe_load(f)

    spawn_actions, nav_actions, starts, colors = [], [], [], []
    for name, charger, color in robots:
        node = paths['nodes'][charger]
        start = (node['x'], node['y'], node.get('yaw', 0.0))
        # file tham so Nav2 rieng: vi tri ban dau cho AMCL
        p = yaml.safe_load(yaml.safe_dump(nav2_base))
        p['amcl']['ros__parameters']['initial_pose'] = {
            'x': start[0], 'y': start[1], 'z': 0.0, 'yaw': start[2]}
        params_file = os.path.join(tmp, f'nav2_{name}.yaml')
        with open(params_file, 'w') as f:
            yaml.safe_dump(p, f)
        a, b = robot_nodes(name, start, color, params_file, map_yaml)
        spawn_actions += a
        nav_actions += b
        starts += list(map(float, start))
        colors.append(color)

    rmf = [
        Node(package='rmf_traffic_ros2', executable='rmf_traffic_schedule', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='rmf_task_ros2', executable='rmf_task_dispatcher', output='screen',
             parameters=[{'use_sim_time': True, 'bidding_time_window': 2.0}]),
        Node(package='rmf_fleet_adapter', executable='full_control', name='amr_fleet_adapter',
             output='screen',
             parameters=[{
                 'use_sim_time': True,
                 'fleet_name': FLEET,
                 'nav_graph_file': nav_graph,
                 # dong hoc (dung de RMF lap lich, nen cham hon toc do that cua Nav2)
                 'linear_velocity': 0.4, 'angular_velocity': 0.6,
                 'linear_acceleration': 0.5, 'angular_acceleration': 1.5,
                 'footprint_radius': 0.35, 'vicinity_radius': 0.9, 'reversible': False,
                 # pin (nho de thay robot tu di sac trong vai phut mo phong;
                 # khop voi drain_* / charge_rate cua amr_fleet_driver.py)
                 'battery_voltage': 24.0, 'battery_capacity': 2.0,
                 'battery_charging_current': 26.4, 'mass': 30.0, 'inertia': 10.0,
                 'friction_coefficient': 0.22, 'ambient_power_drain': 20.0,
                 'tool_power_drain': 0.0, 'drain_battery': True,
                 'recharge_threshold': 0.25, 'recharge_soc': 1.0,
                 # nhiem vu
                 'perform_deliveries': True, 'perform_loop': False, 'perform_cleaning': False,
                 'finishing_request': 'charge',
                 'discovery_timeout': 60.0,
             }]),
        Node(package='warehouse_world', executable='amr_fleet_driver.py', output='screen',
             parameters=[{'use_sim_time': True, 'fleet_name': FLEET, 'paths_file': paths_file,
                          'robots': [r[0] for r in robots], 'starts': starts,
                          'colors': colors}]),
        Node(package='warehouse_world', executable='kho_workcells.py', output='screen',
             parameters=[{'use_sim_time': True, 'world': WORLD, 'paths_file': paths_file,
                          'cargo_xacro': os.path.join(PKG, 'urdf', 'cargo_box.urdf.xacro')}]),
    ]

    return [TimerAction(period=8.0, actions=spawn_actions),
            TimerAction(period=14.0, actions=nav_actions),
            TimerAction(period=20.0, actions=rmf)]


def generate_launch_description():
    # Luu y: tham so cua launch con (rviz:=false) lan ra launch nay, nen RViz o day
    # dung ten rieng 'show_rviz'
    kho = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG, 'launch', 'kho_30x50.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless'), 'rviz': 'false',
                          'conveyor_speed': LaunchConfiguration('conveyor_speed')}.items())

    # RViz: ban do, kho 3D, nav graph + robot + lo trinh (tu /rmf_markers)
    map_tf = Node(package='tf2_ros', executable='static_transform_publisher',
                  arguments=['--frame-id', 'map', '--child-frame-id', 'warehouse_floor'])
    rviz = Node(package='rviz2', executable='rviz2', output='log',
                arguments=['-d', os.path.join(PKG, 'rviz', 'kho_rmf.rviz')],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(LaunchConfiguration('show_rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('localhost_only', default_value='1',
                              description='1 = chi giao tiep ROS trong may nay'),
        # Chi giao tiep trong may nay: tranh /clock, TF... cua may khac cung mang LAN
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', LaunchConfiguration('localhost_only')),
        DeclareLaunchArgument('num_robots', default_value='4', description='1..4 robot'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        DeclareLaunchArgument('conveyor_speed', default_value='0.2'),
        kho,
        map_tf,
        GroupAction([OpaqueFunction(function=setup)]),
        rviz,
    ])
