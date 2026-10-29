#!/usr/bin/env python3
"""FLEET DRIVER: cau noi giua RMF (full_control fleet adapter) va Nav2 cua tung robot.

  RMF  --/robot_path_requests-->  driver  --NavigateToPose-->  /<robot>/navigate_to_pose (Nav2)
  RMF  --/robot_mode_requests-->  driver  (tam dung / chay tiep)
  RMF  <--/fleet_states---------  driver  <-- /<robot>/odom + TF map->odom (AMCL)

Pin duoc mo phong o day: tieu hao khi chay / dung yen, sac khi robot dung yen tren
1 vertex 'charge' trong config/kho_30x50_paths.yaml.
Them /rmf_markers (MarkerArray) de xem robot, pin, lo trinh va nav graph trong RViz.
"""
import math

import rclpy
import yaml
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState, Location, ModeRequest, PathRequest, RobotMode, RobotState
from std_msgs.msg import ColorRGBA
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_sec(t):
    return t.sec + t.nanosec * 1e-9


def parse_color(s):
    r, g, b, a = (float(v) for v in s.split())
    return ColorRGBA(r=r, g=g, b=b, a=a)


class Robot:
    def __init__(self, node, name, x, y, yaw, color):
        self.node = node
        self.name = name
        self.color = color
        # map -> odom: truoc khi AMCL chay, odom trung voi vi tri spawn
        self.map_odom = (x, y, yaw)
        self.odom = (0.0, 0.0, 0.0)
        self.battery = 1.0
        self.path = []            # cac Location con lai, path[0] la dich ke tiep
        self.prev_xy = (x, y)     # waypoint vua di qua
        self.task_id = ''
        self.mode_request_id = 0
        self.paused = False
        self.goal_seq = 0         # tang moi lan gui goal moi
        self.goal_handle = None
        self.goal_busy = False
        self.retry_at = 0.0
        self.moving = False
        cb = node.cb_group
        self.nav = ActionClient(node, NavigateToPose, f'/{name}/navigate_to_pose', callback_group=cb)
        node.create_subscription(Odometry, f'/{name}/odom', self.on_odom, 10, callback_group=cb)
        node.create_subscription(TFMessage, f'/{name}/tf', self.on_tf, 50, callback_group=cb)

    # ----- vi tri -----
    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        v = msg.twist.twist
        self.moving = abs(v.linear.x) > 0.02 or abs(v.angular.z) > 0.05

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                tr = t.transform
                self.map_odom = (tr.translation.x, tr.translation.y, yaw_from_quat(tr.rotation))

    def pose(self):
        tx, ty, tyaw = self.map_odom
        ox, oy, oyaw = self.odom
        c, s = math.cos(tyaw), math.sin(tyaw)
        yaw = math.atan2(math.sin(tyaw + oyaw), math.cos(tyaw + oyaw))
        return tx + c * ox - s * oy, ty + s * ox + c * oy, yaw

    # ----- Nav2 -----
    def cancel_goal(self):
        self.goal_seq += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_busy = False

    def send_goal(self, loc):
        if not self.nav.server_is_ready():
            self.retry_at = self.node.now() + 1.0
            return
        self.goal_seq += 1
        seq = self.goal_seq
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(loc.x)
        goal.pose.pose.position.y = float(loc.y)
        goal.pose.pose.orientation.z = math.sin(loc.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(loc.yaw / 2.0)
        self.goal_busy = True
        fut = self.nav.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_response(f, seq))

    def on_goal_response(self, fut, seq):
        if seq != self.goal_seq:
            return
        handle = fut.result()
        if not handle.accepted:
            self.goal_busy = False
            self.retry_at = self.node.now() + 1.0
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, fut, seq):
        if seq != self.goal_seq:
            return  # goal cu da bi thay the
        status = fut.result().status
        self.goal_handle = None
        self.goal_busy = False
        if status == 4 and self.path:  # STATUS_SUCCEEDED
            reached = self.path.pop(0)
            self.prev_xy = (reached.x, reached.y)
        else:
            self.node.get_logger().warn(f'{self.name}: Nav2 that bai (status {status}), thu lai')
            self.retry_at = self.node.now() + 1.0


class FleetDriver(Node):
    def __init__(self):
        super().__init__('amr_fleet_driver')
        self.cb_group = ReentrantCallbackGroup()
        self.declare_parameter('fleet_name', 'amr')
        self.declare_parameter('level_name', 'L1')
        self.declare_parameter('paths_file', '')
        self.declare_parameter('robots', ['amr1'])
        self.declare_parameter('starts', [0.0, 0.0, 0.0])        # x,y,yaw lien tiep
        self.declare_parameter('colors', ['0.95 0.45 0.05 1'])
        # Toc do pin (ti le / giay thoi gian mo phong). Nen khop voi thong so pin
        # khai bao cho fleet adapter (battery_capacity, ambient_power_drain...) de RMF
        # du doan dung luc nao can di sac.
        self.declare_parameter('drain_moving', 0.0006)   # 0.06 %/s khi chay (~3.6 %/phut)
        self.declare_parameter('drain_idle', 0.00012)    # 0.012 %/s khi dung (20 W / 48 Wh)
        self.declare_parameter('charge_rate', 0.004)     # 0.4 %/s khi sac (~4 phut day)
        self.declare_parameter('charger_radius', 0.5)

        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.fleet = gp('fleet_name')
        self.level = gp('level_name')
        with open(gp('paths_file')) as f:
            self.graph = yaml.safe_load(f)
        self.vnames = list(self.graph['nodes'].keys())
        self.vxy = [(n['x'], n['y']) for n in self.graph['nodes'].values()]
        self.chargers = [(n['x'], n['y']) for n in self.graph['nodes'].values()
                         if n.get('type') == 'charge']

        starts = gp('starts')
        colors = gp('colors')
        self.robots = {}
        for i, name in enumerate(gp('robots')):
            x, y, yaw = starts[3 * i: 3 * i + 3]
            self.robots[name] = Robot(self, name, x, y, yaw, parse_color(colors[i % len(colors)]))

        self.state_pub = self.create_publisher(FleetState, '/fleet_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rmf_markers', 10)
        self.create_subscription(PathRequest, '/robot_path_requests', self.on_path, 10,
                                 callback_group=self.cb_group)
        self.create_subscription(ModeRequest, '/robot_mode_requests', self.on_mode, 10,
                                 callback_group=self.cb_group)
        self.last_t = None
        # vong lap chinh chay tuan tu (khong song song) de tinh pin / dt dung
        self.create_timer(0.1, self.step, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.publish_markers, callback_group=self.cb_group)
        self.get_logger().info(f'Fleet "{self.fleet}": {list(self.robots)}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- lenh tu RMF -----
    def on_path(self, msg):
        r = self.robots.get(msg.robot_name)
        if msg.fleet_name != self.fleet or r is None:
            return
        if msg.task_id == r.task_id and r.path and len(msg.path) and \
                to_sec(msg.path[-1].t) == to_sec(r.path[-1].t):
            return  # gui lai y het, bo qua
        try:
            if r.task_id and int(msg.task_id) < int(r.task_id):
                return  # lenh cu den muon, bo qua
        except ValueError:
            pass
        r.cancel_goal()
        r.task_id = msg.task_id
        x, y, _ = r.pose()
        path = list(msg.path)
        # bo diem dau neu robot da o do (diem xuat phat cua lo trinh)
        if path and math.hypot(path[0].x - x, path[0].y - y) < 0.3:
            path.pop(0)
        r.path = path
        r.prev_xy = (x, y)
        r.retry_at = 0.0
        self.get_logger().info(f'{r.name}: lo trinh moi {len(path)} diem (task "{msg.task_id}")')

    def on_mode(self, msg):
        r = self.robots.get(msg.robot_name)
        if msg.fleet_name != self.fleet or r is None:
            return
        r.mode_request_id = msg.mode.mode_request_id
        r.task_id = msg.task_id
        if msg.mode.mode == RobotMode.MODE_PAUSED:
            r.paused = True
            r.cancel_goal()
        elif msg.mode.mode == RobotMode.MODE_MOVING:
            r.paused = False

    # ----- vong lap chinh -----
    def step(self):
        now = self.now()
        dt = 0.0 if self.last_t is None else min(1.0, max(0.0, now - self.last_t))
        self.last_t = now
        fs = FleetState(name=self.fleet)
        for r in self.robots.values():
            x, y, yaw = r.pose()
            near_charger = any(math.hypot(x - cx, y - cy) < self.get_parameter('charger_radius').value
                               for cx, cy in self.chargers)
            mode = RobotMode.MODE_IDLE

            if r.paused:
                mode = RobotMode.MODE_PAUSED
            elif r.path:
                mode = RobotMode.MODE_MOVING
                tgt = r.path[0]
                same_spot = math.hypot(tgt.x - r.prev_xy[0], tgt.y - r.prev_xy[1]) < 0.1
                if not r.goal_busy and now >= r.retry_at:
                    if same_spot or math.hypot(tgt.x - x, tgt.y - y) < 0.15:
                        # diem cho / quay dau: doi toi thoi diem RMF hen
                        if now >= to_sec(tgt.t):
                            r.path.pop(0)
                            r.prev_xy = (tgt.x, tgt.y)
                    else:
                        r.send_goal(tgt)
            elif near_charger and r.battery < 1.0 and not r.moving:
                mode = RobotMode.MODE_CHARGING

            # pin
            if mode == RobotMode.MODE_CHARGING:
                r.battery = min(1.0, r.battery + self.get_parameter('charge_rate').value * dt)
            else:
                drain = 'drain_moving' if r.moving else 'drain_idle'
                r.battery = max(0.0, r.battery - self.get_parameter(drain).value * dt)

            st = RobotState(name=r.name, model='amr', task_id=r.task_id,
                            battery_percent=float(r.battery * 100.0))
            st.mode = RobotMode(mode=mode, mode_request_id=r.mode_request_id)
            st.location = self.location(x, y, yaw)
            st.path = [self.location(p.x, p.y, p.yaw, p.t) for p in r.path]
            fs.robots.append(st)
        self.state_pub.publish(fs)

    def location(self, x, y, yaw, t=None):
        loc = Location(x=float(x), y=float(y), yaw=float(yaw), level_name=self.level)
        loc.t = t if t is not None else self.get_clock().now().to_msg()
        d = [math.hypot(x - vx, y - vy) for vx, vy in self.vxy]
        loc.index = int(d.index(min(d)))
        return loc

    # ----- hien thi RViz -----
    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def mk(ns, mid, mtype):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        lanes = mk('nav_graph', 0, Marker.LINE_LIST)
        lanes.scale.x = 0.05
        lanes.color = ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.8)
        for a, b in self.graph['edges']:
            na, nb = self.graph['nodes'][a], self.graph['nodes'][b]
            lanes.points += [Point(x=float(na['x']), y=float(na['y']), z=0.05),
                             Point(x=float(nb['x']), y=float(nb['y']), z=0.05)]
        arr.markers.append(lanes)

        for i, r in enumerate(self.robots.values()):
            x, y, yaw = r.pose()
            body = mk('robots', i, Marker.CYLINDER)
            body.pose.position = Point(x=x, y=y, z=0.25)
            body.scale.x = body.scale.y = 0.6
            body.scale.z = 0.5
            body.color = r.color
            arr.markers.append(body)

            label = mk('labels', i, Marker.TEXT_VIEW_FACING)
            label.pose.position = Point(x=x, y=y, z=1.1)
            label.scale.z = 0.45
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = f'{r.name} {r.battery * 100:.0f}%'
            arr.markers.append(label)

            path = mk('paths', i, Marker.LINE_STRIP)
            path.scale.x = 0.12
            path.color = r.color
            path.points = [Point(x=x, y=y, z=0.1)] + \
                [Point(x=float(p.x), y=float(p.y), z=0.1) for p in r.path]
            if len(path.points) < 2:
                path.action = Marker.DELETE
            arr.markers.append(path)
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = FleetDriver()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
