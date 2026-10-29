#!/usr/bin/env python3
"""Gui nhiem vu GIAO HANG cho RMF (task dispatcher se chon robot phu hop nhat).

Chay trong terminal da `source kho_env.sh` (dat ROS_LOCALHOST_ONLY=1 giong kho_rmf.launch.py).

  # 1 nhiem vu: lay o bang tai, tra o khu chong hang (diem stack_2)
  ros2 run warehouse_world dispatch_delivery.py -d stack_2

  # nhieu nhiem vu cung luc (RMF chia cho nhieu robot)
  ros2 run warehouse_world dispatch_delivery.py -d stack_1 stack_3 ship_drop

  # tra o khu xuat hang
  ros2 run warehouse_world dispatch_delivery.py -d ship_drop -dh shipping

  # tu dong: cu 40 giay gui 1 nhiem vu, xoay vong stack_1 / stack_2 / stack_3 / ship_drop
  ros2 run warehouse_world dispatch_delivery.py --auto 40
"""
import argparse
import itertools
import json
import time
import uuid

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rmf_task_msgs.msg import ApiRequest  # noqa: E402

DROPOFFS = [('stack_1', 'stack_area'), ('stack_2', 'stack_area'),
            ('stack_3', 'stack_area'), ('ship_drop', 'shipping')]


class Dispatcher(Node):
    def __init__(self):
        super().__init__('dispatch_delivery')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ApiRequest, '/task_api_requests', qos)

    def send(self, pickup, pickup_handler, dropoff, dropoff_handler):
        payload = [{'sku': 'thung_hang', 'quantity': 1}]
        request = {
            'type': 'dispatch_task_request',
            'request': {
                # khong dat unix_millis_earliest_start_time: RMF dung gio mo phong
                # (use_sim_time), de trong = bat dau ngay
                'category': 'delivery',
                'description': {
                    'pickup': {'place': pickup, 'handler': pickup_handler, 'payload': payload},
                    'dropoff': {'place': dropoff, 'handler': dropoff_handler, 'payload': payload},
                },
            },
        }
        msg = ApiRequest(json_msg=json.dumps(request),
                         request_id=f'giao_hang_{uuid.uuid4().hex[:8]}')
        self.pub.publish(msg)
        self.get_logger().info(f'{msg.request_id}: {pickup} -> {dropoff}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--pickup', default='conv_pick')
    ap.add_argument('-ph', '--pickup-handler', default='conveyor')
    ap.add_argument('-d', '--dropoff', nargs='+', default=['stack_1'],
                    help='1 hoac nhieu diem tra hang, vd: -d stack_1 stack_3 ship_drop')
    ap.add_argument('-dh', '--dropoff-handler', default=None,
                    help='mac dinh: shipping cho ship_drop, stack_area cho stack_*')
    ap.add_argument('--auto', type=float, default=0.0,
                    help='gui lien tuc, moi N giay 1 nhiem vu (xoay vong cac diem tra hang)')
    a, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = Dispatcher()
    # cho toi khi task dispatcher / fleet adapter da ket noi vao topic
    t_end = time.time() + 60.0  # nhieu node Nav2 -> discovery co the cham
    while node.pub.get_subscription_count() == 0 and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.pub.get_subscription_count() == 0:
        node.get_logger().error(
            'Khong thay RMF dispatcher tren /task_api_requests. kho_rmf.launch.py da chay chua? '
            'Terminal nay da `source kho_env.sh` (ROS_LOCALHOST_ONLY=1) chua?')
    time.sleep(0.5)
    try:
        if a.auto > 0:
            for place, handler in itertools.cycle(DROPOFFS):
                node.send(a.pickup, a.pickup_handler, place, handler)
                end = time.time() + a.auto
                while time.time() < end:
                    rclpy.spin_once(node, timeout_sec=0.5)
        else:
            for place in a.dropoff:
                handler = a.dropoff_handler or ('shipping' if place == 'ship_drop' else 'stack_area')
                node.send(a.pickup, a.pickup_handler, place, handler)
                time.sleep(0.3)
            for _ in range(6):  # giu node them 3 s de tin nhan toi noi
                rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
