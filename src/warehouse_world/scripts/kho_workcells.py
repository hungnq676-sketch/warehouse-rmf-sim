#!/usr/bin/env python3
"""TRAM CAP / NHAN HANG cho nhiem vu giao hang (delivery) cua RMF.

  Dispenser 'conveyor'   (tai conv_pick):
      lay thung nam cuoi bang tai -> dat len khay robot, tha 1 thung moi o dau bang.
  Ingestor  'stack_area' (tai stack_1..3): lay thung tren khay -> xep len pallet trong 2 ben.
  Ingestor  'shipping'   (tai ship_drop):  lay thung tren khay -> xep o khu xuat hang.

Giao tiep RMF: /dispenser_requests, /dispenser_results, /dispenser_states
               /ingestor_requests,  /ingestor_results,  /ingestor_states
Thao tac Gazebo: goi dich vu /world/<world>/create va /remove bang lenh `gz service`.
"""
import math
import os
import re
import subprocess
import tempfile
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_fleet_msgs.msg import FleetState
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState

# Bang tai doc - phai khop kho_30x50.urdf.xacro / kho_30x50.launch.py
CONV_X, CONV_Y_LO, CONV_Y_HI, CONV_Z_LO, CONV_Z_HI = 12.6, 17.8, 24.3, 0.35, 1.3
PITCH = math.atan2(CONV_Z_HI - CONV_Z_LO, CONV_Y_HI - CONV_Y_LO)
BELT_LEN = math.hypot(CONV_Y_HI - CONV_Y_LO, CONV_Z_HI - CONV_Z_LO)
BOX = 0.4
TRAY_TOP = 0.42  # mat khay robot (amr.urdf.xacro)

# Cho xep hang cho moi diem tra hang: danh sach (x, y) tam cac o, mat do cao 'base_z'
STACK_PALLET_X = (8.6, 13.4)     # 2 cot pallet hai ben lan x = 11.0
PALLET_TOP = 0.15


def stack_slots(sy):
    """8 o / pallet (2x2, 2 tang) x 2 pallet o hai ben diem stack tai y = sy."""
    slots = []
    for layer in range(2):
        for px in STACK_PALLET_X:
            for dx in (-0.22, 0.22):
                for dy in (-0.25, 0.25):
                    slots.append((px + dx, sy + dy, PALLET_TOP + layer * (BOX + 0.01)))
    return slots


def shipping_slots():
    return [(x, y, layer * (BOX + 0.01))
            for layer in range(2) for x in (10.2, 10.7, 11.2) for y in (-20.0, -19.5, -19.0)]


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class Workcells(Node):
    def __init__(self):
        super().__init__('kho_workcells')
        self.declare_parameter('world', 'kho')
        self.declare_parameter('paths_file', '')
        self.declare_parameter('cargo_xacro', '')
        self.world = self.get_parameter('world').value
        with open(self.get_parameter('paths_file').value) as f:
            nodes = yaml.safe_load(f)['nodes']
        self.places = {n: (v['x'], v['y']) for n, v in nodes.items()}

        # file URDF thung hang dung cho lenh tao entity
        urdf = subprocess.check_output(['xacro', self.get_parameter('cargo_xacro').value,
                                        f'size:={BOX}'], text=True)
        fd, self.cargo_file = tempfile.mkstemp(suffix='.urdf', prefix='cargo_')
        with os.fdopen(fd, 'w') as f:
            f.write(urdf)

        # dispenser / ingestor: guid -> (loai, ham xu ly)
        self.dispensers = {'conveyor': self.dispense_conveyor}
        self.ingestors = {'stack_area': self.ingest_stack, 'shipping': self.ingest_shipping}
        self.busy = {}          # guid -> request_guid dang xu ly
        self.done = {}          # (target_guid, request_guid) -> status
        self.stack_count = {}   # ten diem -> so thung da xep
        self.loads = {}         # robot -> ten thung tren khay
        self.counter = int(time.time()) % 100000 * 10  # tranh trung ten khi chay lai
        self.fleet = None
        self.lock = threading.Lock()

        self.create_subscription(FleetState, '/fleet_states', self.on_fleet, 10)
        self.create_subscription(DispenserRequest, '/dispenser_requests', self.on_disp_req, 10)
        self.create_subscription(IngestorRequest, '/ingestor_requests', self.on_ing_req, 10)
        self.disp_res = self.create_publisher(DispenserResult, '/dispenser_results', 10)
        self.disp_state = self.create_publisher(DispenserState, '/dispenser_states', 10)
        self.ing_res = self.create_publisher(IngestorResult, '/ingestor_results', 10)
        self.ing_state = self.create_publisher(IngestorState, '/ingestor_states', 10)
        self.create_timer(1.0, self.publish_states)
        self.get_logger().info(f'Dispenser: {list(self.dispensers)}  Ingestor: {list(self.ingestors)}')

    # ---------- RMF ----------
    def on_fleet(self, msg):
        self.fleet = msg

    def on_disp_req(self, msg):
        self.process_request(msg, self.dispensers, self.publish_disp_result)

    def on_ing_req(self, msg):
        self.process_request(msg, self.ingestors, self.publish_ing_result)

    def process_request(self, msg, table, publish_result):
        fn = table.get(msg.target_guid)
        if fn is None:
            return
        # RMF dung chung request_guid cho buoc lay hang va tra hang cua 1 nhiem vu,
        # nen phai phan biet theo ca tram (target_guid)
        key = (msg.target_guid, msg.request_guid)
        if key in self.done:                       # RMF gui lai -> tra lai ket qua cu
            publish_result(msg, self.done[key])
            return
        with self.lock:
            if msg.target_guid in self.busy:
                return
            self.busy[msg.target_guid] = msg.request_guid
        publish_result(msg, DispenserResult.ACKNOWLEDGED)

        def work():
            try:
                ok = fn(msg)
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'{msg.target_guid}: {e}')
                ok = False
            status = DispenserResult.SUCCESS if ok else DispenserResult.FAILED
            self.done[key] = status
            with self.lock:
                self.busy.pop(msg.target_guid, None)
            publish_result(msg, status)
        threading.Thread(target=work, daemon=True).start()

    def publish_disp_result(self, req, status):
        self.disp_res.publish(DispenserResult(
            time=self.get_clock().now().to_msg(), request_guid=req.request_guid,
            source_guid=req.target_guid, status=status))

    def publish_ing_result(self, req, status):
        self.ing_res.publish(IngestorResult(
            time=self.get_clock().now().to_msg(), request_guid=req.request_guid,
            source_guid=req.target_guid, status=status))

    def publish_states(self):
        t = self.get_clock().now().to_msg()
        for g in self.dispensers:
            q = [self.busy[g]] if g in self.busy else []
            self.disp_state.publish(DispenserState(
                time=t, guid=g, mode=DispenserState.BUSY if q else DispenserState.IDLE,
                request_guid_queue=q))
        for g in self.ingestors:
            q = [self.busy[g]] if g in self.busy else []
            self.ing_state.publish(IngestorState(
                time=t, guid=g, mode=IngestorState.BUSY if q else IngestorState.IDLE,
                request_guid_queue=q))

    # ---------- tim robot ----------
    def robot_near(self, place_names, radius=1.5):
        if self.fleet is None:
            return None, None
        best = (None, None, radius)
        for r in self.fleet.robots:
            for p in place_names:
                px, py = self.places[p]
                d = math.hypot(r.location.x - px, r.location.y - py)
                if d < best[2]:
                    best = (r, p, d)
        return best[0], best[1]

    # ---------- Gazebo ----------
    def gz(self, service, reqtype, req):
        out = subprocess.run(
            ['gz', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        return 'true' in out.stdout

    def spawn_box(self, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        qx, qy, qz, qw = quat(roll, pitch, yaw)
        req = (f'sdf_filename: "{self.cargo_file}" name: "{name}" allow_renaming: false '
               f'pose: {{position: {{x: {x:.3f} y: {y:.3f} z: {z:.3f}}} '
               f'orientation: {{x: {qx:.5f} y: {qy:.5f} z: {qz:.5f} w: {qw:.5f}}}}}')
        return self.gz('create', 'gz.msgs.EntityFactory', req)

    def remove(self, name):
        return self.gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')

    def model_xyz(self, name):
        out = subprocess.run(['gz', 'model', '-m', name, '-p'], capture_output=True, text=True).stdout
        m = re.search(r'XYZ.*\n\s*\[([-\d.e ]+)\]', out)
        return [float(v) for v in m.group(1).split()] if m else None

    def conveyor_boxes(self):
        names = re.findall(r'- (cargo_\d+)', subprocess.run(
            ['gz', 'model', '--list'], capture_output=True, text=True).stdout)
        boxes = []
        for n in names:
            p = self.model_xyz(n)
            if p and abs(p[0] - CONV_X) < 0.6 and CONV_Y_LO - 0.5 < p[1] < CONV_Y_HI + 0.5:
                boxes.append((p[1], n))
        return sorted(boxes)

    def next_name(self, prefix):
        self.counter += 1
        return f'{prefix}_{self.counter}'

    # ---------- xu ly ----------
    def dispense_conveyor(self, req):
        robot, _ = self.robot_near(['conv_pick'])
        if robot is None:
            self.get_logger().warn('conveyor: khong co robot o conv_pick')
            return False
        boxes = self.conveyor_boxes()
        if boxes:
            self.remove(boxes[0][1])
        # thung len khay robot
        load = self.next_name(f'load_{robot.name}')
        loc = robot.location
        self.spawn_box(load, loc.x, loc.y, TRAY_TOP + BOX / 2 + 0.02, yaw=loc.yaw)
        self.loads[robot.name] = load
        # thung moi tu lo cap hang (dau cao bang tai)
        s = BELT_LEN - 0.5
        y = CONV_Y_LO + s * math.cos(PITCH) - (BOX / 2 + 0.02) * math.sin(PITCH)
        z = CONV_Z_LO + s * math.sin(PITCH) + (BOX / 2 + 0.02) * math.cos(PITCH)
        self.spawn_box(self.next_name('cargo'), CONV_X, y, z, pitch=PITCH, yaw=-math.pi / 2)
        self.get_logger().info(f'conveyor -> {robot.name}: {load}')
        return True

    def take_load(self, robot):
        load = self.loads.pop(robot.name, None)
        if load:
            self.remove(load)
        return load

    def ingest_stack(self, req):
        robot, place = self.robot_near(['stack_1', 'stack_2', 'stack_3'])
        if robot is None:
            self.get_logger().warn('stack_area: khong co robot o stack_1..3')
            return False
        self.take_load(robot)
        slots = stack_slots(self.places[place][1])
        k = self.stack_count.get(place, 0)
        if k < len(slots):
            x, y, z = slots[k]
            self.spawn_box(self.next_name(f'stored_{place}'), x, y, z + BOX / 2 + 0.01)
            self.stack_count[place] = k + 1
        self.get_logger().info(f'{robot.name} -> {place} (o {k + 1})')
        return True

    def ingest_shipping(self, req):
        robot, _ = self.robot_near(['ship_drop'])
        if robot is None:
            self.get_logger().warn('shipping: khong co robot o ship_drop')
            return False
        self.take_load(robot)
        slots = shipping_slots()
        k = self.stack_count.get('ship_drop', 0)
        if k < len(slots):
            x, y, z = slots[k]
            self.spawn_box(self.next_name('shipped'), x, y, z + BOX / 2 + 0.01)
            self.stack_count['ship_drop'] = k + 1
        self.get_logger().info(f'{robot.name} -> ship_drop (o {k + 1})')
        return True


def main():
    rclpy.init()
    node = Workcells()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    os.unlink(node.cargo_file)
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
