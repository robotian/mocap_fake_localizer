#!/usr/bin/env python3
"""A fake OptiTrack Motive with a fake robot, for testing natnet_ref_pose, calibrate_ref_offset and the web UI
without Motive or a robot (not installed; run it from the source tree).

Serves NatNet 2.8 like Motive 1.8, unicast on --address:1510, 100 Hz: one rigid body (--rigid-body) mounted at
--mount on a unicycle robot that follows <namespace>/cmd_vel (geometry_msgs/TwistStamped, 0.3 s timeout), plus
an untracked rigid body 'untracked'. Prints the base_link_offset that natnet_ref_pose should end up with.

  python3 test/fake_motive.py --namespace /fake --mount 0.2 -0.1 0.45 30 -21 0
  ros2 run mocap_fake_localizer natnet_ref_pose.py --ros-args -r __ns:=/fake -p server_ip:=127.0.0.1 -p rigid_body:=fake
  ros2 run mocap_fake_localizer calibrate_ref_offset.py --yes --base-height 0.15 --ros-args -r __ns:=/fake
"""
import argparse
import math
import select
import socket
import struct
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped


def rot_axis(axis, a):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.identity(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def quat(r):
    """(x, y, z, w) of any rotation matrix (Shepperd's method)."""
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return np.array([(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = np.zeros(4)
    q[i], q[j], q[k], q[3] = 0.25 * s, (r[j, i] + r[i, j]) / s, (r[k, i] + r[i, k]) / s, (r[k, j] - r[j, k]) / s
    return q


def message(message_id, payload):
    return struct.pack('<HH', message_id, len(payload)) + payload


def server_info():
    return message(1, b'Motive'.ljust(256, b'\0') + bytes([1, 79, 0, 0]) + bytes([2, 8, 0, 0]))


def model_def(name):
    out = struct.pack('<i', 2)
    for rb_id, rb_name in ((1, name), (2, 'untracked')):
        out += struct.pack('<i', 1) + rb_name.encode() + b'\0' + struct.pack('<ii3f', rb_id, -1, 0, 0, 0)
    return message(5, out)


def frame(number, pos, q):
    bodies = struct.pack('<i3f4fi', 1, *pos, *q, 0) + struct.pack('<fh', 0.0005, 1)
    bodies += struct.pack('<i3f4fi', 2, 0, 0, 0, 0, 0, 0, 1, 0) + struct.pack('<fh', 0.0, 0)
    tail = struct.pack('<ii', 0, 0) + struct.pack('<fIIdhi', 0, 0, 0, 0, 0, 0)  # skeletons, labeled markers, ...
    return message(7, struct.pack('<iii', number, 0, 0) + struct.pack('<i', 2) + bodies + tail)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--address', default='127.0.0.1')
    ap.add_argument('--namespace', default='/fake')
    ap.add_argument('--rigid-body', default='fake')
    ap.add_argument('--mount', type=float, nargs=6, default=[0.2, -0.1, 0.45, 30, -21, 0],
                    metavar=('X', 'Y', 'Z', 'YAW', 'PITCH', 'ROLL'),
                    help="the rigid body's pose in base_link (m, deg)")
    ap.add_argument('--base-height', type=float, default=0.15, help="base_link's height above the floor")
    ap.add_argument('--noise', type=float, default=0.0005, help='position noise (m, 1 sigma)')
    args = ap.parse_args()

    x, y, z, yaw, pitch, roll = args.mount
    mount_r = rot_axis([0, 0, 1], math.radians(yaw)) @ rot_axis([0, 1, 0], math.radians(pitch)) \
        @ rot_axis([1, 0, 0], math.radians(roll))
    mount_t = np.array([x, y, z])
    offset_t = -mount_r.T @ mount_t
    print('expected base_link_offset:', [round(float(v), 5) for v in [*offset_t, *quat(mount_r.T)]], flush=True)

    state = {'x': 1.0, 'y': 2.0, 'yaw': 0.7, 'v': 0.0, 'w': 0.0, 't_cmd': 0.0}
    lock = threading.Lock()

    def on_cmd(msg):
        with lock:
            state['v'], state['w'], state['t_cmd'] = msg.twist.linear.x, msg.twist.angular.z, time.time()

    def serve():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((args.address, 1510))
        clients, number, last = set(), 0, time.time()
        rng = np.random.default_rng(1)
        while True:
            while select.select([sock], [], [], 0)[0]:
                packet, addr = sock.recvfrom(65535)
                message_id = struct.unpack_from('<H', packet)[0]
                if message_id == 0:  # NAT_CONNECT
                    clients.add(addr)
                    sock.sendto(server_info(), addr)
                elif message_id == 4:  # NAT_REQUEST_MODELDEF
                    sock.sendto(model_def(args.rigid_body), addr)
            now = time.time()
            if now - last < 0.01:
                time.sleep(0.001)
                continue
            dt, last = now - last, now
            with lock:
                if now - state['t_cmd'] > 0.3:
                    state['v'] = state['w'] = 0.0
                state['x'] += state['v'] * math.cos(state['yaw']) * dt
                state['y'] += state['v'] * math.sin(state['yaw']) * dt
                state['yaw'] += state['w'] * dt
                base_r = rot_axis([0, 0, 1], state['yaw'])
                base_t = np.array([state['x'], state['y'], args.base_height])
            number += 1
            packet = frame(number, base_t + base_r @ mount_t + rng.normal(0, args.noise, 3), quat(base_r @ mount_r))
            for client in list(clients):
                sock.sendto(packet, client)

    threading.Thread(target=serve, daemon=True).start()
    rclpy.init()
    node = rclpy.create_node('fake_robot', namespace=args.namespace)
    node.create_subscription(TwistStamped, 'cmd_vel', on_cmd, 10)
    print(f'fake Motive on {args.address}:1510, robot on {args.namespace}/cmd_vel', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
