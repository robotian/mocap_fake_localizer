#!/usr/bin/env python3
"""Measure natnet_ref_pose's base_link_offset: where base_link is in the Motive rigid body's frame.

Motive puts a rigid body's pivot at its markers' centroid, oriented however the body was created, so ref_pose
(rigid body * base_link_offset) is only base_link once the offset is right. This drives the robot and corrects
the offset it currently uses, in three phases:

  1. tilt      standing on a flat floor: base_link's roll and pitch must be zero
  2. heading   forward and back: the direction of travel is base_link's +x
  3. position  one turn in place: the centre of the circle ref_pose draws is the rotation centre, taken as
               base_link (a differential drive turns about its axle midpoint; a skid-steer robot about roughly its
               centre, which can shift with load and floor)

z is left as it is unless --base-height (base_link's height above Motive's floor, z = 0) is given: the localizer
flattens the pose to the ground, so only x, y and yaw matter for navigation.

Needs a clear, flat area (--distance ahead and behind, the robot's footprint to turn) and natnet_ref_pose
running. It drives the robot through cmd_vel (geometry_msgs/TwistStamped) only with --yes. Run it in the
robot's namespace:

  ros2 run mocap_fake_localizer calibrate_ref_offset.py --yes --ros-args -r __ns:=/j100_0921
  ... --write config.yaml   also write the result as a params file (e.g. mtu32_bringup's
                            config/ref_localization/<namespace>.yaml), --apply set it on the running client

Run it again afterwards: a correct offset gives a circle radius under ~1 cm and corrections near zero.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node

CLIENT = 'natnet_ref_pose'


# --- rotations (x, y, z, w) and 4x4 transforms -------------------------------------------------------------------

def rot(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat(r):
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


def transform(t, r):
    m = np.identity(4)
    m[:3, :3], m[:3, 3] = r, t
    return m


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_between(a, b):
    """Shortest rotation taking unit vector a to unit vector b."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.identity(3)  # parallel (antiparallel would mean the body is upside down: not handled)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.identity(3) + k + k @ k * ((1 - c) / np.dot(v, v))


def yaw_of(r):
    return math.atan2(r[1, 0], r[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fit_circle(xy):
    """Least-squares circle (Kasa): centre, radius, RMS residual."""
    x, y = xy[:, 0], xy[:, 1]
    a = np.column_stack([x, y, np.ones_like(x)])
    sol, *_ = np.linalg.lstsq(a, x * x + y * y, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    resid = np.hypot(x - cx, y - cy) - r
    return np.array([cx, cy]), r, float(np.sqrt(np.mean(resid ** 2)))


# --- node ---------------------------------------------------------------------------------------------------------

class Calibrator(Node):
    def __init__(self, args):
        super().__init__('calibrate_ref_offset')
        self.args = args
        self.samples = []  # (t, position (3,), rotation (3, 3)) of ref_pose
        self.recording = False
        self.last = None
        self.create_subscription(Odometry, 'ref_pose', self._on_pose, 50)
        self.cmd = self.create_publisher(TwistStamped, 'cmd_vel', 10)

    def _on_pose(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        sample = (self.get_clock().now().nanoseconds * 1e-9, np.array([p.x, p.y, p.z]), rot((o.x, o.y, o.z, o.w)))
        self.last = sample
        if self.recording:
            self.samples.append(sample)

    def spin_for(self, seconds, twist=None):
        """Spin (publishing twist at 20 Hz if given) for `seconds` of the node's clock (sim time in the sim)."""
        clock = self.get_clock()
        end = clock.now().nanoseconds + seconds * 1e9
        next_cmd = 0.0
        while rclpy.ok() and clock.now().nanoseconds < end:
            if twist is not None and time.monotonic() >= next_cmd:
                self.drive(*twist)
                next_cmd = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)

    def drive(self, v, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd.publish(msg)

    def stop(self):
        for _ in range(5):
            self.drive(0.0, 0.0)
            self.spin_for(0.05)

    def record(self, seconds, twist=None):
        self.samples, self.recording = [], True
        self.spin_for(seconds, twist)
        self.recording = False
        return self.samples

    def param_client(self, srv_type, name):
        node = f"{self.get_namespace().rstrip('/')}/{CLIENT}"
        client = self.create_client(srv_type, f'{node}/{name}')
        return client if client.wait_for_service(timeout_sec=2.0) else None

    def current_offset(self):
        client = self.param_client(GetParameters, 'get_parameters')
        if client is None:
            return None
        future = client.call_async(GetParameters.Request(names=['base_link_offset']))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        values = future.result().values if future.result() else []
        return list(values[0].double_array_value) if values and len(values[0].double_array_value) == 7 else None

    def apply_offset(self, offset):
        client = self.param_client(SetParameters, 'set_parameters')
        if client is None:
            return False, f'no {CLIENT} node in this namespace'
        value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=offset)
        future = client.call_async(SetParameters.Request(parameters=[Parameter(name='base_link_offset', value=value)]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        result = future.result().results[0] if future.result() else None
        return (result.successful, result.reason) if result else (False, 'no answer')


def run(node, args):
    log = print
    if node.current_offset() is None and not args.offset:
        log(f'no {CLIENT} in this namespace (e.g. the simulator, whose ref_pose is base_link itself): the '
            'correction is relative to the identity offset')
    offset = args.offset or node.current_offset() or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    O = transform(offset[:3], rot(offset[3:]))

    log('waiting for ref_pose ...')
    node.spin_for(2.0)
    if node.last is None:
        sys.exit('no ref_pose: is natnet_ref_pose running and the rigid body tracked?')

    # 1. tilt ------------------------------------------------------------------------------------------------
    log('1/3 tilt: standing still for 3 s')
    still = node.record(3.0)
    r_mean = np.mean([s[2] for s in still], axis=0)
    u, _, vt = np.linalg.svd(r_mean)
    r_mean = u @ vt  # closest rotation to the average
    # base_link's z axis must be the world's: correct with the shortest rotation that does it (yaw untouched)
    c_tilt = rot_between(np.array([0.0, 0.0, 1.0]), r_mean.T @ np.array([0.0, 0.0, 1.0]))
    tilt = math.degrees(math.acos(min(1.0, (r_mean @ np.array([0, 0, 1.0]))[2])))
    log(f'    reported z axis is {tilt:.2f} deg off vertical')

    # 2. heading ---------------------------------------------------------------------------------------------
    duration = args.distance / args.speed
    log(f'2/3 heading: {args.distance:.2f} m forward and back at {args.speed:.2f} m/s')
    errs = []
    for sign in (1.0, -1.0):
        node.spin_for(0.5)  # settle
        run_ = node.record(duration, (sign * args.speed, 0.0))
        node.stop()
        if len(run_) < 10:
            sys.exit('too few ref_pose samples while driving')
        d = run_[-1][1][:2] - run_[0][1][:2]
        if np.linalg.norm(d) < 0.5 * args.distance:
            sys.exit(f'moved only {np.linalg.norm(d):.2f} m of {args.distance:.2f}: is cmd_vel reaching the robot?')
        travel = math.atan2(sign * d[1], sign * d[0])  # direction of base_link's +x
        yaws = [yaw_of(s[2] @ c_tilt) for s in run_]
        mean_yaw = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
        errs.append(wrap(travel - mean_yaw))
        log(f"    {'forward' if sign > 0 else 'back'}: travel {math.degrees(travel):.1f} deg, reported yaw "
            f'{math.degrees(mean_yaw):.1f} deg')
    if abs(wrap(errs[0] - errs[1])) > math.radians(5):
        log(f'    warning: forward and back disagree by {math.degrees(abs(wrap(errs[0] - errs[1]))):.1f} deg '
            '(drifting / slipping?)')
    d_yaw = math.atan2(np.mean(np.sin(errs)), np.mean(np.cos(errs)))
    c_rot = c_tilt @ rot_z(d_yaw)
    log(f'    yaw correction {math.degrees(d_yaw):.2f} deg')

    # 3. position --------------------------------------------------------------------------------------------
    turn = 2 * math.pi * 1.1 / args.turn_rate
    log(f'3/3 position: one turn in place at {args.turn_rate:.2f} rad/s')
    node.spin_for(0.5)
    spin = node.record(turn, (0.0, args.turn_rate))
    node.stop()
    yaws = np.unwrap([yaw_of(s[2] @ c_rot) for s in spin])
    if abs(yaws[-1] - yaws[0]) < math.radians(300):
        sys.exit(f'turned only {math.degrees(abs(yaws[-1] - yaws[0])):.0f} deg: is cmd_vel reaching the robot?')
    xy = np.array([s[1][:2] for s in spin])
    centre, radius, rms = fit_circle(xy)
    # base_link (the centre, at the reported height) in the corrected frame, averaged over the turn
    t = np.mean([(s[2] @ c_rot).T @ (np.array([*centre, s[1][2]]) - s[1]) for s in spin], axis=0)
    log(f'    circle radius {radius * 100:.1f} cm (fit RMS {rms * 1000:.1f} mm)')
    if args.base_height is not None:
        mean_z = np.mean([s[1][2] for s in still])
        t[2] += args.base_height - mean_z  # the corrected frame is level: its z is the world's
        log(f'    height: reported z {mean_z:.3f} m -> base_link at {args.base_height:.3f} m')

    O_new = O @ transform(np.zeros(3), c_rot) @ transform(t, np.identity(3))
    new = [round(float(v), 5) for v in [*O_new[:3, 3], *quat(O_new[:3, :3])]]
    log(f'\nbase_link_offset correction: x {t[0] * 100:+.1f} y {t[1] * 100:+.1f} z {t[2] * 100:+.1f} cm, '
        f'tilt {tilt:.2f} deg, yaw {math.degrees(d_yaw):+.2f} deg')
    log(f'base_link_offset: {new}')
    return new


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--yes', action='store_true', help='allowed to drive the robot (required)')
    ap.add_argument('--distance', type=float, default=0.6, help='m driven forward, then back (default 0.6)')
    ap.add_argument('--speed', type=float, default=0.15, help='m/s (default 0.15)')
    ap.add_argument('--turn-rate', type=float, default=0.5, help='rad/s for the turn in place (default 0.5)')
    ap.add_argument('--base-height', type=float, help="also set z: base_link's height above the floor (m)")
    ap.add_argument('--offset', type=float, nargs=7, help='the current offset, if not read from the client')
    ap.add_argument('--write', metavar='YAML', help='write the result as a natnet_ref_pose params file')
    ap.add_argument('--apply', action='store_true', help='set it on the running natnet_ref_pose')
    args, ros_args = ap.parse_known_args()
    if not args.yes:
        sys.exit('this drives the robot (forward/back, one turn in place): clear the area, then pass --yes')

    rclpy.init(args=ros_args)
    node = Calibrator(args)
    try:
        new = run(node, args)
        if args.write:
            data = {f'/**/{CLIENT}': {'ros__parameters': {'base_link_offset': new}}}
            try:
                with open(args.write) as f:  # keep the file's other parameters (e.g. rigid_body)
                    old = yaml.safe_load(f) or {}
                old.setdefault(f'/**/{CLIENT}', {}).setdefault('ros__parameters', {})['base_link_offset'] = new
                data = old
            except FileNotFoundError:
                pass
            with open(args.write, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=None)
            print(f'written to {args.write}')
        if args.apply:
            ok, reason = node.apply_offset(new)
            print('applied to the running client' if ok else f'not applied: {reason}')
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
