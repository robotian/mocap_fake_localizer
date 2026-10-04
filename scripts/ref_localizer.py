#!/usr/bin/env python3
"""The robot's only map -> odom publisher, from a reference pose (motion capture / simulator) or from GPS.

Frames:  ref_frame -> map -> odom -> base_link      (and ref_frame -> base_link_ref from the reference source)

ref_frame is a fixed world frame: Motive's coordinate frame in the lab (natnet_ref_pose.py), Isaac Sim's world
frame in simulation (the sim's SIM_REF_POSE). map is placed in it by the `anchor` parameter:

  fixed     map's pose in ref_frame from `map_pose_in_ref`, or from `anchor_file` when that is set
            (navigating on a saved map, e.g. one built earlier with SLAM: save its anchor with ~/save_anchor)
  start     the robot's first reference pose, flattened to the ground: map starts where the robot starts
  external  another node (SLAM, AMCL) owns map -> odom; ref_frame -> map is measured once from that node's
            map -> base_link and the reference pose at the same instant, so the reference pose is ground
            truth in the map. ~/save_anchor then stores it (to `anchor_file`, or ~/.ros/ref_anchor_<ns>.yaml)
            for later `fixed` runs with that file as `anchor_file`.

`source` picks what drives map -> odom (the EKF keeps odom -> base_link):

  ref       the reference pose: map -> odom = (ref->map)^-1 * ref->base_link * (odom->base_link)^-1
  gps       the GPS EKF's odometry/global (ekf_global_node, whose own TF output must be off)
  auto      ref while a fresh reference pose arrives (and map is anchored), otherwise gps
  external  nothing: SLAM / AMCL publishes map -> odom (anchor must then be `external` or `fixed`)

When the active source goes stale, the last map -> odom is kept, so the robot dead-reckons on its EKF.
source, anchor, map_pose_in_ref and anchor_file can be changed at runtime (ros2 param set). ~/status (std_msgs/String, JSON, 1 Hz) reports
the state; ~/save_anchor and ~/reset_anchor (std_srvs/Trigger) store / redo the anchor.
"""
import json
import math
import os
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener

SOURCES = ('auto', 'ref', 'gps', 'external')
ANCHORS = ('fixed', 'start', 'external')
HISTORY_S = 1.0  # reference / GPS poses kept to match against the odom TF's time


# --- 4x4 homogeneous transforms --------------------------------------------------------------------------------

def matrix(t, q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    m = np.identity(4)
    m[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    m[:3, 3] = t
    return m


def quaternion(m):
    """(x, y, z, w) of a rotation matrix (Shepperd's method)."""
    r = m[:3, :3]
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return tuple(q)


def flatten(m):
    """The same pose on the ground plane: x, y, yaw; z, roll, pitch zeroed."""
    yaw = math.atan2(m[1, 0], m[0, 0])
    return matrix((m[0, 3], m[1, 3], 0.0), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def pose_matrix(pose):
    p, o = pose.position, pose.orientation
    return matrix((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))


def transform_matrix(tf):
    t, r = tf.transform.translation, tf.transform.rotation
    return matrix((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))


def to_list(m):
    return [float(v) for v in m[:3, 3]] + [float(v) for v in quaternion(m)]


# --- node -------------------------------------------------------------------------------------------------------

class RefLocalizer(Node):
    def __init__(self):
        super().__init__('ref_localizer')
        p = self.declare_parameter
        self.source = p('source', 'auto').value
        self.anchor = p('anchor', 'fixed').value
        self.map_pose_in_ref = list(p('map_pose_in_ref', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).value)
        # read by the fixed anchor only when set; ~/save_anchor writes here (or to ~/.ros/ref_anchor_<ns>.yaml)
        self.anchor_file = os.path.expanduser(p('anchor_file', '').value)
        self.ref_frame = p('ref_frame', 'ref_frame').value
        self.map_frame = p('map_frame', 'map').value
        self.odom_frame = p('odom_frame', 'odom').value
        self.base_frame = p('base_link_frame', 'base_link').value
        ref_topic = p('ref_topic', 'ref_pose').value
        gps_topic = p('gps_topic', 'odometry/global').value
        self.ref_timeout = float(p('ref_timeout', 0.5).value)  # s: older = stale
        self.gps_timeout = float(p('gps_timeout', 2.0).value)
        rate = float(p('publish_rate', 30.0).value)  # Hz, map -> odom
        # map -> odom is stamped this far ahead, as AMCL / robot_localization do, so lookups of map -> base_link at
        # the latest odom time succeed between two publications
        self.tolerance = Duration(seconds=float(p('transform_tolerance', 0.1).value))
        self._check(self.source, self.anchor)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)

        self.refs = deque()  # (stamp ns, T_ref_base)
        self.gpss = deque()  # (stamp ns, T_map_base)
        self.T_ref_map = None
        self.T_map_odom = None
        self.active = None  # source currently driving map -> odom
        self.anchored_from = None

        self.create_subscription(Odometry, ref_topic, self._on_ref, 20)
        self.create_subscription(Odometry, gps_topic, self._on_gps, 20)
        self.status_pub = self.create_publisher(String, '~/status', 1)
        self.create_service(Trigger, '~/save_anchor', self._save_anchor)
        self.create_service(Trigger, '~/reset_anchor', self._reset_anchor)
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(1.0 / rate, self._update)
        self.create_timer(1.0, self._status)

        self._anchor_fixed()
        self.get_logger().info(f'source {self.source}, anchor {self.anchor}; {ref_topic} ({self.ref_frame}), '
                               f'{gps_topic} -> {self.map_frame} -> {self.odom_frame}')

    @staticmethod
    def _check(source, anchor):
        if source not in SOURCES:
            raise ValueError(f'source must be one of {SOURCES}, not {source!r}')
        if anchor not in ANCHORS:
            raise ValueError(f'anchor must be one of {ANCHORS}, not {anchor!r}')
        if anchor == 'external' and source in ('ref', 'auto'):
            # map would be placed from map -> base_link, which this node would itself be producing
            raise ValueError("anchor 'external' needs another map -> odom publisher: use source 'external' or 'gps'")

    def _on_params(self, params):
        source, anchor, pose, anchor_file = self.source, self.anchor, self.map_pose_in_ref, self.anchor_file
        for prm in params:
            if prm.name == 'source':
                source = prm.value
            elif prm.name == 'anchor':
                anchor = prm.value
            elif prm.name == 'map_pose_in_ref':
                pose = list(prm.value)
            elif prm.name == 'anchor_file':
                anchor_file = os.path.expanduser(prm.value)
        try:
            self._check(source, anchor)
            if len(pose) != 7:
                raise ValueError('map_pose_in_ref needs 7 values [x, y, z, qx, qy, qz, qw]')
        except ValueError as e:
            return SetParametersResult(successful=False, reason=str(e))
        reanchor = anchor != self.anchor or pose != self.map_pose_in_ref or anchor_file != self.anchor_file
        if source != self.source:
            self.get_logger().info(f'source {self.source} -> {source}')
        self.source, self.anchor, self.map_pose_in_ref, self.anchor_file = source, anchor, pose, anchor_file
        if reanchor:
            self.get_logger().info(f'anchor {anchor}: re-anchoring map')
            self.T_ref_map = None
            self._anchor_fixed()
        return SetParametersResult(successful=True)

    # --- anchoring: ref_frame -> map ---------------------------------------------------------------------------

    def _anchor_fixed(self):
        if self.anchor != 'fixed':
            return
        pose, origin = self.map_pose_in_ref, 'map_pose_in_ref'
        if self.anchor_file:
            try:
                with open(self.anchor_file) as f:
                    saved = yaml.safe_load(f) or {}
                pose, origin = list(saved['map_pose_in_ref']), self.anchor_file
            except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as e:
                self.get_logger().error(f'cannot read anchor_file {self.anchor_file} ({e}); using map_pose_in_ref')
        self._set_anchor(matrix(pose[:3], pose[3:]), origin)

    def _set_anchor(self, T_ref_map, origin):
        self.T_ref_map = T_ref_map
        self.anchored_from = origin
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ref_frame
        t.child_frame_id = self.map_frame
        v = to_list(T_ref_map)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = v[:3]
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = v[3:]
        self.tf_static.sendTransform(t)
        yaw = math.degrees(math.atan2(T_ref_map[1, 0], T_ref_map[0, 0]))
        self.get_logger().info(f'{self.map_frame} anchored in {self.ref_frame} ({origin}): '
                               f'x {v[0]:.3f} y {v[1]:.3f} z {v[2]:.3f} yaw {yaw:.1f} deg')

    def _anchor_external(self):
        """ref -> map from the external localizer's map -> base_link and the reference pose at one instant."""
        pair = self._pair_at(self.refs, self.map_frame, quiet=True)
        if pair:  # else SLAM / AMCL is not publishing map -> odom yet
            T_ref_base, T_map_base = pair
            self._set_anchor(flatten(flatten(T_ref_base) @ np.linalg.inv(flatten(T_map_base))),
                             f'{self.map_frame} -> {self.base_frame} from the external localizer')

    def _save_anchor(self, _request, response):
        if self.T_ref_map is None:
            response.success, response.message = False, f'{self.map_frame} is not anchored yet'
            return response
        data = {'map_pose_in_ref': to_list(self.T_ref_map), 'ref_frame': self.ref_frame,
                'map_frame': self.map_frame, 'anchored_from': self.anchored_from}
        path = self.anchor_file or os.path.expanduser(
            f"~/.ros/ref_anchor_{self.get_namespace().strip('/').replace('/', '_') or 'robot'}.yaml")
        try:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=None)
        except OSError as e:
            response.success, response.message = False, f'cannot write {path}: {e}'
            return response
        response.success, response.message = True, f'saved to {path}'
        self.get_logger().info(response.message)
        return response

    def _reset_anchor(self, _request, response):
        self.T_ref_map = None
        self._anchor_fixed()
        response.success = True
        response.message = (f'anchor {self.anchor}: ' +
                            ('reloaded' if self.anchor == 'fixed' else 'will re-anchor on the next reference pose'))
        return response

    # --- inputs -------------------------------------------------------------------------------------------------

    @staticmethod
    def _push(history, stamp_ns, T):
        history.append((stamp_ns, T))
        while history and stamp_ns - history[0][0] > HISTORY_S * 1e9:
            history.popleft()

    def _on_ref(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        T = pose_matrix(msg.pose.pose)
        self._push(self.refs, stamp, T)
        if self.T_ref_map is None and self.anchor == 'start':
            self._set_anchor(flatten(T), 'start pose')

    def _on_gps(self, msg):
        self._push(self.gpss, Time.from_msg(msg.header.stamp).nanoseconds, pose_matrix(msg.pose.pose))

    def _fresh(self, history, timeout):
        return bool(history) and (self.get_clock().now().nanoseconds - history[-1][0]) < timeout * 1e9

    def _choose(self):
        ref_ok = self._fresh(self.refs, self.ref_timeout) and self.T_ref_map is not None
        gps_ok = self._fresh(self.gpss, self.gps_timeout)
        if self.source == 'external':
            return None
        if self.source == 'ref':
            return 'ref' if ref_ok else None
        if self.source == 'gps':
            return 'gps' if gps_ok else None
        return 'ref' if ref_ok and self.anchor != 'external' else ('gps' if gps_ok else None)

    def _pair_at(self, history, frame, quiet=False):
        """The newest (T, frame -> base_link) pair at one instant: a pose no newer than that TF's latest. A pose
        and the TF of the same instant can arrive in either order, so matching only the newest pose can fail."""
        if not history:
            return None
        try:
            latest = self.tf_buffer.lookup_transform(frame, self.base_frame, Time())
        except TransformException as e:
            if not quiet:
                self._warn_throttled(f'no {frame} -> {self.base_frame} TF: {e}')
            return None
        latest_ns = Time.from_msg(latest.header.stamp).nanoseconds
        if latest_ns == 0:  # a static transform: valid at any time
            return history[-1][1], transform_matrix(latest)
        for stamp_ns, T in reversed(history):
            if stamp_ns <= latest_ns:
                try:
                    tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time(nanoseconds=stamp_ns))
                except TransformException:
                    break  # older than the TF buffer
                return T, transform_matrix(tf)
        return history[-1][1], transform_matrix(latest)  # pose newer than every TF: use the latest

    _last_warn = 0

    def _warn_throttled(self, text):
        now = self.get_clock().now().nanoseconds
        if now - self._last_warn > 5e9:
            self._last_warn = now
            self.get_logger().warn(text)

    # --- output -------------------------------------------------------------------------------------------------

    def _update(self):
        if self.T_ref_map is None and self.anchor == 'external':
            self._anchor_external()
        active = self._choose()
        if active != self.active and self.source != 'external':
            if active is None:
                self.get_logger().warn(f'no fresh {self.source if self.source != "auto" else "reference or GPS"} '
                                       f'pose: holding the last {self.map_frame} -> {self.odom_frame}')
            else:
                self.get_logger().info(f'{self.map_frame} -> {self.odom_frame} from {active}')
        self.active = active

        if active == 'ref':
            pair = self._pair_at(self.refs, self.odom_frame)
            if pair:
                T_ref_base, T_odom_base = pair
                T_map_base = np.linalg.inv(self.T_ref_map) @ T_ref_base
                self.T_map_odom = flatten(T_map_base) @ np.linalg.inv(flatten(T_odom_base))
        elif active == 'gps':
            pair = self._pair_at(self.gpss, self.odom_frame)
            if pair:
                T_map_base, T_odom_base = pair
                self.T_map_odom = flatten(T_map_base) @ np.linalg.inv(flatten(T_odom_base))

        if self.source == 'external' or self.T_map_odom is None:
            return
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + self.tolerance).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        v = to_list(self.T_map_odom)
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = v[:3]
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = v[3:]
        self.tf_pub.sendTransform(t)

    def _status(self):
        now = self.get_clock().now().nanoseconds

        def age(history):
            return round((now - history[-1][0]) / 1e9, 3) if history else None

        self.status_pub.publish(String(data=json.dumps({
            'source': self.source, 'anchor': self.anchor, 'active': self.active,
            'anchored': self.T_ref_map is not None, 'anchored_from': self.anchored_from,
            'map_pose_in_ref': to_list(self.T_ref_map) if self.T_ref_map is not None else None,
            'ref_age': age(self.refs), 'gps_age': age(self.gpss),
            'publishing': self.source != 'external' and self.T_map_odom is not None,
        })))


def main(args=None):
    rclpy.init(args=args)
    node = RefLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise  # otherwise shut down by a signal while spinning
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
