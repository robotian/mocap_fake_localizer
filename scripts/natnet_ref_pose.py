#!/usr/bin/env python3
"""NatNet (OptiTrack Motive) client: one rigid body -> the robot's reference pose.

Runs on the robot itself and talks to Motive directly, so every pose is stamped with the robot's own clock
when it arrives: no clock sync between Motive's PC and the robot. (Motive 1.8's NatNet 2.8 "latency" field
holds the frame timestamp, not a latency, so there is nothing to subtract.)

Publishes
  ref_pose (nav_msgs/Odometry)      base_link's pose in ref_frame (Motive's world frame, Z-up)
  tf: ref_frame -> base_link_ref    the same pose as its own TF branch (base_link already has a parent)

Frames that Motive reports as untracked are dropped, not published as zeros.

Only the standard library and rclpy: the NatNet SDK is a binary for x86_64, and the robots' computers are not
all x86_64. Parses NatNet 2.x (Motive 1.x; tested against Motive 1.8 / NatNet 2.8) and 3.x/4.x frame data.
"""
import math
import select
import socket
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# NatNet message ids
NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7


# --- quaternions (x, y, z, w) ---------------------------------------------------------------------------------

def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rotate(q, v):
    x, y, z, _ = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return (x, y, z)


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q) if n > 0 else (0.0, 0.0, 0.0, 1.0)


# Motive's Y-up world -> a Z-up one: +90 deg about x (Motive's +z, toward the viewer, becomes -y).
Y_UP_TO_Z_UP = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))


# --- NatNet parsing -------------------------------------------------------------------------------------------

class Reader:
    def __init__(self, data, offset=0):
        self.b = data
        self.o = offset

    def take(self, fmt):
        v = struct.unpack_from('<' + fmt, self.b, self.o)
        self.o += struct.calcsize('<' + fmt)
        return v

    def i32(self):
        return self.take('i')[0]

    def skip(self, n):
        self.o += n

    def cstr(self):
        end = self.b.index(b'\0', self.o)
        s = self.b[self.o:end].decode(errors='replace')
        self.o = end + 1
        return s


def at_least(version, major, minor=0):
    return tuple(version[:2]) >= (major, minor)


def parse_server_info(data):
    """NAT_SERVERINFO payload: app name, app version, NatNet version."""
    name = data[4:260].split(b'\0')[0].decode(errors='replace')
    return name, tuple(data[260:264]), tuple(data[264:268])


def parse_model_def(data, version):
    """NAT_MODELDEF -> {rigid body id: name}. Stops at the first dataset type it doesn't know."""
    r = Reader(data, 4)
    names = {}
    for _ in range(r.i32()):
        kind = r.i32()
        if at_least(version, 4, 1):
            r.skip(4)  # byte count of this description
        if kind == 0:  # marker set
            r.cstr()
            for _ in range(r.i32()):
                r.cstr()
        elif kind == 1:  # rigid body
            name = r.cstr() if at_least(version, 2) else ''
            rb_id, _parent = r.take('ii')
            r.skip(12)  # offset from parent
            if at_least(version, 3):
                n = r.i32()
                r.skip(n * (12 + 4))  # marker offsets + active labels
                if at_least(version, 4):
                    for _ in range(n):
                        r.cstr()  # marker names
            names[rb_id] = name
        else:  # skeletons, force plates, ...: rigid bodies come first in practice
            break
    return names


def parse_frame(data, version):
    """NAT_FRAMEOFDATA -> (frame number, {id: (pos, quat, mean_error, tracked)}). Stops after the rigid bodies."""
    r = Reader(data, 4)
    sized = at_least(version, 4, 1)  # 4.1+: a byte count after every section's element count
    frame = r.i32()

    n = r.i32()  # marker sets
    if sized:
        r.skip(4)
    for _ in range(n):
        r.cstr()
        r.skip(12 * r.i32())
    n = r.i32()  # unlabeled ("other") markers
    if sized:
        r.skip(4)
    r.skip(12 * n)

    bodies = {}
    n = r.i32()
    if sized:
        r.skip(4)
    for _ in range(n):
        rb_id, x, y, z, qx, qy, qz, qw = r.take('i7f')
        if not at_least(version, 3):
            k = r.i32()
            r.skip(k * 12)
            if at_least(version, 2):
                r.skip(k * 8)
        err = r.take('f')[0] if at_least(version, 2) else 0.0
        tracked = True
        if at_least(version, 2, 6):
            tracked = bool(r.take('h')[0] & 0x01)
        bodies[rb_id] = ((x, y, z), (qx, qy, qz, qw), err, tracked)
    return frame, bodies


# --- node -----------------------------------------------------------------------------------------------------

class NatNetRefPose(Node):
    def __init__(self):
        super().__init__('natnet_ref_pose')
        p = self.declare_parameter
        self.server_ip = p('server_ip', '192.168.50.80').value
        self.command_port = p('command_port', 1510).value
        self.data_port = p('data_port', 1511).value
        # Both of Motive's transmission types work: the client always registers for unicast (one stream per
        # client, the right choice over WiFi) and also joins this multicast group (Motive's default; empty = don't)
        self.multicast_group = p('multicast_group', '239.255.42.99').value
        # empty = the robot's namespace, so a rigid body named after the robot needs no configuration
        rigid_body = p('rigid_body', '').value
        self.rigid_body = rigid_body or self.get_namespace().strip('/')
        self.ref_frame = p('ref_frame', 'ref_frame').value
        self.child_frame = p('child_frame', 'base_link_ref').value
        self.publish_tf = p('publish_tf', True).value
        # Motive's up axis ('z' or 'y'); the published pose is always Z-up
        self.up_axis = p('up_axis', 'z').value.lower()
        # base_link's pose in the rigid body's frame [x, y, z, qx, qy, qz, qw]: the rigid body's pivot is where
        # Motive put it (the markers' centroid), not base_link
        offset = list(p('base_link_offset', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]).value)
        self.offset_t = tuple(offset[:3])
        self.offset_q = q_normalize(tuple(offset[3:7]))
        self.max_rate = float(p('max_rate', 0.0).value)  # Hz, 0 = every frame Motive sends
        self.position_std = float(p('position_std', 0.002).value)  # m, for the covariance
        self.orientation_std = float(p('orientation_std', 0.005).value)  # rad
        self.timeout = float(p('timeout', 1.0).value)  # s without a tracked pose -> warn, reconnect

        if self.up_axis not in ('y', 'z'):
            raise ValueError(f"up_axis must be 'y' or 'z', not {self.up_axis!r}")
        if not self.rigid_body:
            raise ValueError('no rigid_body parameter and no namespace to default it to')

        self.pub = self.create_publisher(Odometry, 'ref_pose', 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        self.version = None  # NatNet version from NAT_SERVERINFO; frames are skipped until it arrives
        self.names = {}
        self.rb_id = None
        self.next_pub = None  # ns: with max_rate, frames before this are skipped
        # wall times: they only pace reconnects, requests and warnings, never stamp anything
        self.last_tracked = time.monotonic()
        self.last_frame = time.monotonic()
        self.last_modeldef_request = time.monotonic()
        self.resolved = False  # a model definition has been matched against rigid_body
        self.frames = 0
        self.untracked = 0
        self.missing = 0  # frames without the rigid body
        self.silent = 0  # 5 s reports in a row without a frame
        self.warned = set()

        self.local_ip = self._local_ip()
        self.cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd.bind((self.local_ip, 0))
        self.data = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.data.bind(('', self.data_port))
        if self.multicast_group:
            mreq = socket.inet_aton(self.multicast_group) + socket.inet_aton(self.local_ip)
            self.data.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        self.get_logger().info(
            f"Motive {self.server_ip}:{self.command_port} from {self.local_ip}, unicast"
            f"{' + multicast ' + self.multicast_group if self.multicast_group else ''}, data port "
            f"{self.data_port}; rigid body '{self.rigid_body}' -> ref_pose ({self.ref_frame} -> {self.child_frame})")

        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        self.create_timer(5.0, self._report)

    def _local_ip(self):
        """This machine's address on the route to Motive (no packet is sent)."""
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect((self.server_ip, self.command_port))
            return s.getsockname()[0]
        finally:
            s.close()

    def _send(self, message_id, sock=None):
        try:
            (sock or self.cmd).sendto(struct.pack('<HH', message_id, 0), (self.server_ip, self.command_port))
        except OSError as e:
            self._warn_once('send', f'cannot reach Motive at {self.server_ip}: {e}')

    def _connect(self):
        # NAT_CONNECT answers with NAT_SERVERINFO (NatNet version). In unicast mode Motive streams the frames to
        # the socket the NAT_CONNECT came from (seen with Motive 1.8), so the data socket sends one too.
        self._send(NAT_CONNECT)
        self._send(NAT_CONNECT, self.data)
        self._send(NAT_REQUEST_MODELDEF)

    def _warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def _receive_loop(self):
        self._connect()
        last_connect = time.monotonic()
        while self.running and rclpy.ok():
            try:
                ready, _, _ = select.select([self.data, self.cmd], [], [], 0.2)
            except (OSError, ValueError):
                break
            for sock in ready:
                try:
                    packet, addr = sock.recvfrom(65535)
                except OSError:
                    continue
                if addr[0] != self.server_ip or len(packet) < 4:
                    continue
                try:
                    self._handle(packet)
                except (struct.error, ValueError, IndexError) as e:
                    self._warn_once(f'parse{packet[0]}', f'cannot parse NatNet message {packet[0]}: {e}')
                except Exception:
                    if not rclpy.ok():
                        return  # shut down (Ctrl-C) while publishing
                    raise
            now = time.monotonic()
            if now - self.last_frame > self.timeout and now - last_connect > self.timeout:
                self._connect()  # Motive restarted or dropped this unicast client
                last_connect = now

    def _handle(self, packet):
        message_id = struct.unpack_from('<H', packet)[0]
        if message_id == NAT_SERVERINFO:
            app, app_version, version = parse_server_info(packet)
            if version != self.version:
                self.version = version
                self.get_logger().info(f"{app} {'.'.join(map(str, app_version))}, NatNet "
                                       f"{'.'.join(map(str, version))}")
        elif self.version is None:
            return  # the frame layout depends on the version
        elif message_id == NAT_MODELDEF:
            self.names = parse_model_def(packet, self.version)
            ids = [i for i, n in self.names.items() if n == self.rigid_body]
            rb_id = ids[0] if ids else None
            if rb_id != self.rb_id or not self.resolved:
                self.rb_id = rb_id
                self.resolved = True
                if rb_id is None:
                    self.get_logger().error(f"Motive has no rigid body '{self.rigid_body}' "
                                            f"(it has: {', '.join(sorted(self.names.values())) or 'none'})")
                else:
                    self.get_logger().info(f"rigid body '{self.rigid_body}' is id {rb_id}")
        elif message_id == NAT_FRAMEOFDATA:
            self._frame(packet)

    def _frame(self, packet):
        stamp = self.get_clock().now()
        _frame_number, bodies = parse_frame(packet, self.version)
        self.frames += 1
        self.last_frame = time.monotonic()
        if self.rb_id is None or self.rb_id not in bodies:
            self.missing += 1
            # not known yet, or renamed / added in Motive: ask again, at most every 2 s
            if self.last_frame - self.last_modeldef_request > 2.0:
                self.last_modeldef_request = self.last_frame
                self._send(NAT_REQUEST_MODELDEF)
            return
        pos, quat, _err, tracked = bodies[self.rb_id]
        if not tracked:
            self.untracked += 1
            return
        self.last_tracked = time.monotonic()
        if self.max_rate > 0:
            if self.next_pub is not None and stamp.nanoseconds < self.next_pub:
                return
            # due times step by the period (not "period after the last one"), so the average rate is max_rate
            period = int(1e9 / self.max_rate)
            due = (self.next_pub or stamp.nanoseconds) + period
            self.next_pub = due if due > stamp.nanoseconds else stamp.nanoseconds + period  # after a gap
        self._publish(stamp, pos, q_normalize(quat))

    def _publish(self, stamp, pos, quat):
        if self.up_axis == 'y':
            pos = q_rotate(Y_UP_TO_Z_UP, pos)
            quat = q_mul(q_mul(Y_UP_TO_Z_UP, quat), q_conj(Y_UP_TO_Z_UP))
        # T_ref_base = T_ref_rb * T_rb_base
        rot = q_rotate(quat, self.offset_t)
        pos = (pos[0] + rot[0], pos[1] + rot[1], pos[2] + rot[2])
        quat = q_normalize(q_mul(quat, self.offset_q))

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.ref_frame
        msg.child_frame_id = self.child_frame
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = pos
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = quat
        cov = [0.0] * 36
        for i in range(3):
            cov[i * 7] = self.position_std ** 2
            cov[(i + 3) * 7] = self.orientation_std ** 2
        msg.pose.covariance = cov
        msg.twist.covariance[0] = -1.0  # no twist
        self.pub.publish(msg)

        if self.tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self.child_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pos
            t.transform.rotation = msg.pose.pose.orientation
            self.tf.sendTransform(t)

    def _report(self):
        self.silent = self.silent + 1 if self.frames == 0 else 0
        if self.silent % 12 == 1:  # at once, then every minute (e.g. a robot outdoors, away from Motive)
            self.get_logger().warn(f'no frames from Motive at {self.server_ip} '
                                   f'(streaming on, reachable from {self.local_ip}?)')
        elif self.rb_id is not None and time.monotonic() - self.last_tracked > self.timeout:
            if self.untracked:
                self.get_logger().warn(f"rigid body '{self.rigid_body}' not tracked "
                                       f"({self.untracked} of {self.frames} frames in 5 s)")
            else:
                self.get_logger().warn(f"{self.missing} frames in 5 s without rigid body '{self.rigid_body}' "
                                       f"(Motive's streaming of rigid bodies off, or it was removed?)")
        self.frames = 0
        self.untracked = 0
        self.missing = 0

    def destroy_node(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cmd.close()
        self.data.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NatNetRefPose()
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
