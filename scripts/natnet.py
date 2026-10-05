"""NatNet (OptiTrack Motive) protocol: message ids and parsers for the frame data and model definitions.

Standard library only, shared by natnet_ref_pose.py (the robot-side client) and multirobot_sim's web UI
(tools/sim_ui, which lists Motive's rigid bodies). NatNet 2.x (Motive 1.x; tested against Motive 1.8 /
NatNet 2.8) and 3.x/4.x layouts.
"""
import struct

# NatNet message ids
NAT_CONNECT = 0
NAT_SERVERINFO = 1
NAT_REQUEST_MODELDEF = 4
NAT_MODELDEF = 5
NAT_FRAMEOFDATA = 7
DEFAULT_MULTICAST = '239.255.42.99'
COMMAND_PORT = 1510
DATA_PORT = 1511


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
