# Mocap Fake Localizer

A ROS2 package that provides utilities for working with mocap (motion capture) systems by simulating localization and EKF functionality.

## Overview

This package contains two nodes that facilitate integration of motion capture data into ROS2 navigation stacks:

- **mocap_fake_localizer_node**: Establishes a static `map → odom` transform based on the initial mocap odometry data
- **mocap_fake_ekf_node**: Acts as a fake Extended Kalman Filter, republishing mocap odometry as filtered data and broadcasting transforms

## Features

### mocap_fake_localizer_node
- Listens to mocap odometry messages
- Creates a static transform between `map` and `odom` frames based on the first received mocap data
- Useful for initializing the map frame in navigation systems that expect a `map → odom → base_link` transform hierarchy
- Automatically stops listening after the static transform is established

### mocap_fake_ekf_node
- Subscribes to mocap odometry data
- Republishes the odometry as `odom_filtered` topic
- Broadcasts dynamic `odom → base_link` transforms
- Configurable frame names for flexibility with different robot setups

### natnet_ref_pose.py (OptiTrack Motive client)

Runs on the robot and reads Motive's NatNet stream directly, so poses are stamped with the robot's own clock on
arrival (no clock sync with Motive's PC). Publishes the robot's reference pose:

- `ref_pose` (`nav_msgs/Odometry`): `base_link`'s pose in `ref_frame` (Motive's world frame, Z-up), child `base_link_ref`
- TF `ref_frame -> base_link_ref` (remap `/tf` to the robot's `tf`, as the other nodes here do)

The simulator publishes the same topic and TF from Isaac Sim's world frame (`SIM_REF_POSE`), so everything downstream
works the same in simulation.

```bash
ros2 run mocap_fake_localizer natnet_ref_pose.py --ros-args -r __ns:=/j100_0921 -r /tf:=tf -p server_ip:=192.168.50.80
```

Parameters: `server_ip` (192.168.50.80), `command_port` (1510), `data_port` (1511), `multicast_group`
(239.255.42.99; the client also registers for unicast, so either Motive transmission type works, unicast is the
better choice over WiFi; empty = unicast only), `rigid_body` (empty = the node's namespace, so naming the rigid
body after the robot needs no configuration), `ref_frame`, `child_frame`, `publish_tf`, `up_axis` (`z` or `y`,
Motive's setting; the output is always Z-up), `base_link_offset` (`base_link`'s pose in the rigid body's frame,
`[x, y, z, qx, qy, qz, qw]`: Motive puts the pivot at the markers' centroid), `max_rate` (Hz, 0 = every frame),
`position_std`/`orientation_std` (covariance), `timeout` (s without frames before it reconnects).

`base_link_offset` and `rigid_body` can be changed at runtime. Untracked frames are dropped, not published; a frame received twice (on both sockets, or unicast and multicast) is used once. The NatNet protocol is in `scripts/natnet.py` (standard library only, also used by multirobot_sim's web UI). Tested against Motive 1.8 (NatNet 2.8) at 100 Hz; NatNet 3.x/4.x
frame layouts are implemented but untested.

### calibrate_ref_offset.py (natnet_ref_pose's base_link_offset)

Motive puts a rigid body's pivot at its markers' centroid, oriented as it was when created, so `ref_pose` is
`base_link` only with the right `base_link_offset`. This drives the robot (only with `--yes`; it needs ~0.6 m clear
ahead and behind and room to turn) and corrects the offset the running client uses:

1. tilt: standing on a flat floor, `base_link`'s roll and pitch must be zero;
2. heading: forward and back, the direction of travel is `base_link`'s +x;
3. position: one turn in place, the centre of the circle `ref_pose` draws is the rotation centre, taken as
   `base_link` (a differential drive turns about its axle midpoint; a skid-steer robot roughly about its centre).

z is kept unless `--base-height` (`base_link`'s height above Motive's floor) is given: the localizer flattens the
pose, so x, y and yaw are what navigation uses.

```bash
ros2 run mocap_fake_localizer calibrate_ref_offset.py --yes --apply --write <file> --ros-args -r __ns:=/j100_0921
```

`--apply` sets it on the running `natnet_ref_pose` (`base_link_offset` and `rigid_body` can change at runtime),
`--write` stores it in a params file (e.g. `mtu32_bringup/config/ref_localization/<namespace>.yaml`, keeping the
file's other parameters). Run it again to check: a correct offset gives a circle radius under ~1 cm and
corrections near zero. Tested with `test/fake_motive.py` (a fake Motive whose robot follows `cmd_vel`, with the
markers mounted at a known pose): 30 deg yaw, 21 deg pitch and a 20/10/45 cm offset recovered within 0.1 mm /
0.03 deg, the second run 1 mm / 0.02 deg.

### ref_localizer.py (map -> odom from the reference pose or GPS)

The robot's only `map -> odom` publisher; the EKF keeps `odom -> base_link`. Frames:
`ref_frame -> map -> odom -> base_link`, plus `ref_frame -> base_link_ref` from the reference source.

- `anchor` places `map` in `ref_frame`: `fixed` (`map_pose_in_ref`, or `anchor_file` when set), `start` (the
  robot's first reference pose, flattened to the ground), `external` (measured once from another localizer's
  `map -> base_link`, e.g. SLAM, and the reference pose at the same instant).
- `source` drives `map -> odom`: `ref` (`ref_pose`), `gps` (`odometry/global` from the GPS EKF, whose own TF must be
  off), `auto` (ref while it is fresh and `map` is anchored, else GPS), `external` (publish nothing: SLAM / AMCL do).
- When the active source goes stale, the last `map -> odom` is held (the robot dead-reckons on its EKF).
- `source`, `anchor` and `map_pose_in_ref` can be changed at runtime (`ros2 param set`).
- `~/status` (`std_msgs/String`, JSON, 1 Hz), `~/save_anchor` and `~/reset_anchor` (`std_srvs/Trigger`).

SLAM workflow: build the map with `source:=external anchor:=external` (the reference pose is then ground truth in
the SLAM map), call `~/save_anchor`, save the map; later runs on that map use `source:=ref anchor:=fixed` with
`anchor_file` set to the saved file.

`mtu32_bringup`'s `bringup_main.launch.py` starts both nodes with `config/ref_localization.yaml` (+
`config/ref_localization/<namespace>.yaml`); the old `mocap_fake_localizer_node` / `mocap_fake_ekf_node` are kept
for older launch files.

### test/fake_motive.py

A fake Motive (NatNet 2.8 like Motive 1.8, unicast on 127.0.0.1:1510, 100 Hz) carrying one rigid body on a fake
robot that follows `<namespace>/cmd_vel`, plus an untracked one: for testing the client, the calibration and
multirobot_sim's web UI without Motive or a robot. Not installed; see its docstring.

## Dependencies

- `rclcpp` - ROS2 C++ client library
- `nav_msgs` - Navigation message definitions
- `tf2_ros` - Transform library for ROS2
- `geometry_msgs` - Geometry message definitions

## Building

```bash
colcon build --packages-select mocap_fake_localizer
```

## Usage

### mocap_fake_localizer_node

```bash
ros2 run mocap_fake_localizer mocap_fake_localizer_node
```

This node supports four different operational modes selected via the `mode` parameter. Set the mode in a YAML config or on the command line (e.g. `--ros-args -p mode:=2`). The modes determine whether mocap data or the local EKF/odometry estimator are used for odometry and/or localization:

1. **Mode 1 – Local EKF + Localizer**
   - The node uses the robot's own odometry estimator (EKF or other) for the `odom` frame.
   - Mocap data is treated as a separate localizer; the static transform is published from the defined reference frame to `map` using the first mocap message.
   - Useful when mocap is only needed to correct drift via a mapping/localization node.

2. **Mode 2 – Mocap for Odometry + Localizer**
   - Mocap odometry replaces the onboard odometry (`odom` topic) while a separate localizer (e.g. SLAM) still computes the `map` frame.
   - The `map` frame transform is initialized from localizer outputs but `odom` is driven by mocap.

3. **Mode 3 – Local EKF + Mocap for Localization** (default mode in example config)
   - The robot's EKF/odometry estimator publishes `odom`, but the map origin is set directly from mocap data.
   - Static `map -> odom` transform is derived from the first mocap message; onboard odometry continues normally.
   - This is ideal when you want the map frame to follow mocap ground truth but still rely on your own odometry for short-term motion.

4. **Mode 4 – Mocap for Both Odometry and Localization**
   - Mocap data is used for both the `odom` and `map` frames; the node effectively passes through ground-truth pose.
   - Suitable for ground-truth based navigation or evaluation of algorithms without any onboard estimation.

**Parameters:**
- `mocap_odom_topic` (string, default: `ground_truth/odom`) - Input odometry topic from mocap system

**Publishes:**
- Static transform: `map → odom` (via `tf_static`)

**Subscribes to:**
- Mocap odometry topic (configurable via parameter)

### mocap_fake_ekf_node

```bash
ros2 run mocap_fake_localizer mocap_fake_ekf_node
```

**Parameters:**
- `mocap_odom_topic` (string, default: `ground_truth/odom`) - Input odometry topic
- `odom_frame` (string, default: `odom`) - Frame ID for the odometry frame
- `base_link_frame` (string, default: `base_link`) - Frame ID for the robot base link

**Publishes:**
- `odom_filtered` (nav_msgs/Odometry) - Filtered odometry message
- Dynamic transform: `odom → base_link` (via `tf`)

**Subscribes to:**
- Mocap odometry topic (configurable via parameter)

## Example Launch Configuration

```xml
<launch>
  <!-- Start the mocap fake localizer -->
  <node pkg="mocap_fake_localizer" exec="mocap_fake_localizer_node">
    <param name="mocap_odom_topic" value="ground_truth/odom" />
  </node>

  <!-- Start the mocap fake EKF -->
  <node pkg="mocap_fake_localizer" exec="mocap_fake_ekf_node">
    <param name="mocap_odom_topic" value="ground_truth/odom" />
    <param name="odom_frame" value="odom" />
    <param name="base_link_frame" value="base_link" />
  </node>
</launch>
```

## Transform Frame Hierarchy

This package helps establish the standard ROS2 transform hierarchy:

```
map → odom → base_link
  ↑        ↑
  │        └─ mocap_fake_ekf_node (dynamic)
  └───────────── mocap_fake_localizer_node (static)
```

- `mocap_fake_localizer_node` provides the static `map → odom` transform
- `mocap_fake_ekf_node` provides the dynamic `odom → base_link` transform
- Ground truth mocap data becomes integrated into the standard navigation frame hierarchy

## Use Cases

- **Simulation/Testing**: Use mocap data as ground truth in simulation environments
- **Initialization**: Establish map frame origin from mocap initial position
- **Navigation**: Integrate motion capture data with ROS2 navigation stack
- **Multi-robot Systems**: Use mocap for ground truth localization of multiple robots

## License

See package.xml for license information.

## Maintainer

See package.xml for maintainer information.
