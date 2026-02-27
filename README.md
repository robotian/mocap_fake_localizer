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
