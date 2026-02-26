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
