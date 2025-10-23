# Robot Patrol Package

## Overview

The `robot_patrol` package implements an autonomous patrol behavior for a mobile robot. The robot continuously moves through an environment while avoiding obstacles using laser scan data.

## Features

- **Autonomous Navigation**: Robot moves forward continuously at a constant velocity
- **Obstacle Avoidance**: Uses laser scan data to detect and avoid obstacles
- **Reactive Control**: Continuously adjusts heading toward the safest (most open) direction
- **RViz Visualization**: Integrated visualization showing robot model, laser scans, odometry trail, and TF frames

## Algorithm

The patrol behavior implements a simple reactive obstacle avoidance algorithm:

1. **Scan Processing**: Analyzes laser scan data in the front 180° field of view
2. **Safe Direction Detection**:
   - Filters out invalid readings (NaN, infinity, out of range)
   - Only considers directions with at least 0.35m clearance
   - Identifies the direction with maximum distance (most open space)
3. **Movement Control**:
   - Linear velocity: constant 0.1 m/s forward
   - Angular velocity: proportional to the safest direction angle (direction/2)
   - Control loop runs at 10 Hz

This reactive approach allows the robot to smoothly navigate around obstacles while maintaining forward motion.

## Package Structure

```
robot_patrol/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── README.md              # This file
├── src/
│   └── patrol.cpp         # Main patrol node implementation
├── launch/
│   └── start_patrolling.launch.py  # Launch file for patrol + RViz
└── config/
    └── display.rviz       # RViz configuration
```

## Dependencies

- **rclcpp**: ROS2 C++ client library
- **sensor_msgs**: For LaserScan messages
- **geometry_msgs**: For Twist (velocity) messages
- **rviz2**: For visualization

## Building

```bash
# From your ROS2 workspace root
colcon build --packages-select robot_patrol

# Source the workspace
source install/setup.bash
```

## Usage

### Running the Patrol Node

Launch the patrol behavior with RViz visualization:

```bash
ros2 launch robot_patrol start_patrolling.launch.py
```

This will start:
- The `patrol_node` which publishes velocity commands
- RViz2 with the pre-configured display settings

## Topics

### Subscribed Topics

- `/fastbot_1/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
  - Laser scan data used for obstacle detection

### Published Topics

- `/fastbot_1/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
  - Velocity commands to control the robot

## Parameters

The patrol behavior uses hardcoded parameters:

- **Linear velocity**: 0.1 m/s (constant forward motion)
- **Obstacle threshold**: 0.35 m (minimum safe clearance)
- **Angular velocity gain**: 0.5 (direction angle divided by 2)
- **Control frequency**: 10 Hz
- **Laser field of view**: Front 180° (-π/2 to +π/2 radians)

## RViz Configuration

The included RViz configuration displays:

- **Grid**: 50x50 cells, 0.1m cell size
- **Axes**: 2.5m length, 0.01m radius
- **Robot Model**: From `/fastbot_1_robot_description` topic
- **TF Frames**: All transforms with names shown, 0.5m marker scale
- **Laser Scan**: Spheres style, 0.025m size
- **Odometry Trail**: Yellow arrows, 250 poses kept, 0.1m position tolerance

**View Settings:**
- Type: Orbit
- Distance: 3.5m
- Yaw: 3.14159 rad (π)
- Pitch: 1.5698 rad (π/2)
- Focal Point: (0, 0, 0)

## Node Details

## Behavior Characteristics

- **Continuous Motion**: Robot never stops, always moving forward
- **Smooth Turning**: Angular velocity is proportional to deviation from straight ahead
- **Obstacle Avoidance**: Naturally steers away from obstacles toward open space
- **No Path Planning**: Pure reactive behavior based on current sensor readings
- **Simple & Robust**: Minimal state, works reliably in various environments
