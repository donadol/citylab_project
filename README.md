# Robot Patrol Package

## Overview

The `robot_patrol` package implements autonomous navigation behaviors for a mobile robot. The package provides multiple navigation strategies including reactive obstacle avoidance, service-based direction planning, and goal-oriented action-based navigation.

## Features

- **Autonomous Patrol**: Robot moves forward continuously while avoiding obstacles
- **Service-Based Navigation**: Direction planning through ROS2 services
- **Action-Based Goal Navigation**: Precise pose control using ROS2 actions
- **Obstacle Avoidance**: Uses laser scan data to detect and avoid obstacles
- **Reactive Control**: Continuously adjusts heading toward the safest direction
- **Odometry Integration**: Real-time position tracking and feedback
- **RViz Visualization**: Integrated visualization showing robot model, laser scans, odometry trail, and TF frames

## Navigation Algorithms

### 1. Basic Patrol (patrol.cpp)

Simple reactive obstacle avoidance:

1. **Scan Processing**: Analyzes laser scan data in the front 180° field of view
2. **Safe Direction Detection**:
   - Filters out invalid readings (NaN, infinity, out of range)
   - Only considers directions with at least 0.35m clearance
   - Identifies the direction with maximum distance (most open space)
3. **Movement Control**:
   - Linear velocity: constant 0.1 m/s forward
   - Angular velocity: proportional to the safest direction angle (direction/2)
   - Control loop runs at 10 Hz

### 2. Service-Based Patrol (patrol_with_service.cpp + direction_service.cpp)

Advanced navigation using ROS2 services:

- **Direction Service**: Analyzes laser scans and divides the front 180° into three 60° sectors (left, front, right)
- **Decision Logic**:
  - If front sector has >0.5m clearance: go forward
  - Otherwise: turn toward the sector with the largest total distance
- **Service Integration**: Patrol node calls direction service to determine optimal heading
- **Multi-threaded**: Uses callback groups for non-blocking service calls

### 3. Goal-Based Navigation (go_to_pose_action.cpp)

Precise pose control using ROS2 actions:

- **Two-Phase Control**:
  - Phase 1: Navigate to target position (x, y) while maintaining forward motion
  - Phase 2: Rotate in place to achieve target orientation (theta)
- **PID-like Control**: Proportional angular velocity control with clamping
- **Odometry Feedback**: Continuous position updates from /odom topic
- **Goal Tracking**: Real-time feedback on current position and distance to goal
- **Tolerances**: 0.1m position tolerance, 0.5 rad orientation tolerance

## Package Structure

```text
robot_patrol/
├── CMakeLists.txt                        # Build configuration
├── package.xml                           # Package manifest
├── README.md                             # This file
├── src/
│   ├── patrol.cpp                        # Basic reactive patrol
│   ├── patrol_with_service.cpp           # Service-based patrol client
│   ├── direction_service.cpp             # Direction service server
│   ├── go_to_pose_action.cpp            # Action server for goal navigation
│   └── test_service.cpp                  # Service testing utility
├── launch/
│   ├── start_patrolling.launch.py        # Basic patrol + RViz
│   ├── start_direction_service.launch.py # Service-based patrol system
│   ├── start_gotopose_action.launch.py  # Action server launch
│   └── main.launch.py                    # Complete system launch
├── srv/
│   └── GetDirection.srv                  # Custom service definition
├── action/
│   └── GoToPose.action                   # Custom action definition
└── config/
    └── display.rviz                      # RViz configuration
```

## Dependencies

- **rclcpp**: ROS2 C++ client library
- **rclcpp_action**: ROS2 action server/client library
- **sensor_msgs**: For LaserScan messages
- **geometry_msgs**: For Twist (velocity) and Pose2D messages
- **nav_msgs**: For Odometry messages
- **tf2**: For coordinate transformations and quaternion operations
- **tf2_geometry_msgs**: For TF2 geometry message conversions
- **rosidl_default_generators**: For custom message/service/action generation
- **rviz2**: For visualization

## Building

```bash
# From your ROS2 workspace root
colcon build --packages-select robot_patrol

# Source the workspace
source install/setup.bash
```

## Usage

### 1. Running Basic Patrol

Launch the basic reactive patrol with RViz visualization:

```bash
ros2 launch robot_patrol start_patrolling.launch.py
```

This will start:

- The `patrol_node` which publishes velocity commands using reactive obstacle avoidance
- RViz2 with the pre-configured display settings

### 2. Running Service-Based Patrol

Launch the service-based navigation system:

```bash
ros2 launch robot_patrol start_direction_service.launch.py
```

This will start:

- The `direction_service` server for laser scan analysis
- The `patrol_with_service` client node that requests directions and controls the robot

### 3. Running Goal-Based Navigation (Action Server)

Launch the action server for goal-oriented navigation:

```bash
ros2 launch robot_patrol start_gotopose_action.launch.py
```

Then send a goal from another terminal:

```bash
# Send a goal: x=2.0, y=1.0, theta=90 degrees
ros2 action send_goal /go_to_pose robot_patrol/action/GoToPose "{goal_pos: {x: 2.0, y: 1.0, theta: 90.0}}"
```

### 4. Running Complete System

Launch all components together:

```bash
ros2 launch robot_patrol main.launch.py
```

### Running on Real Robot

The `real-robot` branch contains the necessary modifications to run this package on a physical robot. Switch to that branch if you need to deploy on real hardware.

## Topics and Interfaces

### Subscribed Topics

- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))
  - Laser scan data used for obstacle detection
  - Used by: patrol nodes and direction service

- `/odom` ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))
  - Odometry data for position tracking
  - Used by: go_to_pose_action

### Published Topics

- `/cmd_vel` ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))
  - Velocity commands to control the robot
  - Published by: all patrol nodes and action server

### Services

- `/direction_service` (robot_patrol/srv/GetDirection)
  - **Request**: LaserScan data
  - **Response**: Direction string ("forward", "left", or "right")
  - Analyzes laser data and returns optimal movement direction

### Actions

- `/go_to_pose` (robot_patrol/action/GoToPose)
  - **Goal**: Pose2D (x, y, theta) - target pose for the robot
  - **Result**: bool status - whether goal was successfully reached
  - **Feedback**: Pose2D current_pos - current robot position during execution
  - Navigates robot to specified goal pose with feedback

## Parameters

### Basic Patrol Parameters

- **Linear velocity**: 0.1 m/s (constant forward motion)
- **Obstacle threshold**: 0.35 m (minimum safe clearance)
- **Angular velocity gain**: 0.5 (direction angle divided by 2)
- **Control frequency**: 10 Hz
- **Laser field of view**: Front 180° (-π/2 to +π/2 radians)

### Direction Service Parameters

- **Sector division**: Three 60° sectors covering front 180°
  - Right: 270° to 330°
  - Front: 330° to 30° (wraps through 0°)
  - Left: 30° to 90°
- **Front clearance threshold**: 0.5 m (50 cm)
- **Decision priority**: Front preferred if clear, otherwise largest distance sector

### Go-To-Pose Action Parameters

- **Linear velocity**: 0.1 m/s (during navigation phase)
- **Angular gain**: 0.8 (proportional control)
- **Max angular velocity**: 1.5 rad/s
- **Min angular velocity**: 0.4 rad/s (overcomes motor deadband)
- **Position tolerance**: 0.1 m (10 cm)
- **Orientation tolerance**: 0.5 rad (~28.6°)
- **Control frequency**: 10 Hz
- **Theta input**: Degrees (converted to radians internally)

## RViz Configuration

The included RViz configuration displays:

- **Grid**: 50x50 cells, 0.1m cell size
- **Axes**: 2.5m length, 0.01m radius
- **Robot Model**: From robot description topic
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

### patrol_node

- **Type**: Basic reactive navigation
- **Behavior**: Continuous forward motion with reactive obstacle avoidance
- **State**: Stateless, purely reactive

### direction_service_node

- **Type**: ROS2 Service Server
- **Function**: Analyzes laser scans and returns movement direction
- **Algorithm**: Sector-based distance analysis with front-priority logic

### patrol_with_service_node

- **Type**: Service client with multi-threaded executor
- **Behavior**: Requests directions from service and executes movement commands
- **Features**: Non-blocking service calls using callback groups

### go_to_pose_action_node

- **Type**: ROS2 Action Server
- **Behavior**: Goal-oriented navigation with continuous feedback
- **Features**: Cancelable goals, real-time position tracking, two-phase control

## Behavior Characteristics

### Basic Patrol

- **Continuous Motion**: Robot never stops, always moving forward
- **Smooth Turning**: Angular velocity is proportional to deviation from straight ahead
- **Obstacle Avoidance**: Naturally steers away from obstacles toward open space
- **No Path Planning**: Pure reactive behavior based on current sensor readings
- **Simple & Robust**: Minimal state, works reliably in various environments

### Service-Based Patrol

- **Strategic Decision Making**: Analyzes entire sectors rather than individual rays
- **Front Preference**: Prioritizes forward motion when path is clear
- **Adaptive**: Chooses safest available direction when front is blocked
- **Asynchronous**: Non-blocking service calls maintain smooth operation

### Goal-Based Navigation

- **Precise Positioning**: Achieves specific x, y, theta coordinates
- **Two-Phase Approach**: Separates position reaching from orientation adjustment
- **Real-time Feedback**: Continuous position updates during goal execution
- **Cancelable**: Goals can be canceled mid-execution
- **Robust Control**: Minimum angular velocity prevents motor stalling

## Custom Interfaces

### GetDirection.srv

Service definition for direction planning:

```text
# Request
sensor_msgs/LaserScan laser_data
---
# Response
string direction
```

**Usage**: Client sends laser scan data, server returns optimal direction string ("forward", "left", or "right").

### GoToPose.action

Action definition for goal-oriented navigation:

```text
# Goal
geometry_msgs/Pose2D goal_pos
---
# Result
bool status
---
# Feedback
geometry_msgs/Pose2D current_pos
```

**Usage**:

- **Goal**: Target pose with x, y (meters), and theta (degrees)
- **Result**: Returns true if goal reached, false if canceled or failed
- **Feedback**: Provides current robot pose throughout execution

## Testing

### Test Direction Service

A test utility is provided to manually test the direction service:

```bash
ros2 run robot_patrol test_service
```

This node subscribes to laser scans and continuously calls the direction service for testing purposes.

### Monitor Action Feedback

To monitor the action server feedback in real-time:

```bash
ros2 action send_goal /go_to_pose robot_patrol/action/GoToPose "{goal_pos: {x: 2.0, y: 1.0, theta: 90.0}}" --feedback
```

## Development Notes

- **Coordinate System**: Uses ROS REP-103 conventions (x forward, y left, z up)
- **Angle Normalization**: All angles normalized to [-π, π] range internally
- **Thread Safety**: Service-based patrol uses callback groups for thread-safe operation
- **Control Loop**: Separate control timer ensures consistent 10 Hz command rate
- **Real Robot Tuning**: Parameters optimized for real robot hardware with motor deadband compensation
