# Lab 1 — ROS 2 and Differential Drive Robot Simulation

**Course:** Introduction to Robotics and Intelligent Systems  
**Program:** BSc Software Engineering  
**Student:** Pomi Yilma

## Project Overview

This project extends a differential-drive robot simulation using ROS 2 and Gazebo. It adds a rear-facing depth camera and an IMU, introduces custom ROS 2 interfaces, and implements Python nodes that control the robot through topics, services, and actions.

The final task is to make the robot drive around a square using two action interfaces.

## Packages

The `lab1_exercise` folder contains four ROS 2 packages:

- `diffdrive_bot` — the differential-drive robot model, simulation, sensor configuration, and control nodes. Modified to include a depth camera and IMU.
- `lab_interfaces` — the supplied robot message, service, and action interfaces.
- `my_robot_interfaces` — custom `RobotStatus` message, `SetTurnSpeed` service, and `Rotate` action.
- `my_robot_nodes` — Python nodes for obstacle avoidance, rotation, and square driving, together with the square launch file.

## Requirements

This project was developed and tested in:

- Ubuntu 24.04 under WSL 2
- ROS 2 Jazzy
- Gazebo Sim
- Python 3

## Build Instructions

Open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Running the Project

### 1. View the robot model

```bash
ros2 launch diffdrive_bot display.launch.py
```

### 2. Run the simulation

```bash
ros2 launch diffdrive_bot gazebo.launch.py
```

### 3. Run the robot control system

```bash
ros2 launch diffdrive_bot control.launch.py
```

### 4. Run the obstacle avoidance node

In another sourced terminal:

```bash
ros2 run my_robot_nodes turn_away
```

Enable the node:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
```

Disable it:

```bash
ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}"
```

### 5. Configure the turning speed

```bash
ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"
```

### 6. Run the rotation action server

Start the simulation, then run:

```bash
ros2 run my_robot_nodes rotate_server
```

Send a rotation goal from another terminal:

```bash
ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
```

### 7. Drive the robot around a square

The combined launch file starts the simulation, rotation action server, and square-driving action client:

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```

The square driver sends four sequential distance goals and four quarter-turn goals. The default side length is 0.5 metres.

## Screenshots

The `screenshots` folder contains:

1. `depth_camera_pointcloud.png` — RViz showing the rear depth camera image and point cloud.
2. `square_rqt_graph.png` — ROS node and topic graph while the square-driving launch is running.

## Issues and Observations

The sensor update rates observed during testing were lower than the configured rates. The depth camera and IMU were configured for 10 Hz and 50 Hz respectively, but the measured rates were lower during simulation. The system was running under WSL with Gazebo and RViz, and the high CPU load likely affected simulation performance.

The zero-angle rotation goal was rejected as expected because the rotation action server is designed to reject a zero rotation request.

## Conclusion

This exercise demonstrates robot modelling, sensor integration, ROS 2 communication, custom interfaces, parameter handling, service implementation, action servers, action clients, and launch-file integration in a simulated differential-drive robot.
