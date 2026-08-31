# WALL-E Robot

ROS 2 autonomous mobile robot for indoor navigation, LiDAR mapping, waypoint missions, and remote interaction.

> Demo video: [Watch on YouTube](https://www.youtube.com/watch?v=8oirRD0c7bY&t=3s)
>
> [![Demo preview](https://img.youtube.com/vi/8oirRD0c7bY/hqdefault.jpg)](https://www.youtube.com/watch?v=8oirRD0c7bY&t=3s)

## Overview

WALL-E Robot is a differential-drive mobile platform designed for real-world indoor automation. It combines a ROS 2 control stack, LiDAR-based environmental perception, Nav2 navigation, and a web/LiveKit interface for remote operations and guided tours.

## Hardware

The robot is built around a compact differential-drive chassis with a LiDAR sensor, wheel encoder feedback, and embedded motor control. The motion system is connected to ROS 2 through `ros2_control`, while the STM32-based hardware layer handles low-level wheel commands and real-time feedback. This combination gives the platform stable odometry, accurate movement, and a clean path to real-world deployment.

The platform is designed for indoor operation and can be used both for autonomous navigation in mapped environments and for guided tours in human-facing spaces. The hardware path is documented in the main package README and the low-level controller package for the drive system.

![Robot hardware](wall-e-robot/img/Screenshot%202026-07-04%20034241.png)

## Key Features

- ROS 2 Humble control stack
- differential-drive motion control with odometry
- LiDAR-based SLAM and map generation
- Nav2 autonomous path planning and goal execution
- multi-waypoint mission routing
- Gazebo simulation support for development and validation
- real robot deployment through `ros2_control` and STM32 hardware integration
- map saving and reuse for known indoor layouts
- web-based waypoint editing and route configuration
- LiveKit integration for remote communication and voice interaction
- support for service robot and tour-guide workflows

## Repository Structure

```text
wall-e-robot/
├── config/
├── description/
├── launch/
├── maps/
├── scripts/
├── web/
├── docs/
├── worlds/
├── diffdrive_arduino/
├── livekit/
├── CMakeLists.txt
├── package.xml
├── README.md
├── LICENSE.md
└── img/
```

## Functional Modules

### Robot platform
- 2-wheel differential drive chassis
- wheel state monitoring and odometry
- hardware abstraction via `ros2_control`

### Mapping and localization
- LiDAR scan processing
- SLAM Toolbox integration
- map generation and map reuse for navigation

### Navigation and planning
- Nav2 integration for path execution
- goal-driven movement
- sequential waypoint missions

### Remote interaction
- web UI for route design and mission setup
- WebSocket backend support
- LiveKit-based audio and remote interaction

### Simulation and testing
- Gazebo-compatible model
- validation of motion and navigation behavior before deployment

## Typical Use Cases

- indoor service robot
- autonomous patrol and waypoint navigation
- tour-guide robot in exhibition or office spaces
- map-based autonomous operation in known environments
- remote monitoring and interaction via web and voice tools

## Web Management Interface

The system includes a browser-based dashboard for tour planning and mission control. From the web interface, users can create waypoint-based tours, configure delay settings, upload route scripts, start or stop autonomous execution, and monitor robot status in real time. The web layer communicates with the robot backend through WebSocket, and the same workflow can be combined with LiveKit voice and remote interaction for guided tours or assistant-style operation.

This interface is useful for operational scenarios such as guided exhibitions, patrol routes, and remote supervision. It provides a simple visual layer above ROS 2 navigation so that non-technical users can manage route execution without directly dealing with command-line tooling.

![Management tour from website](wall-e-robot/img/Screenshot%202026-06-23%20222335.png)

## Notes

- Designed for ROS 2 Humble.
- `source install/setup.bash` should be run after each workspace rebuild.
- For real deployment, ensure LiDAR, DDS, and `ROS_DOMAIN_ID` are configured correctly.

## Related Links

- Demo video: https://www.youtube.com/watch?v=8oirRD0c7bY&t=3s
- Quick Start and package-level setup: [wall-e-robot/README.md](wall-e-robot/README.md)
- Detailed technical guide: [wall-e-robot/docs/WALL_E_ROBOT_GUIDE.md](wall-e-robot/docs/WALL_E_ROBOT_GUIDE.md)
- Hardware support package: [wall-e-robot/diffdrive_arduino/](wall-e-robot/diffdrive_arduino/)