# ROS workspace

## Overview

ROS workspace is a directory where you can build and run ROS packages. It is a recommended way to organize your ROS projects.
It's no different for OpenMowerNext. The root of the repository is a ROS workspace.

## Nodes

- [`map_server`](map-server) - a ROS node responsible for managing map and providing map-related services to navigation stack
- [`map_recorder`](map-recorder) - a ROS node responsible for recording map

## External packages

There are few biggest chunks to mention:

- [ros2_control](https://control.ros.org/master/index.html) - differential drive controller, but also a hardware layer provider
- [nav2](https://navigation.ros.org/) - navigation stack
- [FusionCore](https://github.com/manankharwar/fusioncore) - fuses IMU, wheel odometry and GPS into `/fusion/odom`
- [NTRIP client](https://github.com/LORD-MicroStrain/ntrip_client) - a ROS node to connect to NTRIP caster and get RTK corrections
- [Foxglove bridge](https://foxglove.dev/docs/studio/connection/using-foxglove-bridge) - a ROS node exposing websocket connection to Foxglove Studio. See instructions [here](../visualisation). It can be used for custom Web UIs as well.

Hardware specific packages:

- [VESC driver interface](https://github.com/sbgisen/vesc/tree/humble-devel) - a ros2_control hardware layer implementation for VESC motor controllers
- [ublox_f9p](https://github.com/jkaflik/ublox_f9p) - a ROS2 driver for u-blox F9P GNSS receiver with support of UBX protocol
- [micro-ros-agent](https://github.com/micro-ROS/micro-ROS-Agent) - a micro-ROS agent to connect to micro-ROS nodes. It's used to connect to the [micro-ROS node running on the OpenMower mainboard](omros2-firmware).
