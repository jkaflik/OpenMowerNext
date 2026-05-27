# Visualisation with Foxglove Studio

## Overview

![Foxglove with OpenMowerNext visualisation](./assets/foxglove.png)

[Foxglove Studio](https://foxglove.dev/) is a visualisation tool for robotics developers. It's designed to be easy to use, and to work with any ROS system.
Alternatively RViz2 can be used for visualisation.

## Installation

Download the latest release from the [Foxglove Studio](https://foxglove.dev/download) website.

## Usage

Connect to your ROS system by using Foxglove Websocket connection. This project comes with [Foxglove bridge](https://foxglove.dev/docs/studio/connection/using-foxglove-bridge) installed. Instructions are available [here](https://foxglove.dev/docs/studio/connection/ros2#foxglove-websocket).

Start the bridge from the repository root:

```bash
make foxglove
```

It listens on `0.0.0.0:8765` by default. From another machine on the same LAN, connect Foxglove Studio to `ws://<host>:8765`, for example `ws://lord.local:8765`.

To install it as a user service:

```bash
make foxglove-service-enable
```

Useful service commands are `make foxglove-service-status`, `make foxglove-service-restart`, `make foxglove-service-disable`, and `make foxglove-service-logs`.

## Joystick Control

The Foxglove dashboard uses Josh Newans' `joy-panel.Joystick` panel as the external joystick source. Configure the panel to publish `sensor_msgs/msg/Joy` on `/joy`.

OpenMowerNext runs `teleop_twist_joy` in both the hardware and Webots simulation launches. It subscribes to `/joy` and publishes joystick velocity commands on `/cmd_vel_joy` as `geometry_msgs/msg/TwistStamped` with `base_link` as the frame.

The command flow is:

```text
Foxglove Joystick -> /joy -> teleop_twist_joy -> /cmd_vel_joy
```

The stack intentionally stays on stamped velocity commands: `twist_mux` uses stamped input, `diff_drive_base_controller` accepts stamped velocity, and Nav2 has stamped command velocity enabled.
