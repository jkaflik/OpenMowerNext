# Simulator

## Overview

OpenMowerNext uses [Webots](https://cyberbotics.com/) as its simulation backend. Gazebo is no longer maintained in this repository.

The simulation is built around Webots-native resources:

- `worlds/openmower.wbt` defines the world, clock source, GPS reference, robot and docking station.
- `protos/OpenMower.proto` defines the robot model and stable Webots device names.
- `protos/DockingStation.proto` defines the charging dock model.
- `resource/openmower_webots.urdf` maps Webots devices to ROS 2 topics and `ros2_control`.

## Getting Started

Install ROS dependencies first:

```bash
make deps
```

Run the simulator from the repository root:

```bash
make sim
```

By default this starts Webots with streaming enabled on port `1234`, so another machine on the same LAN can open `http://<host>:1234/index.html`, for example `http://lord.local:1234/index.html`.
The `/` endpoint is not a normal HTTP page and may return an empty response; the actual simulation stream is a WebSocket on the same port.
If the viewer defaults to `ws://localhost:1234`, change it to `ws://<host>:1234`, for example `ws://lord.local:1234`, and click Connect.
Foxglove is intentionally not started by the simulator launch; run it separately with `make foxglove` when needed.

For headless smoke tests on the development host, run:

```bash
WEBOTS_OFFSCREEN=1 ros2 launch open_mower_next sim.launch.py gui:=false mode:=fast
```

If Webots was installed by `webots_ros2_driver`, set `WEBOTS_HOME=~/.ros/webotsR2025a/webots` or use `make sim`, which sets it automatically when that directory exists.
Headless Webots also needs `xvfb` and `libxcb-cursor0` on Ubuntu.

The canonical development flow runs Webots and ROS 2 on the development host. GUI access can be done through Webots streaming, browser/VNC, or SSH forwarding depending on the workstation setup.

## ROS Contract

The simulator publishes or serves the same ROS-facing contract used by the rest of the stack:

- `/clock` from `Ros2Supervisor`
- `/gps/fix` from the Webots GPS device
- `/imu/data_raw` from the Webots IMU plugin
- `/diff_drive_base_controller/odom` from `diff_drive_controller`
- `/power/charger_present`, `/power/charge_voltage`, and `/power` from `sim_node`

Velocity commands still flow through `twist_mux` to `/diff_drive_base_controller/cmd_vel`.

## Current State

- :white_check_mark: Webots world and robot model
- :white_check_mark: Webots ROS 2 driver launch
- :white_check_mark: `webots_ros2_control` integration
- :white_check_mark: GPS device mapping
- :white_check_mark: IMU device mapping
- :white_check_mark: Charging and battery emulation using [sim node](architecture/sim-node.md)

## World Definition

<<< ../worlds/openmower.wbt
