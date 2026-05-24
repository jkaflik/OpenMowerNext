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
