---
title: Getting started
---
# {{ $frontmatter.title }}

Let's get started with a quick overview of the project.

::: warning
Project is in early development stage. Actual mower logic is missing and things are likely to change.
The list of identified missing features can be found in GitHub [issues](https://github.com/jkaflik/OpenMowerROS2/issues).
:::

## Overview

The purpose of this project is no different from original [OpenMower project ROS implementation](https://github.com/ClemensElflein/open_mower_ros).
OpenMowerROS2 is build on top of [ROS2](https://index.ros.org/doc/ros2/) following all the best practices and recommendations.

At this stage of the project, only one intention to use it is contributing to the project. If you are interested in the project, please feel free to contribute.
It was started as a learning project for myself, but I hope it will be useful for others as well.

## Abilities

::: info
This section is not complete yet. It will be updated as the project progresses.
:::

- :white_check_mark: Control robot
- :white_check_mark: Localization
  - :white_check_mark: Odometry
  - :white_check_mark: IMU
  - :white_check_mark: GPS
- :construction: Simulation using Gazebo

### Roadmap

What was already said, the project is in early development stage. Please get familiar with [Roadmap](roadmap) to get an idea of what is planned for the future.

## Requirements

Some of the instructions might be specific to my setup, but I will try to make it as generic as possible.
Project is tested on YardForce Classic 500B model. It should work on other models supported by OpenMower as well, but it is not tested.

### Hardware

- [Setup as for OpenMower](https://openmower.de/docs/robot-assembly/prepare-the-parts/)
  - [OpenMower v0.13.x mainboard](https://openmower.de/docs/robot-assembly/prepare-the-parts/prepare-mainboard/) with [omros2-firmware](https://github.com/jkaflik/omros2-firmware) flashed. Learn more about the custom firmware in [omros2-firmware](omros2-firmware).

### Software

- 64bit Linux (tested on Ubuntu 22.04)
- [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)

or 

There is no need to install ROS2 on your host machine. You can use [Docker](https://docs.docker.com/engine/install/) instead.

## Installation

::: info
This section is not complete yet. It will be updated as the project progresses.
If you are interested in the development, go to the [contributing guide](contributing) to run the project.
:::