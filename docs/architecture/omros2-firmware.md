# OpenMowerNext firmware

## Overview

Original OpenMower firmware is communicating with ROS via serial port using simple packet protocol.
With ROS2 we can use more advanced communication protocols, like [DDS](https://en.wikipedia.org/wiki/Data_Distribution_Service), using [micro-ROS](https://micro.ros.org/).
No need to run a dedicated node that translates serial packets to ROS messages.
Micro-ROS takes care of that.

This firmware supports only the recent OpenMower mainboard v0.13.x.

## Features

* :white_check_mark: DDS communication
* :white_check_mark: `sensor_msgs/Imu` message published on `/imu/data_raw` topic
* :white_check_mark: `sensor_msgs/BatteryState` message published on `/power` topic
* :white_check_mark: `omros2_firmware_msgs/PowerStatus` message published on `/power/status` topic
* :white_check_mark: OpenMower charging logic
* :white_check_mark: ping micro-ROS agent, make sure it is alive
* :red_circle: safety features

## Flash

Follow instructions in [omros2-firmware](https://github.com/jkaflik/omros2-firmware#build) repository.
