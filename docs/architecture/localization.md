---
title: FusionCore localization
---
# {{ $frontmatter.title }}

## Overview

Robot pose is based on an absolute position from GPS and relative readings from wheel odometry and IMU.
Orientation is not known on startup and defaults to 0. 

As soon as robot starts moving, orientation is assumed based on robot motion and sensors reading.
It might take a while to get an accurate orientation. Best to start moving in a straight line and do a few circles.

Currently, there is no fallback scenario if had its position changed externally. For example, if you move the robot manually, it will not be able to recover position itself. It will require same procedure as on startup. 

Later on, when undocking behavior is implemented, it will be possible to recover orientation knowing base station position.

## Sensors

OpenMowerNext uses [FusionCore](https://github.com/manankharwar/fusioncore) for wheel odometry,
GPS and IMU sensor fusion. FusionCore publishes `/fusion/odom` and the `odom -> base_link` TF.
The map datum from `OM_DATUM_LAT` and `OM_DATUM_LONG` is used as the fixed FusionCore reference
origin so the fused odometry aligns with GeoJSON maps.

### Wheel odometry

Default motor controller VESC reports wheel odometry.

### IMU

Accelerometer and gyroscope is required. Magnetometer is not fused.

### GPS

It's expected GPS is RTK capable. Otherwise, localization will be inaccurate.
More on this in [GPS](../gps.md) section.

## Configuration

<<< ../../config/fusioncore.yaml{yaml}
