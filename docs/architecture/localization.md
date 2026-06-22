---
title: FusionCore localization
---
# {{ $frontmatter.title }}

## Overview

Robot pose is based on an absolute position from GPS and relative readings from wheel odometry and IMU.
Orientation is not known on startup and defaults to 0.

FusionCore validates heading from GNSS track motion and, during startup, can also bootstrap heading from a controlled rotation around the GNSS lever arm. Until heading is validated, the GNSS lever-arm correction remains inactive.

If the robot is moved manually while powered, localization should be reset or restarted before autonomous motion. It will need the same heading validation procedure as on startup.

Later on, when undocking behavior is implemented, it will be possible to recover orientation knowing base station position.

## Sensors

OpenMowerNext uses [FusionCore](https://github.com/manankharwar/fusioncore) for wheel odometry,
GPS and IMU sensor fusion. FusionCore publishes `/fusion/odom` and the `odom -> base_link` TF.
The map datum from `OM_DATUM_LAT` and `OM_DATUM_LONG` is used as the fixed FusionCore reference
origin so the fused odometry aligns with GeoJSON maps.

On hardware, the u-blox F9P driver publishes `gps_msgs/GPSFix` on `/gps/fix_extended` and the hardware launch remaps that into FusionCore. This preserves RTK_FLOAT versus RTK_FIX status, satellite count, and receiver accuracy fields that are not available in `sensor_msgs/NavSatFix`. Simulation still uses `/gps/fix` as `NavSatFix`.

### Wheel odometry

Default motor controller VESC reports wheel odometry.

### IMU

Accelerometer and gyroscope is required. Magnetometer is not fused.

### GPS

It's expected GPS is RTK capable. Otherwise, localization will be inaccurate.
More on this in [GPS](../gps.md) section.

## Configuration

<<< ../../config/fusioncore.yaml{yaml}
