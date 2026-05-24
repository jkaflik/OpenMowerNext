---
title: Sim Node
---
# {{ $frontmatter.title }}

## Overview

Sim Node is a ROS node that simulates hardware components of the robot in a virtual environment. It provides:

- Docking station detection
- Battery state simulation
- Charging simulation

The node is designed to work with the Webots simulator, using a configured docking contact pose and publishing hardware-like topics that mimic the real robot's behavior.

> [!IMPORTANT]
> The node requires transform information between the robot's `base_link` and `charging_port` to accurately detect docking.
> It also expects a docking station model to be present in the simulation.

## Docking Simulation

The Sim Node detects when the robot's `charging_port` frame is in close proximity to the docking station's charging contacts. When the robot is properly docked:

- The `/power/charger_present` topic will be `true`
- The battery will start charging
- A charging voltage will be published on the `/power/charge_voltage` topic

The docking station contact pose is configured in the simulation launch file to match the Webots docking station model.

## Battery Simulation

The battery simulation provides a simplified model of a real battery system:

- Battery discharges at a constant rate when not charging
- Battery charges at a constant rate when docked
- Battery health status is reported based on voltage levels

## Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/power/charger_present` | `std_msgs/Bool` | Indicates if the robot is properly docked |
| `/power` | `sensor_msgs/BatteryState` | Battery state information including voltage, percentage, and status |
| `/power/charge_voltage` | `std_msgs/Float32` | Voltage provided by the charger when docked |

## Parameters

| Parameter | Description |
|-----------|-------------|
| `docking_station_frame` | Frame where the dock contact pose is defined, usually `map`. |
| `charging_port_frame` | Robot charging port frame, usually `charging_port`. |
| `docking_station_contact_x` | Dock contact X coordinate in `docking_station_frame`. |
| `docking_station_contact_y` | Dock contact Y coordinate in `docking_station_frame`. |
| `docking_station_contact_z` | Dock contact Z coordinate in `docking_station_frame`. |
| `docking_station_contact_yaw` | Dock contact yaw in `docking_station_frame`. |
| `docking_detection_tolerance_x` | Maximum charging port X offset from the dock contact pose. |
| `docking_detection_tolerance_y` | Maximum charging port Y offset from the dock contact pose. |

## Implementation Details

The simulation uses the TF2 library to get the current position of the robot's charging port relative to the configured docking station frame. It then calculates the relative position to the configured docking contact pose to determine if the robot is correctly docked.

Docking is considered successful when the charging port is within 5cm of the docking station's charging contacts. When docked, the battery charging simulation is activated.

The battery simulation logic is simplified:
- If charger is present: Voltage increases at a constant rate until maximum
- If charger is not present: Voltage decreases at a constant rate
- Battery percentage is calculated based on min/max voltage range
- Battery health status (GOOD, DEAD, OVERVOLTAGE) is determined by voltage levels
