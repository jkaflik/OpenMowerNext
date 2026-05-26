# Hardware Target

`omdev.local` is the conventional example hostname for a physical OpenMowerNext test target on the local network. It is not a required name: replace it with your robot hostname or pass `REMOTE_HOST=...` to the Make targets.

Use this flow when you want to validate OpenMowerNext against real hardware instead of Webots.

## Target Host

A hardware target is usually a small Linux computer mounted on the mower, for example a Raspberry Pi. The host should provide:

- SSH access from the development machine.
- Access to the robot serial devices under `/dev`.
- Podman or Docker if using the container runtime.
- A persistent map directory outside the repository.

The repository defaults to `omdev.local` for remote-device helper commands:

```bash
make remote-devices
```

Override it for another robot:

```bash
make REMOTE_HOST=my-mower.local REMOTE_USER=my-user remote-devices
```

The `omdev-*` targets use `omdev.local` through `REMOTE_HOST` by default and the current local username as `OMDEV_USER`:

```bash
make omdev-status
make omdev-sync
make omdev-build
make omdev-run
make omdev-logs
make omdev-stop
```

Override the target host or user when needed:

```bash
make OMDEV_HOST=my-mower.local OMDEV_USER=pi omdev-status
```

## Configuration Files

Keep site-specific state outside the repository. A typical target layout is:

```text
~/next/map.json
~/omnext/openmower.env
```

The environment file should provide at least:

```bash
OM_DATUM_LAT=<latitude>
OM_DATUM_LONG=<longitude>
OM_MAP_PATH=/next/map.json
```

Use raw `KEY=value` entries in this file. `podman --env-file` passes shell quotes
literally, so `OM_NTRIP_PASSWORD="secret"` includes the quote characters in the
password and can cause NTRIP caster `401 Unauthorized` responses.

Do not commit private coordinates, credentials, NTRIP settings, or real maps to the repository.

## Container Runtime

`omdev.local` is a mutable hardware-development target. Do not run the production `ghcr.io/jkaflik/openmowernext:*` image there during iteration. The default Makefile flow uses a long-lived `docker.io/library/ros:jazzy` container named `next-dev`, mounts the synced workspace, installs dependencies from that workspace, and launches from the local build output.

Sync the workspace and create or start the dev container:

```bash
make omdev-sync
make omdev-dev-start
```

Recreate it after changing the map mount layout, base image, or container options:

```bash
make omdev-dev-recreate
```

`omdev-run-workspace` reloads `~/omnext/openmower.env` for the launch process, so changing datum or NTRIP values does not require recreating the dev container.

Install or refresh dependencies inside the dev container after syncing:

```bash
make omdev-deps
```

`omdev-deps` installs ROS tooling, runs `rosdep update`, and installs dependencies for the root package plus the hardware-relevant vendored packages. It intentionally does not run `make custom-deps` on the target because `vcs import --force` can overwrite the synced `src/lib` checkouts.

If the ROS base image or generated CMake metadata changes, clean target-side build artifacts before rebuilding:

```bash
make omdev-clean
make omdev-build
```

After `make omdev-sync` and `make omdev-build`, run the synced workspace:

```bash
make omdev-run-workspace
make omdev-logs
```

Pass extra launch arguments during bring-up when needed:

```bash
make OMDEV_LAUNCH_ARGS='enable_navigation_readiness:=false' omdev-run-workspace
```

If a non-IMU sensor is intentionally offline during diagnostics, bypass FusionCore's all-sensor startup gate:

```bash
make OMDEV_LAUNCH_ARGS='enable_navigation_readiness:=false init_wait_for_all_sensors:=false init_stationary_window:=0.0' omdev-run-workspace
```

FusionCore still initializes from the first IMU sample, so this does not bypass a missing `/imu/data_raw` publisher.

The image can be changed without editing the Makefile:

```bash
make OMDEV_IMAGE=docker.io/library/ros:jazzy omdev-dev-recreate
```

## Fast Iteration

For rapid development, avoid rebuilding and pushing a runtime image for hardware validation. Prefer this loop:

- Edit and validate on the development machine.
- Sync the working tree to the hardware target with `make omdev-sync`; it excludes `build/`, `install/`, `log/`, docs dependencies, and VCS metadata.
- Run `make omdev-deps` after dependency changes or after recreating the dev container.
- Run `make omdev-clean` before rebuilding if the base image changed or stale generated CMake paths appear.
- Run `make omdev-build` to execute an incremental build of the current branch's hardware-critical packages inside the long-lived dev container.
- Run `make omdev-run-workspace` to launch the synced and built workspace.
- Watch logs before enabling any service or autonomous behavior.

By default `omdev-build` builds `fields2cover`, `vesc_*`, `micro_ros_agent`, `ntrip_client`, `compass_msgs`, `fusioncore_core`, `fusioncore_ros`, and `ublox_f9p` from `src/lib`, then builds `open_mower_next` from the workspace root. Override `OMDEV_FIELDS2COVER_BASE_PATHS`, `OMDEV_LIB_BASE_PATHS`, `OMDEV_LIB_PACKAGES`, or `OMDEV_APP_PACKAGES` if your change needs a wider build.

Use a tagged GHCR image only for explicit release or production-image validation. `omdev.local` bring-up does not require a system service; run the workspace from the ROS Jazzy dev container while iterating.

## Calibration

OpenMowerNext includes an interactive hardware calibration tool that can measure wheel geometry, drive response, maximum observed speed, and stable navigation speed recommendations from real odometry, GPS, and IMU data.

Launch the robot first, then run the calibration tool from the same built workspace or from inside the target container:

```bash
ros2 run open_mower_next calibrate_robot --mode preflight
```

Run the full automatic flow only with the robot in a clear outdoor test area and with an operator ready to stop it:

```bash
ros2 run open_mower_next calibrate_robot --mode full
```

The script publishes stamped commands on `/cmd_vel_calibration`. `twist_mux` gives this input higher priority than Nav2 and lower priority than joystick, so joystick teleop remains the preferred manual override.

If you want to drive manually and only record data, use:

```bash
ros2 run open_mower_next calibrate_robot --mode full --drive-mode manual
```

The generated report is written under `log/calibration/` as JSON and YAML. It includes recommended changes for `config/controllers.yaml`, `config/hardware/yardforce500.yaml`, `config/nav2_params.yaml`, and related limits when enough data was collected.

Apply recommendations only after reviewing the report:

```bash
ros2 run open_mower_next calibrate_robot --mode full --apply
```

The `--apply` mode creates timestamped `.bak.<timestamp>` backups before writing YAML files.

## Safety

Before launching on real hardware:

- Keep the mower blade physically disabled unless the test explicitly requires it.
- Put the robot on stands for wheel and controller tests.
- Start with observation-only launches where possible.
- Confirm GPS, micro-ROS, VESC, TF, and localization topics before sending motion commands.
