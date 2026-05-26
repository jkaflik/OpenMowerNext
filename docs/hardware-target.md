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

Do not commit private coordinates, credentials, NTRIP settings, or real maps to the repository.

## Container Runtime

The current hardware-oriented runtime is containerized. A simple foreground run looks like this:

```bash
sudo podman run --rm --name next \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v "$HOME/next:/next" \
  --env-file "$HOME/omnext/openmower.env" \
  ghcr.io/jkaflik/openmowernext:main
```

Use foreground runs while bringing up a robot. They make launch failures, hardware permissions, and ROS node crashes immediately visible.

## Fast Iteration

For rapid development, avoid rebuilding and pushing the full runtime image for every source change. Prefer this loop:

- Edit and validate on the development machine.
- Sync the working tree to the hardware target with `rsync`, excluding `build/`, `install/`, and `log/`.
- Run an incremental `colcon build --symlink-install` on the target or inside a long-lived development container on the target.
- Launch in the foreground and watch logs before enabling any service or autonomous behavior.

Use a tagged image from GHCR for reproducible runtime tests and systemd/Podman services once the hardware flow is stable.

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
