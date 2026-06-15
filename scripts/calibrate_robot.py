#!/usr/bin/env python3
"""Interactive hardware calibration and speed characterization flow."""

import argparse
import json
import math
import os
import shutil
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple, Union

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix

try:
    import yaml
except ImportError:  # pragma: no cover - package.xml declares python3-yaml
    yaml = None

try:
    from gps_msgs.msg import GPSFix
except ImportError:  # pragma: no cover - NavSatFix mode still works
    GPSFix = None


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass
class OdomSample:
    t: float
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass
class ImuSample:
    t: float
    wz: float


@dataclass
class GpsFixSample:
    t: float
    lat: float
    lon: float
    alt: float
    hacc: Optional[float]
    status: Optional[int]
    satellites: Optional[int]
    source: str


@dataclass
class StageWindow:
    name: str
    command_linear: float
    command_angular: float
    start: float
    end: float


@dataclass
class ConfigSnapshot:
    root: Path
    controllers: Dict[str, Any]
    hardware: Dict[str, Any]
    nav2: Dict[str, Any]
    fusioncore: Dict[str, Any]


def stamp_to_sec(stamp: Any, fallback: Callable[[], float]) -> float:
    if getattr(stamp, "sec", 0) == 0 and getattr(stamp, "nanosec", 0) == 0:
        return fallback()
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(current: float, previous: float) -> float:
    diff = current - previous
    while diff > math.pi:
        diff -= 2.0 * math.pi
    while diff < -math.pi:
        diff += 2.0 * math.pi
    return diff


def odom_yaw_delta(samples: Sequence[OdomSample]) -> Optional[float]:
    if len(samples) < 2:
        return None
    total = 0.0
    previous = samples[0].yaw
    for sample in samples[1:]:
        total += angle_diff(sample.yaw, previous)
        previous = sample.yaw
    return total


def integrate_imu_yaw(samples: Sequence[ImuSample]) -> Optional[float]:
    if len(samples) < 2:
        return None
    total = 0.0
    previous = samples[0]
    for sample in samples[1:]:
        dt = sample.t - previous.t
        if 0.0 < dt <= 1.0:
            total += 0.5 * (previous.wz + sample.wz) * dt
        previous = sample
    return total


def wgs84_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + alt_m) * cos_lat * math.cos(lon)
    y = (radius + alt_m) * cos_lat * math.sin(lon)
    z = (radius * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float,
    y: float,
    z: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_ecef: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    dx = x - ref_ecef[0]
    dy = y - ref_ecef[1]
    dz = z - ref_ecef[2]

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def gps_samples_to_enu(samples: Sequence[GpsFixSample]) -> List[OdomSample]:
    if not samples:
        return []
    ref = samples[0]
    ref_ecef = wgs84_to_ecef(ref.lat, ref.lon, ref.alt)
    points: List[OdomSample] = []
    for sample in samples:
        ecef = wgs84_to_ecef(sample.lat, sample.lon, sample.alt)
        east, north, _ = ecef_to_enu(ecef[0], ecef[1], ecef[2], ref.lat, ref.lon, ref_ecef)
        points.append(OdomSample(sample.t, east, north, 0.0, 0.0, 0.0, 0.0))
    return points


def path_length(samples: Sequence[OdomSample]) -> Optional[float]:
    if len(samples) < 2:
        return None
    total = 0.0
    previous = samples[0]
    for sample in samples[1:]:
        total += math.hypot(sample.x - previous.x, sample.y - previous.y)
        previous = sample
    return total


def median(values: Iterable[float]) -> Optional[float]:
    filtered = [float(v) for v in values if math.isfinite(float(v))]
    if not filtered:
        return None
    return float(statistics.median(filtered))


def population_stddev(values: Iterable[float]) -> Optional[float]:
    filtered = [float(v) for v in values if math.isfinite(float(v))]
    if not filtered:
        return None
    if len(filtered) == 1:
        return 0.0
    return float(statistics.pstdev(filtered))


def percentile(values: Iterable[float], pct: float) -> Optional[float]:
    filtered = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not filtered:
        return None
    if len(filtered) == 1:
        return filtered[0]
    pos = (len(filtered) - 1) * pct
    lower = int(math.floor(pos))
    upper = int(math.ceil(pos))
    if lower == upper:
        return filtered[lower]
    weight = pos - lower
    return filtered[lower] * (1.0 - weight) + filtered[upper] * weight


def rounded(value: Optional[float], digits: int = 4) -> Optional[float]:
    if value is None or not math.isfinite(value):
        return None
    return round(float(value), digits)


def round_down(value: float, step: float) -> float:
    if step <= 0.0:
        return value
    return math.floor(value / step) * step


def clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def parse_sweep(value: str) -> List[float]:
    result: List[float] = []
    for item in value.split(","):
        item = item.strip()
        if item:
            result.append(float(item))
    return result


Key = Union[int, str]


def get_nested(data: Dict[str, Any], keys: Sequence[Key], default: Any = None) -> Any:
    current: Any = data
    for key in keys:
        if isinstance(current, dict) and key in current:
            current = current[key]
        elif isinstance(current, list) and isinstance(key, int) and 0 <= key < len(current):
            current = current[key]
        else:
            return default
    return current


def set_nested(data: Dict[str, Any], keys: Sequence[Key], value: Any) -> None:
    if not keys:
        raise ValueError("key path must not be empty")

    current: Any = data
    for index, key in enumerate(keys[:-1]):
        next_key = keys[index + 1]
        if isinstance(current, dict):
            if key not in current or not isinstance(current[key], (dict, list)):
                current[key] = [] if isinstance(next_key, int) else {}
            current = current[key]
        elif isinstance(current, list) and isinstance(key, int):
            while len(current) <= key:
                current.append({})
            if not isinstance(current[key], (dict, list)):
                current[key] = [] if isinstance(next_key, int) else {}
            current = current[key]
        else:
            raise TypeError(f"cannot descend into {type(current).__name__} with key {key!r}")

    last_key = keys[-1]
    if isinstance(current, dict):
        current[last_key] = value
    elif isinstance(current, list) and isinstance(last_key, int):
        while len(current) <= last_key:
            current.append(None)
        current[last_key] = value
    else:
        raise TypeError(f"cannot set key {last_key!r} on {type(current).__name__}")


def load_yaml_file(path: Path) -> Dict[str, Any]:
    if yaml is None or not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    return loaded if isinstance(loaded, dict) else {}


def load_config_snapshot(root: Path) -> ConfigSnapshot:
    return ConfigSnapshot(
        root=root,
        controllers=load_yaml_file(root / "controllers.yaml"),
        hardware=load_yaml_file(root / "hardware" / "yardforce500.yaml"),
        nav2=load_yaml_file(root / "nav2_params.yaml"),
        fusioncore=load_yaml_file(root / "fusioncore.yaml"),
    )


def config_value_summary(config: ConfigSnapshot) -> Dict[str, Any]:
    wheel_offset = get_nested(config.hardware, ["wheel", "offset"])
    return {
        "controllers.wheel_radius": get_nested(
            config.controllers,
            ["diff_drive_base_controller", "ros__parameters", "wheel_radius"],
        ),
        "controllers.wheel_separation": get_nested(
            config.controllers,
            ["diff_drive_base_controller", "ros__parameters", "wheel_separation"],
        ),
        "hardware.wheel.radius": get_nested(config.hardware, ["wheel", "radius"]),
        "hardware.wheel.gear_ratio": get_nested(config.hardware, ["wheel", "gear_ratio"]),
        "hardware.wheel.offset_y": wheel_offset[1]
        if isinstance(wheel_offset, list) and len(wheel_offset) > 1
        else None,
        "hardware.wheel.velocity.max": get_nested(config.hardware, ["wheel", "velocity", "max"]),
        "nav2.velocity_smoother.max_velocity": get_nested(
            config.nav2, ["velocity_smoother", "ros__parameters", "max_velocity"]
        ),
        "nav2.FollowPath.desired_linear_vel": get_nested(
            config.nav2,
            ["controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"],
        ),
    }


def latest(items: Sequence[Any]) -> Optional[Any]:
    return items[-1] if items else None


def filter_window(samples: Sequence[Any], window: StageWindow, settle: float = 0.0) -> List[Any]:
    start = window.start + settle
    return [sample for sample in samples if start <= sample.t <= window.end]


class CalibrationNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("open_mower_next_calibration")
        self.args = args
        self.wheel_odom: List[OdomSample] = []
        self.fusion_odom: List[OdomSample] = []
        self.gps_odom: List[OdomSample] = []
        self.imu: List[ImuSample] = []
        self.gps_fixes: List[GpsFixSample] = []

        self.command_pub = self.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.create_subscription(Odometry, args.wheel_odom_topic, self._wheel_odom_callback, 50)
        self.create_subscription(Odometry, args.fusion_odom_topic, self._fusion_odom_callback, 50)
        self.create_subscription(Odometry, args.gps_odom_topic, self._gps_odom_callback, 20)
        self.create_subscription(Imu, args.imu_topic, self._imu_callback, 100)
        self.create_subscription(NavSatFix, args.gps_fix_topic, self._navsat_fix_callback, 20)
        if GPSFix is not None:
            self.create_subscription(GPSFix, args.gps_fix_extended_topic, self._gps_fix_callback, 20)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _message_time(self, msg: Any) -> float:
        return stamp_to_sec(msg.header.stamp, self.now_sec)

    def _odom_sample(self, msg: Odometry) -> OdomSample:
        return OdomSample(
            t=self._message_time(msg),
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=yaw_from_quaternion(msg.pose.pose.orientation),
            vx=msg.twist.twist.linear.x,
            vy=msg.twist.twist.linear.y,
            wz=msg.twist.twist.angular.z,
        )

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        self.wheel_odom.append(self._odom_sample(msg))

    def _fusion_odom_callback(self, msg: Odometry) -> None:
        self.fusion_odom.append(self._odom_sample(msg))

    def _gps_odom_callback(self, msg: Odometry) -> None:
        self.gps_odom.append(self._odom_sample(msg))

    def _imu_callback(self, msg: Imu) -> None:
        self.imu.append(ImuSample(t=self._message_time(msg), wz=msg.angular_velocity.z))

    def _navsat_fix_callback(self, msg: NavSatFix) -> None:
        hacc = None
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            cov = msg.position_covariance
            hacc = math.sqrt(max(cov[0], cov[4], 0.0))
        self.gps_fixes.append(
            GpsFixSample(
                t=self._message_time(msg),
                lat=msg.latitude,
                lon=msg.longitude,
                alt=msg.altitude,
                hacc=hacc,
                status=msg.status.status,
                satellites=None,
                source="gps/fix",
            )
        )

    def _gps_fix_callback(self, msg: Any) -> None:
        hacc = None
        if getattr(msg, "err_horz", 0.0) > 0.0:
            hacc = msg.err_horz / 1.96
        elif getattr(msg, "position_covariance_type", 0) != 0:
            cov = msg.position_covariance
            hacc = math.sqrt(max(cov[0], cov[4], 0.0))
        self.gps_fixes.append(
            GpsFixSample(
                t=self._message_time(msg),
                lat=msg.latitude,
                lon=msg.longitude,
                alt=msg.altitude,
                hacc=hacc,
                status=msg.status.status,
                satellites=getattr(msg.status, "satellites_used", None),
                source="gps/fix_extended",
            )
        )

    def spin_for(self, duration: float) -> None:
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def publish_command(self, linear: float, angular: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.command_pub.publish(msg)

    def stop_robot(self, duration: float = 0.7) -> None:
        period = 1.0 / max(self.args.cmd_rate, 1.0)
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=min(period, 0.05))


def confirm(args: argparse.Namespace, prompt: str) -> bool:
    if args.yes:
        return True
    if not sys.stdin.isatty():
        print("Refusing to continue without --yes because stdin is not interactive.")
        return False
    answer = input(f"{prompt} [y/N] ").strip().lower()
    return answer in {"y", "yes"}


def run_stage(
    node: CalibrationNode,
    args: argparse.Namespace,
    name: str,
    linear: float,
    angular: float,
    duration: float,
) -> Optional[StageWindow]:
    print(f"\nStage: {name}")
    print(f"Command: linear={linear:.3f} m/s angular={angular:.3f} rad/s duration={duration:.1f}s")
    if args.drive_mode == "auto":
        if not confirm(args, "Ensure the robot path is clear and start this motion?"):
            print("Stage skipped.")
            return None
    else:
        if not confirm(args, "Start recording this manually driven stage?"):
            print("Stage skipped.")
            return None
        print("Drive the robot manually now; the script will only record sensor data.")

    node.stop_robot(0.4)
    node.spin_for(0.1)
    start = node.now_sec()
    end_monotonic = time.monotonic() + duration
    period = 1.0 / max(args.cmd_rate, 1.0)
    next_publish = time.monotonic()

    try:
        while rclpy.ok() and time.monotonic() < end_monotonic:
            now = time.monotonic()
            if args.drive_mode == "auto" and now >= next_publish:
                node.publish_command(linear, angular)
                next_publish = now + period
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        if args.drive_mode == "auto":
            node.stop_robot()
        node.spin_for(0.2)

    end = node.now_sec()
    return StageWindow(name=name, command_linear=linear, command_angular=angular, start=start, end=end)


def choose_gps_fixes(samples: Sequence[GpsFixSample]) -> List[GpsFixSample]:
    extended = [sample for sample in samples if sample.source == "gps/fix_extended"]
    if len(extended) >= 2:
        return extended
    return list(samples)


def analyze_straight_stage(
    node: CalibrationNode,
    args: argparse.Namespace,
    window: StageWindow,
) -> Dict[str, Any]:
    wheel = filter_window(node.wheel_odom, window)
    fusion = filter_window(node.fusion_odom, window)
    gps_fixes = choose_gps_fixes(filter_window(node.gps_fixes, window))
    gps_points = gps_samples_to_enu(gps_fixes)

    wheel_distance = path_length(wheel)
    gps_distance = path_length(gps_points)
    fusion_distance = path_length(fusion)
    scale = None
    if (
        wheel_distance is not None
        and gps_distance is not None
        and wheel_distance >= args.min_calibration_distance
        and gps_distance >= args.min_calibration_distance
    ):
        scale = gps_distance / wheel_distance

    yaw_delta = odom_yaw_delta(wheel)
    gps_hacc = median(sample.hacc for sample in gps_fixes if sample.hacc is not None)
    return {
        "name": window.name,
        "command_linear": window.command_linear,
        "duration": rounded(window.end - window.start, 2),
        "wheel_distance_m": rounded(wheel_distance),
        "gps_distance_m": rounded(gps_distance),
        "fusion_distance_m": rounded(fusion_distance),
        "effective_wheel_scale": rounded(scale, 6),
        "wheel_yaw_delta_rad": rounded(yaw_delta),
        "gps_samples": len(gps_fixes),
        "wheel_odom_samples": len(wheel),
        "gps_hacc_m_median": rounded(gps_hacc),
        "usable": scale is not None,
    }


def analyze_rotation_stage(node: CalibrationNode, args: argparse.Namespace, window: StageWindow) -> Dict[str, Any]:
    wheel = filter_window(node.wheel_odom, window)
    fusion = filter_window(node.fusion_odom, window)
    imu = filter_window(node.imu, window)

    wheel_yaw = odom_yaw_delta(wheel)
    fusion_yaw = odom_yaw_delta(fusion)
    imu_yaw = integrate_imu_yaw(imu)

    reference_source = None
    reference_yaw = None
    if imu_yaw is not None and abs(imu_yaw) >= args.min_rotation_yaw:
        reference_source = "imu/angular_velocity.z"
        reference_yaw = imu_yaw
    elif fusion_yaw is not None and abs(fusion_yaw) >= args.min_rotation_yaw:
        reference_source = "fusion/odom"
        reference_yaw = fusion_yaw

    separation_scale = None
    if wheel_yaw is not None and reference_yaw is not None and abs(reference_yaw) >= args.min_rotation_yaw:
        separation_scale = abs(wheel_yaw) / abs(reference_yaw)

    return {
        "name": window.name,
        "command_angular": window.command_angular,
        "duration": rounded(window.end - window.start, 2),
        "wheel_yaw_delta_rad": rounded(wheel_yaw),
        "imu_yaw_delta_rad": rounded(imu_yaw),
        "fusion_yaw_delta_rad": rounded(fusion_yaw),
        "reference_source": reference_source,
        "wheel_separation_scale": rounded(separation_scale, 6),
        "wheel_odom_samples": len(wheel),
        "imu_samples": len(imu),
        "usable": separation_scale is not None,
    }


def speed_values_from_odom(samples: Sequence[OdomSample], angular: bool) -> List[float]:
    if angular:
        return [abs(sample.wz) for sample in samples]
    return [abs(sample.vx) for sample in samples]


def speed_values_from_gps_odom(samples: Sequence[OdomSample]) -> List[float]:
    return [math.hypot(sample.vx, sample.vy) for sample in samples]


def estimate_path_speed(samples: Sequence[OdomSample], window: StageWindow, settle: float) -> Optional[float]:
    distance = path_length(samples)
    duration = max(window.end - window.start - settle, 0.0)
    if distance is None or duration <= 0.0:
        return None
    return distance / duration


def estimate_rise_time(values: Sequence[Tuple[float, float]], start: float, target: float) -> Optional[float]:
    if target is None or target <= 0.0:
        return None
    threshold = target * 0.9
    for t, value in values:
        if t >= start and abs(value) >= threshold:
            return max(t - start, 0.0)
    return None


def select_linear_speed_source(
    node: CalibrationNode,
    window: StageWindow,
    settle: float,
) -> Tuple[str, List[float], Optional[float]]:
    gps_odom = filter_window(node.gps_odom, window, settle)
    if len(gps_odom) >= 3:
        values = speed_values_from_gps_odom(gps_odom)
        rise = estimate_rise_time([(s.t, math.hypot(s.vx, s.vy)) for s in gps_odom], window.start, median(values) or 0.0)
        return "gps/odom", values, rise

    gps_fixes = gps_samples_to_enu(choose_gps_fixes(filter_window(node.gps_fixes, window, settle)))
    gps_path_speed = estimate_path_speed(gps_fixes, window, settle)
    if gps_path_speed is not None:
        return "gps/fix_path", [gps_path_speed], None

    fusion = filter_window(node.fusion_odom, window, settle)
    if len(fusion) >= 3:
        values = speed_values_from_odom(fusion, angular=False)
        rise = estimate_rise_time([(s.t, s.vx) for s in fusion], window.start, median(values) or 0.0)
        return "fusion/odom", values, rise

    wheel = filter_window(node.wheel_odom, window, settle)
    values = speed_values_from_odom(wheel, angular=False)
    rise = estimate_rise_time([(s.t, s.vx) for s in wheel], window.start, median(values) or 0.0)
    return "diff_drive_base_controller/odom", values, rise


def select_angular_speed_source(
    node: CalibrationNode,
    window: StageWindow,
    settle: float,
) -> Tuple[str, List[float], Optional[float]]:
    imu = filter_window(node.imu, window, settle)
    if len(imu) >= 3:
        values = [abs(sample.wz) for sample in imu]
        rise = estimate_rise_time([(s.t, s.wz) for s in imu], window.start, median(values) or 0.0)
        return "imu/angular_velocity.z", values, rise

    fusion = filter_window(node.fusion_odom, window, settle)
    if len(fusion) >= 3:
        values = speed_values_from_odom(fusion, angular=True)
        rise = estimate_rise_time([(s.t, s.wz) for s in fusion], window.start, median(values) or 0.0)
        return "fusion/odom", values, rise

    wheel = filter_window(node.wheel_odom, window, settle)
    values = speed_values_from_odom(wheel, angular=True)
    rise = estimate_rise_time([(s.t, s.wz) for s in wheel], window.start, median(values) or 0.0)
    return "diff_drive_base_controller/odom", values, rise


def analyze_speed_stage(
    node: CalibrationNode,
    args: argparse.Namespace,
    window: StageWindow,
    angular: bool,
) -> Dict[str, Any]:
    if angular:
        source, values, rise_time = select_angular_speed_source(node, window, args.speed_settle_time)
        command = abs(window.command_angular)
    else:
        source, values, rise_time = select_linear_speed_source(node, window, args.speed_settle_time)
        command = abs(window.command_linear)

    actual = median(values)
    spread = population_stddev(values)
    tracking_ratio = actual / command if actual is not None and command > 0.0 else None
    stable = False
    if actual is not None and spread is not None and tracking_ratio is not None:
        max_spread = max(args.max_speed_stddev, args.max_relative_speed_stddev * actual)
        stable = tracking_ratio >= args.min_tracking_ratio and spread <= max_spread

    accel = actual / rise_time if actual is not None and rise_time not in (None, 0.0) else None
    return {
        "name": window.name,
        "command": rounded(command),
        "actual_median": rounded(actual),
        "actual_p10": rounded(percentile(values, 0.10)),
        "actual_p90": rounded(percentile(values, 0.90)),
        "stddev": rounded(spread),
        "tracking_ratio": rounded(tracking_ratio),
        "rise_time_s": rounded(rise_time),
        "observed_accel": rounded(accel),
        "source": source,
        "samples": len(values),
        "stable": stable,
    }


def config_limit_warnings(config: ConfigSnapshot) -> List[str]:
    warnings: List[str] = []
    summary = config_value_summary(config)
    radius = summary["controllers.wheel_radius"] or summary["hardware.wheel.radius"]
    wheel_joint_max = summary["hardware.wheel.velocity.max"]
    nav_max = summary["nav2.velocity_smoother.max_velocity"]
    if isinstance(nav_max, list) and len(nav_max) >= 3 and radius and wheel_joint_max:
        required_wheel_speed = abs(float(nav_max[0])) / float(radius)
        if required_wheel_speed > abs(float(wheel_joint_max)) * 1.05:
            warnings.append(
                "Nav2 max linear speed requires %.2f rad/s wheel speed, but "
                "hardware wheel.velocity.max is %.2f. Verify whether that YAML value is "
                "really rad/s; its comment currently says m/s."
                % (required_wheel_speed, float(wheel_joint_max))
            )
        required_turn_wheel_speed = abs(float(nav_max[2])) * float(summary["controllers.wheel_separation"] or 0.0) / (
            2.0 * float(radius)
        )
        if required_turn_wheel_speed > abs(float(wheel_joint_max)) * 1.05:
            warnings.append(
                "Nav2 max angular speed requires %.2f rad/s wheel speed, but "
                "hardware wheel.velocity.max is %.2f."
                % (required_turn_wheel_speed, float(wheel_joint_max))
            )
    return warnings


def run_preflight(node: CalibrationNode, args: argparse.Namespace, config: ConfigSnapshot) -> Dict[str, Any]:
    print("Collecting preflight samples...")
    node.spin_for(args.preflight_seconds)
    topics = {name: types for name, types in node.get_topic_names_and_types()}
    gps = latest(node.gps_fixes)
    message_counts = {
        "wheel_odom": len(node.wheel_odom),
        "fusion_odom": len(node.fusion_odom),
        "gps_odom": len(node.gps_odom),
        "gps_fix": len(node.gps_fixes),
        "imu": len(node.imu),
    }
    warnings = config_limit_warnings(config)
    if len(node.wheel_odom) == 0:
        warnings.append(f"No wheel odometry received on {args.wheel_odom_topic}.")
    if len(node.imu) == 0:
        warnings.append(f"No IMU received on {args.imu_topic}; rotation calibration will be weaker.")
    if gps is None:
        warnings.append("No GPS fix received; straight distance calibration will not be usable.")
    elif gps.hacc is not None and gps.hacc > args.max_gps_hacc:
        warnings.append(
            "Latest GPS horizontal accuracy %.2f m is above %.2f m; RTK-quality "
            "speed and geometry estimates may be poor." % (gps.hacc, args.max_gps_hacc)
        )
    if args.cmd_topic not in topics:
        warnings.append(
            f"Command topic {args.cmd_topic} is not visible yet. This is normal before "
            "the script publishes, but twist_mux must have a matching input."
        )

    report = {
        "message_counts": message_counts,
        "latest_gps": {
            "source": gps.source if gps else None,
            "status": gps.status if gps else None,
            "satellites": gps.satellites if gps else None,
            "hacc_m": rounded(gps.hacc) if gps else None,
        },
        "config": config_value_summary(config),
        "warnings": warnings,
    }

    print(json.dumps(report, indent=2))
    if warnings and not args.yes:
        if not confirm(args, "Preflight has warnings. Continue anyway?"):
            raise RuntimeError("Calibration aborted by user after preflight warnings.")
    return report


def run_geometry(node: CalibrationNode, args: argparse.Namespace) -> Dict[str, Any]:
    result: Dict[str, Any] = {"straight": [], "rotation": []}
    forward = run_stage(
        node,
        args,
        "straight_forward",
        args.geometry_linear_speed,
        0.0,
        args.geometry_linear_duration,
    )
    if forward is not None:
        result["straight"].append(analyze_straight_stage(node, args, forward))

    if args.include_reverse:
        reverse = run_stage(
            node,
            args,
            "straight_reverse",
            -args.geometry_linear_speed,
            0.0,
            args.geometry_linear_duration,
        )
        if reverse is not None:
            result["straight"].append(analyze_straight_stage(node, args, reverse))

    left = run_stage(
        node,
        args,
        "rotate_ccw",
        0.0,
        args.geometry_angular_speed,
        args.geometry_angular_duration,
    )
    if left is not None:
        result["rotation"].append(analyze_rotation_stage(node, args, left))

    right = run_stage(
        node,
        args,
        "rotate_cw",
        0.0,
        -args.geometry_angular_speed,
        args.geometry_angular_duration,
    )
    if right is not None:
        result["rotation"].append(analyze_rotation_stage(node, args, right))
    return result


def run_speed_sweep(node: CalibrationNode, args: argparse.Namespace) -> Dict[str, Any]:
    result: Dict[str, Any] = {"linear": [], "angular": []}

    seen_stable = False
    unstable_after_stable = 0
    for speed in parse_sweep(args.linear_sweep):
        window = run_stage(node, args, f"linear_speed_{speed:.2f}", speed, 0.0, args.speed_step_duration)
        if window is None:
            continue
        analysis = analyze_speed_stage(node, args, window, angular=False)
        result["linear"].append(analysis)
        print(json.dumps(analysis, indent=2))
        if analysis["stable"]:
            seen_stable = True
            unstable_after_stable = 0
        elif seen_stable:
            unstable_after_stable += 1
        if unstable_after_stable >= args.stop_after_unstable:
            print("Stopping linear sweep after repeated unstable steps.")
            break

    seen_stable = False
    unstable_after_stable = 0
    for speed in parse_sweep(args.angular_sweep):
        window = run_stage(node, args, f"angular_speed_{speed:.2f}", 0.0, speed, args.speed_step_duration)
        if window is None:
            continue
        analysis = analyze_speed_stage(node, args, window, angular=True)
        result["angular"].append(analysis)
        print(json.dumps(analysis, indent=2))
        if analysis["stable"]:
            seen_stable = True
            unstable_after_stable = 0
        elif seen_stable:
            unstable_after_stable += 1
        if unstable_after_stable >= args.stop_after_unstable:
            print("Stopping angular sweep after repeated unstable steps.")
            break

    return result


def stable_speed_summary(entries: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    stable = [entry for entry in entries if entry.get("stable") and entry.get("actual_median")]
    observed = [entry for entry in entries if entry.get("actual_median")]
    return {
        "max_observed": max((entry["actual_median"] for entry in observed), default=None),
        "max_stable": max((entry["actual_median"] for entry in stable), default=None),
        "min_stable": min((entry["actual_median"] for entry in stable), default=None),
        "observed_accel_median": median(
            entry["observed_accel"] for entry in stable if entry.get("observed_accel")
        ),
    }


def make_recommendation(
    file_path: str,
    key_path: Sequence[Key],
    current: Any,
    recommended: Any,
    reason: str,
) -> Dict[str, Any]:
    return {
        "file": file_path,
        "key_path": list(key_path),
        "current": current,
        "recommended": recommended,
        "reason": reason,
    }


def build_recommendations(
    args: argparse.Namespace,
    config: ConfigSnapshot,
    geometry: Optional[Dict[str, Any]],
    speed: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    recommendations: List[Dict[str, Any]] = []
    summary = config_value_summary(config)
    current_radius = summary["controllers.wheel_radius"] or summary["hardware.wheel.radius"]
    current_separation = None
    if summary["hardware.wheel.offset_y"] is not None:
        current_separation = 2.0 * abs(float(summary["hardware.wheel.offset_y"]))
    elif summary["controllers.wheel_separation"] is not None:
        current_separation = summary["controllers.wheel_separation"]
    current_gear_ratio = summary["hardware.wheel.gear_ratio"]

    if geometry:
        scales = [stage["effective_wheel_scale"] for stage in geometry["straight"] if stage.get("usable")]
        scale = median(scales)
        if scale is not None and 0.5 <= scale <= 1.5:
            if current_radius:
                radius = round(float(current_radius) * scale, 5)
                recommendations.append(
                    make_recommendation(
                        "controllers.yaml",
                        ["diff_drive_base_controller", "ros__parameters", "wheel_radius"],
                        current_radius,
                        radius,
                        "GPS distance divided by wheel odometry distance from straight calibration runs.",
                    )
                )
                recommendations.append(
                    make_recommendation(
                        "hardware/yardforce500.yaml",
                        ["wheel", "radius"],
                        summary["hardware.wheel.radius"],
                        radius,
                        "Keep URDF wheel radius aligned with diff_drive_controller.",
                    )
                )
            if current_gear_ratio:
                gear_ratio = round(float(current_gear_ratio) * scale, 9)
                recommendations.append(
                    make_recommendation(
                        "hardware/yardforce500.yaml",
                        ["wheel", "gear_ratio"],
                        current_gear_ratio,
                        gear_ratio,
                        "Alternative to changing wheel radius if physical wheel radius is trusted.",
                    )
                )

        sep_scales = [stage["wheel_separation_scale"] for stage in geometry["rotation"] if stage.get("usable")]
        sep_scale = median(sep_scales)
        if sep_scale is not None and current_separation and 0.5 <= sep_scale <= 1.5:
            separation = round(float(current_separation) * sep_scale, 5)
            offset_y = round(separation / 2.0, 5)
            recommendations.append(
                make_recommendation(
                    "hardware/yardforce500.yaml",
                    ["wheel", "offset", 1],
                    summary["hardware.wheel.offset_y"],
                    offset_y,
                    "Wheel separation is derived as 2 * abs(wheel.offset[1]) in launch files.",
                )
            )

    if speed:
        linear = stable_speed_summary(speed.get("linear", []))
        angular = stable_speed_summary(speed.get("angular", []))
        linear_max = linear.get("max_stable")
        angular_max = angular.get("max_stable")
        if linear_max and angular_max:
            safe_linear = round_down(linear_max * args.nav_max_factor, 0.01)
            safe_angular = round_down(angular_max * args.nav_max_factor, 0.01)
            desired_linear = round_down(linear_max * args.nav_desired_factor, 0.01)
            desired_angular = round_down(angular_max * args.nav_desired_factor, 0.01)
            min_linear = round(max(linear.get("min_stable") or 0.03, 0.03), 2)
            min_angular = round(max(angular.get("min_stable") or 0.10, 0.10), 2)
            accel = linear.get("observed_accel_median")
            angular_accel = angular.get("observed_accel_median")
            recommended_accel = round(clamp((accel or 0.5) * 0.7, 0.1, 2.5), 2)
            recommended_angular_accel = round(clamp((angular_accel or 1.0) * 0.7, 0.2, 3.2), 2)

            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["velocity_smoother", "ros__parameters", "max_velocity"],
                    get_nested(config.nav2, ["velocity_smoother", "ros__parameters", "max_velocity"]),
                    [round(safe_linear, 2), 0.0, round(safe_angular, 2)],
                    "85% of highest stable measured linear and angular speed.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["velocity_smoother", "ros__parameters", "min_velocity"],
                    get_nested(config.nav2, ["velocity_smoother", "ros__parameters", "min_velocity"]),
                    [round(-safe_linear, 2), 0.0, round(-safe_angular, 2)],
                    "Symmetric reverse limits matching calibrated safe max speeds.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["velocity_smoother", "ros__parameters", "max_accel"],
                    get_nested(config.nav2, ["velocity_smoother", "ros__parameters", "max_accel"]),
                    [recommended_accel, 0.0, recommended_angular_accel],
                    "70% of observed step-response acceleration, capped conservatively.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["velocity_smoother", "ros__parameters", "max_decel"],
                    get_nested(config.nav2, ["velocity_smoother", "ros__parameters", "max_decel"]),
                    [-recommended_accel, 0.0, -recommended_angular_accel],
                    "Symmetric deceleration limits from measured acceleration.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"],
                    get_nested(
                        config.nav2,
                        ["controller_server", "ros__parameters", "FollowPath", "desired_linear_vel"],
                    ),
                    round(desired_linear, 2),
                    "70% of highest stable measured linear speed for nominal path following.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["controller_server", "ros__parameters", "FollowPath", "regulated_linear_scaling_min_speed"],
                    get_nested(
                        config.nav2,
                        [
                            "controller_server",
                            "ros__parameters",
                            "FollowPath",
                            "regulated_linear_scaling_min_speed",
                        ],
                    ),
                    round(max(min_linear, min(desired_linear * 0.5, desired_linear)), 2),
                    "Do not let RPP regulate below the measured repeatable low-speed range.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["controller_server", "ros__parameters", "FollowPath", "rotate_to_heading_angular_vel"],
                    get_nested(
                        config.nav2,
                        ["controller_server", "ros__parameters", "FollowPath", "rotate_to_heading_angular_vel"],
                    ),
                    round(desired_angular, 2),
                    "70% of highest stable measured angular speed for heading alignment.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["behavior_server", "ros__parameters", "max_rotational_vel"],
                    get_nested(config.nav2, ["behavior_server", "ros__parameters", "max_rotational_vel"]),
                    round(safe_angular, 2),
                    "Safe angular speed limit for Nav2 behavior plugins.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["behavior_server", "ros__parameters", "min_rotational_vel"],
                    get_nested(config.nav2, ["behavior_server", "ros__parameters", "min_rotational_vel"]),
                    min_angular,
                    "Lowest repeatable measured angular speed.",
                )
            )
            docking_speed = round(clamp(min(desired_linear * 0.5, 0.15), min_linear, safe_linear), 2)
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["docking_server", "ros__parameters", "controller", "v_linear_min"],
                    get_nested(config.nav2, ["docking_server", "ros__parameters", "controller", "v_linear_min"]),
                    docking_speed,
                    "Conservative stable low speed for docking approach.",
                )
            )
            recommendations.append(
                make_recommendation(
                    "nav2_params.yaml",
                    ["docking_server", "ros__parameters", "controller", "v_linear_max"],
                    get_nested(config.nav2, ["docking_server", "ros__parameters", "controller", "v_linear_max"]),
                    docking_speed,
                    "Keep docking speed fixed until docking controller is separately tuned.",
                )
            )

    return {"recommendations": recommendations}


def apply_recommendations(
    config_root: Path,
    recommendations: Sequence[Dict[str, Any]],
    timestamp: str,
) -> List[str]:
    if yaml is None:
        raise RuntimeError("Cannot apply recommendations because python3-yaml is unavailable.")

    changed: Dict[Path, Dict[str, Any]] = {}
    for rec in recommendations:
        path = config_root / rec["file"]
        if path not in changed:
            changed[path] = load_yaml_file(path)
        set_nested(changed[path], rec["key_path"], rec["recommended"])

    written: List[str] = []
    for path, data in changed.items():
        if not path.exists():
            raise RuntimeError(f"Cannot apply recommendation; {path} does not exist.")
        backup = path.with_suffix(path.suffix + f".bak.{timestamp}")
        shutil.copy2(path, backup)
        with path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
        written.append(str(path))
    return written


def save_report(report: Dict[str, Any], args: argparse.Namespace, timestamp: str) -> Tuple[Path, Optional[Path]]:
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f"calibration_{timestamp}.json"
    with json_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)

    yaml_path = None
    if yaml is not None:
        yaml_path = output_dir / f"calibration_{timestamp}.yaml"
        with yaml_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False)
    return json_path, yaml_path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run OpenMowerNext hardware calibration and speed characterization."
    )
    parser.add_argument("--mode", choices=["full", "geometry", "speed", "preflight"], default="full")
    parser.add_argument("--drive-mode", choices=["auto", "manual"], default="auto")
    parser.add_argument("--yes", action="store_true", help="Do not prompt before each motion stage.")
    parser.add_argument("--apply", action="store_true", help="Apply recommended YAML values after confirmation.")
    parser.add_argument("--config-root", default="config", help="Directory containing project config YAML files.")
    parser.add_argument("--output-dir", default="log/calibration")
    parser.add_argument("--cmd-topic", default="/cmd_vel_calibration")
    parser.add_argument("--wheel-odom-topic", default="/diff_drive_base_controller/odom")
    parser.add_argument("--fusion-odom-topic", default="/fusion/odom")
    parser.add_argument("--gps-odom-topic", default="/gps/odom")
    parser.add_argument("--gps-fix-topic", default="/gps/fix")
    parser.add_argument("--gps-fix-extended-topic", default="/gps/fix_extended")
    parser.add_argument("--imu-topic", default="/imu/data_raw")
    parser.add_argument("--cmd-rate", type=float, default=20.0)
    parser.add_argument("--preflight-seconds", type=float, default=3.0)
    parser.add_argument("--max-gps-hacc", type=float, default=0.5)
    parser.add_argument("--geometry-linear-speed", type=float, default=0.15)
    parser.add_argument("--geometry-linear-duration", type=float, default=10.0)
    parser.add_argument("--geometry-angular-speed", type=float, default=0.5)
    parser.add_argument("--geometry-angular-duration", type=float, default=8.0)
    parser.add_argument("--include-reverse", action="store_true")
    parser.add_argument("--min-calibration-distance", type=float, default=1.0)
    parser.add_argument("--min-rotation-yaw", type=float, default=1.0)
    parser.add_argument("--linear-sweep", default="0.05,0.10,0.15,0.20,0.26,0.32,0.40,0.50")
    parser.add_argument("--angular-sweep", default="0.20,0.40,0.60,0.80,1.00,1.20")
    parser.add_argument("--speed-step-duration", type=float, default=4.0)
    parser.add_argument("--speed-settle-time", type=float, default=1.0)
    parser.add_argument("--min-tracking-ratio", type=float, default=0.75)
    parser.add_argument("--max-speed-stddev", type=float, default=0.03)
    parser.add_argument("--max-relative-speed-stddev", type=float, default=0.25)
    parser.add_argument("--stop-after-unstable", type=int, default=2)
    parser.add_argument("--nav-max-factor", type=float, default=0.85)
    parser.add_argument("--nav-desired-factor", type=float, default=0.70)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    config = load_config_snapshot(Path(args.config_root))

    rclpy.init(args=None)
    node = CalibrationNode(args)
    report: Dict[str, Any] = {
        "timestamp_utc": timestamp,
        "mode": args.mode,
        "drive_mode": args.drive_mode,
        "arguments": vars(args),
    }

    try:
        if args.drive_mode == "auto" and args.mode != "preflight":
            print("This flow sends motion commands to real hardware.")
            print("Use joystick/emergency stop as the final safety override.")
            if not confirm(args, "Continue with hardware motion calibration?"):
                return 1

        report["preflight"] = run_preflight(node, args, config)

        geometry = None
        speed = None
        if args.mode in {"full", "geometry"}:
            geometry = run_geometry(node, args)
            report["geometry"] = geometry
        if args.mode in {"full", "speed"}:
            speed = run_speed_sweep(node, args)
            report["speed"] = speed

        report["recommendations"] = build_recommendations(args, config, geometry, speed)
        json_path, yaml_path = save_report(report, args, timestamp)
        print(f"\nCalibration report written to {json_path}")
        if yaml_path is not None:
            print(f"Calibration YAML written to {yaml_path}")
        print(json.dumps(report["recommendations"], indent=2))

        recs = report["recommendations"].get("recommendations", [])
        if args.apply and recs:
            if confirm(args, f"Apply {len(recs)} recommendations to {args.config_root}?"):
                written = apply_recommendations(Path(args.config_root), recs, timestamp)
                print("Updated config files:")
                for path in written:
                    print(f"  {path}")
        return 0
    except KeyboardInterrupt:
        print("\nCalibration interrupted; stopping robot.")
        return 130
    except Exception as exc:
        print(f"Calibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        try:
            node.stop_robot()
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
