import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Float32


REQUIRED_TOPICS = {
    "/clock",
    "/gps/fix",
    "/imu/data_raw",
    "/diff_drive_base_controller/odom",
    "/power",
    "/power/charge_voltage",
    "/power/charger_present",
}

LOG_FAILURE_PATTERNS = (
    "[FATAL]",
    "Could not load library",
    "Could not listen to extern controllers",
    "Cannot shutdown a ROS adapter",
    "Webots driver exited unexpectedly",
    "process has died",
    "Traceback (most recent call last)",
)


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def prepare_env() -> dict[str, str]:
    env = os.environ.copy()
    env.setdefault("WEBOTS_OFFSCREEN", "1")

    if not env.get("WEBOTS_HOME"):
        default_webots_home = Path.home() / ".ros" / "webotsR2025a" / "webots"
        if default_webots_home.is_dir():
            env["WEBOTS_HOME"] = str(default_webots_home)

    if not env.get("WEBOTS_HOME") or not Path(env["WEBOTS_HOME"]).is_dir():
        pytest.fail("WEBOTS_HOME must point to a Webots R2025a installation for integration tests")

    root = repo_root()
    env.setdefault("OM_DATUM_LAT", "-22.9")
    env.setdefault("OM_DATUM_LONG", "-43.2")

    map_path = env.get("OM_MAP_PATH")
    if not map_path:
        env["OM_MAP_PATH"] = str(root / ".devcontainer" / "home" / "map.geojson")
    elif not Path(map_path).is_absolute():
        env["OM_MAP_PATH"] = str(root / map_path)

    return env


def start_simulation(log_path: Path, env: dict[str, str]):
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "open_mower_next",
            "sim.launch.py",
            "gui:=false",
            "mode:=fast",
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
        text=True,
    )
    return process, log_file


def stop_simulation(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    if process.poll() is not None:
        return

    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass

    if process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=5)


def assert_no_launch_failures(log_path: Path) -> None:
    log = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    failures = [pattern for pattern in LOG_FAILURE_PATTERNS if pattern in log]
    if failures:
        pytest.fail(f"Launch log contains failure patterns: {', '.join(failures)}\n{log[-6000:]}")


def wait_for_topics_and_motion() -> None:
    rclpy.init()
    node = rclpy.create_node("webots_smoke_test")
    received = set()
    poses = []

    def mark(name):
        return lambda _: received.add(name)

    node.create_subscription(Clock, "/clock", mark("/clock"), 10)
    node.create_subscription(NavSatFix, "/gps/fix", mark("/gps/fix"), 10)
    node.create_subscription(Imu, "/imu/data_raw", mark("/imu/data_raw"), 10)
    node.create_subscription(Bool, "/power/charger_present", mark("/power/charger_present"), 10)
    node.create_subscription(Float32, "/power/charge_voltage", mark("/power/charge_voltage"), 10)
    node.create_subscription(BatteryState, "/power", mark("/power"), 10)
    node.create_subscription(
        Odometry,
        "/diff_drive_base_controller/odom",
        lambda msg: (
            received.add("/diff_drive_base_controller/odom"),
            poses.append((msg.pose.pose.position.x, msg.pose.pose.position.y)),
        ),
        10,
    )
    pub = node.create_publisher(TwistStamped, "/cmd_vel_joy", 10)

    try:
        deadline = time.monotonic() + 90.0
        while not REQUIRED_TOPICS.issubset(received) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        missing = sorted(REQUIRED_TOPICS - received)
        assert not missing, f"Missing expected simulation topics: {missing}"
        assert poses, "No odometry received before motion command"

        start = poses[-1]
        motion_deadline = time.monotonic() + 3.0
        while time.monotonic() < motion_deadline:
            msg = TwistStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_link"
            msg.twist.linear.x = 0.2
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)

        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        pub.publish(stop)

        settle_deadline = time.monotonic() + 2.0
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        finish = poses[-1]
        distance = math.hypot(finish[0] - start[0], finish[1] - start[1])
        assert distance > 0.02, f"Expected odometry to move, got distance={distance:.3f}"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_webots_smoke(tmp_path):
    log_path = tmp_path / "webots_smoke.log"
    process, log_file = start_simulation(log_path, prepare_env())

    try:
        wait_for_topics_and_motion()
        assert process.poll() is None, f"Simulation exited early with code {process.returncode}"
        assert_no_launch_failures(log_path)
    finally:
        stop_simulation(process)
        log_file.close()
