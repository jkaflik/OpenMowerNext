import math
import os
import random
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid, Odometry
from open_mower_next.msg import Map
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix


GOAL_X = -0.2
GOAL_Y = 0.8
GOAL_YAW = 0.0
GOAL_TOLERANCE = 0.3
LOCALIZATION_MIN_X = -2.0
LOCALIZATION_MAX_X = 4.5
LOCALIZATION_MIN_Y = -2.0
LOCALIZATION_MAX_Y = 3.5
NAV2_LIFECYCLE_NODES = (
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    "docking_server",
    "smoother_server",
)

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
    root = repo_root()
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(150 + (os.getpid() % 50))
    env["WEBOTS_PORT"] = str(14000 + (os.getpid() % 1000))
    env["WEBOTS_OFFSCREEN"] = "1"
    env["OM_DATUM_LAT"] = "-22.9"
    env["OM_DATUM_LONG"] = "-43.2"
    env["OM_MAP_PATH"] = str(root / "test" / "integration" / "navigation_docking_map.geojson")

    if not env.get("WEBOTS_HOME"):
        default_webots_home = Path.home() / ".ros" / "webotsR2025a" / "webots"
        if default_webots_home.is_dir():
            env["WEBOTS_HOME"] = str(default_webots_home)

    if not env.get("WEBOTS_HOME") or not Path(env["WEBOTS_HOME"]).is_dir():
        pytest.fail("WEBOTS_HOME must point to a Webots R2025a installation for integration tests")

    return env


def create_randomized_world(tmp_path: Path):
    seed = int(os.environ.get("OPEN_MOWER_NEXT_GPS_ROTATION_TEST_SEED", "7"))
    rng = random.Random(seed)

    start_x = 1.5
    start_y = -0.4
    for _ in range(100):
        candidate_x = rng.uniform(0.9, 2.2)
        candidate_y = rng.uniform(-0.8, 0.2)
        distance = math.hypot(candidate_x - GOAL_X, candidate_y - GOAL_Y)
        if 1.4 <= distance <= 2.8:
            start_x = candidate_x
            start_y = candidate_y
            break
    start_yaw = rng.uniform(-math.pi, math.pi)

    root = repo_root()
    world_path = tmp_path / f"gps_rotation_heading_{seed}.wbt"
    content = (root / "worlds" / "simple_lawn.wbt").read_text(encoding="utf-8")
    content = content.replace(
        'EXTERNPROTO "../protos/OpenMower.proto"',
        f'EXTERNPROTO "{(root / "protos" / "OpenMower.proto").as_posix()}"',
    )
    content = content.replace(
        'EXTERNPROTO "../protos/DockingStation.proto"',
        f'EXTERNPROTO "{(root / "protos" / "DockingStation.proto").as_posix()}"',
    )
    content = content.replace(
        'title "OpenMowerNext simple lawn"',
        f'title "OpenMowerNext GPS rotation heading test seed {seed}"',
        1,
    )
    content = content.replace(
        """OpenMower {
  translation 0 0 0.0925
  rotation 0 0 1 0
  name "openmower"
  controller "<extern>"
  supervisor TRUE
}""",
        f"""OpenMower {{
  translation {start_x:.6f} {start_y:.6f} 0.0925
  rotation 0 0 1 {start_yaw:.9f}
  name "openmower"
  controller "<extern>"
  supervisor TRUE
}}""",
        1,
    )
    world_path.write_text(content, encoding="utf-8")
    return world_path, (start_x, start_y, start_yaw), seed


def start_simulation(log_path: Path, env: dict[str, str], world_path: Path):
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "open_mower_next",
            "sim.launch.py",
            "gui:=false",
            "mode:=realtime",
            "enable_foxglove:=false",
            f"webots_port:={env['WEBOTS_PORT']}",
            f"world:={world_path}",
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
        text=True,
    )
    return process, log_file


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=15)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=10)
        return
    except (ProcessLookupError, subprocess.TimeoutExpired):
        pass
    if process.poll() is None:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=5)


def read_log_tail(log_path: Path, chars: int = 8000) -> str:
    if not log_path.exists():
        return ""
    return log_path.read_text(encoding="utf-8", errors="replace")[-chars:]


def assert_no_launch_failures(log_path: Path) -> None:
    log = log_path.read_text(encoding="utf-8", errors="replace") if log_path.exists() else ""
    shutdown_marker = "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)"
    checked_log = log.split(shutdown_marker, 1)[0]
    failures = [pattern for pattern in LOG_FAILURE_PATTERNS if pattern in checked_log]
    if failures:
        pytest.fail(f"Launch log contains failure patterns: {', '.join(failures)}\n{checked_log[-8000:]}")


def check_process(process: subprocess.Popen, log_path: Path) -> None:
    if process.poll() is not None:
        pytest.fail(f"Simulation exited early with code {process.returncode}\n{read_log_tail(log_path)}")


def yaw_to_quaternion(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def is_finite_odom(msg: Odometry) -> bool:
    pose = msg.pose.pose
    twist = msg.twist.twist
    values = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
        twist.linear.x,
        twist.linear.y,
        twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


class GpsRotationHeadingTestNode(Node):
    def __init__(self):
        super().__init__(
            "gps_rotation_heading_e2e_test",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.clock_received = False
        self.map_grid_received = False
        self.mowing_map_received = False
        self.diff_drive_odom_received = False
        self.gps_fix_received = False
        self.gps_odom_received = False
        self.filtered_odom = None
        self.heading_source = ""
        self.heading_validated = False
        self.last_heading_sigma = math.inf
        self.record_path = False
        self.path_points = []
        self.navigate_feedback = None
        self.spin_feedback = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.spin_client = ActionClient(self, Spin, "/spin")
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_subscription(OccupancyGrid, "/map_grid", self._map_grid_callback, map_qos)
        self.create_subscription(Map, "/mowing_map", self._mowing_map_callback, map_qos)
        self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self._diff_odom_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self._gps_fix_callback, 10)
        self.create_subscription(Odometry, "/gps/odom", self._gps_odom_callback, 10)
        self.create_subscription(Odometry, "/fusion/odom", self._odom_callback, 10)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostics_callback, 10)

    def _clock_callback(self, _msg: Clock) -> None:
        self.clock_received = True

    def _map_grid_callback(self, _msg: OccupancyGrid) -> None:
        self.map_grid_received = True

    def _mowing_map_callback(self, _msg: Map) -> None:
        self.mowing_map_received = True

    def _diff_odom_callback(self, _msg: Odometry) -> None:
        self.diff_drive_odom_received = True

    def _gps_fix_callback(self, _msg: NavSatFix) -> None:
        self.gps_fix_received = True

    def _gps_odom_callback(self, msg: Odometry) -> None:
        if is_finite_odom(msg):
            self.gps_odom_received = True

    def _odom_callback(self, msg: Odometry) -> None:
        self.filtered_odom = msg
        if self.record_path and is_finite_odom(msg):
            point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            if not self.path_points or math.hypot(
                point[0] - self.path_points[-1][0], point[1] - self.path_points[-1][1]
            ) >= 0.02:
                self.path_points.append(point)

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "fusioncore: Filter":
                continue
            values = {item.key: item.value for item in status.values}
            self.heading_source = values.get("heading_source", self.heading_source)
            self.heading_validated = values.get("heading_validated") == "true"
            try:
                self.last_heading_sigma = float(values.get("last_heading_sigma_rad", "inf"))
            except ValueError:
                self.last_heading_sigma = math.inf

    def has_ready_state(self) -> bool:
        if not self.clock_received or not self.map_grid_received or not self.mowing_map_received:
            return False
        if not self.diff_drive_odom_received or not self.gps_fix_received or not self.gps_odom_received:
            return False
        if self.filtered_odom is None or not is_finite_odom(self.filtered_odom):
            return False
        if not self.has_bounded_filtered_pose():
            return False
        return self.has_map_to_base_link_tf()

    def has_map_to_base_link_tf(self) -> bool:
        try:
            return self.tf_buffer.can_transform("map", "base_link", Time(), timeout=Duration(seconds=0.1))
        except Exception:
            return False

    def has_bounded_filtered_pose(self) -> bool:
        if self.filtered_odom is None:
            return False
        position = self.filtered_odom.pose.pose.position
        return LOCALIZATION_MIN_X <= position.x <= LOCALIZATION_MAX_X and LOCALIZATION_MIN_Y <= position.y <= LOCALIZATION_MAX_Y

    def current_pose_xy(self):
        if self.filtered_odom is None:
            return None
        position = self.filtered_odom.pose.pose.position
        return (position.x, position.y)

    def describe_state(self) -> str:
        if self.filtered_odom is None:
            odom_pose = "None"
        else:
            position = self.filtered_odom.pose.pose.position
            yaw = quaternion_to_yaw(self.filtered_odom.pose.pose.orientation)
            odom_pose = f"({position.x:.3f}, {position.y:.3f}, yaw={yaw:.3f})"
        return (
            "state: "
            f"test_time={self.get_clock().now().nanoseconds / 1e9:.3f}, "
            f"clock={self.clock_received}, map_grid={self.map_grid_received}, "
            f"mowing_map={self.mowing_map_received}, diff_drive_odom={self.diff_drive_odom_received}, "
            f"gps_fix={self.gps_fix_received}, gps_odom={self.gps_odom_received}, "
            f"filtered_pose={odom_pose}, map_to_base_link_tf={self.has_map_to_base_link_tf()}, "
            f"heading_source={self.heading_source}, heading_validated={self.heading_validated}, "
            f"last_heading_sigma={self.last_heading_sigma:.3f}, path_points={len(self.path_points)}"
        )


def spin_until(node, predicate, timeout: float, failure_message: str, process, log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail(f"{failure_message}\n{node.describe_state()}\n{read_log_tail(log_path)}")


def spin_for(node, seconds: float, process, log_path: Path) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        check_process(process, log_path)
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_for_future(node, future, timeout: float, label: str, process, log_path: Path):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if future.done():
            return future.result()
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail(f"Timed out waiting for {label}\n{node.describe_state()}\n{read_log_tail(log_path)}")


def wait_for_action_server(node, action_client, name: str, timeout: float, process, log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if action_client.wait_for_server(timeout_sec=1.0):
            return
        rclpy.spin_once(node, timeout_sec=0.1)
    pytest.fail(f"Timed out waiting for action server {name}\n{read_log_tail(log_path)}")


def wait_for_lifecycle_nodes_active(node, names: tuple[str, ...], timeout: float, process, log_path: Path) -> None:
    clients = {name: node.create_client(GetState, f"/{name}/get_state") for name in names}
    states = {name: "unknown" for name in names}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        all_active = True
        for name, client in clients.items():
            if not client.service_is_ready():
                client.wait_for_service(timeout_sec=0.1)
            if not client.service_is_ready():
                states[name] = "service_unavailable"
                all_active = False
                continue
            future = client.call_async(GetState.Request())
            response = wait_for_future(node, future, 2.0, f"{name} lifecycle state", process, log_path)
            states[name] = response.current_state.label or str(response.current_state.id)
            if response.current_state.id != State.PRIMARY_STATE_ACTIVE:
                all_active = False
        if all_active:
            return
        spin_for(node, 0.5, process, log_path)
    pytest.fail(
        f"Timed out waiting for Nav2 lifecycle nodes to become active: {states}\n"
        f"{node.describe_state()}\n{read_log_tail(log_path)}"
    )


def send_spin_goal(node: GpsRotationHeadingTestNode, process, log_path: Path) -> None:
    goal = Spin.Goal()
    goal.target_yaw = math.pi / 2.0
    goal.time_allowance.sec = 30

    goal_future = node.spin_client.send_goal_async(
        goal, feedback_callback=lambda feedback: setattr(node, "spin_feedback", feedback.feedback)
    )
    goal_handle = wait_for_future(node, goal_future, 15.0, "Spin goal response", process, log_path)
    assert goal_handle.accepted, f"Spin goal was rejected\n{node.describe_state()}\n{read_log_tail(log_path)}"

    result_response = wait_for_future(node, goal_handle.get_result_async(), 60.0, "Spin result", process, log_path)
    assert result_response.status == GoalStatus.STATUS_SUCCEEDED, (
        f"Spin failed with status={result_response.status}, "
        f"error_code={result_response.result.error_code}, error_msg={result_response.result.error_msg!r}\n"
        f"{node.describe_state()}\n{read_log_tail(log_path)}"
    )
    assert result_response.result.error_code == Spin.Result.NONE, (
        f"Spin returned error_code={result_response.result.error_code}, "
        f"error_msg={result_response.result.error_msg!r}\n{read_log_tail(log_path)}"
    )


def send_navigation_goal(node: GpsRotationHeadingTestNode, process, log_path: Path):
    start = node.current_pose_xy()
    assert start is not None, f"No filtered odom before navigation\n{node.describe_state()}"
    assert math.hypot(start[0] - GOAL_X, start[1] - GOAL_Y) > GOAL_TOLERANCE, (
        f"Goal starts too close to robot\n{node.describe_state()}"
    )

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = GOAL_X
    goal.pose.pose.position.y = GOAL_Y
    goal.pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(GOAL_YAW)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    node.path_points = [start]
    node.record_path = True
    try:
        goal_future = node.navigate_client.send_goal_async(
            goal, feedback_callback=lambda feedback: setattr(node, "navigate_feedback", feedback.feedback)
        )
        goal_handle = wait_for_future(node, goal_future, 15.0, "NavigateToPose goal response", process, log_path)
        assert goal_handle.accepted, f"NavigateToPose goal was rejected\n{node.describe_state()}\n{read_log_tail(log_path)}"

        result_response = wait_for_future(node, goal_handle.get_result_async(), 120.0, "NavigateToPose result", process, log_path)
    finally:
        node.record_path = False

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED, (
        f"NavigateToPose failed with status={result_response.status}, "
        f"error_code={result_response.result.error_code}, error_msg={result_response.result.error_msg!r}\n"
        f"{node.describe_state()}\n{read_log_tail(log_path)}"
    )
    assert result_response.result.error_code == NavigateToPose.Result.NONE, (
        f"NavigateToPose returned error_code={result_response.result.error_code}, "
        f"error_msg={result_response.result.error_msg!r}\n{read_log_tail(log_path)}"
    )
    spin_for(node, 1.0, process, log_path)
    return start


def assert_path_is_straight(points, start, goal):
    assert len(points) >= 4, f"Too few path points to assess straightness: {len(points)}"
    direct = math.hypot(goal[0] - start[0], goal[1] - start[1])
    assert direct > 0.5, f"Direct path too short: {direct:.3f}"

    path_length = 0.0
    for p0, p1 in zip(points, points[1:]):
        path_length += math.hypot(p1[0] - p0[0], p1[1] - p0[1])

    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    cross_track = [abs(dx * (p[1] - start[1]) - dy * (p[0] - start[0])) / direct for p in points]
    rms_cross_track = math.sqrt(sum(error * error for error in cross_track) / len(cross_track))
    max_cross_track = max(cross_track)
    ratio = path_length / direct

    assert ratio < 1.5, (
        f"Path is too indirect: path_length={path_length:.3f}, direct={direct:.3f}, ratio={ratio:.3f}"
    )
    assert rms_cross_track < 0.35, f"RMS cross-track error too high: {rms_cross_track:.3f}"
    assert max_cross_track < 0.65, f"Max cross-track error too high: {max_cross_track:.3f}"


def test_gps_rotation_heading_spin_then_straight_navigation(tmp_path):
    log_path = tmp_path / "gps_rotation_heading.log"
    env = prepare_env()
    world_path, start_pose, seed = create_randomized_world(tmp_path)
    previous_ros_domain_id = os.environ.get("ROS_DOMAIN_ID")
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    process, log_file = start_simulation(log_path, env, world_path)
    rclpy.init()
    node = GpsRotationHeadingTestNode()

    try:
        spin_until(
            node,
            node.has_ready_state,
            120.0,
            f"Simulation did not reach ready state for seed={seed}, start_pose={start_pose}",
            process,
            log_path,
        )
        wait_for_lifecycle_nodes_active(node, NAV2_LIFECYCLE_NODES, 60.0, process, log_path)
        wait_for_action_server(node, node.spin_client, "/spin", 60.0, process, log_path)
        wait_for_action_server(node, node.navigate_client, "/navigate_to_pose", 60.0, process, log_path)

        send_spin_goal(node, process, log_path)
        spin_until(
            node,
            lambda: node.heading_validated and node.heading_source == "GPS_ROTATION",
            20.0,
            "FusionCore did not validate heading from GPS rotation after Nav2 Spin",
            process,
            log_path,
        )

        navigation_start = send_navigation_goal(node, process, log_path)
        assert_path_is_straight(node.path_points, navigation_start, (GOAL_X, GOAL_Y))
        assert process.poll() is None, f"Simulation exited early with code {process.returncode}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_process(process)
        log_file.close()
        if previous_ros_domain_id is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = previous_ros_domain_id

    assert_no_launch_failures(log_path)
