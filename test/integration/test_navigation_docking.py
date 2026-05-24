import math
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from open_mower_next.action import DockRobotNearest
from open_mower_next.msg import Map
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float32


NAVIGATION_GOAL_X = -0.2
NAVIGATION_GOAL_Y = 0.8
NAVIGATION_GOAL_YAW = 1.0
NAVIGATION_GOAL_TOLERANCE = 0.3
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
    env["ROS_DOMAIN_ID"] = str(100 + (os.getpid() % 100))
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


def start_simulation(log_path: Path, env: dict[str, str]):
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
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        start_new_session=True,
        text=True,
    )
    return process, log_file


def stop_simulation(process: subprocess.Popen) -> None:
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
        pytest.fail(
            f"Simulation exited early with code {process.returncode}\n{read_log_tail(log_path)}"
        )


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


class NavigationDockingTestNode(Node):
    def __init__(self):
        super().__init__(
            "navigation_docking_e2e_test",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.clock_received = False
        self.map_grid_received = False
        self.mowing_map = None
        self.diff_drive_odom_received = False
        self.gps_odom_received = False
        self.filtered_odom_after_gps = False
        self.filtered_odom = None
        self.closest_navigation_goal_distance = math.inf
        self.bounded_localization_since = None
        self.charger_present = None
        self.charge_voltage = None
        self.navigation_feedback = None
        self.docking_feedback = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.dock_client = ActionClient(self, DockRobotNearest, "/dock_robot_nearest")
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(Clock, "/clock", self._clock_callback, 10)
        self.create_subscription(OccupancyGrid, "/map_grid", self._map_grid_callback, map_qos)
        self.create_subscription(Map, "/mowing_map", self._mowing_map_callback, map_qos)
        self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self._diff_odom_callback, 10)
        self.create_subscription(Odometry, "/odometry/gps", self._gps_odom_callback, 10)
        self.create_subscription(Odometry, "/odometry/filtered/map", self._odom_callback, 10)
        self.create_subscription(Bool, "/power/charger_present", self._charger_callback, 10)
        self.create_subscription(Float32, "/power/charge_voltage", self._charge_voltage_callback, 10)

    def _clock_callback(self, _msg: Clock) -> None:
        self.clock_received = True

    def _map_grid_callback(self, _msg: OccupancyGrid) -> None:
        self.map_grid_received = True

    def _mowing_map_callback(self, msg: Map) -> None:
        self.mowing_map = msg

    def _diff_odom_callback(self, _msg: Odometry) -> None:
        self.diff_drive_odom_received = True

    def _gps_odom_callback(self, _msg: Odometry) -> None:
        if not self.gps_odom_received:
            self.filtered_odom_after_gps = False
            self.bounded_localization_since = None
        self.gps_odom_received = True

    def _odom_callback(self, msg: Odometry) -> None:
        self.filtered_odom = msg
        if self.gps_odom_received:
            self.filtered_odom_after_gps = True
        position = msg.pose.pose.position
        distance = math.hypot(position.x - NAVIGATION_GOAL_X, position.y - NAVIGATION_GOAL_Y)
        self.closest_navigation_goal_distance = min(self.closest_navigation_goal_distance, distance)

    def _charger_callback(self, msg: Bool) -> None:
        self.charger_present = msg.data

    def _charge_voltage_callback(self, msg: Float32) -> None:
        self.charge_voltage = msg.data

    def has_ready_state(self) -> bool:
        if not self.clock_received or not self.map_grid_received:
            return False
        if self.mowing_map is None or len(self.mowing_map.docking_stations) != 1:
            return False
        if not self.diff_drive_odom_received:
            return False
        if not self.gps_odom_received:
            return False
        if self.filtered_odom is None or not is_finite_odom(self.filtered_odom):
            return False
        if not self.filtered_odom_after_gps:
            return False
        if not self.has_bounded_filtered_pose():
            self.bounded_localization_since = None
            return False
        if self.bounded_localization_since is None:
            self.bounded_localization_since = time.monotonic()
            return False
        if time.monotonic() - self.bounded_localization_since < 1.0:
            return False
        return self.has_map_to_base_link_tf()

    def describe_state(self) -> str:
        dock_count = None if self.mowing_map is None else len(self.mowing_map.docking_stations)
        odom_ready = self.filtered_odom is not None and is_finite_odom(self.filtered_odom)
        tf_ready = self.has_map_to_base_link_tf()
        if self.mowing_map is None or not self.mowing_map.docking_stations:
            map_dock = "None"
        else:
            dock = self.mowing_map.docking_stations[0]
            dock_pose = dock.pose.pose
            dock_yaw = quaternion_to_yaw(dock_pose.orientation)
            map_dock = (
                f"{dock.id} ({dock_pose.position.x:.3f}, {dock_pose.position.y:.3f}, "
                f"yaw={dock_yaw:.3f})"
            )
        if self.filtered_odom is None:
            odom_pose = "None"
        else:
            position = self.filtered_odom.pose.pose.position
            yaw = quaternion_to_yaw(self.filtered_odom.pose.pose.orientation)
            odom_pose = (
                f"({position.x:.3f}, {position.y:.3f}, yaw={yaw:.3f}), "
                f"goal_distance={self.distance_to_navigation_goal():.3f}"
            )
        if self.navigation_feedback is None:
            nav_feedback = "None"
        else:
            feedback_pose = self.navigation_feedback.current_pose.pose
            feedback_yaw = quaternion_to_yaw(feedback_pose.orientation)
            nav_feedback = (
                f"pose=({feedback_pose.position.x:.3f}, {feedback_pose.position.y:.3f}, "
                f"yaw={feedback_yaw:.3f}), "
                f"distance_remaining={self.navigation_feedback.distance_remaining:.3f}, "
                f"recoveries={self.navigation_feedback.number_of_recoveries}"
            )
        if self.docking_feedback is None:
            docking_feedback = "None"
        else:
            feedback_dock = self.docking_feedback.chosen_docking_station
            feedback_dock_pose = feedback_dock.pose.pose
            feedback_dock_yaw = quaternion_to_yaw(feedback_dock_pose.orientation)
            docking_feedback = (
                f"status={self.docking_feedback.status}, "
                f"message={self.docking_feedback.message!r}, "
                f"retries={self.docking_feedback.num_retries}, "
                f"dock={feedback_dock.id} "
                f"({feedback_dock_pose.position.x:.3f}, "
                f"{feedback_dock_pose.position.y:.3f}, yaw={feedback_dock_yaw:.3f})"
            )
        return (
            "state: "
            f"test_time={self.get_clock().now().nanoseconds / 1e9:.3f}, "
            f"clock={self.clock_received}, "
            f"map_grid={self.map_grid_received}, "
            f"dock_count={dock_count}, "
            f"map_dock={map_dock}, "
            f"diff_drive_odom={self.diff_drive_odom_received}, "
            f"gps_odom={self.gps_odom_received}, "
            f"filtered_odom_after_gps={self.filtered_odom_after_gps}, "
            f"filtered_odom={odom_ready}, "
            f"filtered_pose={odom_pose}, "
            f"closest_goal_distance={self.closest_navigation_goal_distance:.3f}, "
            f"map_to_base_link_tf={tf_ready}, "
            f"charger_present={self.charger_present}, "
            f"charge_voltage={self.charge_voltage}, "
            f"navigation_feedback={nav_feedback}, "
            f"docking_feedback={docking_feedback}"
        )

    def has_map_to_base_link_tf(self) -> bool:
        try:
            return self.tf_buffer.can_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return False

    def has_bounded_filtered_pose(self) -> bool:
        if self.filtered_odom is None:
            return False
        position = self.filtered_odom.pose.pose.position
        return (
            LOCALIZATION_MIN_X <= position.x <= LOCALIZATION_MAX_X
            and LOCALIZATION_MIN_Y <= position.y <= LOCALIZATION_MAX_Y
        )

    def distance_to_navigation_goal(self) -> float:
        if self.filtered_odom is None:
            return math.inf
        pose = self.filtered_odom.pose.pose.position
        return math.hypot(pose.x - NAVIGATION_GOAL_X, pose.y - NAVIGATION_GOAL_Y)


def spin_until(node, predicate, timeout: float, failure_message: str, process, log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if predicate():
            return
        rclpy.spin_once(node, timeout_sec=0.1)

    state = node.describe_state() if hasattr(node, "describe_state") else ""
    pytest.fail(f"{failure_message}\n{state}\n{read_log_tail(log_path)}")


def wait_for_action_server(node, action_client, name: str, timeout: float, process, log_path: Path) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if action_client.wait_for_server(timeout_sec=1.0):
            return
        rclpy.spin_once(node, timeout_sec=0.1)

    pytest.fail(f"Timed out waiting for action server {name}\n{read_log_tail(log_path)}")


def wait_for_lifecycle_nodes_active(
    node, names: tuple[str, ...], timeout: float, process, log_path: Path
) -> None:
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

            response = wait_for_future(
                node,
                client.call_async(GetState.Request()),
                2.0,
                f"{name} lifecycle state",
                process,
                log_path,
            )
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


def wait_for_future(node, future, timeout: float, label: str, process, log_path: Path):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if future.done():
            return future.result()
        rclpy.spin_once(node, timeout_sec=0.1)

    state = node.describe_state() if hasattr(node, "describe_state") else ""
    pytest.fail(f"Timed out waiting for {label}\n{state}\n{read_log_tail(log_path)}")


def spin_for(node, seconds: float, process, log_path: Path) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        check_process(process, log_path)
        rclpy.spin_once(node, timeout_sec=0.1)


def send_navigation_goal(node: NavigationDockingTestNode, process, log_path: Path):
    initial_distance = node.distance_to_navigation_goal()
    assert initial_distance > NAVIGATION_GOAL_TOLERANCE, (
        f"Navigation goal starts too close to the robot: distance={initial_distance:.3f}\n"
        f"{node.describe_state()}"
    )

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = NAVIGATION_GOAL_X
    goal.pose.pose.position.y = NAVIGATION_GOAL_Y
    goal.pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(NAVIGATION_GOAL_YAW)
    goal.pose.pose.orientation.x = qx
    goal.pose.pose.orientation.y = qy
    goal.pose.pose.orientation.z = qz
    goal.pose.pose.orientation.w = qw

    goal_handle = None
    accept_deadline = time.monotonic() + 45.0
    while time.monotonic() < accept_deadline:
        goal_future = node.navigate_client.send_goal_async(
            goal, feedback_callback=lambda feedback: setattr(node, "navigation_feedback", feedback.feedback)
        )
        goal_handle = wait_for_future(
            node, goal_future, 10.0, "NavigateToPose goal response", process, log_path
        )
        if goal_handle.accepted:
            break
        spin_for(node, 1.0, process, log_path)

    assert goal_handle is not None and goal_handle.accepted, (
        f"NavigateToPose goal was rejected\n{node.describe_state()}\n{read_log_tail(log_path)}"
    )

    result_future = goal_handle.get_result_async()
    result_response = None
    deadline = time.monotonic() + 120.0
    while time.monotonic() < deadline:
        check_process(process, log_path)
        if node.distance_to_navigation_goal() <= NAVIGATION_GOAL_TOLERANCE:
            break
        if result_future.done():
            result_response = result_future.result()
            break
        rclpy.spin_once(node, timeout_sec=0.1)

    if result_response is None and node.distance_to_navigation_goal() > NAVIGATION_GOAL_TOLERANCE:
        pytest.fail(
            "Timed out waiting for NavigateToPose to reach the waypoint\n"
            f"{node.describe_state()}\n{read_log_tail(log_path)}"
        )

    if result_response is None and not result_future.done():
        cancel_future = goal_handle.cancel_goal_async()
        cancel_response = wait_for_future(
            node, cancel_future, 15.0, "NavigateToPose cancellation", process, log_path
        )
        if cancel_response.goals_canceling:
            cancel_result = wait_for_future(
                node, result_future, 15.0, "NavigateToPose canceled result", process, log_path
            )
            assert cancel_result.status in (
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_SUCCEEDED,
            ), (
                f"NavigateToPose reached the waypoint but finished with status={cancel_result.status}\n"
                f"{node.describe_state()}\n{read_log_tail(log_path)}"
            )
            spin_for(node, 1.0, process, log_path)
            return
        if result_future.done():
            result_response = result_future.result()
        else:
            pytest.fail(
                "NavigateToPose reached the waypoint but cancellation was not accepted\n"
                f"{node.describe_state()}\n{read_log_tail(log_path)}"
            )

    if result_response is None:
        result_response = result_future.result()

    assert result_response.status == GoalStatus.STATUS_SUCCEEDED, (
        f"NavigateToPose failed with status={result_response.status}, "
        f"error_code={result_response.result.error_code}, error_msg={result_response.result.error_msg!r}\n"
        f"{read_log_tail(log_path)}"
    )
    assert result_response.result.error_code == NavigateToPose.Result.NONE, (
        f"NavigateToPose returned error_code={result_response.result.error_code}, "
        f"error_msg={result_response.result.error_msg!r}\n{read_log_tail(log_path)}"
    )
    assert node.distance_to_navigation_goal() <= NAVIGATION_GOAL_TOLERANCE, (
        f"NavigateToPose reported success away from the requested waypoint\n"
        f"{node.describe_state()}\n{read_log_tail(log_path)}"
    )
    spin_for(node, 1.0, process, log_path)


def send_docking_goal(node: NavigationDockingTestNode, process, log_path: Path):
    goal = DockRobotNearest.Goal()
    goal_future = node.dock_client.send_goal_async(
        goal, feedback_callback=lambda feedback: setattr(node, "docking_feedback", feedback.feedback)
    )
    goal_handle = wait_for_future(node, goal_future, 15.0, "DockRobotNearest goal response", process, log_path)
    assert goal_handle.accepted, f"DockRobotNearest goal was rejected\n{read_log_tail(log_path)}"

    result_future = goal_handle.get_result_async()
    result_response = wait_for_future(
        node, result_future, 180.0, "DockRobotNearest result", process, log_path
    )

    result = result_response.result
    assert result_response.status == GoalStatus.STATUS_SUCCEEDED, (
        f"DockRobotNearest failed with status={result_response.status}, "
        f"code={result.code}, message={result.message!r}\n{read_log_tail(log_path)}"
    )
    assert result.code == DockRobotNearest.Result.CODE_SUCCESS, (
        f"DockRobotNearest returned code={result.code}, message={result.message!r}\n"
        f"{read_log_tail(log_path)}"
    )


def test_navigate_to_point_then_dock(tmp_path):
    log_path = tmp_path / "navigation_docking.log"
    env = prepare_env()
    os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]

    process, log_file = start_simulation(log_path, env)
    rclpy.init()
    node = NavigationDockingTestNode()

    try:
        spin_until(
            node,
            node.has_ready_state,
            120.0,
            "Simulation did not reach navigation-ready state",
            process,
            log_path,
        )
        wait_for_action_server(node, node.navigate_client, "/navigate_to_pose", 60.0, process, log_path)
        wait_for_action_server(node, node.dock_client, "/dock_robot_nearest", 60.0, process, log_path)
        wait_for_lifecycle_nodes_active(node, NAV2_LIFECYCLE_NODES, 60.0, process, log_path)

        send_navigation_goal(node, process, log_path)
        send_docking_goal(node, process, log_path)
        spin_until(
            node,
            lambda: node.charger_present is True and node.charge_voltage is not None and node.charge_voltage > 1.0,
            20.0,
            "Robot did not report charger contact after docking",
            process,
            log_path,
        )

        assert process.poll() is None, f"Simulation exited early with code {process.returncode}"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        stop_simulation(process)
        log_file.close()

    assert_no_launch_failures(log_path)
