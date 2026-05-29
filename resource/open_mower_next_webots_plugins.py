import ctypes
import math
import random
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus

from controller import Field, Node
from controller.wb import wb


wb.wb_supervisor_field_get_mf_node.restype = ctypes.c_void_p


@dataclass
class GnssZone:
    name: str
    shape: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    radius: float
    sigma_xy: float
    bias_x: float
    bias_y: float
    dropout_probability: float
    covariance_xy: float
    edge_width: float
    drift_time_constant: float

    def influence_at(self, x: float, y: float) -> float:
        dx = x - self.x
        dy = y - self.y
        if self.shape in ("circle", "cylinder"):
            radius = self.radius if self.radius > 0.0 else max(self.size_x, self.size_y) * 0.5
            distance_to_edge = radius - math.hypot(dx, dy)
            return _edge_influence(distance_to_edge, min(self.edge_width, radius))

        cos_yaw = math.cos(-self.yaw)
        sin_yaw = math.sin(-self.yaw)
        local_x = cos_yaw * dx - sin_yaw * dy
        local_y = sin_yaw * dx + cos_yaw * dy
        half_x = self.size_x * 0.5
        half_y = self.size_y * 0.5
        if half_x <= 0.0 or half_y <= 0.0:
            return 0.0
        distance_to_edge = min(half_x - abs(local_x), half_y - abs(local_y))
        return _edge_influence(distance_to_edge, min(self.edge_width, half_x, half_y))


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _edge_influence(distance_to_edge: float, edge_width: float) -> float:
    if distance_to_edge <= 0.0:
        return 0.0
    if edge_width <= 0.0 or distance_to_edge >= edge_width:
        return 1.0
    return _smoothstep(distance_to_edge / edge_width)


def _get_mf_node(field: Field, index: int):
    ref = wb.wb_supervisor_field_get_mf_node(field._ref, index)
    return Node(ref=ref) if ref else None


def _field_value(node: Node, name: str, default):
    field = node.getField(name)
    if field is None:
        return default
    value = field.value
    return default if value is None else value


def _field_float(node: Node, name: str, default: float) -> float:
    try:
        return float(_field_value(node, name, default))
    except (TypeError, ValueError):
        return default


def _field_string(node: Node, name: str, default: str) -> str:
    value = _field_value(node, name, default)
    return value if isinstance(value, str) else default


def _field_vec3(node: Node, name: str, default):
    value = _field_value(node, name, default)
    if isinstance(value, list) and len(value) == 3:
        return [float(value[0]), float(value[1]), float(value[2])]
    return default


def _field_rotation_yaw(node: Node) -> float:
    rotation = _field_value(node, "rotation", [0.0, 0.0, 1.0, 0.0])
    if not isinstance(rotation, list) or len(rotation) != 4:
        return 0.0
    axis_z = float(rotation[2])
    angle = float(rotation[3])
    return angle if axis_z >= 0.0 else -angle


def _clamp_probability(value: float) -> float:
    return max(0.0, min(1.0, value))


def _parse_metadata(text: str) -> dict[str, str]:
    values = {}
    for item in text.replace("\n", ";").split(";"):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        key = key.strip()
        if key:
            values[key] = value.strip()
    return values


def _metadata_float(metadata: dict[str, str], name: str, default: float) -> float:
    try:
        return float(metadata.get(name, default))
    except (TypeError, ValueError):
        return default


def _stamp_seconds(stamp):
    try:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except AttributeError:
        return None


class GnssDegradationPlugin:
    def init(self, webots_node, properties):
        if not rclpy.ok():
            rclpy.init(args=None)

        self._robot = webots_node.robot
        self._node = rclpy.create_node("sim_gnss_degradation_node")
        self._rng = random.Random(int(properties.get("seed", "7")))
        self._zones = []
        self._zone_drift = {}
        self._logged_supervisor_warning = False

        self._input_fix_topic = properties.get("inputFixTopic", "/sim/gps/fix_raw")
        self._output_fix_topic = properties.get("outputFixTopic", "/gps/fix")
        self._input_speed_vector_topic = properties.get(
            "inputSpeedVectorTopic", f"{self._input_fix_topic}/speed_vector"
        )
        self._output_speed_vector_topic = properties.get(
            "outputSpeedVectorTopic", f"{self._output_fix_topic}/speed_vector"
        )

        self._fix_publisher = self._node.create_publisher(NavSatFix, self._output_fix_topic, 10)
        self._speed_vector_publisher = self._node.create_publisher(
            Vector3, self._output_speed_vector_topic, 10
        )
        self._fix_subscription = self._node.create_subscription(
            NavSatFix, self._input_fix_topic, self._fix_callback, 10
        )
        self._speed_vector_subscription = self._node.create_subscription(
            Vector3, self._input_speed_vector_topic, self._speed_vector_callback, 10
        )

        self._refresh_zones()
        self._node.get_logger().info(
            f"GNSS degradation publishing {self._output_fix_topic} from {self._input_fix_topic}; "
            f"loaded {len(self._zones)} Webots zone(s)"
        )

    def step(self):
        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.0)

    def stop(self):
        if getattr(self, "_node", None) is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _refresh_zones(self):
        self._zones = []
        if not self._robot.getSupervisor():
            if not self._logged_supervisor_warning:
                self._node.get_logger().warn(
                    "OpenMower Webots robot is not a supervisor; GNSS degradation zones are ignored"
                )
                self._logged_supervisor_warning = True
            return

        root = self._robot.getRoot()
        children = root.getField("children")
        if children is None:
            return

        for index in range(children.getCount()):
            child = _get_mf_node(children, index)
            zone = self._zone_from_node(child) if child is not None else None
            if zone is not None:
                self._zones.append(zone)

    def _zone_from_node(self, node):
        metadata_field = node.getField("metadata")
        if metadata_field is None:
            return None

        metadata = _parse_metadata(_field_string(node, "metadata", ""))

        try:
            position = node.getPosition()
        except Exception:
            position = _field_vec3(node, "translation", [0.0, 0.0, 0.0])
        size = _field_vec3(node, "size", [1.0, 1.0, 0.0])
        name = _field_string(node, "name", node.getTypeName())
        shape = metadata.get("shape", "box").lower()
        sigma_xy = max(0.0, _metadata_float(metadata, "sigmaXY", 0.0))
        covariance_xy = max(0.0, _metadata_float(metadata, "covarianceXY", sigma_xy * sigma_xy))
        radius = max(0.0, _metadata_float(metadata, "radius", 0.0))
        if shape in ("circle", "cylinder"):
            default_radius = radius if radius > 0.0 else max(size[0], size[1]) * 0.5
            default_edge_width = max(0.1, 0.35 * default_radius)
        else:
            default_edge_width = max(0.1, 0.25 * min(size[0], size[1]))

        return GnssZone(
            name=name,
            shape=shape,
            x=float(position[0]),
            y=float(position[1]),
            yaw=_field_rotation_yaw(node),
            size_x=max(0.0, float(size[0])),
            size_y=max(0.0, float(size[1])),
            radius=radius,
            sigma_xy=sigma_xy,
            bias_x=_metadata_float(metadata, "biasX", 0.0),
            bias_y=_metadata_float(metadata, "biasY", 0.0),
            dropout_probability=_clamp_probability(
                _metadata_float(metadata, "dropoutProbability", 0.0)
            ),
            covariance_xy=covariance_xy,
            edge_width=max(0.0, _metadata_float(metadata, "edgeWidth", default_edge_width)),
            drift_time_constant=max(
                0.1, _metadata_float(metadata, "driftTimeConstant", 8.0)
            ),
        )

    def _fix_callback(self, msg: NavSatFix):
        if not self._zones or not self._robot.getSupervisor():
            self._fix_publisher.publish(msg)
            return

        try:
            robot_position = self._robot.getSelf().getPosition()
        except Exception:
            self._fix_publisher.publish(msg)
            return

        active_zones = [
            (zone, influence)
            for zone in self._zones
            if (influence := zone.influence_at(robot_position[0], robot_position[1])) > 0.0
        ]
        if not active_zones:
            self._fix_publisher.publish(msg)
            return

        self._fix_publisher.publish(self._degrade_fix(msg, active_zones))

    def _degrade_fix(self, msg: NavSatFix, zones: list[tuple[GnssZone, float]]) -> NavSatFix:
        out = NavSatFix()
        out.header = msg.header
        out.status.status = msg.status.status
        out.status.service = msg.status.service
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = list(msg.position_covariance)
        out.position_covariance_type = msg.position_covariance_type

        effective_sigma_sq = sum((influence * zone.sigma_xy) ** 2 for zone, influence in zones)
        covariance_xy = sum(
            influence
            * influence
            * max(zone.covariance_xy, zone.sigma_xy * zone.sigma_xy)
            for zone, influence in zones
        )
        dropout_probability = 1.0
        for zone, influence in zones:
            dropout_probability *= 1.0 - zone.dropout_probability * influence * influence
        dropout_probability = 1.0 - dropout_probability

        stamp_seconds = _stamp_seconds(msg.header.stamp)
        offset_x = 0.0
        offset_y = 0.0
        for zone, influence in zones:
            drift_x, drift_y = self._drift_for_zone(zone, stamp_seconds)
            offset_x += influence * (zone.bias_x + drift_x)
            offset_y += influence * (zone.bias_y + drift_y)
        jitter_sigma = math.sqrt(effective_sigma_sq) * 0.15
        if jitter_sigma > 0.0:
            offset_x += self._rng.gauss(0.0, jitter_sigma)
            offset_y += self._rng.gauss(0.0, jitter_sigma)

        meters_per_degree_lat = 111320.0
        meters_per_degree_lon = max(
            1.0, meters_per_degree_lat * math.cos(math.radians(max(-89.9, min(89.9, msg.latitude))))
        )
        out.latitude += offset_y / meters_per_degree_lat
        out.longitude += offset_x / meters_per_degree_lon

        if self._rng.random() < dropout_probability:
            out.status.status = NavSatStatus.STATUS_NO_FIX

        out.position_covariance[0] = max(out.position_covariance[0], covariance_xy)
        out.position_covariance[4] = max(out.position_covariance[4], covariance_xy)
        out.position_covariance[8] = max(out.position_covariance[8], covariance_xy)
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        return out

    def _drift_for_zone(self, zone: GnssZone, stamp_seconds):
        if zone.sigma_xy <= 0.0:
            return 0.0, 0.0

        drift_x, drift_y, last_stamp = self._zone_drift.get(zone.name, (0.0, 0.0, None))
        dt = 0.2
        if stamp_seconds is not None and last_stamp is not None:
            dt = max(0.0, min(2.0, stamp_seconds - last_stamp))
        if dt > 0.0:
            alpha = math.exp(-dt / zone.drift_time_constant)
            drift_sigma = zone.sigma_xy * 0.85
            process_sigma = drift_sigma * math.sqrt(max(0.0, 1.0 - alpha * alpha))
            drift_x = alpha * drift_x + self._rng.gauss(0.0, process_sigma)
            drift_y = alpha * drift_y + self._rng.gauss(0.0, process_sigma)
        self._zone_drift[zone.name] = (drift_x, drift_y, stamp_seconds)
        return drift_x, drift_y

    def _speed_vector_callback(self, msg: Vector3):
        self._speed_vector_publisher.publish(msg)
