#!/usr/bin/env python3
"""Convert supported OpenMower Webots worlds into OpenMower GeoJSON maps."""

from __future__ import annotations

import argparse
import ast
import json
import math
import re
import sys
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

TREE_TRUNK_RADIUS = 0.12
TREE_TRUNK_SEGMENTS = 16
DOCK_CONTACT_OFFSET_X = 0.32
DOCK_ORIENTATION_LENGTH = 0.5
FEATURE_NAMESPACE = uuid.UUID("2e102a30-82dc-4338-8400-edfef8ef00f3")

AREA_COLORS = {
    "navigation": "#0000ff",
    "operation": "#00ff00",
    "exclusion": "#ff0000",
}


@dataclass(frozen=True)
class WebotsNode:
    type: str
    fields: dict[str, object]
    tokens: list[str]


@dataclass(frozen=True)
class WebotsWorld:
    name: str
    datum: tuple[float, float, float]
    nodes: list[WebotsNode]


@dataclass(frozen=True)
class LocalFeature:
    feature_type: str
    name: str
    geometry_type: str
    coordinates: list[tuple[float, float]]


def strip_comments(text: str) -> str:
    output: list[str] = []
    in_string = False
    escaped = False
    i = 0
    while i < len(text):
        char = text[i]
        if in_string:
            output.append(char)
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            i += 1
            continue

        if char == '"':
            in_string = True
            output.append(char)
            i += 1
            continue

        if char == "#":
            while i < len(text) and text[i] != "\n":
                i += 1
            continue

        output.append(char)
        i += 1
    return "".join(output)


TOKEN_RE = re.compile(
    r'"(?:\\.|[^"\\])*"'
    r"|[{}\[\]]"
    r"|[+-]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?"
    r"|[A-Za-z_][A-Za-z0-9_]*"
)


def tokenize(text: str) -> list[str]:
    return TOKEN_RE.findall(strip_comments(text))


def is_number(token: str) -> bool:
    try:
        float(token)
    except ValueError:
        return False
    return True


def is_identifier(token: str) -> bool:
    return bool(re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", token))


def parse_string(token: str) -> str:
    return ast.literal_eval(token)


def iter_direct_token_positions(tokens: Sequence[str]) -> Iterable[tuple[int, str]]:
    brace_depth = 0
    bracket_depth = 0
    for index, token in enumerate(tokens):
        if token == "{":
            brace_depth += 1
            continue
        if token == "}":
            brace_depth -= 1
            continue
        if token == "[":
            bracket_depth += 1
            continue
        if token == "]":
            bracket_depth -= 1
            continue
        if brace_depth == 0 and bracket_depth == 0:
            yield index, token


def direct_numbers(tokens: Sequence[str], field: str, count: int) -> tuple[float, ...] | None:
    direct_positions = dict(iter_direct_token_positions(tokens))
    for index, token in direct_positions.items():
        if token != field:
            continue
        values: list[float] = []
        cursor = index + 1
        while cursor < len(tokens) and len(values) < count:
            if cursor in direct_positions and is_identifier(tokens[cursor]) and values:
                break
            if tokens[cursor] in "{}[]":
                break
            if is_number(tokens[cursor]):
                values.append(float(tokens[cursor]))
            elif values:
                break
            cursor += 1
        if len(values) == count:
            return tuple(values)
    return None


def direct_string(tokens: Sequence[str], field: str) -> str | None:
    direct_positions = dict(iter_direct_token_positions(tokens))
    for index, token in direct_positions.items():
        if token != field:
            continue
        cursor = index + 1
        while cursor < len(tokens):
            if cursor in direct_positions and is_identifier(tokens[cursor]):
                return None
            if tokens[cursor].startswith('"'):
                return parse_string(tokens[cursor])
            if tokens[cursor] in "{}[]":
                return None
            cursor += 1
    return None


def collect_braced_block(tokens: Sequence[str], open_brace_index: int) -> tuple[list[str], int]:
    depth = 1
    cursor = open_brace_index + 1
    content: list[str] = []
    while cursor < len(tokens):
        token = tokens[cursor]
        if token == "{":
            depth += 1
        elif token == "}":
            depth -= 1
            if depth == 0:
                return content, cursor + 1
        content.append(token)
        cursor += 1
    raise ValueError("Unterminated braced block")


def top_level_nodes(tokens: Sequence[str]) -> Iterable[tuple[str, list[str]]]:
    cursor = 0
    while cursor < len(tokens):
        token = tokens[cursor]
        if is_identifier(token) and cursor + 1 < len(tokens) and tokens[cursor + 1] == "{":
            content, cursor = collect_braced_block(tokens, cursor + 1)
            yield token, content
            continue
        cursor += 1


def nested_first_box_size(tokens: Sequence[str]) -> tuple[float, float, float] | None:
    cursor = 0
    while cursor < len(tokens):
        if tokens[cursor] == "Box" and cursor + 1 < len(tokens) and tokens[cursor + 1] == "{":
            content, next_cursor = collect_braced_block(tokens, cursor + 1)
            size = direct_numbers(content, "size", 3)
            if size is not None:
                return size
            cursor = next_cursor
            continue
        cursor += 1
    return None


def parse_node(node_type: str, tokens: list[str]) -> WebotsNode:
    fields: dict[str, object] = {}
    translation = direct_numbers(tokens, "translation", 3)
    rotation = direct_numbers(tokens, "rotation", 4)
    size = direct_numbers(tokens, "size", 3)
    gps_reference = direct_numbers(tokens, "gpsReference", 3)
    name = direct_string(tokens, "name")
    box_size = nested_first_box_size(tokens)

    if translation is not None:
        fields["translation"] = translation
    if rotation is not None:
        fields["rotation"] = rotation
    if size is not None:
        fields["size"] = size
    if gps_reference is not None:
        fields["gpsReference"] = gps_reference
    if name is not None:
        fields["name"] = name
    if box_size is not None:
        fields["box_size"] = box_size

    return WebotsNode(node_type, fields, tokens)


def parse_world(path: Path) -> WebotsWorld:
    tokens = tokenize(path.read_text(encoding="utf-8"))
    nodes = [parse_node(node_type, content) for node_type, content in top_level_nodes(tokens)]
    world_info = next((node for node in nodes if node.type == "WorldInfo"), None)
    if world_info is None or "gpsReference" not in world_info.fields:
        raise ValueError(f"{path} does not define WorldInfo.gpsReference")

    datum = world_info.fields["gpsReference"]
    assert isinstance(datum, tuple)
    return WebotsWorld(path.stem, datum, nodes)


def translation(node: WebotsNode) -> tuple[float, float, float]:
    value = node.fields.get("translation", (0.0, 0.0, 0.0))
    assert isinstance(value, tuple)
    return value


def size(node: WebotsNode, default: tuple[float, float, float]) -> tuple[float, float, float]:
    value = node.fields.get("size", default)
    assert isinstance(value, tuple)
    return value


def node_name(node: WebotsNode) -> str:
    value = node.fields.get("name", node.type)
    assert isinstance(value, str)
    return value


def yaw_from_node(node: WebotsNode) -> float:
    rotation = node.fields.get("rotation", (0.0, 0.0, 1.0, 0.0))
    assert isinstance(rotation, tuple)
    axis_x, axis_y, axis_z, angle = rotation
    if abs(axis_x) < 1.0e-6 and abs(axis_y) < 1.0e-6 and abs(axis_z) > 1.0e-6:
        return math.copysign(angle, axis_z)
    return 0.0


def rotate_point(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw


def transform_local_point(
    origin: tuple[float, float, float], yaw: float, point: tuple[float, float]
) -> tuple[float, float]:
    rotated_x, rotated_y = rotate_point(point[0], point[1], yaw)
    return origin[0] + rotated_x, origin[1] + rotated_y


def rectangle_polygon(
    origin: tuple[float, float, float], footprint_size: tuple[float, float, float], yaw: float
) -> list[tuple[float, float]]:
    half_x = footprint_size[0] / 2.0
    half_y = footprint_size[1] / 2.0
    corners = [
        (-half_x, -half_y),
        (half_x, -half_y),
        (half_x, half_y),
        (-half_x, half_y),
    ]
    points = [transform_local_point(origin, yaw, corner) for corner in corners]
    points.append(points[0])
    return points


def circle_polygon(
    center: tuple[float, float, float], radius: float, segments: int
) -> list[tuple[float, float]]:
    points = [
        (
            center[0] + radius * math.cos(2.0 * math.pi * index / segments),
            center[1] + radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]
    points.append(points[0])
    return points


def operation_from_curbs(world: WebotsWorld) -> LocalFeature | None:
    min_x_edges: list[float] = []
    max_x_edges: list[float] = []
    min_y_edges: list[float] = []
    max_y_edges: list[float] = []

    for node in world.nodes:
        if node.type != "Curb":
            continue
        polygon = rectangle_polygon(translation(node), size(node, (1.0, 0.12, 0.07)), yaw_from_node(node))
        xs = [point[0] for point in polygon[:-1]]
        ys = [point[1] for point in polygon[:-1]]
        width = max(xs) - min(xs)
        height = max(ys) - min(ys)
        center_x, center_y, _ = translation(node)
        if height > width:
            if center_x < 0.0:
                min_x_edges.append(max(xs))
            else:
                max_x_edges.append(min(xs))
        elif width > height:
            if center_y < 0.0:
                min_y_edges.append(max(ys))
            else:
                max_y_edges.append(min(ys))

    if not (min_x_edges and max_x_edges and min_y_edges and max_y_edges):
        return None

    min_x = max(min_x_edges)
    max_x = min(max_x_edges)
    min_y = max(min_y_edges)
    max_y = min(max_y_edges)
    if min_x >= max_x or min_y >= max_y:
        return None

    return LocalFeature(
        "operation",
        f"{world.name}_operation_area",
        "Polygon",
        [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)],
    )


def operation_from_floor(world: WebotsWorld) -> LocalFeature | None:
    candidates: list[tuple[float, WebotsNode, tuple[float, float, float]]] = []
    for node in world.nodes:
        if node.type != "Solid":
            continue
        name = node_name(node).lower()
        if not any(part in name for part in ("grass", "lawn", "floor")):
            continue
        box_size = node.fields.get("box_size")
        if not isinstance(box_size, tuple):
            continue
        candidates.append((box_size[0] * box_size[1], node, box_size))

    if not candidates:
        return None

    _, node, box_size = max(candidates, key=lambda candidate: candidate[0])
    return LocalFeature(
        "operation",
        f"{world.name}_operation_area",
        "Polygon",
        rectangle_polygon(translation(node), box_size, yaw_from_node(node)),
    )


def build_local_features(world: WebotsWorld) -> list[LocalFeature]:
    features: list[LocalFeature] = []
    operation = operation_from_curbs(world) or operation_from_floor(world)
    if operation is None:
        raise ValueError(f"Could not infer an operation area from {world.name}")
    features.append(operation)

    for node in world.nodes:
        if node.type == "GardenBuilding":
            features.append(
                LocalFeature(
                    "exclusion",
                    node_name(node),
                    "Polygon",
                    rectangle_polygon(translation(node), size(node, (4.0, 3.0, 2.0)), yaw_from_node(node)),
                )
            )
        elif node.type == "GardenTree":
            features.append(
                LocalFeature(
                    "exclusion",
                    node_name(node),
                    "Polygon",
                    circle_polygon(translation(node), TREE_TRUNK_RADIUS, TREE_TRUNK_SEGMENTS),
                )
            )
        elif node.type == "DockingStation":
            origin = translation(node)
            yaw = yaw_from_node(node)
            contact = transform_local_point(origin, yaw, (DOCK_CONTACT_OFFSET_X, 0.0))
            orientation = transform_local_point(
                origin, yaw, (DOCK_CONTACT_OFFSET_X - DOCK_ORIENTATION_LENGTH, 0.0)
            )
            features.append(
                LocalFeature("docking_station", node_name(node), "LineString", [contact, orientation])
            )

    return features


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (radius + alt_m) * cos_lat * math.cos(lon)
    y = (radius + alt_m) * cos_lat * math.sin(lon)
    z = (radius * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    alt = 0.0
    for _ in range(8):
        sin_lat = math.sin(lat)
        radius = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        alt = p / math.cos(lat) - radius
        lat = math.atan2(z, p * (1.0 - WGS84_E2 * radius / (radius + alt)))
    return math.degrees(lat), math.degrees(lon), alt


def enu_to_ecef(
    east: float,
    north: float,
    up: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_ecef: tuple[float, float, float],
) -> tuple[float, float, float]:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
    dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
    dz = cos_lat * north + sin_lat * up
    return ref_ecef[0] + dx, ref_ecef[1] + dy, ref_ecef[2] + dz


def local_to_lon_lat(point: tuple[float, float], datum: tuple[float, float, float]) -> list[float]:
    ref_ecef = geodetic_to_ecef(datum[0], datum[1], datum[2])
    ecef = enu_to_ecef(point[0], point[1], 0.0, datum[0], datum[1], ref_ecef)
    lat, lon, _ = ecef_to_geodetic(*ecef)
    return [lon, lat]


def feature_id(world_name: str, feature: LocalFeature) -> str:
    return str(uuid.uuid5(FEATURE_NAMESPACE, f"{world_name}:{feature.feature_type}:{feature.name}"))


def area_feature_to_geojson(
    world: WebotsWorld, feature: LocalFeature, datum: tuple[float, float, float]
) -> dict[str, object]:
    color = AREA_COLORS[feature.feature_type]
    return {
        "type": "Feature",
        "properties": {
            "id": feature_id(world.name, feature),
            "name": feature.name,
            "type": feature.feature_type,
            "fill": color,
            "style": {"color": color},
        },
        "geometry": {
            "type": "Polygon",
            "coordinates": [[local_to_lon_lat(point, datum) for point in feature.coordinates]],
        },
    }


def dock_feature_to_geojson(
    world: WebotsWorld, feature: LocalFeature, datum: tuple[float, float, float]
) -> dict[str, object]:
    return {
        "type": "Feature",
        "properties": {
            "id": feature_id(world.name, feature),
            "name": feature.name,
            "type": "docking_station",
        },
        "geometry": {
            "type": "LineString",
            "coordinates": [local_to_lon_lat(point, datum) for point in feature.coordinates],
        },
    }


def local_features_to_geojson(world: WebotsWorld, features: Sequence[LocalFeature]) -> dict[str, object]:
    geojson_features: list[dict[str, object]] = []
    for feature in features:
        if feature.geometry_type == "Polygon":
            geojson_features.append(area_feature_to_geojson(world, feature, world.datum))
        elif feature.geometry_type == "LineString":
            geojson_features.append(dock_feature_to_geojson(world, feature, world.datum))
        else:
            raise ValueError(f"Unsupported geometry type: {feature.geometry_type}")
    return {"type": "FeatureCollection", "features": geojson_features}


def convert_world(path: Path) -> dict[str, object]:
    world = parse_world(path)
    return local_features_to_geojson(world, build_local_features(world))


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert supported OpenMower Webots worlds into OpenMower GeoJSON maps."
    )
    parser.add_argument("world", type=Path, help="Input Webots .wbt world")
    parser.add_argument("--output", "-o", type=Path, help="Output GeoJSON path; stdout if omitted")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    geojson = convert_world(args.world)
    output = json.dumps(geojson, indent=2) + "\n"
    if args.output is None:
        sys.stdout.write(output)
        return 0

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
