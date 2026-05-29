import importlib.util
import math
import sys
from pathlib import Path


def load_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "webots_world_to_geojson.py"
    spec = importlib.util.spec_from_file_location("webots_world_to_geojson", script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


converter = load_module()


def realistic_garden_features():
    root = Path(__file__).resolve().parents[1]
    world = converter.parse_world(root / "worlds" / "realistic_garden.wbt")
    return world, converter.build_local_features(world)


def find_feature(features, feature_type, name):
    for feature in features:
        if feature.feature_type == feature_type and feature.name == name:
            return feature
    raise AssertionError(f"Feature not found: {feature_type} {name}")


def bounds(points):
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return min(xs), min(ys), max(xs), max(ys)


def test_realistic_garden_operation_area_uses_curb_inner_edges():
    _, features = realistic_garden_features()

    operation = find_feature(features, "operation", "realistic_garden_operation_area")
    actual_bounds = bounds(operation.coordinates)

    for actual, expected in zip(actual_bounds, (-4.12, -3.62, 4.12, 3.62)):
        assert math.isclose(actual, expected, abs_tol=1.0e-9)


def test_realistic_garden_tree_exclusion_is_trunk_only():
    _, features = realistic_garden_features()

    oak = find_feature(features, "exclusion", "oak_tree")

    radii = [math.hypot(x + 2.6, y - 1.8) for x, y in oak.coordinates[:-1]]

    assert len(radii) == converter.TREE_TRUNK_SEGMENTS
    assert all(math.isclose(radius, converter.TREE_TRUNK_RADIUS, abs_tol=1.0e-9) for radius in radii)


def test_realistic_garden_docking_station_matches_webots_contact():
    _, features = realistic_garden_features()

    dock = find_feature(features, "docking_station", "docking_station")

    expected = [(1.82, 1.5), (1.32, 1.5)]
    for actual_point, expected_point in zip(dock.coordinates, expected):
        for actual, expected_value in zip(actual_point, expected_point):
            assert math.isclose(actual, expected_value, abs_tol=1.0e-9)


def test_realistic_garden_geojson_is_deterministic_feature_collection():
    world, features = realistic_garden_features()

    geojson = converter.local_features_to_geojson(world, features)
    repeated_geojson = converter.local_features_to_geojson(world, features)

    assert geojson == repeated_geojson
    assert geojson["type"] == "FeatureCollection"
    assert len(geojson["features"]) == 5
    assert geojson["features"][0]["properties"]["type"] == "operation"
    assert geojson["features"][-1]["properties"]["type"] == "docking_station"
