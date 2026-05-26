import importlib.util
import math
import sys
from pathlib import Path


def load_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "calibrate_robot.py"
    spec = importlib.util.spec_from_file_location("calibrate_robot", script)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


calibrate_robot = load_module()


def test_gps_samples_to_enu_starts_at_origin_and_tracks_east():
    samples = [
        calibrate_robot.GpsFixSample(0.0, 0.0, 0.0, 0.0, None, None, None, "test"),
        calibrate_robot.GpsFixSample(1.0, 0.0, 0.00001, 0.0, None, None, None, "test"),
    ]

    points = calibrate_robot.gps_samples_to_enu(samples)

    assert math.isclose(points[0].x, 0.0, abs_tol=1.0e-6)
    assert math.isclose(points[0].y, 0.0, abs_tol=1.0e-6)
    assert 1.10 < points[1].x < 1.12
    assert abs(points[1].y) < 1.0e-4


def test_odom_yaw_delta_unwraps_across_pi_boundary():
    samples = [
        calibrate_robot.OdomSample(0.0, 0.0, 0.0, math.radians(179.0), 0.0, 0.0, 0.0),
        calibrate_robot.OdomSample(1.0, 0.0, 0.0, math.radians(-179.0), 0.0, 0.0, 0.0),
    ]

    delta = calibrate_robot.odom_yaw_delta(samples)

    assert math.isclose(delta, math.radians(2.0), rel_tol=1.0e-6)


def test_integrate_imu_yaw_uses_trapezoid_rule():
    samples = [
        calibrate_robot.ImuSample(0.0, 0.5),
        calibrate_robot.ImuSample(1.0, 0.5),
        calibrate_robot.ImuSample(2.0, 0.5),
    ]

    delta = calibrate_robot.integrate_imu_yaw(samples)

    assert math.isclose(delta, 1.0, rel_tol=1.0e-6)


def test_speed_summary_ignores_unstable_entries_for_stable_limits():
    entries = [
        {"actual_median": 0.10, "stable": True, "observed_accel": 0.4},
        {"actual_median": 0.20, "stable": True, "observed_accel": 0.6},
        {"actual_median": 0.25, "stable": False, "observed_accel": 0.7},
    ]

    summary = calibrate_robot.stable_speed_summary(entries)

    assert summary["max_observed"] == 0.25
    assert summary["max_stable"] == 0.20
    assert summary["min_stable"] == 0.10
    assert summary["observed_accel_median"] == 0.5


def test_set_nested_updates_list_item():
    data = {"wheel": {"offset": [0.0, 0.16, 0.0]}}

    calibrate_robot.set_nested(data, ["wheel", "offset", 1], 0.18)

    assert data["wheel"]["offset"] == [0.0, 0.18, 0.0]
