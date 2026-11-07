import math

import pytest

from wheelmo_sim.wheelchair_spec import WheelchairSpec, yaw_pitch_quat_xyzw


def _rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    # v' = v + 2w (u x v) + 2 u x (u x v), with u = (x, y, z)
    ux = (y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0])
    uux = (y * ux[2] - z * ux[1], z * ux[0] - x * ux[2], x * ux[1] - y * ux[0])
    return tuple(v[i] + 2 * w * ux[i] + 2 * uux[i] for i in range(3))


def test_wheel_speeds_straight_and_spin():
    spec = WheelchairSpec()
    left, right = spec.wheel_speeds(0.5, 0.0)
    assert left == pytest.approx(right) == pytest.approx(0.5 / spec.drive_wheel_radius)
    left, right = spec.wheel_speeds(0.0, 1.0)  # turn left in place
    assert left == pytest.approx(-right)
    assert right > 0


def test_camera_quaternions_point_where_expected():
    forward_down = _rotate(yaw_pitch_quat_xyzw(0.0, 15.0), (1.0, 0.0, 0.0))
    assert forward_down[0] > 0.9 and forward_down[2] == pytest.approx(-math.sin(math.radians(15)))
    backward_down = _rotate(yaw_pitch_quat_xyzw(180.0, 20.0), (1.0, 0.0, 0.0))
    assert backward_down[0] < -0.9 and backward_down[2] == pytest.approx(-math.sin(math.radians(20)))


def test_sensor_mounts_sit_above_the_chair():
    spec = WheelchairSpec()
    x_min, x_max, y_min, y_max = spec.footprint
    assert x_min <= spec.lidar_pos[0] <= x_max
    # LiDAR must clear the backrest top so the chair does not occlude its own scan.
    assert spec.lidar_pos[2] > spec.z(spec.seat_height + spec.backrest_height)
    assert spec.front_camera_pos[0] < x_max
    # the planner footprint must cover everything mounted on the chair, including the rear camera
    assert x_min <= spec.rear_camera_pos[0]
    # LiDAR rays start above the puck far enough that the lowest (-45 deg) beam clears it (puck radius 4.5 cm)
    assert spec.lidar_above_mast >= 0.045
