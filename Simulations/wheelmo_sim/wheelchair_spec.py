"""Dimensions, masses and sensor mounts of the simulated wheelchair (pure Python).

The defaults describe a generic rear-wheel-drive power wheelchair. Replace them with measurements of the
real chair; every other module (USD asset, sensor mounts, controller) derives its numbers from here.

Frame convention (``base_link``): origin at the midpoint of the drive axle, +x forward, +y left, +z up.
The drive axle sits ``drive_wheel_radius`` above the floor, so a point at floor height ``h`` has
``z = h - drive_wheel_radius`` in ``base_link``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class WheelchairSpec:
    # Drive wheels (rear, motorised). Lengths in metres, masses in kilograms.
    drive_wheel_radius: float = 0.17
    drive_wheel_width: float = 0.07
    drive_track: float = 0.56
    """Distance between the drive-wheel centre planes."""
    drive_wheel_mass: float = 4.0
    drive_torque_limit: float = 80.0
    """Peak motor torque at each wheel [N*m]."""
    drive_speed_limit: float = 12.0
    """Peak wheel speed [rad/s] (about 2 m/s at the default radius)."""

    # Casters (front, passive swivel).
    caster_wheel_radius: float = 0.10
    caster_track: float = 0.50
    caster_wheelbase: float = 0.45
    """Drive axle to caster swivel axis, along x."""
    caster_trail: float = 0.05
    """Swivel axis to caster axle, horizontally (the wheel trails behind the swivel)."""
    caster_fork_height: float = 0.12
    """Caster axle to swivel joint, vertically."""
    caster_fork_mass: float = 0.5
    caster_wheel_mass: float = 0.6

    # Body. The occupant is modelled as mass only (no collision or visual geometry).
    chassis_mass: float = 75.0
    occupant_mass: float = 75.0
    seat_height: float = 0.50
    """Floor to seat surface."""
    seat_width: float = 0.46
    seat_depth: float = 0.46
    backrest_height: float = 0.50
    footrest_front: float = 0.78
    """Drive axle to the front edge of the footrest (frontmost point of the chair)."""

    # Sensor mounts, all expressed in base_link.
    mast_x: float = -0.30
    """x position of the vertical sensor mast behind the backrest."""
    mast_top_height: float = 1.45
    """Floor height of the mast top. The LiDAR's optical centre sits ``lidar_above_mast`` above it."""
    lidar_above_mast: float = 0.05
    front_camera_xy: tuple[float, float] = (0.26, -0.26)
    """Front of the right armrest."""
    front_camera_height: float = 0.80
    front_camera_pitch_deg: float = 15.0
    """Downward pitch of the forward-facing camera."""
    rear_camera_height: float = 1.10
    """Floor height of the rear-facing camera on the mast."""
    rear_camera_pitch_deg: float = 20.0

    # ---- derived quantities -------------------------------------------------------------------------

    def z(self, floor_height: float) -> float:
        """Convert a height above the floor into base_link z."""
        return floor_height - self.drive_wheel_radius

    @property
    def overall_width(self) -> float:
        return self.drive_track + self.drive_wheel_width

    @property
    def rear_extent(self) -> float:
        """Most negative x of the chair: drive-wheel rim, or the rear camera on the mast, whichever is further back."""
        return min(-self.drive_wheel_radius, self.mast_x - 0.035)

    @property
    def footprint(self) -> tuple[float, float, float, float]:
        """Axis-aligned footprint in base_link: (x_min, x_max, y_min, y_max)."""
        half = 0.5 * self.overall_width
        return (self.rear_extent, self.footrest_front, -half, half)

    @property
    def lidar_pos(self) -> tuple[float, float, float]:
        return (self.mast_x, 0.0, self.z(self.mast_top_height + self.lidar_above_mast))

    @property
    def front_camera_pos(self) -> tuple[float, float, float]:
        return (*self.front_camera_xy, self.z(self.front_camera_height))

    @property
    def rear_camera_pos(self) -> tuple[float, float, float]:
        return (self.mast_x - 0.03, 0.0, self.z(self.rear_camera_height))

    @property
    def total_mass(self) -> float:
        return (
            self.chassis_mass
            + self.occupant_mass
            + 2 * (self.drive_wheel_mass + self.caster_fork_mass + self.caster_wheel_mass)
        )

    def wheel_speeds(self, linear: float, angular: float) -> tuple[float, float]:
        """Differential-drive inverse kinematics: body (v [m/s], w [rad/s]) -> (left, right) wheel [rad/s]."""
        half_track = 0.5 * self.drive_track
        return (
            (linear - angular * half_track) / self.drive_wheel_radius,
            (linear + angular * half_track) / self.drive_wheel_radius,
        )


def yaw_pitch_quat_xyzw(yaw_deg: float, pitch_deg: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) for a yaw about +z followed by a pitch about the new +y.

    Positive pitch tilts +x downward, so ``yaw_pitch_quat_xyzw(0, 15)`` is a forward camera looking 15 deg down.
    """
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    # q = q_yaw * q_pitch, with q_yaw = (0, 0, sy, cy) and q_pitch = (0, sp, 0, cp)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)
