"""Geometry of the procedural power wheelchair: detailed visuals plus hidden collision proxies.

Every body gets two groups:

* ``visual``: what viewers and cameras draw (procedural meshes and tube primitives).
* ``collision``: a few boxes/cylinders with ``purpose = guide`` that the physics and the LiDAR use. Each
  proxy sits just inside its visual so it never shows, even in camera sensors that draw colliders.

The dimensions come from :class:`~wheelmo_sim.wheelchair_spec.WheelchairSpec`. Frame (``base_link``):
origin at the drive-axle midpoint, +x forward, +y left, +z up, floor at ``z = -drive_wheel_radius``.
"""

from __future__ import annotations

import math

from . import usd_utils as U
from .meshes import disc_profile, revolve, rounded_box, rounded_rect_profile
from .wheelchair_spec import WheelchairSpec

# palette
FRAME = (0.70, 0.72, 0.75)  # brushed aluminium tubes
SHROUD = (0.13, 0.33, 0.62)  # battery/motor cover
UPHOLSTERY = (0.07, 0.07, 0.08)
DARK = (0.17, 0.17, 0.19)  # plastics, motors
TIRE = (0.04, 0.04, 0.04)
RIM = (0.80, 0.81, 0.83)
SENSOR = (0.12, 0.12, 0.13)
ACCENT = (0.95, 0.55, 0.10)
LENS = (0.02, 0.02, 0.05)

TUBE_R = 0.012  # frame tube radius


def author_wheelchair(stage, root: str, s: WheelchairSpec) -> None:
    """Author all links, geometry and joints under ``root`` (the articulation root prim)."""
    U.group(stage, f"{root}/joints")
    _base_link(stage, root, s)
    for side, sign in (("left", 1.0), ("right", -1.0)):
        _drive_wheel(stage, root, s, side, sign)
        _caster(stage, root, s, side, sign)


# ------------------------------------------------------------------------------------------------------
# base_link: frame, seating, legrests, sensors (chassis and occupant mass lumped together)
# ------------------------------------------------------------------------------------------------------


def _base_link(stage, root: str, s: WheelchairSpec) -> None:
    z = s.z
    base = f"{root}/base_link"
    chassis_com, occupant_com = (0.12, 0.0, z(0.25)), (0.08, 0.0, z(s.seat_height + 0.25))
    mass = s.chassis_mass + s.occupant_mass
    com = tuple((s.chassis_mass * a + s.occupant_mass * b) / mass for a, b in zip(chassis_com, occupant_com, strict=True))
    length, width, height = s.footrest_front - s.rear_extent, s.overall_width, s.seat_height + s.backrest_height
    inertia = (
        mass * (width**2 + height**2) / 12.0,
        mass * (length**2 + height**2) / 12.0,
        mass * (length**2 + width**2) / 12.0,
    )
    U.rigid_body(stage, base, (0.0, 0.0, 0.0), mass, com, inertia)
    v, c = f"{base}/visual", f"{base}/collision"
    U.group(stage, v)
    U.group(stage, c)

    # --- power base: battery/motor shroud between the drive wheels, motors, caster arms ----------------
    shroud_x = (-0.16, s.caster_wheelbase - s.caster_trail - s.caster_wheel_radius)  # stops before the casters
    shroud_w = s.drive_track - s.drive_wheel_width - 0.08
    shroud_c = (0.5 * sum(shroud_x), 0.0, z(0.20))
    U.mesh(stage, f"{v}/shroud", rounded_box((shroud_x[1] - shroud_x[0], shroud_w, 0.22), 0.045), SHROUD, shroud_c)
    U.box(stage, f"{v}/shroud_trim", (shroud_c[0], 0.0, z(0.10)), (shroud_x[1] - shroud_x[0] - 0.03, shroud_w + 0.004, 0.025), DARK)
    U.box(stage, f"{c}/chassis", shroud_c, (shroud_x[1] - shroud_x[0] - 0.06, shroud_w - 0.05, 0.19), SHROUD, proxy=True)
    motor_y = 0.5 * (0.5 * shroud_w + 0.5 * s.drive_track - 0.5 * s.drive_wheel_width)
    motor_len = 0.5 * s.drive_track - 0.5 * s.drive_wheel_width - 0.5 * shroud_w
    for side, sign in (("left", 1.0), ("right", -1.0)):
        U.cylinder(stage, f"{v}/motor_{side}", (0.0, sign * motor_y, 0.0), 0.055, motor_len, "Y", DARK)
        # caster frame arm and the vertical housing the fork stem turns in
        y_c = sign * 0.5 * s.caster_track
        swivel_h = s.caster_wheel_radius + s.caster_fork_height
        U.tube(stage, f"{v}/caster_arm_{side}", (shroud_x[1] - 0.04, sign * 0.17, z(swivel_h + 0.055)),
               (s.caster_wheelbase, y_c, z(swivel_h + 0.055)), 0.014, FRAME)
        U.cylinder(stage, f"{v}/caster_housing_{side}", (s.caster_wheelbase, y_c, z(swivel_h + 0.055)), 0.022, 0.06, "Z", FRAME)
        # anti-tip wheels behind the drive wheels, 2 cm off the floor
        tip = (s.rear_extent + 0.05, sign * 0.13)
        U.tube(stage, f"{v}/anti_tip_arm_{side}", (-0.14, tip[1], z(0.16)), (tip[0], tip[1], z(0.06)), TUBE_R, FRAME)
        U.cylinder(stage, f"{v}/anti_tip_wheel_{side}", (tip[0], tip[1], z(0.05)), 0.03, 0.022, "Y", TIRE)

    # --- seat: lift column, pan, cushion -----------------------------------------------------------------
    seat_x = (-0.19, -0.19 + s.seat_depth)
    seat_cx = 0.5 * sum(seat_x)
    pan_h = s.seat_height - 0.09  # underside of the cushion
    U.cylinder(stage, f"{v}/seat_column", (0.05, 0.0, z(0.5 * (0.31 + pan_h))), 0.04, pan_h - 0.31 + 0.01, "Z", DARK)
    U.box(stage, f"{v}/seat_pan", (seat_cx, 0.0, z(pan_h + 0.01)), (s.seat_depth, s.seat_width + 0.01, 0.02), FRAME)
    U.mesh(stage, f"{v}/seat_cushion", rounded_box((s.seat_depth, s.seat_width, 0.08), 0.03), UPHOLSTERY,
           (seat_cx, 0.0, z(s.seat_height - 0.04)))
    U.box(stage, f"{c}/seat", (seat_cx, 0.0, z(s.seat_height - 0.04)), (s.seat_depth - 0.04, s.seat_width - 0.04, 0.06),
          UPHOLSTERY, proxy=True)

    # --- backrest: canes with push handles, cross bar, cushion --------------------------------------------
    cane_x, cane_y = -0.255, 0.5 * s.seat_width - 0.015
    back_top = s.seat_height + s.backrest_height + 0.03
    for side, sign in (("left", 1.0), ("right", -1.0)):
        y = sign * cane_y
        U.tube(stage, f"{v}/back_cane_{side}", (cane_x, y, z(pan_h)), (cane_x, y, z(back_top)), TUBE_R, FRAME)
        U.tube(stage, f"{v}/push_handle_{side}", (cane_x, y, z(back_top)), (cane_x - 0.035, y, z(back_top)), TUBE_R, FRAME)
        U.tube(stage, f"{v}/grip_{side}", (cane_x - 0.015, y, z(back_top)), (cane_x - 0.045, y, z(back_top)), 0.017, UPHOLSTERY)
    U.tube(stage, f"{v}/back_cross_bar", (cane_x - 0.007, -cane_y, z(0.85)), (cane_x - 0.007, cane_y, z(0.85)), TUBE_R, FRAME)
    back_h = s.backrest_height - 0.02
    back_c = (-0.21, 0.0, z(s.seat_height + 0.01 + 0.5 * back_h))
    U.mesh(stage, f"{v}/back_cushion", rounded_box((0.07, 2 * cane_y - 0.01, back_h), 0.03), UPHOLSTERY, back_c)
    U.box(stage, f"{c}/backrest", back_c, (0.05, 2 * cane_y - 0.05, back_h - 0.04), UPHOLSTERY, proxy=True)

    # --- armrests, side guards, joystick -------------------------------------------------------------------
    arm_y, arm_h = 0.5 * s.seat_width + 0.035, 0.71
    for side, sign in (("left", 1.0), ("right", -1.0)):
        y = sign * arm_y
        for name, x in (("rear", -0.10), ("front", 0.12)):
            U.tube(stage, f"{v}/arm_support_{side}_{name}", (x, y, z(pan_h)), (x, y, z(arm_h - 0.02)), TUBE_R, FRAME)
        U.mesh(stage, f"{v}/arm_pad_{side}", rounded_box((0.36, 0.06, 0.04), 0.015), UPHOLSTERY, (0.03, y, z(arm_h)))
        U.box(stage, f"{v}/side_guard_{side}", (-0.04, sign * (0.5 * s.seat_width + 0.015), z(0.59)), (0.26, 0.006, 0.17), DARK)
    jy = -arm_y  # joystick on the right armrest
    U.mesh(stage, f"{v}/joystick_box", rounded_box((0.10, 0.07, 0.04), 0.012), DARK, (0.17, jy, z(arm_h + 0.045)))
    U.cylinder(stage, f"{v}/joystick_stem", (0.16, jy, z(arm_h + 0.08)), 0.005, 0.03, "Z", FRAME)
    U.sphere(stage, f"{v}/joystick_knob", (0.16, jy, z(arm_h + 0.105)), 0.016, UPHOLSTERY)

    # --- legrests: hangers and split footplates -------------------------------------------------------------
    foot_x = (s.footrest_front - 0.175, s.footrest_front - 0.005)
    foot_h = 0.207
    for side, sign in (("left", 1.0), ("right", -1.0)):
        U.tube(stage, f"{v}/leg_hanger_{side}", (seat_x[1] - 0.02, sign * 0.15, z(pan_h)),
               (foot_x[0], sign * 0.12, z(foot_h + 0.008)), TUBE_R, FRAME)
        plate_c = (0.5 * sum(foot_x), sign * 0.10, z(foot_h))
        U.mesh(stage, f"{v}/footplate_{side}", rounded_box((foot_x[1] - foot_x[0], 0.165, 0.014), 0.006), DARK, plate_c)
        U.box(stage, f"{c}/footplate_{side}", plate_c, (foot_x[1] - foot_x[0] - 0.01, 0.155, 0.01), DARK, proxy=True)

    # --- sensors: mast with LiDAR puck, rear camera, front camera on the joystick -------------------------
    # The LiDAR's optical centre is lidar_above_mast above the puck; keep it >= the puck radius so the
    # lowest (-45 deg) beam clears the housing (the ray caster hits visual geometry too).
    puck_r, puck_h = 0.045, 0.06
    assert s.lidar_above_mast >= puck_r, "LiDAR rays would hit their own housing"
    mast_bottom, mast_top = 0.27, s.mast_top_height - puck_h
    U.tube(stage, f"{v}/mast", (s.mast_x, 0.0, z(mast_bottom)), (s.mast_x, 0.0, z(mast_top)), 0.015, FRAME)
    U.tube(stage, f"{v}/mast_bracket", (shroud_x[0] + 0.02, 0.0, z(mast_bottom)), (s.mast_x, 0.0, z(mast_bottom)), 0.015, FRAME)
    U.tube(stage, f"{v}/mast_clamp", (cane_x - 0.007, 0.0, z(0.85)), (s.mast_x, 0.0, z(0.85)), 0.01, FRAME)
    U.cylinder(stage, f"{v}/mast_collar", (s.mast_x, 0.0, z(0.85)), 0.022, 0.03, "Z", DARK)
    U.mesh(stage, f"{v}/lidar_puck", revolve(disc_profile(puck_r, puck_h, 0.008)), SENSOR,
           (s.mast_x, 0.0, z(mast_top + 0.5 * puck_h)), U.Y_TO_Z)
    U.cylinder(stage, f"{v}/lidar_band", (s.mast_x, 0.0, z(mast_top + 0.035)), puck_r + 0.001, 0.008, "Z", ACCENT)
    for name, pos, facing in (("front", s.front_camera_pos, 1.0), ("rear", s.rear_camera_pos, -1.0)):
        # housing and lens sit *behind* the optical centre so they never appear in their own image
        x, y, zc = pos
        U.mesh(stage, f"{v}/{name}_camera_housing", rounded_box((0.04, 0.07, 0.035), 0.008), SENSOR,
               (x - facing * 0.025, y, zc))
        U.cylinder(stage, f"{v}/{name}_camera_lens", (x - facing * 0.006, y, zc), 0.012, 0.006, "X", LENS)
    fx, fy, fz = s.front_camera_pos
    U.box(stage, f"{v}/front_camera_bracket", (fx - 0.035, fy, 0.5 * (fz - 0.0175 + z(arm_h + 0.065))),
          (0.02, 0.03, fz - 0.0175 - z(arm_h + 0.065) + 0.004), DARK)


# ------------------------------------------------------------------------------------------------------
# wheels
# ------------------------------------------------------------------------------------------------------


def _cylinder_inertia(mass: float, radius: float, width: float):
    radial = mass * (3 * radius**2 + width**2) / 12.0
    return (radial, 0.5 * mass * radius**2, radial)


def _drive_wheel(stage, root: str, s: WheelchairSpec, side: str, sign: float) -> None:
    """14-inch mag wheel: pneumatic tyre, rim, five spokes, hub."""
    r, w = s.drive_wheel_radius, s.drive_wheel_width
    y = sign * 0.5 * s.drive_track
    wheel = f"{root}/{side}_wheel"
    U.rigid_body(stage, wheel, (0.0, y, 0.0), s.drive_wheel_mass, (0.0, 0.0, 0.0), _cylinder_inertia(s.drive_wheel_mass, r, w))
    v, c = f"{wheel}/visual", f"{wheel}/collision"
    U.group(stage, v)
    U.group(stage, c)
    corner = 0.015
    rim_r = r - 0.042
    U.mesh(stage, f"{v}/tire", revolve(rounded_rect_profile(rim_r, r, w, corner), 64), TIRE)
    U.mesh(stage, f"{v}/rim", revolve(rounded_rect_profile(rim_r - 0.022, rim_r + 0.002, 0.05, 0.004), 64), RIM)
    U.mesh(stage, f"{v}/hub", revolve(disc_profile(0.035, 0.06, 0.005), 32), RIM)
    spoke_len = rim_r - 0.022 - 0.03 + 0.01
    for k in range(5):
        a = 2.0 * math.pi * k / 5.0
        mid = 0.03 + 0.5 * spoke_len - 0.005
        U.box(stage, f"{v}/spoke_{k}", (mid * math.cos(a), 0.0, -mid * math.sin(a)), (spoke_len, 0.018, 0.022), RIM,
              quat=U.axis_angle_quat((0.0, 1.0, 0.0), a))
    # contact cylinder narrower than the tyre so its edges stay inside the flat part of the tread
    U.cylinder(stage, f"{c}/tire", (0.0, 0.0, 0.0), r, w - 2 * corner - 0.004, "Y", TIRE, proxy=True)
    U.revolute(stage, f"{root}/joints/{side}_drive_joint", f"{root}/base_link", wheel, "Y", (0.0, y, 0.0))


def _caster(stage, root: str, s: WheelchairSpec, side: str, sign: float) -> None:
    """8-inch caster: swivelling fork (straddles the wheel) and the wheel."""
    z = s.z
    rc, trail, fork_h = s.caster_wheel_radius, s.caster_trail, s.caster_fork_height
    y = sign * 0.5 * s.caster_track
    fork = f"{root}/{side}_caster_fork"
    swivel = (s.caster_wheelbase, y, z(rc + fork_h))
    U.rigid_body(stage, fork, swivel, s.caster_fork_mass, (-0.5 * trail, 0.0, -0.5 * fork_h), (1e-3, 1e-3, 1e-3))
    v = f"{fork}/visual"
    U.group(stage, v)
    U.mesh(stage, f"{v}/crown", rounded_box((trail + 0.05, 0.08, 0.022), 0.006), FRAME, (-0.5 * trail, 0.0, 0.011))
    for name, py in (("left", 0.032), ("right", -0.032)):
        U.box(stage, f"{v}/prong_{name}", (-trail, py, -0.5 * (fork_h + 0.012)), (0.024, 0.006, fork_h + 0.012), FRAME)
    U.cylinder(stage, f"{v}/stem", (0.0, 0.0, 0.035), 0.011, 0.03, "Z", FRAME)
    U.revolute(stage, f"{root}/joints/{side}_caster_swivel_joint", f"{root}/base_link", fork, "Z", swivel)

    cwheel = f"{root}/{side}_caster_wheel"
    width, corner = 0.04, 0.012
    U.rigid_body(stage, cwheel, (s.caster_wheelbase - trail, y, z(rc)), s.caster_wheel_mass, (0.0, 0.0, 0.0),
                 _cylinder_inertia(s.caster_wheel_mass, rc, width))
    v, c = f"{cwheel}/visual", f"{cwheel}/collision"
    U.group(stage, v)
    U.group(stage, c)
    U.mesh(stage, f"{v}/tire", revolve(rounded_rect_profile(0.6 * rc, rc, width, corner), 48), TIRE)
    U.mesh(stage, f"{v}/hub", revolve(disc_profile(0.6 * rc + 0.002, 0.032, 0.004), 32), RIM)
    U.cylinder(stage, f"{v}/axle", (0.0, 0.0, 0.0), 0.008, 0.074, "Y", FRAME)
    U.cylinder(stage, f"{c}/tire", (0.0, 0.0, 0.0), rc, width - 2 * corner - 0.004, "Y", TIRE, proxy=True)
    U.revolute(stage, f"{root}/joints/{side}_caster_wheel_joint", fork, cwheel, "Y", (-trail, 0.0, -fork_h))
