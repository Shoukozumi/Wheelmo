"""Physics sanity check of the URDF in PyBullet (headless).

1. settle: drop on the floor, check it rests level on its four wheels only
2. push:   push the handle forward, check it rolls straight and the casters trail
3. turn:   push one side, check it turns and the casters swivel
4. tip:    press the handle down onto the anti-tip wheels, roll it forward tipped, release
5. armrest: lift the left armrest partway (falls back) and fully (stays up)

usage: python sim_check.py [frames.png]
"""
import math
import sys
from pathlib import Path

import numpy as np
import pybullet as pb
import pybullet_data
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import params as P

URDF = Path(__file__).resolve().parent / "out" / "staxi_wheelchair.urdf"
DT = 1 / 240


def main():
    pb.connect(pb.DIRECT)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    pb.setGravity(0, 0, -9.81)
    pb.setTimeStep(DT)
    plane = pb.loadURDF("plane.urdf")
    pb.changeDynamics(plane, -1, lateralFriction=1.0)
    # true cylinders for the wheels: by default PyBullet turns URDF cylinders into polygonal hulls,
    # which then rest on a flat facet and need a shove to start rolling
    chair = pb.loadURDF(str(URDF), [0, 0, 0.02], flags=pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_IMPLICIT_CYLINDER)
    joints = {pb.getJointInfo(chair, j)[1].decode(): j for j in range(pb.getNumJoints(chair))}
    links = {pb.getJointInfo(chair, j)[12].decode(): j for j in range(pb.getNumJoints(chair))}
    links["base_link"] = -1
    # never let the chair fall asleep: a sleeping body ignores applyExternalForce
    pb.changeDynamics(chair, -1, activationState=pb.ACTIVATION_STATE_DISABLE_SLEEPING)
    for name, j in joints.items():  # free joints: no motors
        pb.setJointMotorControl2(chair, j, pb.VELOCITY_CONTROL, force=0)
        if "wheel" in name:
            # rubber tyres: high grip
            pb.changeDynamics(chair, j, lateralFriction=1.0, rollingFriction=0.0005, spinningFriction=0.001)
        if "anti_tip" in name:
            # PyBullet caps joints at 100 rad/s, only 1.9 m/s for this 38 mm wheel: lift it so it rolls, not skids
            pb.changeDynamics(chair, j, maxJointVelocity=1000)
    total_mass = sum(pb.getDynamicsInfo(chair, l)[0] for l in [-1] + list(joints.values()))
    print(f"loaded {len(joints)} joints, total mass {total_mass:.2f} kg")

    frames = []

    def snap(title):
        pos, _ = base_link_pose()
        view = pb.computeViewMatrixFromYawPitchRoll([pos[0] + 0.15, pos[1], 0.45], 2.0, 35, -20, 0, 2)
        proj = pb.computeProjectionMatrixFOV(45, 4 / 3, 0.05, 10)
        img = pb.getCameraImage(480, 360, view, proj, renderer=pb.ER_TINY_RENDERER)[2]
        frames.append((title, np.reshape(img, (360, 480, 4))[:, :, :3]))

    # PyBullet reports the base pose at its centre of mass, with axes along the principal axes of
    # inertia. Undo that local inertial transform to get the URDF base_link frame.
    inertial_pos, inertial_orn = pb.getDynamicsInfo(chair, -1)[3:5]
    inv_pos, inv_orn = pb.invertTransform(inertial_pos, inertial_orn)

    def base_link_pose():
        p, o = pb.getBasePositionAndOrientation(chair)
        return pb.multiplyTransforms(p, o, inv_pos, inv_orn)

    def to_world(p_local):
        p, o = base_link_pose()
        return pb.multiplyTransforms(p, o, p_local, [0, 0, 0, 1])[0]

    def step(seconds, force=None, at=None, world=False):
        """Apply `force` (base_link frame, or world frame if `world`) at point `at` (base_link frame)
        every step."""
        for _ in range(int(seconds / DT)):
            if force is not None:
                _, o = base_link_pose()
                f_world = force if world else pb.multiplyTransforms([0, 0, 0], o, force, [0, 0, 0, 1])[0]
                pb.applyExternalForce(chair, -1, f_world, to_world(at), pb.WORLD_FRAME)
            pb.stepSimulation()

    def contacts():
        names = {v: k for k, v in links.items()}
        return sorted({names[c[3]] for c in pb.getContactPoints(chair, plane)})

    def state():
        pos, orn = base_link_pose()
        rpy = [math.degrees(a) for a in pb.getEulerFromQuaternion(orn)]
        return np.array(pos), rpy

    def jdeg(name):
        return math.degrees(pb.getJointState(chair, joints[name])[0])

    ok = True

    # 1. settle
    step(2.0)
    pos, rpy = state()
    c = contacts()
    print(f"[settle] base_link origin z={pos[2] * 1000:.1f} mm (expect ~0) roll={rpy[0]:.2f} pitch={rpy[1]:.2f} contacts={c}")
    expect = {"rear_wheel_l", "rear_wheel_r", "caster_wheel_l", "caster_wheel_r"}
    if set(c) != expect or abs(rpy[0]) > 1 or abs(rpy[1]) > 1:
        print("  FAIL: should rest level on exactly the four wheels"); ok = False
    snap("settled")

    # 2. push straight at walking pace (force on the push handle, forward), then coast
    handle = [-0.267, 0.0, 0.998]
    p0, _ = state()
    step(2.0, force=[22, 0, 0], at=handle)
    snap("pushing")
    v_release = np.linalg.norm(pb.getBaseVelocity(chair)[0])
    step(2.0)
    p1, rpy = state()
    v_end = np.linalg.norm(pb.getBaseVelocity(chair)[0])
    d = p1 - p0
    print(f"[push]   moved dx={d[0]:.2f} m dy={d[1]:.3f} m yaw={rpy[2]:.1f} deg; speed {v_release:.2f} m/s at release, "
          f"{v_end:.2f} m/s after 2 s coast (decel {(v_release - v_end) / 2:.2f} m/s^2); "
          f"casters {jdeg('caster_fork_l_joint'):.0f}/{jdeg('caster_fork_r_joint'):.0f} deg")
    if d[0] < 0.5 or abs(rpy[2]) > 10:
        print("  FAIL: expected >0.5 m of roughly straight travel"); ok = False
    snap("coasting")

    # 3. turn: push only the right end of the handle
    yaw0 = state()[1][2]
    step(1.5, force=[50, 0, 0], at=[-0.267, -0.25, 0.998])
    step(1.0)
    yaw1 = state()[1][2]
    print(f"[turn]   yaw change {yaw1 - yaw0:.1f} deg, casters {jdeg('caster_fork_l_joint'):.0f}/{jdeg('caster_fork_r_joint'):.0f} deg")
    if abs(yaw1 - yaw0) < 10:
        print("  FAIL: expected the chair to turn"); ok = False
    snap("after turn")

    # 4. tip back: press down on the push handle, slowly harder, until the chair lands on its anti-tip
    #    wheels; hold it there (lighter) and push it forward: it should keep rolling on the rear and
    #    anti-tip wheels, the anti-tip wheels rolling rather than skidding. Release: back on all four.
    def press(down, forward=0.0):
        _, o = base_link_pose()
        fwd = np.array(pb.multiplyTransforms([0, 0, 0], o, [1, 0, 0], [0, 0, 0, 1])[0])
        fwd[2] = 0
        f = forward * fwd / np.linalg.norm(fwd) + [0, 0, -down]
        pb.applyExternalForce(chair, -1, list(f), to_world(handle), pb.WORLD_FRAME)
        pb.stepSimulation()

    # start from standstill, as an attendant would: the turn test leaves the chair sliding sideways at
    # ~0.7 m/s, and landing on the anti-tip wheels while skidding made this test depend on that motion
    pb.resetBaseVelocity(chair, [0, 0, 0], [0, 0, 0])
    for j in range(pb.getNumJoints(chair)):
        pb.resetJointState(chair, j, pb.getJointState(chair, j)[0], 0.0)
    step(1.0)
    # controlled tilt, like an attendant: ramp the press until the chair starts to tip, then regulate it
    # so the chair tips back at ~15 deg/s (tipping is unstable: at a fixed force it accelerates and lands
    # hard enough to bounce over the anti-tip wheels)
    def tip_rate():  # deg/s, + = tipping back (nose up)
        _, o = base_link_pose()
        y_ax = np.array(pb.multiplyTransforms([0, 0, 0], o, [0, 1, 0], [0, 0, 0, 1])[0])
        return -math.degrees(np.dot(pb.getBaseVelocity(chair)[1], y_ax))
    t, f_tip = 0.0, None
    while not any(n.startswith("anti_tip") for n in contacts()) and t < 5:
        if f_tip is None and state()[1][1] < -0.5:
            f_tip = min(250, 40 + 30 * t)
        press(min(250, 40 + 30 * t) if f_tip is None else max(0.0, f_tip + 3.0 * (15.0 - tip_rate()))); t += DT
    for _ in range(int(1.0 / DT)):
        press(130)
    _, rpy = state()
    print(f"[tip]    pressed the handle down (starts to tip at {f_tip or 0:.0f} N, tilted at ~15 deg/s, held at 130 N): "
          f"pitch {rpy[1]:.1f} deg, contacts={contacts()}")
    snap("tipped back")
    p0, _ = state()
    spin0 = jdeg("anti_tip_wheel_l_joint")
    on_anti, on_rear, n = 0, 0, int(2.0 / DT)
    for _ in range(n):
        v = np.dot(pb.getBaseVelocity(chair)[0], pb.multiplyTransforms([0, 0, 0], base_link_pose()[1], [1, 0, 0], [0, 0, 0, 1])[0])
        press(130, forward=min(25.0, max(0.0, 60.0 * (1.2 - v))))  # push up to walking pace
        c = contacts()
        on_anti += any(k.startswith("anti_tip") for k in c)
        on_rear += any(k.startswith("rear") for k in c)
    p1, rpy = state()
    rolled = np.linalg.norm((p1 - p0)[:2])
    arc = math.radians(jdeg("anti_tip_wheel_l_joint") - spin0) * P.ANTI_TIP_WHEEL_D / 2000
    print(f"[tip]    pushed forward while tipped: rolled {rolled:.2f} m at pitch {rpy[1]:.1f} deg; on anti-tip "
          f"wheels {100 * on_anti / n:.0f}% / rear wheels {100 * on_rear / n:.0f}% of the time; anti-tip wheel "
          f"rolled {arc:.2f} m ({100 * arc / rolled:.0f}% rolling, rest is skid between bounces)")
    if rolled < 0.3 or on_anti < 0.7 * n or on_rear < 0.7 * n or arc / rolled < 0.7 or not -15 < rpy[1] < -5:
        print("  FAIL: should roll tipped back on the rear and anti-tip wheels"); ok = False
    snap("rolling tipped back")
    step(2.0)
    _, rpy = state()
    c = contacts()
    print(f"[tip]    released: pitch {rpy[1]:.1f} deg, contacts={c}")
    if set(c) != expect or abs(rpy[1]) > 1:
        print("  FAIL: should drop back onto its four wheels"); ok = False

    # 5. armrest: lift partway and release (should fall back), then flip fully (should stay up
    #    against its stop, past vertical, like the real flip-up armrest)
    j = joints["armrest_l_joint"]

    def lift(target_deg):
        pb.setJointMotorControl2(chair, j, pb.POSITION_CONTROL, targetPosition=math.radians(target_deg), force=40)
        step(1.5)
        up = jdeg("armrest_l_joint")
        pb.setJointMotorControl2(chair, j, pb.VELOCITY_CONTROL, force=0)
        step(2.0)
        return up, jdeg("armrest_l_joint")

    up, back = lift(50)
    print(f"[armrest] lifted to {up:.0f} deg, released -> {back:.0f} deg (expect falls back to ~0)")
    if back > 5:
        print("  FAIL: armrest should fall back"); ok = False
    up, back = lift(130)
    snap("armrest flipped up")
    print(f"[armrest] lifted to {up:.0f} deg (limit 110), released -> {back:.0f} deg (expect stays at stop)")
    if not (105 < up < 112) or back < 100:
        print("  FAIL: armrest should stop at ~110 and stay"); ok = False
    _, rpy = state()
    print(f"[final]  roll={rpy[0]:.2f} pitch={rpy[1]:.2f}  ->  {'ALL CHECKS PASSED' if ok else 'SOME CHECKS FAILED'}")

    if len(sys.argv) > 1:
        fig, axs = plt.subplots(1, len(frames), figsize=(5 * len(frames), 4))
        for ax, (t, im) in zip(axs, frames):
            ax.imshow(im); ax.set_title(t, fontsize=14); ax.axis("off")
        plt.tight_layout(); plt.savefig(sys.argv[1], dpi=70)
        print("wrote", sys.argv[1])
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
