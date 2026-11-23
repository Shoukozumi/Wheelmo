"""build123d model of the Staxi Standard wheelchair.

Each moving part is built in its own link frame (axes aligned with the model frame at joint
zero). `build()` returns a dict of links:
    name -> dict(shape=<build123d shape>, origin=(x,y,z) in parent frame, parent=..., joint=...,
                 collision=[primitives], mass=kg, paint=[(shape, material)] (optional colour split))
"""
import math

from build123d import (Axis, Box, Circle, Compound, Cylinder, Ellipse, FilletPolyline, Helix, Line, Location, Plane,
                       Polygon, Pos, Rectangle, Rot, SlotCenterToCenter, Sphere, Spline, ThreePointArc, Vector,
                       Kind, chamfer, extrude, fillet, loft, make_face, offset, Polyline, revolve, scale, sweep, Wire)

import params as P

# ---------------------------------------------------------------- helpers


def tube(points, od, bend_r=None, close=False, caps=False):
    """Swept round tube along a polyline with filleted bends. Returns (solid, path).
    caps: True for sphere caps on both ends, "end" for the last point only."""
    pts = [Vector(*p) for p in points]
    if len(pts) == 2:
        path = Line(pts[0], pts[1])
    else:
        path = FilletPolyline(*pts, radius=bend_r, close=close)
    solid = sweep((path ^ 0) * Circle(od / 2), path)
    parts = [solid]
    if caps and not close:
        parts += [Pos(*path.position_at(u)) * Sphere(od / 2) for u in ((1,) if caps == "end" else (0, 1))]
    return Compound(parts) if len(parts) > 1 else solid, path


def oval_tube(points, w, h, bend_r):
    """Swept oval tube along a polyline with filleted bends: w across (horizontal), h tall (in the
    vertical plane through the tube axis). Returns (solid, path)."""
    pts = [Vector(*p) for p in points]
    path = Line(*pts) if len(pts) == 2 else FilletPolyline(*pts, radius=bend_r)
    t0 = path % 0
    x_dir = Vector(0, 0, 1).cross(t0).normalized()
    return sweep(Plane(origin=path @ 0, x_dir=x_dir, z_dir=t0) * Ellipse(w / 2, h / 2), path), path


def base_y_at(x):
    """|y| of the base tube's centre line at x (straight-segment interpolation)."""
    pts = P.BASE_PTS
    for (x0, y0, _), (x1, y1, _) in zip(pts, pts[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise ValueError(x)


def footrest_path(s):
    """Centreline of one footrest tube (closed wire), side s. A flat loop in the tilted loop plane with a round
    inner end; at the outer end both long sides bend up through FOOTREST_BEND["deg"] into straight legs joined
    by a semicircle, so the U leans outward around the downtube."""
    (xb, zb), (xf, zf) = P.FOOTREST_LOOP["back"], P.FOOTREST_LOOP["front"]
    o = Vector((xb + xf) / 2, 0, (zb + zf) / 2)
    eu = Vector(xf - xb, 0, zf - zb)
    h = eu.length / 2
    eu = eu.normalized()
    ew, ev = Vector(-eu.Z, 0, eu.X), Vector(0, s, 0)        # loop normal (up), outward

    def L(u, v, w):
        return o + eu * u + ev * v + ew * w
    vi = P.FOOTREST_INNER_Y + h                             # centre of the round inner end
    bd = P.FOOTREST_BEND
    vb, rb, phi, leg = bd["y"], bd["r"], math.radians(bd["deg"]), bd["leg"]

    def bend(u, a):
        return L(u, vb + rb * math.sin(a), rb - rb * math.cos(a))
    dv, dw = math.cos(phi), math.sin(phi)
    cv, cw = vb + rb * math.sin(phi) + leg * dv, rb - rb * math.cos(phi) + leg * dw   # centre of the U
    return Wire([
        Line(L(-h, vi, 0), L(-h, vb, 0)),
        ThreePointArc(L(-h, vb, 0), bend(-h, phi / 2), bend(-h, phi)),
        Line(bend(-h, phi), L(-h, cv, cw)),
        ThreePointArc(L(-h, cv, cw), L(0, cv + h * dv, cw + h * dw), L(h, cv, cw)),
        Line(L(h, cv, cw), bend(h, phi)),
        ThreePointArc(bend(h, phi), bend(h, phi / 2), L(h, vb, 0)),
        Line(L(h, vb, 0), L(h, vi, 0)),
        ThreePointArc(L(h, vi, 0), L(0, vi - h, 0), L(-h, vi, 0)),
    ])


def capsule_segments(path, od, step=70.0):
    """Collision cylinders that follow a swept path."""
    n = max(1, math.ceil(path.length / step))
    pts = [path.position_at(i / n) for i in range(n + 1)]
    return [dict(type="cylinder", a=tuple(pts[i]), b=tuple(pts[i + 1]), r=od / 2) for i in range(n)]


def sym(p, s):
    """Point given for the left side, placed on side s (+1 left, -1 right)."""
    return (p[0], s * p[1], p[2])


def rot_y_about(p, pivot_xz, ang):
    """Rotate point p about the axis parallel to +Y through (pivot_x, *, pivot_z)."""
    x, y, z = p
    hx, hz = pivot_xz
    vx, vz = x - hx, z - hz
    return (hx + vx * math.cos(ang) + vz * math.sin(ang), y, hz - vx * math.sin(ang) + vz * math.cos(ang))


# ---------------------------------------------------------------- frame (base link)


def build_frame(brake_squeezed=True):
    parts, coll = [], []

    def add(points, od, bend_r=None, close=False, caps=False, collide=True):
        solid, path = tube(points, od, bend_r, close, caps)
        parts.append(solid)
        if collide:
            coll.extend(capsule_segments(path, od))
        return path

    # rear posts + push handle: one bent tube
    pb, pc = P.POST_BOTTOM, P.PUSH_CORNER
    solid, path = oval_tube([sym(pb, -1), sym(pc, -1), sym(pc, 1), sym(pb, 1)], P.POST_TUBE_W, P.POST_TUBE_H,
                            P.PUSH_BEND_R)
    parts.append(solid)
    coll.extend(capsule_segments(path, P.POST_TUBE_W + 8))

    for s in (1, -1):
        # oval base tube (collision: capsules); ellipsoid cap where it meets the front downtube
        solid, path = oval_tube([sym(p, s) for p in P.BASE_PTS], P.BASE_TUBE_W, P.BASE_TUBE_H, P.BASE_BEND_R)
        parts.append(solid)
        coll.extend(capsule_segments(path, P.BASE_TUBE_W + 8))
        parts.append(Pos(*sym(P.BASE_PTS[-1], s)) * scale(Sphere(1), by=(P.BASE_TUBE_W / 2, P.BASE_TUBE_W / 2,
                                                                        P.BASE_TUBE_H / 2)))
        # anti-tip wheel fork on the rear end of the base tube: two cheeks + axle bolt
        ax, ay, az = P.ANTI_TIP_AXLE
        off = P.ANTI_TIP_WHEEL_W / 2 + P.ANTI_TIP_CHEEK_GAP
        x0 = P.BASE_PTS[0][0] + 10  # cheeks start inside the tube end
        for k in (1, -1):
            yc = s * ay + k * off
            parts.append(Pos((x0 + ax) / 2, yc, az) * Box(x0 - ax, 3, 20))
            parts.append(Pos(ax, yc, az) * Rot(90, 0, 0) * Cylinder(10, 3))
            parts.append(Pos(ax, yc + k * 3, az) * Rot(90, 0, 0) * Cylinder(5.5, 3))  # bolt head / nut
        parts.append(Pos(ax, s * ay, az) * Rot(90, 0, 0) * Cylinder(3, 2 * off + 9))
        add([sym(p, s) for p in P.SEAT_RAIL_PTS], P.TUBE_MAIN, P.SEAT_RAIL_BEND_R)
        # front downtube: oval, from below the footrest up to where it bends into the (round) seat rail
        a, b = Vector(*sym(P.SEAT_RAIL_PTS[0], s)), Vector(*sym(P.SEAT_RAIL_PTS[1], s))
        ud = (a - b).normalized()                                  # down the tube
        e1 = ud.cross(Vector(0, 1, 0)).normalized()                # perpendicular, forward-up
        shift = e1 * P.DOWNTUBE_SHIFT
        solid, path = oval_tube([a + ud * P.DOWNTUBE_BELOW_CORNER + shift, b + ud * 21 + shift],
                                P.DOWNTUBE_W, P.DOWNTUBE_H, 1)
        parts.append(solid)
        coll.extend(capsule_segments(path, P.DOWNTUBE_W + 6))
        # footrest: one closed tube, the outer end bent up into a U welded round the downtube
        path = footrest_path(s)
        parts.append(sweep((path ^ 0) * Circle(P.FOOTREST_TUBE_OD / 2), path))
        coll.extend(capsule_segments(path, P.FOOTREST_TUBE_OD, step=25.0))
        # rear axle: bolt through the oval base tube (head inside), spacer, into the wheel's hub recess
        x, z = P.REAR_WHEEL_X, P.REAR_AXLE_Z
        yt = base_y_at(x)
        y_in, y_out = yt - P.BASE_TUBE_W / 2, yt + P.BASE_TUBE_W / 2
        y_boss, y_end = P.REAR_WHEEL_Y + P.REAR_FACE_IN[2][0], P.REAR_WHEEL_Y + P.REAR_FACE_IN[0][0]
        hd, hl = P.REAR_AXLE_HEAD

        def ycyl(r, ya, yb):  # cylinder along Y between |y| ya and yb on this side
            return Pos(x, s * (ya + yb) / 2, z) * Rot(90, 0, 0) * Cylinder(r, yb - ya)
        parts += [ycyl(P.REAR_AXLE_D / 2, y_in - hl, y_end - 1), ycyl(hd / 2, y_in - hl, y_in),
                  ycyl(P.REAR_SPACER_OD / 2, y_out - 1, y_boss)]
        # bent tab on top of the base tube in front of the pawl
        tb = P.PAWL_TAB
        tx, (ty0, ty1), ztop, t, curl = tb["x"], tb["y"], tb["z_top"], tb["t"], tb["curl"]
        zb = P.REAR_AXLE_Z + 0.355 * (tx - P.REAR_WHEEL_X)  # base tube centre line under the tab
        yc, dy = s * (ty0 + ty1) / 2, ty1 - ty0
        parts += [Pos(tx, yc, (zb + ztop) / 2) * Box(t, dy, ztop - zb),
                  Pos(tx - curl / 2, yc, ztop - t / 2) * Box(curl + t, dy, t),
                  Pos(tx - curl, yc, ztop - 6) * Box(t, dy, 12)]
        # caster stem housing: vertical tube under (and into) the base tube; the caster's head turns in it
        ax, ay = P.CASTER_AXIS
        hz0, hz1 = P.CASTER_HOUSING_Z
        parts.append(Pos(ax, s * ay, (hz0 + hz1) / 2) * Cylinder(P.CASTER_HOUSING_OD / 2, hz1 - hz0))

    # stacking basket: left rod -> front U -> right rod
    left = [sym(p, 1) for p in P.BASKET_PTS]
    add(left + [sym(p, -1) for p in reversed(P.BASKET_PTS)], P.TUBE_THIN, P.BASKET_BEND_R)
    for x, z in P.BASKET_CROSSBARS:
        add([(x, 110, z), (x, -110, z)], P.TUBE_THIN)
    cf, cb = P.CROSSBAR_FRONT, P.CROSSBAR_BACK
    add([(cf["x"], cf["half"], cf["z"]), (cf["x"], -cf["half"], cf["z"])], P.TUBE_MAIN)
    add([(cb["x"], cb["half"], cb["z"]), (cb["x"], -cb["half"], cb["z"])], P.TUBE_THIN)

    # seat + back cushion
    prof = Polyline(*[(x, 0, z) for x, z in P.CUSHION_PROFILE], close=True)
    cushion = extrude(make_face(prof), amount=P.CUSHION_HALF_WIDTH_BACK, both=True)
    xs = [p[0] for p in P.CUSHION_PROFILE]
    x0, x1 = min(xs) - 5, max(xs) + 5
    wb, wf = P.CUSHION_HALF_WIDTH_BACK, P.CUSHION_HALF_WIDTH_FRONT
    taper = extrude(make_face(Polyline((x0, -wb, 0), (60, -wb, 0), (x1, -wf, 0), (x1, wf, 0),
                                       (60, wb, 0), (x0, wb, 0), close=True)), amount=1100)
    cushion = cushion & taper
    try:
        cushion = fillet(cushion.edges(), radius=6)
    except Exception:  # noqa: BLE001 - cosmetic rounding only
        pass
    # cushion collision: seat box + back box (oriented along the backrest)
    coll.append(dict(type="box", center=(220, 0, 518), size=(440, 2 * wb, 75), rpy=(0, 0, 0)))
    bx0, bz0, bx1, bz1 = -45, 590, -180, 950  # backrest mid-surface line
    ang = math.atan2(bx0 - bx1, bz1 - bz0)
    coll.append(dict(type="box", center=((bx0 + bx1) / 2, 0, (bz0 + bz1) / 2),
                     size=(50, 2 * wb, math.hypot(bx1 - bx0, bz1 - bz0) + 30), rpy=(0, -ang, 0)))

    # brake bar + pocket, rotated into the squeezed pose (zip-tied)
    a = P.brake_squeeze_angle() if brake_squeezed else 0.0
    hp = P.BRAKE_HINGE
    R = lambda p: rot_y_about(p, hp, a)  # noqa: E731
    bb, bc = P.BRAKE_ARM_BOTTOM, P.BRAKE_CORNER
    add([R(sym(bb, -1)), R(sym(bc, -1)), R(sym(bc, 1)), R(sym(bb, 1))], P.TUBE_BRAKE, P.BRAKE_BEND_R)
    for x, z, half in P.POCKET_RODS:
        add([R((x, half, z)), R((x, -half, z))], P.ROD_POCKET, collide=False)
    (xb, zb, _), (xt, zt, _) = P.POCKET_RODS[0], P.POCKET_RODS[-1]
    for yy in P.POCKET_UPRIGHTS_Y:
        for s in (1, -1):
            add([R((xb, s * yy, zb)), R((xt, s * yy, zt))], P.ROD_POCKET, collide=False)
    for s, side in ((1, "l"), (-1, "r")):
        lift = P.BRAKE_LIFT if brake_squeezed else P.BRAKE_PAWL_SCAN_LIFT[side]
        parts += brake_parts(s, lift)
        parts.append(Pos(hp[0], s * 245, hp[1]) * Rot(90, 0, 0) * Cylinder(10, 30))  # hinge block

    return Compound(parts), cushion, coll


def brake_parts(s, lift):
    """Parking brake on side s: the holder on the rear post (fixed), the spring, and the push-rod with its
    collar and pawl slid `lift` mm up the rod axis (0 = pawl engaged in the cog)."""
    pb, pc = Vector(*sym(P.POST_BOTTOM, s)), Vector(*sym(P.PUSH_CORNER, s))
    u = (pc - pb).normalized()                   # rod axis, up (parallel to the rear post)
    n = Vector(u.Z, 0, -u.X).normalized()        # perpendicular to the rod in the x-z plane, toward the post
    p0 = Vector(*sym(P.BRAKE_ROD_PT, s))

    def at(z):  # point on the (unlifted) rod axis at height z
        return p0 + u * ((z - p0.Z) / u.Z)

    def along(z0, z1, lifted):  # frame centred on the rod between heights z0, z1 (x = n, z = u) and its length
        c = (at(z0) + at(z1)) * 0.5 + (u * lift if lifted else Vector())
        return Plane(origin=c, x_dir=n, z_dir=u), (z1 - z0) / u.Z
    up = u * lift
    out = []
    # rod: straight up beside the post, then bent inward at the top toward the brake bar's lever
    z0, z1 = P.BRAKE_ROD_Z
    top = at(z1)
    hook = Vector(top.X, s * P.BRAKE_ROD_HOOK_Y, top.Z)
    out.append(tube([at(z0) + up, top + up, hook + up], P.BRAKE_ROD_D, bend_r=10)[0])
    # holder: eye around the rod + web welded to the back of the post
    pl, L = along(*P.BRAKE_HOLDER_Z, lifted=False)
    out.append(pl * Cylinder(P.BRAKE_HOLDER_OD / 2, L))
    off = pb - p0
    off = off - u * off.dot(u)                    # rod axis -> post axis, perpendicular
    out.append(Plane(origin=pl.origin + off * 0.5, x_dir=n, z_dir=u) * Box(off.length, P.BRAKE_HOLDER_T, L))
    # collar (spring seat) on the rod
    cz = P.BRAKE_COLLAR["z"]
    pl, L = along(*cz, lifted=True)
    out.append(pl * Cylinder(P.BRAKE_COLLAR["od"] / 2, L))
    # compression spring from the collar up to the holder
    start = at(cz[1]) + up
    length = (P.BRAKE_HOLDER_Z[0] - cz[1]) / u.Z - lift
    helix = Helix(pitch=length / P.BRAKE_SPRING_COILS, height=length,
                  radius=(P.BRAKE_SPRING_OD - P.BRAKE_SPRING_WIRE) / 2, center=start, direction=u)
    out.append(sweep(Plane(origin=helix @ 0, z_dir=helix % 0) * Circle(P.BRAKE_SPRING_WIRE / 2), helix,
                     is_frenet=True))
    # pawl: plate under the collar, flat blade straight down outboard of the rod, then a diagonal section
    # outward and down to the tip. The tip travels along the rod axis on a line through the wheel axle, so it
    # enters a trough straight on; its end is a semicircle seen along the axle (a cylinder along Y).
    (x0, x1), (y0, y1), (zb0, zb1) = P.BRAKE_PAWL_X, P.BRAKE_PAWL_Y, P.BRAKE_PAWL_Z

    def box(xa, xb, ya, yb, za, zb):
        return Pos((xa + xb) / 2, s * (ya + yb) / 2, (za + zb) / 2) * Box(xb - xa, yb - ya, zb - za)
    pawl = [box(x0 + 2, x1 + 1, p0.Y * s - 7, y1, zb1 - 6, zb1),
            box(x0, x1, y0, y1, zb0, zb1)]
    axle = Vector(P.REAR_WHEEL_X, 0, P.REAR_AXLE_Z)
    ur = Vector(u.X, 0, u.Z).normalized()
    w, (ta0, ta1), L = P.BRAKE_PAWL_TIP_W, P.BRAKE_PAWL_TIP_A, P.BRAKE_PAWL_TIP_LEN
    c0 = axle + ur * (P.BRAKE_PAWL_TIP_R + w / 2)                  # centre of the rounded end
    c0 = Vector(c0.X, s * (P.REAR_WHEEL_Y + (ta0 + ta1) / 2), c0.Z)
    dy = ta1 - ta0
    pawl += [Plane(origin=c0 + ur * (L / 2), x_dir=n, z_dir=ur) * Box(w, dy, L),
             Pos(*c0) * Rot(90, 0, 0) * Cylinder(w / 2, dy)]
    bottom = Plane(origin=((x0 + x1) / 2, s * (y0 + y1) / 2, zb0 + 1), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
    pawl.append(loft([bottom * Rectangle(x1 - x0, y1 - y0),
                      Plane(origin=c0 + ur * L, x_dir=n, z_dir=ur) * Rectangle(w, dy)], ruled=True))
    out += [Pos(*up) * q for q in pawl]
    return out


# ---------------------------------------------------------------- wheels, casters, armrests


def stripes(shape, n):
    """Cut a wheel (axle along Y through the origin) into n equal sectors, alternately white and black,
    so you can see it turn. Returns [(sector, material), ...]."""
    R = 1000.0
    out = []
    for i in range(n):
        arc = [2 * math.pi * (i + k / 4) / n for k in range(5)]
        wedge = extrude(Plane.XZ * Polygon((0, 0), *[(R * math.cos(t), R * math.sin(t)) for t in arc], align=None),
                        amount=R, both=True)
        out.append((shape & wedge, "stripe_white" if i % 2 == 0 else "stripe_black"))
    return out


def pawl_angle():
    """Angle (deg about the axle, 0 = +X forward, 90 = up) of the pawl's line of travel, which passes
    through the axle. At joint zero a cog gap is centred there."""
    u = Vector(*P.PUSH_CORNER) - Vector(*P.POST_BOTTOM)
    return math.degrees(math.atan2(u.Z, u.X))


def build_cog_ring(out, xy):
    """The cog: a ring on the wheel's inner face whose outer edge is a smooth wave. It is a disc minus
    COG_N round-bottomed radial slots (the troughs, one centred on the pawl's line at joint zero), with the
    crest corners rounded off; extruded along the axle and given its shallow conical face."""
    n, rc, rt, w = P.COG_N, P.COG_CREST_R, P.COG_TROUGH_R, P.COG_TROUGH_W
    slot_len = rc + 8 - (rt + w / 2)                 # arc centre to arc centre, reaching past the crests
    sk = Circle(rc)
    for k in range(n):
        th = pawl_angle() + k * 360 / n
        c = rt + w / 2 + slot_len / 2
        sk -= Pos(c * math.cos(math.radians(th)), c * math.sin(math.radians(th))) * Rot(0, 0, th) * \
            SlotCenterToCenter(slot_len, w)
    # round the crests: shrink the outline by the rounding radius, then grow it back (rounds only the
    # convex corners; the troughs' round bottoms come back unchanged)
    f = P.COG_CREST_FILLET
    sk = make_face(sk.faces()[0].outer_wire().offset_2d(-f, kind=Kind.ARC).offset_2d(f, kind=Kind.ARC)) - \
        Circle(P.COG_INNER_R)
    a_face, a_back = min(P.COG_FACE) - 1.5, P.COG_BACK_A
    depth = a_back - a_face
    # sketch XY -> wheel plane XZ (angle from +X toward +Z), extruded toward -Y; shift onto [a_face, a_back]
    ring = Pos(0, max(out * a_back, out * a_face), 0) * Rot(90, 0, 0) * extrude(sk, depth)
    (f0, f1), r0, r1 = P.COG_FACE, 50.0, 60.0
    face_a = lambda r: f0 + (f1 - f0) * (r - r0) / (r1 - r0)  # noqa: E731
    keep = revolve(make_face(Polyline(xy(face_a(40), 40), xy(face_a(70), 70), xy(a_back + 5, 70),
                                      xy(a_back + 5, 40), close=True)), Axis.Y)
    return ring & keep


def build_rear_wheel(inner_sign):
    """Rear wheel in its own frame (origin at the axle centre on the tyre mid-plane, axle along Y):
    revolved tyre + wheel centre, and the wave-edged cog ring on the inner face.
    inner_sign: +1 if the chair centre is toward +Y (right wheel), -1 for the left wheel."""
    out = -inner_sign

    def xy(a, r):  # profile plane: x = radius, y = axle (a measured outward)
        return (r, out * a, 0)
    tp = P.REAR_TIRE_PROFILE
    sec = list(reversed(tp)) + [(-a, r) for a, r in tp[1:]]          # rim (outer) -> crown -> rim (inner)
    tire = revolve(make_face([Spline(*[xy(a, r) for a, r in sec]), Line(xy(*sec[-1]), xy(*sec[0]))]), Axis.Y)
    centre = revolve(make_face(Polyline(*[xy(a, r) for a, r in P.REAR_FACE_IN + P.REAR_FACE_OUT], close=True)),
                     Axis.Y)
    ring = build_cog_ring(out, xy)
    try:
        centre = centre.fuse(ring).clean()
    except Exception:  # noqa: BLE001 - fall back to overlapping solids (fine for meshes)
        centre = Compound([centre, ring])
    R, W = P.REAR_WHEEL_R, P.REAR_TREAD_W
    coll = [dict(type="cylinder", a=(0, -W / 2, 0), b=(0, W / 2, 0), r=R)]
    paint = stripes(tire, 8) + [(centre, "plastic")]
    return Compound([tire, centre]), coll, paint


def build_caster_fork():
    """Swivel caster yoke in its joint frame: origin on the swivel axis at the top of the swivel head,
    +X = rolling direction (the wheel trails to -X), Z up. A pressed-steel hood and two J-shaped legs
    hang from a bell-shaped swivel head; the axle runs through the leg tips.
    Returns (shape, collision, wheel axle position in this frame)."""
    tr, drop, t = P.CASTER_TRAIL, P.CASTER_AXLE_DROP, P.CASTER_YOKE_T
    yi, hr, xb = P.CASTER_YOKE_INNER, P.CASTER_HOOD_R, P.CASTER_HOOD_BACK_X
    z0, z1 = P.CASTER_HOOD_Z
    axle = (-tr, 0.0, -drop)

    # hood: D-shaped in plan (front rounded about the swivel axis), rear corners rounded, back sloped,
    # hollowed to a plate shell open underneath, front-bottom edge raised, and cleared around the tyre
    plan = (Line((xb, -hr), (0, -hr)) + ThreePointArc((0, -hr), (hr, 0), (0, hr))
            + Line((0, hr), (xb, hr)) + Line((xb, hr), (xb, -hr)))
    hood = extrude(Pos(0, 0, z0) * make_face(plan), amount=z1 - z0)
    hood = fillet(hood.edges().filter_by(Axis.Z).group_by(Axis.X)[0], radius=8)
    (sx0, sz0), (sx1, sz1) = P.CASTER_HOOD_SLOPE
    tilt = math.degrees(math.atan2(sx1 - sx0, sz1 - sz0))  # the back face leans back by this much
    hood -= Location((sx0, 0, sz0)) * Rot(0, -tilt, 0) * Pos(0, 0, 200) * Box(400, 400, 400)
    hood = offset(hood, amount=-t, openings=hood.faces().sort_by(Axis.Z)[0])
    (fx0, fz0), (fx1, fz1) = P.CASTER_HOOD_FRONT_CUT
    rise = math.degrees(math.atan2(fz1 - fz0, fx1 - fx0))
    hood -= Location((fx0, 0, fz0)) * Rot(0, -rise, 0) * Pos(200, 0, -200) * Box(400, 400, 400)
    hood -= Pos(*axle) * Rot(90, 0, 0) * Cylinder(P.CASTER_WHEEL_R + P.CASTER_TIRE_CLEAR, 2 * (yi - 0.5))

    # swivel head: a bell revolved about the swivel axis; it turns inside the frame's housing
    prof = P.CASTER_HEAD_PROFILE
    zb, (rt, zt) = prof[0][1], prof[-1]
    outline = (Line((0, 0, zb), (prof[0][0], 0, zb)) + Spline(*[(r, 0, z) for r, z in prof])
               + Line((rt, 0, zt), (0, 0, zt)) + Line((0, 0, zt), (0, 0, zb)))
    head = revolve(make_face(outline), Axis.Z)

    # legs: J-shaped plates from under the head down to a rounded tip around the axle
    front, back = P.CASTER_LEG_FRONT, P.CASTER_LEG_BACK
    tip_x, tip_z, tip_r = (front[0][0] + back[-1][0]) / 2, front[0][1], (front[0][0] - back[-1][0]) / 2
    edge = Polyline(*[(x, 0, z) for x, z in front + back])
    tip = ThreePointArc((back[-1][0], 0, back[-1][1]), (tip_x, 0, tip_z - tip_r), (front[0][0], 0, front[0][1]))
    leg = extrude(make_face(edge + tip), amount=t / 2, both=True)
    legs = [Pos(0, s * (yi + t / 2), 0) * leg for s in (1, -1)]

    # axle: flush with the outside of the legs, thin rivet heads (nothing sticks out in the scan)
    y_out = yi + t
    hd, hh = P.CASTER_AXLE_HEAD
    shaft = Pos(*axle) * Rot(90, 0, 0) * Cylinder(P.CASTER_AXLE_D / 2, 2 * y_out)
    heads = [Pos(axle[0], s * (y_out + hh / 2), axle[2]) * Rot(90, 0, 0) * Cylinder(hd / 2, hh) for s in (1, -1)]

    shape = Compound([hood, head, shaft] + heads + legs)
    coll = [dict(type="box", center=((xb + hr) / 2, 0, (z0 + z1) / 2), size=(hr - xb, 2 * hr, z1 - z0), rpy=(0, 0, 0))]
    return shape, coll, axle


def build_caster_wheel():
    """Caster wheel in its own frame (origin at the axle centre, axle along Y): a tyre with rounded
    shoulders on a wheel centre (hub) that is wider than the tyre."""
    R, ri, W, hw = P.CASTER_WHEEL_R, P.CASTER_TIRE_R_IN, P.CASTER_TIRE_W, P.CASTER_HUB_W
    base, side, tread = hw / 2 - 2.0, W / 2, W / 2 - 2.0  # tyre half-widths: at the hub, sidewall, tread
    section = make_face(Polyline((ri - 0.5, -base), (ri + 9, -side), (R, -tread), (R, tread), (ri + 9, side),
                                 (ri - 0.5, base), close=True))
    section = fillet(section.vertices().filter_by_position(Axis.X, R - 0.1, R + 0.1), radius=P.CASTER_TIRE_CORNER)
    tire = revolve(Plane.XY * section, Axis.Y)
    hub = Rot(90, 0, 0) * Cylinder(ri, hw)
    hub = chamfer(hub.edges(), length=1.5)
    boss = Rot(90, 0, 0) * Cylinder(11, hw + 1)  # bearing boss / spacer against the leg plates
    # contact: the flat of the tread (the shoulders never touch a level floor)
    coll = [dict(type="cylinder", a=(0, -tread, 0), b=(0, tread, 0), r=R)]
    return Compound([tire, hub, boss]), coll, stripes(tire, 8) + [(Compound([hub, boss]), "plastic")]


def build_anti_tip_wheel():
    R, W = P.ANTI_TIP_WHEEL_D / 2, P.ANTI_TIP_WHEEL_W
    ax = Rot(90, 0, 0)
    tire = fillet((ax * Cylinder(R, W)).edges(), radius=4)
    hub = ax * Cylinder(6, 2 * P.ANTI_TIP_CHEEK_GAP + W - 3)  # spacer out to the fork cheeks
    coll = [dict(type="cylinder", a=(0, -W / 2, 0), b=(0, W / 2, 0), r=R)]
    return Compound([tire, hub]), coll, stripes(tire, 6) + [(hub, "metal")]


def build_armrest(s):
    """Armrest in its frame: origin at the hinge, geometry in the scanned pose."""
    h = sym(P.ARM_HINGE, s)
    pts = [tuple(a - b for a, b in zip(sym(p, s), h)) for p in P.ARM_PTS]
    bar, bar_path = tube(pts, P.TUBE_ARM, P.ARM_BEND_R)
    pad_pts = pts[P.ARM_PAD_FROM - 1:]
    a, b = Vector(*pad_pts[-2]), Vector(*pad_pts[-1])
    a = a + (b - a) * 0.15  # pad starts a little after the bend
    pad, pad_path = tube([tuple(a), tuple(b)], P.PAD_ARM, caps=True)
    knuckle = Rot(90, 0, 0) * Cylinder(14, 30)
    coll = capsule_segments(pad_path, P.PAD_ARM, step=400) + capsule_segments(bar_path, P.TUBE_ARM, step=120)
    return Compound([bar, pad, knuckle]), coll


# ---------------------------------------------------------------- assembly


def build(brake_squeezed=True):
    links = {}
    metal, cushion, coll = build_frame(brake_squeezed)
    m_cushion = P.MASS_CUSHION
    links["base_link"] = dict(shape=Compound([metal, cushion]), collision=coll, mass=P.MASS["frame"],
                              mass_parts=[(metal, P.MASS["frame"] - m_cushion, "shell"), (cushion, m_cushion)],
                              parent=None)
    for s, side in ((1, "l"), (-1, "r")):
        # wheel bearings: very low friction (only a trace of damping)
        shape, coll, paint = build_rear_wheel(inner_sign=-s)
        links[f"rear_wheel_{side}"] = dict(
            shape=shape, collision=coll, paint=paint, mass=P.MASS["rear_wheel"], parent="base_link",
            joint=dict(type="continuous", origin=(P.REAR_WHEEL_X, s * P.REAR_WHEEL_Y, P.REAR_AXLE_Z),
                       axis=(0, 1, 0), damping=1e-4, friction=0.0))
        fork, coll, wheel_at = build_caster_fork()
        ax, ay = P.CASTER_AXIS
        links[f"caster_fork_{side}"] = dict(
            shape=fork, collision=coll, mass=P.MASS["caster_fork"], parent="base_link",
            joint=dict(type="continuous", origin=(ax, s * ay, P.CASTER_TOP_Z), axis=(0, 0, 1),
                       damping=0.002, friction=0.0))
        shape, coll, paint = build_caster_wheel()
        links[f"caster_wheel_{side}"] = dict(
            shape=shape, collision=coll, paint=paint, mass=P.MASS["caster_wheel"], parent=f"caster_fork_{side}",
            joint=dict(type="continuous", origin=wheel_at, axis=(0, 1, 0), damping=1e-4, friction=0.0))
        shape, coll, paint = build_anti_tip_wheel()
        ax, ay, az = P.ANTI_TIP_AXLE
        links[f"anti_tip_wheel_{side}"] = dict(
            shape=shape, collision=coll, paint=paint, mass=P.MASS["anti_tip_wheel"], parent="base_link",
            # a tiny wheel (I ~1e-5 kg m^2): any more damping stops it between bounces and it skids
            joint=dict(type="continuous", origin=(ax, s * ay, az), axis=(0, 1, 0), damping=1e-5, friction=0.0))
        shape, coll = build_armrest(s)
        links[f"armrest_{side}"] = dict(
            shape=shape, collision=coll, mass=P.MASS["armrest"], parent="base_link",
            joint=dict(type="revolute", origin=sym(P.ARM_HINGE, s), axis=(0, -1, 0),
                       lower=0.0, upper=math.radians(P.ARM_LIMIT_DEG), damping=0.5, friction=0.2,
                       effort=50.0, velocity=3.0))
    return links


def link_world_transforms(links, q=None):
    """World Location of every link for joint values q (dict joint-name -> rad)."""
    q = q or {}
    out = {}

    def world(name):
        if name in out:
            return out[name]
        L = links[name]
        if L["parent"] is None:
            out[name] = Location()
            return out[name]
        j = L["joint"]
        ang = math.degrees(q.get(name, 0.0))
        rot = Rot(*[ang * c for c in j["axis"]])  # joint axes are unit X, Y or Z (or their negatives)
        out[name] = world(L["parent"]) * Location(j["origin"]) * rot
        return out[name]

    for n in links:
        world(n)
    return out


def assembly(links, q=None):
    T = link_world_transforms(links, q)
    return Compound([T[n] * L["shape"] for n, L in links.items()])
