"""Every dimension of the Staxi Standard model, in one place.

Frame: X forward, Y left, Z up, millimetres. Origin on the floor midway between the rear
wheels, directly below the rear axle. Points are given for the chair's LEFT side (+Y);
the right side is the mirror image (Y -> -Y).

Source tags:  [scan] measured on the scaled scan   [spec] Staxi published spec
              [user] told by the user              [est]  estimate, please verify
"""
import math

# ---------------------------------------------------------------- materials / tubes
TUBE_MAIN = 25.4    # [est] 1" OD seat rails, front crossbar (scan reads 24-28 mm); base tube + posts are oval
TUBE_THIN = 15.9    # [est] 5/8" OD stacking basket, crossbars, footrest loops
TUBE_BRAKE = 19.05  # [est] 3/4" OD brake bar
TUBE_ARM = 22.0     # [est] armrest bar
PAD_ARM = 36.0      # [scan] armrest pad thickness
ROD_POCKET = 6.0    # [est] back-pocket wire

# ---------------------------------------------------------------- rear post + push handle
# One bent tube: left post -> top crossbar -> right post. Oval like the base tube: 24 across x 40 deep
# (fore-aft on the posts; sections at z 320-880 read 39-43 deep, 22-26 across), and the crossbar's
# section is the same oval carried round the bends (37 x 25, tilted ~15 deg).
POST_BOTTOM = (42.8, 255, 127)     # [scan] joins the base tube near the axle
PUSH_CORNER = (-261.2, 253, 1000)  # [scan] virtual corner where the post meets the top crossbar
POST_TUBE_W = 24.0                 # [scan] across
POST_TUBE_H = 40.0                 # [scan] deep (fore-aft)
PUSH_BEND_R = 60                   # [scan]

# ---------------------------------------------------------------- side frame
# Base tube: rear stacking extension -> under the axle -> forward to the front corner.
# The rear end stops short of the anti-tip wheel's fork (scan section narrows to tube size at x -200).
# It is an OVAL tube, 22 wide x 40 tall, not round like the posts: sections every 20 mm along both sides
# read 22-24 mm across and 39-44 mm tall (measured vertically) from x -160 to +550. Next to the wheel its
# centre line is z = 97.5 + 0.355 x (the rear axle bolt passes straight through it) at |y| 252-254.
BASE_PTS = [(-196, 254, 49), (-101, 254, 61.6), (50, 253, 115.3), (230, 222, 181),
            (300, 212, 193), (450, 183, 195), (638, 166, 203)]              # [scan]
BASE_BEND_R = 60
BASE_TUBE_W = 22.0   # [scan] oval section, across (horizontal)
BASE_TUBE_H = 40.0   # [scan] oval section, tall (in the vertical plane through the tube axis)
BASE_REAR_EXT = dict(x=-168.0, od=27.0)  # [scan] behind x -168 the tube is smaller (28-33 tall incl. noise at x -180, vs 40),
                                         # its bottom level with the anti-tip wheels and rising at the rounded end (z 43
                                         # at x -200): modelled round, 27 mm, with a round end
# Front downtube + seat rail: front corner -> up to the seat front -> back to the post.
SEAT_RAIL_PTS = [(638, 166, 203), (411, 187, 508), (40, 236, 494), (-76, 253, 455)]  # [scan]
SEAT_RAIL_BEND_R = 40
# Main crossbar between the base tubes (in front of the stacking basket).
CROSSBAR_FRONT = dict(x=453, z=193, half=183)                                # [scan]
# Crossbar between the rear posts, behind the backrest.
CROSSBAR_BACK = dict(x=-105, z=590, half=250)                                # [scan]
# Small bent tab welded on top of the base tube just in front of the pawl, curled backward at the
# top (|y| 243-263, up to z 148). Seen on both sides; purpose unknown (pawl stop or guard).
PAWL_TAB = dict(x=-21.0, y=(243.0, 263.0), z_top=148.0, t=4.0, curl=7.0)    # [scan]

# ---------------------------------------------------------------- stacking basket (thin rod)
# One continuous rod per side, from under the seat front, down the back, forward along the
# bottom, joined across the front by a U.
BASKET_PTS = [(150, 110, 495), (130, 110, 442), (-55, 110, 405), (-55, 110, 238),
              (200, 110, 229), (345, 110, 216), (450, 110, 197), (563, 110, 182)]  # [scan]
BASKET_BEND_R = 15
BASKET_CROSSBARS = [(-7, 236), (167, 228), (345, 216)]   # [scan] (x, z), span |y| <= 110

# ---------------------------------------------------------------- footrests (fixed)
# Each footrest is one closed loop of 5/8" tube: a flat rectangle with rounded ends, tilted front-edge-up.
# Its outer end (nearest the frame) is bent up into a standing U that leans outward and wraps around the
# outside of the front downtube, which comes down between the loop's long sides; the U is welded to it.
# Measured in the loop's own frame (u = fore-aft in the loop plane, v = |y|, w = loop normal) on both sides.
FOOTREST_TUBE_OD = 16.0                                   # [scan] circle fits r 7.5-8.7 along both long sides
FOOTREST_LOOP = dict(back=(657.9, 126.8), front=(712.9, 156.0))  # [scan] long-side centrelines (x, z): 62.3 apart,
                                                          # 28 deg front-up (mean of l/r; the right loop is 6 mm further forward)
FOOTREST_INNER_Y = 11.0                                   # [scan] |y| of the inner end's centreline (left 13, right 9.5)
FOOTREST_BEND = dict(y=148.0, r=12.0, deg=66.0, leg=22.7)  # [scan] long sides straight to |y| 148, then a tight bend
                                                          # (R 12) up through 66 deg, 23 mm straight legs, then the U (a
                                                          # semicircle the loop's width) whose top is 56 mm above the loop at |y| 181
# Front downtube (the seat rail's lower leg): an oval tube like the rear posts, 24 across x 40 deep, its axis 7 mm
# forward-up of the SEAT_RAIL_PTS line. It runs on past the base-tube corner down between the footrest's long
# sides and ends 95 mm below the corner (z ~ 127). Above it the seat rail (hidden under the cushion) stays round.
DOWNTUBE_W, DOWNTUBE_H = 24.0, 40.0                       # [scan] 23.3-24.4 x 37.3-40.5 along z 243-363, both sides
DOWNTUBE_SHIFT = 7.0                                      # [scan] forward-up of the SEAT_RAIL_PTS line
DOWNTUBE_BELOW_CORNER = 95.0                              # [scan] tube ends this far past (below) SEAT_RAIL_PTS[0]

# ---------------------------------------------------------------- seat + back cushion
# One moulded L-shaped cushion, profile at the centreline (x, z), extruded across Y.
# [scan] measured surface-by-surface on |y| < 150 (the backrest is flat across its width).
CUSHION_PROFILE = [
    # top cap
    (-208, 948), (-185, 960), (-160, 955),
    # backrest front face, top -> bottom (measured every 25 mm of height)
    (-155, 935), (-140, 910), (-130, 885), (-123, 860), (-114, 835), (-108, 810), (-101, 785),
    (-93, 760), (-81, 735), (-68, 710), (-61, 685), (-53, 660), (-40, 635), (-25, 610),
    (-18, 585), (-9, 565), (5, 552),
    # seat top, back -> front edge (measured every 20 mm)
    (40, 545), (100, 551), (200, 556), (300, 561), (340, 560), (380, 555), (400, 548),
    (420, 532), (436, 505), (440, 485),
    # seat underside, front -> back
    (430, 473), (400, 477), (380, 494), (340, 508), (300, 514), (240, 508), (200, 488),
    (120, 495), (40, 490), (0, 488),
    # backrest rear face, bottom -> top
    (-40, 505), (-80, 552), (-105, 600), (-118, 650), (-125, 700), (-138, 735), (-144, 760),
    (-149, 785), (-155, 810), (-163, 835), (-174, 860), (-180, 885), (-194, 910), (-206, 935),
]                                                                            # [scan]
CUSHION_HALF_WIDTH_BACK = 228   # [scan] seat 20" wide per [spec]
CUSHION_HALF_WIDTH_FRONT = 210  # [scan] seat narrows toward the front
SEAT_COLL_SLABS = [0, 120, 240, 380, 440]  # x edges of the seat's collision boxes (follow CUSHION_PROFILE)

# ---------------------------------------------------------------- brake bar (zip-tied squeezed)
BRAKE_HINGE = (-178, 662)            # [scan][user: hinge near the lower middle of the backrest] (x, z)
BRAKE_ARM_BOTTOM = (-182, 233, 672)  # [scan]
BRAKE_CORNER = (-361, 230, 946)      # [scan] virtual corner arm -> top bar
BRAKE_BEND_R = 60
BRAKE_SQUEEZED_GAP = 10.0            # [est] surface gap to the push handle when squeezed
# Back pocket wires, mounted on the brake bar: (x, z, half-span) at rest pose.
POCKET_RODS = [(-185, 650, 225), (-210, 690, 225), (-240, 725, 225), (-280, 775, 250)]  # [scan]
POCKET_UPRIGHTS_Y = [75, 200]                                                          # [est]

# ---------------------------------------------------------------- parking brake (one per side)
# Squeezing the brake bar lifts a push-rod on each side. A compression spring between a holder on the
# rear post and a collar on the rod pushes the rod back down when the bar is let go, and the pawl on the
# rod's lower end drops into a gap of the cog on the wheel's inner face. The rod runs parallel to the
# rear post, 21 mm behind it; the pawl's line of travel passes through the wheel axle, so it enters the
# cog radially. Values below are the LEFT side with the pawl engaged (bar released, as scanned);
# squeezing slides rod + collar + pawl BRAKE_LIFT up the rod axis and shortens the spring.
BRAKE_ROD_PT = (-67.5, 251.7, 360.0)  # [scan] a point on the rod axis (axis direction = rear post)
BRAKE_ROD_D = 8.0                     # [scan] 8-9
BRAKE_ROD_Z = (244.0, 648.0)          # [scan] lower end (in the collar) and upper end (turns inward)
BRAKE_ROD_HOOK_Y = 233.0              # [est] the top bends inward into the brake bar's lever; the linkage
                                      #       around the bar's hinge is not resolved in the scan
BRAKE_HOLDER_Z = (296.0, 306.0)       # [scan] eye welded to the back of the post by a short web
BRAKE_HOLDER_OD = 24.0                # [scan] eye reads 25 across from above
BRAKE_HOLDER_T = 5.0                  # [est] web thickness (y)
BRAKE_SPRING_OD = 19.0                # [scan] 18-20
BRAKE_SPRING_WIRE = 2.0               # [est]
BRAKE_SPRING_COILS = 9                # [est]
BRAKE_COLLAR = dict(z=(244.0, 250.0), od=21.0)  # [scan] spring seat clamped on the rod
# Pawl: a flat bar hung outboard of the collar. It runs straight down, then diagonally outward and down
# to its tip; the tip's end is a semicircle (seen along the axle) that seats in a cog trough.
BRAKE_PAWL_X = (-40.0, -18.0)         # [scan] blade, 22 wide
BRAKE_PAWL_Y = (261.0, 269.0)         # [scan] blade, 8 thick
BRAKE_PAWL_Z = (180.0, 244.0)         # [scan] vertical blade (a plate under the collar joins it to the rod);
                                      #        below z 180 the bar runs diagonally outward to the tip
BRAKE_PAWL_TIP_W = 9.0                # [scan] tip width across the trough (troughs are 10); rounded end R 4.5
BRAKE_PAWL_TIP_A = (-46.0, -34.0)     # [scan] tip span in wheel-axial a (|y| 272..284 at REAR_WHEEL_Y 318),
                                      #        stopping ~1 mm short of the wheel face behind the troughs
BRAKE_PAWL_TIP_LEN = 12.0             # [est] straight part of the tip above its rounded end, along the rod axis
BRAKE_PAWL_TIP_R = 51.5               # [scan] engaged: the rounded end sits 0.5 mm off the trough bottom
BRAKE_LIFT = 13.0                     # [est] rod travel when squeezed: 9 mm of trough depth + 4 mm clear
BRAKE_PAWL_SCAN_LIFT = {"l": 9.0, "r": 0.0}  # [scan] in the scan the left pawl rests on a crest

# ---------------------------------------------------------------- rear wheels
# 8" solid tyre on a moulded wheel centre, measured on both wheels in the wheel frame: a = axial from
# the tyre mid-plane (+ = outward, away from the chair), r = radius from the axle. The two wheels agree in
# shape to ~1 mm; the right one sits ~5 mm further out (|y| 321 vs 315), the model uses the mean.
REAR_WHEEL_X = 0.5          # [scan] axle x (left -1.2, right 2.3)
REAR_AXLE_Z = 97.5          # [scan] axle height. The tyres reach 3.5 mm below the scan's floor plane,
                            #        so the chair settles that much higher than the frame's z = 0.
REAR_WHEEL_Y = 318.0        # [scan] tyre mid-plane |y|
REAR_WHEEL_R = 101.0        # [scan] tread crown radius (8" nominal = 101.6 [spec])
REAR_TIRE_W = 55.0          # [scan] widest, at r 75 (the tyre bulges past the tread)
REAR_TREAD_W = 42.0         # contact band for the collision cylinder (crown drops 3.5 mm at a = +-12)
# Tyre section (a, r), outer half, crown -> shoulder -> widest -> rim pinch; mirrored for the inner half.
# Crown follows r = 101 - 0.025 a^2.
REAR_TIRE_PROFILE = [(0, 101.0), (6, 100.1), (12, 97.6), (18, 93.5), (22, 89.8), (25.5, 85.5),
                     (27.3, 80.0), (27.5, 75.0), (26.5, 69.0), (24.0, 62.0)]          # [scan]
REAR_RIM_R = 62.0           # [scan] tyre / wheel-centre junction (the pinch)
# Wheel-centre outer face (a, r) from the rim to the axle: slightly domed, nothing sticks out at the axle.
REAR_FACE_OUT = [(24.0, 62.0), (23.0, 55.0), (23.0, 48.0), (24.3, 38.0), (25.0, 28.0), (25.0, 0.0)]  # [scan]
# Inner side (a, r) from the axle out: axle-bolt recess, 49 mm hub boss (bears on the spacer), the dished
# disc face, the step up into the cog ring and its conical floor, back to the rim.
REAR_FACE_IN = [(-40.0, 0.0), (-40.0, 11.0), (-49.5, 12.0), (-49.5, 24.5), (-43.8, 26.5), (-43.4, 32.0),
                (-42.0, 37.0), (-40.5, 42.0), (-39.6, 47.0), (-39.6, 50.0), (-34.2, 50.0), (-25.9, 62.0),
                (-24.0, 62.0)]                                                     # [scan]
# Cog (parking-brake ring) on the inner face: a ring whose outer edge is a smooth wave of 20 rounded
# crests and 20 troughs, no sharp teeth. Each trough is a U, parallel sides and a semicircular bottom that
# the pawl's rounded tip seats in; the crests are rounded, so a tip that comes down on one slides off into
# a trough. Measured by folding all 20 troughs of both wheels onto one, seen along the axle.
COG_N = 20                  # [scan] 18 deg pitch: periodogram peak on both wheels
COG_CREST_R = 60.5          # [scan] crest tops
COG_TROUGH_R = 51.0         # [scan] trough bottoms: 9.5 mm deep
COG_TROUGH_W = 10.0         # [scan] trough width (semicircular bottom, R 5)
COG_CREST_FILLET = 3.5      # [scan] crest corner rounding: crests are 7.5 mm wide where the troughs'
                            # round bottoms end, 8.9 at the top, so they come out almost fully round
COG_INNER_R = 46.0          # ring's inner edge, blended into the dished disc
COG_FACE = (-40.5, -38.0)   # [scan] ring face a at r 50 and r 60 (a shallow cone)
COG_BACK_A = -25.5          # the ring runs back into the wheel body to here. A trough's back wall is the
                            # wheel face behind the ring (REAR_FACE_IN: a -34.2 at r 50 to -25.9 at r 62)
# Axle: a bolt through the oval base tube, a spacer, into the hub. Nothing shows on the outer face.
REAR_AXLE_D = 12.0          # [est]
REAR_AXLE_HEAD = (19.0, 7.0)  # [est] bolt head (diameter, length) against the tube's inner face
REAR_SPACER_OD = 20.0       # [est] between the tube's outer face (|y| 265) and the hub boss (|y| 268.5)

# ---------------------------------------------------------------- casters
# Stem-mounted swivel caster of the shopping-cart kind: a pressed-steel yoke (hood + two J-shaped
# legs) hangs from a bell-shaped swivel head that turns in a vertical housing on the frame; the wheel
# (tyre on a wide hub) runs on an axle bolt through the leg tips. Measured on both casters of the scan,
# which agree to within ~2 mm, after rotating each into its own frame (see README, "Front casters").
# Fork-frame coordinates: origin on the swivel axis at the top of the swivel head (the joint), +X =
# rolling direction (the wheel trails to -X), Z up.
CASTER_AXIS = (569.3, 173.6)  # [scan] swivel axis (x, |y|); l/r casters measured at |y| 175.6 / 171.7
CASTER_HOUSING_OD = 39.0      # [scan] vertical stem housing on the frame (40 incl. scan noise)
CASTER_HOUSING_Z = (166.0, 200.0)  # [scan] housing bottom .. top (the top is inside the base tube)
CASTER_WHEEL_R = 64.0         # [scan] tread radius, circle fit rms 1.3 mm (5" nominal = 63.5)
CASTER_TIRE_W = 36.0          # [scan] tyre width at the sidewalls
CASTER_TIRE_R_IN = 48.0       # [scan] where the tyre meets the wheel centre
CASTER_TIRE_CORNER = 7.0      # [scan] shoulder radius (flat tread ~22 mm)
CASTER_HUB_W = 48.0           # [scan] wheel centre is wider than the tyre (faces at +-24)
CASTER_TRAIL = 37.1           # [scan] axle behind the swivel axis (l 38.9, r 35.2)
CASTER_AXLE_DROP = 105.0      # [scan] swivel-head top down to the axle
# The scan's floor plane sits ~2-3 mm above the lowest point of every wheel (casters 3.0 mm, rear
# 1.3/2.3 mm), so the scan heights are a little low. The caster keeps its measured internal dimensions
# and its wheel touches the floor; its head top then sits 3 mm above the scanned housing bottom.
CASTER_TOP_Z = CASTER_WHEEL_R + CASTER_AXLE_DROP  # swivel joint height (169)
CASTER_YOKE_INNER = 24.5      # [scan] half the inside width between the legs (hub faces at 24)
CASTER_YOKE_T = 3.0           # [scan] leg / hood plate thickness
CASTER_HOOD_R = 29.5          # [scan] hood half-width = radius of its rounded front about the axis
CASTER_HOOD_BACK_X = -46.0    # [scan] hood's rear lip
CASTER_HOOD_Z = (-48.0, -19.0)  # [scan] hood bottom .. top
CASTER_HOOD_SLOPE = ((-46.0, -41.0), (-22.0, -8.0))  # [scan] (x, z) line the hood's back slopes along
CASTER_HEAD_PROFILE = [(29.5, -26.0), (29.0, -20.0), (27.0, -15.0), (25.0, -10.0), (23.0, -5.0), (21.0, 0.0)]  # [scan] (r, z) bell
# J-shaped leg plate outline (x, z): front edge bottom->top, then back edge top->bottom; the tip below
# the axle is a half circle joining the two ends.  [scan] 3rd/97th percentiles of both legs of both casters
CASTER_LEG_FRONT = [(-18.5, -110), (-18.0, -91), (-16.5, -79), (-12.0, -73), (-7.5, -67), (-3.0, -61), (3.0, -55),
                    (7.0, -49), (8.5, -43), (10.0, -37), (13.0, -31), (14.5, -25), (12.0, -19), (9.5, -13)]
CASTER_LEG_BACK = [(-17.0, -13), (-26.5, -19), (-31.0, -25), (-37.0, -31), (-40.0, -37), (-42.0, -43), (-45.0, -49),
                   (-46.5, -55), (-49.0, -61), (-51.5, -67), (-52.5, -79), (-52.5, -91), (-52.0, -103), (-52.0, -110)]
CASTER_HOOD_FRONT_CUT = ((5.0, -48.0), (29.5, -40.0))  # [scan] (x, z) line the hood's front-bottom edge rises along
CASTER_AXLE_D = 8.0           # [est] axle
CASTER_AXLE_HEAD = (11.0, 1.5)  # [est] rivet head (diameter, height): nothing sticks out past the legs in the scan
CASTER_TIRE_CLEAR = 3.0       # [est] radial clearance kept between the tyre and the hood

# ---------------------------------------------------------------- anti-tip wheels
# A small wheel in a fork behind the rear end of each base tube, on the tube's axis. It sits 36 mm
# off the floor, so it only touches down when the chair tips back (~9 deg) and lets it keep rolling.
ANTI_TIP_AXLE = (-217, 259, 55)  # [scan] (x, |y|, z); scan bulb centre x -214..-218, z 52..57
ANTI_TIP_WHEEL_D = 38.0          # [scan] bulb ~40 tall incl. scan noise (1.5")
ANTI_TIP_WHEEL_W = 16.0          # [est]
ANTI_TIP_CHEEK_GAP = 2.5         # [est] wheel face to fork cheek centre

# ---------------------------------------------------------------- armrests
ARM_HINGE = (-110, 252, 655)         # [scan] hinge axis along Y through this point
ARM_PTS = [(-110, 252, 655), (-30, 252, 652), (100, 290, 694), (330, 290, 770)]  # [scan] pose as scanned
ARM_BEND_R = 35
ARM_PAD_FROM = 2                     # pad covers ARM_PTS[ARM_PAD_FROM-1] .. tip
ARM_LIMIT_DEG = 110.0                # [user] flips up ~110 deg

# ---------------------------------------------------------------- masses (kg)
MASS_TOTAL = 57 * 0.4536             # [spec] 57 lb
MASS = dict(rear_wheel=1.2, caster_fork=0.25, caster_wheel=0.35, armrest=0.75, anti_tip_wheel=0.06)  # [est]
MASS["frame"] = MASS_TOTAL - 2 * sum(MASS.values())
MASS_CUSHION = 3.5                   # [est] moulded seat/back, part of the frame's mass


def mirror(p):
    """Left-side point -> right-side point."""
    return (p[0], -p[1], p[2]) if len(p) == 3 else p


def brake_squeeze_angle():
    """Rotation (rad, about +Y through BRAKE_HINGE) that brings the brake bar's top to
    BRAKE_SQUEEZED_GAP from the push-handle crossbar."""
    hx, hz = BRAKE_HINGE
    tx, tz = BRAKE_CORNER[0], BRAKE_CORNER[2]
    px, pz = PUSH_CORNER[0], PUSH_CORNER[2]
    target = BRAKE_SQUEEZED_GAP + (POST_TUBE_H + TUBE_BRAKE) / 2  # bar top meets the oval handle front-on
    vx, vz = tx - hx, tz - hz
    lo, hi = 0.0, math.radians(30)
    for _ in range(60):  # bisection on the gap
        a = (lo + hi) / 2
        x = hx + vx * math.cos(a) + vz * math.sin(a)
        z = hz - vx * math.sin(a) + vz * math.cos(a)
        if math.hypot(x - px, z - pz) > target:
            lo = a
        else:
            hi = a
    return (lo + hi) / 2
