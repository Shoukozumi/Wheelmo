"""Procedural layout of a tightly packed corridor (pure Python, no simulator needed).

The corridor runs along +x from ``x = 0`` to ``x = length`` and is centred on ``y = 0``. Obstacles are pushed
against the two walls with random sizes and spacing. Feasibility rule: any two obstacles on opposite walls that
are closer than ``pair_window`` along x must together leave a straight lane at least ``min_lane`` wide, so the
corridor is tight but never fully blocked. Doorways are wall stubs that narrow the corridor to
``doorway_width``.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field


@dataclass
class CorridorSpec:
    length: float = 20.0
    width: float = 1.6
    """Clear width wall-to-wall [m]."""
    wall_height: float = 2.4
    wall_thickness: float = 0.10
    min_lane: float = 0.90
    """Narrowest free lane that obstacles may leave [m]. The default chair is 0.63 m wide."""
    pair_window: float = 1.5
    """Opposite-wall obstacles closer than this along x must jointly leave ``min_lane``."""
    gap_range: tuple[float, float] = (0.3, 1.2)
    """Free x gap between consecutive obstacles on the same wall [m]."""
    wall_gap_range: tuple[float, float] = (0.02, 0.12)
    """Distance between an obstacle and the wall it stands against [m]."""
    start_clear: float = 2.5
    """Obstacle-free zone at the start of the corridor (the chair spawns at x = 1)."""
    end_clear: float = 2.0
    doorway_x: tuple[float, ...] = (10.0,)
    doorway_width: float = 0.90
    doorway_depth: float = 0.15
    doorway_clear: float = 1.2
    """Obstacle-free approach zone on each side of a doorway [m]."""
    seed: int = 0


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    color: tuple[float, float, float]

    def y_interval(self) -> tuple[float, float]:
        return (self.center[1] - 0.5 * self.size[1], self.center[1] + 0.5 * self.size[1])

    def x_interval(self) -> tuple[float, float]:
        return (self.center[0] - 0.5 * self.size[0], self.center[0] + 0.5 * self.size[0])


@dataclass(frozen=True)
class Cylinder:
    name: str
    center: tuple[float, float, float]
    radius: float
    height: float
    color: tuple[float, float, float]

    def y_interval(self) -> tuple[float, float]:
        return (self.center[1] - self.radius, self.center[1] + self.radius)

    def x_interval(self) -> tuple[float, float]:
        return (self.center[0] - self.radius, self.center[0] + self.radius)


Shape = Box | Cylinder


@dataclass(frozen=True)
class CorridorLayout:
    spec: CorridorSpec
    floor: Box
    """Floor slab; its top face is at z = 0."""
    walls: tuple[Box, ...]
    """Side walls, end walls and doorway stubs."""
    obstacles: tuple[Shape, ...]
    start_xy: tuple[float, float]
    goal_xy: tuple[float, float]
    shapes: tuple[Shape, ...] = field(init=False)
    """Walls followed by obstacles (everything standing on the floor)."""

    def __post_init__(self):
        object.__setattr__(self, "shapes", self.walls + self.obstacles)


# Indoor clutter catalogue: (name, kind, length along wall, depth from wall, height, colour).
# For cylinders, "length" and "depth" are both the diameter.
_CATALOGUE: tuple[tuple[str, str, float, float, float, tuple[float, float, float]], ...] = (
    ("cart", "box", 0.90, 0.50, 1.00, (0.30, 0.45, 0.70)),
    ("cabinet", "box", 0.60, 0.45, 1.80, (0.55, 0.40, 0.30)),
    ("chair", "box", 0.50, 0.50, 0.90, (0.20, 0.55, 0.35)),
    ("boxes", "box", 0.60, 0.40, 0.60, (0.75, 0.60, 0.35)),
    ("bench", "box", 1.40, 0.40, 0.45, (0.45, 0.45, 0.50)),
    ("bin", "cylinder", 0.44, 0.44, 0.75, (0.25, 0.25, 0.25)),
    ("plant", "cylinder", 0.50, 0.50, 1.20, (0.30, 0.60, 0.25)),
)


def generate_corridor_layout(spec: CorridorSpec | None = None) -> CorridorLayout:
    """Generate a deterministic corridor layout for ``spec.seed``."""
    spec = spec or CorridorSpec()
    if spec.min_lane >= spec.width:
        raise ValueError("min_lane must be smaller than the corridor width.")
    rng = random.Random(spec.seed)
    half_w = 0.5 * spec.width
    t, h = spec.wall_thickness, spec.wall_height
    wall_color = (0.86, 0.83, 0.76)

    walls: list[Box] = [
        Box("wall_left", (0.5 * spec.length, half_w + 0.5 * t, 0.5 * h), (spec.length + 2 * t, t, h), wall_color),
        Box("wall_right", (0.5 * spec.length, -half_w - 0.5 * t, 0.5 * h), (spec.length + 2 * t, t, h), wall_color),
        Box("wall_start", (-0.5 * t, 0.0, 0.5 * h), (t, spec.width, h), wall_color),
        Box("wall_end", (spec.length + 0.5 * t, 0.0, 0.5 * h), (t, spec.width, h), wall_color),
    ]
    door_color = (0.70, 0.66, 0.58)
    for i, xd in enumerate(spec.doorway_x):
        max_shift = 0.5 * (spec.width - spec.doorway_width) - 0.05
        yc = rng.uniform(-max_shift, max_shift) if max_shift > 0 else 0.0
        left_lo, right_hi = yc + 0.5 * spec.doorway_width, yc - 0.5 * spec.doorway_width
        walls.append(
            Box(f"door{i}_left", (xd, 0.5 * (left_lo + half_w), 0.5 * h), (spec.doorway_depth, half_w - left_lo, h), door_color)
        )
        walls.append(
            Box(f"door{i}_right", (xd, 0.5 * (right_hi - half_w), 0.5 * h), (spec.doorway_depth, right_hi + half_w, h), door_color)
        )

    def in_doorway_zone(x0: float, x1: float) -> float | None:
        """Return the end of the doorway exclusion zone that [x0, x1] overlaps, if any."""
        for xd in spec.doorway_x:
            if x1 > xd - spec.doorway_clear and x0 < xd + spec.doorway_clear:
                return xd + spec.doorway_clear
        return None

    max_protrusion = spec.width - spec.min_lane
    placed: list[tuple[float, float, int, float]] = []  # (x0, x1, side, protrusion) for the pair rule
    obstacles: list[Shape] = []
    # Each wall (+1 = left/+y, -1 = right/-y) is filled independently, always advancing the wall that lags.
    end_x = spec.length - spec.end_clear
    cursors = {side: spec.start_clear + rng.uniform(0.0, 0.8) for side in (1, -1)}
    while min(cursors.values()) < end_x:
        side = min(cursors, key=cursors.get)
        name, kind, length, depth, height, color = rng.choice(_CATALOGUE)
        scale = rng.uniform(0.85, 1.15)
        length, depth, height = length * scale, depth * scale, height * rng.uniform(0.9, 1.1)
        if kind == "cylinder":
            length = depth  # diameter
        x0 = cursors[side] + rng.uniform(*spec.gap_range)
        x1 = x0 + length
        if x1 > end_x:
            cursors[side] = end_x
            continue
        door_end = in_doorway_zone(x0, x1)
        if door_end is not None:
            cursors[side] = door_end
            continue
        wall_gap = rng.uniform(*spec.wall_gap_range)
        # Room left by nearby obstacles on the opposite wall.
        opposite = [p for (a, b, s, p) in placed if s == -side and x0 - b < spec.pair_window and a - x1 < spec.pair_window]
        allowed = max_protrusion - max(opposite, default=0.0)
        if kind == "cylinder":
            # A cylinder can't be squashed in one direction only; shrink its diameter to fit.
            depth = min(depth, allowed - wall_gap)
            length = depth
            x1 = x0 + length
        else:
            depth = min(depth, allowed - wall_gap)
        if depth < 0.15:
            cursors[side] = x0
            continue
        protrusion = wall_gap + depth
        y_inner = side * (half_w - protrusion)  # face pointing into the corridor
        y_center = y_inner + side * 0.5 * depth
        x_center = 0.5 * (x0 + x1)
        label = f"{name}_{len(obstacles):02d}"
        if kind == "cylinder":
            obstacles.append(Cylinder(label, (x_center, y_center, 0.5 * height), 0.5 * depth, height, color))
        else:
            obstacles.append(Box(label, (x_center, y_center, 0.5 * height), (length, depth, height), color))
        placed.append((x0, x1, side, protrusion))
        cursors[side] = x1

    floor_size = (spec.length + 2 * t + 1.0, spec.width + 2 * t + 1.0, 0.1)
    return CorridorLayout(
        spec=spec,
        floor=Box("floor", (0.5 * spec.length, 0.0, -0.05), floor_size, (0.62, 0.60, 0.56)),
        walls=tuple(walls),
        obstacles=tuple(obstacles),
        start_xy=(1.0, 0.0),
        goal_xy=(spec.length - 1.5, 0.0),  # the chair (1.1 m long) must fit before the end wall
    )


def free_interval_at(layout: CorridorLayout, x: float) -> tuple[float, float]:
    """Free y interval at station ``x``, treating every shape as its axis-aligned bounding box."""
    half_w = 0.5 * layout.spec.width
    lo, hi = -half_w, half_w
    for shape in layout.shapes:
        if shape.name in ("wall_left", "wall_right", "wall_start", "wall_end"):
            continue
        x0, x1 = shape.x_interval()
        if not x0 <= x <= x1:
            continue
        y0, y1 = shape.y_interval()
        if y0 + y1 > 0.0:
            hi = min(hi, y0)  # attached to the left (+y) wall
        else:
            lo = max(lo, y1)  # attached to the right (-y) wall
    return lo, hi
