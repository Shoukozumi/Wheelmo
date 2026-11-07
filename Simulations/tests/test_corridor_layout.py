import pytest

from wheelmo_sim.corridor_layout import Box, CorridorSpec, free_interval_at, generate_corridor_layout

SEEDS = range(10)


def _stations(spec: CorridorSpec, step: float = 0.02):
    return [i * step for i in range(int(spec.length / step) + 1)]


def test_layout_is_deterministic_per_seed():
    a = generate_corridor_layout(CorridorSpec(seed=3))
    b = generate_corridor_layout(CorridorSpec(seed=3))
    c = generate_corridor_layout(CorridorSpec(seed=4))
    assert a == b
    assert a.obstacles != c.obstacles


@pytest.mark.parametrize("seed", SEEDS)
def test_free_lane_never_narrower_than_min_lane(seed):
    spec = CorridorSpec(seed=seed)
    layout = generate_corridor_layout(spec)
    for x in _stations(spec):
        lo, hi = free_interval_at(layout, x)
        if any(abs(x - xd) <= 0.5 * spec.doorway_depth for xd in spec.doorway_x):
            assert hi - lo == pytest.approx(spec.doorway_width)
        else:
            assert hi - lo >= spec.min_lane - 1e-9, f"seed {seed}: lane at x={x:.2f} is {hi - lo:.3f} m"


@pytest.mark.parametrize("seed", SEEDS)
def test_start_goal_and_doorway_zones_are_clear(seed):
    spec = CorridorSpec(seed=seed)
    layout = generate_corridor_layout(spec)
    assert len(layout.obstacles) >= 6, "corridor should be tightly packed"
    for shape in layout.obstacles:
        x0, x1 = shape.x_interval()
        assert x0 >= spec.start_clear
        assert x1 <= spec.length - spec.end_clear
        for xd in spec.doorway_x:
            assert x1 <= xd - spec.doorway_clear or x0 >= xd + spec.doorway_clear


def test_floor_top_is_at_zero_and_covers_corridor():
    spec = CorridorSpec()
    floor = generate_corridor_layout(spec).floor
    assert isinstance(floor, Box)
    assert floor.center[2] + 0.5 * floor.size[2] == pytest.approx(0.0)
    assert floor.x_interval()[0] < 0.0 and floor.x_interval()[1] > spec.length
