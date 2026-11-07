import pytest
import torch

from wheelmo_sim.controller import LocalPlanner, LocalPlannerCfg, ObstacleMemory
from wheelmo_sim.wheelchair_spec import WheelchairSpec

SPEC = WheelchairSpec()
FORWARD = torch.tensor([[1.0, 0.0]])


def _planner() -> LocalPlanner:
    # a large control_dt disables the acceleration limit so the chosen arc is returned directly
    return LocalPlanner(LocalPlannerCfg(), SPEC, num_envs=1, control_dt=10.0, device="cpu")


def _wall(x: float, y0: float, y1: float, step: float = 0.05) -> torch.Tensor:
    ys = torch.arange(y0, y1 + 1e-6, step)
    return torch.stack([torch.full_like(ys, x), ys], dim=-1)


def test_open_space_goes_full_speed_straight():
    v, w, ttc = _planner()(torch.zeros(1, 1, 2), torch.zeros(1, 1, dtype=torch.bool), FORWARD)
    assert v.item() == pytest.approx(max(LocalPlannerCfg().speeds))
    assert abs(w.item()) < 1e-6
    assert ttc.item() == pytest.approx(LocalPlannerCfg().horizon)


def test_blocked_lane_steers_toward_the_gap():
    # an obstacle 1.2 m ahead covering the right half of the corridor; free on the left (+y)
    points = torch.cat([_wall(1.2, -0.8, 0.05), _wall(-1.0, -0.8, 0.8)])  # plus a wall behind
    v, w, ttc = _planner()(points[None], torch.ones(1, len(points), dtype=torch.bool), FORWARD)
    assert w.item() > 0.0, "should turn left, toward the gap"
    assert ttc.item() >= LocalPlannerCfg().safe_time


def test_dead_end_never_drives_forward_into_it():
    points = _wall(SPEC.footrest_front + 0.08, -1.0, 1.0)  # wall just ahead of the footrest
    v, _, _ = _planner()(points[None], torch.ones(1, len(points), dtype=torch.bool), FORWARD)
    assert v.item() <= 0.0


def test_obstacle_memory_remembers_and_resets():
    memory = ObstacleMemory(2, (0.0, 4.0), (-1.0, 1.0), 0.1, "cpu")
    points = torch.tensor([[[1.03, 0.52], [3.0, -0.5]], [[0.5, 0.5], [9.0, 0.0]]])  # last one is out of bounds
    valid = torch.tensor([[True, False], [True, True]])
    memory.insert(points, valid)
    assert memory.occupied.sum(dim=1).tolist() == [1, 1]
    cells, ok = memory.nearest(torch.tensor([[1.0, 0.5], [0.0, 0.0]]), k=3)
    assert ok[0].sum() == 1 and torch.allclose(cells[0, 0], torch.tensor([1.05, 0.55]))
    memory.reset(torch.tensor([0]))
    assert memory.occupied.sum(dim=1).tolist() == [0, 1]
