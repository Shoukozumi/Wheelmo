"""Baseline autonomy: LiDAR obstacle memory + footprint-aware dynamic-window local planner.

This is a placeholder for the real autonomy stack. It shows the full sensor -> command loop:

1. :class:`ObstacleMemory` accumulates LiDAR returns into a 2D occupancy grid. A mast-mounted LiDAR loses
   sight of low obstacles (benches, bins) once the chair is within ~1 m of them, so obstacles seen from
   further away must be remembered. This is the same reason Nav2-style stacks keep a costmap.
2. :class:`LocalPlanner` rolls out a fan of constant (v, w) arcs, checks the chair's rectangular footprint
   against the remembered obstacles along each arc, and picks the safe arc that best trades progress along
   the goal direction against clearance (a dynamic-window approach). Reversing arcs are included so the
   chair can back out of tight spots.

Everything is batched over environments with torch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import torch

from .wheelchair_spec import WheelchairSpec


class ObstacleMemory:
    """Persistent 2D occupancy grid in each environment's frame.

    Static-world assumption: cells are only cleared on :meth:`reset`. Moving obstacles would need free-space
    clearing (ray tracing) or a decay time.
    """

    def __init__(
        self,
        num_envs: int,
        x_range: tuple[float, float],
        y_range: tuple[float, float],
        resolution: float,
        device: str | torch.device,
    ):
        self.res = resolution
        self.origin = torch.tensor([x_range[0], y_range[0]], device=device)
        self.nx = math.ceil((x_range[1] - x_range[0]) / resolution)
        self.ny = math.ceil((y_range[1] - y_range[0]) / resolution)
        self.occupied = torch.zeros(num_envs, self.nx * self.ny, dtype=torch.bool, device=device)
        ix, iy = torch.meshgrid(torch.arange(self.nx, device=device), torch.arange(self.ny, device=device), indexing="ij")
        self.centers = (torch.stack([ix, iy], dim=-1).reshape(-1, 2).float() + 0.5) * resolution + self.origin

    def reset(self, env_ids: torch.Tensor) -> None:
        self.occupied[env_ids] = False

    def insert(self, points_xy: torch.Tensor, valid: torch.Tensor) -> None:
        """Mark cells hit by ``points_xy`` (N, P, 2), in the grid's frame, where ``valid`` (N, P) is True."""
        idx = torch.floor((torch.nan_to_num(points_xy, posinf=-1e6, neginf=-1e6) - self.origin) / self.res).long()
        inside = valid & (idx[..., 0] >= 0) & (idx[..., 0] < self.nx) & (idx[..., 1] >= 0) & (idx[..., 1] < self.ny)
        env = torch.arange(points_xy.shape[0], device=points_xy.device).unsqueeze(-1).expand_as(inside)
        self.occupied[env[inside], (idx[..., 0] * self.ny + idx[..., 1])[inside]] = True

    def nearest(self, xy: torch.Tensor, k: int) -> tuple[torch.Tensor, torch.Tensor]:
        """The ``k`` occupied cells nearest to ``xy`` (N, 2): centres (N, k, 2) and validity mask (N, k)."""
        dist = torch.linalg.norm(self.centers.unsqueeze(0) - xy.unsqueeze(1), dim=-1)
        dist = torch.where(self.occupied, dist, torch.full_like(dist, math.inf))
        d, idx = dist.topk(min(k, dist.shape[1]), dim=-1, largest=False)
        return self.centers[idx], torch.isfinite(d)


@dataclass
class LocalPlannerCfg:
    speeds: tuple[float, ...] = (-0.15, 0.0, 0.15, 0.3, 0.45, 0.6)
    """Candidate forward speeds [m/s]. Indoor power-chair speeds are typically 0.5-1.2 m/s."""
    max_yaw_rate: float = 0.8
    """[rad/s]."""
    num_yaw_rates: int = 17
    horizon: float = 2.0
    """Rollout duration [s]."""
    num_steps: int = 10
    """Footprint checks per rollout."""
    safe_time: float = 1.0
    """An arc is admissible if it stays collision-free for at least this long [s]."""
    margin: float = 0.03
    """Footprint inflation for collision checks [m]."""
    max_points: int = 2048
    """Nearest remembered obstacle cells considered per environment."""
    clearance_cap: float = 0.20
    """Clearance beyond this distance [m] earns no extra reward."""
    max_accel: float = 0.8
    """Linear acceleration limit applied to the output command [m/s^2]."""
    max_yaw_accel: float = 2.0
    """Yaw acceleration limit applied to the output command [rad/s^2]."""
    # Scoring weights.
    w_progress: float = 2.0
    """Reward per metre travelled along the goal direction."""
    w_clearance: float = 1.0
    """Reward per metre of (capped) clearance between the footprint and obstacles along the arc."""
    w_time: float = 0.5
    """Reward for time-to-collision, normalised by the horizon."""
    w_heading: float = 0.3
    """Cost per radian between the final heading and the goal direction."""
    w_reverse: float = 0.5
    """Penalty for reversing arcs, so they are only used to escape."""
    w_idle: float = 0.6
    """Penalty for standing still, so the chair backs out of dead ends instead of freezing."""


class LocalPlanner:
    """Dynamic-window local planner over constant-twist arcs, checked against the chair's rectangular footprint."""

    def __init__(self, cfg: LocalPlannerCfg, spec: WheelchairSpec, num_envs: int, control_dt: float, device: str | torch.device):
        self.cfg, self.dt = cfg, control_dt
        speeds = torch.tensor(cfg.speeds, device=device)
        rates = torch.linspace(-cfg.max_yaw_rate, cfg.max_yaw_rate, cfg.num_yaw_rates, device=device)
        self.v = speeds.repeat_interleave(len(rates))  # (C,)
        self.w = rates.repeat(len(speeds))  # (C,)
        self.t = torch.linspace(cfg.horizon / cfg.num_steps, cfg.horizon, cfg.num_steps, device=device)  # (K,)
        # Constant-twist (unicycle) poses along each arc, relative to the current pose: (C, K)
        self.theta = self.w[:, None] * self.t[None]
        straight = self.w.abs() < 1e-4
        w_safe = torch.where(straight, torch.ones_like(self.w), self.w)[:, None]
        self.px = torch.where(straight[:, None], self.v[:, None] * self.t, self.v[:, None] / w_safe * torch.sin(self.theta))
        self.py = torch.where(straight[:, None], torch.zeros_like(self.theta), self.v[:, None] / w_safe * (1 - torch.cos(self.theta)))
        x_min, x_max, y_min, y_max = spec.footprint
        m = cfg.margin
        self.box = (x_min - m, x_max + m, y_min - m, y_max + m)
        self.idle = (self.v == 0) & (self.w == 0)
        self.command = torch.zeros(num_envs, 2, device=device)  # rate-limited (v, w) actually sent

    def reset(self, env_ids: torch.Tensor) -> None:
        self.command[env_ids] = 0.0

    def _inside(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        x0, x1, y0, y1 = self.box
        return (x > x0) & (x < x1) & (y > y0) & (y < y1)

    def _distance_to_box(self, x: torch.Tensor, y: torch.Tensor) -> torch.Tensor:
        x0, x1, y0, y1 = self.box
        dx = torch.clamp(torch.maximum(x0 - x, x - x1), min=0.0)
        dy = torch.clamp(torch.maximum(y0 - y, y - y1), min=0.0)
        return torch.hypot(dx, dy)

    @torch.no_grad()
    def __call__(
        self, obstacles_b: torch.Tensor, valid: torch.Tensor, direction_b: torch.Tensor
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Choose a body command.

        Args:
            obstacles_b: Obstacle points in base_link (x, y), shape (N, M, 2).
            valid: Which points are real obstacles, shape (N, M).
            direction_b: Unit vector of the desired travel direction in base_link, shape (N, 2).

        Returns:
            Linear velocity (N,) [m/s] and yaw rate (N,) [rad/s] after acceleration limits, and the chosen
            arc's time-to-collision (N,) [s].
        """
        cfg = self.cfg
        ox, oy = obstacles_b.unbind(-1)
        # Points already within the inflated footprint are contacts, not something to plan around.
        valid = valid & ~self._inside(ox, oy)
        # Obstacles expressed in each rollout pose's frame: (N, M, C, K)
        dx = ox[:, :, None, None] - self.px
        dy = oy[:, :, None, None] - self.py
        cos_t, sin_t = torch.cos(self.theta), torch.sin(self.theta)
        lx, ly = cos_t * dx + sin_t * dy, -sin_t * dx + cos_t * dy
        mask = valid[:, :, None, None]
        hit = (mask & self._inside(lx, ly)).any(dim=1)  # (N, C, K)
        far = torch.full_like(lx, cfg.clearance_cap)
        clearance = torch.where(mask, self._distance_to_box(lx, ly), far).amin(dim=(1, 3)).clamp(max=cfg.clearance_cap)

        any_hit = hit.any(dim=-1)
        first = torch.where(any_hit, hit.float().argmax(dim=-1), torch.full_like(any_hit, cfg.num_steps, dtype=torch.long))
        ttc = first.float() * (cfg.horizon / cfg.num_steps)  # time of the last collision-free sample
        last = (first - 1).clamp(min=0).unsqueeze(-1)
        end_x, end_y, end_t = (
            torch.where(first > 0, a.expand(len(first), -1, -1).gather(-1, last).squeeze(-1), 0.0)
            for a in (self.px, self.py, self.theta)
        )

        gx, gy = direction_b[:, 0:1], direction_b[:, 1:2]
        progress = end_x * gx + end_y * gy
        heading_err = torch.atan2(torch.sin(end_t - torch.atan2(gy, gx)), torch.cos(end_t - torch.atan2(gy, gx))).abs()
        score = (
            cfg.w_progress * progress
            + cfg.w_clearance * clearance
            + cfg.w_time * ttc / cfg.horizon
            - cfg.w_heading * heading_err
            - cfg.w_reverse * (self.v < 0)
            - cfg.w_idle * self.idle
        )
        # Prefer admissible arcs; if none, take the arc that stays clear the longest.
        score = torch.where(ttc >= cfg.safe_time, score, -1e3 + ttc)
        best = score.argmax(dim=-1)
        target = torch.stack([self.v[best], self.w[best]], dim=-1)
        limit = torch.tensor([cfg.max_accel, cfg.max_yaw_accel], device=target.device) * self.dt
        self.command += (target - self.command).clamp(-limit, limit)
        return self.command[:, 0].clone(), self.command[:, 1].clone(), ttc.gather(-1, best.unsqueeze(-1)).squeeze(-1)
