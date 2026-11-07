"""Write sensor data to disk for inspection: camera frames as PNG, LiDAR as a bird's-eye-view PNG."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw

from .wheelchair_spec import WheelchairSpec


def _to_torch(data) -> torch.Tensor:
    return data.torch if hasattr(data, "torch") else data


def save_rgb(images, path: Path, env: int = 0) -> None:
    """Save environment ``env``'s image from a camera batch of shape (N, H, W, 3|4), uint8 or float in [0, 1]."""
    img = _to_torch(images)[env, ..., :3].detach().cpu()
    if img.dtype != torch.uint8:
        img = (img.clamp(0.0, 1.0) * 255).to(torch.uint8)
    Image.fromarray(img.numpy()).save(path)


def save_bev(points_b: torch.Tensor, spec: WheelchairSpec, path: Path, extent: float = 6.0, px_per_m: int = 60) -> None:
    """Top-down image of LiDAR hits in base_link (forward = up), coloured by height, with the chair footprint."""
    size = int(2 * extent * px_per_m)
    canvas = Image.new("RGB", (size, size), (20, 20, 24))
    draw = ImageDraw.Draw(canvas)

    def to_px(x: float, y: float) -> tuple[float, float]:
        return (size / 2 - y * px_per_m, size / 2 - x * px_per_m)

    for r in range(1, int(extent) + 1):  # range rings every metre
        c = size / 2
        draw.ellipse((c - r * px_per_m, c - r * px_per_m, c + r * px_per_m, c + r * px_per_m), outline=(45, 45, 52))

    pts = _to_torch(points_b).detach().cpu().numpy()
    pts = pts[np.isfinite(pts).all(axis=1)]
    height = pts[:, 2] + spec.drive_wheel_radius  # height above the floor
    pts = pts[(height > 0.04) & (np.abs(pts[:, 0]) < extent) & (np.abs(pts[:, 1]) < extent)]
    height = pts[:, 2] + spec.drive_wheel_radius
    t = np.clip(height / 2.0, 0.0, 1.0)
    colors = np.stack([255 * t, 255 * (1 - np.abs(2 * t - 1)), 255 * (1 - t)], axis=1).astype(np.uint8)
    for (x, y), color in zip(pts[:, :2], colors, strict=True):
        u, v = to_px(float(x), float(y))
        draw.point((u, v), fill=tuple(int(c) for c in color))

    x0, x1, y0, y1 = spec.footprint
    (u0, v0), (u1, v1) = to_px(x1, y1), to_px(x0, y0)
    draw.rectangle((u0, v0, u1, v1), outline=(255, 255, 255))
    draw.line((*to_px(0.0, 0.0), *to_px(x1 + 0.3, 0.0)), fill=(255, 255, 255))
    canvas.save(path)
