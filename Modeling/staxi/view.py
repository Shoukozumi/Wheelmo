"""Show the CAD model in the OCP CAD Viewer panel in VS Code, one named and coloured part per link.

usage: python view.py [--scan-pose] [--armrest DEG]
"""
import argparse
import math
import sys

from ocp_vscode import show
from ocp_vscode.comms import get_port

import model
from build import SCAN_POSE

COLORS = {"metal": "#c0c2c7", "cushion": "#2a3550", "wheel": "#38383c", "fork": "#8c9199", "armrest": "#262626",
          "stripe_white": "#eeeeee", "stripe_black": "#111111", "plastic": "#38383d"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan-pose", action="store_true", help="brake bar unsqueezed, casters as scanned")
    ap.add_argument("--armrest", type=float, default=0.0, help="armrest flip-up angle in degrees (0-110)")
    a = ap.parse_args()
    if get_port() is None:
        sys.exit("No OCP CAD Viewer is open. In VS Code: OCP CAD Viewer sidebar -> Open viewer (or Ctrl+K V), then re-run.")

    links = model.build(brake_squeezed=not a.scan_pose)
    q = dict(SCAN_POSE) if a.scan_pose else {}
    q.update({"armrest_l": math.radians(a.armrest), "armrest_r": math.radians(a.armrest)})
    T = model.link_world_transforms(links, q)

    shapes, names, colors = [], [], []
    for name, L in links.items():
        if name == "base_link":
            metal, cushion = (p[0] for p in L["mass_parts"])
            parts = [(metal, "frame", "metal"), (cushion, "cushion", "cushion")]
        elif "paint" in L:
            parts = [(shape, f"{name}_{i}", mat) for i, (shape, mat) in enumerate(L["paint"])]
        else:
            kind = "wheel" if "wheel" in name else ("fork" if "fork" in name else "armrest")
            parts = [(L["shape"], name, kind)]
        for shape, label, kind in parts:
            shapes.append(T[name] * shape); names.append(label); colors.append(COLORS[kind])
    show(*shapes, names=names, colors=colors)


if __name__ == "__main__":
    main()
