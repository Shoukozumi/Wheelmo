#!/usr/bin/env bash
# URDF -> USD for Isaac Lab (free base, joint drives off). Run after every build.py.
# The URDF importer never overwrites: next to an old copy it writes staxi_wheelchair_1, _2, ...
# so the old USD is cleared first.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ISAACLAB="${ISAACLAB:-$HOME/Development/Learning/Robo/IsaacLab}"

rm -rf "$HERE/out/usd"
cd "$ISAACLAB"
uv run python scripts/tools/convert_urdf.py "$HERE/out/staxi_wheelchair.urdf" "$HERE/out/usd" \
  --joint-stiffness 0.0 --joint-damping 0.0 --joint-target-type none
