#!/usr/bin/env bash
# Run isaac_stack.py from any folder, in the Isaac Lab checkout's Python environment.
# All arguments go to the script, e.g.
#   bash staxi/run_stack.sh --viz viser                    # browser at http://localhost:8080
#   bash staxi/run_stack.sh --viz newton_gl                # desktop window
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ISAACLAB="${ISAACLAB:-$HOME/Development/Learning/Robo/IsaacLab}"

exec uv run --project "$ISAACLAB" --extra viser python "$HERE/isaac_stack.py" "$@"
