# OCP CAD Viewer — controls cheat sheet

For the VS Code **OCP CAD Viewer** extension (v4.3.0). Press `h` inside the viewer for the built-in help.

## Showing the wheelchair

1. Open the viewer: OCP CAD Viewer icon in the left sidebar → **Open viewer** (or `Ctrl+K` then `V`).
2. In the VS Code terminal, from the `Modeling/staxi` folder:

   ```bash
   ~/.venvs/wheelchair/bin/python view.py                 # brake bar zip-tied, casters straight
   ~/.venvs/wheelchair/bin/python view.py --armrest 110   # armrests flipped up (0-110 deg)
   ~/.venvs/wheelchair/bin/python view.py --scan-pose     # pose as scanned (bar unsqueezed, casters turned)
   ```

Model axes: **X** forward, **Y** to the chair's left, **Z** up; units are mm. Press `A` to show the axes.

## Rotation mode

This folder's `.vscode/settings.json` sets `"OcpCadViewer.view.orbit_control": true`:

- **Orbit** (on): Z stays up. Drag left/right to spin around the vertical axis; drag up/down to tilt. The model never rolls sideways.
- **Trackball** (the extension's default): free tumbling in any direction. To use it, delete that line, then close and reopen the Viewer tab.

## Mouse navigation

| Action | Input |
|---|---|
| Rotate | left-drag |
| Rotate up / down only | `Ctrl` + left-drag |
| Rotate left / right only | `Win` + left-drag (Windows may grab the Win key) |
| Pan | right-drag, or `Shift` + left-drag |
| Zoom | mouse wheel, or middle-drag |

Rotation pivots around the point under the cursor; if the drag starts on empty background, it pivots around the model's centre. Zoom moves toward the cursor.

## Keyboard shortcuts

Click inside the viewer first so it has focus.

**Camera views**

| Key | View |
|---|---|
| `1` | front |
| `3` | rear |
| `4` | left |
| `6` | right |
| `8` | top |
| `2` | bottom |
| `5` | isometric |
| `R` | reset camera |
| `r` | resize / fit to window |

**Display toggles**

| Key | Toggle |
|---|---|
| `t` | transparency (see the frame through the cushion) |
| `b` | black edges |
| `x` | explode view |
| `p` | perspective / orthographic |
| `A` | axes |
| `0` | axes at the origin |
| `g` | grid |
| `G` | XY grid |
| `L` | Z scale |

**Tools**

| Key | Tool |
|---|---|
| `D` | measure distance |
| `P` | show properties |
| `I` | select |
| `h` | help |

**Side-panel tabs**

| Key | Tab |
|---|---|
| `T` | object tree |
| `C` | clipping (cut the model with planes) |
| `M` | material |
| `Z` | zebra (surface smoothness) |
| `S` | studio (nicer lighting) |

**Animation:** `Space` plays and `Esc` stops.

## Selecting and hiding parts

| Action | In the 3D view | In the object tree |
|---|---|---|
| Pick / select | double-click a part (shows its bounding box) | click a label |
| Isolate (show only this) | `Shift` + double-click | `Shift` + click a label |
| Hide | `Win` + double-click | `Win` + click a label |
| Hide all others | — | `Shift` + `Win` + click a label |
| Set the camera target | `Shift` + `Win` + double-click | — |

The eye icons in the object tree also show or hide each part.

## Measure mode (`D` or `P`)

| Action | Input |
|---|---|
| Select the 1st (and 2nd) object | left-click |
| Measure to the center instead of the closest point | `Shift` + click the 2nd object |
| Filter what can be picked | `a` all, `s` solid, `f` face, `e` edge, `v` vertex |
| Unselect the last pick | right-click |
| Unselect all | `Esc` |
