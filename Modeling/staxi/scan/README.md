# Scan files (not in git)

These are too big for git (see `../../.gitignore`), so they're shared separately (Git LFS or a shared drive):

| File | Size | What |
|---|---|---|
| `model.obj` | 188 MB | Raw Artec phone scan of the Staxi: 1.33 M vertices, 2.64 M triangles, includes floor and room clutter. Units are arbitrary (see the scale note in `../../README.md`) |
| `model.mtl` | tiny | Material file: points `model.obj` at the texture below |
| `model_1.jpg` | 8 MB | 4096x4096 colour texture atlas for `model.obj` (mostly floor) |
| `wheelchair_clean_aligned.stl` | 53 MB | Output of `../clean_scan.py`: floor removed, chair isolated, aligned (X = chair right, Y = forward, Z = up), still in scan units |

Keep the three `model.*` files together, or viewers lose the colour.
Rebuild the STL with `python clean_scan.py` (about a minute).
