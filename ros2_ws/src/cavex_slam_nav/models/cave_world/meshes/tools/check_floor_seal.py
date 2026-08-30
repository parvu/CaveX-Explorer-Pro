#!/usr/bin/env python3
"""Grid-raycast the corridor floor and print a coverage map.

Reads cave_world_holed.obj (the real mesh) AND cave_floor_seal.obj (the
collision-only seal) together -- a column counts as covered if EITHER has a
surface below world-z 9.

'.' = floor present   'X' = hole   '#' = hole inside a region the seal must cover

After add_floor_seal.py the covered regions should be hole-free and the
flooded-basin void (world x [0,32]) should still read as holes.
"""
import sys
from pathlib import Path

import numpy as np

MESH_DIR = Path(__file__).resolve().parent.parent
OBJS = [MESH_DIR / "cave_world_holed.obj"]  # seal is baked in


def main() -> int:
    V, F = [], []
    for obj in OBJS:
        if not obj.exists():
            continue
        base = len(V)
        for ln in obj.read_text().splitlines():
            if ln.startswith("v "):
                p = ln.split()
                V.append((float(p[1]), float(p[2]), float(p[3])))
            elif ln.startswith("f "):
                idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
                for i in range(1, len(idx) - 1):
                    F.append((base + idx[0] - 1, base + idx[i] - 1, base + idx[i + 1] - 1))
    V = np.array(V)
    F = np.array(F)
    wx = 2 * V[:, 2] + 18.7830
    wy = 2 * V[:, 0] + 31.4050
    wz = 2 * V[:, 1] + 5.9826
    W = np.column_stack([wx, wy, wz])
    a, b, c = W[F[:, 0]], W[F[:, 1]], W[F[:, 2]]
    ax, ay = a[:, 0], a[:, 1]
    e0 = b[:, :2] - a[:, :2]
    e1 = c[:, :2] - a[:, :2]
    d00 = (e0 * e0).sum(1)
    d01 = (e0 * e1).sum(1)
    d11 = (e1 * e1).sum(1)
    den = d00 * d11 - d01 * d01
    ok = np.abs(den) > 1e-12
    bx0 = np.minimum.reduce([a[:, 0], b[:, 0], c[:, 0]])
    bx1 = np.maximum.reduce([a[:, 0], b[:, 0], c[:, 0]])
    by0 = np.minimum.reduce([a[:, 1], b[:, 1], c[:, 1]])
    by1 = np.maximum.reduce([a[:, 1], b[:, 1], c[:, 1]])

    def has_floor(px, py):
        m = ok & (bx0 <= px) & (bx1 >= px) & (by0 <= py) & (by1 >= py)
        if not m.any():
            return False
        v2x = px - ax[m]
        v2y = py - ay[m]
        d20 = e0[m, 0] * v2x + e0[m, 1] * v2y
        d21 = e1[m, 0] * v2x + e1[m, 1] * v2y
        vv = (d11[m] * d20 - d01[m] * d21) / den[m]
        ww = (d00[m] * d21 - d01[m] * d20) / den[m]
        uu = 1 - vv - ww
        ins = (uu >= -1e-6) & (vv >= -1e-6) & (ww >= -1e-6)
        if not ins.any():
            return False
        z = uu[ins] * a[m][ins, 2] + vv[ins] * b[m][ins, 2] + ww[ins] * c[m][ins, 2]
        return bool((z < 9.0).any())

    # drivable regions that the seal must fully cover (world frame), matching
    # add_floor_seal.py's PANELS minus a 1 m inset so edge sampling is fair and
    # the open basin-void boundary columns are excluded.
    COVER = [
        (-40.0, -1.0, -12.0, 12.0),    # water approach / dry corridor
        (33.0, 40.0, -12.0, 12.0),     # corridor east of the basin (mesh cut at x=40)
        (-120.0, -60.0, -45.9, -16.9), # spawn area  (world -88.78,-31.4)
        (-62.0, -38.0, -45.0, 11.0),   # spawn -> approach link
    ]

    def _in_cover(px, py):
        return any(x0 <= px <= x1 and y0 <= py <= y1 for x0, x1, y0, y1 in COVER)

    xs = np.arange(-124, 69, 1.0)
    ys = np.arange(-48, 15, 1.0)
    print("floor coverage (world frame)  '.'=floor  'X'=hole  '#'=hole in a covered region")
    print("     " + "".join("%+d" % x if x % 10 == 0 else " " for x in xs))
    panel_holes = 0
    for y in ys:
        row = ""
        for x in xs:
            f = has_floor(x + 0.01, y + 0.01)
            if f:
                row += "."
            elif _in_cover(x, y):
                row += "#"
                panel_holes += 1
            else:
                row += "X"
        print("%+4d %s" % (y, row))
    print(f"\nunsealed holes inside the covered regions: {panel_holes}")
    return 1 if panel_holes else 0


if __name__ == "__main__":
    sys.exit(main())
