#!/usr/bin/env python3
"""Bake a graded water-entry ramp straight into the cave mesh OBJs.

Replaces the pitched-box `water_entry_ramp` model: a triangulated sheet that
starts FLUSH with the cave floor (z=5.98) west of the basin-void edge, then
grades gently down into the water. Because it is mesh, welded to the floor
level at its top edge, there is no box end-face for the tracks to catch on.

Appended between `# >>> water_grade >>>` markers to BOTH cave_world_holed.obj
(collision) and cave_world_visual.obj (visual). Idempotent: strips its own
block first. Run AFTER add_floor_seal.py.

World <-> mesh-local (model.sdf <scale>2 2 2</scale>, include pose
`18.7830 31.4050 5.9826 1.5708 0 1.5708`):
    world (x,y,z) = 2*(z_l, x_l, y_l) + (18.783, 31.405, 5.9826)
so  x_l=(wy-31.405)/2,  y_l=(wz-5.9826)/2,  z_l=(wx-18.783)/2 ; world +Z == local +Y.
"""
import sys
from pathlib import Path

MESH_DIR = Path(__file__).resolve().parent.parent
BEGIN = "# >>> water_grade (tools/grade_water_ramp.py) >>>"
END = "# <<< water_grade <<<"

# world-frame ramp: x[X0,X1] y[Y0,Y1]; z(x) = Z_TOP for x<=XG_TOP, then linear
# down to Z_BOT at x=XG_BOT (flat/submerged past that).
X0, X1 = -8.0, 12.0
Y0, Y1 = -14.0, 26.0
Z_TOP, Z_BOT = 5.98, 5.30
XG_TOP, XG_BOT = 0.0, 14.0       # flat top x[-8,0] at z=5.98 (coplanar with the
                                # cave floor), then ~4 deg slope x[0,14]. The
                                # flat top runs well west of the cull seam so
                                # the ramp<->floor join is flat-on-flat.
GRID = 0.4   # smaller than the 0.6 m track box so the tracks do not catch on triangle edges
DROP = 0.0                        # flat top EXACTLY flush with the cave floor (5.98) -- no step at x=-4

# Option B: cull every near-horizontal floor triangle the ramp overlaps
# (original cave floor + add_floor_seal.py sheet) so the ramp is the SOLE
# collision surface through the shore -- no coincident sheets for the box
# tracks to wedge between. World-frame box; only faces with ALL 3 verts in
# the z band are dropped, so walls/ceiling are untouched.
CULL_X = (-7.5, 11.5)   # inside the ramp footprint (X0=-8) so the ramp overhangs
CULL_Y = (-13.5, 25.5)  # the cull seam and the join is flat-on-flat coplanar
CULL_Z = (5.50, 6.15)


def _to_world(lx, ly, lz):
    # model.sdf <scale>2 2 2</scale> + include pose 18.783 31.405 5.9826 / rpy 90,0,90
    return (2.0 * lz + 18.7830, 2.0 * lx + 31.4050, 2.0 * ly + 5.9826)


def cull_floor(lines):
    """Drop original floor/seal faces inside the ramp footprint. Vertices are
    left in place (orphans are harmless); face indices stay valid."""
    verts = []  # 1-indexed -> world xyz
    for ln in lines:
        if ln.startswith("v "):
            p = ln.split()
            verts.append(_to_world(float(p[1]), float(p[2]), float(p[3])))
    out, dropped = [], 0
    for ln in lines:
        if not ln.startswith("f "):
            out.append(ln)
            continue
        idx = [int(t.split("/")[0]) for t in ln.split()[1:]]
        w = [verts[i - 1] for i in idx]
        cx = sum(v[0] for v in w) / len(w)
        cy = sum(v[1] for v in w) / len(w)
        in_box = (CULL_X[0] <= cx <= CULL_X[1] and CULL_Y[0] <= cy <= CULL_Y[1])
        flat = all(CULL_Z[0] <= v[2] <= CULL_Z[1] for v in w)
        if in_box and flat:
            dropped += 1
            continue
        out.append(ln)
    return out, dropped


def ramp_z(x):
    if x <= XG_TOP:
        return Z_TOP - DROP
    if x >= XG_BOT:
        return Z_BOT - DROP
    f = (x - XG_TOP) / (XG_BOT - XG_TOP)
    return (Z_TOP + f * (Z_BOT - Z_TOP)) - DROP


def strip_block(lines):
    out, skip = [], False
    for ln in lines:
        if ln.startswith(BEGIN):
            skip = True
            continue
        if ln.startswith(END):
            skip = False
            continue
        if not skip:
            out.append(ln)
    return out


def bake(path: Path):
    lines = strip_block(path.read_text().splitlines())
    lines, culled = cull_floor(lines)
    base_v = sum(1 for ln in lines if ln.startswith("v "))
    base_vn = sum(1 for ln in lines if ln.startswith("vn "))

    xs = [X0 + i * GRID for i in range(int((X1 - X0) / GRID) + 1)]
    ys = [Y0 + i * GRID for i in range(int((Y1 - Y0) / GRID) + 1)]
    nx, ny = len(xs), len(ys)

    vlines, flines = [], []
    for wy in ys:
        for wx in xs:
            wz = ramp_z(wx)
            lx = (wy - 31.4050) / 2.0
            ly = (wz - 5.9826) / 2.0
            lz = (wx - 18.7830) / 2.0
            vlines.append(f"v {lx:.6f} {ly:.6f} {lz:.6f}")
    nrm = base_vn + 1
    for j in range(ny - 1):
        for i in range(nx - 1):
            a = base_v + j * nx + i + 1
            b = base_v + j * nx + (i + 1) + 1
            c = base_v + (j + 1) * nx + (i + 1) + 1
            d = base_v + (j + 1) * nx + i + 1
            flines.append(f"f {a}//{nrm} {b}//{nrm} {c}//{nrm}")
            flines.append(f"f {a}//{nrm} {c}//{nrm} {d}//{nrm}")

    out = lines + [BEGIN, "o water_grade"] + vlines \
        + ["vn 0.000000 1.000000 0.000000"] + flines + [END]
    path.write_text("\n".join(out) + "\n")
    return len(vlines), len(flines), culled


def main() -> int:
    for name in ("cave_world_holed.obj", "cave_world_visual.obj"):
        p = MESH_DIR / name
        if not p.exists():
            print(f"skip (missing): {name}")
            continue
        nv, nf, culled = bake(p)
        print(f"{name}: -{culled} old floor tris, +{nv} verts, +{nf} ramp tris "
              f"(ramp world x[{X0},{X1}] y[{Y0},{Y1}], z {Z_TOP}->{Z_BOT} "
              f"over x[{XG_TOP},{XG_BOT}])")
    return 0


if __name__ == "__main__":
    sys.exit(main())
