#!/usr/bin/env python3
"""Trim a cave OBJ: crop to a world-frame box, then drop isolated mesh regions.

  python3 trim_cave_mesh.py [SRC [DST]]

Default SRC=DST=cave_world_holed.obj (in-place). Pass e.g.
`cave_world.obj cave_world_visual.obj` to make the trimmed visual mesh.

Steps:
  1. keep a face only if its centroid is inside
     [X_MIN,X_MAX] x [Y_MIN,Y_MAX] x (-inf, Z_MAX]  (world frame)
  2. of what's left, keep only connected components with >= KEEP_FRAC of the
     faces -- removes floating debris / detached tiles.

Only `f` lines are rewritten; `v/vt/vn` stay (unreferenced verts are ignored by
loaders). `git checkout` reverts.

World <-> mesh-local (model.sdf <scale>2 2 2</scale>, include pose
`18.7830 31.4050 5.9826 1.5708 0 1.5708`):  world (x,y,z) = 2*(z_l,x_l,y_l) + t.
"""
import sys
from pathlib import Path

MESH_DIR = Path(__file__).resolve().parent.parent

X_MIN, X_MAX = -130.0, 40.0
Y_MIN, Y_MAX = -60.0, 30.0
Z_MAX = 50.0
KEEP_FRAC = 0.02  # drop connected components smaller than this fraction of faces

MARK_PREFIX = "# trimmed by tools/trim_cave_mesh.py"


def w(v):  # mesh-local -> world (x, y, z)
    x, y, z = v
    return (2 * z + 18.7830, 2 * x + 31.4050, 2 * y + 5.9826)


class DSU:
    def __init__(self, n):
        self.p = list(range(n))

    def find(self, a):
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.p[ra] = rb


def main() -> int:
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else MESH_DIR / "cave_world_holed.obj"
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else src
    if not src.is_absolute():
        src = MESH_DIR / src
    if not dst.is_absolute():
        dst = MESH_DIR / dst

    lines = src.read_text().splitlines()
    verts = [None]
    faces = []  # (line_index, [vertex ids])
    for li, ln in enumerate(lines):
        if ln.startswith("v "):
            p = ln.split()
            verts.append((float(p[1]), float(p[2]), float(p[3])))
        elif ln.startswith("f "):
            faces.append((li, [int(t.split("/")[0]) for t in ln.split()[1:]]))

    # --- step 1: box crop
    box_keep = []
    for li, ids in faces:
        n = len(ids)
        cx = sum(w(verts[i])[0] for i in ids) / n
        cy = sum(w(verts[i])[1] for i in ids) / n
        cz = sum(w(verts[i])[2] for i in ids) / n
        if X_MIN <= cx <= X_MAX and Y_MIN <= cy <= Y_MAX and cz <= Z_MAX:
            box_keep.append((li, ids))

    # --- step 2: connected components over shared vertices, keep the big ones
    dsu = DSU(len(verts))
    for _, ids in box_keep:
        for k in range(1, len(ids)):
            dsu.union(ids[0], ids[k])
    comp_count = {}
    for _, ids in box_keep:
        r = dsu.find(ids[0])
        comp_count[r] = comp_count.get(r, 0) + 1
    total = len(box_keep)
    min_faces = max(1, int(KEEP_FRAC * total))
    big = {r for r, c in comp_count.items() if c >= min_faces}
    sizes = sorted(comp_count.values(), reverse=True)
    print(f"components: {len(comp_count)}  top sizes: {sizes[:10]}  (keep >= {min_faces})")

    keep_lines = {
        li for li, ids in box_keep if dsu.find(ids[0]) in big
    }

    out, kept, dropped = [], 0, 0
    for li, ln in enumerate(lines):
        if ln.startswith("f "):
            if li in keep_lines:
                kept += 1
                out.append(ln)
            else:
                dropped += 1
        elif ln.startswith(MARK_PREFIX):
            continue
        else:
            out.append(ln)

    mark = (f"{MARK_PREFIX}: box x[{X_MIN},{X_MAX}] y[{Y_MIN},{Y_MAX}] z<={Z_MAX}, "
            f"components >= {KEEP_FRAC:.0%}")
    out.insert(1, mark)
    dst.write_text("\n".join(out) + "\n")
    print(f"{src.name} -> {dst.name}: faces kept {kept}, dropped {dropped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
