#!/usr/bin/env python3
"""Cap every open corridor mouth / floor gap in the cave mesh.

The vendored cave (and the trim that crops it to the play box) leaves
corridors and floor patches ending in an open ring of boundary edges
(edges used by one triangle only). A vehicle driving/falling out one of
those drops into the void (seen live: z = -199). This welds a centroid
triangle-fan across every such loop -- a vertical wall for a corridor
mouth, a floor patch for a horizontal gap.

Every boundary loop of >= MIN_LOOP edges is capped except the deliberate
circular basin void. RUN trim_cave_mesh.py FIRST, then this, and do not
re-trim after.

Baked between `# >>> cave_caps (tools/cap_open_edges.py) >>>` markers into
cave_world_holed.obj (collision) and cave_world_visual.obj (visual).
Idempotent: strips its own block first.

World <-> mesh-local (model.sdf <scale>2 2 2</scale>, include pose
`18.7830 31.4050 5.9826 1.5708 0 1.5708`):
    world (x,y,z) = 2*(z_l, x_l, y_l) + (18.783, 31.405, 5.9826)
"""
import sys
from pathlib import Path

MESH_DIR = Path(__file__).resolve().parent.parent
BEGIN = "# >>> cave_caps (tools/cap_open_edges.py) >>>"
END = "# <<< cave_caps <<<"

MIN_LOOP = 4                    # ignore boundary loops shorter than this (mesh noise)
BASIN_X = (6.0, 36.0)          # world -- the deliberate basin void, never cap
BASIN_Y = (-12.0, 24.0)
BASIN_Z = (2.0, 8.0)


def to_world(lx, ly, lz):
    return (2.0 * lz + 18.7830, 2.0 * lx + 31.4050, 2.0 * ly + 5.9826)


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


def parse(lines):
    verts, faces = [], []
    for ln in lines:
        p = ln.split()
        if not p:
            continue
        if p[0] == "v":
            verts.append((float(p[1]), float(p[2]), float(p[3])))
        elif p[0] == "f":
            faces.append([int(t.split("/")[0]) for t in p[1:4]])   # 1-indexed
    return verts, faces


def boundary_loops(faces):
    """Edges used by exactly one triangle, walked into closed loops."""
    from collections import defaultdict
    count = defaultdict(int)
    for a, b, c in faces:
        for u, v in ((a, b), (b, c), (c, a)):
            count[(min(u, v), max(u, v))] += 1
    bedges = [e for e, n in count.items() if n == 1]
    adj = defaultdict(list)
    for u, v in bedges:
        adj[u].append(v)
        adj[v].append(u)
    seen = set()
    loops = []
    for u0, v0 in bedges:
        if (min(u0, v0), max(u0, v0)) in seen:
            continue
        loop = [u0]
        cur, prev = v0, u0
        seen.add((min(u0, v0), max(u0, v0)))
        while cur != u0:
            loop.append(cur)
            nxts = [w for w in adj[cur]
                    if (min(cur, w), max(cur, w)) not in seen]
            if not nxts:
                break
            nxt = nxts[0]
            seen.add((min(cur, nxt), max(cur, nxt)))
            prev, cur = cur, nxt
        loops.append(loop)
    return loops


def bake(path: Path):
    lines = strip_block(path.read_text().splitlines())
    verts, faces = parse(lines)
    base_v = len(verts)
    base_vn = sum(1 for ln in lines if ln.startswith("vn "))

    loops = boundary_loops(faces)
    vlines, flines = [], []
    nrm = base_vn + 1
    next_v = base_v + 1
    capped = skipped = 0
    for loop in loops:
        if len(loop) < MIN_LOOP:
            skipped += 1
            continue
        pts = [verts[i - 1] for i in loop]
        wc = [to_world(*p) for p in pts]
        cx = sum(w[0] for w in wc) / len(wc)
        cy = sum(w[1] for w in wc) / len(wc)
        cz = sum(w[2] for w in wc) / len(wc)
        if (BASIN_X[0] <= cx <= BASIN_X[1] and BASIN_Y[0] <= cy <= BASIN_Y[1]
                and BASIN_Z[0] <= cz <= BASIN_Z[1]):
            skipped += 1
            continue
        lx = sum(p[0] for p in pts) / len(pts)
        ly = sum(p[1] for p in pts) / len(pts)
        lz = sum(p[2] for p in pts) / len(pts)
        vlines.append(f"v {lx:.6f} {ly:.6f} {lz:.6f}")
        cidx = next_v
        next_v += 1
        n = len(loop)
        for k in range(n):
            a = loop[k]
            b = loop[(k + 1) % n]
            flines.append(f"f {a}//{nrm} {b}//{nrm} {cidx}//{nrm}")
            flines.append(f"f {b}//{nrm} {a}//{nrm} {cidx}//{nrm}")   # both windings
        capped += 1

    out = lines + [BEGIN, "o cave_caps"] + vlines \
        + ["vn 0.000000 1.000000 0.000000"] + flines + [END]
    path.write_text("\n".join(out) + "\n")
    return capped, skipped, len(flines)


def main() -> int:
    for name in ("cave_world_holed.obj", "cave_world_visual.obj"):
        p = MESH_DIR / name
        if not p.exists():
            print(f"skip (missing): {name}")
            continue
        c, s, nf = bake(p)
        print(f"{name}: capped {c} open loops, skipped {s}, +{nf} tris")
    return 0


if __name__ == "__main__":
    sys.exit(main())
