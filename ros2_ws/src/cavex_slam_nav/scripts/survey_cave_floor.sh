#!/usr/bin/env bash
# Live probe-drop survey of the real vendored cave mesh (cave_world.obj), used to
# re-derive cavex_world.world's water_surface region (commit d579bc3c) from actual
# floor geometry instead of an unverified whole-bounding-box guess.
#
# Method: spawn small 0.5m boxes from z=25 (above the mesh's tallest recorded point,
# z=36.18) at a grid of (x, y) points, let them settle under gravity, and read back
# their rest pose from `gz topic -t /world/cavex_world/pose/info`. A point that
# settles near its drop (x, y) with z close to the box's half-height (0.25) is real,
# flat, open floor; a point that drifts far or settles much higher indicates an
# obstruction (rubble, a ledge, a slope) between the drop height and the floor.
#
# Requires a running `cavex_world` simulation (gazebo_tracked_vehicle.launch.py or
# equivalent). Run from a sourced ROS2/Gazebo shell.
#
# Usage: survey_cave_floor.sh <name_prefix> <x1> <x2> ... -- <y1> <y2> ...
# Example (this project's actual water-region survey):
#   survey_cave_floor.sh gp 15 20 25 30 35 40 45 50 55 60 65 -- -5 0 5

set -euo pipefail

WORLD=cavex_world
PREFIX="${1:?name prefix required}"
shift

xs=()
while [[ "${1:-}" != "--" ]]; do
  xs+=("$1"); shift
done
shift # consume --
ys=("$@")

drop_probe() {
  local name=$1 x=$2 y=$3
  gz service -s /world/$WORLD/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 3000 \
    --req "sdf: '<sdf version=\"1.9\"><model name=\"$name\"><pose>$x $y 25 0 0 0</pose><link name=\"l\"><inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial><collision name=\"c\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"v\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>' name: '$name' pose: {position: {x: $x, y: $y, z: 25}}" \
    > /dev/null 2>&1
}

yname() { # encode a signed y into a name-safe token, e.g. -5 -> n5, 5 -> 5
  local y=$1
  if [[ "$y" == -* ]]; then echo "n${y#-}"; else echo "$y"; fi
}

echo "spawning probe grid..." >&2
for x in "${xs[@]}"; do
  for y in "${ys[@]}"; do
    drop_probe "${PREFIX}_${x}_$(yname "$y")" "$x" "$y"
  done
done

echo "waiting to settle (12s)..." >&2
sleep 12

gz topic -t /world/$WORLD/pose/info -e -n 1 2>/dev/null > /tmp/${PREFIX}_survey_dump.txt

python3 - "$PREFIX" << 'EOF'
import re, sys
prefix = sys.argv[1]
txt = open(f'/tmp/{prefix}_survey_dump.txt').read()
blocks = re.findall(
    rf'name: "({prefix}_[^"]+)"\s*id: \d+\s*position \{{\s*x: ([\-\d.]+)\s*y: ([\-\d.]+)\s*z: ([\-\d.]+)',
    txt)
rows = []
for name, x, y, z in blocks:
    parts = name.split('_')
    x0 = float(parts[1])
    ytok = parts[2]
    y0 = -float(ytok[1:]) if ytok.startswith('n') else float(ytok)
    rows.append((x0, y0, float(x), float(y), float(z)))
rows.sort()
print(f"{'x0':>7} {'y0':>7} {'x_rest':>8} {'y_rest':>8} {'z_rest':>8}")
for x0, y0, x, y, z in rows:
    print(f"{x0:7.1f} {y0:7.1f} {x:8.2f} {y:8.2f} {z:8.2f}")
EOF

echo "cleaning up probes..." >&2
for x in "${xs[@]}"; do
  for y in "${ys[@]}"; do
    gz service -s /world/$WORLD/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 \
      --req "name: '${PREFIX}_${x}_$(yname "$y")', type: MODEL" > /dev/null 2>&1 || true
  done
done
