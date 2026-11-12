#!/usr/bin/env bash
# Render preview images of a generated world without a GUI.
#
#   scripts/render_world.sh GENERATOR [OUT_DIR]
#   e.g. scripts/render_world.sh make_sar_test_world.py /tmp/sar_test_preview
#
# Runs GENERATOR --preview OUT_DIR (it writes OUT_DIR/<name>_preview.sdf with saving
# cameras and OUT_DIR/cameras.txt), runs the Gazebo server with headless rendering
# until every camera has saved a few frames, then stops it. Output: OUT_DIR/<camera>.png
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEN="$1"
OUT="$(realpath -m "${2:-/tmp/world_preview}")"
mkdir -p "$OUT"

python3 "$HERE/$GEN" --preview "$OUT"
export GZ_SIM_RESOURCE_PATH="$HERE/../models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
SDF="$(ls "$OUT"/*_preview.sdf | head -1)"
mapfile -t cams < "$OUT/cameras.txt"
for c in "${cams[@]}"; do mkdir -p "$OUT/$c"; find "$OUT/$c" -name '*.png' -delete; done

gz sim -s -r --headless-rendering "$SDF" > "$OUT/gz_server.log" 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true; wait $GZ_PID 2>/dev/null || true' EXIT

# Cameras only render while someone subscribes to them.
sleep 5
subs=()
for c in "${cams[@]}"; do
  gz topic -e -t "/preview/$c" > /dev/null 2>&1 &
  subs+=($!)
done

for _ in $(seq 1 180); do
  done_all=1
  for c in "${cams[@]}"; do
    [ "$(find "$OUT/$c" -name '*.png' | wc -l)" -ge 3 ] || done_all=0
  done
  [ $done_all = 1 ] && break
  sleep 1
done
kill "${subs[@]}" 2>/dev/null || true
kill $GZ_PID 2>/dev/null || true
wait $GZ_PID 2>/dev/null || true

# The newest frame may have been cut off when the server stopped: take the one before.
for c in "${cams[@]}"; do
  last="$(find "$OUT/$c" -name '*.png' | sort -V | tail -2 | head -1)"
  if [ -n "$last" ]; then cp "$last" "$OUT/$c.png"; echo "rendered $OUT/$c.png"; else echo "NO IMAGE for $c"; fi
done
