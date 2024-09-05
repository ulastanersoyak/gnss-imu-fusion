#!/usr/bin/env bash
# Builds positioning_sim, starts Gazebo with the positioning world and runs
# the bridge. Needs Gazebo Harmonic (on Fedora: run inside the `gazebo`
# distrobox, e.g. `distrobox enter gazebo -- sim/run.sh`).
#
#   sim/run.sh [--headless] [positioning_sim options, see --help]
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
build=$root/build-sim

headless=false
args=()
for arg in "$@"; do
  if [[ $arg == --headless ]]; then headless=true; else args+=("$arg"); fi
done

cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build" -j --target positioning_sim AccuracyOverlay
export GZ_GUI_PLUGIN_PATH="$build/sim${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"

if $headless; then
  gz sim -s -r "$root/sim/worlds/positioning.sdf" &>"$build/gz.log" &
else
  gz sim -r --gui-config "$root/sim/gui.config" \
    "$root/sim/worlds/positioning.sdf" &>"$build/gz.log" &
fi
gz_pid=$!
cleanup() {
  kill "$gz_pid" 2>/dev/null || true
  wait "$gz_pid" 2>/dev/null || true
}
trap cleanup EXIT

"$build/sim/positioning_sim" "${args[@]}"
