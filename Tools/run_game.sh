#!/bin/bash
# Start the game with ROS 2 sourced.
#
#   Tools/run_game.sh [level] [--headless] [launcher flags...]
#   Tools/run_game.sh --list
#
# The level defaults to demolevel. --headless runs without rendering, so without the camera.

ROS_SETUP=/opt/ros/jazzy/setup.bash
PROJECT_ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"
LAUNCHER_DIR="$PROJECT_ROOT/build/linux/bin/profile"
LEVELS_DIR="$PROJECT_ROOT/Cache/linux/levels"

list_levels() {
  find "$LEVELS_DIR" -mindepth 2 -maxdepth 2 -name '*.spawnable' -printf '%f\n' 2>/dev/null | sed 's/\.spawnable$//' | sort
}

if [ "$1" = "--list" ]; then
  list_levels
  exit 0
fi

level=demolevel
if [ -n "$1" ] && [[ "$1" != -* ]]; then
  level="${1,,}"
  shift
fi

flags=(--r_renderScale=0.5)
args=()
for arg in "$@"; do
  if [ "$arg" = "--headless" ]; then
    flags=(--rhi=null)
  else
    args+=("$arg")
  fi
done

if [ ! -x "$LAUNCHER_DIR/DroneSim.GameLauncher" ]; then
  echo "no DroneSim.GameLauncher in '$LAUNCHER_DIR': build the project first" >&2
  exit 1
fi

if [ ! -f "$LEVELS_DIR/$level/$level.spawnable" ]; then
  echo "level '$level' is not compiled. Available:" >&2
  list_levels >&2
  exit 1
fi

source "$ROS_SETUP"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

cd "$LAUNCHER_DIR"
exec ./DroneSim.GameLauncher --project-path="$PROJECT_ROOT" --bg_ConnectToAssetProcessor=0 \
  --t_simulationTickDeltaOverride=17000 "${flags[@]}" \
  --loadlevel="levels/$level/$level.spawnable" "${args[@]}"
