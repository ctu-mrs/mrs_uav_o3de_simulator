#!/bin/bash
# Start the O3DE Editor, or Project Manager with --manager, with ROS 2 sourced.
#
# Without ROS, the ROS and Multirotor Gems fail to load: the Asset Processor compiles prefabs without
# their components, and saving in the Editor removes those components from the prefabs for good.

ROS_SETUP=/opt/ros/jazzy/setup.bash
O3DE_BIN=/opt/O3DE/26.05/bin/Linux/profile/Default
PROJECT_ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"

if [ ! -f "$ROS_SETUP" ]; then
  echo "$ROS_SETUP not found" >&2
  exit 1
fi

source "$ROS_SETUP"

if [ -z "$ROS_DISTRO" ]; then
  echo "sourcing $ROS_SETUP did not set ROS_DISTRO" >&2
  exit 1
fi

# A desktop shortcut does not inherit this from the shell, and the MRS UAV System uses Zenoh.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"

if [ "$1" = "--manager" ]; then
  exec "$O3DE_BIN/o3de"
fi

exec "$O3DE_BIN/Editor" --project-path="$PROJECT_ROOT" "$@"
