#!/bin/bash
# Start the O3DE Editor, or Project Manager with --manager, with ROS 2 sourced.
#
# Without ROS, the ROS and Multirotor Gems fail to load: the Asset Processor compiles prefabs without
# their components, and saving in the Editor removes those components from the prefabs for good.

ROS_SETUP=/opt/ros/jazzy/setup.bash
PROJECT_ROOT="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"

# The engine the project names, as registered in the O3DE manifest. O3DE_ENGINE_PATH overrides it.
find_engine() {
  python3 - "$PROJECT_ROOT/project.json" "$HOME/.o3de/o3de_manifest.json" <<'PYTHON'
import json
import sys

wanted = json.load(open(sys.argv[1])).get("engine")
manifest = json.load(open(sys.argv[2]))
for entry in manifest.get("engines", []):
    path = entry if isinstance(entry, str) else entry.get("path")
    try:
        if json.load(open(f"{path}/engine.json")).get("engine_name") == wanted:
            print(path)
            break
    except OSError:
        pass
PYTHON
}

ENGINE="${O3DE_ENGINE_PATH:-$(find_engine)}"
O3DE_BIN="$ENGINE/bin/Linux/profile/Default"

if [ ! -x "$O3DE_BIN/Editor" ]; then
  echo "no O3DE Editor in '$O3DE_BIN': register the engine, or set O3DE_ENGINE_PATH" >&2
  exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
  echo "$ROS_SETUP not found" >&2
  exit 1
fi

source "$ROS_SETUP"

if [ -z "$ROS_DISTRO" ]; then
  echo "sourcing $ROS_SETUP did not set ROS_DISTRO" >&2
  exit 1
fi

# A desktop shortcut does not inherit this from the shell.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [ "$1" = "--manager" ]; then
  exec "$O3DE_BIN/o3de"
fi

exec "$O3DE_BIN/Editor" --project-path="$PROJECT_ROOT" "$@"
