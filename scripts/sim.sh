#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
if [ -n "$SNAP" ]; then
  unset GSETTINGS_SCHEMA_DIR GTK_PATH GIO_MODULE_DIR LOCPATH
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}"
fi
exec ros2 launch biped_sim sim.launch.py "$@"
