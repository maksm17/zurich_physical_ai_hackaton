#!/usr/bin/env bash
# MuJoCo simulation + gait on the laptop.
#   scripts/sim.sh                    # viewer + gait
#   scripts/sim.sh viewer:=false      # headless
# Then in another terminal:  ros2 param set /gait walk true
set -e
cd "$(dirname "$0")/../ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# VS Code installed as snap leaks its GTK/GLib env into the terminal -> GLFW viewer crashes
if [ -n "$SNAP" ]; then
  unset GSETTINGS_SCHEMA_DIR GTK_PATH GIO_MODULE_DIR LOCPATH
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}"
fi
exec ros2 launch biped_sim sim.launch.py "$@"
