#!/usr/bin/env bash
# RViz on the laptop showing the real robot's joints (robot must run bringup, e.g. read_only:=true).
#   scripts/rviz_robot.sh [ssh_host]
set -e
cd "$(dirname "$0")/.."
source /opt/ros/jazzy/setup.bash
source ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}   # local only, don't mix with sims
if [ -n "$SNAP" ]; then   # VS Code snap env breaks Qt/GLFW windows
  unset GSETTINGS_SCHEMA_DIR GTK_PATH GIO_MODULE_DIR LOCPATH
  export XDG_DATA_DIRS="${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-/usr/local/share:/usr/share}"
fi
python3 scripts/rviz_robot.py "${1:-rpi}" &
RELAY=$!
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(xacro ws/src/biped_description/urdf/biped.xacro use_mock:=true)" &
RSP=$!
trap 'kill $RELAY $RSP 2>/dev/null' EXIT
rviz2 -d ws/src/biped_description/config/display.rviz
