#!/usr/bin/env bash
set -e
detect() {
  local dev
  dev=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -1)
  [ -z "$dev" ] && { echo "no adapter found" >&2; exit 1; }
  udevadm info -q property -n "$dev" | sed -n 's/^ID_PATH=//p'
}
read -rp "Plug in ONLY the LEFT leg adapter, press Enter"
LEFT=$(detect); echo "left:  $LEFT"
read -rp "Unplug it, plug in ONLY the RIGHT leg adapter, press Enter"
RIGHT=$(detect); echo "right: $RIGHT"
[ "$LEFT" = "$RIGHT" ] && { echo "same USB port for both - use different ports"; exit 1; }
sudo tee /etc/udev/rules.d/99-biped.rules >/dev/null <<RULES
SUBSYSTEM=="tty", ENV{ID_PATH}=="$LEFT",  SYMLINK+="biped_left",  MODE="0666"
SUBSYSTEM=="tty", ENV{ID_PATH}=="$RIGHT", SYMLINK+="biped_right", MODE="0666"
RULES
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "Done. Plug both adapters back into the SAME ports, then: ls -l /dev/biped_*"
