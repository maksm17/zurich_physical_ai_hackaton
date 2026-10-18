#!/usr/bin/env bash
# One-time RPi 5 setup (Ubuntu 24.04 or Raspberry Pi OS, 64-bit). Run ON the RPi.
set -euo pipefail
if ! command -v docker >/dev/null; then
  curl -fsSL https://get.docker.com | sh
fi
sudo usermod -aG docker "$USER"
sudo apt-get install -y i2c-tools rsync
# I2C: Raspberry Pi OS -> raspi-config; Ubuntu -> enabled by default (dtparam=i2c_arm=on)
CFG=/boot/firmware/config.txt
grep -q '^dtparam=i2c_arm=on' $CFG || echo 'dtparam=i2c_arm=on' | sudo tee -a $CFG
echo "Re-login (docker group), reboot if I2C was just enabled, then: i2cdetect -y 1  (expect 68)"
