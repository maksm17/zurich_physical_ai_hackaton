# zurich_physical_ai_hackaton

6-DOF biped: ST3215 servos (2× Waveshare Bus Servo Adapter, one per leg), RPi 5, MPU6050 + Madgwick, ROS 2 Jazzy.

```
ws/src/
├── biped_description/   URDF + biped.ros2control (joint -> servo ID / port / sign / zero)
├── st3215_hardware/     ros2_control SystemInterface (C++), STS protocol, sync read/write, N ports
└── biped_control/       bringup launch, gait_node (feedforward + IMU), mpu6050_node, st3215_tool
```

| joint | servo ID | adapter | role |
|---|---|---|---|
| l_up / r_up | 1 / 4 | left / right | hip roll |
| l_mid / r_mid | 2 / 5 | left / right | hip pitch |
| l_down / r_down | 3 / 6 | left / right | ankle pitch |

## Docker (recommended)

One-time on the RPi (copy the script over or paste it):

```bash
scripts/setup_rpi.sh          # docker, i2c-tools, enables I2C; then re-login / reboot
```

From the laptop, every time you change something:

```bash
scripts/deploy.sh pi@raspberrypi.local          # rsync repo (without keys) -> build on RPi -> start
SSH_KEY=./zurich_key scripts/deploy.sh pi@<ip>  # with a specific key
```

On the RPi:

```bash
docker compose up -d                                         # bringup (real servos)
docker compose logs -f
docker exec -it biped bash                                   # shell with ROS sourced
docker exec -it biped ros2 param set /gait walk true
docker compose run --rm robot ros2 run biped_control st3215_tool --port /dev/ttyAMA0 scan
LAUNCH_ARGS="gait:=false" docker compose up -d               # extra launch args
docker compose down
```

- `./ws/src` is mounted into the container: Python / launch / yaml changes only need `docker compose restart`.
  C++ (`st3215_hardware`) changes need `docker compose up -d --build`.
- `network_mode: host` + same `ROS_DOMAIN_ID` (default 42) → RViz / `ros2 topic` on the laptop see the robot.
- Laptop test without hardware: `LAUNCH_ARGS="use_mock:=true imu:=false" docker compose up`.

## Setup without Docker (RPi)

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-imu-filter-madgwick \
                 ros-jazzy-xacro python3-serial python3-smbus i2c-tools
sudo usermod -aG dialout,i2c $USER        # re-login
i2cdetect -y 1                            # should show 68
cd ws && colcon build --symlink-install && source install/setup.bash
```

## 1. Servo IDs (once, ONE servo connected at a time — new servos are all ID 1)

```bash
ros2 run biped_control st3215_tool --port /dev/ttyAMA0 scan
ros2 run biped_control st3215_tool --port /dev/ttyAMA0 set-id 1 4
```

## 2. Wiring: RPi 5 UART -> bus servo adapters (jumper on B / UART)

| | left leg (ID 1-3) `/dev/ttyAMA0` | right leg (ID 4-6) `/dev/ttyAMA2` |
|---|---|---|
| RPi TX -> adapter RX | pin 8 (GPIO14) | pin 7 (GPIO4) |
| RPi RX <- adapter TX | pin 10 (GPIO15) | pin 29 (GPIO5) |
| GND | pin 6 | pin 9 |

One-time (frees UART0 from the serial console, enables UART2):

```bash
ssh -t rpi 'sudo sed -i "s/console=serial0,115200 //" /boot/firmware/cmdline.txt \
  && sudo systemctl mask --now serial-getty@ttyAMA0.service \
  && echo "dtoverlay=uart2-pi5" | sudo tee -a /boot/firmware/config.txt && sudo reboot'
```

USB adapters instead: `udev/setup_udev.sh` + `left_port:=/dev/biped_left right_port:=/dev/biped_right`.

## 3. Zero calibration

Put the robot in the URDF zero pose (legs straight, feet flat), then:

```bash
ros2 run biped_control st3215_tool --port /dev/ttyAMA0  set-middle 1 2 3
ros2 run biped_control st3215_tool --port /dev/ttyAMA2 set-middle 4 5 6
```

Check directions: `ros2 launch biped_control bringup.launch.py gait:=false imu:=false rviz:=true`,
then `ros2 control set_hardware_component_state BipedSystem inactive` (torque off), and move legs by hand.
If RViz moves the opposite way, flip `sign` for that joint in `biped_description/urdf/biped.ros2control`.

## 4. Walk

```bash
ros2 launch biped_control bringup.launch.py              # stands up in 2 s
ros2 param set /gait walk true
ros2 param set /gait step_amp 0.2                        # live tuning: freq, roll_amp, phase_deg, trim, ...
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

Laptop without hardware: `ros2 launch biped_control bringup.launch.py use_mock:=true imu:=false rviz:=true`.

Safety (gait_node): tilt > `fall_angle_deg` (40°) or e-stop → walking stops and servo targets follow the
measured positions (servos go limp, no fighting the ground).

```bash
ros2 service call /gait/estop std_srvs/srv/Trigger     # stop now
ros2 service call /gait/resume std_srvs/srv/Trigger    # stand up smoothly from the current pose
```

## Driving the real robot (laptop)

```bash
scripts/robot.sh start-ro        # torque OFF, joint_states + IMU only (checks, calibration)
scripts/robot.sh rviz            # RViz showing the real joints
scripts/robot.sh start           # torque ON, stands up (gait_filed10mm.yaml)
scripts/robot.sh walk | stop
scripts/robot.sh teleop          # i forward, j/l turn, k stop (safe ranges clipped in gait_node)
scripts/robot.sh estop | resume
scripts/robot.sh set step_amp 0.15
scripts/robot.sh status | logs | down
```

### Navigation state machine (nav_fsm)
`scripts/robot.sh start gait_filed10mm.yaml nav:=true` (sim: `scripts/sim.sh ... nav:=true`)

- `IDLE → TURN ⇄ SETTLE → WALK → IDLE`, plus `OBSTACLE` (camera) and `HALTED` (fall / e-stop).
- Polar goal: `scripts/robot.sh goal 0.5 45` (r = 0.5 m, θ = +45° = left) → topic `/goal_polar`
  (`geometry_msgs/Point`, x = r, y = θ in degrees).
- Camera gestures (default `gesture_source:=array`): ONE topic `/camera/goal` (`gesture_topic:=...`, teammates' robot_controller.py, Int8MultiArray, 20 Hz)
  with 5 values `[forward, back, left, right, stop]`, exactly one 1, e.g. `0,0,0,1,0` = right; type auto-detected
  (Int*/Float*MultiArray or String "0,1,0,0,0"/"01000"); all zeros = nothing, two 1s = stop. Publish ~10 Hz.
  (Alternative `gesture_source:=topics`: 5 `std_msgs/Bool` topics `/camera/forward` ... `/camera/stop`.)
  `gesture_mode:=hold`: moves while true, stops when all false or the topics go silent > 1 s;
  priority stop > back > forward > left > right; `back` = stop. `gesture_mode:=step`: one 0.3 m step / 45° turn
  per new gesture. Old Flask JSON still possible with `gesture_source:=http`.
- **Same `ROS_DOMAIN_ID` as the camera nodes** (compose default 42; set `ROS_DOMAIN_ID=...` before `docker compose up`).
- No odometry: distance = speed × time (`walk_speed`, calibrate), heading from the IMU.
- Turning is weak on filed feet (no hip yaw): turns in chunks (8 s turn, stop, 2.5 s settle, re-measure),
  right turns fall back to "left the long way" if they make no progress (`turn_mode`).

### First powered run — checklist
1. Body + IMU mounted. `start-ro`, robot standing straight on the table: `status` → IMU roll/pitch ≈ 0.
   If not, fix `rpy` of `imu_link` (biped.ros2control).
2. Re-zero the hips (body changes their zero): robot in zero pose →
   `st3215_tool --port /dev/ttyAMA0 set-middle 1` and `--port /dev/ttyAMA2 set-middle 4`.
3. `rviz`, move each joint by hand: model must follow the real leg (signs).
4. **Hang the robot (feet off the ground)**, one hand on `scripts/robot.sh estop`.
5. `start` → it moves to the stand pose over 2 s. Check: no buzzing, servo temp ok (`st3215_tool scan`).
6. Half amplitude first: `set step_amp 0.12`, `set roll_amp 0.04`, `walk`. Watch it, `stop`.
7. Full gait in the air: `set step_amp 0.25`, `set roll_amp 0.08`, `walk`.
8. On the floor, hand around the robot: `walk`; tune `turn` (heading trim) so it goes straight.
9. IMU feedback is on (`k_roll -1.0`, `k_pitch 0.35`). If it oscillates/gets worse: `set k_roll 0`, `set k_pitch 0`
   → IMU axes/signs are wrong.

## MuJoCo simulation (laptop)

`biped_sim` builds a MuJoCo model straight from `biped.xacro` and stands in for the real hardware
(same topics: `position_controller/commands` in, `joint_states` + `imu/data` out), so the same `gait_node` runs on it.

```bash
pip install --user --break-system-packages mujoco      # once
scripts/sim.sh                                         # viewer + gait (fixes snap VS Code env)
ros2 param set /gait walk true
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
ros2 service call /sim/reset std_srvs/srv/Trigger
```

Soles: `sole_size:="[x, y, thick]"` flat box (default 4×4.5 cm), `sole_size:="[R, width]"` rocker
curved front-back, `flat_sole:=false` raw CAD meshes.

Findings:
- **The CAD feet have a dome bottom (R ≈ 1.7 cm, CoM at 15.5 cm)** — unstable from any small perturbation,
  0/54 gaits survive. RL (PPO, 3.4M steps, `rl/`) did not beat the plain reference either: physics, not tuning.
- A curved sole is stable only if **R > CoM height (15.5 cm)**. Rocker sole curved front-back, **R = 20 cm,
  width 4.5 cm, length ≥ 6 cm**: stands 5/5 from perturbed starts, 51/54 gaits stable, default gait 5.6 cm/s,
  robust to kp/torque/friction/mass. Turning by step-length difference does **not** work on it (rolls straight).
- Flat 4×4.5 cm soles: 43/54 gaits, ~4.5 cm/s, turning via `cmd_vel` works.
- **Servo model** (`biped_sim/servo.py`, on by default): measured on the real ST3215
  (`scripts/servo_test.py`): ~80 ms command→motion in the ROS loop, deadband ~0.4°, `acc/speed 0`
  is the fastest setting. Sim: 35 ms dead time + 15 ms lag + 0.46° deadband on top of the actuator.
  With it: filed-feet gait + IMU feedback survive 81 % of push trials (33 % without IMU);
  step-length steering does **not** work on filed feet (always ~−3°/s right curve) → `turn` 0,
  range −0.1…0.05; turning is left to the RL policy.
- **CAD feet filed flat** (`flat_sole:=false foot_cut:=<m>`): stands from 4 mm (patch 2.3 cm),
  **10 mm recommended** (patch ~3.4×3.4 cm) with `config/gait_filed10mm.yaml`: ~5.4 cm/s, robust to kp 4–15,
  friction 0.5, mass ×1.3; `turn` only trims heading (≈ −0.15 … +0.3).
  Robot: `ros2 launch biped_control bringup.launch.py gait_params:=.../gait_filed10mm.yaml`
- Default gait (`freq 1.2, roll_amp 0.10, step_amp 0.25, phase_deg -90`) walks ~4.5 cm/s and survived
  kp 4–15, 1.5 Nm torque limit, friction 0.5, mass ×1.5 and 0–30 ms command delay.
- Changes of `forward`/`turn` are low-passed (`cmd_tau`), abrupt amplitude changes mid-step tip it over.

## Gait

Feedforward by default — the trajectory is fixed, no sensors needed:

- **sines** (default): hip roll rocks the weight onto one foot, hip pitch swings the other leg
  (`phase_deg` behind), ankle = −hip pitch keeps the sole flat.
- **CSV** (`trajectory_file:=/abs/path.csv`): any joint trajectory computed offline (e.g. Crocoddyl/Pinocchio
  with the URDF masses), played in a loop. Format: `ws/src/biped_control/trajectories/example_sway.csv`.

IMU feedback is added on top (`k_roll`, `k_pitch`, default 0). Zero the reference while standing:
`ros2 service call /gait/zero_imu std_srvs/srv/Trigger`.
