#!/usr/bin/env bash
set -euo pipefail
usage() {
  cat <<'USAGE'
Drive the real robot from the laptop (everything runs in the 'biped' container on the RPi).

  ROS_DOMAIN_ID=0 scripts/robot.sh start ...   run in another ROS domain (e.g. the camera's)
  scripts/robot.sh start [gait.yaml] [launch args]  bringup with torque, stands up
                                        (default gait_filed10mm.yaml; e.g. ... imu:=false)
  scripts/robot.sh start-rl [policy]   bringup with the RL policy (default plates_v1) instead of gait
  scripts/robot.sh rl-on | rl-off      start / stop the RL policy (fall -> limp; rl-on restarts)
  scripts/robot.sh start-ro            read-only bringup: torque OFF, only joint_states + IMU
  scripts/robot.sh zero                zero calibration: torque off, hold the robot in the zero
                                        pose, Enter -> all 6 servos store it as 2048 (EEPROM)
  scripts/robot.sh walk | stop         start / stop walking
  scripts/robot.sh teleop              keyboard driving (i = forward, j/l = turn, k = stop)
  scripts/robot.sh estop | resume      e-stop (servos soft, torque still on) / stand up again
  scripts/robot.sh torque-off | torque-on  really disable / re-enable servo torque (robot falls!)
  scripts/robot.sh lean-fwd | lean-back [deg]   shift the ankle lean (trim) by 0.3 deg (or [deg])
  scripts/robot.sh set <param> <val>   live tuning, e.g. set step_amp 0.15
  scripts/robot.sh goal <r_m> <theta_deg>   polar goal for nav_fsm (start ... nav:=true)
  scripts/robot.sh nav | nav-cancel    nav_fsm state / cancel the goal
  scripts/robot.sh record | record-stop   rosbag of joints/commands/IMU/states -> bags/ on laptop
  scripts/robot.sh floor-test [s] [steps]  staged gait test with recording: sway 0.03/0.05/0.08, then
                                        steps 0.06/0.12/0.18/0.25 (s seconds each, default 8);
                                        stops on fall, Ctrl+C = e-stop; analysis at the end;
                                        'steps' skips the sway-only stages
  scripts/robot.sh status              hardware, controllers, IMU tilt
  scripts/robot.sh logs | rviz | down
USAGE
}
HOST=${ROBOT_HOST:-rpi}
CFG=/ws/install/biped_control/share/biped_control/config
cd "$(dirname "$0")/.."

ros() {  # run commands inside the running container (sent via stdin: no nested quoting)
  printf 'source /ws/install/setup.bash\n%s\n' "$*" | ssh "$HOST" docker exec -i biped bash
}

case "${1:-}" in
  start)
    ssh "$HOST" "cd zurich && docker compose down >/dev/null 2>&1; \
      ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} LAUNCH_ARGS='gait_params:=$CFG/${2:-gait_filed10mm.yaml} ${*:3}' docker compose up -d"
    echo "Robot will stand up in ~2 s after bringup. Then: scripts/robot.sh walk | teleop" ;;
  zero)
    ros "timeout 5 ros2 service call /gait/estop std_srvs/srv/Trigger >/dev/null 2>&1; sleep 0.3
         ros2 control set_hardware_component_state BipedSystem inactive" >/dev/null 2>&1 || true
    ssh "$HOST" "cd zurich && docker compose down" >/dev/null 2>&1
    echo "Torque OFF. Put the robot in the ZERO pose: body upright, both legs straight, vertical and"
    echo "parallel, feet flat on the floor and level with each other. Hold it still."
    read -rp "Press Enter to store this pose as zero... "
    ssh "$HOST" "cd zurich && \
      docker compose run --rm robot ros2 run biped_control st3215_tool --port /dev/ttyAMA0 set-middle 1 2 3 && \
      docker compose run --rm robot ros2 run biped_control st3215_tool --port /dev/ttyAMA2 set-middle 4 5 6" \
      2>&1 | grep -v Container
    echo "Done (stored in the servos). Next: scripts/robot.sh start" ;;
  start-rl)
    ssh "$HOST" "cd zurich && docker compose down >/dev/null 2>&1; \
      ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} LAUNCH_ARGS='policy:=${2:-plates_v1} ${*:3}' docker compose up -d"
    echo "RL policy loaded, robot stands up. Then: scripts/robot.sh rl-on | rl-off" ;;
  rl-on)  ros ros2 param set /rl_policy enabled true ;;
  rl-off) ros ros2 param set /rl_policy enabled false ;;
  start-ro)
    ssh "$HOST" "cd zurich && docker compose down >/dev/null 2>&1; \
      ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} LAUNCH_ARGS='read_only:=true' docker compose up -d" ;;
  walk)   ros ros2 param set /gait walk true ;;
  stop)   ros ros2 param set /gait walk false ;;
  estop)  ros ros2 service call /gait/estop std_srvs/srv/Trigger ;;
  resume) ros ros2 service call /gait/resume std_srvs/srv/Trigger ;;
  set)    ros "ros2 param set /gait $2 '$3'" ;;   # quoted: arrays like trim '[0,0,-0.03,0,0,-0.03]' work
  lean-fwd|lean-back)   # adjust both ankle trims; negative ankle = lean forward
    step=${2:-0.3}; [ "$1" = "lean-back" ] || step=-$step
    ros "python3 - <<'PY'
import math, subprocess
out = subprocess.run(['ros2', 'param', 'get', '/gait', 'trim'], capture_output=True, text=True).stdout
vals = [float(v) for v in out.split(':', 1)[1].strip().strip('[]').split(',')]
d = math.radians($step)
vals[2] += d; vals[5] += d
subprocess.run(['ros2', 'param', 'set', '/gait', 'trim', str(vals)], capture_output=True)
print(f'ankle trim now {math.degrees(vals[2]):+.2f} deg  ->  trim: {[round(v, 4) for v in vals]}')
PY" ;;
  torque-off)  # e-stop first (gait targets follow the measured pose, so torque-on won't jump),
    ros "timeout 5 ros2 service call /gait/estop std_srvs/srv/Trigger >/dev/null 2>&1; sleep 0.3
         ros2 control set_hardware_component_state BipedSystem inactive" ;;
  torque-on)   # hardware -> active at the current pose (refused if a joint is out of limits)
    ros "ros2 control set_hardware_component_state BipedSystem active
         ros2 control switch_controllers --activate position_controller"
    echo "Torque on, holding the current pose. Stand up smoothly: scripts/robot.sh resume" ;;
  teleop)
    ssh -t "$HOST" "docker exec -it biped bash -c 'source /ws/install/setup.bash && \
      ros2 run teleop_twist_keyboard teleop_twist_keyboard'" ;;
  goal)   ros "ros2 topic pub --once /goal_polar geometry_msgs/msg/Point '{x: $2, y: $3}'" ;;
  nav)    ros "timeout 3 ros2 topic echo --once /nav/state --field data; timeout 3 ros2 topic echo --once /gait/state --field data" ;;
  nav-cancel) ros ros2 service call /nav/cancel std_srvs/srv/Trigger ;;
  record)   # detached recorder in the container (survives this ssh / exec session)
    ros "pkill -INT -f 'bag record' 2>/dev/null; sleep 1; mkdir -p /tmp/bags"
    name=run_$(date +%H%M%S)
    ssh "$HOST" "docker exec -d biped bash -c 'source /ws/install/setup.bash && exec ros2 bag record \
      -o /tmp/bags/$name /joint_states /imu/data /imu/data_raw /position_controller/commands \
      /gait/state /nav/state /cmd_vel /gait/leg_gains > /tmp/bags/$name.log 2>&1'"
    sleep 3
    ros "grep -c 'Subscribed to topic' /tmp/bags/$name.log | sed 's/^/subscribed topics: /'"
    echo "Recording $name. Do the test, then: scripts/robot.sh record-stop" ;;
  record-stop)
    name=$(ros "ls -t /tmp/bags | grep -v log | head -1" | tail -1)
    [ -n "$name" ] || { echo "no recording found"; exit 1; }
    ros "pkill -INT -f 'bag record'; for i in \$(seq 1 30); do [ -f /tmp/bags/$name/metadata.yaml ] && break; sleep 0.5; done; ls /tmp/bags/$name"
    ssh "$HOST" "rm -rf /tmp/bags_out && docker cp biped:/tmp/bags/$name /tmp/bags_out"
    mkdir -p bags && rm -rf "bags/$name" && scp -qr "$HOST:/tmp/bags_out" "bags/$name"
    echo "saved bags/$name"; python3 scripts/analyze_bag.py "bags/$name" || true ;;
  floor-test)
    dur=${2:-8}
    trap 'echo; echo "Ctrl+C -> e-stop"; "$0" estop; "$0" record-stop; exit 1' INT
    "$0" record | tail -1
    stages=("0.03 0.0" "0.05 0.0" "0.08 0.0" "0.08 0.06" "0.08 0.12" "0.08 0.18" "0.08 0.25")
    [ "${3:-}" = "steps" ] && stages=("0.08 0.0" "0.08 0.06" "0.08 0.12" "0.08 0.18" "0.08 0.25")
    for stage in "${stages[@]}"; do
      set -- $stage
      echo "== roll_amp $1  step_amp $2  ($dur s)"
      "$0" set roll_amp "$1" >/dev/null; "$0" set step_amp "$2" >/dev/null; "$0" walk >/dev/null
      end=$((SECONDS + dur)); halted=""
      while [ $SECONDS -lt $end ]; do
        st=$(ros "timeout 2 ros2 topic echo --once /gait/state --field data 2>/dev/null | head -1" 2>/dev/null)
        case "$st" in halted*) halted=$st; break ;; esac
        sleep 1
      done
      if [ -n "$halted" ]; then echo "!! $halted at roll_amp $1 step_amp $2"; break; fi
    done
    "$0" stop >/dev/null; sleep 2
    "$0" record-stop ;;
  status)
    ssh "$HOST" "cd zurich && docker compose logs --since 30s 2>&1 | grep -E 'NOT ANSWERING|NO IMU DATA|I2C read failed' | tail -3 | sed 's/.*]: /!! /'" || true
    ros "ros2 control list_hardware_components | grep -E 'name|state'
         ros2 control list_controllers
         python3 - <<'PY'
import math, time, rclpy
from sensor_msgs.msg import Imu, JointState
rclpy.init(); n = rclpy.create_node('robot_status'); d = {}
n.create_subscription(Imu, 'imu/data', lambda m: d.update(imu=m), 10)
n.create_subscription(JointState, 'joint_states', lambda m: d.update(js=m), 10)
t = time.time()
while time.time() - t < 3 and len(d) < 2: rclpy.spin_once(n, timeout_sec=0.1)
if 'imu' in d:
    o = d['imu'].orientation
    r = math.degrees(math.atan2(2*(o.w*o.x + o.y*o.z), 1 - 2*(o.x*o.x + o.y*o.y)))
    p = math.degrees(math.asin(max(-1, min(1, 2*(o.w*o.y - o.z*o.x)))))
    print(f'IMU  roll {r:+6.1f} deg   pitch {p:+6.1f} deg')
else:
    print('IMU  no data on imu/data')
if 'js' in d:
    js = d['js']
    print('joints [deg] ' + '  '.join(f'{k} {math.degrees(v):+5.1f}' for k, v in sorted(zip(js.name, js.position))))
else:
    print('joints  no data on joint_states')
PY" ;;
  logs)   ssh "$HOST" "cd zurich && docker compose logs -f --tail 50" ;;
  rviz)   exec scripts/rviz_robot.sh "$HOST" ;;
  down)   # torque off first (stopping the container alone may leave the servos holding)
    ros "timeout 5 ros2 service call /gait/estop std_srvs/srv/Trigger >/dev/null 2>&1; sleep 0.3
         ros2 control set_hardware_component_state BipedSystem inactive" 2>/dev/null || true
    ssh "$HOST" "cd zurich && docker compose down" ;;
  *)      usage; exit 1 ;;
esac
