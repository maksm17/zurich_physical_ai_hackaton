"""Navigation state machine: polar goals (r, theta) and camera gestures -> gait primitives.

  IDLE --goal--> TURN <--> SETTLE --heading reached--> WALK --distance reached--> IDLE
  (turning in chunks: turn <= turn_chunk_s, stop, settle, re-measure heading with the IMU)
  WALK --obstacle--> OBSTACLE --clear 1 s--> WALK
  any  --gait halted (fall / e-stop)--> HALTED --robot standing again--> IDLE

Inputs
  /goal_polar   geometry_msgs/Point   x = r [m], y = theta [deg, + = left]
  camera        gesture_source 'array' (default): ONE topic (gesture_array_topic) with 5 values in
                  gesture_order [forward, back, left, right, stop] (teammates' robot_controller.py,
                  /camera/goal Int8MultiArray @20 Hz), exactly one 1, e.g. 0,0,0,1,0 = right. Message type auto-detected: std_msgs/(U)Int8/16/32/64MultiArray,
                  Float32/64MultiArray or String ("0,1,0,0,0" / "01000"). All zeros = no gesture,
                  more than one 1 or wrong length -> stop.
                gesture_source 'topics': std_msgs/Bool on gesture_topics
                  [forward, right, left, back, stop]; priority stop > back > forward > left > right
                  gesture_mode 'hold': move while a gesture is true (MANUAL state), stop when none
                               'step': each new gesture = one goal (gesture_step_m / gesture_turn_deg)
                  back_action 'stop' (no walking backwards on these feet)
                gesture_source 'http': teammates' Flask JSON {"command": FORWARD|BACKWARD|LEFT|
                  RIGHT|NO_FINGERS, "obstacle_detected": bool} at camera_url
                obstacle: topic obstacle_topic (std_msgs/Bool) or the HTTP field
  imu/data      heading (relative yaw), gait/state (halted?)
  nav/cancel    std_srvs/Trigger
Outputs
  gait/leg_gains  [k_left, k_right, roll]  per-leg step gains for gait_node
  nav/state       std_msgs/String

There is no odometry: distance = speed x time; heading from the IMU (drifts slowly, no magnetometer).
Turning is weak on these feet (no hip yaw): in MuJoCo only the left arc gives net rotation
(~2.5 deg/s after the stop transient, the robot partly unwinds when it stops). Hence closed loop on
the IMU heading in chunks; turn_mode 'both' tries the mirrored right arc and falls back to turning
left the long way ('left_only' behaviour) when right turning makes no progress.
"""
import json
import math
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class NavFsm(Node):

    def __init__(self):
        super().__init__('nav_fsm')
        p = self.declare_parameter
        p('rate', 10.0)
        p('walk_speed', 0.06)
        p('arc_speed', 0.037)
        p('prim_forward', [1.0, 1.0])
        p('prim_left', [1.0, 0.5, 0.05])
        p('prim_right', [0.5, 1.0, 0.05])
        p('heading_tol_deg', 10.0)
        p('turn_mode', 'both')
        p('turn_chunk_s', 8.0)
        p('settle_s', 2.5)
        p('max_turn_chunks', 12)
        p('heading_hold_deg', 12.0)
        p('max_distance', 2.0)
        p('gesture_source', 'array')
        p('gesture_array_topic', '/camera/goal')
        p('gesture_order', ['forward', 'back', 'left', 'right', 'stop'])
        p('gesture_topics', ['/camera/forward', '/camera/right', '/camera/left',
                             '/camera/back', '/camera/stop'])
        p('obstacle_topic', '/camera/obstacle')
        p('gesture_mode', 'hold')
        p('gesture_timeout', 1.0)
        p('back_action', 'stop')
        p('camera_url', 'http://127.0.0.1:5000/status')
        p('gesture_hold', 3)
        p('gesture_step_m', 0.3)
        p('gesture_turn_deg', 45.0)
        p('use_obstacle', True)

        self.state = 'IDLE'
        self.yaw = None
        self.gait_state = ''
        self.goal = None
        self.target_yaw = 0.0
        self.dist = 0.0
        self.t_state = time.monotonic()
        self.clear_since = None
        self.chunks = 0
        self.turn_dir = 0
        self.right_failed = False
        self.err_before = None
        self.cam = {'command': 'NO_FINGERS', 'obstacle_detected': False, 'ok': False}
        self.lock = threading.Lock()

        self.legs_pub = self.create_publisher(Float64MultiArray, 'gait/leg_gains', 10)
        self.state_pub = self.create_publisher(String, 'nav/state', 10)
        self.create_subscription(Point, 'goal_polar', self.on_goal, 10)
        self.create_subscription(Imu, 'imu/data', self.on_imu, 10)
        self.create_subscription(String, 'gait/state', self.on_gait_state, 10)
        self.create_service(Trigger, 'nav/cancel', self.on_cancel)
        self.dt = 1.0 / self.get_parameter('rate').value
        self.create_timer(self.dt, self.tick)
        self.gest = {}
        self.manual = None
        src = self.get_parameter('gesture_source').value
        if src == 'http' and self.get_parameter('camera_url').value:
            threading.Thread(target=self.poll_camera, daemon=True).start()
        elif src == 'array':
            self.array_sub = None
            self.create_timer(1.0, self.find_array_topic)
            self.get_logger().info(f'gestures from {self.get_parameter("gesture_array_topic").value} '
                                   f'order {self.get_parameter("gesture_order").value} '
                                   f'({self.get_parameter("gesture_mode").value} mode)')
            self.create_subscription(Bool, self.get_parameter('obstacle_topic').value,
                                     self.on_obstacle, 10)
        elif src == 'topics':
            names = ['forward', 'right', 'left', 'back', 'stop']
            for name, topic in zip(names, self.get_parameter('gesture_topics').value):
                self.create_subscription(Bool, topic, lambda m, k=name: self.on_bool(k, m), 10)
            self.create_subscription(Bool, self.get_parameter('obstacle_topic').value,
                                     self.on_obstacle, 10)
            self.get_logger().info(f'gestures from topics {self.get_parameter("gesture_topics").value}'
                                   f' ({self.get_parameter("gesture_mode").value} mode)')
        self.get_logger().info('nav_fsm ready: /goal_polar (x=r m, y=theta deg) or gestures')

    def on_imu(self, m):
        o = m.orientation
        self.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))

    def on_gait_state(self, m):
        self.gait_state = m.data

    def on_goal(self, m):
        self.start_goal(float(m.x), float(m.y))

    def on_cancel(self, req, res):
        self.set_state('IDLE', 'cancelled')
        res.success, res.message = True, 'cancelled'
        return res

    def on_bool(self, name, m):
        prev = self.gest_active(name)
        self.gest[name] = (bool(m.data), time.monotonic())
        if self.get_parameter('gesture_mode').value == 'step' and m.data and not prev:
            self.on_gesture({'forward': 'FORWARD', 'left': 'LEFT', 'right': 'RIGHT',
                             'back': 'BACKWARD', 'stop': 'BACKWARD'}[name])

    def find_array_topic(self):
        """Subscribe once the gesture topic shows up, with whatever std_msgs type it has."""
        if self.array_sub is not None:
            return
        import std_msgs.msg as sm
        topic = self.get_parameter('gesture_array_topic').value
        types = dict(self.get_topic_names_and_types()).get(topic)
        if not types:
            return
        name = types[0].split('/')[-1]
        cls = getattr(sm, name, None)
        if cls is None or not (name.endswith('MultiArray') or name == 'String'):
            self.get_logger().error(f'{topic}: unsupported type {types[0]}', throttle_duration_sec=10)
            return
        self.array_sub = self.create_subscription(cls, topic, self.on_array, 10)
        self.get_logger().info(f'subscribed to {topic} [{types[0]}]')

    def on_array(self, m):
        if hasattr(m, 'data') and isinstance(m.data, str):
            s = m.data.strip().strip('[]')
            vals = [float(v) for v in (s.replace(' ', ',').split(',') if ',' in s or ' ' in s
                                         else list(s)) if v != '']
        else:
            vals = [float(v) for v in m.data]
        order = self.get_parameter('gesture_order').value
        ones = sum(v > 0.5 for v in vals)
        if len(vals) == len(order) and ones == 0:
            pass
        elif len(vals) != len(order) or ones != 1:
            self.get_logger().warn(f'gesture array {vals}: expected {len(order)} values with '
                                   'exactly one 1 -> stop', throttle_duration_sec=2)
            vals = [1.0 if k == 'stop' else 0.0 for k in order]
        for k, v in zip(order, vals):
            self.on_bool(k, Bool(data=v > 0.5))

    def on_obstacle(self, m):
        with self.lock:
            self.cam = dict(self.cam, obstacle_detected=bool(m.data), ok=True)

    def gest_active(self, name):
        v, t = self.gest.get(name, (False, 0.0))
        return v and time.monotonic() - t < self.get_parameter('gesture_timeout').value

    def hold_gesture(self):
        """Highest-priority active gesture in hold mode, or None."""
        for name in ('stop', 'back', 'forward', 'left', 'right'):
            if self.gest_active(name):
                return name
        return None

    def poll_camera(self):
        url = self.get_parameter('camera_url').value
        last_cmd, count, fired, warned = None, 0, None, False
        while rclpy.ok():
            try:
                with urllib.request.urlopen(url, timeout=0.3) as r:
                    data = json.loads(r.read().decode())
                warned = False
                cmd = data.get('command', 'NO_FINGERS')
                with self.lock:
                    self.cam = dict(data, ok=True)
                count = count + 1 if cmd == last_cmd else 1
                last_cmd = cmd
                if count >= self.get_parameter('gesture_hold').value and cmd != fired:
                    fired = cmd
                    self.on_gesture(cmd)
            except Exception as e:
                with self.lock:
                    self.cam['ok'] = False
                if not warned:
                    self.get_logger().warn(f'camera status unavailable ({url}): {e}')
                    warned = True
            time.sleep(0.2)

    def on_gesture(self, cmd):
        g = lambda n: self.get_parameter(n).value
        self.get_logger().info(f'gesture {cmd}')
        if cmd == 'FORWARD':
            self.start_goal(g('gesture_step_m'), 0.0)
        elif cmd == 'LEFT':
            self.start_goal(0.0, g('gesture_turn_deg'))
        elif cmd == 'RIGHT':
            self.start_goal(0.0, -g('gesture_turn_deg'))
        elif cmd == 'BACKWARD':
            self.set_state('IDLE', 'stop gesture')

    def start_goal(self, r, theta_deg):
        if self.state == 'HALTED':
            self.get_logger().warn('goal ignored: gait halted (resume first)')
            return
        if self.yaw is None:
            self.get_logger().warn('goal ignored: no IMU heading yet')
            return
        r = max(0.0, min(r, self.get_parameter('max_distance').value))
        self.goal = (r, theta_deg)
        self.target_yaw = wrap(self.yaw + math.radians(theta_deg))
        self.dist = 0.0
        self.chunks = 0
        self.right_failed = False
        tol = self.get_parameter('heading_tol_deg').value
        self.turn_dir = 0
        self.set_state('TURN' if abs(theta_deg) > tol else 'WALK',
                       f'goal r={r:.2f} m theta={theta_deg:+.0f} deg')

    def set_state(self, s, why=''):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s} {why}')
        self.state = s
        self.t_state = time.monotonic()
        self.clear_since = None
        if s == 'IDLE':
            self.goal = None

    def send(self, prim):
        self.legs_pub.publish(Float64MultiArray(data=[float(v) for v in prim]))

    def tick(self):
        g = lambda n: self.get_parameter(n).value
        with self.lock:
            cam = dict(self.cam)

        if self.gait_state.startswith('halted') and self.state != 'HALTED':
            self.set_state('HALTED', self.gait_state)
        if (self.get_parameter('gesture_source').value in ('topics', 'array') and
                self.get_parameter('gesture_mode').value == 'hold' and self.state != 'HALTED'):
            gst = self.hold_gesture()
            if gst in ('stop', 'back') and g('back_action') == 'stop' or gst == 'stop':
                if self.state != 'IDLE':
                    self.set_state('IDLE', f'gesture {gst}')
                gst = None
            if gst is not None and self.state != f'MANUAL_{gst.upper()}':
                self.set_state(f'MANUAL_{gst.upper()}', 'gesture')
            elif gst is None and self.state.startswith('MANUAL'):
                self.set_state('IDLE', 'gesture released')
        if self.state.startswith('MANUAL'):
            obstacle = g('use_obstacle') and cam.get('ok') and cam.get('obstacle_detected')
            prim = {'MANUAL_FORWARD': g('prim_forward'), 'MANUAL_LEFT': g('prim_left'),
                    'MANUAL_RIGHT': g('prim_right')}.get(self.state)
            if prim is not None and not (obstacle and self.state == 'MANUAL_FORWARD'):
                self.send(prim)
        elif self.state == 'HALTED':
            if self.gait_state == 'standing':
                self.set_state('IDLE', 'robot standing again')
        elif self.state == 'TURN':
            err = wrap(self.target_yaw - self.yaw)
            if self.turn_dir == 0:
                want_left = err > 0 or g('turn_mode') == 'left_only' or self.right_failed
                self.turn_dir = 1 if want_left else -1
                self.err_before = err
            rem = (err if self.turn_dir > 0 else -err) % (2 * math.pi)
            tol = math.radians(g('heading_tol_deg'))
            reached = rem < tol or rem > 2 * math.pi - tol
            if reached or time.monotonic() - self.t_state > g('turn_chunk_s'):
                self.chunks += 1
                self.set_state('SETTLE', f'chunk {self.chunks}, heading error {math.degrees(err):+.0f} deg')
            else:
                self.send(g('prim_left') if self.turn_dir > 0 else g('prim_right'))
        elif self.state == 'SETTLE':
            if time.monotonic() - self.t_state > g('settle_s'):
                err = wrap(self.target_yaw - self.yaw)
                if self.turn_dir < 0 and abs(err) > abs(self.err_before) - math.radians(2):
                    self.right_failed = True
                    self.get_logger().warn('right turning makes no progress, turning left instead')
                self.turn_dir = 0
                if abs(math.degrees(err)) < g('heading_tol_deg'):
                    self.set_state('WALK' if self.goal[0] > 0.01 else 'IDLE',
                                   f'heading reached ({math.degrees(err):+.0f} deg)')
                elif self.chunks >= g('max_turn_chunks'):
                    self.set_state('IDLE', f'turn failed, heading error {math.degrees(err):+.0f} deg')
                else:
                    self.set_state('TURN', f'continue, heading error {math.degrees(err):+.0f} deg')
        elif self.state == 'WALK':
            if g('use_obstacle') and cam.get('ok') and cam.get('obstacle_detected'):
                self.set_state('OBSTACLE', 'camera obstacle')
            elif self.dist >= self.goal[0]:
                self.set_state('IDLE', f'reached ~{self.dist:.2f} m')
            else:
                err = wrap(self.target_yaw - self.yaw)
                if math.degrees(err) > g('heading_hold_deg'):
                    self.send(g('prim_left'))
                    self.dist += g('arc_speed') * self.dt
                else:
                    self.send(g('prim_forward'))
                    self.dist += g('walk_speed') * self.dt
        elif self.state == 'OBSTACLE':
            if cam.get('obstacle_detected'):
                self.clear_since = None
            else:
                self.clear_since = self.clear_since or time.monotonic()
                if time.monotonic() - self.clear_since > 1.0:
                    self.set_state('WALK', 'obstacle cleared')
        self.state_pub.publish(String(data=self.state if self.goal is None else
                                      f'{self.state} r={self.goal[0]:.2f} '
                                      f'theta={self.goal[1]:+.0f} dist={self.dist:.2f}'))


def main():
    rclpy.init()
    try:
        rclpy.spin(NavFsm())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
