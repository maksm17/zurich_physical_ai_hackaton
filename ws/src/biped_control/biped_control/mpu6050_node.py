"""MPU6050 over I2C -> sensor_msgs/Imu on imu/data_raw (feed into imu_filter_madgwick).

The MPU6050 can lock up the I2C bus (holds SDA) after a glitch; only a power cycle helps.
With power_gpio >= 0 the board is powered from that GPIO (wire VCC to it, e.g. GPIO17 = pin 11)
and the node power-cycles it automatically when reads keep failing.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:
    from smbus import SMBus

PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

G = 9.80665
ACCEL_LSB = 8192.0
GYRO_LSB = 65.5


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu6050Node(Node):

    def __init__(self):
        super().__init__('mpu6050')
        self.declare_parameter('bus', 1)
        self.declare_parameter('address', 0x68)
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calib_samples', 300)
        self.declare_parameter('power_gpio', -1)
        self.declare_parameter('gpio_chip_label', 'pinctrl-rp1')
        self.declare_parameter('fail_cycles_reset', 20)

        self.bus = SMBus(self.get_parameter('bus').value)
        self.addr = self.get_parameter('address').value
        self.frame_id = self.get_parameter('frame_id').value
        self.errors = 0
        self.power = None
        self.setup_power()

        while rclpy.ok():
            try:
                self.configure()
                break
            except OSError as e:
                self.get_logger().error(f'MPU6050 not answering on I2C ({e}) - '
                                        + ('power-cycling it' if self.power else
                                           'check wiring, retrying in 1 s'),
                                        throttle_duration_sec=5.0)
                self.power_cycle() if self.power else time.sleep(1.0)

        self.gyro_bias = self.calibrate_gyro(self.get_parameter('calib_samples').value)
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def setup_power(self):
        pin = self.get_parameter('power_gpio').value
        if pin < 0:
            return
        try:
            import glob
            import gpiod
            label = self.get_parameter('gpio_chip_label').value
            path = next(p for p in sorted(glob.glob('/dev/gpiochip*'))
                        if gpiod.Chip(p).label() == label)
            line = gpiod.Chip(path).get_line(pin)
            line.request(consumer='mpu6050-power', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[1])
            self.power = line
            time.sleep(0.2)
            self.get_logger().info(f'MPU6050 powered from GPIO{pin} ({path}): auto power-cycle on')
        except Exception as e:
            self.get_logger().error(f'cannot drive GPIO{pin} for MPU power ({e}); no auto reset')
            self.power = None

    def power_cycle(self):
        self.power.set_value(0)
        time.sleep(0.5)
        self.power.set_value(1)
        time.sleep(0.2)
        self.resets = getattr(self, 'resets', 0) + 1
        self.get_logger().warn(f'MPU6050 power-cycled ({self.resets}x)')

    def configure(self):
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x08)
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x08)

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        acc = [s16(d[0], d[1]), s16(d[2], d[3]), s16(d[4], d[5])]
        gyr = [s16(d[8], d[9]), s16(d[10], d[11]), s16(d[12], d[13])]
        return acc, gyr

    def calibrate_gyro(self, n):
        self.get_logger().info(f'Gyro calibration ({n} samples) - keep the robot still')
        s, ok = [0.0, 0.0, 0.0], 0
        for _ in range(n):
            try:
                _, g = self.read_raw()
            except OSError:
                continue
            s = [a + b for a, b in zip(s, g)]
            ok += 1
        bias = [v / max(ok, 1) for v in s]
        self.get_logger().info(f'Gyro bias (raw): {[round(b, 1) for b in bias]}')
        return bias

    def tick(self):
        try:
            acc, gyr = self.read_raw()
        except OSError as e:
            self.errors += 1
            self.get_logger().warn(f'I2C read failed ({self.errors}x): {e}', throttle_duration_sec=1.0)
            if self.errors % self.get_parameter('fail_cycles_reset').value == 0:
                if self.power:
                    self.power_cycle()
                try:
                    self.configure()
                except OSError:
                    pass
            return
        self.errors = 0
        a_g = math.sqrt(sum(v * v for v in acc)) / ACCEL_LSB
        if not 0.3 < a_g < 3.0 or max(abs(v) for v in gyr) >= 32767 or len(set(acc + gyr)) == 1:
            self.rejected = getattr(self, 'rejected', 0) + 1
            self.get_logger().warn(f'implausible IMU sample dropped (|a|={a_g:.2f} g, '
                                   f'{self.rejected} so far) - check wiring', throttle_duration_sec=2.0)
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.linear_acceleration.x = acc[0] / ACCEL_LSB * G
        m.linear_acceleration.y = acc[1] / ACCEL_LSB * G
        m.linear_acceleration.z = acc[2] / ACCEL_LSB * G
        k = math.radians(1.0) / GYRO_LSB
        m.angular_velocity.x = (gyr[0] - self.gyro_bias[0]) * k
        m.angular_velocity.y = (gyr[1] - self.gyro_bias[1]) * k
        m.angular_velocity.z = (gyr[2] - self.gyro_bias[2]) * k
        m.orientation_covariance[0] = -1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(Mpu6050Node())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
