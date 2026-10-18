"""MPU6050 over I2C -> sensor_msgs/Imu on imu/data_raw (feed into imu_filter_madgwick)."""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    from smbus2 import SMBus
except ImportError:  # python3-smbus from apt has the same API
    from smbus import SMBus

PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B

G = 9.80665
ACCEL_LSB = 8192.0         # +-4 g
GYRO_LSB = 65.5            # +-500 deg/s


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

        self.bus = SMBus(self.get_parameter('bus').value)
        self.addr = self.get_parameter('address').value
        self.frame_id = self.get_parameter('frame_id').value

        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)   # wake, PLL gyro X clock
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)       # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x08)  # +-500 dps
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x08)  # +-4 g

        self.gyro_bias = self.calibrate_gyro(self.get_parameter('calib_samples').value)
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

    def read_raw(self):
        d = self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14)
        acc = [s16(d[0], d[1]), s16(d[2], d[3]), s16(d[4], d[5])]
        gyr = [s16(d[8], d[9]), s16(d[10], d[11]), s16(d[12], d[13])]
        return acc, gyr

    def calibrate_gyro(self, n):
        self.get_logger().info(f'Gyro calibration ({n} samples) - keep the robot still')
        s = [0.0, 0.0, 0.0]
        for _ in range(n):
            _, g = self.read_raw()
            s = [a + b for a, b in zip(s, g)]
        bias = [v / n for v in s]
        self.get_logger().info(f'Gyro bias (raw): {[round(b, 1) for b in bias]}')
        return bias

    def tick(self):
        try:
            acc, gyr = self.read_raw()
        except OSError as e:
            self.get_logger().warn(f'I2C read failed: {e}', throttle_duration_sec=1.0)
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
        m.orientation_covariance[0] = -1.0  # no orientation in raw data
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
