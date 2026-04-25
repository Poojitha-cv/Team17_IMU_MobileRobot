import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math


class LowPassFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.value = None

    def update(self, raw):
        if self.value is None:
            self.value = raw
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class TiltDetector(Node):

    def __init__(self):
        super().__init__('tilt_detector')

        self.declare_parameter('tilt_enter', 15.0)
        self.declare_parameter('tilt_exit',  10.0)
        self.declare_parameter('alpha',       0.2)
        self.declare_parameter('imu_topic',  '/imu/data')

        self.TILT_ENTER = self.get_parameter('tilt_enter').get_parameter_value().double_value
        self.TILT_EXIT  = self.get_parameter('tilt_exit').get_parameter_value().double_value
        alpha           = self.get_parameter('alpha').get_parameter_value().double_value
        imu_topic       = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.roll_filter  = LowPassFilter(alpha=alpha)
        self.pitch_filter = LowPassFilter(alpha=alpha)
        self.tilt_active  = False

        # FIX: 250 frames (5s) was too long — use 5 frames (0.1s) for tilt detection
        # Tilt is a safety condition; it must trigger quickly
        self.HOLD_FRAMES      = 5
        self.tilt_enter_count = 0
        self.tilt_exit_count  = 0

        self.msg_count   = 0
        self.PRINT_EVERY = 50

        self.sub = self.create_subscription(
            Imu, imu_topic, self.imu_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_timer = self.create_timer(0.05, self.stop_callback)

        self.watchdog      = self.create_timer(2.0, self.watchdog_callback)
        self.last_msg_time = self.get_clock().now()

        self.get_logger().info(
            f'TiltDetector ready  ENTER={self.TILT_ENTER}°  EXIT={self.TILT_EXIT}°  '
            f'alpha={alpha}  topic={imu_topic}  hold={self.HOLD_FRAMES} frames (0.1 s)')

    def imu_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        q = msg.orientation
        roll_raw, pitch_raw, _ = quaternion_to_euler(q.x, q.y, q.z, q.w)

        roll_deg  = math.degrees(self.roll_filter.update(roll_raw))
        pitch_deg = math.degrees(self.pitch_filter.update(pitch_raw))
        max_tilt  = max(abs(roll_deg), abs(pitch_deg))

        if not self.tilt_active:
            if max_tilt > self.TILT_ENTER:
                self.tilt_enter_count += 1
                self.tilt_exit_count   = 0
                if self.tilt_enter_count >= self.HOLD_FRAMES:
                    self.tilt_active      = True
                    self.tilt_enter_count = 0
                    self.get_logger().warn(
                        f'[TILT] DETECTED  roll={roll_deg:.1f}°  pitch={pitch_deg:.1f}°  '
                        f'-- ROBOT STOPPED')
            else:
                self.tilt_enter_count = 0
        else:
            if max_tilt < self.TILT_EXIT:
                self.tilt_exit_count  += 1
                self.tilt_enter_count  = 0
                if self.tilt_exit_count >= self.HOLD_FRAMES:
                    self.tilt_active     = False
                    self.tilt_exit_count = 0
                    self.get_logger().info(
                        f'[TILT] Cleared  roll={roll_deg:.1f}°  pitch={pitch_deg:.1f}°  '
                        f'-- ROBOT RESUMED')
            else:
                self.tilt_exit_count = 0

        self.msg_count += 1
        if self.msg_count >= self.PRINT_EVERY:
            self.msg_count = 0
            self.get_logger().info(
                f'[TiltDetector] roll={roll_deg:+.1f}°  pitch={pitch_deg:+.1f}°  '
                f'max={max_tilt:.1f}°  tilt_active={self.tilt_active}')

    def stop_callback(self):
        if self.tilt_active:
            self.cmd_pub.publish(Twist())

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().error(
                f'No /imu/data for {elapsed:.1f}s  --  check Gazebo IMU plugin!')


def main(args=None):
    rclpy.init(args=args)
    node = TiltDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
