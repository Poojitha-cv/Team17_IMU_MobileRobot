#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class LowPassFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.value = None

    def update(self, raw):
        if self.value is None:
            self.value = raw
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value


class MotionClassifier(Node):

    def __init__(self):
        super().__init__('motion_classifier')

        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('lin_x_threshold', 0.15)
        self.declare_parameter('ang_z_threshold', 0.10)

        alpha        = self.get_parameter('alpha').value
        self.lin_thr = self.get_parameter('lin_x_threshold').value
        self.ang_thr = self.get_parameter('ang_z_threshold').value

        self.lin_filter = LowPassFilter(alpha=alpha)
        self.ang_filter = LowPassFilter(alpha=alpha)

        # velocity estimate via integration
        self.estimated_vel     = 0.0
        self.last_time         = None
        # FIX: decay applied per-second via dt (not per-callback at 50Hz)
        self.VEL_DECAY_PER_SEC = 0.5
        self.VEL_THRESHOLD     = 0.08   # lowered for better sensitivity

        self.current_state    = 'STATIONARY'
        self.pending_state    = 'STATIONARY'
        # FIX: was 250 (5s) — now 10 frames (0.2s), responsive but not jittery
        self.HOLD_FRAMES      = 10
        self.state_hold_count = 0

        self.msg_count   = 0
        self.PRINT_EVERY = 50

        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.watchdog      = self.create_timer(2.0, self.watchdog_callback)
        self.last_msg_time = self.get_clock().now()

        self.get_logger().info(
            f'MotionClassifier started — lin_thr={self.lin_thr} '
            f'ang_thr={self.ang_thr}  hold={self.HOLD_FRAMES} frames (0.2 s)')

    def classify(self, lin_x, ang_z, est_vel):
        moving_linear  = abs(lin_x) > self.lin_thr or abs(est_vel) > self.VEL_THRESHOLD
        moving_angular = abs(ang_z) > self.ang_thr

        if moving_linear and moving_angular:
            return 'MOVING_AND_TURNING'
        elif moving_linear:
            return 'MOVING_STRAIGHT'
        elif moving_angular:
            return 'TURNING'
        else:
            return 'STATIONARY'

    def imu_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        now = self.get_clock().now()

        if self.last_time is None:
            dt = 0.02
        else:
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0.0 or dt > 1.0:
                dt = 0.02
        self.last_time = now

        lin_x = self.lin_filter.update(msg.linear_acceleration.x)
        ang_z = self.ang_filter.update(msg.angular_velocity.z)

        # FIX: decay is now rate-independent (applied as per-second decay via dt)
        decay = self.VEL_DECAY_PER_SEC ** dt
        self.estimated_vel = self.estimated_vel * decay + lin_x * dt

        candidate = self.classify(lin_x, ang_z, self.estimated_vel)

        # FIX: original logic reset hold counter whenever candidate == current_state,
        # meaning the counter only grew when state differed, but reset on any stable
        # frame — so it NEVER reached HOLD_FRAMES and states never transitioned.
        # Correct: track the pending candidate; commit when it persists for HOLD_FRAMES.
        if candidate == self.pending_state:
            self.state_hold_count += 1
            if self.state_hold_count >= self.HOLD_FRAMES and candidate != self.current_state:
                prev               = self.current_state
                self.current_state = candidate
                self.state_hold_count = 0
                self.get_logger().info(
                    f'State change: {prev} -> {self.current_state}  '
                    f'(lin_x={lin_x:.3f} ang_z={ang_z:.3f} est_vel={self.estimated_vel:.3f})')
        else:
            self.pending_state    = candidate
            self.state_hold_count = 1

        self.msg_count += 1
        if self.msg_count >= self.PRINT_EVERY:
            self.msg_count = 0
            self.get_logger().info(
                f'[MotionClassifier] lin_x={lin_x:.3f}  ang_z={ang_z:.3f}  '
                f'est_vel={self.estimated_vel:.3f}  state={self.current_state}  '
                f'pending={self.pending_state}  hold={self.state_hold_count}/{self.HOLD_FRAMES}')

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().error(
                f'No /imu/data for {elapsed:.1f}s — IMU stopped publishing!')


def main(args=None):
    rclpy.init(args=args)
    node = MotionClassifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
