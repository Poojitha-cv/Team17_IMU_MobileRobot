#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import csv
import os
from datetime import datetime


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


class ImuReader(Node):

    def __init__(self):
        super().__init__('imu_reader')

        self.declare_parameter('alpha', 0.2)
        self.declare_parameter('log_file', '~/imu_logs/imu_log.csv')

        alpha    = self.get_parameter('alpha').value
        log_path = os.path.expanduser(
            self.get_parameter('log_file').value)

        os.makedirs(os.path.dirname(log_path), exist_ok=True)

        # 9 independent LPF channels
        self.f_roll  = LowPassFilter(alpha)
        self.f_pitch = LowPassFilter(alpha)
        self.f_yaw   = LowPassFilter(alpha)
        self.f_lx    = LowPassFilter(alpha)
        self.f_ly    = LowPassFilter(alpha)
        self.f_lz    = LowPassFilter(alpha)
        self.f_ax    = LowPassFilter(alpha)
        self.f_ay    = LowPassFilter(alpha)
        self.f_az    = LowPassFilter(alpha)

        self.csv_file = open(log_path, 'w', newline='')
        self.writer   = csv.writer(self.csv_file)
        self.writer.writerow([
            'timestamp',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z'
        ])

        # log to CSV every 10 messages (5Hz) — not every message
        self.msg_count    = 0
        self.LOG_EVERY    = 10
        # print to terminal once per second
        self.PRINT_EVERY  = 50

        self.sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        self.watchdog      = self.create_timer(2.0, self.watchdog_callback)
        self.last_msg_time = self.get_clock().now()

        self.get_logger().info(f'ImuReader started — logging to {log_path}')

    def imu_callback(self, msg):
        self.last_msg_time = self.get_clock().now()

        q = msg.orientation
        r_raw, p_raw, y_raw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        roll  = math.degrees(self.f_roll.update(r_raw))
        pitch = math.degrees(self.f_pitch.update(p_raw))
        yaw   = math.degrees(self.f_yaw.update(y_raw))

        lx = self.f_lx.update(msg.linear_acceleration.x)
        ly = self.f_ly.update(msg.linear_acceleration.y)
        lz = self.f_lz.update(msg.linear_acceleration.z)

        ax = self.f_ax.update(msg.angular_velocity.x)
        ay = self.f_ay.update(msg.angular_velocity.y)
        az = self.f_az.update(msg.angular_velocity.z)

        self.msg_count += 1

        # write to CSV at 5Hz
        if self.msg_count % self.LOG_EVERY == 0:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            self.writer.writerow([
                ts,
                f'{roll:.4f}', f'{pitch:.4f}', f'{yaw:.4f}',
                f'{lx:.4f}', f'{ly:.4f}', f'{lz:.4f}',
                f'{ax:.4f}', f'{ay:.4f}', f'{az:.4f}'
            ])
            self.csv_file.flush()

        # print to terminal at 1Hz
        if self.msg_count >= self.PRINT_EVERY:
            self.msg_count = 0
            self.get_logger().info(
                f'[ImuReader] roll={roll:.2f}° pitch={pitch:.2f}° yaw={yaw:.2f}° '
                f'| lin=({lx:.3f},{ly:.3f},{lz:.3f}) '
                f'| ang=({ax:.3f},{ay:.3f},{az:.3f})')

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().error(
                f'No /imu/data for {elapsed:.1f}s — check IMU!')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
