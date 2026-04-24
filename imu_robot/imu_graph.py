#!/usr/bin/env python3
"""
imu_graph.py — Live scrolling graph of IMU readings + estimated robot path.

Subscribes to /imu/data and shows a 4-panel live dashboard:
  - Panel 1: Roll / Pitch / Yaw (degrees)
  - Panel 2: Linear Acceleration X/Y/Z (m/s²)
  - Panel 3: Angular Velocity X/Y/Z (rad/s)
  - Panel 4: Estimated 2-D robot path (dead-reckoning from IMU)

Run after launching imu_demo.launch.py:
    ros2 run imu_robot imu_graph
"""

import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import matplotlib
matplotlib.use('TkAgg')   # change to 'Qt5Agg' if TkAgg is not installed
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ── tunables ──────────────────────────────────────────────────────────────────
WINDOW  = 200    # number of samples visible in time-series plots
DT_NOM  = 0.02   # fallback dt (50 Hz)
ALPHA   = 0.2    # LPF coefficient (matches imu_reader default)
MAX_VEL = 2.0    # velocity clamp for dead-reckoning (m/s)
DECAY   = 0.98   # per-sample velocity decay to limit IMU drift


# ── helpers ───────────────────────────────────────────────────────────────────
def quaternion_to_euler(x, y, z, w):
    """Convert quaternion → (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class LowPassFilter:
    """Simple exponential low-pass filter."""

    def __init__(self, alpha=ALPHA):
        self.alpha = alpha
        self.value = None

    def update(self, raw):
        if self.value is None:
            self.value = raw
        else:
            self.value = self.alpha * raw + (1.0 - self.alpha) * self.value
        return self.value


# ── ROS 2 subscriber node (runs in a background thread) ───────────────────────
class ImuGraphNode(Node):

    def __init__(self):
        super().__init__('imu_graph')

        # 9-channel LPF pipeline (mirrors imu_reader.py)
        self.f_roll  = LowPassFilter()
        self.f_pitch = LowPassFilter()
        self.f_yaw   = LowPassFilter()
        self.f_lx    = LowPassFilter()
        self.f_ly    = LowPassFilter()
        self.f_lz    = LowPassFilter()
        self.f_ax    = LowPassFilter()
        self.f_ay    = LowPassFilter()
        self.f_az    = LowPassFilter()

        # rolling buffers for the three time-series panels
        def _buf():
            return deque([0.0] * WINDOW, maxlen=WINDOW)

        self.buf_roll  = _buf(); self.buf_pitch = _buf(); self.buf_yaw   = _buf()
        self.buf_lx    = _buf(); self.buf_ly    = _buf(); self.buf_lz    = _buf()
        self.buf_ax    = _buf(); self.buf_ay    = _buf(); self.buf_az    = _buf()

        # dead-reckoning state for the 2-D path panel
        self.pos_x   = 0.0
        self.pos_y   = 0.0
        self.vel_x   = 0.0
        self.vel_y   = 0.0
        self.path_x  = deque([0.0], maxlen=2000)
        self.path_y  = deque([0.0], maxlen=2000)

        self.last_time     = None
        self._lock         = threading.Lock()

        self.sub = self.create_subscription(
            Imu, '/imu/data', self._imu_callback, 10)

        self.watchdog      = self.create_timer(2.0, self._watchdog_callback)
        self.last_msg_time = self.get_clock().now()

        self.get_logger().info('imu_graph: waiting for /imu/data …')

    # ── IMU callback ──────────────────────────────────────────────────────────
    def _imu_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        now = self.get_clock().now()

        # compute dt
        if self.last_time is None:
            dt = DT_NOM
        else:
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0.0 or dt > 1.0:
                dt = DT_NOM
        self.last_time = now

        # orientation → Euler (degrees)
        q = msg.orientation
        r_raw, p_raw, y_raw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        roll  = math.degrees(self.f_roll.update(r_raw))
        pitch = math.degrees(self.f_pitch.update(p_raw))
        yaw   = math.degrees(self.f_yaw.update(y_raw))

        # linear acceleration (m/s²)
        lx = self.f_lx.update(msg.linear_acceleration.x)
        ly = self.f_ly.update(msg.linear_acceleration.y)
        lz = self.f_lz.update(msg.linear_acceleration.z)

        # angular velocity (rad/s)
        ax = self.f_ax.update(msg.angular_velocity.x)
        ay = self.f_ay.update(msg.angular_velocity.y)
        az = self.f_az.update(msg.angular_velocity.z)

        # ── dead-reckoning: gravity compensation + world-frame integration ──
        g = 9.81
        heading = math.radians(yaw)

        # subtract gravity component from tilt
        accel_body_x = lx - g * math.sin(math.radians(pitch))
        accel_body_y = ly + g * math.sin(math.radians(roll))

        # rotate body-frame accel → world frame using current yaw
        accel_world_x = (accel_body_x * math.cos(heading)
                         - accel_body_y * math.sin(heading))
        accel_world_y = (accel_body_x * math.sin(heading)
                         + accel_body_y * math.cos(heading))

        # integrate with decay to suppress drift
        self.vel_x = self.vel_x * DECAY + accel_world_x * dt
        self.vel_y = self.vel_y * DECAY + accel_world_y * dt
        self.vel_x = max(-MAX_VEL, min(MAX_VEL, self.vel_x))
        self.vel_y = max(-MAX_VEL, min(MAX_VEL, self.vel_y))

        self.pos_x += self.vel_x * dt
        self.pos_y += self.vel_y * dt

        # push to buffers (thread-safe)
        with self._lock:
            self.buf_roll.append(roll)
            self.buf_pitch.append(pitch)
            self.buf_yaw.append(yaw)
            self.buf_lx.append(lx)
            self.buf_ly.append(ly)
            self.buf_lz.append(lz)
            self.buf_ax.append(ax)
            self.buf_ay.append(ay)
            self.buf_az.append(az)
            self.path_x.append(self.pos_x)
            self.path_y.append(self.pos_y)

    def _watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().error(
                f'No /imu/data for {elapsed:.1f}s — check IMU / Gazebo!')

    def snapshot(self):
        """Thread-safe copy of all buffers for matplotlib."""
        with self._lock:
            return (
                list(self.buf_roll),  list(self.buf_pitch), list(self.buf_yaw),
                list(self.buf_lx),    list(self.buf_ly),    list(self.buf_lz),
                list(self.buf_ax),    list(self.buf_ay),    list(self.buf_az),
                list(self.path_x),    list(self.path_y),
            )


# ── matplotlib live dashboard ─────────────────────────────────────────────────
def run_gui(node: ImuGraphNode):
    """Build and display the 4-panel live dashboard. Blocks until closed."""

    # ── colours ───────────────────────────────────────────────────────────────
    BG     = '#1e1e2e'
    PANEL  = '#2a2a3e'
    GRID   = '#3a3a5e'
    WHITE  = '#cdd6f4'
    LBLCLR = '#aaaacc'

    fig = plt.figure(figsize=(14, 10), facecolor=BG)
    fig.suptitle('IMU Live Dashboard  —  imu_robot (Gazebo)',
                 color=WHITE, fontsize=13, fontweight='bold', y=0.985)

    gs = fig.add_gridspec(4, 1, hspace=0.60,
                          left=0.07, right=0.97, top=0.95, bottom=0.06)

    def _make_ax(row, title, ylabel):
        ax = fig.add_subplot(gs[row])
        ax.set_facecolor(PANEL)
        ax.set_title(title, color='#b4befe', fontsize=9, pad=3, loc='left')
        ax.set_ylabel(ylabel, color=LBLCLR, fontsize=7)
        ax.tick_params(colors=LBLCLR, labelsize=7)
        ax.grid(True, color=GRID, linewidth=0.5, linestyle='--', alpha=0.7)
        for spine in ax.spines.values():
            spine.set_edgecolor(GRID)
        return ax

    xs = list(range(WINDOW))

    # ── Panel 0: Orientation ──────────────────────────────────────────────────
    ax_ori = _make_ax(0, '① Orientation  (Roll / Pitch / Yaw)', 'degrees')
    ln_roll,  = ax_ori.plot(xs, [0]*WINDOW, color='#f38ba8', lw=1.3, label='Roll')
    ln_pitch, = ax_ori.plot(xs, [0]*WINDOW, color='#89dceb', lw=1.3, label='Pitch')
    ln_yaw,   = ax_ori.plot(xs, [0]*WINDOW, color='#f9e2af', lw=1.3, label='Yaw')
    ax_ori.legend(loc='upper right', fontsize=7, facecolor=PANEL,
                  labelcolor=WHITE, framealpha=0.85, ncol=3)

    # ── Panel 1: Linear Acceleration ─────────────────────────────────────────
    ax_lin = _make_ax(1, '② Linear Acceleration  (m/s²)', 'm/s²')
    ln_lx, = ax_lin.plot(xs, [0]*WINDOW, color='#fab387', lw=1.3, label='Lin X')
    ln_ly, = ax_lin.plot(xs, [0]*WINDOW, color='#cba6f7', lw=1.3, label='Lin Y')
    ln_lz, = ax_lin.plot(xs, [0]*WINDOW, color='#a6e3a1', lw=1.3, label='Lin Z')
    ax_lin.legend(loc='upper right', fontsize=7, facecolor=PANEL,
                  labelcolor=WHITE, framealpha=0.85, ncol=3)

    # ── Panel 2: Angular Velocity ─────────────────────────────────────────────
    ax_ang = _make_ax(2, '③ Angular Velocity  (rad/s)', 'rad/s')
    ln_ax2, = ax_ang.plot(xs, [0]*WINDOW, color='#f38ba8', lw=1.3, label='Ang X')
    ln_ay2, = ax_ang.plot(xs, [0]*WINDOW, color='#89b4fa', lw=1.3, label='Ang Y')
    ln_az2, = ax_ang.plot(xs, [0]*WINDOW, color='#94e2d5', lw=1.3, label='Ang Z')
    ax_ang.legend(loc='upper right', fontsize=7, facecolor=PANEL,
                  labelcolor=WHITE, framealpha=0.85, ncol=3)

    # ── Panel 3: 2-D Robot Path ───────────────────────────────────────────────
    ax_path = _make_ax(3, '④ Estimated 2-D Robot Path  (dead-reckoning from IMU)', 'Y (m)')
    ax_path.set_xlabel('X (m)', color=LBLCLR, fontsize=7)
    ax_path.set_aspect('equal', adjustable='datalim')

    path_line,  = ax_path.plot([], [], color='#a6e3a1', lw=1.8, label='Path')
    path_start, = ax_path.plot([0], [0], 'o', color='#f9e2af',
                               ms=9, label='Start', zorder=5)
    path_head,  = ax_path.plot([], [], 's', color='#f38ba8',
                               ms=9, label='Current pos', zorder=6)
    ax_path.legend(loc='upper right', fontsize=7, facecolor=PANEL,
                   labelcolor=WHITE, framealpha=0.85)

    # ── animation update ──────────────────────────────────────────────────────
    def _update(_frame):
        (roll, pitch, yaw,
         lx, ly, lz,
         ax_v, ay_v, az_v,
         px, py) = node.snapshot()

        # update time-series lines
        for ln, data in [
            (ln_roll, roll), (ln_pitch, pitch), (ln_yaw, yaw),
            (ln_lx,   lx),   (ln_ly,   ly),     (ln_lz,  lz),
            (ln_ax2,  ax_v), (ln_ay2,  ay_v),   (ln_az2, az_v),
        ]:
            ln.set_ydata(data)

        # auto-scale y-axis of each time-series panel
        for ax_ts in (ax_ori, ax_lin, ax_ang):
            ax_ts.relim()
            ax_ts.autoscale_view(scalex=False)

        # update path
        if len(px) > 1:
            path_line.set_data(px, py)
            path_head.set_data([px[-1]], [py[-1]])

            # dynamic symmetric zoom with a minimum half-range of 0.5 m
            half = max(0.5,
                       max(abs(v) for v in px),
                       max(abs(v) for v in py))
            half *= 1.25
            ax_path.set_xlim(-half, half)
            ax_path.set_ylim(-half, half)

        return (ln_roll, ln_pitch, ln_yaw,
                ln_lx,   ln_ly,   ln_lz,
                ln_ax2,  ln_ay2,  ln_az2,
                path_line, path_head)

    ani = animation.FuncAnimation(
        fig, _update, interval=100, blit=True, cache_frame_data=False)

    plt.show()
    return ani   # must stay referenced or GC kills the animation


# ── entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ImuGraphNode()

    # ROS spin runs in a daemon thread; GUI runs on main thread (required by Tk/Qt)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        run_gui(node)          # blocks until the window is closed
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
