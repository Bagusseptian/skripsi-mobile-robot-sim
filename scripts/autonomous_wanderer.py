#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Mulus (Smooth Continuous Clearance-Seeking & Damped Centering Explorer).
Dirancang untuk pemetaan komprehensif 100% tanpa osilasi/oleng di lorong dan tanpa berputar di tempat.

Fitur:
1. Damped Proportional Centering with Deadband (0.12m): Robot melaju lurus sempurna di tengah lorong.
2. Dynamic Gap Seeking (9-Sector Arc): Memilih bukaan ruangan paling luas secara kontinu dan mulus.
3. Smooth Steering Filter: Filter low-pass pada kecepatan sudut untuk menghilangkan gerakan patah-patah.
4. Distance-Based Memory (Cumulative Trajectory): Hanya mencatat jejak saat berpindah >0.75m.
5. Adaptive Deceleration: Melaju 0.24 m/s di jalan bebas, melambat anggun ke 0.08 m/s saat mendekati sudut.
"""

import sys
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

class AutonomousWanderer(Node):
    def __init__(self):
        super().__init__('autonomous_wanderer')

        # Publisher kecepatan
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        if self.bridge:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Status Sensor
        self.sector_ranges = np.full(9, 5.0)  # 9 sektor dari -80° s.d. +80°
        self.sector_angles = np.linspace(-math.radians(70), math.radians(70), 9)
        self.left_side_dist = 5.0
        self.right_side_dist = 5.0
        self.min_front_dist = 5.0
        self.min_depth_cam_dist = 5.0

        # State Odometri & Memori Jarak
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_logged_x = 0.0
        self.last_logged_y = 0.0
        self.total_dist_traveled = 0.0
        self.visited_cells = {}  # (cx, cy) -> count
        self.cell_size = 1.0     # Grid memori 1.0m

        # Filter Kecepatan Halus
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # State Machine Sederhana untuk Escape
        self.is_escaping = False
        self.escape_phase = 0    # 1: mundur lurus, 2: putar ke arah terbuka
        self.escape_timer = 0
        self.escape_turn_dir = 1.0

        # Loop Kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*65)
        print("  AUTONOMOUS WANDERER V3 (SMOOTH CONTINUOUS GAP SEEKING)")
        print("="*65)
        print("  Status   : Aktif & Mulus!")
        print("  Kontrol  : Damped Centering + 9-Sector Clearance Stream")
        print("  Navigasi : Tanpa Osilasi, Lurus Rapi, 100% Full Ruangan")
        print("="*65 + "\n", flush=True)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = pos.x
        self.robot_y = pos.y

        # Konversi quaternion ke yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Catat sel HANYA jika berpindah > 0.75 meter
        dist_moved = math.hypot(self.robot_x - self.last_logged_x, self.robot_y - self.last_logged_y)
        if dist_moved >= 0.75:
            self.total_dist_traveled += dist_moved
            self.last_logged_x = self.robot_x
            self.last_logged_y = self.robot_y
            cx = int(round(self.robot_x / self.cell_size))
            cy = int(round(self.robot_y / self.cell_size))
            self.visited_cells[(cx, cy)] = self.visited_cells.get((cx, cy), 0) + 1

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        if num_readings == 0:
            return

        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)
        step = (msg.angle_max - msg.angle_min) / num_readings
        idx_zero = int(-msg.angle_min / step) if step != 0 else num_readings // 2

        # 1. Bagi busur depan [-70°, +70°] ke dalam 9 sektor
        arc_half = math.radians(70)
        idx_span_arc = int(arc_half / step)
        start_idx = max(0, idx_zero - idx_span_arc)
        end_idx = min(num_readings, idx_zero + idx_span_arc)
        
        front_arc = ranges[start_idx:end_idx]
        if len(front_arc) > 9:
            splits = np.array_split(front_arc, 9)
            self.sector_ranges = np.array([float(np.min(s)) for s in splits])
        
        # 2. Sektor depan tengah [-20°, +20°]
        idx_20 = int(math.radians(20) / step)
        c_left = ranges[idx_zero: min(num_readings, idx_zero + idx_20)]
        c_right = ranges[max(0, idx_zero - idx_20): idx_zero]
        center = np.concatenate([c_left, c_right]) if len(c_left) and len(c_right) else ranges
        self.min_front_dist = float(np.min(center)) if len(center) > 0 else 5.0

        # 3. Dinding samping: Kiri [+50°, +85°] dan Kanan [-85°, -50°]
        idx_50 = int(math.radians(50) / step)
        idx_85 = int(math.radians(85) / step)
        side_l = ranges[min(num_readings, idx_zero + idx_50): min(num_readings, idx_zero + idx_85)]
        side_r = ranges[max(0, idx_zero - idx_85): max(0, idx_zero - idx_50)]
        self.left_side_dist = float(np.mean(side_l)) if len(side_l) > 0 else 5.0
        self.right_side_dist = float(np.mean(side_r)) if len(side_r) > 0 else 5.0

    def depth_callback(self, msg: Image):
        try:
            if '32F' in msg.encoding:
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_img = depth_raw.astype(np.float32) / 1000.0

            h, w = depth_img.shape
            ch, cw = h // 2, w // 2
            center_crop = depth_img[max(0, ch-30): min(h, ch+30), max(0, cw-35): min(w, cw+35)]
            valid = center_crop[np.isfinite(center_crop) & (center_crop > 0.1)]
            if len(valid) > 0:
                self.min_depth_cam_dist = float(np.min(valid))
            else:
                self.min_depth_cam_dist = 5.0
        except Exception:
            pass

    def control_loop(self):
        cmd = Twist()
        effective_front = min(self.min_front_dist, self.min_depth_cam_dist)

        # 1. State: EMERGENCY ESCAPE (Jika terdesak < 0.35m di sudut dinding)
        if self.is_escaping:
            if self.escape_phase == 1:
                # Mundur lurus selama 1.2 detik
                cmd.linear.x = -0.12
                cmd.angular.z = 0.0
                self.escape_timer -= 1
                if self.escape_timer <= 0:
                    self.escape_phase = 2
                    self.escape_timer = 12 # Putar 1.2 detik
            elif self.escape_phase == 2:
                # Putar badan ke arah lebih terbuka
                cmd.linear.x = 0.02
                cmd.angular.z = self.escape_turn_dir * 0.45
                self.escape_timer -= 1
                if self.escape_timer <= 0:
                    self.is_escaping = False
            self.cmd_pub.publish(cmd)
            return

        if effective_front < 0.35:
            self.is_escaping = True
            self.escape_phase = 1
            self.escape_timer = 12 # Mundur 1.2 detik
            self.escape_turn_dir = 1.0 if self.left_side_dist > self.right_side_dist else -1.0
            cmd.linear.x = -0.12
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 2. Evaluasi Sektor Bukaan (Gap Seeking)
        # Sektor 0 s.d 8: Sektor 4 adalah tengah (0 rad), sektor 0 kanan (-70°), sektor 8 kiri (+70°)
        # Berikan penalti ke sektor yang terlalu dekat rintangan (< 0.6m)
        clearances = np.copy(self.sector_ranges)
        
        # Bias memori: Jika sudah jalan > 10m, berikan prioritas ke sektor yang mengarah ke area belum terjamah
        if self.total_dist_traveled > 10.0:
            for i, ang in enumerate(self.sector_angles):
                proj_yaw = self.robot_yaw + ang
                look_dist = min(2.0, clearances[i])
                px = self.robot_x + look_dist * math.cos(proj_yaw)
                py = self.robot_y + look_dist * math.sin(proj_yaw)
                cx = int(round(px / self.cell_size))
                cy = int(round(py / self.cell_size))
                visits = self.visited_cells.get((cx, cy), 0)
                # Beri bonus clearance jika sel itu belum pernah dikunjungi
                if visits == 0:
                    clearances[i] += 1.5
                elif visits >= 4:
                    clearances[i] *= 0.6

        best_sector = int(np.argmax(clearances))
        best_angle = self.sector_angles[best_sector]
        best_clearance = clearances[best_sector]

        # 3. Hitung Koreksi Centering Lorong (Damped PD Controller)
        # Jika kedua dinding kiri dan kanan terlihat (< 1.2m), robot berada di lorong!
        omega_center = 0.0
        in_corridor = (self.left_side_dist < 1.1) and (self.right_side_dist < 1.1)
        if in_corridor:
            lateral_error = self.left_side_dist - self.right_side_dist
            # DEADBAND: Jika selisih < 0.12m, robot sudah cukup di tengah, setir lurus!
            if abs(lateral_error) > 0.12:
                omega_center = np.clip(0.35 * lateral_error, -0.22, 0.22)

        # 4. Penentuan Kecepatan Maju (Linear X) & Kecepatan Sudut (Angular Z)
        target_linear_x = 0.0
        target_angular_z = 0.0

        if effective_front > 1.2:
            # Jalan bebas luas: Maju stabil
            target_linear_x = 0.24
            if in_corridor:
                # Di lorong, utamakan lurus di tengah
                target_angular_z = omega_center
            else:
                # Di ruang terbuka, arahkan halus ke sektor paling lapang
                target_angular_z = np.clip(0.35 * best_angle, -0.30, 0.30)
        elif effective_front > 0.60:
            # Mulai mendekati rintangan / sudut: Deselerasi halus proporsional
            ratio = (effective_front - 0.60) / 0.60
            target_linear_x = 0.08 + 0.14 * ratio
            # Belok halus ke arah celah terbuka terbaik
            target_angular_z = np.clip(0.55 * best_angle, -0.45, 0.45)
        else:
            # Jarak dekat (< 0.60m): Rem lambat dan putar badan tegas ke arah celah
            target_linear_x = 0.03
            turn_sign = 1.0 if best_angle >= 0 else -1.0
            target_angular_z = turn_sign * 0.42

        # 5. Low-Pass Filter untuk Kelembutan Gerak (Mencegah Oleng)
        self.current_linear_x = 0.7 * self.current_linear_x + 0.3 * target_linear_x
        self.current_angular_z = 0.65 * self.current_angular_z + 0.35 * target_angular_z

        cmd.linear.x = float(self.current_linear_x)
        cmd.angular.z = float(self.current_angular_z)
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = AutonomousWanderer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
