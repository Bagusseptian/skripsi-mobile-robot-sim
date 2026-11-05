#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Mulus & Dinamis (Pure Smooth Continuous Wanderer).
Navigasi murni tanpa RTH dan tanpa logika penyelesaian buatan.

Prinsip Kerja:
1. Dynamic Wide Gap-Seeking (13-Sector Clearance Arc dari -95° s.d. +95°):
   Mendeteksi bukaan depan dan celah lorong lateral antar-meja secara kontinu.
2. Heatmap Memory & Dispersion Drive:
   Mencatat frekuensi kunjungan spasial (grid 0.8m). Area yang sudah sering
   dilewati dipenalti, sedangkan bukaan ke area baru (0 kunjungan) diberi
   daya tarik eksplorasi tinggi sehingga robot menjelajahi seluruh ruangan
   secara alami tanpa terjebak looping di satu sisi.
3. Anti-Looping Loop Breaker:
   Jika robot berputar di area lokal (<2.0m) lebih dari 12 detik, robot
   otomatis memicu dispersi ke arah paling jarang dikunjungi.
4. Damped Centering with Exit Allowance:
   Menjaga posisi tengah lorong tanpa menghalangi robot belok ke celah baru.
5. Smooth Motion:
   Low-pass velocity filter menghasilkan gerakan meluncur mulus tanpa patah-patah.
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

        # Publisher kecepatan gerak robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers sensor
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        if self.bridge:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Status Sensor Laser (13 Sektor mencakup -95° s.d. +95°)
        self.num_sectors = 13
        self.sector_ranges = np.full(self.num_sectors, 5.0)
        self.sector_angles = np.linspace(-math.radians(95), math.radians(95), self.num_sectors)
        
        self.left_side_dist = 5.0
        self.right_side_dist = 5.0
        self.min_front_dist = 5.0
        self.min_depth_cam_dist = 5.0

        # Odometri & Memori Eksplorasi Spasial (Heatmap)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_logged_x = 0.0
        self.last_logged_y = 0.0
        self.total_dist_traveled = 0.0
        
        self.cell_size = 0.8  # Grid resolusi 0.8m
        self.visited_cells = {}  # (cx, cy) -> jumlah kunjungan

        # Deteksi Looping & Anti-Stuck
        self.recent_positions = []  # Riwayat posisi terakhir [(time, x, y)]
        self.is_dispersing = False
        self.dispersion_timer = 0
        self.dispersion_target_yaw = 0.0

        # Filter Kecepatan Halus
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # State Emergency Escape (Jika jarak < 0.35m dari rintangan)
        self.is_escaping = False
        self.escape_phase = 0
        self.escape_timer = 0
        self.escape_turn_dir = 1.0

        # Timer Loop Kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*65)
        print("  AUTONOMOUS WANDERER (PURE CONTINUOUS EXPLORER)")
        print("="*65)
        print("  Mode     : Pure Wandering (No RTH, No Fake Stop)")
        print("  Navigasi : 13-Sector Clearance Arc + Heatmap Dispersion")
        print("  Fokus    : Jelajah Alami, Masuk ke Semua Celah, Bebas Looping")
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

        now = time.time()
        self.recent_positions.append((now, self.robot_x, self.robot_y))
        # Pertahankan riwayat 12 detik terakhir
        while self.recent_positions and (now - self.recent_positions[0][0] > 12.0):
            self.recent_positions.pop(0)

        # Catat sel memori spasial setiap berpindah minimal 0.40m
        dist_moved = math.hypot(self.robot_x - self.last_logged_x, self.robot_y - self.last_logged_y)
        if dist_moved >= 0.40:
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

        # Filter nilai tidak valid (NaN, Inf, < range_min)
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)
        step = (msg.angle_max - msg.angle_min) / num_readings
        idx_zero = int(-msg.angle_min / step) if step != 0 else num_readings // 2

        # 1. Busur Depan-Samping Luas: [-95°, +95°] dibagi 13 Sektor
        arc_half = math.radians(95)
        idx_span_arc = int(arc_half / step)
        start_idx = max(0, idx_zero - idx_span_arc)
        end_idx = min(num_readings, idx_zero + idx_span_arc)
        
        front_arc = ranges[start_idx:end_idx]
        if len(front_arc) >= self.num_sectors:
            splits = np.array_split(front_arc, self.num_sectors)
            self.sector_ranges = np.array([np.percentile(s, 25) for s in splits])

        # 2. Jarak Depan Kritis (Kerucut Sempit [-20°, +20°])
        idx_front = int(math.radians(20) / step)
        f_start = max(0, idx_zero - idx_front)
        f_end = min(num_readings, idx_zero + idx_front)
        if f_end > f_start:
            self.min_front_dist = float(np.min(ranges[f_start:f_end]))

        # 3. Jarak Dinding Samping Kiri (+80° s.d. +100°) & Kanan (-100° s.d. -80°)
        idx_side_w = int(math.radians(12) / step)
        idx_left_center = min(num_readings - 1, idx_zero + int(math.radians(90) / step))
        l_s = max(0, idx_left_center - idx_side_w)
        l_e = min(num_readings, idx_left_center + idx_side_w)
        if l_e > l_s:
            self.left_side_dist = float(np.percentile(ranges[l_s:l_e], 25))

        idx_right_center = max(0, idx_zero - int(math.radians(90) / step))
        r_s = max(0, idx_right_center - idx_side_w)
        r_e = min(num_readings, idx_right_center + idx_side_w)
        if r_e > r_s:
            self.right_side_dist = float(np.percentile(ranges[r_s:r_e], 25))

    def depth_callback(self, msg: Image):
        if not self.bridge:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_arr = np.array(cv_img, dtype=np.float32)
            h, w = depth_arr.shape
            center_roi = depth_arr[int(h*0.35):int(h*0.65), int(w*0.35):int(w*0.65)]
            valid_depths = center_roi[(center_roi > 0.15) & ~np.isnan(center_roi) & ~np.isinf(center_roi)]
            if len(valid_depths) > 0:
                self.min_depth_cam_dist = float(np.min(valid_depths))
            else:
                self.min_depth_cam_dist = 5.0
        except Exception:
            self.min_depth_cam_dist = 5.0

    def check_local_loop(self):
        """Mendeteksi apakah robot terjebak berputar di area sempit (<2.0m) selama >10 detik"""
        if len(self.recent_positions) < 60:  # Butuh minimal 6 detik data (pada 10Hz)
            return False
        xs = [p[1] for p in self.recent_positions]
        ys = [p[2] for p in self.recent_positions]
        span_x = max(xs) - min(xs)
        span_y = max(ys) - min(ys)
        # Jika bentang gerak selama 12 detik sangat kecil (< 1.8m), robot terjebak looping
        return (span_x < 1.8) and (span_y < 1.8)

    def control_loop(self):
        cmd = Twist()
        effective_front = min(self.min_front_dist, self.min_depth_cam_dist)

        # -------------------------------------------------------------
        # STATE 1: EMERGENCY ESCAPE (Jika terhimpit rintangan < 0.35m)
        # -------------------------------------------------------------
        if self.is_escaping:
            if self.escape_phase == 1:
                # Mundur lurus selama 1.0 detik
                cmd.linear.x = -0.12
                cmd.angular.z = 0.0
                self.escape_timer -= 1
                if self.escape_timer <= 0:
                    self.escape_phase = 2
                    self.escape_timer = 12  # Putar 1.2 detik
            elif self.escape_phase == 2:
                # Putar badan ke arah yang lebih lapang
                cmd.linear.x = 0.02
                cmd.angular.z = self.escape_turn_dir * 0.48
                self.escape_timer -= 1
                if self.escape_timer <= 0:
                    self.is_escaping = False
            self.cmd_pub.publish(cmd)
            return

        if effective_front < 0.36:
            self.is_escaping = True
            self.escape_phase = 1
            self.escape_timer = 10  # Mundur 1.0 detik
            self.escape_turn_dir = 1.0 if self.left_side_dist > self.right_side_dist else -1.0
            cmd.linear.x = -0.12
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # -------------------------------------------------------------
        # STATE 2: ANTI-LOOPING DISPERSION (Pemecah Siklus Terjebak)
        # -------------------------------------------------------------
        if self.is_dispersing:
            self.dispersion_timer -= 1
            yaw_diff = math.atan2(math.sin(self.dispersion_target_yaw - self.robot_yaw),
                                  math.cos(self.dispersion_target_yaw - self.robot_yaw))
            if abs(yaw_diff) > 0.25 and self.dispersion_timer > 15:
                # Belok tegas menuju haluan dispersi
                cmd.linear.x = 0.05
                cmd.angular.z = np.clip(0.8 * yaw_diff, -0.55, 0.55)
            else:
                # Maju lurus keluar dari area lama
                cmd.linear.x = 0.20
                cmd.angular.z = np.clip(0.4 * yaw_diff, -0.20, 0.20)

            if self.dispersion_timer <= 0 or effective_front < 0.55:
                self.is_dispersing = False
                self.recent_positions.clear()
            self.cmd_pub.publish(cmd)
            return

        if self.check_local_loop():
            # Memicu dorongan keluar ke arah berlawanan atau sektor yang belum dijelajahi
            self.is_dispersing = True
            self.dispersion_timer = 35  # Berlangsung 3.5 detik
            # Balik haluan 120 derajat ke arah sisi yang lebih lapang
            turn_bias = 1.0 if self.left_side_dist > self.right_side_dist else -1.0
            self.dispersion_target_yaw = self.robot_yaw + turn_bias * (math.pi * 0.65)
            self.dispersion_target_yaw = math.atan2(math.sin(self.dispersion_target_yaw),
                                                    math.cos(self.dispersion_target_yaw))
            print(f"[WANDERER] Anti-Looping Terpicu! Mendispersi robot ke arah baru.", flush=True)

        # -------------------------------------------------------------
        # STATE 3: DYNAMIC GAP SEEKING & HEATMAP EXPLORATION DRIVE
        # -------------------------------------------------------------
        clearances = np.copy(self.sector_ranges)

        # Evaluasi proyeksi masa depan untuk setiap sektor (Heatmap Attractiveness)
        for i, ang in enumerate(self.sector_angles):
            proj_yaw = self.robot_yaw + ang
            look_dist = min(2.5, clearances[i])
            px = self.robot_x + look_dist * math.cos(proj_yaw)
            py = self.robot_y + look_dist * math.sin(proj_yaw)
            cx = int(round(px / self.cell_size))
            cy = int(round(py / self.cell_size))
            visits = self.visited_cells.get((cx, cy), 0)

            # Sektor yang mengarah ke area belum pernah dikunjungi diberi BONUS TINGGI
            if visits == 0:
                clearances[i] += 2.8  # Sangat memikat robot untuk mencoba celah baru!
            elif visits == 1:
                clearances[i] += 0.8
            elif visits >= 3:
                clearances[i] *= 0.45  # Menekan koridor yang sudah bolak-balik dilewati

            # Penalti untuk sektor yang terlalu dekat rintangan (< 0.55m)
            if self.sector_ranges[i] < 0.55:
                clearances[i] *= 0.2

        best_sector = int(np.argmax(clearances))
        best_angle = self.sector_angles[best_sector]
        best_clearance = clearances[best_sector]

        # -------------------------------------------------------------
        # 4. DAMPED CENTERING DI LORONG (Hanya jika jalan lurus terbuka)
        # -------------------------------------------------------------
        omega_center = 0.0
        in_corridor = (self.left_side_dist < 1.15) and (self.right_side_dist < 1.15)
        if in_corridor:
            lateral_error = self.left_side_dist - self.right_side_dist
            # Deadband 0.12m: Jika selisih kecil, jangan oleng
            if abs(lateral_error) > 0.12:
                omega_center = np.clip(0.35 * lateral_error, -0.22, 0.22)

        # -------------------------------------------------------------
        # 5. PENENTUAN KECEPATAN LINEAR & ANGULAR HALUS
        # -------------------------------------------------------------
        target_linear_x = 0.0
        target_angular_z = 0.0

        if effective_front > 1.2:
            # Jalan bebas luas: Maju stabil
            target_linear_x = 0.24
            # Jika ada bukaan samping yang sangat menarik (area baru), robot boleh belok
            if abs(best_angle) > math.radians(35) and best_clearance > 3.5:
                target_angular_z = np.clip(0.45 * best_angle, -0.38, 0.38)
            elif in_corridor:
                target_angular_z = omega_center
            else:
                target_angular_z = np.clip(0.35 * best_angle, -0.30, 0.30)
        elif effective_front > 0.60:
            # Mendekati rintangan/tikungan: Deselerasi halus
            ratio = (effective_front - 0.60) / 0.60
            target_linear_x = 0.08 + 0.14 * ratio
            target_angular_z = np.clip(0.60 * best_angle, -0.48, 0.48)
        else:
            # Jarak dekat (< 0.60m): Putar mantap ke arah celah terbaik
            target_linear_x = 0.04
            turn_sign = 1.0 if best_angle >= 0 else -1.0
            target_angular_z = turn_sign * 0.45

        # Low-pass filter untuk kelembutan gerak (Mencegah patah-patah & osilasi)
        self.current_linear_x = 0.70 * self.current_linear_x + 0.30 * target_linear_x
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
