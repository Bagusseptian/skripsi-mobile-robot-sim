#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Murni (Pure LiDAR-Only Straight-Drive & Wall-Bouncing Explorer).
100% Bebas Melingkar / Orbiting. Menggunakan HANYA RPLiDAR A1 360° (/scan).

Mengapa versi sebelumnya muter-muter?
Kamera kedalaman Astra menangkap lantai di jarak 0.66m, sehingga robot salah
mengira ada rintangan permanen di depannya dan terjebak berputar selamanya.
Dengan murni LiDAR 360° pada bidang horizontal:
- Tidak ada deteksi lantai palsu.
- Di ruang terbuka robot melaju LURUS MURNI (angular.z = 0.0) menembus lorong.
- Saat menabrak dinding (< 0.65m), robot memutar badan di tempat ke arah terbuka,
  lalu melaju lurus kembali menembus ruangan lain.
"""

import sys
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class AutonomousWanderer(Node):
    def __init__(self):
        super().__init__('autonomous_wanderer')

        # Publisher kecepatan motor
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers: HANYA LiDAR 360° dan Odometri (Tanpa Kamera Kedalaman!)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Status Sektor Laser (Indeks 180 = Depan Lurus 0°)
        self.min_front = 5.0
        self.min_left_diag = 5.0
        self.min_right_diag = 5.0
        self.left_side = 5.0
        self.right_side = 5.0

        # Odometri & Memori Eksplorasi Spasial
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_logged_x = 0.0
        self.last_logged_y = 0.0

        self.cell_size = 1.0  # Grid 1.0 meter
        self.visited_cells = {}  # (cx, cy) -> jumlah kunjungan

        # State Machine Kontrol: 'FORWARD', 'TURNING', 'BACKUP'
        self.state = 'FORWARD'
        self.turn_direction = 1.0  # 1.0 = Kiri, -1.0 = Kanan
        self.straight_ticks = 0
        self.backup_ticks = 0
        self.last_branch_time = time.time()

        # Loop kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*65)
        print("  AUTONOMOUS WANDERER (PURE LIDAR STRAIGHT-DRIVE EXPLORER)")
        print("="*65)
        print("  Sensor   : 100% RPLiDAR A1 360° (/scan)")
        print("  Navigasi : Lurus Murni (Bebas Orbit), Menembus Seluruh Lorong")
        print("  Belok    : Wall-Bouncing & Lateral Corridor Diving")
        print("="*65 + "\n", flush=True)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = pos.x
        self.robot_y = pos.y

        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Catat memori kunjungan setiap berpindah > 0.40m
        dist_moved = math.hypot(self.robot_x - self.last_logged_x, self.robot_y - self.last_logged_y)
        if dist_moved >= 0.40:
            self.last_logged_x = self.robot_x
            self.last_logged_y = self.robot_y
            cx = int(round(self.robot_x / self.cell_size))
            cy = int(round(self.robot_y / self.cell_size))
            self.visited_cells[(cx, cy)] = self.visited_cells.get((cx, cy), 0) + 1

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        # Filter NaN / Inf
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)
        step = (msg.angle_max - msg.angle_min) / n
        idx_zero = int(-msg.angle_min / step) if step != 0 else n // 2

        def get_min_in_sector(deg_min, deg_max):
            i_start = max(0, idx_zero + int(math.radians(deg_min) / step))
            i_end = min(n, idx_zero + int(math.radians(deg_max) / step))
            if i_end > i_start:
                return float(np.min(ranges[i_start:i_end]))
            return 5.0

        # Sektor Sesuai Geometri RPLiDAR (0° = Depan, +90° = Kiri, -90° = Kanan)
        self.min_front = get_min_in_sector(-25, 25)
        self.min_left_diag = get_min_in_sector(25, 65)
        self.min_right_diag = get_min_in_sector(-65, -25)
        self.left_side = get_min_in_sector(75, 105)
        self.right_side = get_min_in_sector(-105, -75)

    def choose_turn_direction(self):
        """Memilih arah belok yang lebih lapang dan paling sedikit dikunjungi"""
        # Evaluasi proyeksi arah kiri (+90°) vs kanan (-90°)
        yaw_l = self.robot_yaw + math.radians(85)
        yaw_r = self.robot_yaw - math.radians(85)

        px_l = self.robot_x + 2.0 * math.cos(yaw_l)
        py_l = self.robot_y + 2.0 * math.sin(yaw_l)
        cx_l, cy_l = int(round(px_l / self.cell_size)), int(round(py_l / self.cell_size))
        v_l = self.visited_cells.get((cx_l, cy_l), 0)

        px_r = self.robot_x + 2.0 * math.cos(yaw_r)
        py_r = self.robot_y + 2.0 * math.sin(yaw_r)
        cx_r, cy_r = int(round(px_r / self.cell_size)), int(round(py_r / self.cell_size))
        v_r = self.visited_cells.get((cx_r, cy_r), 0)

        # Skor = Keterbukaan fisik - bobot kunjungan masa lalu
        score_l = (self.left_side * 1.5 + self.min_left_diag) - (v_l * 2.0)
        score_r = (self.right_side * 1.5 + self.min_right_diag) - (v_r * 2.0)

        # 1.0 = Kiri, -1.0 = Kanan
        return 1.0 if score_l >= score_r else -1.0

    def control_loop(self):
        cmd = Twist()
        now = time.time()

        # -------------------------------------------------------------
        # STATE 1: BACKUP (Mundur darurat jika jarak < 0.32m)
        # -------------------------------------------------------------
        if self.state == 'BACKUP':
            self.backup_ticks -= 1
            cmd.linear.x = -0.14
            cmd.angular.z = 0.0
            if self.backup_ticks <= 0:
                self.state = 'TURNING'
                self.turn_direction = self.choose_turn_direction()
            self.cmd_pub.publish(cmd)
            return

        if self.min_front < 0.32:
            self.state = 'BACKUP'
            self.backup_ticks = 10  # Mundur 1.0 detik
            cmd.linear.x = -0.14
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # -------------------------------------------------------------
        # STATE 2: TURNING (Putar haluan di tempat saat ketemu rintangan)
        # -------------------------------------------------------------
        if self.state == 'TURNING':
            # Putar badan mantap di tempat
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction * 0.52

            # Syarat selesai belok: Depan sudah terbuka lapang (> 1.30m)
            diag = self.min_left_diag if self.turn_direction > 0 else self.min_right_diag
            if self.min_front > 1.30 and diag > 0.75:
                self.state = 'FORWARD'
                self.straight_ticks = 0
                print(f"[WANDERER] Depan lapang ({self.min_front:.2f}m). Maju lurus!", flush=True)

            self.cmd_pub.publish(cmd)
            return

        # -------------------------------------------------------------
        # STATE 3: FORWARD (Maju Lurus Murni Menembus Ruangan & Lorong)
        # -------------------------------------------------------------
        self.straight_ticks += 1

        # Cek jika mendekati dinding/meja di depan (< 0.65m)
        if self.min_front < 0.65:
            self.state = 'TURNING'
            self.turn_direction = self.choose_turn_direction()
            print(f"[WANDERER] Rintangan depan ({self.min_front:.2f}m) -> Belok arah {self.turn_direction}", flush=True)
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction * 0.50
            self.cmd_pub.publish(cmd)
            return

        # Cek jika ada lorong samping (Lateral Gap Diving antar-meja)
        # Hanya jika sudah jalan lurus minimal 2.0 detik dan jeda 4.0 detik
        if self.straight_ticks > 20 and (now - self.last_branch_time > 4.0):
            # Cek bukaan kiri (> 1.7m)
            if self.left_side > 1.70 and self.min_left_diag > 1.30:
                px = self.robot_x + 2.0 * math.cos(self.robot_yaw + math.pi/2)
                py = self.robot_y + 2.0 * math.sin(self.robot_yaw + math.pi/2)
                cx, cy = int(round(px / self.cell_size)), int(round(py / self.cell_size))
                if self.visited_cells.get((cx, cy), 0) == 0:
                    self.state = 'TURNING'
                    self.turn_direction = 1.0  # Belok kiri masuk lorong baru
                    self.last_branch_time = now
                    print(f"[WANDERER] Menyelam masuk lorong KIRI (Area Baru)!", flush=True)
                    return

            # Cek bukaan kanan (> 1.7m)
            if self.right_side > 1.70 and self.min_right_diag > 1.30:
                px = self.robot_x + 2.0 * math.cos(self.robot_yaw - math.pi/2)
                py = self.robot_y + 2.0 * math.sin(self.robot_yaw - math.pi/2)
                cx, cy = int(round(px / self.cell_size)), int(round(py / self.cell_size))
                if self.visited_cells.get((cx, cy), 0) == 0:
                    self.state = 'TURNING'
                    self.turn_direction = -1.0  # Belok kanan masuk lorong baru
                    self.last_branch_time = now
                    print(f"[WANDERER] Menyelam masuk lorong KANAN (Area Baru)!", flush=True)
                    return

        # Centering lorong halus jika diapit dinding/meja (< 1.05m di kedua sisi)
        omega_center = 0.0
        if self.left_side < 1.05 and self.right_side < 1.05:
            diff = self.left_side - self.right_side
            if abs(diff) > 0.12:
                omega_center = np.clip(0.30 * diff, -0.20, 0.20)

        # MAJU LURUS CEPAT TANPA KURVATUR!
        cmd.linear.x = 0.24
        cmd.angular.z = omega_center  # 0.0 jika di ruang terbuka bebas!
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
