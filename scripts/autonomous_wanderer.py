#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Cerdas (Auto-Saturation Map Coverage & Auto-RTH Return to Home).
Dirancang untuk pemetaan komprehensif:
1. Peta 2D Murni LiDAR 360° (/scan) & Peta 3D Murni Kamera Kedalaman (/camera).
2. Deteksi Kejenuhan Peta (Map Saturation Detector):
   Memantau laju pertambahan sel /map. Jika sudah jenuh (ruangan 100% terscan),
   robot otomatis menyimpulkan pemetaan selesai.
3. Auto-RTH (Return to Home):
   Robot secara otomatis berbalik arah dan bernavigasi pulang menuju koordinat awal (0.0, 0.0),
   lalu parkir rapi dan otomatis memicu penyimpanan peta 2D & 3D.
"""

import os
import sys
import math
import time
import subprocess
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
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

        # QoS Latched untuk /map
        map_qos = QoSProfile(depth=1)
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = ReliabilityPolicy.RELIABLE

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        if self.bridge:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # Status Sensor
        self.sector_ranges = np.full(9, 5.0)
        self.sector_angles = np.linspace(-math.radians(70), math.radians(70), 9)
        self.left_side_dist = 5.0
        self.right_side_dist = 5.0
        self.min_front_dist = 5.0
        self.min_depth_cam_dist = 5.0

        # Koordinat Robot & Titik Rumah (Home)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.last_logged_x = 0.0
        self.last_logged_y = 0.0
        self.total_dist_traveled = 0.0
        self.visited_cells = {}
        self.cell_size = 1.0

        # Filter Kecepatan
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # State Machine: 'EXPLORE', 'RTH', 'FINISHED', 'ESCAPE'
        self.state = 'EXPLORE'
        self.is_escaping = False
        self.escape_phase = 0
        self.escape_timer = 0
        self.escape_turn_dir = 1.0

        # Deteksi Saturasi Peta (/map)
        self.start_time = time.time()
        self.last_map_check_time = time.time()
        self.prev_explored_cells = 0
        self.saturation_count = 0
        self.total_explored_cells = 0

        # Timer loop kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*70)
        print("  AUTONOMOUS WANDERER V4 (COVERAGE SATURATION & AUTO-RTH)")
        print("="*70)
        print("  Sensor 2D : RPLiDAR A1 360° (/scan) -> Kartografi Presisi")
        print("  Sensor 3D : Astra RGB-D (/camera)   -> Rekonstruksi Vertikal")
        print("  Fitur RTH : Otomatis Pulang ke Titik Awal saat 100% Terscan")
        print("="*70 + "\n", flush=True)

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = pos.x
        self.robot_y = pos.y

        # Simpan titik start pertama kali sebagai Titik Rumah (Home)
        if self.home_x is None:
            self.home_x = pos.x
            self.home_y = pos.y
            self.last_logged_x = pos.x
            self.last_logged_y = pos.y
            self.get_logger().info(f"[WANDERER] Titik Rumah (Home) terkunci pada: ({self.home_x:.2f}, {self.home_y:.2f})")

        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        dist_moved = math.hypot(self.robot_x - self.last_logged_x, self.robot_y - self.last_logged_y)
        if dist_moved >= 0.75:
            self.total_dist_traveled += dist_moved
            self.last_logged_x = self.robot_x
            self.last_logged_y = self.robot_y
            cx = int(round(self.robot_x / self.cell_size))
            cy = int(round(self.robot_y / self.cell_size))
            self.visited_cells[(cx, cy)] = self.visited_cells.get((cx, cy), 0) + 1

    def map_callback(self, msg: OccupancyGrid):
        # Hitung jumlah sel yang bukan -1 (known: free or occupied)
        data = np.array(msg.data, dtype=np.int8)
        self.total_explored_cells = int(np.sum(data != -1))

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        if num_readings == 0:
            return

        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)
        step = (msg.angle_max - msg.angle_min) / num_readings
        idx_zero = int(-msg.angle_min / step) if step != 0 else num_readings // 2

        # Sektor busur depan [-70°, +70°]
        arc_half = math.radians(70)
        idx_span_arc = int(arc_half / step)
        start_idx = max(0, idx_zero - idx_span_arc)
        end_idx = min(num_readings, idx_zero + idx_span_arc)
        
        front_arc = ranges[start_idx:end_idx]
        if len(front_arc) > 9:
            splits = np.array_split(front_arc, 9)
            self.sector_ranges = np.array([float(np.min(s)) for s in splits])

        idx_20 = int(math.radians(20) / step)
        c_left = ranges[idx_zero: min(num_readings, idx_zero + idx_20)]
        c_right = ranges[max(0, idx_zero - idx_20): idx_zero]
        center = np.concatenate([c_left, c_right]) if len(c_left) and len(c_right) else ranges
        self.min_front_dist = float(np.min(center)) if len(center) > 0 else 5.0

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

    def check_map_saturation(self):
        """Memeriksa apakah pertambahan sel peta sudah jenuh (100% terscan)."""
        now = time.time()
        elapsed = now - self.start_time

        # Periksa tiap 5 detik setelah robot menjelajah minimal 45 detik dan map sudah terbangun
        if elapsed >= 45.0 and (self.total_explored_cells > 500) and (now - self.last_map_check_time >= 5.0):
            delta_cells = self.total_explored_cells - self.prev_explored_cells
            self.prev_explored_cells = self.total_explored_cells
            self.last_map_check_time = now

            # Jika sel baru bertambah sangat sedikit (< 20 sel dalam 5 detik)
            if delta_cells < 20:
                self.saturation_count += 1
                self.get_logger().info(f"[COVERAGE] Kejenuhan peta terdeteksi: streak {self.saturation_count}/3 (+{delta_cells} sel baru)")
            else:
                self.saturation_count = max(0, self.saturation_count - 1)

            # Jika jenuh 3 kali berturut-turut (15 detik) ATAU waktu jelajah > 110s:
            if self.saturation_count >= 3 or elapsed >= 110.0:
                self.state = 'RTH'
                hx = self.home_x if self.home_x is not None else 0.0
                hy = self.home_y if self.home_y is not None else 0.0
                self.get_logger().info("\n" + "*"*65)
                self.get_logger().info(f">>> [SATURASI 100%] RUANGAN SELESAI DIPETAKAN LENGKAP ({self.total_explored_cells} sel)! <<<")
                self.get_logger().info(f">>> MEMULAI AUTO-RTH: KEMBALI KE TITIK AWAL ({hx:.2f}, {hy:.2f}) <<<")
                self.get_logger().info("*"*65 + "\n")

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        effective_front = min(self.min_front_dist, self.min_depth_cam_dist)

        # 0. Jika Misi Sudah Selesai & Parkir
        if self.state == 'FINISHED':
            self.stop()
            return

        # 1. State: EMERGENCY ESCAPE (Bebas dari jepitan dinding < 0.35m)
        if self.is_escaping:
            if self.escape_phase == 1:
                cmd.linear.x = -0.12
                cmd.angular.z = 0.0
                self.escape_timer -= 1
                if self.escape_timer <= 0:
                    self.escape_phase = 2
                    self.escape_timer = 12
            elif self.escape_phase == 2:
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
            self.escape_timer = 12
            self.escape_turn_dir = 1.0 if self.left_side_dist > self.right_side_dist else -1.0
            cmd.linear.x = -0.12
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # 2. Cek Saturasi Pemetaan
        if self.state == 'EXPLORE':
            self.check_map_saturation()

        # 3. State: AUTO-RTH (Navigasi Pulang ke Titik Awal)
        if self.state == 'RTH':
            dx = (self.home_x if self.home_x is not None else 0.0) - self.robot_x
            dy = (self.home_y if self.home_y is not None else 0.0) - self.robot_y
            dist_to_home = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)
            heading_err = math.atan2(math.sin(target_yaw - self.robot_yaw), math.cos(target_yaw - self.robot_yaw))

            # JIKA SUDAH SAMPAI DI RUMAH (< 0.40m): PARKIR & SELESAI!
            if dist_to_home < 0.40:
                self.state = 'FINISHED'
                self.stop()
                self.get_logger().info("\n" + "="*65)
                self.get_logger().info(">>> [RTH SUKSES] ROBOT TIBA DI TITIK AWAL & PARKIR SEMPURNA! <<<")
                self.get_logger().info(f">>> Sisa Galat Parkir: {dist_to_home*100:.1f} cm <<<")
                self.get_logger().info(">>> Memanggil map_saver_unified secara otomatis... <<<")
                self.get_logger().info("="*65 + "\n")
                
                # Picu penyimpanan peta otomatis
                subprocess.Popen([
                    'ros2', 'run', 'robot_sim', 'map_saver_unified.py',
                    '--name', 'map_world_c',
                    '--output-dir', os.path.expanduser('~/skripsi_ws/maps')
                ])
                return

            # Jika rintangan di depan, hindari dulu sambil tetap mengarah ke rumah
            if effective_front < 0.70:
                best_sector = int(np.argmax(self.sector_ranges))
                best_angle = self.sector_angles[best_sector]
                target_linear_x = 0.08
                target_angular_z = np.clip(0.5 * best_angle, -0.4, 0.4)
            else:
                # Menuju rumah langsung
                target_linear_x = float(np.clip(dist_to_home * 0.25, 0.08, 0.22))
                target_angular_z = float(np.clip(0.65 * heading_err, -0.45, 0.45))

            self.current_linear_x = 0.7 * self.current_linear_x + 0.3 * target_linear_x
            self.current_angular_z = 0.65 * self.current_angular_z + 0.35 * target_angular_z
            cmd.linear.x = float(self.current_linear_x)
            cmd.angular.z = float(self.current_angular_z)
            self.cmd_pub.publish(cmd)
            return

        # 4. State: EXPLORE (Jelajah Halus Bebas Osilasi)
        clearances = np.copy(self.sector_ranges)
        if self.total_dist_traveled > 10.0:
            for i, ang in enumerate(self.sector_angles):
                proj_yaw = self.robot_yaw + ang
                look_dist = min(2.0, clearances[i])
                px = self.robot_x + look_dist * math.cos(proj_yaw)
                py = self.robot_y + look_dist * math.sin(proj_yaw)
                cx = int(round(px / self.cell_size))
                cy = int(round(py / self.cell_size))
                visits = self.visited_cells.get((cx, cy), 0)
                if visits == 0:
                    clearances[i] += 1.5
                elif visits >= 4:
                    clearances[i] *= 0.6

        best_sector = int(np.argmax(clearances))
        best_angle = self.sector_angles[best_sector]

        in_corridor = (self.left_side_dist < 1.1) and (self.right_side_dist < 1.1)
        omega_center = 0.0
        if in_corridor:
            lateral_error = self.left_side_dist - self.right_side_dist
            if abs(lateral_error) > 0.12:
                omega_center = np.clip(0.35 * lateral_error, -0.22, 0.22)

        if effective_front > 1.2:
            target_linear_x = 0.24
            target_angular_z = omega_center if in_corridor else np.clip(0.35 * best_angle, -0.30, 0.30)
        elif effective_front > 0.60:
            ratio = (effective_front - 0.60) / 0.60
            target_linear_x = 0.08 + 0.14 * ratio
            target_angular_z = np.clip(0.55 * best_angle, -0.45, 0.45)
        else:
            target_linear_x = 0.03
            turn_sign = 1.0 if best_angle >= 0 else -1.0
            target_angular_z = turn_sign * 0.42

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
