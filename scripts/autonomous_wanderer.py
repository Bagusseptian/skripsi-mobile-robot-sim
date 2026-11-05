#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Seimbang 360° (Uniform Spatial Coverage & Anti-Clustering Explorer).
100% Murni LiDAR RPLiDAR A1 360° (/scan).

Mengapa versi sebelumnya numpuk di satu sisi (dinding utara)?
Karena saat mendekati dinding, robot hanya memilih belok kiri (+90°) atau kanan (-90°),
yang keduanya sejajar dengan dinding utara. Robot tidak pernah memutar badan ke arah
belakang (135°-180°) untuk kembali menyeberang ke lorong selatan.

Solusi Baru:
1. Full 360° Candidate Heading Selector (16 Sektor):
   Robot mengevaluasi seluruh 360 derajat arah gerak di sekelilingnya.
2. Quadrant-Balanced Exploration Score:
   Arah yang mengarah ke kuadran/lorong yang masih kosong (sedikit kunjungan)
   diberi skor tertinggi. Ini memaksa robot keluar dari sisi yang sudah ramai
   dan menyeberangi ruangan untuk meraba sisi yang masih kosong (100% merata).
3. Straight-Line Traverse:
   Setelah memilih arah terbaik, robot melaju lurus cepat menembus ruangan.
4. Active Lateral Branching:
   Mendeteksi celah lorong antar-meja di kiri/kanan dan langsung menyelam masuk.
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

        # Publisher kecepatan
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers: HANYA LiDAR 360° dan Odometri
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # 360 Sektor Laser (Indeks 180 = Depan Lurus 0°)
        self.ranges = np.full(360, 5.0)
        self.min_front = 5.0
        self.left_side = 5.0
        self.right_side = 5.0

        # Odometri & Memori Spasial (Grid 1.0m)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_logged_x = 0.0
        self.last_logged_y = 0.0

        self.cell_size = 1.0
        self.visited_cells = {}  # (cx, cy) -> jumlah kunjungan
        # Kunjungan per Kuadran: NW(1), NE(2), SE(3), SW(4)
        self.quadrant_visits = {1: 0, 2: 0, 3: 0, 4: 0}

        # State Machine Kontrol: 'FORWARD', 'TURNING', 'BACKUP'
        self.state = 'FORWARD'
        self.turn_target_yaw = 0.0
        self.turn_direction = 1.0
        self.straight_ticks = 0
        self.backup_ticks = 0
        self.last_branch_time = time.time()
        self.last_quadrant_check = time.time()

        # Loop kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*65)
        print("  AUTONOMOUS WANDERER (360° UNIFORM COVERAGE EXPLORER)")
        print("="*65)
        print("  Navigasi : Full 360° Candidate Selector (Bebas Numpuk)")
        print("  Eksplor  : Distribusi Merata ke 4 Kuadran & Semua Lorong")
        print("  Sensor   : 100% RPLiDAR A1 360° (/scan)")
        print("="*65 + "\n", flush=True)

    def get_quadrant(self, x, y):
        """1: NW (X<0, Y>0), 2: NE (X>=0, Y>0), 3: SE (X>=0, Y<=0), 4: SW (X<0, Y<=0)"""
        if x < 0 and y > 0:
            return 1
        elif x >= 0 and y > 0:
            return 2
        elif x >= 0 and y <= 0:
            return 3
        else:
            return 4

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.robot_x = pos.x
        self.robot_y = pos.y

        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Catat sel memori
        dist_moved = math.hypot(self.robot_x - self.last_logged_x, self.robot_y - self.last_logged_y)
        if dist_moved >= 0.40:
            self.last_logged_x = self.robot_x
            self.last_logged_y = self.robot_y
            cx = int(round(self.robot_x / self.cell_size))
            cy = int(round(self.robot_y / self.cell_size))
            self.visited_cells[(cx, cy)] = self.visited_cells.get((cx, cy), 0) + 1
            
            q = self.get_quadrant(self.robot_x, self.robot_y)
            self.quadrant_visits[q] = self.quadrant_visits.get(q, 0) + 1

    def scan_callback(self, msg: LaserScan):
        r = np.array(msg.ranges)
        n = len(r)
        if n == 0:
            return

        r = np.where(np.isnan(r) | np.isinf(r) | (r < msg.range_min), msg.range_max, r)
        self.ranges = r
        step = (msg.angle_max - msg.angle_min) / n
        idx_zero = int(-msg.angle_min / step) if step != 0 else n // 2

        def get_min(deg_min, deg_max):
            i_s = max(0, idx_zero + int(math.radians(deg_min) / step))
            i_e = min(n, idx_zero + int(math.radians(deg_max) / step))
            if i_e > i_s:
                return float(np.min(r[i_s:i_e]))
            return 5.0

        self.min_front = get_min(-25, 25)
        self.left_side = get_min(75, 105)
        self.right_side = get_min(-105, -75)

    def select_best_360_heading(self):
        """
        Mengevaluasi 16 kandidat arah 360° di sekitar robot.
        Memilih arah yang:
        1. Memiliki clearance fisik cukup (> 0.85m)
        2. Mengarah ke sel grid dan kuadran yang paling SEDIKIT dikunjungi
        """
        n = len(self.ranges)
        step = (2.0 * math.pi) / n if n > 0 else (math.pi / 180.0)
        idx_zero = n // 2

        # 16 kandidat sudut relatif: -157.5° s.d. +180° dengan interval 22.5°
        candidate_rel_angles = np.linspace(-math.pi + math.pi/8, math.pi, 16)
        best_score = -1e9
        best_target_yaw = self.robot_yaw + math.pi  # Fallback putar balik
        best_rel_angle = math.pi

        for rel_ang in candidate_rel_angles:
            # Hitung clearance di kerucut sudut +/- 15°
            deg_c = math.degrees(rel_ang)
            i_s = max(0, idx_zero + int(math.radians(deg_c - 15) / step))
            i_e = min(n, idx_zero + int(math.radians(deg_c + 15) / step))
            if i_e <= i_s:
                continue

            clearance = float(np.percentile(self.ranges[i_s:i_e], 25))

            # Jika arah ini terblokir dinding/meja dekat (< 0.70m), abaikan
            if clearance < 0.70:
                continue

            # Proyeksi koordinat global
            target_global_yaw = math.atan2(math.sin(self.robot_yaw + rel_ang),
                                           math.cos(self.robot_yaw + rel_ang))
            look_d = min(2.5, clearance)
            px = self.robot_x + look_d * math.cos(target_global_yaw)
            py = self.robot_y + look_d * math.sin(target_global_yaw)
            cx = int(round(px / self.cell_size))
            cy = int(round(py / self.cell_size))

            cell_v = self.visited_cells.get((cx, cy), 0)
            target_q = self.get_quadrant(px, py)
            q_v = self.quadrant_visits.get(target_q, 0)

            # SKOR EKSPROLASI CERDAS:
            # + Clearance fisik
            # - Penalti sel lokal yang sudah sering dilewati
            # - Penalti kuadran global (memaksa robot migrasi ke sisi kosong!)
            score = (clearance * 1.5) - (cell_v * 2.5) - (q_v * 0.4)

            # Bonus untuk sudut yang memutar badan keluar dari dinding (belok > 60°)
            if abs(rel_ang) > math.radians(60):
                score += 1.0

            if score > best_score:
                best_score = score
                best_target_yaw = target_global_yaw
                best_rel_angle = rel_ang

        # Arah putar: +1.0 jika rel_angle > 0, -1.0 jika < 0
        turn_dir = 1.0 if best_rel_angle >= 0 else -1.0
        return best_target_yaw, turn_dir

    def control_loop(self):
        cmd = Twist()
        now = time.time()

        # -------------------------------------------------------------
        # STATE 1: BACKUP (Mundur jika terdesak < 0.32m)
        # -------------------------------------------------------------
        if self.state == 'BACKUP':
            self.backup_ticks -= 1
            cmd.linear.x = -0.14
            cmd.angular.z = 0.0
            if self.backup_ticks <= 0:
                self.state = 'TURNING'
                self.turn_target_yaw, self.turn_direction = self.select_best_360_heading()
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
        # STATE 2: TURNING (Menuju Target Yaw 360° yang Dipilih)
        # -------------------------------------------------------------
        if self.state == 'TURNING':
            yaw_diff = math.atan2(math.sin(self.turn_target_yaw - self.robot_yaw),
                                  math.cos(self.turn_target_yaw - self.robot_yaw))

            if abs(yaw_diff) > 0.20:
                # Putar badan mantap di tempat
                cmd.linear.x = 0.02
                turn_sign = 1.0 if yaw_diff > 0 else -1.0
                cmd.angular.z = turn_sign * 0.55
            else:
                # Sudah menghadap arah lapang baru
                self.state = 'FORWARD'
                self.straight_ticks = 0
                print(f"[WANDERER] Arah baru tercapai! Melaju lurus menembus kuadran...", flush=True)

            self.cmd_pub.publish(cmd)
            return

        # -------------------------------------------------------------
        # STATE 3: FORWARD (Maju Lurus Murni Tanpa Orbit)
        # -------------------------------------------------------------
        self.straight_ticks += 1

        # Cek jika mendekati dinding/meja di depan (< 0.65m)
        if self.min_front < 0.65:
            self.state = 'TURNING'
            self.turn_target_yaw, self.turn_direction = self.select_best_360_heading()
            print(f"[WANDERER] Rintangan depan ({self.min_front:.2f}m) -> Evaluasi 360° menuju zona paling kosong", flush=True)
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction * 0.50
            self.cmd_pub.publish(cmd)
            return

        # Anti-Clumping Periodic Migration:
        # Jika sudah jalan lurus stabil (>3.0s) dan sisi saat ini sudah terlalu ramai,
        # periksa apakah ada dorongan untuk migrasi ke kuadran seberang
        if (now - self.last_quadrant_check > 8.0) and self.straight_ticks > 30:
            current_q = self.get_quadrant(self.robot_x, self.robot_y)
            # Cari kuadran paling sedikit dikunjungi
            min_q = min(self.quadrant_visits, key=self.quadrant_visits.get)
            if self.quadrant_visits[current_q] > self.quadrant_visits[min_q] + 8:
                # Migrasi terpicu! Pilih arah 360° baru yang menuju kuadran seberang
                self.state = 'TURNING'
                self.turn_target_yaw, self.turn_direction = self.select_best_360_heading()
                self.last_quadrant_check = now
                print(f"[WANDERER] Migrasi Anti-Numpuk! Dari Kuadran {current_q} menuju Kuadran {min_q}", flush=True)
                return

        # Lateral Gap Diving: Menyapu lorong antar-meja di kiri/kanan
        if self.straight_ticks > 15 and (now - self.last_branch_time > 4.0):
            # Cek bukaan kiri (> 1.60m)
            if self.left_side > 1.60:
                px = self.robot_x + 2.0 * math.cos(self.robot_yaw + math.pi/2)
                py = self.robot_y + 2.0 * math.sin(self.robot_yaw + math.pi/2)
                cx, cy = int(round(px / self.cell_size)), int(round(py / self.cell_size))
                if self.visited_cells.get((cx, cy), 0) == 0:
                    self.state = 'TURNING'
                    self.turn_target_yaw = math.atan2(math.sin(self.robot_yaw + math.pi/2),
                                                      math.cos(self.robot_yaw + math.pi/2))
                    self.turn_direction = 1.0
                    self.last_branch_time = now
                    print(f"[WANDERER] Menyelam masuk lorong KIRI (Area Kosong)!", flush=True)
                    return

            # Cek bukaan kanan (> 1.60m)
            if self.right_side > 1.60:
                px = self.robot_x + 2.0 * math.cos(self.robot_yaw - math.pi/2)
                py = self.robot_y + 2.0 * math.sin(self.robot_yaw - math.pi/2)
                cx, cy = int(round(px / self.cell_size)), int(round(py / self.cell_size))
                if self.visited_cells.get((cx, cy), 0) == 0:
                    self.state = 'TURNING'
                    self.turn_target_yaw = math.atan2(math.sin(self.robot_yaw - math.pi/2),
                                                      math.cos(self.robot_yaw - math.pi/2))
                    self.turn_direction = -1.0
                    self.last_branch_time = now
                    print(f"[WANDERER] Menyelam masuk lorong KANAN (Area Kosong)!", flush=True)
                    return

        # Centering lorong halus jika diapit meja (< 1.05m di kedua sisi)
        omega_center = 0.0
        if self.left_side < 1.05 and self.right_side < 1.05:
            diff = self.left_side - self.right_side
            if abs(diff) > 0.12:
                omega_center = np.clip(0.30 * diff, -0.20, 0.20)

        # MAJU LURUS CEPAT
        cmd.linear.x = 0.24
        cmd.angular.z = omega_center
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
