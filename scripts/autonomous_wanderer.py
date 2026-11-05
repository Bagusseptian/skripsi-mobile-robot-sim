#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Reaktif Bebas Looping (Autonomous Wandering with Anti-Looping & Dispersion).
Menggabungkan data LiDAR 360° (/scan), Kamera Kedalaman RGB-D (/camera/depth/image_raw), dan Odometri (/odom).

Fitur:
1. Turn Commitment: Mengunci durasi belok 1.2-2.0 detik agar robot berputar 70°-120° ke lorong baru,
   mencegah 'orbit loop' di sekitar meja.
2. Anti-Looping Odometry Memory: Mencatat riwayat sel (x, y). Jika mendeteksi area yang sudah sering dilewati,
   robot memicu putaran dispersi 120°-180° untuk mencari area belum terjamah.
3. Periodic Heading Perturbation: Setiap 15 detik jalan lurus, robot berbelok acak masuk lorong silang.
4. Emergency Escape: Manuver mundur otomatis jika terjepit di sudut sempit (< 0.38m).
"""

import sys
import math
import time
import random
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

        # State navigasi sensor
        self.min_front_dist = 10.0
        self.min_left_dist = 10.0
        self.min_right_dist = 10.0
        self.min_depth_cam_dist = 10.0

        # Parameter kontrol gerak
        self.safe_threshold = 0.65       # Jarak mulai belok (meter)
        self.emergency_threshold = 0.38  # Jarak darurat mundur (meter)
        self.target_linear_speed = 0.22  # Kecepatan maju normal (m/s)
        self.turn_speed = 0.48           # Kecepatan putar (rad/s)

        # State machine kontrol
        self.state = 'FORWARD'           # 'FORWARD', 'TURNING', 'ESCAPE', 'DISPERSION'
        self.state_ticks_remaining = 0
        self.turn_direction = 1.0        # 1.0 kiri, -1.0 kanan

        # Anti-looping memory berbasis odometri
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.visited_cells = {}          # (cell_x, cell_y) -> visit_count
        self.cell_size = 0.75            # Ukuran grid memori (meter)
        self.last_cell_log_time = time.time()
        self.last_perturbation_time = time.time()

        # Timer loop kontrol 10 Hz (0.1 detik)
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*65)
        print("  AUTONOMOUS WANDERER V2 (ANTI-LOOPING & COVERAGE DISPERSION)")
        print("="*65)
        print("  Status   : Aktif!")
        print("  Sensor   : LiDAR 360° + RGB-D Depth + Odometry Memory")
        print("  Fitur    : Turn Commitment + Visited Cell Loop Breaker")
        print("  Stop     : Tekan Ctrl+C untuk menghentikan eksplorasi")
        print("="*65 + "\n", flush=True)

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        now = time.time()
        # Catat sel kunjungan tiap 0.5 detik
        if now - self.last_cell_log_time >= 0.5:
            cx = int(round(self.robot_x / self.cell_size))
            cy = int(round(self.robot_y / self.cell_size))
            self.visited_cells[(cx, cy)] = self.visited_cells.get((cx, cy), 0) + 1
            self.last_cell_log_time = now

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        if num_readings == 0:
            return

        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)

        step = (msg.angle_max - msg.angle_min) / num_readings
        idx_zero = int(-msg.angle_min / step) if step != 0 else num_readings // 2

        idx_span_35 = int(math.radians(35) / step)
        idx_span_85 = int(math.radians(85) / step)

        # Sektor depan [-35°, +35°]
        front_left = ranges[max(0, idx_zero): min(num_readings, idx_zero + idx_span_35)]
        front_right = ranges[max(0, idx_zero - idx_span_35): max(0, idx_zero)]
        front = np.concatenate([front_left, front_right]) if len(front_left) and len(front_right) else ranges

        # Sektor samping kiri [+35°, +85°] dan kanan [-85°, -35°]
        left = ranges[min(num_readings, idx_zero + idx_span_35): min(num_readings, idx_zero + idx_span_85)]
        right = ranges[max(0, idx_zero - idx_span_85): max(0, idx_zero - idx_span_35)]

        self.min_front_dist = float(np.min(front)) if len(front) > 0 else 10.0
        self.min_left_dist = float(np.min(left)) if len(left) > 0 else 10.0
        self.min_right_dist = float(np.min(right)) if len(right) > 0 else 10.0

    def depth_callback(self, msg: Image):
        try:
            if '32F' in msg.encoding:
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_img = depth_raw.astype(np.float32) / 1000.0

            h, w = depth_img.shape
            ch, cw = h // 2, w // 2
            center_crop = depth_img[max(0, ch-25): min(h, ch+25), max(0, cw-30): min(w, cw+30)]
            valid = center_crop[np.isfinite(center_crop) & (center_crop > 0.1)]
            if len(valid) > 0:
                self.min_depth_cam_dist = float(np.min(valid))
            else:
                self.min_depth_cam_dist = 10.0
        except Exception:
            pass

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        effective_front = min(self.min_front_dist, self.min_depth_cam_dist)

        # 1. State: ESCAPE (Mundur darurat jika terjepit di sudut)
        if self.state == 'ESCAPE':
            if self.state_ticks_remaining > 0:
                cmd.linear.x = -0.12
                cmd.angular.z = self.turn_direction * 0.45
                self.state_ticks_remaining -= 1
                self.cmd_pub.publish(cmd)
                return
            else:
                self.state = 'FORWARD'

        # Trigger ESCAPE jika terlalu dekat (< 0.38m)
        if effective_front < self.emergency_threshold or (self.min_left_dist < self.emergency_threshold and self.min_right_dist < self.emergency_threshold):
            self.state = 'ESCAPE'
            self.state_ticks_remaining = 16 # 1.6 detik mundur
            self.turn_direction = 1.0 if self.min_left_dist > self.min_right_dist else -1.0
            cmd.linear.x = -0.12
            cmd.angular.z = self.turn_direction * 0.45
            self.cmd_pub.publish(cmd)
            return

        # 2. State: TURNING (Kunci belokan terencana, jangan langsung lurus lagi)
        if self.state in ['TURNING', 'DISPERSION']:
            if self.state_ticks_remaining > 0:
                cmd.linear.x = 0.04
                cmd.angular.z = self.turn_direction * self.turn_speed
                self.state_ticks_remaining -= 1
                self.cmd_pub.publish(cmd)
                return
            else:
                self.state = 'FORWARD'

        # 3. Anti-Looping Check: Apakah area saat ini sudah terlalu sering dikunjungi?
        cx = int(round(self.robot_x / self.cell_size))
        cy = int(round(self.robot_y / self.cell_size))
        visits = self.visited_cells.get((cx, cy), 0)

        if visits >= 6: # Sel ini sudah dikunjungi >= 6 kali (indikasi kuat terjebak orbit loop)
            # Paksa dispersi belok tajam ke arah berlawanan
            self.state = 'DISPERSION'
            self.state_ticks_remaining = random.randint(18, 25) # 1.8 - 2.5 detik (putar ~120°-180°)
            self.turn_direction = -1.0 if random.random() < 0.5 else 1.0
            # Reset counter sel ini agar tidak memicu terus menerus
            self.visited_cells[(cx, cy)] = 1
            cmd.linear.x = 0.03
            cmd.angular.z = self.turn_direction * self.turn_speed
            self.cmd_pub.publish(cmd)
            return

        # 4. Periodic Heading Perturbation: Tiap 12 detik, belok acak masuk lorong silang
        if now - self.last_perturbation_time >= 12.0:
            self.last_perturbation_time = now
            self.state = 'TURNING'
            self.state_ticks_remaining = random.randint(10, 16) # 1.0 - 1.6 detik
            self.turn_direction = 1.0 if self.min_left_dist > self.min_right_dist else -1.0
            cmd.linear.x = 0.05
            cmd.angular.z = self.turn_direction * self.turn_speed
            self.cmd_pub.publish(cmd)
            return

        # 5. Obstacle Avoidance: Rintangan di depan (< safe_threshold)
        if effective_front < self.safe_threshold:
            self.state = 'TURNING'
            # Kunci durasi belok 1.3 - 2.0 detik (13 - 20 ticks) agar robot benar-benar ganti lorong
            self.state_ticks_remaining = random.randint(13, 20)
            
            # Pilih arah dengan clearance terjauh + sedikit variasi
            if self.min_left_dist > self.min_right_dist:
                self.turn_direction = 1.0
            elif self.min_right_dist > self.min_left_dist:
                self.turn_direction = -1.0
            else:
                self.turn_direction = 1.0 if random.random() < 0.5 else -1.0

            cmd.linear.x = 0.04
            cmd.angular.z = self.turn_direction * self.turn_speed
            self.cmd_pub.publish(cmd)
            return

        # 6. Wall Avoidance Samping (Centering di lorong)
        if self.min_left_dist < 0.42:
            cmd.linear.x = self.target_linear_speed * 0.75
            cmd.angular.z = -0.25
        elif self.min_right_dist < 0.42:
            cmd.linear.x = self.target_linear_speed * 0.75
            cmd.angular.z = 0.25
        else:
            # 7. Maju Lurus Bebas
            cmd.linear.x = self.target_linear_speed
            cmd.angular.z = 0.0

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
