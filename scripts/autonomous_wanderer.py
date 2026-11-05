#!/usr/bin/env python3
"""
autonomous_wanderer.py
======================
Node Penjelajah Otonom Reaktif (Autonomous Wandering & Multi-Sensor Obstacle Avoidance).
Menggabungkan data LiDAR 360° (/scan) dan Kamera Kedalaman RGB-D (/camera/depth/image_raw).
Fungsi:
- Menjelajah ruangan secara mandiri tanpa kendali manual (hands-free mapping).
- Mencegah slip/oversteering dengan akselerasi halus.
- Otomatis membelokkan robot ke arah lorong paling bebas.
- Manuver mundur (escape behavior) saat menemui sudut sempit/buntu.
"""

import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
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
        
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None
        if self.bridge:
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)

        # State navigasi
        self.min_front_dist = 10.0
        self.min_left_dist = 10.0
        self.min_right_dist = 10.0
        self.min_depth_cam_dist = 10.0

        # Parameter kontrol
        self.safe_threshold = 0.65       # Jarak mulai belok (meter)
        self.emergency_threshold = 0.38  # Jarak darurat untuk mundur (meter)
        self.target_linear_speed = 0.22  # Kecepatan maju normal (m/s)
        self.turn_speed = 0.42           # Kecepatan putar (rad/s)

        self.escape_steps = 0
        self.turn_direction = 1.0        # 1.0 kiri, -1.0 kanan

        # Timer loop kontrol 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        print("\n" + "="*60)
        print("     AUTONOMOUS WANDERER (MULTI-SENSOR REAKTIF)")
        print("="*60)
        print("Status: Aktif! Robot akan menjelajah mandiri untuk pemetaan.")
        print("Sensor: LiDAR 360° + RGB-D Camera Depth aktif.")
        print("Tekan Ctrl+C untuk menghentikan eksplorasi.")
        print("="*60 + "\n", flush=True)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        num_readings = len(ranges)
        if num_readings == 0:
            return

        # Ganti nilai inf / nan dengan jarak maksimum
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges < msg.range_min), msg.range_max, ranges)

        # Pembagian sektor sudut:
        # Front: [-30°, +30°]
        # Left : [+30°, +85°]
        # Right: [-85°, -30°]
        step = (msg.angle_max - msg.angle_min) / num_readings
        idx_zero = int(-msg.angle_min / step) if step != 0 else num_readings // 2

        idx_span_30 = int(math.radians(30) / step)
        idx_span_85 = int(math.radians(85) / step)

        # Sektor depan
        front_left = ranges[max(0, idx_zero): min(num_readings, idx_zero + idx_span_30)]
        front_right = ranges[max(0, idx_zero - idx_span_30): max(0, idx_zero)]
        front = np.concatenate([front_left, front_right]) if len(front_left) and len(front_right) else ranges

        # Sektor samping
        left = ranges[min(num_readings, idx_zero + idx_span_30): min(num_readings, idx_zero + idx_span_85)]
        right = ranges[max(0, idx_zero - idx_span_85): max(0, idx_zero - idx_span_30)]

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
            # Sampel area tengah (40x40 piksel)
            center_crop = depth_img[max(0, ch-20): min(h, ch+20), max(0, cw-20): min(w, cw+20)]
            valid = center_crop[np.isfinite(center_crop) & (center_crop > 0.1)]
            if len(valid) > 0:
                self.min_depth_cam_dist = float(np.min(valid))
            else:
                self.min_depth_cam_dist = 10.0
        except Exception:
            pass

    def control_loop(self):
        cmd = Twist()

        # Jarak efektif depan adalah minimum dari laser depan dan kamera kedalaman
        effective_front = min(self.min_front_dist, self.min_depth_cam_dist)

        # 1. Mode Darurat (Mundur jika terlalu mepet di sudut)
        if self.escape_steps > 0:
            cmd.linear.x = -0.12
            cmd.angular.z = self.turn_direction * 0.45
            self.escape_steps -= 1
            self.cmd_pub.publish(cmd)
            return

        if effective_front < self.emergency_threshold or (self.min_left_dist < self.emergency_threshold and self.min_right_dist < self.emergency_threshold):
            self.escape_steps = 15 # Mundur selama 1.5 detik
            self.turn_direction = 1.0 if self.min_left_dist > self.min_right_dist else -1.0
            cmd.linear.x = -0.12
            cmd.angular.z = self.turn_direction * 0.45
            self.cmd_pub.publish(cmd)
            return

        # 2. Mode Belok Menghindar
        if effective_front < self.safe_threshold:
            # Pilih arah dengan clearance terjauh
            if self.min_left_dist >= self.min_right_dist:
                cmd.linear.x = 0.05
                cmd.angular.z = self.turn_speed
            else:
                cmd.linear.x = 0.05
                cmd.angular.z = -self.turn_speed
        elif self.min_left_dist < 0.45:
            # Dinding kiri terlalu dekat, serong kanan
            cmd.linear.x = self.target_linear_speed * 0.7
            cmd.angular.z = -0.25
        elif self.min_right_dist < 0.45:
            # Dinding kanan terlalu dekat, serong kiri
            cmd.linear.x = self.target_linear_speed * 0.7
            cmd.angular.z = 0.25
        else:
            # 3. Mode Maju Lurus Bebas
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
