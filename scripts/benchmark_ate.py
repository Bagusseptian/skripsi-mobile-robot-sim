#!/usr/bin/env python3
"""
benchmark_ate.py
================
Menghitung Absolute Trajectory Error (ATE) antara Ground Truth Gazebo (/ground_truth/odom)
dan estimasi pose dari SLAM (TF map -> base_footprint atau topik estimasi odometri/pose).
Metrik evaluasi sesuai Bagian 7 Project Plan Tahap Simulasi:
- Root Mean Square Error (RMSE)
- Mean Error
- Maximum Error
- Standard Deviation Error
- Ekspor log CSV dan plot kurva lintasan (PNG)
"""

import sys
import os
import time
import math
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros

try:
    import matplotlib
    matplotlib.use('Agg') # Headless backend
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

class ATEEvaluator(Node):
    def __init__(self, mode='tf', est_topic='/odom', output_dir='eval_results'):
        super().__init__('benchmark_ate_node')
        self.mode = mode
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.gt_poses = []   # list of (t, x, y, yaw)
        self.est_poses = []  # list of (t, x, y, yaw)

        # Ground truth subscriber dari Gazebo P3D
        self.create_subscription(Odometry, '/ground_truth/odom', self.gt_callback, 10)
        self.last_gt = None

        if self.mode == 'topic':
            self.create_subscription(Odometry, est_topic, self.est_callback, 10)
            self.get_logger().info(f"[ATE Evaluator] Mode: Topic ({est_topic}) vs Ground Truth")
        else:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.timer = self.create_timer(0.1, self.tf_timer_callback) # 10 Hz sampling
            self.get_logger().info("[ATE Evaluator] Mode: TF (map -> base_footprint) vs Ground Truth")

        self.start_time = self.get_clock().now()

    def gt_callback(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Konversi quaternion ke yaw
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.last_gt = (t, pos.x, pos.y, yaw)

    def est_callback(self, msg: Odometry):
        if self.last_gt is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.gt_poses.append(self.last_gt)
        self.est_poses.append((t, pos.x, pos.y, yaw))

    def tf_timer_callback(self):
        if self.last_gt is None:
            return
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            t = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
            pos = trans.transform.translation
            rot = trans.transform.rotation
            siny_cosp = 2 * (rot.w * rot.z + rot.x * rot.y)
            cosy_cosp = 1 - 2 * (rot.y * rot.y + rot.z * rot.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            self.gt_poses.append(self.last_gt)
            self.est_poses.append((t, pos.x, pos.y, yaw))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    def compute_and_save_results(self, exp_name='slam_test'):
        if len(self.gt_poses) < 5:
            self.get_logger().warn("Data lintasan tidak mencukupi untuk evaluasi (minimal 5 sampel).")
            return

        gt_arr = np.array(self.gt_poses)
        est_arr = np.array(self.est_poses)

        # Hitung translational error Euclidean
        dx = gt_arr[:, 1] - est_arr[:, 1]
        dy = gt_arr[:, 2] - est_arr[:, 2]
        errors = np.sqrt(dx**2 + dy**2)

        rmse = np.sqrt(np.mean(errors**2))
        mean_err = np.mean(errors)
        max_err = np.max(errors)
        std_err = np.std(errors)

        timestamp_str = time.strftime("%Y%m%d_%H%M%S")
        report_file = os.path.join(self.output_dir, f"ate_report_{exp_name}_{timestamp_str}.txt")
        csv_file = os.path.join(self.output_dir, f"ate_data_{exp_name}_{timestamp_str}.csv")
        plot_file = os.path.join(self.output_dir, f"ate_trajectory_{exp_name}_{timestamp_str}.png")

        # Simpan CSV
        with open(csv_file, 'w') as f:
            f.write("time_gt,x_gt,y_gt,yaw_gt,time_est,x_est,y_est,yaw_est,error_m\n")
            for i in range(len(errors)):
                f.write(f"{gt_arr[i,0]:.3f},{gt_arr[i,1]:.4f},{gt_arr[i,2]:.4f},{gt_arr[i,3]:.4f},"
                        f"{est_arr[i,0]:.3f},{est_arr[i,1]:.4f},{est_arr[i,2]:.4f},{est_arr[i,3]:.4f},"
                        f"{errors[i]:.4f}\n")

        summary = f"""
============================================================
       HASIL EVALUASI AKURASI LOKALISASI SLAM (ATE)
============================================================
Eksperimen            : {exp_name}
Jumlah Sampel         : {len(errors)}
Metrik Translasi:
  - Absolute Error RMSE : {rmse:.4f} meter
  - Mean Error          : {mean_err:.4f} meter
  - Max Error           : {max_err:.4f} meter
  - Standard Deviation  : {std_err:.4f} meter
File Data CSV         : {csv_file}
File Gambar Plot      : {plot_file}
============================================================
"""
        print(summary)
        with open(report_file, 'w') as f:
            f.write(summary)

        # Plot perbandingan jika matplotlib tersedia
        if HAS_MATPLOTLIB:
            plt.figure(figsize=(9, 7))
            plt.plot(gt_arr[:, 1], gt_arr[:, 2], 'g-', linewidth=2.0, label='Ground Truth (Gazebo P3D)')
            plt.plot(est_arr[:, 1], est_arr[:, 2], 'b--', linewidth=1.8, label=f'Estimated Trajectory ({exp_name})')
            plt.title(f"Perbandingan Lintasan: Ground Truth vs {exp_name}\n(ATE RMSE: {rmse:.4f} m)")
            plt.xlabel("X (meter)")
            plt.ylabel("Y (meter)")
            plt.legend()
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.axis('equal')
            plt.tight_layout()
            plt.savefig(plot_file, dpi=150)
            plt.close()
            print(f"[Grafik Disimpan] {plot_file}")

def main():
    parser = argparse.ArgumentParser(description="Evaluasi ATE (Absolute Trajectory Error) ROS 2")
    parser.add_argument('--name', default='slam_eval', help='Nama eksperimen evaluasi')
    parser.add_argument('--mode', default='tf', choices=['tf', 'topic'], help='Sumber estimasi pose')
    parser.add_argument('--topic', default='/odom', help='Topik estimasi jika mode=topic')
    parser.add_argument('--duration', type=int, default=60, help='Durasi perekaman data (detik)')
    parser.add_argument('--output-dir', default='/home/sugab/skripsi_ws/eval_results', help='Direktori output')

    args = parser.parse_args()

    rclpy.init()
    node = ATEEvaluator(mode=args.mode, est_topic=args.topic, output_dir=args.output_dir)

    print(f"\n[ATE Benchmark] Perekaman dimulai selama {args.duration} detik...")
    print("Gerakkan robot memakai teleop_wasd atau Nav2...")
    
    start_t = time.time()
    try:
        while rclpy.ok() and (time.time() - start_t < args.duration):
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = int(time.time() - start_t)
            print(f"\rMerekam... {elapsed}/{args.duration} detik | Sampel terkumpul: {len(node.gt_poses)}   ", end='', flush=True)
    except KeyboardInterrupt:
        print("\nPerekaman dihentikan lebih awal oleh pengguna.")

    print("\nMenghitung metrik ATE dan menyimpan hasil...")
    node.compute_and_save_results(exp_name=args.name)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
