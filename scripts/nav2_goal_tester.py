#!/usr/bin/env python3
"""
nav2_goal_tester.py
===================
Penguji Navigasi Otonom Nav2 Otomatis (Bagian 7 Project Plan Tahap Simulasi).
- Mengirim serangkaian waypoint / goal pose ke Nav2 menggunakan BasicNavigator
- Mengukur:
  * Tingkat Keberhasilan Navigasi (%)
  * Waktu Tempuh per Titik Tujuan (detik)
  * Status Keberhasilan (Succeeded, Failed, Aborted)
- Mengekspor data ke file CSV dan laporan teks untuk Bab Hasil Skripsi.
"""

import os
import sys
import time
import math
import argparse
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Titik uji default untuk World A (Basic Arena 8x8m)
WAYPOINTS_WORLD_A = [
    {'name': 'Goal 1 (Pojok Kanan Depan)', 'x': 2.5, 'y': 2.5, 'yaw': 0.78},
    {'name': 'Goal 2 (Pojok Kiri Depan)',  'x': -2.5, 'y': 2.5, 'yaw': 2.35},
    {'name': 'Goal 3 (Pojok Kiri Belakang)','x': -2.5, 'y': -2.5, 'yaw': -2.35},
    {'name': 'Goal 4 (Pojok Kanan Belakang)','x': 2.5, 'y': -2.5, 'yaw': -0.78},
    {'name': 'Goal 5 (Kembali ke Titik Asal)','x': 0.0, 'y': 0.0, 'yaw': 0.0}
]

# Titik uji default untuk World B (Corridor 14x10m) - sirkuit lorong berurutan
WAYPOINTS_WORLD_B = [
    {'name': 'Goal 1 (Ujung Lorong Timur)',   'x': 5.2, 'y': -3.8, 'yaw': 0.0},
    {'name': 'Goal 2 (Belok Lorong Utara)',   'x': 5.8, 'y': 3.5,  'yaw': 1.57},
    {'name': 'Goal 3 (Ujung Lorong Barat)',   'x': -5.8, 'y': 3.5, 'yaw': 3.14},
    {'name': 'Goal 4 (Kembali Lorong Selatan)','x': -5.8, 'y': -3.8, 'yaw': -1.57},
    {'name': 'Goal 5 (Kembali Titik Start)',   'x': 0.0, 'y': -3.8, 'yaw': 0.0}
]

# Titik uji default untuk World C (Classroom 12x10m)
WAYPOINTS_WORLD_C = [
    {'name': 'Goal 1 (Depan Papan Tulis)', 'x': 0.0, 'y': 3.5, 'yaw': 1.57},
    {'name': 'Goal 2 (Meja Lab Kanan 1)',   'x': 4.2, 'y': 1.0, 'yaw': 0.0},
    {'name': 'Goal 3 (Lorong Belakang Kanan)','x': 4.2, 'y': -3.5, 'yaw': -1.57},
    {'name': 'Goal 4 (Lorong Belakang Kiri)', 'x': -4.2, 'y': -3.5, 'yaw': 3.14},
    {'name': 'Goal 5 (Kembali Depan Kelas)', 'x': 0.0, 'y': 1.5, 'yaw': 1.57}
]

def run_navigation_test(world='world_a', exp_name='nav2_eval', localizer='bt_navigator', output_dir='/home/sugab/skripsi_ws/eval_results'):
    sys.stdout.reconfigure(line_buffering=True)
    os.makedirs(output_dir, exist_ok=True)
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_file = os.path.join(output_dir, f"nav_benchmark_{exp_name}_{timestamp_str}.csv")
    report_file = os.path.join(output_dir, f"nav_report_{exp_name}_{timestamp_str}.txt")

    rclpy.init()
    navigator = BasicNavigator()
    # Pastikan node penguji menggunakan clock simulasi jika Gazebo aktif
    try:
        navigator.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    except Exception:
        pass

    print("\n[Nav2 Tester] Menunggu Nav2 stack aktif sepenuhnya...", flush=True)
    try:
        navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='bt_navigator')
    except Exception as e:
        print(f"[Nav2 Tester] Warning kesiapan: {e}, melanjutkan...", flush=True)
    print("[Nav2 Tester] Nav2 siap menerima target goal.", flush=True)

    if 'b' in world:
        waypoints = WAYPOINTS_WORLD_B
    elif 'c' in world:
        waypoints = WAYPOINTS_WORLD_C
    else:
        waypoints = WAYPOINTS_WORLD_A

    results = []
    total_goals = len(waypoints)
    succeeded_goals = 0

    with open(csv_file, 'w') as f:
        f.write("goal_index,goal_name,x,y,status,duration_sec\n")

    print(f"\n============================================================")
    print(f"       PENGUJIAN NAVIGASI OTONOM ({world.upper()})")
    print(f"============================================================")
    print(f"Jumlah Titik Target: {total_goals}")

    for idx, wp in enumerate(waypoints, start=1):
        print(f"\n---> Mengirim Target {idx}/{total_goals}: {wp['name']} (x={wp['x']}, y={wp['y']})")
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = wp['x']
        goal_pose.pose.position.y = wp['y']
        goal_pose.pose.position.z = 0.0
        
        # Orientasi dasar
        goal_pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal_pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        start_time = time.time()
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                dist = feedback.distance_remaining
                print(f"\r  Jarak tersisa: {dist:.2f} m | Berjalan: {time.time()-start_time:.1f}s   ", end='', flush=True)
            if time.time() - start_time > 60.0:
                print(f"\n  [TIMEOUT] Batas waktu tercapai (60s), membatalkan target {idx}...", flush=True)
                navigator.cancelTask()
                break
            time.sleep(0.5)

        duration = time.time() - start_time
        task_res = navigator.getResult()

        if task_res == TaskResult.SUCCEEDED:
            status_str = "SUCCEEDED"
            succeeded_goals += 1
            print(f"\n  [BERHASIL] Titik {idx} tercapai dalam {duration:.2f} detik.")
        elif task_res == TaskResult.CANCELED:
            status_str = "CANCELED"
            print(f"\n  [DIBATALKAN] Titik {idx} dibatalkan setelah {duration:.2f} detik.")
        elif task_res == TaskResult.FAILED:
            status_str = "FAILED"
            print(f"\n  [GAGAL] Navigasi ke Titik {idx} gagal ({duration:.2f}s).")
        else:
            status_str = "TIMEOUT/UNKNOWN"
            print(f"\n  [TIMEOUT] Respon tidak diketahui ({duration:.2f}s).")

        results.append({
            'idx': idx,
            'name': wp['name'],
            'x': wp['x'],
            'y': wp['y'],
            'status': status_str,
            'duration': duration
        })

        with open(csv_file, 'a') as f:
            f.write(f"{idx},{wp['name']},{wp['x']},{wp['y']},{status_str},{duration:.2f}\n")

        time.sleep(1.0) # Jeda antar waypoint

    success_rate = (succeeded_goals / total_goals) * 100.0
    durations = [r['duration'] for r in results if r['status'] == 'SUCCEEDED']
    avg_duration = (sum(durations) / len(durations)) if durations else 0.0

    summary = f"""
============================================================
           RINGKASAN EVALUASI NAVIGASI OTONOM
============================================================
Eksperimen            : {exp_name}
Dunia Pengujian       : {world}
Total Titik Target    : {total_goals}
Titik Berhasil (Goal) : {succeeded_goals}
Tingkat Keberhasilan  : {success_rate:.1f}%
Rata-rata Waktu Capai : {avg_duration:.2f} detik per goal

File Log CSV          : {csv_file}
============================================================
"""
    print(summary)
    with open(report_file, 'w') as f:
        f.write(summary)

    rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Penguji Navigasi Otonom Nav2")
    parser.add_argument('--world', default='world_a', choices=['world_a', 'world_b', 'world_c'], help='Skenario dunia yang aktif')
    parser.add_argument('--name', default='nav_test', help='Nama pengujian')
    parser.add_argument('--localizer', default='bt_navigator', help='Lifecycle node acuan kesiapan (default: bt_navigator, atau amcl)')
    parser.add_argument('--output-dir', default='/home/sugab/skripsi_ws/eval_results', help='Folder output')
    args, _ = parser.parse_known_args()

    run_navigation_test(world=args.world, exp_name=args.name, localizer=args.localizer, output_dir=args.output_dir)

if __name__ == '__main__':
    main()
