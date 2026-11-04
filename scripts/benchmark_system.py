#!/usr/bin/env python3
"""
benchmark_system.py
===================
Memantau beban komputasi (Penggunaan CPU % dan RAM MB) sistem simulasi dan SLAM
sesuai Bagian 7 Project Plan:
- Penggunaan CPU (%) dan RAM (MB) untuk Gazebo, SLAM (slam_toolbox / rtabmap), Nav2, dan RViz
- Menghasilkan ringkasan rata-rata dan beban puncak (peak)
- Mengekspor data berkala ke format CSV untuk analisis skripsi
"""

import os
import sys
import time
import argparse
import psutil

TARGET_KEYWORDS = [
    'gzserver',
    'gzclient',
    'async_slam',
    'rtabmap',
    'bt_navigator',
    'controller_server',
    'planner_server',
    'rviz2'
]

def find_processes(keywords):
    proc_map = {}
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmd = " ".join(proc.info['cmdline'] or [])
            for kw in keywords:
                if kw in proc.info['name'] or kw in cmd:
                    label = kw
                    if 'async_slam' in cmd:
                        label = 'slam_toolbox'
                    elif 'rtabmap' in cmd:
                        label = 'rtabmap'
                    proc_map[proc.pid] = (label, proc)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return proc_map

def monitor(duration=60, interval=1.0, exp_name='system_eval', output_dir='/home/sugab/skripsi_ws/eval_results'):
    os.makedirs(output_dir, exist_ok=True)
    timestamp_str = time.strftime("%Y%m%d_%H%M%S")
    csv_file = os.path.join(output_dir, f"system_load_{exp_name}_{timestamp_str}.csv")
    report_file = os.path.join(output_dir, f"system_report_{exp_name}_{timestamp_str}.txt")

    print(f"\n============================================================")
    print(f"       BENCHMARK BEBAN KOMPUTASI SISTEM (CPU & RAM)")
    print(f"============================================================")
    print(f"Eksperimen : {exp_name}")
    print(f"Durasi     : {duration} detik (sampling tiap {interval} s)")
    print(f"Output     : {csv_file}")
    print(f"============================================================\n")

    records = []
    start_time = time.time()

    with open(csv_file, 'w') as f:
        f.write("timestamp_sec,total_cpu_pct,total_ram_mb,gazebo_cpu,gazebo_ram_mb,slam_cpu,slam_ram_mb,nav2_cpu,nav2_ram_mb\n")

    try:
        while time.time() - start_time < duration:
            t_now = time.time() - start_time
            procs = find_processes(TARGET_KEYWORDS)

            cat_stats = {
                'gazebo': {'cpu': 0.0, 'ram': 0.0},
                'slam': {'cpu': 0.0, 'ram': 0.0},
                'nav2': {'cpu': 0.0, 'ram': 0.0},
                'rviz': {'cpu': 0.0, 'ram': 0.0}
            }

            for pid, (label, proc) in procs.items():
                try:
                    cpu = proc.cpu_percent(interval=None)
                    ram = proc.memory_info().rss / (1024 * 1024) # MB
                    if 'gz' in label:
                        cat_stats['gazebo']['cpu'] += cpu
                        cat_stats['gazebo']['ram'] += ram
                    elif 'slam' in label or 'rtabmap' in label:
                        cat_stats['slam']['cpu'] += cpu
                        cat_stats['slam']['ram'] += ram
                    elif 'nav' in label or 'controller' in label or 'planner' in label:
                        cat_stats['nav2']['cpu'] += cpu
                        cat_stats['nav2']['ram'] += ram
                    elif 'rviz' in label:
                        cat_stats['rviz']['cpu'] += cpu
                        cat_stats['rviz']['ram'] += ram
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            tot_cpu = psutil.cpu_percent()
            tot_ram = psutil.virtual_memory().used / (1024 * 1024)

            records.append({
                't': t_now,
                'tot_cpu': tot_cpu,
                'tot_ram': tot_ram,
                'gz_cpu': cat_stats['gazebo']['cpu'],
                'gz_ram': cat_stats['gazebo']['ram'],
                'slam_cpu': cat_stats['slam']['cpu'],
                'slam_ram': cat_stats['slam']['ram'],
                'nav_cpu': cat_stats['nav2']['cpu'],
                'nav_ram': cat_stats['nav2']['ram']
            })

            with open(csv_file, 'a') as f:
                f.write(f"{t_now:.1f},{tot_cpu:.1f},{tot_ram:.1f},"
                        f"{cat_stats['gazebo']['cpu']:.1f},{cat_stats['gazebo']['ram']:.1f},"
                        f"{cat_stats['slam']['cpu']:.1f},{cat_stats['slam']['ram']:.1f},"
                        f"{cat_stats['nav2']['cpu']:.1f},{cat_stats['nav2']['ram']:.1f}\n")

            print(f"\r[{t_now:5.1f}s] Total CPU: {tot_cpu:4.1f}% | SLAM CPU: {cat_stats['slam']['cpu']:4.1f}% | SLAM RAM: {cat_stats['slam']['ram']:5.1f}MB | Gazebo CPU: {cat_stats['gazebo']['cpu']:4.1f}%   ", end='', flush=True)
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nMonitoring dihentikan lebih awal.")

    if not records:
        print("\nTidak ada data yang terekam.")
        return

    # Hitung ringkasan rata-rata dan puncak
    avg_slam_cpu = sum(r['slam_cpu'] for r in records) / len(records)
    peak_slam_cpu = max(r['slam_cpu'] for r in records)
    avg_slam_ram = sum(r['slam_ram'] for r in records) / len(records)
    peak_slam_ram = max(r['slam_ram'] for r in records)

    avg_gz_cpu = sum(r['gz_cpu'] for r in records) / len(records)
    avg_gz_ram = sum(r['gz_ram'] for r in records) / len(records)

    avg_sys_cpu = sum(r['tot_cpu'] for r in records) / len(records)

    summary = f"""
============================================================
           RINGKASAN BEBAN KOMPUTASI SISTEM
============================================================
Eksperimen              : {exp_name}
Jumlah Sampel           : {len(records)} detik
Rata-rata CPU Total     : {avg_sys_cpu:.2f}%

Beban Node SLAM ({exp_name}):
  - Rata-rata Penggunaan CPU : {avg_slam_cpu:.2f}%
  - Beban Puncak (Peak) CPU  : {peak_slam_cpu:.2f}%
  - Rata-rata RAM Digunakan  : {avg_slam_ram:.2f} MB
  - Beban Puncak (Peak) RAM  : {peak_slam_ram:.2f} MB

Beban Simulator Gazebo:
  - Rata-rata CPU Gazebo     : {avg_gz_cpu:.2f}%
  - Rata-rata RAM Gazebo     : {avg_gz_ram:.2f} MB

File Data Lengkap       : {csv_file}
============================================================
"""
    print(summary)
    with open(report_file, 'w') as f:
        f.write(summary)
    print(f"Laporan beban komputasi tersimpan di: {report_file}")

def main():
    parser = argparse.ArgumentParser(description="Monitor Beban Komputasi CPU dan RAM")
    parser.add_argument('--name', default='slam_load_test', help='Nama skenario pengujian')
    parser.add_argument('--duration', type=int, default=60, help='Durasi monitoring dalam detik')
    parser.add_argument('--interval', type=float, default=1.0, help='Interval sampling detik')
    parser.add_argument('--output-dir', default='/home/sugab/skripsi_ws/eval_results', help='Folder output data')
    args = parser.parse_args()

    monitor(duration=args.duration, interval=args.interval, exp_name=args.name, output_dir=args.output_dir)

if __name__ == '__main__':
    main()
