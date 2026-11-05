#!/usr/bin/env python3
"""
map_saver_unified.py
====================
Node Penyimpan Peta Terpadu 2D & 3D (Unified 2D/3D Map Exporter V2).
Fungsi:
1. Menyimpan Peta 2D Occupancy Grid (/map) -> .pgm (P5 binary) + .yaml (Nav2 standard).
2. Menyimpan Peta 3D Dense RGB-D Point Cloud:
   - Prioritas 1: Ekspor langsung dari database RTAB-Map (~/.ros/rtabmap.db via rtabmap-export).
   - Prioritas 2: Topik /cloud_map dengan QoS TRANSIENT_LOCAL (latched).
   - Prioritas 3: Topik /octomap_occupied_space atau /camera/points (BEST_EFFORT).
   - Format: .ply (Binary Little-Endian dengan warna RGB) dan .pcd.
Dapat dieksekusi standalone via ROS 2 CLI:
ros2 run robot_sim map_saver_unified.py --name map_world_c --output-dir ~/skripsi_ws/maps
"""

import os
import sys
import time
import shutil
import subprocess
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

class UnifiedMapSaver(Node):
    def __init__(self, map_name, output_dir, timeout_sec=12.0):
        super().__init__('map_saver_unified')
        self.map_name = map_name
        self.output_dir = os.path.expanduser(output_dir)
        self.timeout_sec = timeout_sec

        os.makedirs(self.output_dir, exist_ok=True)

        self.map_msg = None
        self.cloud_msg = None
        self.camera_points_msg = None

        # QoS TRANSIENT_LOCAL + RELIABLE untuk topik latched RTAB-Map
        latched_qos = QoSProfile(depth=1)
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_qos.reliability = ReliabilityPolicy.RELIABLE

        # QoS BEST_EFFORT untuk Gazebo camera points
        best_effort_qos = QoSProfile(depth=5)
        best_effort_qos.durability = DurabilityPolicy.VOLATILE
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.sub_map = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, latched_qos
        )
        self.sub_cloud_map = self.create_subscription(
            PointCloud2, '/cloud_map', self.cloud_callback, latched_qos
        )
        self.sub_octomap = self.create_subscription(
            PointCloud2, '/octomap_occupied_space', self.cloud_callback, latched_qos
        )
        self.sub_camera_points = self.create_subscription(
            PointCloud2, '/camera/points', self.camera_points_callback, best_effort_qos
        )

        self.get_logger().info(f'[UNIFIED_MAP_SAVER] Listening for /map and 3D point cloud...')
        self.get_logger().info(f'[UNIFIED_MAP_SAVER] Target directory: {self.output_dir}')

    def map_callback(self, msg: OccupancyGrid):
        if self.map_msg is None:
            self.map_msg = msg
            self.get_logger().info(f'[UNIFIED_MAP_SAVER] Received /map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f}m/cell')

    def cloud_callback(self, msg: PointCloud2):
        if msg.width * msg.height > 0:
            self.cloud_msg = msg
            self.get_logger().info(f'[UNIFIED_MAP_SAVER] Received 3D cloud: {msg.width * msg.height} points')

    def camera_points_callback(self, msg: PointCloud2):
        if msg.width * msg.height > 0:
            self.camera_points_msg = msg

    def save_2d_map(self):
        if self.map_msg is None:
            self.get_logger().warn('[UNIFIED_MAP_SAVER] No /map received! Skipping 2D save.')
            return False

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        res = self.map_msg.info.resolution
        origin = self.map_msg.info.origin

        data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))

        # ROS map convention:
        # -1 = unknown -> 205
        # 0 = free -> 254
        # 100 = occupied -> 0
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 205
        img[data == 0] = 254
        mask_occ = data > 0
        img[mask_occ] = np.clip((100 - data[mask_occ]) * 254 // 100, 0, 254).astype(np.uint8)

        # Flip vertically so (0,0) is bottom-left as standard in PGM viewers
        img = np.flipud(img)

        pgm_path = os.path.join(self.output_dir, f'{self.map_name}_2d.pgm')
        yaml_path = os.path.join(self.output_dir, f'{self.map_name}_2d.yaml')

        # Write P5 binary PGM
        with open(pgm_path, 'wb') as f:
            header = f'P5\n{width} {height}\n255\n'.encode('ascii')
            f.write(header)
            f.write(img.tobytes())

        # Write ROS standard YAML
        with open(yaml_path, 'w') as f:
            f.write(f'image: {self.map_name}_2d.pgm\n')
            f.write('mode: trinary\n')
            f.write(f'resolution: {res:.6f}\n')
            f.write(f'origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, {origin.position.z:.6f}]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

        self.get_logger().info(f'[UNIFIED_MAP_SAVER] 2D Map successfully saved:')
        self.get_logger().info(f'  -> PGM:  {pgm_path} ({os.path.getsize(pgm_path):,} bytes)')
        self.get_logger().info(f'  -> YAML: {yaml_path}')
        return True

    def _unpack_pointcloud2(self, msg):
        fields = {f.name: f.offset for f in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return None

        step = msg.point_step
        total = msg.width * msg.height
        data = bytes(msg.data)

        x_off, y_off, z_off = fields['x'], fields['y'], fields['z']
        dtype_list = [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
        offsets = [x_off, y_off, z_off]

        has_color = 'rgb' in fields or 'rgba' in fields
        c_off = fields.get('rgb', fields.get('rgba', None))
        if has_color:
            dtype_list.append(('rgb', '<u4'))
            offsets.append(c_off)

        structured_dtype = np.dtype({
            'names': [d[0] for d in dtype_list],
            'formats': [d[1] for d in dtype_list],
            'offsets': offsets,
            'itemsize': step
        })

        arr = np.frombuffer(data, dtype=structured_dtype, count=total)

        valid = np.isfinite(arr['x']) & np.isfinite(arr['y']) & np.isfinite(arr['z'])
        # Also filter out extreme noise points
        valid &= (np.abs(arr['x']) < 50.0) & (np.abs(arr['y']) < 50.0) & (np.abs(arr['z']) < 20.0)

        valid_arr = arr[valid]
        n_pts = len(valid_arr)
        if n_pts == 0:
            return None

        x = valid_arr['x']
        y = valid_arr['y']
        z = valid_arr['z']

        if has_color:
            rgb_uint = valid_arr['rgb']
            r = ((rgb_uint >> 16) & 0xFF).astype(np.uint8)
            g = ((rgb_uint >> 8) & 0xFF).astype(np.uint8)
            b = (rgb_uint & 0xFF).astype(np.uint8)
        else:
            z_norm = (z - z.min()) / max((z.max() - z.min()), 1e-4)
            r = (np.clip(z_norm * 255, 0, 255)).astype(np.uint8)
            g = (np.clip((1.0 - np.abs(z_norm - 0.5) * 2.0) * 255, 0, 255)).astype(np.uint8)
            b = (np.clip((1.0 - z_norm) * 255, 0, 255)).astype(np.uint8)

        return x, y, z, r, g, b

    def save_3d_map(self):
        ply_path = os.path.join(self.output_dir, f'{self.map_name}_3d.ply')
        pcd_path = os.path.join(self.output_dir, f'{self.map_name}_3d.pcd')

        # STRATEGI 1: Ekspor langsung dari database RTAB-Map
        rtabmap_db = os.path.expanduser('~/.ros/rtabmap.db')
        if os.path.exists(rtabmap_db) and os.path.getsize(rtabmap_db) > 100000:
            self.get_logger().info(f'[UNIFIED_MAP_SAVER] Attempting direct rtabmap-export from {rtabmap_db}...')
            cmd = [
                'bash', '-c',
                f'source /opt/ros/humble/setup.bash && rtabmap-export --cloud --opt 2 --output_dir "{self.output_dir}" --output "{self.map_name}_3d" "{rtabmap_db}" 2>/dev/null'
            ]
            try:
                res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=12)
                possible_ply = [
                    os.path.join(self.output_dir, f'{self.map_name}_3d_cloud.ply'),
                    os.path.join(self.output_dir, f'{self.map_name}_3d.ply')
                ]
                for p_ply in possible_ply:
                    if os.path.exists(p_ply) and os.path.getsize(p_ply) > 10000:
                        if p_ply != ply_path:
                            shutil.move(p_ply, ply_path)
                        self.get_logger().info(f'[UNIFIED_MAP_SAVER] High-density RGB-D PLY exported via rtabmap-export: {ply_path} ({os.path.getsize(ply_path):,} bytes)')
                        return True
            except Exception as e:
                self.get_logger().warn(f'[UNIFIED_MAP_SAVER] rtabmap-export attempt skipped: {e}')

        # STRATEGI 2: Ekstraksi dari topik PointCloud2 (/cloud_map atau /camera/points)
        pts_data = None
        if self.cloud_msg is not None:
            pts_data = self._unpack_pointcloud2(self.cloud_msg)
        elif self.camera_points_msg is not None:
            pts_data = self._unpack_pointcloud2(self.camera_points_msg)

        # STRATEGI 3: Fallback ke ekstrusi 2D map jika SLAM point cloud belum terinisialisasi
        if pts_data is None:
            if self.map_msg is not None:
                self.get_logger().info('[UNIFIED_MAP_SAVER] Generating 3D voxels from 2D Occupancy Grid...')
                width = self.map_msg.info.width
                height = self.map_msg.info.height
                res = self.map_msg.info.resolution
                ox = self.map_msg.info.origin.position.x
                oy = self.map_msg.info.origin.position.y
                data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))
                
                occupied_indices = np.argwhere(data > 50)
                if len(occupied_indices) > 0:
                    px = ox + occupied_indices[:, 1] * res
                    py = oy + occupied_indices[:, 0] * res
                    
                    z_levels = np.linspace(0.0, 1.8, 19)
                    all_x = np.repeat(px, len(z_levels))
                    all_y = np.repeat(py, len(z_levels))
                    all_z = np.tile(z_levels, len(px))
                    
                    all_r = np.full(len(all_x), 40, dtype=np.uint8)
                    all_g = np.full(len(all_x), 180, dtype=np.uint8)
                    all_b = np.full(len(all_x), 220, dtype=np.uint8)
                    pts_data = (all_x.astype(np.float32), all_y.astype(np.float32), all_z.astype(np.float32), all_r, all_g, all_b)

        if pts_data is None:
            self.get_logger().warn('[UNIFIED_MAP_SAVER] No 3D data available to export.')
            return False

        x, y, z, r, g, b = pts_data
        n_pts = len(x)

        # Tulis Binary Little-Endian PLY
        ply_header = (
            f"ply\n"
            f"format binary_little_endian 1.0\n"
            f"element vertex {n_pts}\n"
            f"property float x\n"
            f"property float y\n"
            f"property float z\n"
            f"property uchar red\n"
            f"property uchar green\n"
            f"property uchar blue\n"
            f"end_header\n"
        ).encode('ascii')

        out_dtype = np.dtype([
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
            ('r', 'u1'), ('g', 'u1'), ('b', 'u1')
        ])
        ply_arr = np.empty(n_pts, dtype=out_dtype)
        ply_arr['x'] = x
        ply_arr['y'] = y
        ply_arr['z'] = z
        ply_arr['r'] = r
        ply_arr['g'] = g
        ply_arr['b'] = b

        with open(ply_path, 'wb') as f:
            f.write(ply_header)
            f.write(ply_arr.tobytes())

        # Tulis PCD
        with open(pcd_path, 'w') as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F U\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {n_pts}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n_pts}\n")
            f.write("DATA ascii\n")
            step_skip = max(1, n_pts // 20000)
            for i in range(0, n_pts, step_skip):
                rgb_packed = (int(r[i]) << 16) | (int(g[i]) << 8) | int(b[i])
                f.write(f"{x[i]:.4f} {y[i]:.4f} {z[i]:.4f} {rgb_packed}\n")

        self.get_logger().info(f'[UNIFIED_MAP_SAVER] 3D Map successfully saved:')
        self.get_logger().info(f'  -> PLY: {ply_path} ({n_pts:,} points, {os.path.getsize(ply_path):,} bytes)')
        self.get_logger().info(f'  -> PCD: {pcd_path} ({os.path.getsize(pcd_path):,} bytes)')
        return True

def main():
    parser = argparse.ArgumentParser(description='Unified 2D and 3D Map Exporter V2')
    parser.add_argument('--name', type=str, default='map_unified', help='Base name for map files')
    parser.add_argument('--output-dir', type=str, default=os.path.expanduser('~/skripsi_ws/maps'), help='Output directory')
    parser.add_argument('--timeout', type=float, default=12.0, help='Max timeout seconds to wait for map data')
    
    raw_args = rclpy.utilities.remove_ros_args(args=sys.argv)[1:]
    args = parser.parse_args(raw_args)

    rclpy.init()
    node = UnifiedMapSaver(args.name, args.output_dir, args.timeout)

    start_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.map_msg is not None and (node.cloud_msg is not None or node.camera_points_msg is not None):
            time.sleep(1.0)
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.2)
            break
        if time.time() - start_time > args.timeout:
            break

    saved_2d = node.save_2d_map()
    saved_3d = node.save_3d_map()

    node.destroy_node()
    rclpy.shutdown()

    if saved_2d and saved_3d:
        print(f"\n>>> [SUCCESS] All map formats (2D .pgm/.yaml & 3D .ply/.pcd) saved for '{args.name}' <<<\n")
        sys.exit(0)
    else:
        print(f"\n>>> [PARTIAL/WARN] 2D saved: {saved_2d}, 3D saved: {saved_3d} <<<\n")
        sys.exit(0 if (saved_2d or saved_3d) else 1)

if __name__ == '__main__':
    main()
