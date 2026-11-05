#!/usr/bin/env python3
"""
rgbd_spatial_interactive.py
============================
Node Interaktif Penginderaan Spasial 3D Kamera RGB-D Orbbec Astra Pro.
Fitur Unggulan untuk Praktikum & Demonstrasi Skripsi:
1. Interactive Point Inspector (RViz Publish Point Tool):
   - Mahasiswa mengklik sembarang titik/objek pada visualisasi 3D RViz2
   - Menghitung koordinat 3D (X, Y, Z), jarak riil dari robot, dan estimasi jenis objek
   - Menampilkan Marker 3D teks melayang (Billboard) dan sphere penanda di RViz2
2. Real-Time Obstacle Distance Meter:
   - Mengukur jarak rintangan tepat di depan mata kamera robot (Center Crosshair Depth)
   - Menerbitkan topik /camera/detected_obstacle_distance (Float32)
"""

import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False

class RGBDSpatialInteractive(Node):
    def __init__(self):
        super().__init__('rgbd_spatial_interactive')
        
        # TF Buffer untuk transformasi koordinat
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriber klik interaktif dari RViz2 (Tool "Publish Point")
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        
        # Subscriber citra kedalaman untuk HUD jarak rintangan depan
        if HAS_CV_BRIDGE:
            self.bridge = CvBridge()
            self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        # Publisher marker 3D interaktif di RViz2
        self.marker_pub = self.create_publisher(MarkerArray, '/spatial_inspector/markers', 10)
        
        # Publisher jarak rintangan depan
        self.dist_pub = self.create_publisher(Float32, '/camera/detected_obstacle_distance', 10)
        
        self.marker_id_counter = 0
        self.last_center_dist = 0.0

        print("\n" + "="*65)
        print("    NODE INTERAKTIF PENGINDERAAN SPASIAL 3D (RGB-D ASTRA PRO)")
        print("="*65)
        print("Petunjuk Interaksi:")
        print("1. Di RViz2, klik tombol 'Publish Point' di toolbar atas.")
        print("2. Klik objek apa saja di map 3D (meja, dinding, rintangan).")
        print("3. Sistem akan menghitung jarak 3D dan memunculkan label melayang.")
        print("="*65 + "\n", flush=True)

    def depth_callback(self, msg: Image):
        try:
            # Konversi depth image ke numpy array (32FC1 meter atau 16UC1 milimeter)
            if '32F' in msg.encoding:
                depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            else:
                depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                depth_img = depth_raw.astype(np.float32) / 1000.0

            h, w = depth_img.shape
            # Ambil rata-rata area tengah 20x20 piksel (Center Crosshair)
            ch, cw = h // 2, w // 2
            center_region = depth_img[ch-10:ch+10, cw-10:cw+10]
            valid_depths = center_region[np.isfinite(center_region) & (center_region > 0.1)]

            if len(valid_depths) > 0:
                dist = float(np.median(valid_depths))
                self.last_center_dist = dist
                msg_out = Float32()
                msg_out.data = dist
                self.dist_pub.publish(msg_out)
        except Exception:
            pass

    def clicked_point_callback(self, msg: PointStamped):
        target_x = msg.point.x
        target_y = msg.point.y
        target_z = msg.point.z
        frame_id = msg.header.frame_id if msg.header.frame_id else 'map'

        # Ambil posisi robot saat ini dari TF (map -> base_footprint)
        robot_x, robot_y, robot_z = 0.0, 0.0, 0.0
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y
            robot_z = trans.transform.translation.z
        except Exception:
            pass

        # Hitung jarak Euclidean 3D dan 2D
        dist_3d = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2 + (target_z - robot_z)**2)
        dist_2d = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

        # Klasifikasi objek sederhana berbasis ketinggian Z
        if target_z < 0.15:
            object_class = "Lantai / Permukaan Dasar"
        elif target_z < 0.85:
            object_class = "Meja Lab / Kotak Rintangan 3D"
        else:
            object_class = "Dinding Ruangan / Papan Tulis"

        print(f"\n[KLIK SPASIAL INTERAKTIF]")
        print(f"  * Posisi 3D Objek : X = {target_x:.2f} m, Y = {target_y:.2f} m, Z = {target_z:.2f} m")
        print(f"  * Jarak dari Robot: {dist_3d:.2f} meter (Horizontal: {dist_2d:.2f} m)")
        print(f"  * Estimasi Objek  : {object_class}")
        print(f"  * Jarak Lurus FPV : {self.last_center_dist:.2f} m (Kamera Depan Astra)\n", flush=True)

        # Terbitkan Visual 3D Marker ke RViz2
        self.publish_interactive_markers(target_x, target_y, target_z, dist_3d, object_class, frame_id)

    def publish_interactive_markers(self, x, y, z, dist, label, frame_id):
        marker_array = MarkerArray()
        self.marker_id_counter += 1

        # Marker 1: Bola Penanda Neon Cyan di Titik Klik
        sphere = Marker()
        sphere.header.frame_id = frame_id
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = "spatial_inspection_points"
        sphere.id = self.marker_id_counter * 2
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = z
        sphere.scale.x = 0.20
        sphere.scale.y = 0.20
        sphere.scale.z = 0.20
        sphere.color.r = 0.0
        sphere.color.g = 0.95
        sphere.color.b = 1.0
        sphere.color.a = 0.9
        marker_array.markers.append(sphere)

        # Marker 2: Teks Billboard Melayang di Atas Objek
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "spatial_inspection_labels"
        text_marker.id = self.marker_id_counter * 2 + 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z + 0.35 # Melayang 35 cm di atas objek
        text_marker.scale.z = 0.18 # Ukuran font
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.2
        text_marker.color.a = 1.0
        text_marker.text = f"{label}\nJarak: {dist:.2f}m | (X:{x:.2f}, Y:{y:.2f})"
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = RGBDSpatialInteractive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
