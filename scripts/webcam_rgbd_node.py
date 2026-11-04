#!/usr/bin/env python3
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class WebcamRGBDNode(Node):
    def __init__(self):
        super().__init__('webcam_rgbd_node')
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        self.bridge = CvBridge()
        
        # Try opening camera 0
        self.cap = cv2.VideoCapture(0)
        self.use_fallback = not self.cap.isOpened()
        
        if self.use_fallback:
            self.get_logger().warn('Hardware webcam tidak terdeteksi di WSL2. Menggunakan Generator Citra Uji Coba Cerdas (Smart Synthetic RGB-D).')
        else:
            self.get_logger().info('Hardware Webcam BERHASIL terhubung!')
            
        self.timer = self.create_timer(0.05, self.timer_cb) # 20 Hz
        self.angle = 0.0

    def timer_cb(self):
        now = self.get_clock().now().to_msg()
        
        if not self.use_fallback:
            ret, frame = self.cap.read()
            if not ret:
                self.use_fallback = True
                frame = self.generate_synthetic_frame()
            else:
                frame = cv2.resize(frame, (320, 240))
        else:
            frame = self.generate_synthetic_frame()

        # Compute pseudo-depth (16UC1 format in millimeters)
        # Closer objects (bottom of image) = ~800mm (0.8m), Far objects (top) = ~3000mm (3.0m)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Base depth ramp: Y-coordinate gradient
        y_coords = np.linspace(3000, 800, 240, dtype=np.uint16)[:, None]
        depth_map = np.repeat(y_coords, 320, axis=1)
        
        # Detect bright / distinct objects to create real 3D depth pop-out
        mask = gray > 180
        depth_map[mask] = np.clip(depth_map[mask] - 400, 400, 5000)
        
        # Publish RGB
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_depth_optical_frame'
        self.image_pub.publish(img_msg)
        
        # Publish Depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding='16UC1')
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        self.depth_pub.publish(depth_msg)
        
        # Publish CameraInfo
        ci = CameraInfo()
        ci.header.stamp = now
        ci.header.frame_id = 'camera_depth_optical_frame'
        ci.width = 320
        ci.height = 240
        ci.k = [260.0, 0.0, 160.0, 0.0, 260.0, 120.0, 0.0, 0.0, 1.0]
        self.info_pub.publish(ci)

    def generate_synthetic_frame(self):
        # 320x240 frame with room background and a moving target object
        img = np.full((240, 320, 3), 200, dtype=np.uint8)
        # Floor
        img[140:, :] = [140, 140, 150]
        # Moving target object (e.g. red bottle/box moving left and right)
        self.angle += 0.05
        cx = int(160 + 80 * np.sin(self.angle))
        cy = 150
        cv2.rectangle(img, (cx - 25, cy - 40), (cx + 25, cy + 30), (30, 30, 220), -1)
        cv2.putText(img, "Target (Botol)", (cx - 40, cy - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 20, 20), 1)
        return img

def main():
    rclpy.init()
    node = WebcamRGBDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
