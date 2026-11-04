#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from tf2_ros import TransformBroadcaster

class LightweightRobotSim(Node):
    def __init__(self):
        super().__init__('lightweight_robot_sim', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, False)
        ])
        
        # State: x, y, theta
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self.wheel_pos = 0.0
        
        # Subscriptions & Publishers
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        from sensor_msgs.msg import JointState
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Virtual Room Boundaries: [-3, 3] in X and Y (Original 6m x 6m Lab Arena)
        self.walls = [
            ((-3.0, -3.0), (3.0, -3.0)),   # South Wall
            ((3.0, -3.0), (3.0, 3.0)),     # East Wall
            ((3.0, 3.0), (-3.0, 3.0)),     # North Wall
            ((-3.0, 3.0), (-3.0, -3.0)),   # West Wall
            
            # Meja Lab (1.5, 1.0)
            ((1.1, 0.4), (1.9, 0.4)), ((1.9, 0.4), (1.9, 1.6)),
            ((1.9, 1.6), (1.1, 1.6)), ((1.1, 1.6), (1.1, 0.4)),
            
            # Kotak Merah (1.5, -1.5)
            ((1.25, -1.75), (1.75, -1.75)), ((1.75, -1.75), (1.75, -1.25)),
            ((1.75, -1.25), (1.25, -1.25)), ((1.25, -1.25), (1.25, -1.75)),
            
            # Pilar Hijau (-1.2, -1.2)
            ((-1.5, -1.2), (-0.9, -1.2)), ((-1.2, -1.5), (-1.2, -0.9))
        ]
        
        # Daftar Landmark Visual Objek Lab 6x6m
        self.visual_landmarks = [
            {"name": "Kotak Merah", "x": 1.5,  "y": -1.5, "color": [30, 30, 220], "radius": 0.35},
            {"name": "Pilar Hijau", "x": -1.2, "y": -1.2, "color": [30, 210, 40], "radius": 0.35},
            {"name": "Meja Lab",    "x": 1.5,  "y": 1.0,  "color": [20, 110, 220], "radius": 0.55}
        ]
        
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_loop)
        self.get_logger().info('Original 6x6m Arena Sim (RViz-Only) Active!')

    def cmd_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update_loop(self):
        now = self.get_clock().now().to_msg()
        
        # Kinematics integration
        self.theta += self.wz * self.dt
        self.x += self.vx * math.cos(self.theta) * self.dt
        self.y += self.vx * math.sin(self.theta) * self.dt
        
        # 1. TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # 2. Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        # Anti-bentrok dengan simulator lain (Gazebo)
        if self.count_publishers('/odom') <= 1:
            self.tf_broadcaster.sendTransform(t)
            self.odom_pub.publish(odom)

        # 2b. Joint States for RobotModel in RViz
        self.wheel_pos += self.vx * self.dt / 0.05
        from sensor_msgs.msg import JointState
        js = JointState()
        js.header.stamp = now
        js.name = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']
        js.position = [self.wheel_pos, self.wheel_pos, self.wheel_pos, self.wheel_pos]
        self.joint_pub.publish(js)
        
        # 3. Simulated 2D LiDAR (/scan)
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0
        
        ranges = []
        for i in range(360):
            ray_angle = self.theta + scan.angle_min + i * scan.angle_increment
            dist = self.raycast(self.x, self.y, ray_angle)
            ranges.append(float(dist))
        scan.ranges = ranges

        if self.count_publishers('/scan') <= 1:
            self.scan_pub.publish(scan)
        
        # 4. Generate Synthetic Camera & Depth Frame
        synth_img = np.zeros((240, 320, 3), dtype=np.uint8)
        synth_img[:, :] = [185, 185, 195] # Dinding lab netral
        synth_depth = np.full((240, 320), 4000, dtype=np.uint16)
        
        fov = math.radians(60.0)
        for obj in self.visual_landmarks:
            dx = obj["x"] - self.x
            dy = obj["y"] - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.2 or dist > 8.0:
                continue
            
            obj_bearing = math.atan2(dy, dx)
            rel_bearing = (obj_bearing - self.theta + math.pi) % (2.0 * math.pi) - math.pi
            
            if abs(rel_bearing) < (fov / 2.0):
                col_center = int(160 - (rel_bearing / (fov / 2.0)) * 160)
                box_w = int(np.clip(180.0 / dist, 25, 180))
                box_h = int(box_w * 1.1)
                
                x1 = max(0, col_center - box_w // 2)
                x2 = min(320, col_center + box_w // 2)
                y1 = max(0, 120 - box_h // 2)
                y2 = min(240, 120 + box_h // 2)
                
                if x2 > x1 and y2 > y1:
                    synth_img[y1:y2, x1:x2] = obj["color"]
                    synth_depth[y1:y2, x1:x2] = int(dist * 1000)
        
        # RGB Image Message
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_depth_optical_frame'
        img_msg.height = 240
        img_msg.width = 320
        img_msg.encoding = 'bgr8'
        img_msg.step = 320 * 3
        img_msg.data = synth_img.tobytes()
        
        # Depth Image Message (16UC1)
        depth_msg = Image()
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = 'camera_depth_optical_frame'
        depth_msg.height = 240
        depth_msg.width = 320
        depth_msg.encoding = '16UC1'
        depth_msg.step = 320 * 2
        depth_msg.data = synth_depth.tobytes()
        
        # Camera Info
        ci = CameraInfo()
        ci.header.stamp = now
        ci.header.frame_id = 'camera_depth_optical_frame'
        ci.width = 320
        ci.height = 240
        ci.k = [260.0, 0.0, 160.0, 0.0, 260.0, 120.0, 0.0, 0.0, 1.0]

        # Anti-bentrok Kamera
        if self.count_publishers('/camera/image_raw') <= 1:
            self.img_pub.publish(img_msg)
            self.depth_pub.publish(depth_msg)
            self.cam_info_pub.publish(ci)
        else:
            if not getattr(self, '_warned_conflict', False):
                self.get_logger().warn(
                    "[BENTROK TERDETEKSI] Gazebo atau hardware aktif pada /camera/image_raw! "
                    "Dummy sim menonaktifkan kamera virtual agar citra tidak kelap-kelip."
                )
                self._warned_conflict = True

    def raycast(self, ox, oy, angle):
        dx = math.cos(angle)
        dy = math.sin(angle)
        min_dist = 12.0
        
        for (p1, p2) in self.walls:
            x1, y1 = p1
            x2, y2 = p2
            denom = (x1 - x2) * dy - (y1 - y2) * dx
            if abs(denom) < 1e-6:
                continue
            t = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
            u = -((x1 - x2) * (y1 - oy) - (y1 - y2) * (x1 - ox)) / denom
            if 0.0 <= t <= 1.0 and u > 0.15:
                if u < min_dist:
                    min_dist = u
        return min_dist

def main():
    rclpy.init()
    node = LightweightRobotSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
