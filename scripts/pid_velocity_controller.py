#!/usr/bin/env python3
"""
pid_velocity_controller.py
===========================
Pengontrol Kecepatan dan Kemudi Berbasis PID (Proportional-Integral-Derivative)
untuk Mengatasi Oversteering dan Pergerakan Menghentak (Jerky Motion) pada Robot 4WD.

Mekanisme Kerja:
1. Menerima target kecepatan (/cmd_vel) dari Nav2 atau Teleop.
2. Membaca umpan balik kecepatan aktual dari odometri (/odom).
3. Menghitung error kecepatan linier (e_v) dan error kecepatan sudut (e_w).
4. Menghitung keluaran koreksi PID dengan Anti-Windup dan Slew Rate Limiter (Soft Acceleration).
5. Menerbitkan kecepatan terkontrol halus ke Gazebo (/cmd_vel_pid).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PIDVelocityController(Node):
    def __init__(self):
        super().__init__('pid_velocity_controller')

        # Parameter PID Linier
        self.declare_parameter('kp_v', 0.4)
        self.declare_parameter('ki_v', 0.01)
        self.declare_parameter('kd_v', 0.005)

        # Parameter PID Angular (Damping Oversteering & Oscillation)
        self.declare_parameter('kp_w', 0.4)
        self.declare_parameter('ki_w', 0.01)
        self.declare_parameter('kd_w', 0.05)

        # Batasan Kinematika & Akselerasi Halus (Anti-Jerk)
        self.declare_parameter('max_lin_vel', 0.30)
        self.declare_parameter('max_ang_vel', 0.60)
        self.declare_parameter('max_lin_acc', 0.60)  # m/s^2
        self.declare_parameter('max_ang_acc', 0.80)  # rad/s^2
        self.declare_parameter('control_rate', 20.0) # Hz

        self.kp_v = self.get_parameter('kp_v').value
        self.ki_v = self.get_parameter('ki_v').value
        self.kd_v = self.get_parameter('kd_v').value

        self.kp_w = self.get_parameter('kp_w').value
        self.ki_w = self.get_parameter('ki_w').value
        self.kd_w = self.get_parameter('kd_w').value

        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.max_lin_acc = self.get_parameter('max_lin_acc').value
        self.max_ang_acc = self.get_parameter('max_ang_acc').value
        self.control_rate = self.get_parameter('control_rate').value

        # State Target & Aktual
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.act_v = 0.0
        self.act_w = 0.0

        # State PID
        self.integral_v = 0.0
        self.integral_w = 0.0
        self.prev_error_v = 0.0
        self.prev_error_w = 0.0

        # Output Sebelumnya (untuk Slew Rate Limiter)
        self.out_v = 0.0
        self.out_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_update_clock = self.get_clock().now()

        # Subscriber & Publisher
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.motor_pub = self.create_publisher(
            Twist, '/cmd_vel_pid', 10
        )

        dt = 1.0 / self.control_rate
        self.timer = self.create_timer(dt, self.control_loop)
        self.get_logger().info("PID Velocity & Anti-Oversteering Controller AKTIF.")

    def cmd_callback(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry):
        self.act_v = msg.twist.twist.linear.x
        self.act_w = msg.twist.twist.angular.z

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_clock).nanoseconds / 1e9
        if dt <= 0.001 or dt > 0.5:
            dt = 1.0 / self.control_rate
        self.last_update_clock = now

        # Watchdog timeout: jika tidak ada pesan /cmd_vel baru dalam 0.5 detik, hentikan robot
        time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > 0.5:
            self.cmd_v = 0.0
            self.cmd_w = 0.0

        # Jika target nol mutlak, reset integral dan rem langsung (mencegah creeping)
        if abs(self.cmd_v) < 0.005 and abs(self.cmd_w) < 0.005:
            self.integral_v = 0.0
            self.integral_w = 0.0
            self.prev_error_v = 0.0
            self.prev_error_w = 0.0
            self.out_v = 0.0
            self.out_w = 0.0
            stop_msg = Twist()
            self.motor_pub.publish(stop_msg)
            return

        # 1. Error Perhitungan
        err_v = self.cmd_v - self.act_v
        err_w = self.cmd_w - self.act_w

        # 2. Integrator dengan Anti-Windup Clamping
        self.integral_v = max(-0.15, min(0.15, self.integral_v + err_v * dt))
        self.integral_w = max(-0.25, min(0.25, self.integral_w + err_w * dt))

        # 3. Derivatif dengan Filter
        deriv_v = (err_v - self.prev_error_v) / dt
        deriv_w = (err_w - self.prev_error_w) / dt
        self.prev_error_v = err_v
        self.prev_error_w = err_w

        # 4. Kalkulasi Output PID (Feedforward Target + Koreksi PID)
        pid_v = self.cmd_v + (self.kp_v * err_v + self.ki_v * self.integral_v + self.kd_v * deriv_v)
        pid_w = self.cmd_w + (self.kp_w * err_w + self.ki_w * self.integral_w + self.kd_w * deriv_w)

        # 5. Slew Rate Limiter (Akselerasi Lembut / Anti-Jerk)
        max_delta_v = self.max_lin_acc * dt
        max_delta_w = self.max_ang_acc * dt

        delta_v = max(-max_delta_v, min(max_delta_v, pid_v - self.out_v))
        delta_w = max(-max_delta_w, min(max_delta_w, pid_w - self.out_w))

        self.out_v += delta_v
        self.out_w += delta_w

        # 6. Batasan Kecepatan Maksimum
        self.out_v = max(-self.max_lin_vel, min(self.max_lin_vel, self.out_v))
        self.out_w = max(-self.max_ang_vel, min(self.max_ang_vel, self.out_w))

        # Terbitkan pesan Twist terkontrol
        msg = Twist()
        msg.linear.x = float(self.out_v)
        msg.angular.z = float(self.out_w)
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
