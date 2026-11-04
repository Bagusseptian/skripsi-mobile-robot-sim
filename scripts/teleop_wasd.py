#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
============================================================
           KONTROL ROBOT CERDAS (WASD + STEERING)
============================================================
              [ Q ] Serong Kiri    [ W ] Maju     [ E ] Serong Kanan
              [ A ] Putar Kiri     [ S ] Mundur   [ D ] Putar Kanan
              [ Z ] Mundur Kiri                   [ C ] Mundur Kanan

              [ SPASI ] / [ X ] : Berhenti (Stop)
              [ + ] : Tambah Kecepatan
              [ - ] : Kurangi Kecepatan
              [ Ctrl+C ] : Keluar
============================================================
"""

class TeleopWASD(Node):
    def __init__(self):
        super().__init__('teleop_wasd')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed = 0.6  # Kecepatan linier (m/s)
        self.turn = 1.5   # Kecepatan sudut putar (rad/s)
        
        self.settings = termios.tcgetattr(sys.stdin)
        self.print_status()

    def print_status(self):
        print(BANNER)
        print(f"Kecepatan: Linier = {self.speed:.2f} m/s | Putar = {self.turn:.2f} rad/s\n")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key().lower()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.speed
                    print(f"\r[↑] MAJU LURUS ({self.speed:.2f} m/s)                 ", end='', flush=True)
                elif key == 's':
                    twist.linear.x = -self.speed
                    print(f"\r[↓] MUNDUR LURUS ({self.speed:.2f} m/s)               ", end='', flush=True)
                elif key == 'a':
                    twist.angular.z = self.turn
                    print(f"\r[↶] PUTAR KIRI DI TEMPAT ({self.turn:.2f} rad/s)       ", end='', flush=True)
                elif key == 'd':
                    twist.angular.z = -self.turn
                    print(f"\r[↷] PUTAR KANAN DI TEMPAT ({self.turn:.2f} rad/s)      ", end='', flush=True)
                elif key == 'q':
                    twist.linear.x = self.speed
                    twist.angular.z = self.turn * 0.7
                    print(f"\r[↖] MAJU BELOK KIRI                                    ", end='', flush=True)
                elif key == 'e':
                    twist.linear.x = self.speed
                    twist.angular.z = -self.turn * 0.7
                    print(f"\r[↗] MAJU BELOK KANAN                                   ", end='', flush=True)
                elif key == 'z':
                    twist.linear.x = -self.speed
                    twist.angular.z = -self.turn * 0.7
                    print(f"\r[↙] MUNDUR BELOK KIRI                                  ", end='', flush=True)
                elif key == 'c':
                    twist.linear.x = -self.speed
                    twist.angular.z = self.turn * 0.7
                    print(f"\r[↘] MUNDUR BELOK KANAN                                 ", end='', flush=True)
                elif key in [' ', 'x']:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    print(f"\r[■] BERHENTI (STOP)                                    ", end='', flush=True)
                elif key in ['+', '=']:
                    self.speed = min(2.5, self.speed + 0.1)
                    self.turn = min(3.5, self.turn + 0.2)
                    print(f"\r[+] Speed Naik: Linier={self.speed:.2f} Turn={self.turn:.2f}   ", end='', flush=True)
                elif key in ['-', '_']:
                    self.speed = max(0.1, self.speed - 0.1)
                    self.turn = max(0.2, self.turn - 0.2)
                    print(f"\r[-] Speed Turun: Linier={self.speed:.2f} Turn={self.turn:.2f}  ", end='', flush=True)
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    continue

                self.pub.publish(twist)

        except Exception as e:
            print(f"\nError: {e}")
        finally:
            stop_twist = Twist()
            self.pub.publish(stop_twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nKontrol WASD selesai.")

def main():
    rclpy.init()
    node = TeleopWASD()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
