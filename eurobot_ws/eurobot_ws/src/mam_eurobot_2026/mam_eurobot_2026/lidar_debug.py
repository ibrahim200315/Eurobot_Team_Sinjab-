#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class ScanDebug(Node):
    def __init__(self):
        super().__init__('scan_debug')

        # Subscribe to LIDAR data
        self.sub = self.create_subscription(LaserScan, '/lidar/scan', self.cb, 10)

        # Create publisher to move robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: every 0.1s, publish small rotation command
        self.timer = self.create_timer(0.1, self.rotate_slowly)
        self.start_time = time.time()

    def rotate_slowly(self):
        # Rotate slowly for the first 10 seconds
        twist = Twist()
        if time.time() - self.start_time < 10:
            twist.angular.z = 0.4  # spin in place
            self.cmd_pub.publish(twist)
        else:
            twist.angular.z = 0.0  # stop after 10s
            self.cmd_pub.publish(twist)
            self.timer.cancel()  # stop timer

    def cb(self, msg: LaserScan):
        vals = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        if not vals:
            self.get_logger().info('No hits yet — still scanning...')
            return
        mn = min(vals)
        idx = msg.ranges.index(mn)
        ang = msg.angle_min + idx * msg.angle_increment
        self.get_logger().info(f'Nearest object: {mn:.2f} m at {ang:.2f} rad')

def main():
    rclpy.init()
    node = ScanDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

