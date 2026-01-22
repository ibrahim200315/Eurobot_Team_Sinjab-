#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class BlueCrateFollower(Node):
    def __init__(self):
        super().__init__('blue_crate_follower')

        # Subscribe to camera and publish velocity
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.bridge = CvBridge()
        self.linear_speed = 0.2      # forward speed
        self.angular_speed = 0.005   # turning speed
        self.stop_distance_pixels = 50  # how close to stop (depends on camera view)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue color range in HSV
        lower_blue = (100, 150, 50)
        upper_blue = (140, 255, 255)

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            cx = x + w // 2
            frame_center = frame.shape[1] // 2

            error_x = cx - frame_center
            twist.angular.z = -error_x * self.angular_speed

            if w < self.stop_distance_pixels:
                twist.linear.x = self.linear_speed
            else:
                twist.linear.x = 0
                self.get_logger().info("Reached crate!")

        else:
            twist.linear.x = 0
            twist.angular.z = 0

        self.cmd_pub.publish(twist)

        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow("Camera", frame)
        cv2.imshow("Blue Mask", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BlueCrateFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()