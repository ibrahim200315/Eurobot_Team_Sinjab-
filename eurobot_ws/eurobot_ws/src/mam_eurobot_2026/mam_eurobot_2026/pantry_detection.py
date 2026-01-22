#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from eurobot_interfaces.msg import PantryDetection, PantryDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PantryDetectionNode(Node):
    def __init__(self):
        super().__init__('pantry_detection_node')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('debug_view', True)
        self.declare_parameter('green_lower_hsv', [40, 40, 40])
        self.declare_parameter('green_upper_hsv', [80, 255, 255])
        self.declare_parameter('min_area', 5000)  # pixel area threshold
        self.declare_parameter('known_pantry_width', 0.15)  # meters
        self.declare_parameter('focal_length_px', 600.0)
        self.declare_parameter('camera_center_offset', 0.0)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.debug_view = self.get_parameter('debug_view').value
        self.lower_green = np.array(self.get_parameter('green_lower_hsv').value)
        self.upper_green = np.array(self.get_parameter('green_upper_hsv').value)
        self.min_area = self.get_parameter('min_area').value
        self.pantry_width = self.get_parameter('known_pantry_width').value
        self.focal_length = self.get_parameter('focal_length_px').value
        self.camera_offset = self.get_parameter('camera_center_offset').value

        self.bridge = CvBridge()

        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(PantryDetectionArray, '/pantry/detections', 10)

        self.pantry_counter = 0
        self.detected_pantries = []

        self.get_logger().info("Pantry detection node started with coordinate conversion")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = PantryDetectionArray()
        detections.header = msg.header

        frame_center_x = cv_image.shape[1] / 2  # image center in pixels

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w/2
            cy = y + h/2

            # -------------------- MODIFIED SECTION START -------------------- #
            # Estimate distance to pantry (meters)
            distance = (self.pantry_width * self.focal_length) / w

            # Angle relative to camera center
            offset_px = cx - frame_center_x
            angle_rad = math.atan2(offset_px, self.focal_length)

            # Robot-relative coordinates
            rel_x = distance * math.cos(angle_rad) + self.camera_offset  # forward
            rel_y = distance * math.sin(angle_rad)                       # lateral
            # -------------------- MODIFIED SECTION END -------------------- #

            # Avoid duplicates
            if not any(math.hypot(rel_x - px, rel_y - py) < 0.5 for px, py in self.detected_pantries):
                self.pantry_counter += 1
                self.detected_pantries.append((rel_x, rel_y))

                pantry = PantryDetection()
                pantry.id = self.pantry_counter
                pantry.x = float(rel_x)  # robot-relative meters
                pantry.y = float(rel_y)  # robot-relative meters
                detections.detections.append(pantry)

                if self.debug_view:
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(cv_image, f"Pantry {self.pantry_counter}", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        if detections.detections:
            self.pub.publish(detections)

        if self.debug_view:
            cv2.imshow("Pantry Detection", cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PantryDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#____________________Old One___________________

"""import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from eurobot_interfaces.msg import PantryDetection, PantryDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class PantryDetectionNode(Node):
    def __init__(self):
        super().__init__('pantry_detection')
        self.declare_parameter('camera_topic', '/camera/color/image_raw')
        self.declare_parameter('debug_view', True)
        self.declare_parameter('green_lower_hsv', [40, 40, 40])
        self.declare_parameter('green_upper_hsv', [80, 255, 255])
        self.declare_parameter('min_area', 5000)  # pixel area threshold

        self.camera_topic = self.get_parameter('camera_topic').value
        self.debug_view = self.get_parameter('debug_view').value
        self.lower_green = np.array(self.get_parameter('green_lower_hsv').value)
        self.upper_green = np.array(self.get_parameter('green_upper_hsv').value)
        self.min_area = self.get_parameter('min_area').value

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.create_subscription(Image, self.camera_topic, self.image_callback, 10)
        self.pub = self.create_publisher(PantryDetectionArray, '/pantry/detections', 10)

        self.pantry_counter = 0
        self.detected_pantries = []  # (x, y) of pantry centers to avoid duplicates

        self.get_logger().info("Pantry detection node started — detecting on-the-go.")

    def image_callback(self, msg):
        # Convert to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Mask green regions
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = PantryDetectionArray()
        detections.header = msg.header

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w/2, y + h/2

            # Avoid duplicates using proximity check
            if not any(math.hypot(cx - px, cy - py) < 50 for px, py in self.detected_pantries):
                self.pantry_counter += 1
                self.detected_pantries.append((cx, cy))

                pantry = PantryDetection()
                pantry.id = self.pantry_counter
                pantry.x = float(cx)  # pixel space for now
                pantry.y = float(cy)
                detections.detections.append(pantry)

                self.get_logger().info(f"Detected Pantry #{self.pantry_counter} at ({cx:.1f}, {cy:.1f})")

                if self.debug_view:
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,255,0), 2)
                    cv2.putText(cv_image, f"Pantry {self.pantry_counter}", (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        if detections.detections:
            self.pub.publish(detections)

        if self.debug_view:
            cv2.imshow("Pantry Detection", cv_image)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PantryDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
