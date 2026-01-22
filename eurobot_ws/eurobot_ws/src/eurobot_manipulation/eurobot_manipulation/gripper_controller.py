#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller_node')

        # Publisher to gripper controller
        self.pub = self.create_publisher(Float64MultiArray, '/gripper_controller/commands', 10)

        # Subscribers to contact sensors
        self.left_contact = False
        self.right_contact = False
        self.create_subscription(Contacts, '/gripper/left/contact', self.left_contact_cb, 10)
        self.create_subscription(Contacts, '/gripper/right/contact', self.right_contact_cb, 10)

        # Subscribe to joint_states
        self.left_pos = 0.03
        self.right_pos = 0.03
        self.joint_initialized = False
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Subscribe to commands from master coordinator
        self.create_subscription(String, '/gripper/command', self.command_callback, 10)

        # Publisher for gripper state
        self.state_pub = self.create_publisher(String, '/gripper/state', 10)

        # Timer to send commands (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        # Gripper limits
        self.min_pos = -0.03
        self.max_pos = 0.03
        self.step = 0.005

        # State machine
        self.state = "initializing"
        
        # Initialization timer
        self.init_timer = self.create_timer(1.0, self.initialize_gripper)
        self.init_sent = False

        self.get_logger().info("Gripper Controller Initialized")

    def initialize_gripper(self):
        """Send initial OPEN command"""
        if not self.init_sent:
            msg = Float64MultiArray()
            msg.data = [0.03, 0.03]
            self.pub.publish(msg)
            self.get_logger().info("Gripper initialized to OPEN")
            self.state = "idle"
            self.init_sent = True
            self.init_timer.cancel()

    def command_callback(self, msg):
        """Handle commands from master coordinator"""
        command = msg.data.lower()
        if command == 'close':
            self.close_gripper()
        elif command == 'open':
            self.open_gripper()
        else:
            self.get_logger().warn(f"Unknown gripper command: {command}")

    def left_contact_cb(self, msg):
        self.left_contact = len(msg.contacts) > 0
        if self.left_contact and self.state == "closing":
            self.get_logger().info("Left finger contact!", throttle_duration_sec=1.0)

    def right_contact_cb(self, msg):
        self.right_contact = len(msg.contacts) > 0
        if self.right_contact and self.state == "closing":
            self.get_logger().info("Right finger contact!", throttle_duration_sec=1.0)

    def joint_state_cb(self, msg):
        """Update current joint positions"""
        try:
            idx_left = msg.name.index('left_finger_joint')
            idx_right = msg.name.index('right_finger_joint')
            self.left_pos = msg.position[idx_left]
            self.right_pos = msg.position[idx_right]
            
            if not self.joint_initialized:
                self.joint_initialized = True
                self.get_logger().info(f"Joint states: left={self.left_pos:.4f}, right={self.right_pos:.4f}")
        except (ValueError, IndexError):
            pass

    def control_loop(self):
        """Main control loop - 20 Hz"""
        
        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        if self.state == "initializing":
            return
        
        elif self.state == "closing":
            if self.left_contact and self.right_contact:
                self.state = "gripping"
                self.get_logger().info("Crate grasped!")
                return
            
            if not self.left_contact and self.left_pos > self.min_pos:
                self.left_pos -= self.step
            if not self.right_contact and self.right_pos > self.min_pos:
                self.right_pos -= self.step
            
            if self.left_pos <= self.min_pos and self.right_pos <= self.min_pos:
                if not (self.left_contact and self.right_contact):
                    self.get_logger().warn("Gripper fully closed but no object!")
                    self.state = "idle"

        elif self.state == "opening":
            if self.left_pos < self.max_pos:
                self.left_pos += self.step
            if self.right_pos < self.max_pos:
                self.right_pos += self.step
            
            if self.left_pos >= self.max_pos and self.right_pos >= self.max_pos:
                self.get_logger().info("Gripper fully opened")
                self.state = "idle"

        elif self.state == "gripping":
            pass  # Hold position
        
        elif self.state == "idle":
            pass  # Don't move

        # Clamp positions
        self.left_pos = max(self.min_pos, min(self.max_pos, self.left_pos))
        self.right_pos = max(self.min_pos, min(self.max_pos, self.right_pos))

        # Publish positions
        msg = Float64MultiArray()
        msg.data = [self.left_pos, self.right_pos]
        self.pub.publish(msg)

    def close_gripper(self):
        """Start closing the gripper"""
        if self.state in ["idle", "opening"]:
            self.get_logger().info("Closing gripper...")
            self.state = "closing"
            self.left_contact = False
            self.right_contact = False
        else:
            self.get_logger().warn(f"Cannot close in state: {self.state}")

    def open_gripper(self):
        """Start opening the gripper"""
        if self.state in ["idle", "gripping", "closing"]:
            self.get_logger().info("Opening gripper...")
            self.state = "opening"
            self.left_contact = False
            self.right_contact = False
        else:
            self.get_logger().warn(f"Cannot open in state: {self.state}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()