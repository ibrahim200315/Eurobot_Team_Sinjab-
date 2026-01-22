#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from eurobot_interfaces.msg import CrateDetectionArray
import math
import time

class simple_crate_navigator(Node):
    def __init__(self):
        super().__init__('simple_crate_navigator')
        
        # Parameters
        self.declare_parameter('target_color', 'yellow')  # or 'blue'
        self.declare_parameter('approach_distance', 0.29)  # meters - stop distance
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.1)
        self.declare_parameter('angle_tolerance', 2.0)  # degrees
        self.declare_parameter('distance_tolerance', 0.01)  # meters
        self.declare_parameter('kp_linear', 1.0)  # proportional gain for linear
        self.declare_parameter('kp_angular', 3.0)  # proportional gain for angular
        self.declare_parameter('enable_approach', True)  # Enable/disable with param
        
        self.target_color = self.get_parameter('target_color').value
        self.approach_dist = self.get_parameter('approach_distance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.angle_tol = self.get_parameter('angle_tolerance').value
        self.dist_tol = self.get_parameter('distance_tolerance').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.enabled = self.get_parameter('enable_approach').value
        
        # State
        self.state = 'SEARCHING'  # SEARCHING, ALIGNING, APPROACHING, ALIGNED
        self.target_crate = None
        self.last_detection_time = None
        self.detection_timeout = 1.0  # seconds - increased tolerance
        
        # Publishers and Subscribers
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.get_logger().info(f"Publishing to: {cmd_topic}")
        self.det_sub = self.create_subscription(
            CrateDetectionArray, 
            '/crate/detections', 
            self.detection_callback, 
            10
        )
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info(f"=== Crate Approach Controller Started ===")
        self.get_logger().info(f"Target Color: {self.target_color}")
        self.get_logger().info(f"Approach Distance: {self.approach_dist}m")
        self.get_logger().info(f"Enabled: {self.enabled}")
    
    def detection_callback(self, msg):
        """Process crate detections and select target"""
        if not self.enabled:
            return
            
        # Find crates of target color
        target_crates = [d for d in msg.detections if d.color == self.target_color]
        
        if not target_crates:
            # No target detected this cycle
            return
        
        # Update last detection time
        self.last_detection_time = time.time()
        
        # Select closest crate
        self.target_crate = min(target_crates, key=lambda c: c.distance)
        
        # Log detection
        self.get_logger().info(
            f"[DETECTION] {self.target_crate.color} crate: "
            f"dist={self.target_crate.distance:.3f}m, "
            f"angle={self.target_crate.angle:.1f}°, "
            f"x={self.target_crate.x:.3f}m (fwd), y={self.target_crate.y:.3f}m",
            throttle_duration_sec=0.5
        )
        
        # State transition from SEARCHING
        if self.state == 'SEARCHING':
            self.state = 'ALIGNING'
            self.get_logger().info(f"[STATE] SEARCHING -> ALIGNING")
    
    def is_target_lost(self):
        """Check if target was lost (no recent detections)"""
        if self.last_detection_time is None:
            return True
        return (time.time() - self.last_detection_time) > self.detection_timeout
    
    def control_loop(self):
        """Main control loop for approaching crates"""
        cmd = Twist()
        
        if not self.enabled:
            self.cmd_pub.publish(cmd)
            return
        
        # Check if target is lost
        if self.is_target_lost() and self.state != 'SEARCHING':
            self.get_logger().warn("[STATE] Target lost -> SEARCHING")
            self.state = 'SEARCHING'
            self.target_crate = None
        
        # Handle states
        if self.state == 'SEARCHING':
            # No target - stop and wait (don't rotate, let perception work)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("[SEARCHING] Waiting for crate detection...", 
                                 throttle_duration_sec=2.0)
        
        elif self.target_crate is not None:
            # Calculate errors
            angle_error = self.target_crate.angle  # degrees
            distance_error = self.target_crate.distance - self.approach_dist
            
            self.get_logger().info(
                f"[{self.state}] angle_err={angle_error:.1f}°, dist_err={distance_error:.3f}m",
                throttle_duration_sec=0.5
            )
            
            if self.state == 'ALIGNING':
                # First align to the crate
                if abs(angle_error) > self.angle_tol:
                    # Pure rotation to align
                    cmd.linear.x = 0.0
                    # INVERTED: Negative angle (right) needs negative angular.z (CW)
                    # Robot's convention is opposite, so we negate
                    cmd.angular.z = self.clamp(
                        -self.kp_angular * math.radians(angle_error),  # Note the negative sign
                        -self.max_angular,
                        self.max_angular
                    )
                    self.get_logger().info(
                        f"[ALIGNING] angle_err={angle_error:.1f}° -> angular.z={cmd.angular.z:.3f} rad/s",
                        throttle_duration_sec=0.3
                    )
                else:
                    self.get_logger().info("[STATE] ALIGNING -> APPROACHING")
                    self.state = 'APPROACHING'
            
            elif self.state == 'APPROACHING':
                # Check if reached target
                if abs(distance_error) <= self.dist_tol:
                    self.get_logger().info("[STATE] APPROACHING -> ALIGNED")
                    self.state = 'ALIGNED'
                else:
                    # Move forward while maintaining alignment
                    cmd.linear.x = self.clamp(
                        self.kp_linear * distance_error,
                        -self.max_linear,
                        self.max_linear
                    )
                    
                    # Small angular correction while approaching
                    if abs(angle_error) > self.angle_tol * 0.5:
                        cmd.angular.z = self.clamp(
                            -self.kp_angular * 0.3 * math.radians(angle_error),  # Note the negative sign
                            -self.max_angular * 0.3,
                            self.max_angular * 0.3
                        )
                    
                    self.get_logger().info(
                        f"[APPROACHING] linear={cmd.linear.x:.3f}, angular={cmd.angular.z:.3f}",
                        throttle_duration_sec=0.5
                    )
            
            elif self.state == 'ALIGNED':
                # Stay stopped at target
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info(
                    "[ALIGNED] Ready for gripper action (dist={:.3f}m, angle={:.1f}°)".format(
                        self.target_crate.distance, self.target_crate.angle
                    ),
                    throttle_duration_sec=2.0
                )
        
        # Publish command
        self.cmd_pub.publish(cmd)
    
    def clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(value, max_val))


def main(args=None):
    rclpy.init(args=args)
    node = simple_crate_navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()