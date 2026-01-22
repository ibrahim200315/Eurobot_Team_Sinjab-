#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from eurobot_interfaces.msg import CrateDetectionArray, PantryDetectionArray, CrateManipulation
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__('task_manager')

        # Parameters
        self.declare_parameter('crate_topic', '/crate/detections')
        self.declare_parameter('pantry_topic', '/pantry/detections')
        self.declare_parameter('amcl_topic', '/amcl_pose')
        self.declare_parameter('max_crates_per_pantry', 3)
        self.declare_parameter('goal_threshold', 0.25)  # meters to trigger pick/place

        self.crate_topic = self.get_parameter('crate_topic').value
        self.pantry_topic = self.get_parameter('pantry_topic').value
        self.amcl_topic = self.get_parameter('amcl_topic').value
        self.max_crates_per_pantry = self.get_parameter('max_crates_per_pantry').value
        self.goal_threshold = self.get_parameter('goal_threshold').value

        # Robot state
        self.robot_pose = None  # (x, y, theta)
        self.current_task = None  # {"type": "crate" or "pantry", "id": id, "x": , "y": }

        # Memory
        self.crates = []  # List of dicts: {"id", "x", "y", "color", "collected"}
        self.pantries = {}  # Dict: {pantry_id: {"x", "y", "crates": []}}

        # Subscribers
        self.create_subscription(CrateDetectionArray, self.crate_topic, self.crate_cb, 10)
        self.create_subscription(PantryDetectionArray, self.pantry_topic, self.pantry_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.amcl_topic, self.amcl_cb, 10)

        # Publishers
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/task_manager/nav_goal', 10)
        self.manip_pub = self.create_publisher(CrateManipulation, '/manipulation/command', 10)

        self.get_logger().info("Task Manager Node with pick/place triggers started")

    # -------------------- Callbacks -------------------- #
    def amcl_cb(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        theta = math.atan2(siny_cosp, cosy_cosp)
        self.robot_pose = (pose.position.x, pose.position.y, theta)
        # Check if current goal reached
        self.check_goal_reached()

    def crate_cb(self, msg):
        if not self.robot_pose:
            return
        for det in msg.detections:
            map_x, map_y = self.robot_to_map(det.x, det.y)
            exists = any(math.isclose(c["x"], map_x, abs_tol=0.05) and
                         math.isclose(c["y"], map_y, abs_tol=0.05)
                         for c in self.crates)
            if not exists:
                crate_id = len(self.crates) + 1
                self.crates.append({
                    "id": crate_id,
                    "x": map_x,
                    "y": map_y,
                    "color": det.color,
                    "collected": False
                })
                self.get_logger().info(
                    f"New crate detected: ID={crate_id}, color={det.color}, pos=({map_x:.2f},{map_y:.2f})"
                )
        self.process_task()

    def pantry_cb(self, msg):
        if not self.robot_pose:
            return
        for det in msg.detections:
            pantry_id = det.id
            if pantry_id not in self.pantries:
                map_x, map_y = self.robot_to_map(det.x, det.y)
                self.pantries[pantry_id] = {"x": map_x, "y": map_y, "crates": []}
                self.get_logger().info(
                    f"New pantry detected: ID={pantry_id}, pos=({map_x:.2f},{map_y:.2f})"
                )

    # -------------------- Helper Functions -------------------- #
    def robot_to_map(self, rel_x, rel_y):
        rx, ry, theta = self.robot_pose
        map_x = rx + rel_x * math.cos(theta) - rel_y * math.sin(theta)
        map_y = ry + rel_x * math.sin(theta) + rel_y * math.cos(theta)
        return map_x, map_y

    def distance(self, p1, p2):
        return math.hypot(p1[0]-p2[0], p1[1]-p2[1])

    def select_nearest_crate(self):
        uncollected = [c for c in self.crates if not c["collected"]]
        if not uncollected:
            return None
        robot_pos = (self.robot_pose[0], self.robot_pose[1])
        return min(uncollected, key=lambda c: self.distance(robot_pos, (c["x"], c["y"])))

    def select_nearest_pantry(self):
        robot_pos = (self.robot_pose[0], self.robot_pose[1])
        available = [p for p in self.pantries.values()
                     if len(p["crates"]) < self.max_crates_per_pantry]
        if not available:
            return None
        return min(available, key=lambda p: self.distance(robot_pos, (p["x"], p["y"])))

    # -------------------- Task Management -------------------- #
    def process_task(self):
        if not self.robot_pose:
            return
        if self.current_task:
            return  # already moving to a target

        # Pick crate
        crate = self.select_nearest_crate()
        if crate:
            self.send_goal(crate["x"], crate["y"])
            self.current_task = {"type": "crate", "id": crate["id"], "x": crate["x"], "y": crate["y"]}
            self.get_logger().info(f"Navigating to crate {crate['id']} at ({crate['x']:.2f},{crate['y']:.2f})")
            return

        # Drop at pantry
        pantry = self.select_nearest_pantry()
        if pantry:
            self.send_goal(pantry["x"], pantry["y"])
            self.current_task = {"type": "pantry", "id": None, "x": pantry["x"], "y": pantry["y"]}
            self.get_logger().info(f"Navigating to pantry at ({pantry['x']:.2f},{pantry['y']:.2f})")

    def check_goal_reached(self):
        if not self.current_task or not self.robot_pose:
            return
        robot_pos = (self.robot_pose[0], self.robot_pose[1])
        target_pos = (self.current_task["x"], self.current_task["y"])
        if self.distance(robot_pos, target_pos) < self.goal_threshold:
            if self.current_task["type"] == "crate":
                self.trigger_manipulation(self.current_task["id"], 0)  # 0 = PICK
                # Mark crate as collected
                for c in self.crates:
                    if c["id"] == self.current_task["id"]:
                        c["collected"] = True
                self.current_task = None  # allow next task
            elif self.current_task["type"] == "pantry":
                nearest_crate = next((c for c in self.crates if c["collected"]), None)
                if nearest_crate:
                    self.trigger_manipulation(nearest_crate["id"], 1)  # 1 = PLACE
                    # Add crate to nearest pantry memory
                    nearest_pantry = self.select_nearest_pantry()
                    if nearest_pantry:
                        nearest_pantry["crates"].append(nearest_crate["id"])
                self.current_task = None

            # Trigger next task if available
            self.process_task()

    # -------------------- Manipulation -------------------- #
    def trigger_manipulation(self, crate_id, action):
        msg = CrateManipulation()
        msg.crate_id = crate_id
        msg.command = action
        self.manip_pub.publish(msg)
        if action == 0:
            self.get_logger().info(f"Published PICK command for crate {crate_id}")
        else:
            self.get_logger().info(f"Published PLACE command for crate {crate_id}")

    # -------------------- Navigation -------------------- #
    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.nav_goal_pub.publish(goal)

# -------------------- Main -------------------- #
def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()

