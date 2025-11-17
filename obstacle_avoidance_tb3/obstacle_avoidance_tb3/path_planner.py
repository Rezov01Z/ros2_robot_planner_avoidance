#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion

class SimplePathPlanner(Node):
    def __init__(self):
        super().__init__('simple_path_planner')
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Parameters
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        
        # State
        self.current_pose = None
        self.goal_position = None
        self.obstacle_detected = False
        
        # Timer
        self.create_timer(0.5, self.planning_cycle)
        
        self.get_logger().info("Simple Path Planner - Only replan for obstacles")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        ranges = msg.ranges
        self.obstacle_detected = False
        
        for i in range(330, 390):
            idx = i % 360
            if 0.2 < ranges[idx] < 0.8:
                self.obstacle_detected = True
                break

    def create_path(self):
        if not self.current_pose:
            return None
            
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        self.goal_position = (goal_x, goal_y)
        
        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"
        
        # Thêm điểm xuất phát
        self.add_pose_to_path(path, start_x, start_y)
        
        if self.obstacle_detected:
            # Tạo điểm tránh
            avoid_x = start_x + math.cos(current_yaw - math.pi/2) * 1.0
            avoid_y = start_y + math.sin(current_yaw - math.pi/2) * 1.0
            self.add_pose_to_path(path, avoid_x, avoid_y)
            self.get_logger().info("🔄 Created avoidance path")
        
        # Thêm điểm goal
        self.add_pose_to_path(path, goal_x, goal_y)
        
        return path

    def add_pose_to_path(self, path, x, y):
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    def get_yaw_from_quaternion(self, quaternion):
        if not quaternion:
            return 0.0
        try:
            orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            return yaw
        except:
            return 0.0

    def planning_cycle(self):
        path = self.create_path()
        if path:
            self.path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Path planner shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()