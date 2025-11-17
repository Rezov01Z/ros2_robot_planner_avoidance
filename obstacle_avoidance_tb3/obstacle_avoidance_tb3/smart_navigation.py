#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty

class SmartNavigation(Node):
    def __init__(self):
        super().__init__('smart_navigation')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', -1.0)
        
        # State
        self.current_pose = None
        self.goal_position = None
        self.laser_data = None
        
        # Navigation states
        self.state = "APPROACH_GOAL"  # States: APPROACH_GOAL, TURN_TO_AVOID, MOVE_AVOID, TURN_BACK_TO_GOAL
        self.avoidance_start_pose = None
        self.avoidance_direction = 1  # 1 for right, -1 for left
        self.avoidance_point = None
        self.target_avoidance_angle = None
        self.avoidance_start_position = None
        
        # Control parameters
        self.max_linear = 0.15
        self.max_angular = 0.4
        self.goal_tolerance = 0.3
        self.angle_tolerance = 0.1
        
        # Avoidance parameters
        self.avoidance_distance = 0.2  # Distance to move during avoidance
        self.avoidance_angle = math.pi / 2  # 90 degrees
        
        # Timer
        self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info("Smart Navigation - Sequential Avoidance")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.laser_data = msg
    def reset_simulation(self):
        """Reset toàn bộ simulation về trạng thái ban đầu"""
        client = self.create_client(Empty, '/reset_simulation')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /reset_simulation not available, waiting...')
        
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Simulation reset successfully')
        else:
            self.get_logger().error('Failed to reset simulation')
        
        # Đợi một chút để simulation ổn định
        self.create_timer(2.0, self._delayed_init, oneshot=True)

    def _delayed_init(self):
        """Khởi tạo sau khi reset simulation"""
        self.get_logger().info("Simulation ready - starting navigation")
    def get_yaw_from_quaternion(self, quaternion):
        if not quaternion:
            return 0.0
        try:
            orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            return yaw
        except:
            return 0.0

    def calculate_direction_to_goal(self):
        """Tính hướng và khoảng cách đến goal"""
        if not self.current_pose:
            return 0.0, 0.0, float('inf')
            
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        self.goal_position = (goal_x, goal_y)
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        
        # Vector đến goal
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Góc đến goal
        goal_angle = math.atan2(dy, dx)
        angle_error = goal_angle - current_yaw
        
        # Chuẩn hóa góc
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        return angle_error, distance, goal_angle

    def check_obstacle_ahead(self):
        """Kiểm tra vật cản phía trước"""
        if not self.laser_data:
            return False
            
        ranges = self.laser_data.ranges
        obstacle_detected = False
        
        # Kiểm tra vật cản trong vùng phía trước (330-30 độ)
        for i in range(330, 390):
            idx = i % 360
            if 0.1 < ranges[idx] < 0.4:  # Vật cản trong khoảng 0.1-0.6m
                obstacle_detected = True
                break
                
        return obstacle_detected

    def calculate_avoidance_waypoint(self, current_x, current_y, current_yaw):
        """Tính điểm tránh vật cản (90 độ so với hướng hiện tại)"""
        # Xác định hướng tránh (ưu tiên phải)
        self.avoidance_direction = 1  # 1 = phải, -1 = trái
        
        # Tính góc tránh: 90 độ so với hướng hiện tại
        avoidance_angle = current_yaw + (self.avoidance_direction * self.avoidance_angle)
        
        # Tính vị trí điểm tránh
        avoidance_x = current_x + self.avoidance_distance * math.cos(avoidance_angle)
        avoidance_y = current_y + self.avoidance_distance * math.sin(avoidance_angle)
        
        return avoidance_x, avoidance_y, avoidance_angle

    def publish_path(self, include_avoidance=False, avoidance_point=None):
        """Publish đường đi (có thể bao gồm điểm tránh)"""
        if not self.current_pose or not self.goal_position:
            return
            
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x, goal_y = self.goal_position
        
        # Thêm vị trí hiện tại
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        path.poses.append(pose)
        
        # Thêm điểm tránh nếu có
        if include_avoidance and avoidance_point:
            avoidance_x, avoidance_y = avoidance_point
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = avoidance_x
            pose.pose.position.y = avoidance_y
            path.poses.append(pose)
        
        # Thêm goal
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        path.poses.append(pose)
        
        self.path_pub.publish(path)

    def calculate_distance(self, x1, y1, x2, y2):
        """Tính khoảng cách giữa hai điểm"""
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def navigation_loop(self):
        """Vòng lặp điều hướng chính với logic tuần tự"""
        if not self.current_pose:
            return
            
        # Tính khoảng cách và hướng đến goal
        angle_error, distance, goal_angle = self.calculate_direction_to_goal()
        
        # Kiểm tra xem đã đến goal chưa
        if distance < self.goal_tolerance:
            self.get_logger().info("🎯 Goal reached!")
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        cmd_vel = Twist()
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # STATE MACHINE
        if self.state == "APPROACH_GOAL":
            # Kiểm tra vật cản
            if self.check_obstacle_ahead():
                self.get_logger().warn("⚠️ Obstacle detected - starting avoidance sequence")
                self.state = "TURN_TO_AVOID"
                # Tính toán điểm tránh và góc mục tiêu
                self.avoidance_point_x, self.avoidance_point_y, self.target_avoidance_angle = self.calculate_avoidance_waypoint(
                    current_x, current_y, current_yaw)
                self.avoidance_start_position = (current_x, current_y)
                self.get_logger().info(f"Target avoidance angle: {self.target_avoidance_angle:.2f}")
                self.get_logger().info(f"Avoidance point: ({self.avoidance_point_x:.2f}, {self.avoidance_point_y:.2f})")
            else:
                # Điều khiển bình thường đến goal
                linear_vel = min(self.max_linear, distance * 0.3)
                angular_vel = angle_error * 1.0
                
                if abs(angle_error) > 0.5:
                    linear_vel *= 0.7
                    
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                
                # Publish path thẳng đến goal
                self.publish_path(include_avoidance=False)
        
        elif self.state == "TURN_TO_AVOID":
            # Quay 90 độ tại chỗ
            angle_diff = self.target_avoidance_angle - current_yaw
            
            # Chuẩn hóa góc
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.get_logger().info(f"Turning to avoid: current={current_yaw:.2f}, target={self.target_avoidance_angle:.2f}, diff={angle_diff:.2f}")
            
            if abs(angle_diff) < self.angle_tolerance:
                self.get_logger().info("✅ Turn completed - moving forward")
                self.state = "MOVE_AVOID"
                self.avoidance_start_position = (current_x, current_y)  # Reset start position for movement
            else:
                cmd_vel.angular.z = self.avoidance_direction * 0.3
                # Publish path có điểm tránh
                self.publish_path(include_avoidance=True, avoidance_point=(self.avoidance_point_x, self.avoidance_point_y))
        
        elif self.state == "MOVE_AVOID":
            # Di chuyển thẳng về phía trước (theo hướng đã quay)
            start_x, start_y = self.avoidance_start_position
            distance_moved = self.calculate_distance(start_x, start_y, current_x, current_y)
            
            self.get_logger().info(f"Moving to avoidance: {distance_moved:.2f}/{self.avoidance_distance}")
            
            if distance_moved >= self.avoidance_distance:
                self.get_logger().info("✅ Avoidance move completed - turning back to goal")
                self.state = "TURN_BACK_TO_GOAL"
            else:
                cmd_vel.linear.x = 0.1  # Tốc độ chậm để di chuyển chính xác
                # Publish path có điểm tránh
                self.publish_path(include_avoidance=True, avoidance_point=(self.avoidance_point_x, self.avoidance_point_y))
        
        elif self.state == "TURN_BACK_TO_GOAL":
            # Quay lại hướng về goal
            angle_error, distance, goal_angle = self.calculate_direction_to_goal()
            
            self.get_logger().info(f"Turning back to goal: angle error={angle_error:.2f}")
            
            if abs(angle_error) < self.angle_tolerance:
                self.get_logger().info("✅ Back to goal direction - resuming navigation")
                self.state = "APPROACH_GOAL"
            else:
                cmd_vel.angular.z = angle_error * 1.0
                # Publish path thẳng đến goal
                self.publish_path(include_avoidance=False)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SmartNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()