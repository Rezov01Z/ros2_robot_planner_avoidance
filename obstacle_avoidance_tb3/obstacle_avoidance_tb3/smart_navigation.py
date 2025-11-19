#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
import math
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Empty

class PIDController:
    """Bộ điều khiển PID đơn giản"""
    def __init__(self, kp, ki, kd, max_output, min_output=0.0):
        self.kp = kp
        self.ki = ki 
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error, dt):
        if self.last_time is None:
            self.last_time = dt
            return self.kp * error
        
        # Tính tích phân
        self.integral += error * dt
        
        # Tính đạo hàm
        derivative = (error - self.previous_error) / dt if dt > 0 else 0.0
        
        # Tính đầu ra PID
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # Giới hạn đầu ra
        output = max(-self.max_output, min(output, self.max_output))
        
        # Cập nhật trạng thái
        self.previous_error = error
        self.last_time = dt
        
        return output

    def reset(self):
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = None

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
        
        # PID Parameters
        self.declare_parameter('angular_kp', 2.0)  # Tăng Kp cho góc
        self.declare_parameter('angular_ki', 0.02)
        self.declare_parameter('angular_kd', 0.5)  # Tăng Kd để giảm oscillation
        self.declare_parameter('linear_kp', 0.8)
        self.declare_parameter('linear_ki', 0.05)
        self.declare_parameter('linear_kd', 0.1)
        
        # State
        self.current_pose = None
        self.goal_position = None
        self.laser_data = None
        
        # Navigation states
        self.state = "APPROACH_GOAL"
        self.avoidance_direction = 1
        self.avoidance_point_x = 0.0
        self.avoidance_point_y = 0.0
        self.target_avoidance_angle = 0.0
        self.avoidance_start_position = None
        self.turn_back_start_time = None
        
        # Control parameters
        self.max_linear = 0.15
        self.max_angular = 0.6  # Tăng tốc độ góc tối đa
        self.goal_tolerance = 0.3
        self.angle_tolerance = 0.15  # Tăng tolerance để tránh kẹt
        
        # Avoidance parameters
        self.avoidance_distance = 0.3  # Tăng khoảng cách tránh
        self.avoidance_angle = math.pi / 2
        
        # PID Controllers
        self.angular_pid = PIDController(
            kp=self.get_parameter('angular_kp').value,
            ki=self.get_parameter('angular_ki').value,
            kd=self.get_parameter('angular_kd').value,
            max_output=self.max_angular
        )
        
        self.linear_pid = PIDController(
            kp=self.get_parameter('linear_kp').value,
            ki=self.get_parameter('linear_ki').value,
            kd=self.get_parameter('linear_kd').value,
            max_output=self.max_linear
        )
        
        # Timing for PID
        self.last_control_time = self.get_clock().now()
        
        # Timer
        self.create_timer(0.1, self.navigation_loop)
        
        self.get_logger().info("Smart Navigation with Improved PID Control")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        self.laser_data = msg

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
        
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        goal_angle = math.atan2(dy, dx)
        angle_error = goal_angle - current_yaw
        
        # Chuẩn hóa góc về [-pi, pi]
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
            if 0.1 < ranges[idx] < 0.4:
                obstacle_detected = True
                break
                
        return obstacle_detected

    def calculate_avoidance_waypoint(self, current_x, current_y, current_yaw):
        """Tính điểm tránh vật cản (90 độ so với hướng hiện tại)"""
        self.avoidance_direction = 1
        
        avoidance_angle = current_yaw + (self.avoidance_direction * self.avoidance_angle)
        avoidance_x = current_x + self.avoidance_distance * math.cos(avoidance_angle)
        avoidance_y = current_y + self.avoidance_distance * math.sin(avoidance_angle)
        
        return avoidance_x, avoidance_y, avoidance_angle

    def publish_path(self, include_avoidance=False, avoidance_point=None):
        """Publish đường đi"""
        if not self.current_pose or not self.goal_position:
            return
            
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"
        
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x, goal_y = self.goal_position
        
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = current_x
        pose.pose.position.y = current_y
        path.poses.append(pose)
        
        if include_avoidance and avoidance_point:
            avoidance_x, avoidance_y = avoidance_point
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = avoidance_x
            pose.pose.position.y = avoidance_y
            path.poses.append(pose)
        
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        path.poses.append(pose)
        
        self.path_pub.publish(path)

    def calculate_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def navigation_loop(self):
        """Vòng lặp điều hướng chính với PID control cải tiến"""
        if not self.current_pose:
            return
            
        # Tính thời gian delta cho PID
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = current_time
        
        # Tính khoảng cách và hướng đến goal
        angle_error, distance, goal_angle = self.calculate_direction_to_goal()
        
        # Kiểm tra xem đã đến goal chưa
        if distance < self.goal_tolerance:
            self.get_logger().info("🎯 Goal reached!")
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            # Reset PID khi đến đích
            self.angular_pid.reset()
            self.linear_pid.reset()
            return
        
        cmd_vel = Twist()
        current_yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        # STATE MACHINE với PID Control cải tiến
        if self.state == "APPROACH_GOAL":
            if self.check_obstacle_ahead():
                self.get_logger().warn("⚠️ Obstacle detected - starting avoidance sequence")
                self.state = "TURN_TO_AVOID"
                self.avoidance_point_x, self.avoidance_point_y, self.target_avoidance_angle = self.calculate_avoidance_waypoint(
                    current_x, current_y, current_yaw)
                self.avoidance_start_position = (current_x, current_y)
                self.get_logger().info(f"Target avoidance angle: {self.target_avoidance_angle:.2f}")
                # Reset PID khi chuyển trạng thái
                self.angular_pid.reset()
            else:
                # Sử dụng PID cho cả góc và khoảng cách
                angular_vel = self.angular_pid.compute(angle_error, dt)
                linear_vel = self.linear_pid.compute(distance, dt)
                
                # Giảm tốc độ khi góc lệch lớn
                if abs(angle_error) > 1.0:  # ~57 độ
                    linear_vel *= 0.4
                elif abs(angle_error) > 0.5:  # ~29 độ
                    linear_vel *= 0.7
                
                cmd_vel.linear.x = linear_vel
                cmd_vel.angular.z = angular_vel
                
                self.publish_path(include_avoidance=False)
        
        elif self.state == "TURN_TO_AVOID":
            # Sử dụng PID cho việc quay
            angle_diff = self.target_avoidance_angle - current_yaw
            
            # Chuẩn hóa góc
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.get_logger().info(f"Turning to avoid: diff={angle_diff:.2f}")
            
            if abs(angle_diff) < self.angle_tolerance:
                self.get_logger().info("✅ Turn completed - moving forward")
                self.state = "MOVE_AVOID"
                self.avoidance_start_position = (current_x, current_y)
                # Reset PID khi chuyển trạng thái
                self.linear_pid.reset()
            else:
                # Sử dụng PID với tốc độ góc tối đa cao hơn
                angular_vel = self.angular_pid.compute(angle_diff, dt)
                cmd_vel.angular.z = angular_vel
                self.publish_path(include_avoidance=True, avoidance_point=(self.avoidance_point_x, self.avoidance_point_y))
        
        elif self.state == "MOVE_AVOID":
            # Di chuyển thẳng với PID
            start_x, start_y = self.avoidance_start_position
            distance_moved = self.calculate_distance(start_x, start_y, current_x, current_y)
            remaining_distance = self.avoidance_distance - distance_moved
            
            self.get_logger().info(f"Moving to avoidance: {distance_moved:.2f}/{self.avoidance_distance}")
            
            if distance_moved >= self.avoidance_distance:
                self.get_logger().info("✅ Avoidance move completed - turning back to goal")
                self.state = "TURN_BACK_TO_GOAL"
                self.turn_back_start_time = self.get_clock().now()  # Ghi nhận thời gian bắt đầu quay về
                # Reset PID khi chuyển trạng thái
                self.angular_pid.reset()
            else:
                # Sử dụng PID để điều khiển tốc độ di chuyển
                linear_vel = self.linear_pid.compute(remaining_distance, dt)
                cmd_vel.linear.x = linear_vel
                self.publish_path(include_avoidance=True, avoidance_point=(self.avoidance_point_x, self.avoidance_point_y))
        
        elif self.state == "TURN_BACK_TO_GOAL":
            # Sử dụng PID để quay về hướng goal với logic đặc biệt cho góc lớn
            angle_error, distance, goal_angle = self.calculate_direction_to_goal()
            
            # Kiểm tra timeout - nếu quá 10 giây mà không xoay được thì chuyển trạng thái
            current_time = self.get_clock().now()
            turn_back_duration = (current_time - self.turn_back_start_time).nanoseconds / 1e9
            
            if turn_back_duration > 10.0:  # 10 giây timeout
                self.get_logger().warn("🕒 Turn back timeout - forcing state change")
                self.state = "APPROACH_GOAL"
                self.angular_pid.reset()
                self.linear_pid.reset()
                return
            
            self.get_logger().info(f"Turning back to goal: angle error={angle_error:.2f}, duration={turn_back_duration:.1f}s")
            
            if abs(angle_error) < self.angle_tolerance:
                self.get_logger().info("✅ Back to goal direction - resuming navigation")
                self.state = "APPROACH_GOAL"
                # Reset PID khi chuyển trạng thái
                self.angular_pid.reset()
                self.linear_pid.reset()
            else:
                # Đối với góc lớn, sử dụng tốc độ góc cao hơn
                if abs(angle_error) > 1.0:  # Góc lớn hơn 57 độ
                    # Sử dụng tốc độ góc cố định cho góc lớn
                    base_angular_vel = 0.5
                    # Thêm thành phần P để tăng tốc độ khi góc lớn
                    angular_vel = base_angular_vel * (1.0 + 0.3 * min(abs(angle_error), 2.0))
                    angular_vel = min(angular_vel, self.max_angular)
                    angular_vel = angular_vel if angle_error > 0 else -angular_vel
                else:
                    # Đối với góc nhỏ, sử dụng PID bình thường
                    angular_vel = self.angular_pid.compute(angle_error, dt)
                
                cmd_vel.angular.z = angular_vel
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