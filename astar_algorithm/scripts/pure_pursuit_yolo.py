#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool
from math import atan2, hypot, sqrt, pi, sin, cos

from yolov8_msgs.msg import Yolov8Inference, ObjectDistance

def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

class P_Path(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.path_subscription = self.create_subscription(Path, '/robot_path', self.path_callback, 1)
        self.obstacle_detected_subscription = self.create_subscription(Bool, '/obstacle_collision_with_path', self.obstacle_detected_callback, 1)
        self.pp_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pose_estimate_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_estimate_callback, 1)
        self.goal_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 1)
        
        self.distance_sub = self.create_subscription(ObjectDistance, '/object_distance', self.distance_callback, 10)

        self.path = Path()
        self.rx = []
        self.ry = []
        self.gx = 0.0
        self.gy = 0.0
        self.final_yaw = 0.0

        self.start = 0
        
        self.sx_sy_updated = False
        self.estimate_odom_updated = False

        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        self.estimate_odom_x = 0.0
        self.estimate_odom_y = 0.0
        self.goal = 0
        self.obstacle_detected = False

        self.lookahead_distance = 1.0
        self.expansion_size = 2  # for the wall
        self.i = 10  # Initialize the path index
        # self.flag = 2  # Initialize the flag
        self.yaw = 0.0
        
    def distance_callback(self, msg):
        
        self.get_logger().info(f"Received distance message:")
        self.get_logger().info(f"Class: {msg.class_name}")
        self.get_logger().info(f"Distance: {msg.distance:.2f} meters")
        self.get_logger().info(f"Header: {msg.header}")    

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = msg.data
        self.get_logger().info(f'Obstacle detected: {self.obstacle_detected}')

    def pose_estimate_callback(self, msg):
        if not self.estimate_odom_updated:
            self.estimate_odom_x = msg.pose.pose.position.x
            self.estimate_odom_y = msg.pose.pose.position.y
            self.sx = self.estimate_odom_x
            self.sy = self.estimate_odom_y
            self.gx = self.sx
            self.gy = self.sy
            self.goal = True
            self.estimate_odom_updated = True
            self.get_logger().info('Pose estimate updated')

    def odom_callback(self, msg):
        pose = msg.pose.pose
        self.sx = pose.position.x
        self.sy = pose.position.y

        if self.sx_sy_updated and self.goal:
            self.sx += self.estimate_odom_x
            self.sy += self.estimate_odom_y

        self.get_logger().info('Odom received: x = %f, y = %f' % (self.sx, self.sy))
        self.sx_sy_updated = True

        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z

        self.yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        if len(self.rx) > 2:
            for i in range(len(self.rx)):
                distance_to_point = distance(self.sx, self.sy, self.rx[i], self.ry[i])
                if distance_to_point < 0.2:
                    self.gx = self.rx[i]
                    self.gy = self.ry[i]
                    break
        
        
        # Pure Pursuit for Angular Velocity
        alpha = atan2(self.gy - self.sy, self.gx - self.sx) - self.yaw
        if alpha > pi:
            alpha -= 2 * pi
        elif alpha < -pi:
            alpha += 2 * pi

        # kp_alpha = 1.0  # Gain for steering angle (alpha)
        # twist = Twist()
        # twist.angular.z = kp_alpha * alpha
        
        # k_angular = 1.5
        ld = self.lookahead_distance
        
        curvature = 2 * sin(alpha) / ld
        twist = Twist()
        twist.angular.z = curvature #* k_angular 
        
        # twist.angular.z = curvature * alpha 

        # Linear Velocity Control (as in the original code)
        direction = hypot(self.gx - self.sx, self.gy - self.sy)
        velocity_error = direction
        kp_v = 2.0 #1.5  # Gain for linear velocity

        if abs(alpha) > 1.0:
            twist.linear.x = 0.0
        else:
            twist.linear.x = kp_v * velocity_error
        
        # if abs(alpha) > 1.0:
        #     twist.linear.x = 0.0
        #     twist.angular.z = curvature * alpha
        # else:
        #     twist.linear.x = kp_v * velocity_error
        #     twist.angular.z = curvature * alpha

        if self.obstacle_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Obstacle detected, stopping')

        twist.linear.x = max(min(twist.linear.x, 0.30), -0.30)
        twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

        if self.start:
            self.pp_controller.publish(twist)
            self.get_logger().info(f'Publishing twist: {twist}')

    def goal_pose_callback(self, msg):
        self.estimate_odom_updated = True

    def path_callback(self, msg):
        self.path = msg
        self.rx = [pose.pose.position.x for pose in msg.poses]
        self.ry = [pose.pose.position.y for pose in msg.poses]
        if len(msg.poses) > 0:
            self.final_orientation = msg.poses[-1].pose.orientation
            qw = self.final_orientation.w
            qx = self.final_orientation.x
            qy = self.final_orientation.y
            qz = self.final_orientation.z
            self.final_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        self.start = True  # Ensure the robot starts moving
        self.get_logger().info('Path received and started')

def main(args=None):
    rclpy.init(args=args)
    node = P_Path()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
