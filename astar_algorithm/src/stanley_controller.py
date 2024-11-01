#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool
from math import atan2, hypot, sqrt, pi, sin, cos

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

        self.k_stanley = 1.0  # Stanley gain
        self.k_speed = 1.0  # Speed gain

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
        
        # Stanley Controller
        front_axle_x = self.sx + 0.5 * cos(self.yaw)
        front_axle_y = self.sy + 0.5 * sin(self.yaw)
        
        if len(self.rx) == 0 or len(self.ry) == 0:
            self.get_logger().error("Path is empty. Waiting for path...")
            return

        dx = [front_axle_x - icx for icx in self.rx]
        dy = [front_axle_y - icy for icy in self.ry]
        d = [abs(sqrt(idx ** 2 + idy ** 2)) for idx, idy in zip(dx, dy)]

        # if not d:
        #     self.get_logger().error("Distance list is empty. Cannot find nearest point.")
        #     return

        nearest_index = d.index(min(d))
        nearest_point_x = self.rx[nearest_index]
        nearest_point_y = self.ry[nearest_index]
        
        cross_track_error = min(d) * cos(atan2(nearest_point_y - self.sy, nearest_point_x - self.sx) - self.yaw)
        
        path_yaw = atan2(self.ry[nearest_index] - self.sy, self.rx[nearest_index] - self.sx)
        heading_error = path_yaw - self.yaw
        
        if heading_error > pi:
            heading_error -= 2 * pi
        elif heading_error < -pi:
            heading_error += 2 * pi
        
        twist = Twist()
        twist.angular.z = heading_error + atan2(self.k_stanley * cross_track_error, self.k_speed)

        # Linear Velocity Control (as in the original code)
        direction = hypot(self.gx - self.sx, self.gy - self.sy)
        linear_vel_error = direction
        kp_v = 2.0  # Gain for linear velocity

        if abs(heading_error) > 1.0:
            twist.linear.x = 0.0
        else:
            twist.linear.x = kp_v * linear_vel_error

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
