#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import Bool
from math import atan2, hypot, sqrt, pi


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

    # def pure_pursuit(self, current_x, current_y, current_heading, path, index):
    #     closest_point = None
    #     v = 0.5  # Default speed
    #     for i in range(index, len(path)):
    #         x = path[i][0]
    #         y = path[i][1]
    #         distance_to_point = hypot(current_x - x, current_y - y)
    #         if self.lookahead_distance < distance_to_point:
    #             closest_point = (x, y)
    #             index = i
    #             break
    #     if closest_point is not None:
    #         target_heading = atan2(closest_point[1] - current_y, closest_point[0] - current_x)
    #         desired_steering_angle = target_heading - current_heading
    #     else:
    #         target_heading = atan2(path[-1][1] - current_y, path[-1][0] - current_x)
    #         desired_steering_angle = target_heading - current_heading
    #         index = len(path) - 1

    #     if desired_steering_angle > pi:
    #         desired_steering_angle -= 2 * pi
    #     elif desired_steering_angle < -pi:
    #         desired_steering_angle += 2 * pi

    #     if abs(desired_steering_angle) > pi / 6:
    #         sign = 1 if desired_steering_angle > 0 else -1
    #         desired_steering_angle = sign * pi / 4
    #         v = 0.0

    #     return v, desired_steering_angle, index

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
                distance_to_trajectory = distance(self.sx, self.sy, self.rx[i], self.ry[i])
                if distance_to_trajectory < 0.2:
                    self.gx = self.rx[i]
                    self.gy = self.ry[i]
                    break
        
        direction = [self.gx - self.sx, self.gy - self.sy]
        velocity_error = hypot(direction[0], direction[1])
        theta_error = atan2(direction[1], direction[0]) - self.yaw

        if theta_error > pi:
            theta_error -= 2 * pi
        elif theta_error < -pi:
            theta_error += 2 * pi
        
        kp_v = 1.5
        
        twist = Twist()
        
        if abs(theta_error) > 1.0:
            twist.linear.x = 0.0
            twist.angular.z = self.lookahead_distance * theta_error
        else:
            twist.linear.x = kp_v * velocity_error
            twist.angular.z = self.lookahead_distance * theta_error
        
        if len(self.rx) > 0 and len(self.ry) > 0:  # Ensure rx and ry are not empty
            if distance(self.sx, self.sy, self.rx[0], self.ry[0]) < 0.2:
                theta_error = self.final_yaw - self.yaw
                self.get_logger().info(f'Theta difference: {theta_error}')
                if theta_error > pi:
                    theta_error -= 2 * pi
                elif theta_error < -pi:
                    theta_error += 2 * pi

                twist.linear.x = 0.0

                if abs(theta_error) > 0.5:
                    twist.angular.z = theta_error
                else:
                    twist.angular.z = 0.0

        if self.obstacle_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('Obstacle detected, stopping')

        twist.linear.x = max(min(twist.linear.x, 0.26), -0.26)
        twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

        if self.start:
            self.pp_controller.publish(twist)
            self.get_logger().info(f'Publishing twist: {twist}')

    def goal_pose_callback(self, msg):
        # self.goal = True
        # self.start = False
        # self.i = 0  # Reset the path index for the new goal
        # self.get_logger().info('New goal received')
        # self.rx.clear()  # Clear previous path
        # self.ry.clear()
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
