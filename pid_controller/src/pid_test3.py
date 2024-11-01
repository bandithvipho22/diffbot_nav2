#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 

class Controller_Node(Node):
    
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")

        self.desired_x = 5.0  # Adjust as needed
        self.desired_y = 5.0  # Adjust as needed

        # Publisher and Subscriber
        
        self.my_odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.my_vel_command = self.create_publisher(Twist, "/cmd_vel", 10)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        # twist = msg.twist.twist

        # Extracting position and orientation data from odometry message
        current_x = pose.position.x
        current_y = pose.position.y

        # Extracting yaw from orientation quaternion
        orientation = pose.orientation
        _, _, current_theta = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        self.get_logger().info(f"Current x={current_x} current y={current_y} and current angle = {current_theta}")
        
        # Calculate errors in position
        err_x = self.desired_x - current_x
        err_y = self.desired_y - current_y
        err_dist = math.sqrt(err_x**2 + err_y**2)

        self.get_logger().info(f"Error in x {err_x} and error in y {err_y}")

        # Desired heading based on the position error
        desired_theta = math.atan2(err_y, err_x)
        
        err_theta = desired_theta - current_theta

        # Ensure angle is within the range [-pi, pi]
        err_theta = self.normalize_angle(err_theta)

        self.get_logger().info(f"Desired Angle = {desired_theta} current angle {current_theta} Error angle {err_theta}")

        # PID constants for linear and angular velocity control
        Kp_dist = 1.8
        Kp_theta = 1.5

        # PID control for linear velocity
        l_v = Kp_dist * err_dist

        # PID control for angular velocity
        a_v = Kp_theta * err_theta

        # Send the velocities
        self.send_velocity(l_v, a_v)

    def send_velocity(self, l_v, a_v):
        self.get_logger().info(f"Commanding linear = {l_v} and angular = {a_v}")
        my_msg = Twist()
        my_msg.linear.x = l_v
        my_msg.angular.z = a_v
        self.my_vel_command.publish(my_msg)

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
