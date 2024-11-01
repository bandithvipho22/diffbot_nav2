#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )

        # Subscribe to the /initialpose topic to detect manual pose updates
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initial_pose_callback,
            qos_profile
        )

        # topic for AMCL  /initialpose
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.last_pose = None
        self.continuous_publish = True
        self.manual_update = False

        # Timer to periodically publish the pose
        self.timer = self.create_timer(1.0, self.publish_pose_callback)

    def listener_callback(self, msg):
        if not self.manual_update:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())

                # Convert Odometry to PoseWithCovarianceStamped
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'map'

                # Apply the transform to the odometry pose
                pose_msg.pose.pose.position.x = msg.pose.pose.position.x + transform.transform.translation.x
                pose_msg.pose.pose.position.y = msg.pose.pose.position.y + transform.transform.translation.y
                pose_msg.pose.pose.position.z = msg.pose.pose.position.z + transform.transform.translation.z
                pose_msg.pose.pose.orientation = msg.pose.pose.orientation

        
                # Store the latest pose
                self.last_pose = pose_msg
                self.get_logger().info(f'Received pose: {pose_msg.pose.pose.position.x:.2f}, {pose_msg.pose.pose.position.y:.2f}')

            except TransformException as ex:
                self.get_logger().warn(f'Could not transform odometry to map: {ex}')

    def initial_pose_callback(self, msg):
        self.last_pose = msg
        self.manual_update = True
        self.continuous_publish = False
        self.get_logger().info('Received manual pose update. Stopping continuous publishing.')

    def publish_pose_callback(self):
        if self.last_pose and self.continuous_publish:
            # Publish the latest pose to /initialpose
            self.publisher.publish(self.last_pose)
            self.get_logger().info(f'Publishing pose: {self.last_pose.pose.pose.position.x:.2f}, {self.last_pose.pose.pose.position.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSetter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()