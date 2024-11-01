import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.last_pose = None

        # Subscribe to the topic providing the current pose
        self.subscription = self.create_subscription(
            Odometry, 
            '/odom',  
            self.listener_callback,
            10)

        # Publish the pose to the initialpose topic
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10)

    def listener_callback(self, msg):
        # Store the latest pose
        self.last_pose = msg

    def publish_final_pose(self):
        if self.last_pose:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header = self.last_pose.header
            pose_msg.pose.pose = self.last_pose.pose.pose
        
            pose_msg.pose.covariance = [0.0] * 36
            self.publisher.publish(pose_msg)
            self.get_logger().info('Publishing final pose on shutdown.')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()

    try:
        rclpy.spin(node)
    finally:
        node.publish_final_pose()  
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
