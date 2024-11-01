#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RobotPoseAndMapPublisher(Node):
    def __init__(self):
        super().__init__('robot_pose')
        
        # Create TF listener and broadcaster
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publisher for static map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.initial_transform = None
        
        # Timer to capture initial pose, publish TF, and publish map
        self.timer = self.create_timer(0.1, self.publish_callback)

    def publish_callback(self):
        if self.initial_transform is None:
            try:
                # Try to get the transform from map to base_link
                self.initial_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                self.get_logger().info(f'Captured initial pose: '
                                       f'x={self.initial_transform.transform.translation.x:.2f}, '
                                       f'y={self.initial_transform.transform.translation.y:.2f}')
                
                # Publish initial static map
                self.publish_static_map()
            
            except Exception as ex:
                self.get_logger().warn(f'Could not transform from map to base_link: {ex}')
                return
        
        if self.initial_transform:
            # Create a new transform with updated timestamp
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.transform = self.initial_transform.transform
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform)

    def publish_static_map(self):
        # Create a simple empty map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
     
        self.map_publisher.publish(map_msg)
        self.get_logger().info('Published static map')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPoseAndMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()