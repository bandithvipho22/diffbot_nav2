import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
from std_srvs.srv import Empty
import sys
import termios
import tty

class MapSaver(Node):

    def __init__(self):
        super().__init__('map_saver')
        self.srv_save_map = self.create_client(SaveMap, 'save_map')
        self.srv_stop_slam = self.create_client(Empty, 'stop_slam')
        self.srv_publish_map = self.create_client(Empty, 'publish_map')

        self.declare_parameter('map_name', 'saved_map')

        self.get_logger().info('Press SPACE to save and publish the map.')

    def save_and_publish_map(self):
        # Call the save_map service
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        request = SaveMap.Request()
        request.map_topic = 'map'
        request.map_url = map_name
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        # Save the map
        while not self.srv_save_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for save_map service...')
        self.srv_save_map.call_async(request)

        # Stop SLAM
        while not self.srv_stop_slam.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for stop_slam service...')
        self.srv_stop_slam.call_async(Empty.Request())

        # Publish the map
        while not self.srv_publish_map.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for publish_map service...')
        self.srv_publish_map.call_async(Empty.Request())

    def listen_for_space_key(self):
        # Listening for the space key press
        tty.setcbreak(sys.stdin.fileno())
        key = sys.stdin.read(1)
        if key == ' ':
            self.get_logger().info('Space key pressed. Saving and publishing map.')
            self.save_and_publish_map()

def main(args=None):
    rclpy.init(args=args)
    node = MapSaver()
    try:
        while rclpy.ok():
            node.listen_for_space_key()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
