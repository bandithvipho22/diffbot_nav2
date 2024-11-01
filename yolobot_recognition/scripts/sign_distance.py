#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import Yolov8Inference, ObjectDistance
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

class DistanceCalculator(Node):

    def __init__(self):
        super().__init__('distance_calculator')
        
        self.inference_subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.inference_callback,
            10)
        
        self.publisher = self.create_publisher(
            ObjectDistance,
            '/object_distance',
            10)

        self.camera_resolution = (640, 480)  # Camera frame size in pixels
        self.object_real_width = 0.3  # Example real width of the object in meters
        self.focal_length = 500  # Example focal length in pixels (hypothetical)

    def inference_callback(self, msg):
        for inference_result in msg.yolov8_inference:
            if inference_result.class_name in ["stop sign", "person"]:
                distance = self.calculate_distance_from_size(inference_result.width)
                
                self.get_logger().info(f"Class: {inference_result.class_name}, Distance: {distance:.2f} meters")
                
                distance_msg = ObjectDistance()
                distance_msg.header = Header()
                distance_msg.header.stamp = self.get_clock().now().to_msg()
                distance_msg.header.frame_id = "distance_calculation"
                distance_msg.class_name = inference_result.class_name
                distance_msg.distance = float(distance)
                
                self.publisher.publish(distance_msg)

    def calculate_distance_from_size(self, width_pixels):
        # Calculate distance based on object width in pixels
        
        # Convert width_pixels to actual width in meters
        object_width_meters = (self.object_real_width * width_pixels) / self.camera_resolution[0]
        
        # Calculate distance using the formula: distance = (real_width * focal_length) / object_width
        distance_w = (self.object_real_width * self.focal_length) / object_width_meters
        distance = float(distance_w / 1000.0)
        return distance
    # def calculate_distance_from_size(self, width_pixels):
    #     # Assuming focal_length is provided in millimeters
    #     focal_length_mm = 50  # Example focal length in millimeters
    #     focal_length_m = focal_length_mm / 1000.0  # Convert focal length to meters
        
    #     # Convert width_pixels to actual width in meters
    #     object_width_meters = (self.object_real_width * width_pixels) / self.camera_resolution[0]
        
    #     # Calculate distance using the formula: distance = (real_width * focal_length) / object_width
    #     distance = (self.object_real_width * focal_length_m) / object_width_meters
    #     return distance


def main(args=None):
    rclpy.init(args=args)
    distance_calculator = DistanceCalculator()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
