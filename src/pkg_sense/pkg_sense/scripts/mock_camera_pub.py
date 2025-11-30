#!/usr/bin/env python3
import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_camera_publisher')

        # publish to the topic your Sense node expects:
        topic = "/camera/camera/image_raw"
        self.publisher = self.create_publisher(Image, topic, 10)

        self.bridge = CvBridge()

        # load image once
        pkg_share = get_package_share_directory("pkg_sense")
        test_img_path = os.path.join(pkg_share, "test_images", "klotski_test.jpg")

        self.img = cv2.imread(test_img_path)
        if self.img is None:
            raise RuntimeError("Failed to load test image")

        # publish at ~2 Hz
        self.timer = self.create_timer(0.5, self.publish_frame)
        self.get_logger().info(f"Mock camera publishing to {topic}")

    def publish_frame(self):
        msg = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
