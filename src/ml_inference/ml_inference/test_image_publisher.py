#!/usr/bin/env python3
"""Publish a test image for sanity-checking the detector node."""

from ament_index_python.packages import get_package_share_directory
import cv2
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image


class TestImagePublisher(Node):
    """Load and repeatedly publish a test image on the camera topic."""

    def __init__(self):
        super().__init__('test_image_publisher')

        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('image_topic', '/camera/image_raw')

        publish_rate = (
            self.get_parameter('publish_rate')
            .get_parameter_value().double_value
        )
        image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value().string_value
        )

        # Load and resize the puppy image once
        pkg_dir = get_package_share_directory('ml_inference')
        image_path = os.path.join(pkg_dir, 'resource', 'puppy.jpeg')
        bgr = cv2.imread(image_path)
        if bgr is None:
            raise RuntimeError(f'Failed to load image: {image_path}')
        bgr = cv2.resize(bgr, (500, 500))
        self.image_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        self.get_logger().info(f'Loaded test image from {image_path} (500x500)')

        # Match the detector's sensor QoS
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.publisher = self.create_publisher(
            Image, image_topic, sensor_qos
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)
        self.frame_count = 0

        self.get_logger().info(
            f'Publishing 500x500 puppy image on {image_topic} '
            f'at {publish_rate} Hz'
        )

    def publish_image(self):
        """Publish the loaded test image."""
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'test_camera'
        msg.height = 500
        msg.width = 500
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 500 * 3
        msg.data = self.image_rgb.tobytes()

        self.publisher.publish(msg)
        self.frame_count += 1
        self.get_logger().info(
            f'Published frame {self.frame_count}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
