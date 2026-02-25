#!/usr/bin/env python3
"""ROS 2 node for object detection using torchvision pretrained models."""

import time

import torch
import torchvision
from torchvision.transforms import functional as F

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from cv_bridge import CvBridge

from ml_inference.coco_labels import COCO_LABELS


# Supported torchvision detection models (all share the same output API)
SUPPORTED_MODELS = {
    'fasterrcnn_mobilenet_v3_large_fpn':
        torchvision.models.detection.fasterrcnn_mobilenet_v3_large_fpn,
    'fasterrcnn_resnet50_fpn':
        torchvision.models.detection.fasterrcnn_resnet50_fpn,
    'fcos_resnet50_fpn':
        torchvision.models.detection.fcos_resnet50_fpn,
    'ssd300_vgg16':
        torchvision.models.detection.ssd300_vgg16,
    'ssdlite320_mobilenet_v3_large':
        torchvision.models.detection.ssdlite320_mobilenet_v3_large,
}


class DetectorNode(Node):
    """Object detection node using torchvision pretrained models."""

    def __init__(self):
        super().__init__('detector_node')

        # Declare ROS parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter(
            'model_name', 'fasterrcnn_mobilenet_v3_large_fpn'
        )
        self.declare_parameter('model_path', '')

        # Read parameters
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold')
            .get_parameter_value().double_value
        )
        device_str = (
            self.get_parameter('device')
            .get_parameter_value().string_value
        )
        image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value().string_value
        )
        model_name = (
            self.get_parameter('model_name')
            .get_parameter_value().string_value
        )
        model_path = (
            self.get_parameter('model_path')
            .get_parameter_value().string_value
        )

        # Set up device with CUDA fallback
        if device_str.startswith('cuda') and not torch.cuda.is_available():
            self.get_logger().warn(
                f'CUDA requested ({device_str}) but not available. '
                'Falling back to CPU.'
            )
            device_str = 'cpu'
        self.device = torch.device(device_str)
        self.get_logger().info(f'Using device: {self.device}')

        # Load model
        self.model = self._load_model(model_name, model_path)
        self.model.to(self.device)
        self.model.eval()
        self.get_logger().info(
            f'Model {model_name} loaded and ready for inference'
        )

        # cv_bridge for Image <-> numpy conversion
        self.bridge = CvBridge()

        # QoS: sensor data profile (best effort, keep last 1)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            sensor_qos,
        )

        # Publisher
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '~/detections',
            10,
        )

        # Skip frames while inference is running (single-threaded spin)
        self._processing = False

        self.get_logger().info(
            f'Subscribed to {image_topic}, '
            f'publishing detections on ~/detections'
        )

    def _load_model(self, model_name, model_path):
        """Load a torchvision detection model."""
        if model_name not in SUPPORTED_MODELS:
            self.get_logger().error(
                f'Unknown model: {model_name}. '
                f'Supported: {list(SUPPORTED_MODELS.keys())}'
            )
            raise ValueError(f'Unsupported model: {model_name}')

        model_factory = SUPPORTED_MODELS[model_name]

        if model_path:
            self.get_logger().info(
                f'Loading model weights from: {model_path}'
            )
            model = model_factory(weights=None)
            state_dict = torch.load(
                model_path, map_location=self.device,
                weights_only=True,
            )
            model.load_state_dict(state_dict)
        else:
            self.get_logger().info(
                f'Loading pretrained {model_name} '
                '(downloading if not cached)...'
            )
            model = model_factory(weights='DEFAULT')

        return model

    def image_callback(self, msg):
        """Process incoming image messages."""
        if self._processing:
            return
        self._processing = True

        try:
            # Convert ROS Image -> RGB numpy array -> tensor [C, H, W]
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='rgb8'
            )
            input_tensor = F.to_tensor(cv_image).to(self.device)

            # Run inference
            t_start = time.perf_counter()
            with torch.no_grad():
                predictions = self.model([input_tensor])[0]
            inference_ms = (time.perf_counter() - t_start) * 1000.0
            self.get_logger().info(
                f'Inference time: {inference_ms:.1f} ms'
            )

            # Build Detection2DArray message
            det_array = Detection2DArray()
            det_array.header = msg.header

            boxes = predictions['boxes'].cpu()
            labels = predictions['labels'].cpu()
            scores = predictions['scores'].cpu()

            for box, label, score in zip(boxes, labels, scores):
                if score.item() < self.confidence_threshold:
                    continue

                detection = Detection2D()
                detection.header = msg.header

                # BBox: torchvision gives [x1, y1, x2, y2]
                x1, y1, x2, y2 = box.tolist()
                detection.bbox.center.position.x = (x1 + x2) / 2.0
                detection.bbox.center.position.y = (y1 + y2) / 2.0
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                # Classification result
                hyp = ObjectHypothesisWithPose()
                class_id = label.item()
                hyp.hypothesis.class_id = str(class_id)
                hyp.hypothesis.score = score.item()
                detection.results.append(hyp)

                # Human-readable label as the detection ID
                detection.id = COCO_LABELS.get(
                    class_id, f'class_{class_id}'
                )

                det_array.detections.append(detection)

            self.detection_pub.publish(det_array)

            if det_array.detections:
                summary = ', '.join(
                    f'{d.id}({d.results[0].hypothesis.score:.2f})'
                    for d in det_array.detections[:5]
                )
                extra = len(det_array.detections) - 5
                if extra > 0:
                    summary += f', ... +{extra} more'
                self.get_logger().info(
                    f'Detected {len(det_array.detections)} objects: '
                    f'{summary}'
                )

        except Exception as e:
            self.get_logger().error(f'Inference failed: {e}')
        finally:
            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
