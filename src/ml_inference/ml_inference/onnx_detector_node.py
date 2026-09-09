#!/usr/bin/env python3
"""ROS 2 node for object detection using ONNX Runtime."""

import os
import time

import cv2
import numpy as np
import onnxruntime as ort

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


# Supported output format presets
SUPPORTED_FORMATS = ['onnx_zoo', 'yolo', 'normalized']

# Map device parameter to ONNX Runtime execution providers
DEVICE_TO_PROVIDERS = {
    'cpu': ['CPUExecutionProvider'],
    'cuda': ['CUDAExecutionProvider', 'CPUExecutionProvider'],
    'tensorrt': [
        'TensorrtExecutionProvider',
        'CUDAExecutionProvider',
        'CPUExecutionProvider',
    ],
}

# YOLO models use contiguous 0-79 class IDs.  Map them to the
# official COCO 91-category IDs (1-90 with gaps) used by COCO_LABELS.
COCO_80_TO_91 = [
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 27, 28, 31, 32, 33, 34,
    35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
    46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
    56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
    67, 70, 72, 73, 74, 75, 76, 77, 78, 79,
    80, 81, 82, 84, 85, 86, 87, 88, 89, 90,
]


class OnnxDetectorNode(Node):
    """Object detection node using ONNX Runtime."""

    def __init__(self):
        """Initialize the ONNX detector node."""
        super().__init__('onnx_detector_node')

        # Declare ROS parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_format', 'onnx_zoo')
        self.declare_parameter('input_size', 640)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.45)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('image_topic', '/camera/image_raw')

        # Read parameters
        model_path = (
            self.get_parameter('model_path')
            .get_parameter_value().string_value
        )
        self.model_format = (
            self.get_parameter('model_format')
            .get_parameter_value().string_value
        )
        self.input_size = (
            self.get_parameter('input_size')
            .get_parameter_value().integer_value
        )
        self.confidence_threshold = (
            self.get_parameter('confidence_threshold')
            .get_parameter_value().double_value
        )
        self.nms_threshold = (
            self.get_parameter('nms_threshold')
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

        # Validate parameters
        if not model_path:
            raise ValueError(
                'model_path parameter is required. '
                'Set it to the path of an .onnx model file.'
            )
        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f'ONNX model not found: {model_path}'
            )
        if self.model_format not in SUPPORTED_FORMATS:
            raise ValueError(
                f'Unknown model_format: {self.model_format}. '
                f'Supported: {SUPPORTED_FORMATS}'
            )

        # Set up execution providers with fallback
        self.providers = self._resolve_providers(device_str)
        self.get_logger().info(
            f'Using execution providers: {self.providers}'
        )

        # Load ONNX model
        self.session = self._load_model(model_path)

        # Store input metadata for inference calls
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.input_type = model_input.type  # e.g. 'tensor(uint8)'

        # cv_bridge for Image <-> numpy conversion
        self.bridge = CvBridge()

        # Output format dispatch table
        self._output_parsers = {
            'onnx_zoo': self._parse_onnx_zoo,
            'yolo': self._parse_yolo,
            'normalized': self._parse_normalized,
        }

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

        # Skip frames while inference is running
        self._processing = False

        self.get_logger().info(
            f'ONNX model loaded ({self.model_format} format), '
            f'subscribed to {image_topic}, '
            f'publishing detections on ~/detections'
        )

    def _resolve_providers(self, device_str):
        """Map device string to execution providers with fallback."""
        # Normalize cuda:N to cuda
        device_key = device_str.split(':')[0].lower()

        if device_key not in DEVICE_TO_PROVIDERS:
            self.get_logger().warn(
                f'Unknown device: {device_str}. '
                'Falling back to CPU.'
            )
            device_key = 'cpu'

        requested = DEVICE_TO_PROVIDERS[device_key]
        available = ort.get_available_providers()

        # Filter to only available providers
        providers = [p for p in requested if p in available]
        if not providers:
            self.get_logger().warn(
                f'Requested providers {requested} not available. '
                f'Available: {available}. Falling back to CPU.'
            )
            providers = ['CPUExecutionProvider']
        elif providers != requested:
            self.get_logger().warn(
                f'Some providers unavailable. Using: {providers}'
            )

        return providers

    def _load_model(self, model_path):
        """Load an ONNX model into an InferenceSession."""
        self.get_logger().info(f'Loading ONNX model from: {model_path}')

        session_options = ort.SessionOptions()
        session_options.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )

        session = ort.InferenceSession(
            model_path,
            sess_options=session_options,
            providers=self.providers,
        )

        # Log model metadata
        inputs = session.get_inputs()
        outputs = session.get_outputs()
        self.get_logger().info(
            f'Model inputs: '
            f'{[(i.name, i.shape) for i in inputs]}'
        )
        self.get_logger().info(
            f'Model outputs: '
            f'{[(o.name, o.shape) for o in outputs]}'
        )

        return session

    def _preprocess(self, cv_image):
        """Resize and normalize image, return (tensor, orig_h, orig_w)."""
        orig_h, orig_w = cv_image.shape[:2]

        # Models expecting uint8 (e.g. SSD MobileNet) take NHWC as-is
        if 'uint8' in self.input_type:
            tensor = np.expand_dims(cv_image, axis=0)  # NHWC
            return tensor, orig_h, orig_w

        resized = cv2.resize(
            cv_image, (self.input_size, self.input_size),
            interpolation=cv2.INTER_LINEAR,
        )

        # HWC uint8 -> CHW float32 [0, 1]
        tensor = resized.astype(np.float32) / 255.0
        tensor = np.transpose(tensor, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0)

        return tensor, orig_h, orig_w

    def image_callback(self, msg):
        """Process incoming image messages."""
        if self._processing:
            return
        self._processing = True

        try:
            # Convert ROS Image -> RGB numpy array
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='rgb8'
            )

            # Preprocess
            input_tensor, orig_h, orig_w = (
                self._preprocess(cv_image)
            )

            # Run inference
            t_start = time.perf_counter()
            outputs = self.session.run(
                None, {self.input_name: input_tensor}
            )
            inference_ms = (time.perf_counter() - t_start) * 1000.0
            self.get_logger().info(
                f'Inference time: {inference_ms:.1f} ms'
            )

            # Parse output based on model format
            parser = self._output_parsers[self.model_format]
            detections = parser(outputs, orig_h, orig_w)

            # Build and publish Detection2DArray
            det_array = self._build_detection_array(
                detections, msg.header
            )
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

    def _parse_onnx_zoo(self, outputs, orig_h, orig_w):
        """Parse ONNX Model Zoo format (boxes, labels, scores)."""
        # Squeeze batch dimension if present (e.g. [1, N, 4] -> [N, 4])
        boxes = np.squeeze(outputs[0])
        labels = np.squeeze(outputs[1])
        scores = np.squeeze(outputs[2])

        scale_x = orig_w / self.input_size
        scale_y = orig_h / self.input_size

        detections = []
        for i in range(len(scores)):
            score = float(scores[i])
            if score < self.confidence_threshold:
                continue
            x1 = float(boxes[i][0]) * scale_x
            y1 = float(boxes[i][1]) * scale_y
            x2 = float(boxes[i][2]) * scale_x
            y2 = float(boxes[i][3]) * scale_y
            class_id = int(labels[i])
            detections.append((x1, y1, x2, y2, class_id, score))

        return detections

    def _parse_normalized(self, outputs, orig_h, orig_w):
        """Parse normalized [0,1] coordinate format (TF convention)."""
        # Squeeze batch dimension if present (e.g. [1, N, 4] -> [N, 4])
        boxes = np.squeeze(outputs[0])
        labels = np.squeeze(outputs[1])
        scores = np.squeeze(outputs[2])

        detections = []
        for i in range(len(scores)):
            score = float(scores[i])
            if score < self.confidence_threshold:
                continue
            y1 = float(boxes[i][0]) * orig_h
            x1 = float(boxes[i][1]) * orig_w
            y2 = float(boxes[i][2]) * orig_h
            x2 = float(boxes[i][3]) * orig_w
            class_id = int(labels[i])
            detections.append((x1, y1, x2, y2, class_id, score))

        return detections

    def _parse_yolo(self, outputs, orig_h, orig_w):
        """Parse YOLO-style single-tensor output with NMS."""
        predictions = outputs[0][0]  # [N, 85] for COCO

        boxes_cxcywh = predictions[:, :4]
        objectness = predictions[:, 4]
        class_scores = predictions[:, 5:]

        # Combined confidence = objectness * class score
        scores = objectness[:, np.newaxis] * class_scores
        class_ids = np.argmax(scores, axis=1)
        max_scores = np.max(scores, axis=1)

        # Filter by confidence
        mask = max_scores >= self.confidence_threshold
        boxes_cxcywh = boxes_cxcywh[mask]
        class_ids = class_ids[mask]
        max_scores = max_scores[mask]

        if len(max_scores) == 0:
            return []

        # Convert cx,cy,w,h -> x1,y1,x2,y2 in original image coords
        scale_x = orig_w / self.input_size
        scale_y = orig_h / self.input_size

        x1 = (boxes_cxcywh[:, 0] - boxes_cxcywh[:, 2] / 2.0) * scale_x
        y1 = (boxes_cxcywh[:, 1] - boxes_cxcywh[:, 3] / 2.0) * scale_y
        x2 = (boxes_cxcywh[:, 0] + boxes_cxcywh[:, 2] / 2.0) * scale_x
        y2 = (boxes_cxcywh[:, 1] + boxes_cxcywh[:, 3] / 2.0) * scale_y

        # NMS via OpenCV
        boxes_for_nms = np.stack(
            [x1, y1, x2 - x1, y2 - y1], axis=1
        )
        indices = cv2.dnn.NMSBoxes(
            boxes_for_nms.tolist(),
            max_scores.tolist(),
            self.confidence_threshold,
            self.nms_threshold,
        )

        detections = []
        if len(indices) > 0:
            for idx in indices.flatten():
                # Map YOLO 0-79 IDs to COCO 1-90 IDs
                yolo_id = int(class_ids[idx])
                if yolo_id < len(COCO_80_TO_91):
                    coco_id = COCO_80_TO_91[yolo_id]
                else:
                    coco_id = yolo_id
                detections.append((
                    float(x1[idx]), float(y1[idx]),
                    float(x2[idx]), float(y2[idx]),
                    coco_id, float(max_scores[idx]),
                ))

        return detections

    def _build_detection_array(self, detections, header):
        """Build Detection2DArray from parsed detections."""
        det_array = Detection2DArray()
        det_array.header = header

        for x1, y1, x2, y2, class_id, score in detections:
            detection = Detection2D()
            detection.header = header

            # BBox: convert x1,y1,x2,y2 to center + size
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            # Classification result
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(class_id)
            hyp.hypothesis.score = score
            detection.results.append(hyp)

            # Human-readable label
            detection.id = COCO_LABELS.get(
                class_id, f'class_{class_id}'
            )

            det_array.detections.append(detection)

        return det_array


def main(args=None):
    """Run the ONNX detector node."""
    rclpy.init(args=args)
    node = OnnxDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
