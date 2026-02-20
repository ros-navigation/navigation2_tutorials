#!/usr/bin/env python3
"""ROS2 node for semantic segmentation inference using ONNX Runtime."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import LabelInfo, VisionClass
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory



class SegmentationNode(Node):
    """
    ROS2 node that performs semantic segmentation using ONNX Runtime.

    NOTE:
      This node runs on CPU by default for compatibility with all hardware, but can run on GPU if you install the required 
      ONNX Runtime GPU dependencies. See instructions at:
      https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html#requirements
    """

    def __init__(self):
        super().__init__('segmentation_node')
        
        # Get package share directory using ament_index
        package_share = Path(get_package_share_directory('semantic_segmentation_node'))
        model_path = package_share / 'models' / 'model.onnx'
        config_path = package_share / 'config' / 'ontology.yaml'
        
        # Load config
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        self.class_names = [cls['name'] for cls in config['ontology']['classes']]
        self.class_colors = [cls['color'] for cls in config['ontology']['classes']]  # BGR format
        self.num_classes = len(self.class_names) + 1  # +1 for background
        
        # Get device setting from config
        device = config.get('model', {}).get('device', 'cpu').lower()
        
        self.get_logger().info(f'Loading ONNX model from: {model_path}')
        self.get_logger().info(f'Number of classes: {self.num_classes}')
        self.get_logger().info(f'Device setting: {device}')
        
        # Set providers based on device setting
        if device == 'cuda':
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
        else:
            providers = ['CPUExecutionProvider']
        
        self.session = ort.InferenceSession(str(model_path), providers=providers)
        
        # Get model device
        provider = self.session.get_providers()[0]
        self.get_logger().info(f'Using provider: {provider}')
        
        # Detect model input type (FP32 or FP16)
        input_meta = self.session.get_inputs()[0]
        self.input_dtype = input_meta.type
        self.use_fp16 = 'float16' in str(self.input_dtype).lower()
        self.get_logger().info(f'Model input type: {self.input_dtype} (FP16: {self.use_fp16})')
        
        # Image normalization (ImageNet normalization)
        # Use float16 for mean/std if model expects FP16, otherwise float32
        dtype = np.float16 if self.use_fp16 else np.float32
        self.mean = np.array([0.485, 0.456, 0.406], dtype=dtype).reshape(1, 3, 1, 1)
        self.std = np.array([0.229, 0.224, 0.225], dtype=dtype).reshape(1, 3, 1, 1)
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Declare parameters
        self.declare_parameter('input_topic', '/rgbd_camera/image')
        self.declare_parameter('mask_topic', '/segmentation/mask')
        self.declare_parameter('confidence_topic', '/segmentation/confidence')
        self.declare_parameter('label_info_topic', '/segmentation/label_info')
        self.declare_parameter('overlay_topic', '/segmentation/overlay')
        self.declare_parameter('publish_overlay', True)
        
        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        mask_topic = self.get_parameter('mask_topic').get_parameter_value().string_value
        confidence_topic = self.get_parameter('confidence_topic').get_parameter_value().string_value
        label_info_topic = self.get_parameter('label_info_topic').get_parameter_value().string_value
        overlay_topic = self.get_parameter('overlay_topic').get_parameter_value().string_value
        publish_overlay = self.get_parameter('publish_overlay').get_parameter_value().bool_value
        
        # Create subscribers and publishers
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.mask_publisher = self.create_publisher(
            Image,
            mask_topic,
            10
        )
        
        self.confidence_publisher = self.create_publisher(
            Image,
            confidence_topic,
            10
        )
        
        # Create overlay publisher if enabled
        self.overlay_publisher = None
        if publish_overlay:
            self.overlay_publisher = self.create_publisher(
                Image,
                overlay_topic,
                10
            )
        
        # Create LabelInfo publisher with transient local QoS
        label_info_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.label_info_publisher = self.create_publisher(
            LabelInfo,
            label_info_topic,
            label_info_qos
        )
        
        # Create and publish LabelInfo message
        self.publish_label_info()
        
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing mask to: {mask_topic}')
        self.get_logger().info(f'Publishing confidence to: {confidence_topic}')
        self.get_logger().info(f'Publishing label info to: {label_info_topic}')
        if publish_overlay:
            self.get_logger().info(f'Publishing overlay to: {overlay_topic}')
    
    def publish_label_info(self):
        """Publish LabelInfo message with class mappings."""
        label_info = LabelInfo()
        label_info.header.stamp = self.get_clock().now().to_msg()
        label_info.header.frame_id = ''  # Not tied to a specific frame
        
        # Build class map: background is class 0, then classes from config
        class_map = []
        
        # Background class (class 0)
        bg_class = VisionClass()
        bg_class.class_id = 0
        bg_class.class_name = 'background'
        class_map.append(bg_class)
        
        # Add classes from ontology
        for idx, class_name in enumerate(self.class_names, start=1):
            vc = VisionClass()
            vc.class_id = idx
            vc.class_name = class_name
            class_map.append(vc)
        
        label_info.class_map = class_map
        label_info.threshold = 0.5  # Default confidence threshold
        
        self.label_info_publisher.publish(label_info)
        self.get_logger().info(f'Published LabelInfo with {len(class_map)} classes')
    
    def create_colored_mask(self, mask: np.ndarray) -> np.ndarray:
        """
        Convert class ID mask to colored visualization.
        
        Args:
            mask: Single-channel mask with class IDs [H, W]
            
        Returns:
            Colored mask in BGR format [H, W, 3]
        """
        h, w = mask.shape
        colored = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Background stays black
        for class_id in range(1, self.num_classes):
            if class_id <= len(self.class_colors):
                color = self.class_colors[class_id - 1]
                colored[mask == class_id] = color
        
        return colored
    
    def image_callback(self, msg):
        """Process incoming image and publish segmentation results."""
        # Convert ROS image to OpenCV format (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Preprocess image
        # Convert to the model's expected dtype (FP16 or FP32)
        dtype = np.float16 if self.use_fp16 else np.float32
        input_tensor = rgb_image.transpose(2, 0, 1).astype(dtype) / 255.0
        # Apply ImageNet normalization
        input_tensor = (input_tensor - self.mean.squeeze(0)) / self.std.squeeze(0)
        # Add batch dimension
        input_tensor = np.expand_dims(input_tensor, axis=0)
        
        # Run ONNX inference
        outputs = self.session.run(None, {'input': input_tensor})
        output = outputs[0]  # Shape: [1, num_classes, H, W]
        
        # Get prediction (class IDs)
        prediction = np.argmax(output, axis=1).squeeze(0).astype(np.uint8)
        
        # Get confidence (max probability per pixel)
        # Apply softmax manually
        exp_output = np.exp(output - np.max(output, axis=1, keepdims=True))
        probabilities = exp_output / np.sum(exp_output, axis=1, keepdims=True)
        confidence = np.max(probabilities, axis=1).squeeze(0)
        confidence_uint8 = (confidence * 255.0).astype(np.uint8)
        
        # Create mask image message
        mask_msg = self.bridge.cv2_to_imgmsg(prediction, encoding='mono8')
        mask_msg.header = msg.header
        
        # Create confidence image message
        confidence_msg = self.bridge.cv2_to_imgmsg(confidence_uint8, encoding='mono8')
        confidence_msg.header = msg.header
        
        # Publish mask and confidence
        self.mask_publisher.publish(mask_msg)
        self.confidence_publisher.publish(confidence_msg)
        
        # Create and publish overlay if enabled
        if self.overlay_publisher is not None:
            pred_colored = self.create_colored_mask(prediction)
            overlay = cv2.addWeighted(cv_image, 0.7, pred_colored, 0.3, 0)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            overlay_msg.header = msg.header
            self.overlay_publisher.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

