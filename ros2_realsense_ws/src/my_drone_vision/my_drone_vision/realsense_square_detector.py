#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class RealsenseSquareDetector(Node):
    """Detect square targets from the D415 RealSense color stream."""

    def __init__(self) -> None:
        super().__init__('realsense_square_detector')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('annotated_topic', '/camera/squares/annotated')
        self.declare_parameter('boxes_topic', '/camera/squares/bounding_boxes')
        self.declare_parameter('min_area', 800.0)
        self.declare_parameter('max_area', 250000.0)
        self.declare_parameter('approx_epsilon_ratio', 0.04)
        self.declare_parameter('square_tolerance', 0.25)
        self.declare_parameter('show_window', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.boxes_topic = self.get_parameter('boxes_topic').value
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        self.approx_epsilon_ratio = float(self.get_parameter('approx_epsilon_ratio').value)
        self.square_tolerance = float(self.get_parameter('square_tolerance').value)
        self.show_window = bool(self.get_parameter('show_window').value)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, qos_profile_sensor_data)
        self.boxes_pub = self.create_publisher(Float32MultiArray, self.boxes_topic, 10)

        self.get_logger().info(f'Subscribed to {self.image_topic}')
        self.get_logger().info(f'Publishing annotated image on {self.annotated_topic}')
        self.get_logger().info(f'Publishing bounding boxes on {self.boxes_topic}')

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated, boxes = self.detect_squares(frame)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)
        self.publish_boxes(boxes)

        if self.show_window:
            cv2.imshow('RealSense square detector', annotated)
            cv2.waitKey(1)

    def detect_squares(self, frame: np.ndarray) -> tuple[np.ndarray, list[tuple[int, int, int, int]]]:
        annotated = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        edges = cv2.dilate(edges, None, iterations=1)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes: list[tuple[int, int, int, int]] = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, self.approx_epsilon_ratio * perimeter, True)
            if len(approx) != 4 or not cv2.isContourConvex(approx):
                continue

            x, y, w, h = cv2.boundingRect(approx)
            if h == 0:
                continue

            aspect_ratio = w / float(h)
            if abs(1.0 - aspect_ratio) > self.square_tolerance:
                continue

            boxes.append((x, y, w, h))
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                'square',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return annotated, boxes

    def publish_boxes(self, boxes: list[tuple[int, int, int, int]]) -> None:
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='boxes', size=len(boxes), stride=len(boxes) * 4),
            MultiArrayDimension(label='xywh', size=4, stride=4),
        ]
        msg.data = [float(value) for box in boxes for value in box]
        self.boxes_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseSquareDetector()

    try:
        rclpy.spin(node)
    finally:
        if node.show_window:
            cv2.destroyAllWindows()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
