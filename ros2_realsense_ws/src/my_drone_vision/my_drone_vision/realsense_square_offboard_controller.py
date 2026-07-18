#!/usr/bin/env python3

import math
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


class RealsenseSquareOffboardController(Node):
    """PX4 offboard controller driven by square detections from the D415 image."""

    def __init__(self) -> None:
        super().__init__('realsense_square_offboard_controller')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('annotated_topic', '/camera/squares/annotated')
        self.declare_parameter('boxes_topic', '/camera/squares/bounding_boxes')
        self.declare_parameter('vehicle_local_position_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        self.declare_parameter('vehicle_command_ack_topic', '/fmu/out/vehicle_command_ack')
        self.declare_parameter('takeoff_height', -1.5)
        self.declare_parameter('forward_velocity', 0.0)
        self.declare_parameter('lateral_gain', 0.3)
        self.declare_parameter('vertical_gain', 0.25)
        self.declare_parameter('max_lateral_velocity', 0.08)
        self.declare_parameter('max_vertical_velocity', 0.2)
        self.declare_parameter('full_target_velocity_scale', 0.2)
        self.declare_parameter('lateral_axis_sign', 1.0)
        self.declare_parameter('lateral_local_axis', 'y')
        self.declare_parameter('center_deadband_px', 45.0)
        self.declare_parameter('min_area', 800.0)
        self.declare_parameter('max_area', 250000.0)
        self.declare_parameter('approx_epsilon_ratio', 0.04)
        self.declare_parameter('rectangle_min_aspect_ratio', 0.15)
        self.declare_parameter('rectangle_max_aspect_ratio', 6.0)
        self.declare_parameter('target_border_margin_px', 8)
        self.declare_parameter('position_limit_enabled', True)
        self.declare_parameter('position_limit_margin_m', 0.5)
        self.declare_parameter('position_x_min_m', -0.35)
        self.declare_parameter('position_x_max_m', 2.95)
        self.declare_parameter('position_y_min_m', -6.70)
        self.declare_parameter('position_y_max_m', -0.10)
        self.declare_parameter('show_window', True)
        self.declare_parameter('display_scale', 0.5)
        self.declare_parameter('depth_display_stride', 5)
        self.declare_parameter('depth_percentile_stride', 4)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('offboard_start_setpoints', 20)
        self.declare_parameter('command_retry_interval_sec', 1.0)
        self.declare_parameter('release_on_external_mode_change', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.annotated_topic = self.get_parameter('annotated_topic').value
        self.boxes_topic = self.get_parameter('boxes_topic').value
        self.vehicle_local_position_topic = self.get_parameter('vehicle_local_position_topic').value
        self.vehicle_status_topic = self.get_parameter('vehicle_status_topic').value
        self.vehicle_command_ack_topic = self.get_parameter('vehicle_command_ack_topic').value
        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.forward_velocity = float(self.get_parameter('forward_velocity').value)
        self.lateral_gain = float(self.get_parameter('lateral_gain').value)
        self.vertical_gain = float(self.get_parameter('vertical_gain').value)
        self.max_lateral_velocity = float(self.get_parameter('max_lateral_velocity').value)
        self.max_vertical_velocity = float(self.get_parameter('max_vertical_velocity').value)
        self.full_target_velocity_scale = float(self.get_parameter('full_target_velocity_scale').value)
        self.lateral_axis_sign = float(self.get_parameter('lateral_axis_sign').value)
        self.lateral_local_axis = str(self.get_parameter('lateral_local_axis').value).lower()
        self.center_deadband_px = float(self.get_parameter('center_deadband_px').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.max_area = float(self.get_parameter('max_area').value)
        self.approx_epsilon_ratio = float(self.get_parameter('approx_epsilon_ratio').value)
        self.rectangle_min_aspect_ratio = float(self.get_parameter('rectangle_min_aspect_ratio').value)
        self.rectangle_max_aspect_ratio = float(self.get_parameter('rectangle_max_aspect_ratio').value)
        self.target_border_margin_px = int(self.get_parameter('target_border_margin_px').value)
        self.position_limit_enabled = bool(self.get_parameter('position_limit_enabled').value)
        self.position_limit_margin_m = float(self.get_parameter('position_limit_margin_m').value)
        self.position_x_min_m = float(self.get_parameter('position_x_min_m').value)
        self.position_x_max_m = float(self.get_parameter('position_x_max_m').value)
        self.position_y_min_m = float(self.get_parameter('position_y_min_m').value)
        self.position_y_max_m = float(self.get_parameter('position_y_max_m').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.display_scale = float(self.get_parameter('display_scale').value)
        self.depth_display_stride = max(1, int(self.get_parameter('depth_display_stride').value))
        self.depth_percentile_stride = max(1, int(self.get_parameter('depth_percentile_stride').value))
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self.offboard_start_setpoints = int(self.get_parameter('offboard_start_setpoints').value)
        self.command_retry_interval_ns = int(
            float(self.get_parameter('command_retry_interval_sec').value) * 1_000_000_000
        )
        self.release_on_external_mode_change = bool(
            self.get_parameter('release_on_external_mode_change').value
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.latest_box: Optional[tuple[int, int, int, int]] = None
        self.latest_box_fully_visible = False
        self.latest_image_size: Optional[tuple[int, int]] = None
        self.latest_depth_raw: Optional[np.ndarray] = None
        self.latest_depth_view: Optional[np.ndarray] = None
        self.display_frame_counter = 0
        self.have_local_position = False
        self.desired_x: Optional[float] = None
        self.desired_y: Optional[float] = None
        self.desired_z: Optional[float] = None
        self.offboard_setpoint_counter = 0
        self.last_command_time_ns = 0
        self.last_status_log_ns = 0
        self.has_entered_offboard = False
        self.offboard_released = False

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self.vehicle_local_position_callback,
            px4_qos,
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self.vehicle_status_callback,
            px4_qos,
        )
        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            self.vehicle_command_ack_topic,
            self.vehicle_command_ack_callback,
            px4_qos,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, qos_profile_sensor_data)
        self.boxes_pub = self.create_publisher(Float32MultiArray, self.boxes_topic, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(f'Subscribed to {self.image_topic}')
        self.get_logger().info(f'Subscribed to {self.depth_topic}')
        self.get_logger().info(f'Subscribed to {self.vehicle_local_position_topic}')
        self.get_logger().info(f'Subscribed to {self.vehicle_status_topic}')
        self.get_logger().info(f'Subscribed to {self.vehicle_command_ack_topic}')
        self.get_logger().info('Publishing PX4 offboard velocity setpoints')

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        annotated, boxes = self.detect_squares(frame)
        self.latest_image_size = (frame.shape[1], frame.shape[0])
        fully_visible_boxes = self._fully_visible_boxes(boxes, self.latest_image_size)
        target_boxes = fully_visible_boxes if fully_visible_boxes else boxes
        self.latest_box = max(target_boxes, key=lambda box: box[2] * box[3]) if target_boxes else None
        self.latest_box_fully_visible = self.latest_box in fully_visible_boxes if self.latest_box else False

        if self.latest_box is not None:
            x, y, w, h = self.latest_box
            cv2.circle(annotated, (x + w // 2, y + h // 2), 5, (0, 0, 255), -1)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (255, 0, 0), 3)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)
        self.publish_boxes(boxes)

        if self.show_window:
            cv2.imshow('RealSense square offboard controller', self._combined_view(annotated))
            cv2.waitKey(1)

    def depth_callback(self, msg: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.latest_depth_raw = depth

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg
        self.have_local_position = True
        self._initialize_position_hold()

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg
        is_offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if is_offboard:
            self.has_entered_offboard = True

        if (
            self.release_on_external_mode_change
            and self.has_entered_offboard
            and not is_offboard
            and not self.offboard_released
        ):
            self.offboard_released = True
            self.get_logger().info(
                f'PX4 left Offboard mode, releasing control. nav_state={msg.nav_state}'
            )

    def vehicle_command_ack_callback(self, msg: VehicleCommandAck) -> None:
        if msg.command not in (
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        ):
            return

        result_names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: 'ACCEPTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: 'TEMPORARILY_REJECTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: 'DENIED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: 'UNSUPPORTED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: 'FAILED',
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: 'IN_PROGRESS',
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: 'CANCELLED',
        }
        result = result_names.get(msg.result, str(msg.result))
        self.get_logger().info(
            f'PX4 command ack: command={msg.command} result={result} '
            f'result_param1={msg.result_param1} result_param2={msg.result_param2}'
        )

    def timer_callback(self) -> None:
        if self.offboard_released:
            now_ns = self.get_clock().now().nanoseconds

            if now_ns - self.last_status_log_ns >= 2_000_000_000:
                self.get_logger().info(
                    f'Offboard released; not publishing heartbeat or setpoints. '
                    f'arming_state={self.vehicle_status.arming_state}, '
                    f'nav_state={self.vehicle_status.nav_state}'
                )
                self.last_status_log_ns = now_ns

            return

        self.publish_offboard_control_mode()
        self.publish_velocity_setpoint()

        if self.offboard_setpoint_counter < self.offboard_start_setpoints:
            self.offboard_setpoint_counter += 1
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_command_time_ns >= self.command_retry_interval_ns:
            if (
                self.auto_offboard
                and self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ):
                self.engage_offboard_mode()

            if self.auto_arm and self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.arm()

            self.last_command_time_ns = now_ns

        if now_ns - self.last_status_log_ns >= 2_000_000_000:
            self.get_logger().info(
                f'PX4 state: arming_state={self.vehicle_status.arming_state}, '
                f'nav_state={self.vehicle_status.nav_state}'
            )
            self.last_status_log_ns = now_ns

    def publish_velocity_setpoint(self) -> None:
        self._initialize_position_hold()

        if self.desired_x is None or self.desired_y is None or self.desired_z is None:
            return

        if (
            self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and self.latest_box is not None
            and self.latest_image_size is not None
        ):
            image_width, image_height = self.latest_image_size
            x, y, w, h = self.latest_box
            error_x = (x + w * 0.5) - (image_width * 0.5)
            error_y = (y + h * 0.5) - (image_height * 0.5)

            if abs(error_x) > self.center_deadband_px:
                lateral_velocity = self._clamp(
                    self.lateral_axis_sign * self.lateral_gain * error_x / (image_width * 0.5),
                    -self.max_lateral_velocity,
                    self.max_lateral_velocity,
                )
                if self.latest_box_fully_visible:
                    lateral_velocity *= self.full_target_velocity_scale
                self._move_desired_lateral_position(lateral_velocity * 0.1)

            if abs(error_y) > self.center_deadband_px:
                vz = self._clamp(
                    self.vertical_gain * error_y / (image_height * 0.5),
                    -self.max_vertical_velocity,
                    self.max_vertical_velocity,
                )
                if self.latest_box_fully_visible:
                    vz *= self.full_target_velocity_scale
                self.desired_z += vz * 0.1

        msg = TrajectorySetpoint()
        msg.position = [float(self.desired_x), float(self.desired_y), float(self.desired_z)]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.yaw = math.nan
        msg.yawspeed = math.nan
        msg.timestamp = self.timestamp_us()
        self.trajectory_setpoint_pub.publish(msg)

    def _altitude_hold_velocity(self) -> float:
        if not self.have_local_position or math.isnan(self.vehicle_local_position.z):
            return 0.0

        error_z = self.takeoff_height - self.vehicle_local_position.z
        return self._clamp(error_z * 0.6, -self.max_vertical_velocity, self.max_vertical_velocity)

    def _initialize_position_hold(self) -> None:
        if not self.have_local_position:
            return

        if (
            math.isnan(self.vehicle_local_position.x)
            or math.isnan(self.vehicle_local_position.y)
            or math.isnan(self.vehicle_local_position.z)
        ):
            return

        if self.desired_x is None:
            self.desired_x = self.vehicle_local_position.x

        if self.desired_y is None:
            self.desired_y = self.vehicle_local_position.y

        if self.desired_z is None:
            self.desired_z = self.takeoff_height

        self._clamp_desired_position()

    def _move_desired_lateral_position(self, delta_m: float) -> None:
        if self.desired_x is None or self.desired_y is None:
            return

        if self.lateral_local_axis == 'x':
            self.desired_x += delta_m
        else:
            self.desired_y += delta_m

        self._clamp_desired_position()

    def _clamp_desired_position(self) -> None:
        if self.desired_x is not None:
            self.desired_x = self._clamp(
                self.desired_x,
                self.position_x_min_m,
                self.position_x_max_m,
            )

        if self.desired_y is not None:
            self.desired_y = self._clamp(
                self.desired_y,
                self.position_y_min_m,
                self.position_y_max_m,
            )

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.timestamp_us()
        self.offboard_control_mode_pub.publish(msg)

    def arm(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self) -> None:
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.vehicle_command_pub.publish(msg)

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
            if len(approx) not in (3, 4) or not cv2.isContourConvex(approx):
                continue

            x, y, w, h = cv2.boundingRect(approx)
            if h == 0:
                continue

            aspect_ratio = w / float(h)
            if not self.rectangle_min_aspect_ratio <= aspect_ratio <= self.rectangle_max_aspect_ratio:
                continue

            boxes.append((x, y, w, h))
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(
                annotated,
                'target',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return annotated, boxes

    def _fully_visible_boxes(
        self,
        boxes: list[tuple[int, int, int, int]],
        image_size: tuple[int, int],
    ) -> list[tuple[int, int, int, int]]:
        image_width, image_height = image_size
        margin = self.target_border_margin_px

        return [
            (x, y, w, h)
            for x, y, w, h in boxes
            if (
                x > margin
                and y > margin
                and x + w < image_width - margin
                and y + h < image_height - margin
            )
        ]

    def _combined_view(self, rgb_frame: np.ndarray) -> np.ndarray:
        self.display_frame_counter += 1
        if (
            self.latest_depth_raw is not None
            and (
                self.latest_depth_view is None
                or self.display_frame_counter % self.depth_display_stride == 0
            )
        ):
            self.latest_depth_view = self._colorize_depth(self.latest_depth_raw)

        if self.latest_depth_view is None:
            rgb_labeled = self._label_frame(rgb_frame, 'RGB')
            return self._resize_display(rgb_labeled)

        depth_frame = self.latest_depth_view
        if depth_frame.shape[:2] != rgb_frame.shape[:2]:
            depth_frame = cv2.resize(
                depth_frame,
                (rgb_frame.shape[1], rgb_frame.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

        rgb_labeled = self._label_frame(rgb_frame, 'RGB')
        depth_labeled = self._label_frame(depth_frame, 'Depth')
        return self._resize_display(np.hstack((rgb_labeled, depth_labeled)))

    def _resize_display(self, frame: np.ndarray) -> np.ndarray:
        if self.display_scale <= 0.0 or self.display_scale == 1.0:
            return frame

        return cv2.resize(
            frame,
            None,
            fx=self.display_scale,
            fy=self.display_scale,
            interpolation=cv2.INTER_AREA,
        )

    def _colorize_depth(self, depth: np.ndarray) -> np.ndarray:
        if depth.ndim == 3:
            depth = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)

        depth_float = depth.astype(np.float32)
        valid = np.isfinite(depth_float) & (depth_float > 0.0)

        if not np.any(valid):
            return np.zeros((*depth_float.shape[:2], 3), dtype=np.uint8)

        sampled_depth = depth_float[::self.depth_percentile_stride, ::self.depth_percentile_stride]
        sampled_valid = np.isfinite(sampled_depth) & (sampled_depth > 0.0)
        if not np.any(sampled_valid):
            sampled_depth = depth_float
            sampled_valid = valid

        lower = float(np.percentile(sampled_depth[sampled_valid], 2.0))
        upper = float(np.percentile(sampled_depth[sampled_valid], 98.0))
        if upper <= lower:
            upper = lower + 1.0

        normalized = np.zeros(depth_float.shape[:2], dtype=np.uint8)
        clipped = np.clip(depth_float, lower, upper)
        normalized[valid] = ((clipped[valid] - lower) * 255.0 / (upper - lower)).astype(np.uint8)

        return cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)

    def _label_frame(self, frame: np.ndarray, label: str) -> np.ndarray:
        labeled = frame.copy()
        cv2.rectangle(labeled, (8, 8), (110, 38), (0, 0, 0), -1)
        cv2.putText(
            labeled,
            label,
            (18, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return labeled

    def publish_boxes(self, boxes: list[tuple[int, int, int, int]]) -> None:
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='boxes', size=len(boxes), stride=len(boxes) * 4),
            MultiArrayDimension(label='xywh', size=4, stride=4),
        ]
        msg.data = [float(value) for box in boxes for value in box]
        self.boxes_pub.publish(msg)

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return max(lower, min(upper, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealsenseSquareOffboardController()

    try:
        rclpy.spin(node)
    finally:
        if node.show_window:
            cv2.destroyAllWindows()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
