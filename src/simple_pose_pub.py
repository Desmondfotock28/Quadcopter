#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import traceback

class SimplePosePublisher(Node):
    def __init__(self):
        super().__init__('simple_pose_publisher')
        
        # Create QoS profile matching MAVROS (BEST_EFFORT)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribe to odometry with matching QoS
        self.sub = self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.odom_callback,
            qos)
        
        # Publish PoseStamped with same QoS
        self.pub = self.create_publisher(
            PoseStamped,
            '/drone_pose',
            qos)
        
        # Create timer to publish test pose if no data received
        self.got_data = False
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Simple pose publisher started')
        self.count = 0
    
    def odom_callback(self, msg):
        try:
            self.got_data = True
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self.pub.publish(pose)
            
            self.count += 1
            if self.count % 30 == 0:  # Log every 30th message (~1Hz at 30Hz)
                self.get_logger().info(
                    f'Publishing drone at x={pose.pose.position.x:.2f}, '
                    f'y={pose.pose.position.y:.2f}, z={pose.pose.position.z:.2f}')
        except Exception as e:
            self.get_logger().error(f'Error in callback: {e}')
            traceback.print_exc()
    
    def timer_callback(self):
        if not self.got_data:
            # Don't publish a default pose - we want to know if real data is flowing
            self.get_logger().warn('No MAVROS data received yet')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimplePosePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()