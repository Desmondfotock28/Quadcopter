#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PoseRepublisher(Node):
    def __init__(self):
        super().__init__('pose_republisher')
        
        # Subscribe to odometry
        self.sub = self.create_subscription(
            Odometry,
            '/mavros/global_position/local',
            self.odom_callback,
            10)
        
        # Publish as PoseStamped
        self.pub = self.create_publisher(
            PoseStamped,
            '/drone_pose',
            10)
        
        self.get_logger().info('Republishing odometry as pose...')
    
    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pub.publish(pose)
        # Log every 10th message to avoid spam
        if not hasattr(self, 'count'):
            self.count = 0
        self.count += 1
        if self.count % 10 == 0:
            self.get_logger().info(f'Published pose #{self.count} at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f})')

def main(args=None):
    rclpy.init(args=args)
    node = PoseRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()