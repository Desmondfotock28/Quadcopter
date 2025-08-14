#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        # Create static transforms for MAVROS frame chain
        now = self.get_clock().now().to_msg()
        transforms = []
        
        # Transform 1: map -> odom 
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        transforms.append(map_to_odom)
        
        # Transform 2: odom -> base_link (MAVROS publishes pose in odom frame)
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = now
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = 0.0
        odom_to_base.transform.translation.y = 0.0
        odom_to_base.transform.translation.z = 0.0
        odom_to_base.transform.rotation.x = 0.0
        odom_to_base.transform.rotation.y = 0.0
        odom_to_base.transform.rotation.z = 0.0
        odom_to_base.transform.rotation.w = 1.0
        transforms.append(odom_to_base)
        
        self.tf_static_broadcaster.sendTransform(transforms)
        self.get_logger().info('Publishing static transforms: map -> odom -> base_link')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()