#!/usr/bin/env python3
"""
将里程计消息转换为TF变换
从 /mavros/local_position/odom 读取，发布 map -> base_link TF
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # 参数
        self.declare_parameter('odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('publish_tf', True)

        odom_topic = self.get_parameter('odom_topic').value
        publish_tf = self.get_parameter('publish_tf').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # 订阅里程计
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10)

        self.publish_tf = publish_tf
        self.get_logger().info(f'Odom to TF节点启动')
        self.get_logger().info(f'  订阅话题: {odom_topic}')
        self.get_logger().info(f'  发布TF: {publish_tf}')

    def odom_callback(self, msg):
        if not self.publish_tf:
            return

        # 创建TF变换
        t = TransformStamped()

        # 时间戳
        t.header.stamp = self.get_clock().now().to_msg()

        # 坐标系
        t.header.frame_id = msg.header.frame_id  # 通常是 'map'
        t.child_frame_id = msg.child_frame_id     # 通常是 'base_link'

        # 位置
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # 姿态（四元数）
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # 发布TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
