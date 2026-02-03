#!/usr/bin/env python3
"""
点云坐标系转换节点
将点云从传感器坐标系转换到目标坐标系
适用于 plan_env 包
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformException, Buffer, TransformListener
from rclpy.duration import Duration
import numpy as np

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pointcloud_transformer')

        # 参数
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_map')
        self.declare_parameter('timeout', 0.1)

        self.target_frame = self.get_parameter('target_frame').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        timeout = self.get_parameter('timeout').value

        # TF缓冲区
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 订阅和发布
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10)

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            10)

        self.timeout_duration = Duration(seconds=timeout)
        self.transform_count = 0
        self.error_count = 0

        self.get_logger().info(f'点云转换节点启动')
        self.get_logger().info(f'  输入话题: {input_topic}')
        self.get_logger().info(f'  输出话题: {output_topic}')
        self.get_logger().info(f'  目标坐标系: {self.target_frame}')

    def pointcloud_callback(self, msg):
        try:
            # 如果已经在目标坐标系，直接发布
            if msg.header.frame_id == self.target_frame:
                self.publisher.publish(msg)
                return

            # 查找TF变换
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=self.timeout_duration)

            # 提取变换矩阵
            t = transform.transform.translation
            r = transform.transform.rotation

            # 四元数转旋转矩阵
            qx, qy, qz, qw = r.x, r.y, r.z, r.w
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            T = np.array([t.x, t.y, t.z])

            # 读取并变换点云
            points = []
            for point in pc2.read_points(msg, skip_nans=True):
                # 变换点坐标
                p_in = np.array([point[0], point[1], point[2]])
                p_out = R @ p_in + T

                # 保留其他字段（如intensity, ring等）
                if len(point) > 3:
                    points.append([p_out[0], p_out[1], p_out[2]] + list(point[3:]))
                else:
                    points.append([p_out[0], p_out[1], p_out[2]])

            # 创建输出点云
            if points:
                output_msg = pc2.create_cloud(msg.header, msg.fields, points)
                output_msg.header.frame_id = self.target_frame
                output_msg.header.stamp = msg.header.stamp

                self.publisher.publish(output_msg)

                self.transform_count += 1
                if self.transform_count % 100 == 0:
                    self.get_logger().info(
                        f'已转换 {self.transform_count} 帧点云 '
                        f'({msg.header.frame_id} -> {self.target_frame})')

        except TransformException as ex:
            self.error_count += 1
            if self.error_count % 10 == 0:  # 每10次错误才报警一次
                self.get_logger().warn(
                    f'TF变换失败 (第{self.error_count}次): {ex}')

        except Exception as e:
            self.get_logger().error(f'处理点云时出错: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
