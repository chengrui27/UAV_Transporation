#!/usr/bin/env python3

import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GazeboVisionBridge(Node):
    def __init__(self):
        super().__init__('gazebo_vision_bridge')

        self.declare_parameter('pose_source', 'gz')  # gz or ros
        self.declare_parameter('model_states_topic', '/model_states')
        self.declare_parameter('gz_pose_topic', '')
        self.declare_parameter('gz_world_name', '')
        self.declare_parameter('mavros_ns', '')
        self.declare_parameter(
            'vehicle_mappings',
            'iris_0:uav1,iris_1:uav2,iris_2:uav3',
        )
        self.declare_parameter('publish_vision_pose', True)
        self.declare_parameter('publish_odometry', False)
        self.declare_parameter('world_frame_id', 'world')
        self.declare_parameter('child_frame_id', 'base_link')

        self._pose_source = self.get_parameter('pose_source').value
        self._model_states_topic = self.get_parameter('model_states_topic').value
        self._gz_pose_topic = self.get_parameter('gz_pose_topic').value
        self._gz_world_name = self.get_parameter('gz_world_name').value
        self._mavros_ns = self.get_parameter('mavros_ns').value
        self._publish_vision_pose = self.get_parameter('publish_vision_pose').value
        self._publish_odometry = self.get_parameter('publish_odometry').value
        self._world_frame_id = self.get_parameter('world_frame_id').value
        self._child_frame_id_template = self.get_parameter('child_frame_id').value

        self._model_states_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        raw_mappings = self.get_parameter('vehicle_mappings').value
        self._vehicle_mappings = self._parse_mappings(raw_mappings)
        self._warned_missing = set()

        self._pose_pubs = {}
        self._odom_pubs = {}
        self._gz_proc = None
        self._gz_thread = None

        if not self._vehicle_mappings:
            self.get_logger().error(
                'No valid vehicle_mappings provided; nothing will be published.'
            )
        else:
            for model_name, namespace in self._vehicle_mappings.items():
                ns = namespace.strip('/')
                pose_topic = self._build_topic(ns, 'vision_pose/pose')
                odom_topic = self._build_topic(ns, 'odometry/in')

                if self._publish_vision_pose:
                    self._pose_pubs[model_name] = self.create_publisher(
                        PoseStamped,
                        pose_topic,
                        10,
                    )
                if self._publish_odometry:
                    self._odom_pubs[model_name] = self.create_publisher(
                        Odometry,
                        odom_topic,
                        10,
                    )

                self.get_logger().info(
                    f'Map model "{model_name}" -> namespace "{ns}" '
                    f'(vision_pose={self._publish_vision_pose}, '
                    f'odometry={self._publish_odometry})'
                )

        if self._pose_source == 'ros':
            self.create_subscription(
                ModelStates,
                self._model_states_topic,
                self._model_states_cb,
                self._model_states_qos,
            )
            self.get_logger().info(
                f'Subscribed to {self._model_states_topic} (best-effort QoS)'
            )
        elif self._pose_source == 'gz':
            self._start_gz_pose_reader()
        else:
            self.get_logger().error(
                f'Unknown pose_source "{self._pose_source}". Use "gz" or "ros".'
            )

    def _parse_mappings(self, raw_value):
        items = []
        if isinstance(raw_value, list):
            items = raw_value
        elif isinstance(raw_value, str):
            items = [item for item in raw_value.split(',') if item.strip()]

        mappings = {}
        for item in items:
            if ':' not in item:
                self.get_logger().warn(
                    f'Invalid mapping "{item}". Use "model:namespace".'
                )
                continue
            model_name, namespace = item.split(':', 1)
            model_name = model_name.strip()
            namespace = namespace.strip()
            if not model_name or not namespace:
                self.get_logger().warn(
                    f'Invalid mapping "{item}". Use "model:namespace".'
                )
                continue
            mappings[model_name] = namespace
        return mappings

    def _build_topic(self, namespace, suffix):
        ns = namespace.strip('/')
        mavros_ns = str(self._mavros_ns).strip('/')
        parts = []
        if ns:
            parts.append(ns)
        if mavros_ns:
            parts.append(mavros_ns)
        parts.append(suffix.strip('/'))
        return '/' + '/'.join(parts)

    def _detect_gz_pose_topic(self):
        try:
            result = subprocess.run(
                ['gz', 'topic', '-l'],
                check=True,
                capture_output=True,
                text=True,
            )
        except (FileNotFoundError, subprocess.CalledProcessError) as exc:
            self.get_logger().error(f'Failed to run "gz topic -l": {exc}')
            return ''

        pose_topics = []
        for line in result.stdout.splitlines():
            line = line.strip()
            if line.endswith('/pose/info'):
                pose_topics.append(line)

        if not pose_topics:
            return ''
        return pose_topics[0]

    def _resolve_gz_pose_topic(self):
        if self._gz_pose_topic:
            return self._gz_pose_topic
        if self._gz_world_name:
            return f'/gazebo/{self._gz_world_name}/pose/info'
        return self._detect_gz_pose_topic()

    def _start_gz_pose_reader(self):
        self._gz_pose_topic = self._resolve_gz_pose_topic()
        if not self._gz_pose_topic:
            self.get_logger().error(
                'Unable to resolve Gazebo pose topic. Set gz_pose_topic or '
                'gz_world_name explicitly.'
            )
            return

        try:
            self._gz_proc = subprocess.Popen(
                ['gz', 'topic', '-e', self._gz_pose_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError as exc:
            self.get_logger().error(f'Failed to start gz: {exc}')
            return

        self._gz_thread = threading.Thread(
            target=self._gz_read_loop,
            name='gz_pose_reader',
            daemon=True,
        )
        self._gz_thread.start()
        self.get_logger().info(
            f'Reading Gazebo pose data from {self._gz_pose_topic}'
        )

    def _gz_read_loop(self):
        if self._gz_proc is None or self._gz_proc.stdout is None:
            return

        current_pose = None
        context = []

        for line in self._gz_proc.stdout:
            stripped = line.strip()
            if not stripped:
                continue

            if stripped == 'pose {':
                current_pose = {'position': {}, 'orientation': {}}
                context = ['pose']
                continue

            if stripped == 'position {':
                context.append('position')
                continue

            if stripped == 'orientation {':
                context.append('orientation')
                continue

            if stripped.startswith('name:') and current_pose is not None:
                _, value = stripped.split(':', 1)
                current_pose['name'] = value.strip().strip('"')
                continue

            if stripped.startswith(('x:', 'y:', 'z:', 'w:')) and context:
                key, value = stripped.split(':', 1)
                key = key.strip()
                try:
                    num = float(value.strip())
                except ValueError:
                    continue
                if current_pose is None:
                    continue
                if context[-1] == 'position':
                    current_pose['position'][key] = num
                elif context[-1] == 'orientation':
                    current_pose['orientation'][key] = num
                continue

            if stripped == '}':
                if not context:
                    continue
                last = context.pop()
                if last == 'pose' and current_pose:
                    self._handle_gz_pose(current_pose)
                    current_pose = None

        if self._gz_proc and self._gz_proc.poll() is not None:
            self.get_logger().warn('gz pose reader exited.')

    def _handle_gz_pose(self, pose_data):
        name = pose_data.get('name')
        if not name or name not in self._vehicle_mappings:
            return

        position = pose_data.get('position', {})
        orientation = pose_data.get('orientation', {})
        if not position or not orientation:
            return

        namespace = self._vehicle_mappings[name]
        stamp = self.get_clock().now().to_msg()

        if self._publish_vision_pose:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self._world_frame_id
            pose_msg.pose.position.x = position.get('x', 0.0)
            pose_msg.pose.position.y = position.get('y', 0.0)
            pose_msg.pose.position.z = position.get('z', 0.0)
            pose_msg.pose.orientation.x = orientation.get('x', 0.0)
            pose_msg.pose.orientation.y = orientation.get('y', 0.0)
            pose_msg.pose.orientation.z = orientation.get('z', 0.0)
            pose_msg.pose.orientation.w = orientation.get('w', 1.0)
            self._pose_pubs[name].publish(pose_msg)

        if self._publish_odometry:
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = self._world_frame_id
            odom_msg.child_frame_id = self._child_frame_id(namespace)
            odom_msg.pose.pose.position.x = position.get('x', 0.0)
            odom_msg.pose.pose.position.y = position.get('y', 0.0)
            odom_msg.pose.pose.position.z = position.get('z', 0.0)
            odom_msg.pose.pose.orientation.x = orientation.get('x', 0.0)
            odom_msg.pose.pose.orientation.y = orientation.get('y', 0.0)
            odom_msg.pose.pose.orientation.z = orientation.get('z', 0.0)
            odom_msg.pose.pose.orientation.w = orientation.get('w', 1.0)
            self._odom_pubs[name].publish(odom_msg)

    def _child_frame_id(self, namespace):
        try:
            return self._child_frame_id_template.format(ns=namespace)
        except Exception:
            return self._child_frame_id_template

    def _model_states_cb(self, msg):
        if not self._vehicle_mappings:
            return

        name_to_index = {name: idx for idx, name in enumerate(msg.name)}
        stamp = self.get_clock().now().to_msg()

        for model_name, namespace in self._vehicle_mappings.items():
            idx = name_to_index.get(model_name)
            if idx is None:
                if model_name not in self._warned_missing:
                    self.get_logger().warn(
                        f'Model "{model_name}" not found in '
                        f'{self._model_states_topic}'
                    )
                    self._warned_missing.add(model_name)
                continue

            pose = msg.pose[idx]

            if self._publish_vision_pose:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = stamp
                pose_msg.header.frame_id = self._world_frame_id
                pose_msg.pose = pose
                self._pose_pubs[model_name].publish(pose_msg)

            if self._publish_odometry:
                odom_msg = Odometry()
                odom_msg.header.stamp = stamp
                odom_msg.header.frame_id = self._world_frame_id
                odom_msg.child_frame_id = self._child_frame_id(namespace)
                odom_msg.pose.pose = pose
                if idx < len(msg.twist):
                    odom_msg.twist.twist = msg.twist[idx]
                self._odom_pubs[model_name].publish(odom_msg)

    def destroy_node(self):
        if self._gz_proc and self._gz_proc.poll() is None:
            self._gz_proc.terminate()
            try:
                self._gz_proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self._gz_proc.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboVisionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
