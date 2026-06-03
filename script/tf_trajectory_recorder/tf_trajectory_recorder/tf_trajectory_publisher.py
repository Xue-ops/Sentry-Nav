import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('tf_trajectory_publisher')

        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'odin1_base_link')
        self.declare_parameter('path_topic', '/tf_trajectory')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('min_distance', 0.02)
        self.declare_parameter('max_poses', 5000)
        self.declare_parameter('tf_timeout', 0.1)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.path_topic = self.get_parameter('path_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.min_distance = float(self.get_parameter('min_distance').value)
        self.max_poses = int(self.get_parameter('max_poses').value)
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)

        if self.publish_rate <= 0.0:
            self.get_logger().warn('publish_rate must be positive, using 10.0 Hz')
            self.publish_rate = 10.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)

        self.path = Path()
        self.path.header.frame_id = self.parent_frame
        self.last_pose = None
        self.last_tf_warn_time = self.get_clock().now()
        self.last_record_log_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'Publishing trajectory {self.parent_frame} -> {self.child_frame} '
            f'as {self.path_topic}'
        )

    def timer_callback(self):
        self.publish_path()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as ex:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 2_000_000_000:
                self.get_logger().warn(
                    f'Cannot record trajectory: no TF from {self.parent_frame} '
                    f'to {self.child_frame}: {ex}'
                )
                self.last_tf_warn_time = now
            return

        pose = PoseStamped()
        pose.header.stamp = transform.header.stamp
        pose.header.frame_id = self.parent_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation

        if self.should_append_pose(pose):
            self.path.poses.append(pose)
            if self.max_poses > 0 and len(self.path.poses) > self.max_poses:
                self.path.poses = self.path.poses[-self.max_poses:]
            self.last_pose = pose
            self.log_recorded_pose()

        self.publish_path()

    def should_append_pose(self, pose):
        if self.last_pose is None:
            return True

        dx = pose.pose.position.x - self.last_pose.pose.position.x
        dy = pose.pose.position.y - self.last_pose.pose.position.y
        dz = pose.pose.position.z - self.last_pose.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz) >= self.min_distance

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = self.parent_frame
        self.path_pub.publish(self.path)

    def log_recorded_pose(self):
        now = self.get_clock().now()
        if (now - self.last_record_log_time).nanoseconds <= 2_000_000_000:
            return

        pose = self.path.poses[-1].pose.position
        self.get_logger().info(
            f'Recorded {len(self.path.poses)} poses, latest '
            f'x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}'
        )
        self.last_record_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = TfTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
