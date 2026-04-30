import csv
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from ament_index_python.packages import get_package_share_directory


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')

        self.publisher_ = self.create_publisher(PoseArray, 'global_planner', 10)

        csv_path = os.path.join(
            get_package_share_directory('sig_stack'),
            'maps',
            'Spielberg_centerline.csv'
        )
        self.poses = self._load_centerline(csv_path)
        self.timer_ = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):
        self._publish()
        self.timer_.cancel()

    def _load_centerline(self, path):
        points = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # skip header
            for row in reader:
                x, y = float(row[0].strip()), float(row[1].strip())
                points.append((x, y))

        poses = []
        n = len(points)
        for i, (x, y) in enumerate(points):
            nx, ny = points[(i + 1) % n]
            yaw = math.atan2(ny - y, nx - x)

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = math.cos(yaw / 2)
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(yaw / 2)
            poses.append(pose)

        self.get_logger().info(f'Loaded {len(poses)} waypoints from centerline')
        return poses

    def _publish(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.poses = self.poses
        self.publisher_.publish(msg)
        self.get_logger().info('Published global path')


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
