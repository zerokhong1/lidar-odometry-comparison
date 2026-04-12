#!/usr/bin/env python3
"""Record nav_msgs/Odometry to TUM format file."""
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class PoseRecorder(Node):
    def __init__(self, topic, output_file):
        super().__init__('pose_recorder')
        self.f = open(output_file, 'w')
        self.count = 0
        self.sub = self.create_subscription(
            Odometry, topic, self.callback, 10)
        self.get_logger().info(f'Recording {topic} -> {output_file}')

    def callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} '
                     f'{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        self.f.flush()
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Recorded {self.count} poses')

    def destroy_node(self):
        self.f.close()
        super().destroy_node()


def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <topic> <output_file>')
        sys.exit(1)
    rclpy.init()
    node = PoseRecorder(sys.argv[1], sys.argv[2])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
