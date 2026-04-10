#!/usr/bin/env python3
"""Subscribe to /odometry and save poses in TUM format.
Usage: python3 save_odometry_tum.py <output_file> [topic]
"""
import sys
import os
import signal
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSaver(Node):
    def __init__(self, output_file, topic='/odometry'):
        super().__init__('odom_saver')
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        self.fout = open(output_file, 'w')
        self.count = 0
        self.sub = self.create_subscription(Odometry, topic, self.cb, 500)
        self.get_logger().info(f'Saving {topic} → {output_file}')

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.fout.write(f'{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} '
                        f'{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n')
        self.count += 1
        if self.count % 500 == 0:
            self.fout.flush()
            self.get_logger().info(f'Saved {self.count} poses...')

    def close(self):
        self.fout.flush()
        self.fout.close()
        self.get_logger().info(f'Done. Total poses saved: {self.count}')

def main():
    output = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/results/resple_poses.txt')
    topic  = sys.argv[2] if len(sys.argv) > 2 else '/odometry'
    rclpy.init()
    node = OdomSaver(output, topic)
    def shutdown(sig, frame):
        node.close()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
