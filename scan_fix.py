#!/usr/bin/env python3
"""Va range_max cho RPLIDAR A2M8 firmware 1.22.

QoS: subscribe BEST_EFFORT (khop driver lidar),
     publish RELIABLE (tuong thich ca RELIABLE lan BEST_EFFORT subscriber).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan

SUB_QOS = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=5)
PUB_QOS = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                     history=QoSHistoryPolicy.KEEP_LAST, depth=10)

class ScanFix(Node):
    def __init__(self):
        super().__init__('scan_fix')
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.rmax = float(self.get_parameter('range_max').value)
        self.rmin = float(self.get_parameter('range_min').value)
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.n = 0
        self.pub = self.create_publisher(LaserScan, out, PUB_QOS)
        self.sub = self.create_subscription(LaserScan, inp, self.cb, SUB_QOS)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'scan_fix: {inp} -> {out} | range_max={self.rmax}')

    def cb(self, msg):
        msg.range_max = self.rmax
        msg.range_min = self.rmin
        self.pub.publish(msg)
        self.n += 1

    def report(self):
        self.get_logger().info(f'da chuyen tiep {self.n} scan')

def main():
    rclpy.init()
    try:
        rclpy.spin(ScanFix())
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
