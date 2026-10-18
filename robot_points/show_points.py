#!/usr/bin/env python3
"""Hien thi cac diem trong points.yaml len RViz duoi dang Marker.

    python3 show_points.py
    python3 show_points.py --points /duong/dan/points.yaml

Trong RViz: Add -> MarkerArray -> topic /task_points
Fixed Frame phai la 'map'.

Moi diem ve 3 thu:
  - hinh tru mau  : vi tri
  - mui ten       : huong (yaw)
  - chu           : ten diem, noi ngay tren dau

Marker dung QoS transient_local nen RViz mo sau van nhan duoc.
"""

import argparse
import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_POINTS = os.path.join(HERE, "points.yaml")

# mau cho tung diem, RGBA 0..1
COLORS = {
    "start": (0.2, 0.8, 0.2, 0.9),   # xanh la
    "end":   (0.9, 0.2, 0.2, 0.9),   # do
    "A":     (0.2, 0.4, 0.9, 0.9),   # xanh duong
    "B":     (0.9, 0.6, 0.1, 0.9),   # cam
    "C":     (0.7, 0.3, 0.9, 0.9),   # tim
}
DEFAULT_COLOR = (0.5, 0.5, 0.5, 0.9)


class PointMarkers(Node):
    def __init__(self, points):
        super().__init__("task_point_markers")

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, "/task_points", qos)
        self.points = points

        self.publish_markers()
        # phat lai moi 2s de RViz khoi dong muon van thay
        self.create_timer(2.0, self.publish_markers)

    def base(self, mid, mtype, name):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = name
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish_markers(self):
        arr = MarkerArray()
        mid = 0

        for name, spec in self.points.items():
            x = float(spec["x"])
            y = float(spec["y"])
            yaw = float(spec.get("yaw", 0.0))
            r, g, b, a = COLORS.get(name, DEFAULT_COLOR)

            # --- hinh tru danh dau vi tri ---
            cyl = self.base(mid, Marker.CYLINDER, name)
            mid += 1
            cyl.pose.position.x = x
            cyl.pose.position.y = y
            cyl.pose.position.z = 0.02
            cyl.scale.x = 0.30
            cyl.scale.y = 0.30
            cyl.scale.z = 0.04
            cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = r, g, b, a
            arr.markers.append(cyl)

            # --- mui ten chi huong ---
            arw = self.base(mid, Marker.ARROW, name)
            mid += 1
            arw.pose.position.x = x
            arw.pose.position.y = y
            arw.pose.position.z = 0.05
            arw.pose.orientation.z = math.sin(yaw / 2.0)
            arw.pose.orientation.w = math.cos(yaw / 2.0)
            arw.scale.x = 0.45      # chieu dai
            arw.scale.y = 0.06      # be ngang than
            arw.scale.z = 0.06      # be ngang dau
            arw.color.r, arw.color.g, arw.color.b, arw.color.a = r, g, b, 1.0
            arr.markers.append(arw)

            # --- chu ten diem ---
            txt = self.base(mid, Marker.TEXT_VIEW_FACING, name)
            mid += 1
            txt.pose.position.x = x
            txt.pose.position.y = y
            txt.pose.position.z = 0.45
            txt.scale.z = 0.28      # co chu
            txt.color.r = txt.color.g = txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = name
            arr.markers.append(txt)

        self.pub.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", default=DEFAULT_POINTS)
    args = ap.parse_args()

    with open(args.points) as f:
        data = yaml.safe_load(f)
    points = data["points"]

    rclpy.init()
    node = PointMarkers(points)
    node.get_logger().info(
        f"Dang phat {len(points)} diem len /task_points. "
        f"Trong RViz: Add -> MarkerArray -> topic /task_points"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()