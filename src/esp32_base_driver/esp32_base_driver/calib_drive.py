#!/usr/bin/env python3
"""GIAI DOAN 4 - hieu chuan odom. Tu lai chinh xac hon teleop bang tay.

    ros2 run esp32_base_driver calib_drive straight --dist 2.0
    ros2 run esp32_base_driver calib_drive spin --turns 10

Xe tu chay den dung moc odom roi dung. Sau do ban do gia tri THAT bang thuoc
(hoac dem vong) va nhap vao, node tinh ra tham so moi.

LAM TINH TIEN TRUOC, QUAY SAU. Phep quay dung lai ket qua cua phep tinh tien.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CalibDrive(Node):
    RATE = 20.0

    def __init__(self, mode, target, speed, slow_frac, cur_diam, cur_sep):
        super().__init__("calib_drive")
        self.mode = mode
        self.target = target          # m (straight) hoac rad (spin)
        self.speed = speed
        self.slow_frac = slow_frac
        self.cur_diam = cur_diam
        self.cur_sep = cur_sep

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

        self.x = self.y = 0.0
        self.th_unwrap = 0.0
        self.th_prev = None
        self.origin = None
        self.progress = 0.0
        self.done = False
        self.settle = 0
        self.have = False

        self.create_timer(1.0 / self.RATE, self.tick)

    def on_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        th = yaw_of(msg)
        if self.th_prev is not None:
            d = th - self.th_prev
            self.th_unwrap += math.atan2(math.sin(d), math.cos(d))
        self.th_prev = th
        self.have = True

    def tick(self) -> None:
        if not self.have or self.done:
            return
        if self.origin is None:
            self.origin = (self.x, self.y, self.th_unwrap)
            unit = "m" if self.mode == "straight" else "vong"
            amt = self.target if self.mode == "straight" else self.target / (2 * math.pi)
            self.get_logger().info(f"Bat dau: muc tieu {amt:.3f} {unit}")

        x0, y0, th0 = self.origin
        if self.mode == "straight":
            self.progress = math.hypot(self.x - x0, self.y - y0)
        else:
            self.progress = abs(self.th_unwrap - th0)

        remain = self.target - self.progress
        msg = Twist()

        if remain <= 0.0:
            self.pub.publish(msg)            # dung han
            self.settle += 1
            if self.settle > int(self.RATE * 1.5):   # cho banh dung han roi chot
                self.done = True
            return

        # giam toc o doan cuoi de banh khong truot khi phanh gap
        slow_zone = self.target * self.slow_frac
        v = self.speed
        if remain < slow_zone:
            v = max(self.speed * 0.25, self.speed * remain / slow_zone)

        if self.mode == "straight":
            msg.linear.x = v
            msg.angular.z = 0.0              # ep w = 0 tuyet doi
        else:
            msg.linear.x = 0.0               # ep v = 0 tuyet doi
            msg.angular.z = v
        self.pub.publish(msg)

    def result(self):
        x0, y0, th0 = self.origin
        return (math.hypot(self.x - x0, self.y - y0),
                math.degrees(self.th_unwrap - th0))


def ask_float(prompt):
    while True:
        s = input(prompt).strip().replace(",", ".")
        try:
            return float(s)
        except ValueError:
            print("  Nhap so, vi du 1.985")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["straight", "spin"])
    ap.add_argument("--dist", type=float, default=2.0, help="quang duong (m)")
    ap.add_argument("--turns", type=float, default=10.0, help="so vong")
    ap.add_argument("--speed", type=float, default=None,
                    help="m/s cho straight, rad/s cho spin")
    ap.add_argument("--slow-frac", type=float, default=0.15,
                    help="ty le doan cuoi de giam toc")
    ap.add_argument("--wheel-diameter", type=float, default=0.095)
    ap.add_argument("--wheel-separation", type=float, default=0.3890)
    args = ap.parse_args(sys.argv[1:])

    if args.mode == "straight":
        target = args.dist
        speed = args.speed if args.speed is not None else 0.15
    else:
        target = args.turns * 2 * math.pi
        speed = args.speed if args.speed is not None else 0.6

    rclpy.init()
    node = CalibDrive(args.mode, target, speed, args.slow_frac,
                      args.wheel_diameter, args.wheel_separation)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        dist, dyaw = node.result()
        node.pub.publish(Twist())
    except KeyboardInterrupt:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print("\nDung giua chung, khong co ket qua.")
        return

    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

    print("\n" + "=" * 60)
    if args.mode == "straight":
        print(f"ODOM BAO   : {dist:.4f} m")
        print(f"Lech huong : yaw doi {dyaw:+.2f} deg trong luc di thang")
        if abs(dyaw) > 2.0:
            print("  !! Lech > 2 deg -> xe di cong, KHONG dung lan do nay.")
            print("     Nguyen nhan: 2 banh khong bam cung toc do (deadband/PID).")
        print("=" * 60)
        real = ask_float("\nDo bang thuoc, quang duong THAT (m) = ")
        new = args.wheel_diameter * real / dist
        print(f"\n  wheel_diameter hien tai : {args.wheel_diameter:.5f} m")
        print(f"  wheel_diameter MOI      : {new:.5f} m  ({new * 1000:.2f} mm)")
        print(f"  thay doi                : {(new / args.wheel_diameter - 1) * 100:+.2f} %")
        print(f"\n  Sua trong config/base_params.yaml:  wheel_diameter: {new:.5f}")
        print("  Lam lai 3 lan roi lay trung binh se chac hon.")
    else:
        turns_rep = dyaw / 360.0
        print(f"ODOM BAO   : {dyaw:+.2f} deg = {turns_rep:+.4f} vong")
        print(f"Troi vi tri: {dist * 1000:.1f} mm (ly tuong gan 0)")
        if dist > 0.05:
            print("  !! Troi > 50mm -> tam quay lech nhieu, kiem tra banh co truot khong.")
        print("=" * 60)
        real_t = ask_float("\nDem mat, so vong THAT da quay = ")
        real = real_t * 360.0
        new = args.wheel_separation * dyaw / real
        print(f"\n  wheel_separation hien tai : {args.wheel_separation:.5f} m")
        print(f"  wheel_separation MOI      : {new:.5f} m  ({new * 1000:.1f} mm)")
        print(f"  thay doi                  : {(new / args.wheel_separation - 1) * 100:+.2f} %")
        print(f"\n  Sua trong config/base_params.yaml:  wheel_separation: {new:.5f}")
        print("  Nho da hieu chuan wheel_diameter TRUOC khi lam phep nay.")
    print()


if __name__ == "__main__":
    main()