#!/usr/bin/env python3
"""Theo doi /odom truc tiep de hieu chuan. Chay song song voi teleop.

    ros2 run esp32_base_driver odom_watch
    # hoac chay thang:
    python3 odom_watch.py

Gio phim:
    Enter  : dat lai moc 0 (bat dau mot lan do moi)
    d      : chot va in ket qua lan do
    q      : thoat

Cach dung cho hieu chuan TINH TIEN:
    1. Danh dau vi tri banh tren san, do truoc 2.000 m bang thuoc
    2. Bam Enter de dat moc
    3. Teleop cho xe chay toi vach dich, dung lai
    4. Bam d -> doc quang duong odom bao
    5. wheel_diameter_moi = wheel_diameter_cu * (do_that / odom_bao)

Cach dung cho hieu chuan QUAY (lam SAU khi xong tinh tien):
    1. Danh dau huong dau xe tren san
    2. Bam Enter, teleop quay tai cho dung 10 vong, ve dung huong cu
    3. Bam d -> doc yaw tich luy (ky vong 3600 deg)
    4. wheel_separation_moi = wheel_separation_cu * (yaw_bao / 3600)
"""
import math
import sys
import select
import termios
import tty

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomWatch(Node):
    def __init__(self) -> None:
        super().__init__("odom_watch")
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

        self.x = self.y = self.th = 0.0
        self.x0 = self.y0 = self.th0 = 0.0
        self.th_unwrap = 0.0          # yaw tich luy, khong bi cuon ve +-180
        self.th_prev = None
        self.th_unwrap0 = 0.0
        self.path_len = 0.0           # quang duong di duoc theo duong cong
        self.path0 = 0.0
        self.last_xy = None
        self.have = False

        self.create_timer(0.1, self.draw)
        self.create_timer(0.05, self.keys)

        self.old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print("Enter = dat moc 0 | d = chot ket qua | q = thoat\n")

    def restore(self) -> None:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_term)

    def on_odom(self, msg: Odometry) -> None:
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        th = yaw_of(msg)

        if self.th_prev is not None:          # go cuon goc
            d = th - self.th_prev
            d = math.atan2(math.sin(d), math.cos(d))
            self.th_unwrap += d
        self.th_prev = th
        self.th = th

        if self.last_xy is not None:
            self.path_len += math.hypot(self.x - self.last_xy[0],
                                        self.y - self.last_xy[1])
        self.last_xy = (self.x, self.y)
        self.have = True

    def mark(self) -> None:
        self.x0, self.y0 = self.x, self.y
        self.th_unwrap0 = self.th_unwrap
        self.path0 = self.path_len
        print("\n>> DAT MOC 0. Bat dau chay.\n")

    def report(self) -> None:
        dx, dy = self.x - self.x0, self.y - self.y0
        straight = math.hypot(dx, dy)
        path = self.path_len - self.path0
        dth = math.degrees(self.th_unwrap - self.th_unwrap0)
        print("\n" + "=" * 56)
        print("KET QUA LAN DO")
        print(f"  Khoang cach duong thang : {straight:.4f} m")
        print(f"  Quang duong theo duong  : {path:.4f} m")
        print(f"  Goc quay tich luy       : {dth:+.2f} deg ({dth/360:+.3f} vong)")
        print(f"  dx={dx:+.4f}  dy={dy:+.4f}")
        print("-" * 56)
        if straight > 0.3:
            print(f"  TINH TIEN: neu do that = L m thi")
            print(f"     wheel_diameter_moi = 0.095 * L / {straight:.4f}")
            for L in (1.0, 2.0, 3.0):
                print(f"        L={L:.1f} m -> {0.095 * L / straight:.5f}")
        if abs(dth) > 180:
            n = round(dth / 360)
            if n != 0:
                print(f"  QUAY: neu that su quay {n} vong ({n*360} deg) thi")
                print(f"     wheel_separation_moi = 0.390 * {dth:.1f} / {n*360} "
                      f"= {0.390 * dth / (n * 360):.5f}")
        print("=" * 56 + "\n")

    def keys(self) -> None:
        if not select.select([sys.stdin], [], [], 0)[0]:
            return
        c = sys.stdin.read(1)
        if c in ("\n", "\r"):
            self.mark()
        elif c == "d":
            self.report()
        elif c == "q":
            raise KeyboardInterrupt

    def draw(self) -> None:
        if not self.have:
            return
        dx, dy = self.x - self.x0, self.y - self.y0
        dth = math.degrees(self.th_unwrap - self.th_unwrap0)
        sys.stdout.write(
            f"\r  x={dx:+.3f}  y={dy:+.3f}  thang={math.hypot(dx, dy):.3f} m  "
            f"duong={self.path_len - self.path0:.3f} m  yaw={dth:+8.2f} deg   ")
        sys.stdout.flush()


def main() -> None:
    rclpy.init()
    node = OdomWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()