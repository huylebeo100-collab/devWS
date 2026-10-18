#!/usr/bin/env python3
"""Dieu khien bam quy dao tron cho robot vi sai (luat Kanayama).

    python3 circle_track.py                       # R=0.75, v=0.15, 1 vong
    python3 circle_track.py --radius 0.75 --vel 0.12 --laps 2
    python3 circle_track.py --frame map           # do bang AMCL thay vi odom

Ghi track.csv de ve do thi bang plot_track.py hoac MATLAB.

TAT Nav2 truoc khi chay, neu khong hai ben se tranh /cmd_vel.

--------------------------------------------------------------------
LUAT DIEU KHIEN (Kanayama 1990)

Sai lech trong he toa do gan than xe:
    ex  =  cos(th)*(xr - x) + sin(th)*(yr - y)      doc truc xe
    ey  = -sin(th)*(xr - x) + cos(th)*(yr - y)      ngang truc xe
    eth =  thr - th                                  goc

Luat dieu khien:
    v = vr*cos(eth) + Kx*ex
    w = wr + vr*(Ky*ey + Kth*sin(eth))

Kx  keo xe bat kip diem tham chieu theo phuong doc
Ky  keo xe ve duong quy dao theo phuong ngang
Kth chinh huong cho khop tiep tuyen
--------------------------------------------------------------------
"""

import argparse
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "track.csv")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class CircleTracker(Node):
    def __init__(self, args):
        super().__init__("circle_tracker")
        self.a = args

        self.R = args.radius
        self.vr = args.vel
        self.wr = self.vr / self.R          # vr = R * wr
        self.T = 2.0 * math.pi / self.wr    # thoi gian mot vong
        self.total = self.T * args.laps

        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.x0 = self.y0 = self.th0 = None
        self.t0 = None
        self.rows = []

        self.get_logger().info(
            f"R={self.R} m  v={self.vr} m/s  w={self.wr:.4f} rad/s  "
            f"mot vong {self.T:.1f}s  tong {self.total:.1f}s")

        self.timer = self.create_timer(1.0 / args.rate, self.step)

    # ---------------------------------------------------------- doc pose
    def pose(self):
        try:
            t = self.tf_buf.lookup_transform(
                self.a.frame, "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_of(t.transform.rotation))
        except Exception:
            return None

    # ------------------------------------------------- diem tham chieu
    def reference(self, t):
        """Duong tron tam ben TRAI xe luc xuat phat, di nguoc kim dong ho.
        Tai t=0 diem tham chieu trung goc xuat phat, tiep tuyen trung huong xe."""
        a = self.wr * t
        # trong he toa do xuat phat
        lx = self.R * math.sin(a)
        ly = self.R * (1.0 - math.cos(a))
        lth = a
        # doi sang he toa do the gioi
        c, s = math.cos(self.th0), math.sin(self.th0)
        xr = self.x0 + c * lx - s * ly
        yr = self.y0 + s * lx + c * ly
        thr = wrap(self.th0 + lth)
        return xr, yr, thr

    # ------------------------------------------------------- vong lap
    def step(self):
        p = self.pose()
        if p is None:
            self.get_logger().warn(
                f"chua co TF {self.a.frame} -> base_link", once=True)
            return
        x, y, th = p

        if self.t0 is None:
            self.x0, self.y0, self.th0 = x, y, th
            self.t0 = time.time()
            self.get_logger().info(
                f"xuat phat tai ({x:.3f}, {y:.3f}, {math.degrees(th):.1f} do)")
            return

        t = time.time() - self.t0
        if t > self.total:
            self.finish()
            return

        xr, yr, thr = self.reference(t)

        # sai lech trong he gan than xe
        dx, dy = xr - x, yr - y
        ex = math.cos(th) * dx + math.sin(th) * dy
        ey = -math.sin(th) * dx + math.cos(th) * dy
        eth = wrap(thr - th)

        # luat Kanayama
        v = self.vr * math.cos(eth) + self.a.kx * ex
        w = self.wr + self.vr * (self.a.ky * ey + self.a.kth * math.sin(eth))

        v = max(-self.a.vmax, min(self.a.vmax, v))
        w = max(-self.a.wmax, min(self.a.wmax, w))

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

        # sai lech ngang so voi duong tron (duong kinh tuyen)
        cx = self.x0 - self.R * math.sin(self.th0)
        cy = self.y0 + self.R * math.cos(self.th0)
        e_radial = math.hypot(x - cx, y - cy) - self.R

        self.rows.append([
            f"{t:.3f}", f"{xr:.4f}", f"{yr:.4f}", f"{thr:.4f}",
            f"{x:.4f}", f"{y:.4f}", f"{th:.4f}",
            f"{ex:.4f}", f"{ey:.4f}", f"{eth:.4f}",
            f"{math.hypot(dx, dy):.4f}", f"{e_radial:.4f}",
            f"{v:.4f}", f"{w:.4f}",
        ])

        if len(self.rows) % (self.a.rate * 2) == 0:
            self.get_logger().info(
                f"t={t:5.1f}s  sai lech {math.hypot(dx, dy)*100:5.1f} cm  "
                f"ngang {ey*100:+5.1f} cm  v={v:.3f} w={w:+.3f}")

    # -------------------------------------------------------- ket thuc
    def finish(self):
        self.timer.cancel()
        self.pub.publish(Twist())          # dung xe
        time.sleep(0.2)
        self.pub.publish(Twist())

        header = ["t", "xr", "yr", "thr", "x", "y", "th",
                  "ex", "ey", "eth", "e_pos", "e_radial", "v", "w"]
        with open(CSV_PATH, "w", newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(header)
            wcsv.writerows(self.rows)

        ep = [abs(float(r[10])) for r in self.rows]
        er = [abs(float(r[11])) for r in self.rows]
        eth = [abs(float(r[9])) for r in self.rows]
        n = len(ep)
        rmse = math.sqrt(sum(e * e for e in ep) / n)

        print("\n" + "=" * 54)
        print(f"  R = {self.R} m,  v = {self.vr} m/s,  {self.a.laps} vong")
        print("-" * 54)
        print(f"  sai lech vi tri   TB {sum(ep)/n*100:6.2f} cm   "
              f"max {max(ep)*100:6.2f} cm")
        print(f"  RMSE vi tri          {rmse*100:6.2f} cm")
        print(f"  sai lech ban kinh TB {sum(er)/n*100:6.2f} cm   "
              f"max {max(er)*100:6.2f} cm")
        print(f"  sai lech goc      TB {math.degrees(sum(eth)/n):6.2f} do  "
              f"max {math.degrees(max(eth)):6.2f} do")
        print("-" * 54)
        print(f"  {n} mau, da ghi {CSV_PATH}")
        print("=" * 54 + "\n")

        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=0.75)
    ap.add_argument("--vel", type=float, default=0.15, help="van toc dai m/s")
    ap.add_argument("--laps", type=float, default=1.0)
    ap.add_argument("--frame", default="odom", choices=["odom", "map"])
    ap.add_argument("--rate", type=int, default=20, help="Hz")
    ap.add_argument("--kx", type=float, default=1.0)
    ap.add_argument("--ky", type=float, default=8.0)
    ap.add_argument("--kth", type=float, default=4.0)
    ap.add_argument("--vmax", type=float, default=0.25)
    ap.add_argument("--wmax", type=float, default=0.60)
    args = ap.parse_args()

    rclpy.init()
    node = CircleTracker(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
        print("\nda dung xe")
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()