#!/usr/bin/env python3
"""Nap duong di ve tay vao Nav2 qua action /follow_path, co ghi sai lech.

CACH DE NHAT — khong phai copy toa do nao ca.
Dat xe o giua cho trong, dau xe huong sao cho BEN TRAI thoang, roi:

    python3 path_gen.py --around-robot 0.6 --view     # xem truoc
    python3 path_gen.py --around-robot 0.6 --split 4  # chay that

Tam duoc tinh tu dong: vuong goc ben trai dau xe, cach dung R.
Xe nam san tren duong tron, huong xe trung tiep tuyen -> khong canh bao.
Them --cw de tam nam ben phai (chay thuan kim dong ho).

CACH CU — tam co dinh, phai go toa do THAT (khong phai chu CX, CY):

    python3 path_gen.py --circle="-0.0800446,-9.34933,0.6" --view

Duong gap khuc:

    python3 path_gen.py --points "0,0 2,0 2,1.5" --split 1

Ghi track.csv (cung dinh dang circle_track.py) -> ve bang plot_track.py.

Yeu cau:
  - Nav2 dang chay, AMCL da hoi tu (scan trung khit tuong trong RViz)
  - Ten goal checker trong nav2_params phai khop tham so --goal-checker
    (mac dinh o day la "goal_checker", dung voi file cua xe nay)

Path duoc publish tren /custom_path (QoS transient_local) de xem trong RViz.
"""

import argparse
import csv
import math
import os
import sys


def check_distro(force=False):
    """Chan truong hop chay bang Jazzy trong khi robot la Humble.

    nav2_msgs/action/FollowPath KHAC NHAU giua hai ban:
        Humble: path, controller_id, goal_checker_id
        Jazzy : path, controller_id, goal_checker_id, progress_checker_id
    Goal gui tu Jazzy -> server Humble giai ma sai het, bao
    "Invalid path, Path is empty" va "No controller was specified".
    """
    d = os.environ.get("ROS_DISTRO", "")
    if d == "humble":
        return
    print("=" * 66)
    print(f"  ROS_DISTRO = '{d or 'KHONG DAT'}'  — script nay can HUMBLE")
    print()
    print("  Robot chay Humble. Action FollowPath cua Jazzy co them truong")
    print("  progress_checker_id, nen goal gui di se bi giai ma sai:")
    print("      controller_server: Invalid path, Path is empty")
    print()
    print("  Chay trong container Humble tren laptop:")
    print("      cd ~/ros_humble_docker && ./run_humble.sh")
    print("      python3 /home/dev/host/path_gen.py --around-robot 0.6 --view")
    print()
    print("  Hoac chep sang Jetson roi chay trong container amr_nav.")
    print()
    print("  Bo qua kiem tra (KHONG khuyen khich): them --force-distro")
    print("=" * 66)
    sys.exit(1)


if "--force-distro" not in sys.argv:
    check_distro()
else:
    sys.argv.remove("--force-distro")

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
CSV_PATH = os.path.join(HERE, "track.csv")


def quat_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathRunner(Node):
    def __init__(self, args):
        super().__init__("path_gen")
        self.a = args
        self.ref = None          # ("circle", cx, cy, R, ccw) hoac ("poly", pts)

        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/custom_path", qos)

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.v = self.w = 0.0

        self.client = ActionClient(self, FollowPath, "follow_path")
        self.rows = []
        self.t0 = None
        self.seg_i = 0
        self.segments = []
        self.rec_timer = None

    def on_cmd(self, m):
        self.v, self.w = m.linear.x, m.angular.z

    # ------------------------------------------------------------- pose
    def pose(self):
        try:
            t = self.tf_buf.lookup_transform(
                self.a.frame, "base_link", rclpy.time.Time())
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_of(t.transform.rotation))
        except Exception:
            return None

    def wait_pose(self, timeout=8.0):
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            p = self.pose()
            if p is not None:
                return p
        return None

    # -------------------------------------------------- sinh danh sach diem
    def circle_points(self, cx, cy, R, start_ang, sweep, n):
        pts = []
        for k in range(n + 1):
            a = start_ang + sweep * k / float(n)
            tang = a + (math.pi / 2 if sweep > 0 else -math.pi / 2)
            pts.append((cx + R * math.cos(a), cy + R * math.sin(a), tang))
        return pts

    def poly_points(self, raw):
        pts = []
        for i in range(len(raw) - 1):
            x0, y0 = raw[i]
            x1, y1 = raw[i + 1]
            L = math.hypot(x1 - x0, y1 - y0)
            yaw = math.atan2(y1 - y0, x1 - x0)
            n = max(2, int(L / self.a.spacing))
            for k in range(n):
                r = k / float(n)
                pts.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0), yaw))
        x1, y1 = raw[-1]
        yaw = math.atan2(y1 - raw[-2][1], x1 - raw[-2][0])
        pts.append((x1, y1, yaw))
        return pts

    def to_path(self, pts):
        p = Path()
        p.header.frame_id = self.a.frame
        p.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in pts:
            ps = PoseStamped()
            ps.header = p.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = quat_z(yaw)
            p.poses.append(ps)
        return p

    # -------------------------------------------------------- diem gan nhat
    def nearest(self, x, y):
        if self.ref[0] == "circle":
            _, cx, cy, R, ccw = self.ref
            a = math.atan2(y - cy, x - cx)
            xr, yr = cx + R * math.cos(a), cy + R * math.sin(a)
            thr = wrap(a + (math.pi / 2 if ccw else -math.pi / 2))
            e_rad = math.hypot(x - cx, y - cy) - R
            return xr, yr, thr, e_rad
        pts = self.ref[1]
        best = min(pts, key=lambda p: (p[0] - x) ** 2 + (p[1] - y) ** 2)
        return best[0], best[1], best[2], math.hypot(best[0] - x, best[1] - y)

    # ------------------------------------------------------------ ghi mau
    def record(self):
        p = self.pose()
        if p is None:
            return
        x, y, th = p
        if self.t0 is None:
            self.t0 = self.get_clock().now().nanoseconds
        t = (self.get_clock().now().nanoseconds - self.t0) / 1e9

        xr, yr, thr, e_rad = self.nearest(x, y)
        dx, dy = xr - x, yr - y
        ex = math.cos(th) * dx + math.sin(th) * dy
        ey = -math.sin(th) * dx + math.cos(th) * dy
        eth = wrap(thr - th)

        self.rows.append([
            f"{t:.3f}", f"{xr:.4f}", f"{yr:.4f}", f"{thr:.4f}",
            f"{x:.4f}", f"{y:.4f}", f"{th:.4f}",
            f"{ex:.4f}", f"{ey:.4f}", f"{eth:.4f}",
            f"{math.hypot(dx, dy):.4f}", f"{e_rad:.4f}",
            f"{self.v:.4f}", f"{self.w:.4f}",
        ])

    # --------------------------------------------------------- gui tung cung
    def send_next(self):
        if self.seg_i >= len(self.segments):
            self.finish()
            return
        seg = self.segments[self.seg_i]
        self.get_logger().info(
            f"cung {self.seg_i + 1}/{len(self.segments)}, {len(seg.poses)} diem")

        goal = FollowPath.Goal()
        goal.path = seg
        goal.controller_id = self.a.controller
        goal.goal_checker_id = self.a.goal_checker
        self.client.send_goal_async(goal).add_done_callback(self.on_accept)

    def on_accept(self, fut):
        h = fut.result()
        if not h.accepted:
            self.get_logger().error(
                "goal bi tu choi — kiem tra --goal-checker va --controller "
                "co khop ten trong nav2_params khong")
            self.finish()
            return
        h.get_result_async().add_done_callback(self.on_done)

    def on_done(self, fut):
        st = fut.result().status
        if st != 4:          # 4 = SUCCEEDED
            self.get_logger().warn(f"cung {self.seg_i + 1} ket thuc status {st}")
        self.seg_i += 1
        self.send_next()

    # -------------------------------------------------------------- xong
    def finish(self):
        if self.rec_timer:
            self.rec_timer.cancel()
        if not self.rows:
            self.get_logger().warn("khong ghi duoc mau nao")
            rclpy.shutdown()
            return

        header = ["t", "xr", "yr", "thr", "x", "y", "th",
                  "ex", "ey", "eth", "e_pos", "e_radial", "v", "w"]
        with open(CSV_PATH, "w", newline="") as f:
            c = csv.writer(f)
            c.writerow(header)
            c.writerows(self.rows)

        ep = [abs(float(r[10])) for r in self.rows]
        er = [abs(float(r[11])) for r in self.rows]
        n = len(ep)
        print("\n" + "=" * 54)
        print(f"  sai lech vi tri   TB {sum(ep)/n*100:6.2f} cm   max {max(ep)*100:6.2f} cm")
        print(f"  RMSE vi tri          {math.sqrt(sum(e*e for e in ep)/n)*100:6.2f} cm")
        print(f"  sai lech ban kinh TB {sum(er)/n*100:6.2f} cm   max {max(er)*100:6.2f} cm")
        print(f"  {n} mau, da ghi {CSV_PATH}")
        print("=" * 54 + "\n")
        rclpy.shutdown()

    # -------------------------------------------------------------- chay
    def run(self):
        full, self.segments = self.plan()
        self.path_pub.publish(full)
        self.get_logger().info(
            f"{len(full.poses)} diem, da publish /custom_path")

        if self.a.view:
            self.get_logger().info("che do xem truoc. Ctrl+C de thoat.")
            return

        if not self.client.wait_for_server(timeout_sec=5.0):
            sys.exit("khong thay /follow_path — controller_server chua active?")

        self.rec_timer = self.create_timer(1.0 / self.a.rate, self.record)
        self.send_next()

    # --------------------------------------------------------- lap ke hoach
    def plan(self):
        if self.a.circle or self.a.around_robot:
            ccw = not self.a.cw

            p = self.wait_pose()
            if p is None:
                sys.exit(f"khong doc duoc TF {self.a.frame} -> base_link")

            if self.a.around_robot:
                # Tam dat vuong goc voi dau xe, cach dung R.
                # -> xe nam SAN tren duong tron, huong xe TRUNG tiep tuyen.
                R = self.a.around_robot
                sgn = 1.0 if ccw else -1.0        # ccw: tam ben trai
                x, y, th = p
                cx = x - sgn * R * math.sin(th)
                cy = y + sgn * R * math.cos(th)
                self.get_logger().info(
                    f"xe tai ({x:.3f}, {y:.3f}) yaw {math.degrees(th):.1f} do")
                self.get_logger().info(
                    f"tam vong tu dong: ({cx:.4f}, {cy:.4f})  R={R}  "
                    f"{'nguoc' if ccw else 'thuan'} kim dong ho")
            else:
                cx, cy, R = [float(v) for v in self.a.circle.split(",")]

            self.ref = ("circle", cx, cy, R, ccw)

            start_ang = math.atan2(p[1] - cy, p[0] - cx)
            d = math.hypot(p[0] - cx, p[1] - cy)
            self.get_logger().info(
                f"xe cach tam {d:.2f} m, vao vong tai goc "
                f"{math.degrees(start_ang):.0f} do")
            if abs(d - R) > 0.5:
                self.get_logger().warn(
                    f"xe cach duong tron {abs(d-R):.2f} m — "
                    "nen dua xe lai gan truoc khi chay")

            sweep_total = 2 * math.pi * self.a.laps * (1 if ccw else -1)
            k = max(1, self.a.split)
            segs, all_pts = [], []
            for i in range(k):
                a0 = start_ang + sweep_total * i / k
                pts = self.circle_points(
                    cx, cy, R, a0, sweep_total / k,
                    max(8, int(abs(sweep_total / k) * R / self.a.spacing)))
                segs.append(self.to_path(pts))
                all_pts += pts
            return self.to_path(all_pts), segs

        raw = [tuple(float(v) for v in tok.split(","))
               for tok in self.a.points.split()]
        if len(raw) < 2:
            sys.exit("can it nhat 2 diem")
        pts = self.poly_points(raw)
        self.ref = ("poly", pts)
        k = max(1, self.a.split)
        step = len(pts) // k
        segs = [self.to_path(pts[i * step:(i + 1) * step + 1])
                for i in range(k)]
        return self.to_path(pts), segs


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--around-robot", type=float, metavar="R",
                   help="CHI CAN BAN KINH. Tam tu tinh sao cho xe nam san "
                        "tren duong tron, huong xe trung tiep tuyen. "
                        "Khong phai copy toa do.")
    g.add_argument("--circle", help="cx,cy,R trong he map (toa do THAT, "
                                    'vd: --circle="-0.08,-9.35,0.6")')
    g.add_argument("--points", help='vd: "0,0 2,0 2,1.5"')

    ap.add_argument("--laps", type=float, default=1.0)
    ap.add_argument("--cw", action="store_true", help="quay theo kim dong ho")
    ap.add_argument("--split", type=int, default=4,
                    help="chia thanh may cung gui tuan tu (vong tron nen >= 2)")
    ap.add_argument("--spacing", type=float, default=0.05)
    ap.add_argument("--frame", default="map")
    ap.add_argument("--rate", type=int, default=20)
    ap.add_argument("--controller", default="FollowPath")
    ap.add_argument("--goal-checker", default="goal_checker",
                    help="ten trong goal_checker_plugins cua nav2_params")
    ap.add_argument("--view", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = PathRunner(args)
    try:
        node.run()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.create_publisher(Twist, "/cmd_vel", 1).publish(Twist())
        print("\nda dung")
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()