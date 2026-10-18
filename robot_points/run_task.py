#!/usr/bin/env python3
"""Chay chuoi diem qua Nav2, TU DONG do sai so tai moi diem.

    python3 run_task.py start A B C end
    python3 run_task.py start A B C end --repeat 10
    python3 run_task.py --stats                  # xem thong ke tu CSV da co
    python3 run_task.py --list

Sau khi toi moi diem, script doi xe dung han roi doc TF map->base_link,
so voi toa do dich, in sai so ra man hinh va ghi vao errors.csv.

Cuoi moi lan chay in bang tom tat. Voi --repeat in them thong ke tong.
"""

import argparse
import csv
import math
import os
import statistics
import sys
import time
from datetime import datetime

import yaml

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_POINTS = os.path.join(HERE, "points.yaml")
CSV_PATH = os.path.join(HERE, "errors.csv")

# doi bao lau sau khi Nav2 bao "toi noi" moi do, cho xe dung han
SETTLE_SEC = 1.5


# ------------------------------------------------------------------ utils
def load_points(path):
    with open(path) as f:
        data = yaml.safe_load(f)
    pts = data.get("points")
    if not pts:
        sys.exit(f"Khong tim thay muc 'points' trong {path}")
    return pts


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def yaw_from_quat(z, w):
    return 2.0 * math.atan2(z, w)


def ang_diff(a, b):
    """Hieu goc, chuan hoa ve [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def read_pose(nav, tf_buffer, timeout=3.0):
    """Doc TF map->base_link. Tra ve (x, y, yaw) hoac None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            t = tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
            q = t.transform.rotation
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    yaw_from_quat(q.z, q.w))
        except Exception:
            rclpy.spin_once(nav, timeout_sec=0.1)
    return None


# ------------------------------------------------------------------ ghi CSV
CSV_HEADER = ["thoi_gian", "lan", "diem",
              "dich_x", "dich_y", "dich_yaw",
              "that_x", "that_y", "that_yaw",
              "dx", "dy", "euclid_m", "dyaw_deg", "ket_qua"]


def append_csv(row):
    new = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(CSV_HEADER)
        w.writerow(row)


# ------------------------------------------------------------------ chay
def go_to(nav, name, spec):
    pose = make_pose(nav, spec["x"], spec["y"], spec.get("yaw", 0.0))
    nav.goToPose(pose)

    last = 0.0
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        now = time.time()
        if fb and now - last > 2.0:
            last = now
            print(f"      con {fb.distance_remaining:.2f} m", flush=True)

    r = nav.getResult()
    if r == TaskResult.SUCCEEDED:
        return "toi_noi"
    if r == TaskResult.CANCELED:
        return "bi_huy"
    return "that_bai"


def measure(nav, tf_buffer, run_no, name, spec, outcome):
    """Do sai so tai diem hien tai, in ra va ghi CSV. Tra ve dict."""
    time.sleep(SETTLE_SEC)
    actual = read_pose(nav, tf_buffer)
    if actual is None:
        print("      khong doc duoc TF map->base_link")
        return None

    ax, ay, ayaw = actual
    tx, ty = float(spec["x"]), float(spec["y"])
    tyaw = float(spec.get("yaw", 0.0))

    dx, dy = ax - tx, ay - ty
    euclid = math.hypot(dx, dy)
    dyaw = math.degrees(ang_diff(ayaw, tyaw))

    print(f"      sai so: {euclid*100:5.1f} cm | "
          f"goc {dyaw:+6.2f} do  (dx={dx*100:+.1f} dy={dy*100:+.1f} cm)")

    append_csv([
        datetime.now().strftime("%Y-%m-%d %H:%M:%S"), run_no, name,
        f"{tx:.4f}", f"{ty:.4f}", f"{tyaw:.4f}",
        f"{ax:.4f}", f"{ay:.4f}", f"{ayaw:.4f}",
        f"{dx:.4f}", f"{dy:.4f}", f"{euclid:.4f}", f"{dyaw:.2f}", outcome,
    ])
    return {"diem": name, "euclid": euclid, "dyaw": abs(dyaw),
            "ket_qua": outcome}


def dwell(seconds, name):
    if seconds <= 0:
        return
    print(f"      dung {int(seconds)}s")
    for k in range(int(seconds), 0, -1):
        print(f"        {k} ", end="\r", flush=True)
        time.sleep(1.0)
    print("           ")


# ------------------------------------------------------------------ bang
def summary(rows, tieu_de):
    if not rows:
        return
    print(f"\n{tieu_de}")
    print("-" * 62)
    print(f"{'diem':<8}{'lan':>5}{'TB cm':>9}{'max cm':>9}"
          f"{'TB do':>9}{'max do':>9}")
    print("-" * 62)

    by_point = {}
    for r in rows:
        by_point.setdefault(r["diem"], []).append(r)

    for name, rs in by_point.items():
        e = [r["euclid"] * 100 for r in rs]
        a = [r["dyaw"] for r in rs]
        print(f"{name:<8}{len(rs):>5}{statistics.mean(e):>9.1f}"
              f"{max(e):>9.1f}{statistics.mean(a):>9.2f}{max(a):>9.2f}")

    e_all = [r["euclid"] * 100 for r in rows]
    a_all = [r["dyaw"] for r in rows]
    print("-" * 62)
    print(f"{'TAT CA':<8}{len(rows):>5}{statistics.mean(e_all):>9.1f}"
          f"{max(e_all):>9.1f}{statistics.mean(a_all):>9.2f}{max(a_all):>9.2f}")

    if len(e_all) > 1:
        print(f"\n  do lech chuan:  vi tri {statistics.stdev(e_all):.1f} cm"
              f"   |   goc {statistics.stdev(a_all):.2f} do")
    print("-" * 62)


def show_csv_stats():
    if not os.path.exists(CSV_PATH):
        sys.exit(f"Chua co {CSV_PATH}")
    rows = []
    with open(CSV_PATH) as f:
        for r in csv.DictReader(f):
            rows.append({"diem": r["diem"],
                         "euclid": float(r["euclid_m"]),
                         "dyaw": abs(float(r["dyaw_deg"])),
                         "ket_qua": r["ket_qua"]})
    summary(rows, f"THONG KE TU {os.path.basename(CSV_PATH)}")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("route", nargs="*")
    ap.add_argument("--points", default=DEFAULT_POINTS)
    ap.add_argument("--repeat", type=int, default=1,
                    help="chay lo trinh bao nhieu lan")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--stats", action="store_true",
                    help="chi in thong ke tu errors.csv roi thoat")
    ap.add_argument("--keep-going", action="store_true")
    args = ap.parse_args()

    if args.stats:
        show_csv_stats()
        return

    points = load_points(args.points)

    if args.list:
        for n, s in points.items():
            print(f"  {n:8s} x={s['x']:7.2f} y={s['y']:7.2f} "
                  f"yaw={s.get('yaw', 0.0):6.2f}  dwell={s.get('dwell', 0)}s")
        return

    if not args.route:
        sys.exit("Vi du: python3 run_task.py start A C B end --repeat 10")

    unknown = [n for n in args.route if n not in points]
    if unknown:
        sys.exit(f"Khong co diem: {', '.join(unknown)}")

    rclpy.init()
    nav = BasicNavigator()
    tf_buffer = Buffer()
    TransformListener(tf_buffer, nav)

    print("Cho Nav2 san sang...")
    nav.waitUntilNav2Active()
    print(f"Lo trinh: {' -> '.join(args.route)}   x{args.repeat} lan\n")

    all_rows = []
    t0 = time.time()

    try:
        for run_no in range(1, args.repeat + 1):
            print(f"\n{'='*62}\n  LAN {run_no}/{args.repeat}\n{'='*62}")
            run_rows = []

            for i, name in enumerate(args.route, 1):
                spec = points[name]
                print(f"\n  [{i}/{len(args.route)}] -> {name}")

                outcome = go_to(nav, name, spec)
                print(f"      {outcome}")

                m = measure(nav, tf_buffer, run_no, name, spec, outcome)
                if m:
                    run_rows.append(m)
                    all_rows.append(m)

                if outcome == "toi_noi":
                    dwell(spec.get("dwell", 0), name)
                elif outcome == "bi_huy":
                    raise KeyboardInterrupt
                elif not args.keep_going:
                    print("\n  Dung lo trinh (dung --keep-going de di tiep)")
                    break

            if args.repeat > 1:
                summary(run_rows, f"  Tom tat lan {run_no}")

    except KeyboardInterrupt:
        print("\n\nCtrl+C - dang huy...")
        nav.cancelTask()
        time.sleep(1.0)

    summary(all_rows, "TONG KET")
    print(f"\nThoi gian: {time.time() - t0:.0f}s")
    print(f"Da ghi vao: {CSV_PATH}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()