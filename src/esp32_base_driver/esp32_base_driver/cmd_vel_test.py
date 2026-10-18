#!/usr/bin/env python3
"""BAC 3 - node phat /cmd_vel theo kich ban va doi chieu voi /odom.

Chay cung base_node. Khong lien quan gi toi SLAM / Nav2.

    ros2 launch esp32_base_driver test_drive.launch.py
    ros2 run esp32_base_driver cmd_vel_test --ros-args -p phase:=spin_left
    ros2 run esp32_base_driver cmd_vel_test --ros-args -p lifted:=false
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CmdVelTest(Node):
    RATE = 20.0

    def __init__(self) -> None:
        super().__init__("cmd_vel_test")

        self.declare_parameter("linear", 0.15)     # m/s
        self.declare_parameter("angular", 0.8)     # rad/s
        self.declare_parameter("duration", 3.0)    # s moi pha
        self.declare_parameter("pause", 1.5)       # s nghi giua cac pha
        self.declare_parameter("wheel_separation", 0.3890)
        self.declare_parameter("lifted", True)     # True = dang ke banh len
        self.declare_parameter("phase", "all")

        v = self.get_parameter("linear").value
        w = self.get_parameter("angular").value
        d = self.get_parameter("duration").value
        self.pause = self.get_parameter("pause").value
        self.sep = self.get_parameter("wheel_separation").value
        self.lifted = self.get_parameter("lifted").value

        # (ten, v, w, giay, ky vong nhin tu tren xuong)
        allp = [
            ("forward",    v,   0.0, d, "ca 2 banh TIEN, bang toc do"),
            ("backward",  -v,   0.0, d, "ca 2 banh LUI"),
            ("spin_left",  0.0,  w,  d, "TRAI lui, PHAI tien, yaw TANG"),
            ("spin_right", 0.0, -w,  d, "TRAI tien, PHAI lui, yaw GIAM"),
            ("arc_left",   v,  w / 2, d, "ca 2 tien, PHAI nhanh hon"),
            ("arc_right",  v, -w / 2, d, "ca 2 tien, TRAI nhanh hon"),
        ]
        sel = self.get_parameter("phase").value
        self.phases = allp if sel == "all" else [p for p in allp if p[0] == sel]
        if not self.phases:
            self.get_logger().error(f"Khong co pha ten '{sel}'")
            raise SystemExit(1)

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

        self.odom = None
        self.mark = None
        self.idx = 0
        self.t = 0.0
        self.resting = False
        self.started = False

        self.get_logger().info("Cho /odom tu base_node...")
        self.create_timer(1.0 / self.RATE, self.tick)

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def snapshot(self):
        o = self.odom.pose.pose.position
        return (o.x, o.y, yaw_of(self.odom))

    # ------------------------------------------------------------ report
    def announce(self, name, v, w, dur, note) -> None:
        vl = v - w * self.sep / 2
        vr = v + w * self.sep / 2
        self.get_logger().info(
            f"\n=== {name} ===  v={v:+.2f} m/s  w={w:+.2f} rad/s  ({dur:.0f}s)"
            f"\n    ky vong    : {note}"
            f"\n    banh TRAI  : {vl:+.3f} m/s"
            f"\n    banh PHAI  : {vr:+.3f} m/s")

    def report(self, name, v, w, dur) -> None:
        x0, y0, th0 = self.mark
        x1, y1, th1 = self.snapshot()
        dth = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
        dist = math.hypot(x1 - x0, y1 - y0)
        exp_d, exp_th = v * dur, w * dur

        lines = [f"--- ket qua {name} ---",
                 f"    yaw   : do {math.degrees(dth):+7.1f} deg | "
                 f"ky vong {math.degrees(exp_th):+7.1f} deg",
                 f"    quang duong: do {dist:6.3f} m | ky vong {abs(exp_d):6.3f} m"]

        if abs(exp_th) > 0.1 and dth * exp_th < 0:
            lines.append("    !! YAW NGUOC DAU -> doi cheo day 2 dong co, "
                         "hoac sua M2_DIR/M2_ENC_DIR")
        if abs(exp_d) > 0.05 and abs(dth) > 0.35 and abs(w) < 0.01:
            lines.append("    !! Le nhieu khi di thang -> 2 banh khong bam "
                         "cung toc do, tuning lai PID hoac do lai deadband")
        if self.lifted:
            lines.append("    (dang ke banh len -> chi doc DAU va TY LE, "
                         "khong doc gia tri tuyet doi)")
        self.get_logger().info("\n".join(lines))

    # ------------------------------------------------------------ loop
    def tick(self) -> None:
        if self.odom is None:
            return
        if not self.started:
            self.started = True
            self.get_logger().info("Bat dau kich ban.")

        if self.idx >= len(self.phases):
            self.pub.publish(Twist())
            self.get_logger().info("HOAN TAT. Ctrl-C de thoat.")
            rclpy.shutdown()
            return

        name, v, w, dur, note = self.phases[self.idx]

        if self.resting:
            self.pub.publish(Twist())
            self.t += 1.0 / self.RATE
            if self.t >= self.pause:
                self.resting = False
                self.t = 0.0
                self.idx += 1
                self.mark = None
            return

        if self.mark is None:
            self.mark = self.snapshot()
            self.announce(name, v, w, dur, note)

        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        self.t += 1.0 / self.RATE

        if self.t >= dur:
            self.report(name, v, w, dur)
            self.pub.publish(Twist())
            self.resting = True
            self.t = 0.0


def main() -> None:
    rclpy.init()
    node = CmdVelTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
