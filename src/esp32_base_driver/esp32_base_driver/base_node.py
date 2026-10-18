#!/usr/bin/env python3
"""Cau noi giua ROS 2 va firmware ESP32-S3.

Subscribe : /cmd_vel        (geometry_msgs/Twist)
Publish   : /odom           (nav_msgs/Odometry)
            /joint_states   (sensor_msgs/JointState)
            TF odom -> base_link  (tat duoc neu dung robot_localization)
Service   : ~/reset_odom    (std_srvs/Empty)

Doc serial bang timer 200 Hz thay vi thread rieng: it cho hong hon, va
giong cau truc uart_check_pi.py da chay on dinh.

--------------------------------------------------------------------
MOI: BU VUNG CHET (deadband compensation)

Dong co co ma sat tinh. Duoi mot nguong toc do nhat dinh, lenh gui
xuong lam dong co rit ma banh khong quay. DWB cua Nav2 khong biet
dieu do, no van ra lenh quay 0.05 rad/s o pha chinh goc cuoi cung,
va xe dung im vinh vien.

Node nay khuech dai lenh qua nho len vua du de banh thuc su quay,
giu nguyen ti le v/w nen ban kinh cua khong doi.

Chi kich hoat khi CA HAI banh deu duoi nguong — vi ma sat tinh chi
can tro luc xuat phat tu trang thai dung yen. Khi mot banh da quay,
banh kia lan theo, khong can bu.

QUAN TRONG: phai tat deadband cua velocity_smoother, neu khong lenh
bi nen ve 0 truoc khi toi day va khong con gi de bu:
    deadband_velocity: [0.0, 0.0, 0.0]
--------------------------------------------------------------------
"""
import math

import rclpy
import serial
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from tf2_ros import TransformBroadcaster

EPS = 1e-6


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Esp32BaseNode(Node):
    POLL_HZ = 200.0          # tan so doc serial

    # ESP32 gui 50 Hz, toc do toi da ~279 rpm -> ~93 xung moi goi.
    # Nguong nay chan buoc nhay khi ESP32 khoi dong lai giua chung.
    #MAX_TICK_STEP = 500

    # Toc do toi da 279 rpm x 998.4 CPR = 4643 xung/giay. Nhan 2 lam le.
    MAX_TICK_RATE = 9300        # xung moi giay

    def __init__(self) -> None:
        super().__init__("esp32_base_driver")

        p = self.declare_parameter
        p("port", "/dev/ttyS0")
        p("baud", 115200)
        p("wheel_diameter", 0.095)
        p("wheel_separation", 0.389)
        p("ticks_per_rev", 998.4)          # da la che do x4
        p("max_linear", 0.80)              # m/s
        p("max_angular", 2.0)              # rad/s
        p("cmd_rate", 20.0)                # Hz gui xuong ESP32
        p("cmd_timeout", 0.5)              # s khong co cmd_vel -> gui 0
        p("publish_tf", True)
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("left_joint", "left_wheel_joint")
        p("right_joint", "right_wheel_joint")

        # --- bu vung chet ---
        # Toc do dai nho nhat mot banh con quay duoc tu trang thai dung yen.
        # Do bang teleop, xem phan huong dan o cuoi file.
        #   tien thang : nguong = gia tri do duoc truc tiep
        #   quay tai cho: nguong = w_do * wheel_separation / 2
        p("wheel_min_speed", 0.032)        # m/s
        p("boost_max", 6.0)                # he so khuech dai toi da
        p("deadband_enable", True)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.port = g("port")
        self.baud = int(g("baud"))
        self.wheel_sep = float(g("wheel_separation"))
        self.max_lin = float(g("max_linear"))
        self.max_ang = float(g("max_angular"))
        self.cmd_timeout = float(g("cmd_timeout"))
        self.publish_tf = bool(g("publish_tf"))
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.left_joint = g("left_joint")
        self.right_joint = g("right_joint")

        self.wheel_min = float(g("wheel_min_speed"))
        self.boost_max = float(g("boost_max"))
        self.deadband_on = bool(g("deadband_enable"))

        diam = float(g("wheel_diameter"))
        self.wheel_radius = diam * 0.5
        self.m_per_tick = (math.pi * diam) / float(g("ticks_per_rev"))

        # trang thai odom
        self.x = self.y = self.th = 0.0
        self.vx = self.wz = 0.0
        self.pos_l = self.pos_r = 0.0
        self.last_tl = self.last_tr = None
        self.last_stamp = None

        self.cmd_v = self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.boost_logged = False

        self.ser = None
        self.buf = bytearray()
        self.n_odom = 0
        self.warned_silent = False
        self.open_port()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        self.js_pub = self.create_publisher(JointState, "joint_states", qos)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, qos)
        self.tf_br = TransformBroadcaster(self)
        self.create_service(Empty, "~/reset_odom", self.on_reset_odom)

        self.create_timer(1.0 / self.POLL_HZ, self.poll_serial)
        self.create_timer(1.0 / float(g("cmd_rate")), self.tx_loop)
        self.create_timer(3.0, self.health_check)

        w_equiv = self.wheel_min * 2.0 / self.wheel_sep
        self.get_logger().info(
            f"Bu vung chet: {'BAT' if self.deadband_on else 'TAT'}, "
            f"wheel_min={self.wheel_min:.3f} m/s "
            f"(tuong duong v_min={self.wheel_min:.3f} m/s, "
            f"w_min={w_equiv:.3f} rad/s)")

    # -------------------------------------------------- serial
    def open_port(self) -> bool:
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=0, write_timeout=1.0,
            )
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Da mo {self.port} @ {self.baud} 8N1")
            self.ser.write(b"Z\n")          # xin ESP32 dat lai bo dem xung
            self.ser.flush()
            self.ser.reset_input_buffer()   # bo cac goi cu truoc khi reset
            self.last_tl = self.last_tr = None
            return True
        except serial.SerialException as e:
            self.get_logger().error(f"Khong mo duoc {self.port}: {e}")
            self.ser = None
            return False

    def poll_serial(self) -> None:
        if self.ser is None:
            self.open_port()
            return
        try:
            n = self.ser.in_waiting
            data = self.ser.read(n) if n else b""
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Mat ket noi: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.buf.clear()
            return

        if not data:
            return
        self.buf.extend(data)
        while b"\n" in self.buf:
            raw, _, rest = self.buf.partition(b"\n")
            self.buf = bytearray(rest)
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.handle_line(line)
        if len(self.buf) > 512:
            self.get_logger().warn("Buffer tran, xoa bo dem")
            self.buf.clear()

    def write(self, s: str) -> None:
        if self.ser is None:
            return
        try:
            self.ser.write(s.encode("ascii", errors="ignore"))
        except (serial.SerialException, serial.SerialTimeoutException) as e:
            self.get_logger().error(f"Loi ghi: {e}")

    def health_check(self) -> None:
        """Canh bao mot lan neu khong co odom - thuong la sai cong hoac dut day."""
        if self.n_odom == 0 and not self.warned_silent:
            self.warned_silent = True
            self.get_logger().error(
                f"Khong nhan duoc dong odom nao tu {self.port}. "
                "Kiem tra day TX/RX cheo nhau, GND chung, va firmware dang bat odom "
                "(bam 'k' tren monitor USB xem odom=1).")

    # -------------------------------------------------- bu vung chet
    def apply_deadband(self, v: float, w: float):
        """Khuech dai lenh qua nho de banh thuc su quay.

        Giu nguyen ti le v/w -> ban kinh cua khong doi, chi chay nhanh hon.
        Chi can thiep khi CA HAI banh deu duoi nguong.
        """
        if not self.deadband_on:
            return v, w
        if abs(v) < EPS and abs(w) < EPS:
            return 0.0, 0.0                 # lenh dung phai la dung that

        half = 0.5 * self.wheel_sep
        vl = v - w * half
        vr = v + w * half
        fastest = max(abs(vl), abs(vr))

        if fastest >= self.wheel_min or fastest < EPS:
            return v, w                     # da du nhanh, khong can bu

        k = min(self.wheel_min / fastest, self.boost_max)
        v, w = v * k, w * k

        # kep lai theo gioi han dong hoc, van giu ti le v/w
        s = 1.0
        if abs(v) > self.max_lin:
            s = min(s, self.max_lin / abs(v))
        if abs(w) > self.max_ang:
            s = min(s, self.max_ang / abs(w))
        v, w = v * s, w * s

        if not self.boost_logged:
            self.boost_logged = True
            self.get_logger().info(
                f"Bu vung chet lan dau: he so {k:.2f} -> v={v:.4f} w={w:.4f}")
        return v, w

    # -------------------------------------------------- cmd_vel
    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_v = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        self.cmd_w = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def tx_loop(self) -> None:
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.apply_deadband(self.cmd_v, self.cmd_w)
        self.write(f"V {v:.4f} {w:.4f}\n")

    # -------------------------------------------------- odom
    def on_reset_odom(self, req, res):
        self.x = self.y = self.th = 0.0
        self.pos_l = self.pos_r = 0.0
        self.last_tl = self.last_tr = None   # lay moc lai tu goi ke tiep
        self.write("Z\n")
        self.get_logger().info("Da dat lai odom ve goc")
        return res

    def handle_line(self, line: str) -> None:
        if not line.startswith("O "):
            self.get_logger().info(f"ESP32: {line}")
            return
        parts = line.split()
        if len(parts) != 6:
            return
        try:
            vl, vr = float(parts[2]), float(parts[3])
            tl, tr = int(parts[4]), int(parts[5])
        except ValueError:
            return
        self.n_odom += 1
        self.update_odom(vl, vr, tl, tr)

    def update_odom(self, vl: float, vr: float, tl: int, tr: int) -> None:
        now = self.get_clock().now()
        if self.last_tl is None:
            self.last_tl, self.last_tr, self.last_stamp = tl, tr, now
            self.get_logger().info(f"Moc xung ban dau: L={tl} R={tr}")
            return

        dt = (now - self.last_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        d_tl = tl - self.last_tl
        d_tr = tr - self.last_tr
        # Nguong co gian theo dt that -> rot goi khong bi bat nham,
        # vi ESP32 gui so dem TUYET DOI nen delta van dung sau khi rot.
        max_step = self.MAX_TICK_RATE * dt
        if abs(d_tl) > max_step or abs(d_tr) > max_step:
            # Chi xay ra khi ESP32 khoi dong lai: bo dem ve 0, delta vo nghia.
            self.get_logger().warn(
                f"Bo qua buoc nhay xung dL={d_tl} dR={d_tr} "
                f"(dt={dt:.3f}s, nguong {max_step:.0f}), lay moc lai")
            self.last_tl, self.last_tr, self.last_stamp = tl, tr, now
            return

        self.last_stamp = now
        dl = d_tl * self.m_per_tick
        dr = d_tr * self.m_per_tick
        self.last_tl, self.last_tr = tl, tr

        d = 0.5 * (dl + dr)
        dth = (dr - dl) / self.wheel_sep

        th_mid = self.th + 0.5 * dth              # tich phan dung diem giua
        self.x += d * math.cos(th_mid)
        self.y += d * math.sin(th_mid)
        self.th = math.atan2(math.sin(self.th + dth), math.cos(self.th + dth))

        self.vx = 0.5 * (vl + vr)
        self.wz = (vr - vl) / self.wheel_sep
        self.pos_l += dl / self.wheel_radius
        self.pos_r += dr / self.wheel_radius

        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.th)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        moving = abs(self.vx) > 1e-3 or abs(self.wz) > 1e-3
        pc = 0.05 if moving else 1e-4
        tc = 0.02 if moving else 1e-4
        odom.pose.covariance[0] = pc
        odom.pose.covariance[7] = pc
        odom.pose.covariance[35] = tc
        odom.twist.covariance[0] = tc
        odom.twist.covariance[35] = tc
        for i in (14, 21, 28):                    # z, roll, pitch: khoa cung
            odom.pose.covariance[i] = 1e6
            odom.twist.covariance[i] = 1e6
        self.odom_pub.publish(odom)

        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        js.name = [self.left_joint, self.right_joint]
        js.position = [self.pos_l, self.pos_r]
        js.velocity = [vl / self.wheel_radius, vr / self.wheel_radius]
        self.js_pub.publish(js)

    def stop_motors(self) -> None:
        try:
            self.write("S\n")
            if self.ser:
                self.ser.flush()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = Esp32BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()