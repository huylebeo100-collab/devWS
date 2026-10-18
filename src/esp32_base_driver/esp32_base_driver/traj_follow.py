#!/usr/bin/env python3
# traj_follow.py - chay quy dao tron/elip, publish ref vs actual, ghi CSV
import math, os, csv, sys, select, datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path

L = 0.3890  # wheel_separation (m)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


class TrajFollow(Node):
    def __init__(self):
        super().__init__('traj_follow')
        d = self.declare_parameter
        self.shape = d('shape', 'circle').value       # circle | ellipse
        self.a = d('a', 0.5).value                    # ban truc theo huong xuat phat (circle: R)
        self.b = d('b', 0.5).value                    # ban truc ngang
        if self.shape == 'circle':
            self.b = self.a
        self.v = d('v', 0.15).value                   # toc do dai (m/s)
        self.laps = d('laps', 1.0).value
        self.s = 1.0 if d('dir', 1).value >= 0 else -1.0   # 1 re trai, -1 re phai
        self.closed = d('closed_loop', True).value
        self.wait = d('wait', True).value             # true: ve ref, nhan Enter moi chay
        self.kx = d('kx', 1.5).value
        self.ky = d('ky', 20.0).value
        self.kth = d('kth', 6.0).value
        self.ramp = d('ramp', 1.0).value
        self.v_max = d('v_max', 0.35).value
        self.w_max = d('w_max', 1.2).value
        log_dir = os.path.expanduser(d('log_dir', '~/traj_logs').value)

        kmax = max(self.a/self.b**2, self.b/self.a**2)
        v_in = self.v*(1 - kmax*L/2)
        self.get_logger().info(f'{self.shape} a={self.a} b={self.b} v={self.v} '
                               f'closed={self.closed} | banh trong cham nhat {v_in:.3f} m/s')
        if v_in < 0.08:
            self.get_logger().warn('banh trong < 0.08 m/s, de ket deadband -> tang v hoac a,b')

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_ref = self.create_publisher(Path, 'traj/ref', 10)
        self.pub_act = self.create_publisher(Path, 'traj/actual', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)

        self.odom = None
        self.start = None
        self.go = not self.wait
        self.running = False
        self.prompted = False
        self.phi = 0.0
        self.done = False
        self.k = 0
        self.ref_path = Path(); self.ref_path.header.frame_id = 'odom'
        self.act_path = Path(); self.act_path.header.frame_id = 'odom'

        os.makedirs(log_dir, exist_ok=True)
        st = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        mode = 'cl' if self.closed else 'ol'
        self.csv_path = os.path.join(log_dir, f'traj_{self.shape}_{mode}_{st}.csv')
        self.f = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(['t', 'xr', 'yr', 'thr', 'vr', 'wr', 'x', 'y', 'th',
                              'ex', 'ey', 'eth', 'v_cmd', 'w_cmd', 'v_meas', 'w_meas'])

        self.create_timer(0.05, self.loop)
        self.create_timer(1.0, self.republish_ref)

    # ---------- tien ich ----------
    def on_odom(self, msg):
        self.odom = msg

    def cur(self):
        o = self.odom.pose.pose
        return o.position.x, o.position.y, yaw_of(o.orientation)

    def ref_local(self, phi):
        a, b, s = self.a, self.b, self.s
        x = a*math.sin(phi)
        y = s*b*(1 - math.cos(phi))
        th = math.atan2(s*b*math.sin(phi), a*math.cos(phi))
        g = math.hypot(a*math.cos(phi), b*math.sin(phi))   # |dP/dphi|
        kappa = s*a*b/g**3
        return x, y, th, g, kappa

    def to_odom(self, x, y, th):
        x0, y0, t0 = self.start
        c, sn = math.cos(t0), math.sin(t0)
        return x0 + c*x - sn*y, y0 + sn*x + c*y, wrap(t0 + th)

    def pose(self, x, y, th):
        p = PoseStamped(); p.header.frame_id = 'odom'
        p.pose.position.x = x; p.pose.position.y = y
        p.pose.orientation.z = math.sin(th/2); p.pose.orientation.w = math.cos(th/2)
        return p

    def set_start(self, x, y, th):
        self.start = (x, y, th)
        self.ref_path.poses = []
        n = int(200*self.laps) + 1
        for i in range(n):
            xl, yl, tl, _, _ = self.ref_local(2*math.pi*self.laps*i/(n - 1))
            self.ref_path.poses.append(self.pose(*self.to_odom(xl, yl, tl)))
        self.pub_ref.publish(self.ref_path)

    def republish_ref(self):
        if self.start:
            self.pub_ref.publish(self.ref_path)

    def stop(self):
        try:
            self.pub.publish(Twist())
        except Exception:
            pass

    def in_start_frame(self):
        x, y, th = self.cur()
        x0, y0, t0 = self.start
        dx, dy = x - x0, y - y0
        return (math.cos(t0)*dx + math.sin(t0)*dy,
                -math.sin(t0)*dx + math.cos(t0)*dy, wrap(th - t0))

    # ---------- vong lap 20 Hz ----------
    def loop(self):
        if self.done or self.odom is None:
            return
        x, y, th = self.cur()
        now = self.get_clock().now().nanoseconds*1e-9

        if not self.go:                               # dang cho Enter
            self.k += 1
            if self.k % 10 == 1:
                self.set_start(x, y, th)              # ref di theo xe cho den khi bam Enter
            if not self.prompted:
                self.get_logger().info('Kiem tra duong DO tren RViz. Enter = chay, Ctrl+C = huy')
                self.prompted = True
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                self.go = True
            return

        if not self.running:                          # khoa diem xuat phat
            self.set_start(x, y, th)
            self.t0 = now; self.t_prev = now
            self.running = True; self.k = 0
            self.get_logger().info(f'CHAY. log: {self.csv_path}')

        t = now - self.t0
        dt = now - self.t_prev; self.t_prev = now

        if self.phi >= 2*math.pi*self.laps:
            self.stop(); self.done = True; self.f.close()
            self.get_logger().info('XONG quy dao')
            return

        vr = self.v*min(1.0, t/self.ramp) if self.ramp > 0 else self.v
        xl, yl, tl, g, kappa = self.ref_local(self.phi)
        self.phi += vr*dt/g
        xr, yr, thr = self.to_odom(xl, yl, tl)
        wr = vr*kappa

        dx, dy = xr - x, yr - y
        ex = math.cos(th)*dx + math.sin(th)*dy
        ey = -math.sin(th)*dx + math.cos(th)*dy
        eth = wrap(thr - th)
        if self.closed:
            v = vr*math.cos(eth) + self.kx*ex
            w = wr + vr*(self.ky*ey + self.kth*math.sin(eth))
        else:
            v, w = vr, wr
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))
        cmd = Twist(); cmd.linear.x = v; cmd.angular.z = w
        self.pub.publish(cmd)

        tw = self.odom.twist.twist
        self.writer.writerow([f'{t:.3f}', f'{xr:.4f}', f'{yr:.4f}', f'{thr:.4f}',
                              f'{vr:.4f}', f'{wr:.4f}', f'{x:.4f}', f'{y:.4f}', f'{th:.4f}',
                              f'{ex:.4f}', f'{ey:.4f}', f'{eth:.4f}', f'{v:.4f}', f'{w:.4f}',
                              f'{tw.linear.x:.4f}', f'{tw.angular.z:.4f}'])
        self.k += 1
        if self.k % 2 == 0:
            self.act_path.poses.append(self.pose(x, y, th))
            self.pub_act.publish(self.act_path)


def main():
    rclpy.init()
    n = TrajFollow()
    try:
        while rclpy.ok() and not n.done:
            rclpy.spin_once(n, timeout_sec=0.1)
        end = n.get_clock().now().nanoseconds*1e-9 + 1.5     # cho xe dung han
        while rclpy.ok() and n.get_clock().now().nanoseconds*1e-9 < end:
            n.stop(); rclpy.spin_once(n, timeout_sec=0.1)
        if n.running:
            xl, yl, tl = n.in_start_frame()
            n.get_logger().info(f'Vi tri cuoi so voi diem xuat phat (odom): x={xl*1000:.0f} mm '
                                f'y={yl*1000:.0f} mm yaw={math.degrees(tl):.1f} deg')
    except KeyboardInterrupt:
        pass
    finally:
        n.stop()
        if not n.f.closed:
            n.f.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()