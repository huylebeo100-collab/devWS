#!/usr/bin/env python3
"""mission_manager: QR -> station, multi-task A-B-C, dashboard interface (ROS 2 Humble).

Inputs
  <qr_topic>              std_msgs/String   QR text from amr_vision (via rosbridge)
  /mission/request        std_msgs/String   JSON or plain text from dashboard / CLI
  /mission/save_station   std_msgs/String   station name -> save current map pose to YAML
  /mission/cancel         std_srvs/Trigger  stop in place, drop queue
  /mission/home           std_srvs/Trigger  abort and return to home
  /mission/confirm        std_srvs/Trigger  continue a step with wait = "confirm"
  /mission/reload         std_srvs/Trigger  reload stations YAML (only when idle)
Outputs
  /mission/status         std_msgs/String   JSON, transient_local
Uses
  navigate_to_pose        nav2_msgs/action/NavigateToPose
  TF map -> base_link     to check "robot is at home" and to save stations
"""
import csv
import json
import math
import os
import re
import tempfile
import time
import uuid
from datetime import datetime, timezone

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from unique_identifier_msgs.msg import UUID

from amr_mission.mission_core import (
    DELIVERY_STATES, NAME_RE, MissionCore, MissionError, Pose2D, QrGate, State,
    build_mission, extract_pallet_ids, extract_qr_text, inbound_check, match_name,
    pallet_to_station, parse_stations, parse_text_request,
)

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MissionManager(Node):
    def __init__(self):
        super().__init__("mission_manager")
        p = self.declare_parameter
        self.stations_file = p("stations_file", "/root/amr_ws/mission/stations.yaml").value
        self.qr_topic = p("qr_topic", "/vision/qr").value
        self.qr_json_key = p("qr_json_key", "").value
        self.qr_prefix = p("qr_prefix", "").value
        self.qr_wait_s = float(p("qr_wait_s", 5.0).value)
        self.qr_rearm_s = float(p("qr_rearm_s", 5.0).value)
        self.qr_gap_s = float(p("qr_gap_s", 2.0).value)
        self.default_wait_s = float(p("default_wait_s", 5.0).value)
        self.home_radius = float(p("home_radius", 0.35).value)
        self.nav_retries = int(p("nav_retries", 1).value)
        self.dry_run = bool(p("dry_run", False).value)
        self.dry_run_nav_s = float(p("dry_run_nav_s", 3.0).value)
        self.map_frame = p("map_frame", "map").value
        self.base_frame = p("base_frame", "base_link").value
        nav_action = p("nav_action", "navigate_to_pose").value
        # Nhập kho: vision đọc mã pallet -> đi trạm theo mã ("" = tắt)
        self.inbound_topic = p("inbound_topic", "/amr_vision/inbound").value
        self.pallet_regex = re.compile(p("pallet_regex", r"PAL-(\d+)-([A-Za-z0-9_]+)").value)
        self.pallet_cooldown_s = float(p("pallet_cooldown_s", 60.0).value)
        self.inbound_max_age_s = float(p("inbound_max_age_s", 30.0).value)
        self._pallet_last = {}
        self._inbound_seen = []           # result_id đã xử lý (giữ 200 cái gần nhất)
        # /mission/request: "replace" = lệnh mới thay nhiệm vụ đang chạy, "queue" = xếp hàng
        self.request_default_mode = p("request_default_mode", "replace").value
        # trạng thái action navigate_to_pose: phát hiện goal của người khác (web, RViz)
        self.nav_status_topic = p("nav_status_topic", "/navigate_to_pose/_action/status").value
        self._our_goals = []              # UUID (bytes) của goal mình gửi, 20 cái gần nhất
        self._external_active = False
        self._external_clear_t = None
        self.resume_delay_s = float(p("resume_delay_s", 1.0).value)
        # Chế độ: AUTO = tự động nhập kho (nhận mã pallet/QR), MANUAL = ra lệnh từ web
        self.mode = "AUTO" if bool(p("default_auto", False).value) else "MANUAL"
        self.mode_pending = None
        # Đánh giá sai số tại trạm: "" = tắt
        self.eval_csv = p("eval_csv", "/root/amr_eval/station_errors.csv").value
        self.eval_delay_s = float(p("eval_delay_s", 1.0).value)
        self.session = time.strftime("%Y%m%d_%H%M%S")
        self._mission_no = 0
        self._mission_name = ""

        self.pallet_map = {}
        self.stations, self.home, self.presets = self._load_config()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, nav_action)
        self._goal_handle = None
        self._goal_seq = -1
        self._feedback = {}
        self._timers_once = []
        self._dead_timers = []

        self.core = MissionCore(
            self.stations, self.home, self._send_goal, self._cancel_goal,
            clock=time.monotonic, log=self.get_logger().info,
            on_event=self._on_event, nav_retries=self.nav_retries)
        self.gate = QrGate(gap_s=self.qr_gap_s)
        self._last_reject = ""
        self._last_status_json = ""
        self._last_status_t = 0.0
        self._last_status_key = None

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, "/mission/status", latched)
        self.pub_event = self.create_publisher(String, "/mission/event", 20)
        if self.nav_status_topic and not self.dry_run:
            self.create_subscription(GoalStatusArray, self.nav_status_topic, self._on_nav_status, 10)
        self.create_subscription(String, "/mission/request", self._on_request, 10)
        self.create_subscription(String, self.qr_topic, self._on_qr, 10)
        if self.inbound_topic:
            self.create_subscription(String, self.inbound_topic, self._on_inbound, 10)
        self.create_subscription(String, "/mission/save_station", self._on_save_station, 10)
        self.create_service(Trigger, "/mission/cancel", self._srv(self.core.cancel))
        self.create_service(Trigger, "/mission/home", self._srv(self.core.go_home))
        self.create_service(Trigger, "/mission/confirm", self._srv(self.core.confirm))
        self.create_service(Trigger, "/mission/reload", self._srv(self._reload))
        self.create_service(SetBool, "/mission/set_auto", self._srv_set_auto)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"mission_manager ready | stations={list(self.stations)} home={self.home} "
            f"missions={list(self.presets)} qr_topic={self.qr_topic} prefix='{self.qr_prefix}' "
            f"inbound_topic={self.inbound_topic or 'off'} pallets={self.pallet_map or 'theo chữ cuối'} "
            f"mode={self.mode} "
            f"dry_run={self.dry_run}")
        if self.dry_run:
            self.get_logger().warn("DRY RUN: không gửi goal cho Nav2, xe KHÔNG chạy")
        if self.eval_csv and not self.dry_run:
            self.get_logger().info(f"ghi sai số tại trạm -> {self.eval_csv} (session {self.session})")

    # ------------------------------------------------------------ config
    def _read_yaml(self):
        with open(self.stations_file, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    def _load_config(self):
        cfg = self._read_yaml()
        stations = parse_stations(cfg.get("stations"))
        home = str(cfg.get("home_station", "home"))
        if home not in stations:
            raise MissionError(f"home_station '{home}' không có trong stations")
        presets = {}
        for name, raw in (cfg.get("missions") or {}).items():
            try:
                build_mission(dict(raw), stations, {}, self.default_wait_s)
                presets[str(name)] = dict(raw)
            except (MissionError, TypeError) as e:
                self.get_logger().error(f"bỏ mission '{name}' trong YAML: {e}")
        pallet_map = {}
        for pid, st in (cfg.get("pallets") or {}).items():
            key = match_name(str(st), stations)
            if key is None:
                self.get_logger().error(f"bỏ pallet '{pid}': không có trạm '{st}'")
            else:
                pallet_map[str(pid)] = key
        self.pallet_map = pallet_map
        return stations, home, presets

    def _reload(self):
        if self.core.busy:
            return False, "đang chạy mission, không reload"
        try:
            stations, home, presets = self._load_config()
            self.core.update_stations(stations, home)
        except (OSError, MissionError, yaml.YAMLError) as e:
            return False, f"reload lỗi: {e}"
        self.stations, self.home, self.presets = stations, home, presets
        return True, f"stations={list(stations)} missions={list(presets)}"

    # ------------------------------------------------------------ helpers
    def _srv(self, fn):
        def cb(_req, res):
            ok, msg = fn()
            res.success, res.message = bool(ok), str(msg)
            (self.get_logger().info if ok else self.get_logger().warn)(f"service: {msg}")
            return res
        return cb

    def _once(self, delay, fn):
        """One-shot timer."""
        holder = {}

        def cb():
            t = holder["t"]
            t.cancel()
            if t in self._timers_once:
                self._timers_once.remove(t)
            self._dead_timers.append(t)   # destroyed later in _tick, not inside its own callback
            fn()
        holder["t"] = self.create_timer(delay, cb)
        self._timers_once.append(holder["t"])

    def _robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return None
        tr = t.transform.translation
        return Pose2D(tr.x, tr.y, quat_to_yaw(t.transform.rotation))

    def _dist_to_home(self):
        pose = self._robot_pose()
        if pose is None:
            return None
        h = self.stations[self.home]
        return math.hypot(pose.x - h.x, pose.y - h.y)

    def _reject(self, msg, **data):
        self._last_reject = msg
        self.get_logger().warn(msg)
        self._pub_event("rejected", reason=msg, **data)

    # ------------------------------------------------------------ nav port
    def _send_goal(self, seq, station, pose):
        self._feedback = {}
        self.get_logger().info(
            f"[nav #{seq}] -> {station} x={pose.x:.2f} y={pose.y:.2f} "
            f"yaw={math.degrees(pose.yaw):.0f}deg")
        if self.dry_run:
            self._once(self.dry_run_nav_s, lambda: self.core.on_nav_result(seq, True, "dry_run"))
            return
        if not self.nav.server_is_ready():
            self._once(1.0, lambda: self.core.on_nav_result(seq, False, "Nav2 chưa sẵn sàng"))
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        q = yaw_to_quat(pose.yaw)
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        gid = uuid.uuid4().bytes
        self._our_goals = (self._our_goals + [gid])[-20:]
        fut = self.nav.send_goal_async(goal, feedback_callback=lambda fb: self._on_feedback(seq, fb),
                                       goal_uuid=UUID(uuid=list(gid)))
        fut.add_done_callback(lambda f: self._on_goal_response(seq, f))

    def _on_goal_response(self, seq, fut):
        try:
            gh = fut.result()
        except Exception as e:  # noqa: BLE001
            self.core.on_nav_result(seq, False, f"send_goal lỗi: {e}")
            return
        if not gh.accepted:
            self.core.on_nav_result(seq, False, "Nav2 từ chối goal")
            return
        if seq != self.core.nav_seq:      # cancelled while waiting for acceptance
            gh.cancel_goal_async()
            return
        self._goal_handle, self._goal_seq = gh, seq
        gh.get_result_async().add_done_callback(lambda f: self._on_result(seq, f))

    def _on_result(self, seq, fut):
        if self._goal_seq == seq:
            self._goal_handle = None
        try:
            status = fut.result().status
        except Exception as e:  # noqa: BLE001
            self.core.on_nav_result(seq, False, f"get_result lỗi: {e}")
            return
        self.core.on_nav_result(seq, status == GoalStatus.STATUS_SUCCEEDED,
                                STATUS_NAMES.get(status, f"status={status}"))

    def _on_feedback(self, seq, fb):
        if seq == self.core.nav_seq:
            f = fb.feedback
            self._feedback = {"distance_remaining": round(f.distance_remaining, 2),
                              "recoveries": f.number_of_recoveries}

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info("hủy goal Nav2 hiện tại")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        if self.dry_run:
            for t in list(self._timers_once):
                t.cancel()
                self.destroy_timer(t)
            self._timers_once.clear()

    # ------------------------------------------------------------ mode
    def _apply_mode(self, mode, reason):
        self.mode_pending = None
        if mode == self.mode:
            return
        self.mode = mode
        self.get_logger().info(f"chế độ -> {mode} ({reason})")
        self._pub_event("mode_changed", mode=mode, reason=reason)
        if mode == "AUTO":
            d = self._dist_to_home()
            if self.core.state in (State.STOPPED, State.ERROR) or d is None or \
                    (self.home_radius > 0 and d > self.home_radius):
                self.core.go_home()

    def _srv_set_auto(self, req, res):
        want = "AUTO" if req.data else "MANUAL"
        name = "Tự động nhập kho" if want == "AUTO" else "Ra lệnh"
        if want == self.mode:
            if self.mode_pending is not None:
                self.mode_pending = None
                self._pub_event("mode_pending", mode=None)
                res.success, res.message = True, f"hủy chờ chuyển chế độ, giữ {name}"
            else:
                res.success, res.message = True, f"đang ở chế độ {name}"
        elif self.core.busy or self.core.queue:
            self.mode_pending = want
            self._pub_event("mode_pending", mode=want)
            res.success = True
            res.message = (f"sẽ chuyển sang {name} sau khi robot xong nhiệm vụ và về home" if want == "MANUAL"
                           else f"sẽ chuyển sang {name} sau khi xong lộ trình hiện tại")
        else:
            self._apply_mode(want, "người vận hành")
            res.success, res.message = True, f"đã chuyển sang chế độ {name}"
        self.get_logger().info(f"set_auto: {res.message}")
        return res

    def _on_nav_status(self, msg):
        active = [st for st in msg.status_list
                  if st.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)]
        external = [st for st in active if bytes(st.goal_info.goal_id.uuid) not in self._our_goals]
        self._external_active = bool(external)
        if not external:
            return
        self._external_clear_t = None
        reason = "có goal Nav2 từ bên ngoài (web/RViz)"
        if self.mode == "AUTO":
            if self.core.state in DELIVERY_STATES:
                # đang giao pallet: nhường tạm, goal ngoài xong thì làm tiếp
                self._goal_handle = None
                self.core.pause_external(reason)
            elif self.core.state != State.PAUSED:
                # đang về home hoặc đứng chờ: chuyển sang Ra lệnh
                if self.core.busy or self.core.queue:
                    self._goal_handle = None
                    self.core.external_override(reason + " - nhường quyền")
                self._apply_mode("MANUAL", reason)
        elif self.core.busy or self.core.queue:
            self._goal_handle = None
            self.core.external_override(reason + " - nhường quyền, dừng nhiệm vụ")

    # ------------------------------------------------------------ events
    def _pub_event(self, name, **data):
        ev = {"time": datetime.now(timezone.utc).isoformat(timespec="seconds"), "event": name}
        ev.update({k: v for k, v in data.items() if v is not None})
        self.pub_event.publish(String(data=json.dumps(ev, ensure_ascii=False)))

    def _on_event(self, name, data):
        self._pub_event(name, **data)
        if name == "mission_started":
            self._mission_no += 1
            self._mission_name = data.get("mission", "")
        elif name == "arrived":
            step = self.core.mission.steps[self.core.step_idx]
            # xe còn đứng ở trạm ít nhất eval_delay_s thì đợi cho xe dừng hẳn, không thì lấy ngay
            stays = step.wait < 0 or step.wait >= self.eval_delay_s + 0.2
            self._schedule_eval(data["station"], self.eval_delay_s if stays else 0.0)
        elif name == "returned_home":
            self.gate.block(self.qr_rearm_s)
            # còn mission trong hàng đợi thì xe đi ngay -> lấy ngay
            self._schedule_eval(self.home, 0.0 if self.core.queue else self.eval_delay_s)

    # ------------------------------------------------------------ eval
    EVAL_FIELDS = ["time", "session", "mission_no", "mission", "station",
                   "x_goal", "y_goal", "yaw_goal_deg", "x", "y", "yaw_deg",
                   "e_long_cm", "e_lat_cm", "e_xy_cm", "e_yaw_deg"]

    def _schedule_eval(self, station, delay):
        if not self.eval_csv or self.dry_run:
            return
        no, name = self._mission_no, self._mission_name
        if delay <= 0:
            self._record_eval(station, no, name)
        else:
            self._once(delay, lambda: self._record_eval(station, no, name))

    def _record_eval(self, station, mission_no, mission):
        pose = self._robot_pose()
        if pose is None:
            self.get_logger().warn(f"[eval] bỏ qua {station}: không có TF {self.map_frame}->{self.base_frame}")
            return
        g = self.stations[station]
        dx, dy = pose.x - g.x, pose.y - g.y
        c, s_ = math.cos(g.yaw), math.sin(g.yaw)
        e_long = c * dx + s_ * dy           # + : vượt quá trạm theo hướng trạm
        e_lat = -s_ * dx + c * dy           # + : lệch sang trái
        e_yaw = math.degrees(math.atan2(math.sin(pose.yaw - g.yaw), math.cos(pose.yaw - g.yaw)))
        row = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "session": self.session,
            "mission_no": mission_no, "mission": mission, "station": station,
            "x_goal": f"{g.x:.3f}", "y_goal": f"{g.y:.3f}", "yaw_goal_deg": f"{math.degrees(g.yaw):.1f}",
            "x": f"{pose.x:.3f}", "y": f"{pose.y:.3f}", "yaw_deg": f"{math.degrees(pose.yaw):.1f}",
            "e_long_cm": f"{e_long * 100:.1f}", "e_lat_cm": f"{e_lat * 100:.1f}",
            "e_xy_cm": f"{math.hypot(dx, dy) * 100:.1f}", "e_yaw_deg": f"{e_yaw:.1f}",
        }
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.eval_csv)), exist_ok=True)
            new = not os.path.exists(self.eval_csv) or os.path.getsize(self.eval_csv) == 0
            with open(self.eval_csv, "a", newline="", encoding="utf-8") as f:
                w = csv.DictWriter(f, fieldnames=self.EVAL_FIELDS)
                if new:
                    w.writeheader()
                w.writerow(row)
        except OSError as e:
            self.get_logger().error(f"[eval] ghi CSV lỗi: {e}")
            return
        self.get_logger().info(
            f"[eval] {station}: e_xy={row['e_xy_cm']} cm (dọc {row['e_long_cm']}, ngang {row['e_lat_cm']}) "
            f"e_yaw={row['e_yaw_deg']} deg")

    def _submit(self, req, source, replace=False):
        try:
            m = build_mission(req, self.stations, self.presets, self.default_wait_s, source)
        except MissionError as e:
            self._reject(f"request bị từ chối: {e}", mission_id=str(req.get("id", "")) or None)
            return
        ok, msg = self.core.submit(m, replace=replace)
        if ok:
            self._last_reject = ""
            self.get_logger().info(msg)
        else:
            self._reject(msg, mission_id=m.mid or None)

    def _on_request(self, msg):
        text = msg.data.strip()
        try:
            req = json.loads(text)
        except ValueError:
            req = parse_text_request(text, "", self.stations, self.presets)
            if req is None:
                self._reject(f"request không hiểu: {text!r}")
                return
        if isinstance(req, str):
            req = parse_text_request(req, "", self.stations, self.presets) or {"name": req}
        if not isinstance(req, dict):
            self._reject(f"request phải là JSON object: {text!r}")
            return
        if self.mode == "AUTO":
            self._reject("đang ở chế độ Tự động nhập kho: chuyển sang Ra lệnh trước "
                         "(robot đổi chế độ sau khi xong nhiệm vụ và về home)",
                         mission_id=str(req.get("id", "")) or None)
            return
        mode = str(req.get("mode", self.request_default_mode))
        self._submit(req, source=str(req.get("source", "web")), replace=(mode == "replace"))

    def _qr_blocker(self):
        if self.mode != "AUTO":
            return "chế độ Ra lệnh: không nhận mã"
        if self.mode_pending == "MANUAL":
            return "đang chờ về home để chuyển sang Ra lệnh"
        if self.core.busy or self.core.queue:
            return f"robot đang bận ({self.core.state.value})"
        if self.core.state == State.ERROR:
            return "đang ERROR, gọi /mission/home hoặc /mission/cancel trước"
        if self._external_active:
            return "Nav2 đang chạy goal từ bên ngoài (web/RViz)"
        if self.dry_run or self.home_radius <= 0:
            return ""
        d = self._dist_to_home()
        if d is None:
            return f"chưa có TF {self.map_frame}->{self.base_frame} (chưa định vị AMCL?)"
        if d > self.home_radius:
            return f"xe không ở vị trí chờ (cách {d:.2f} m > {self.home_radius} m)"
        return ""

    def _on_qr(self, msg):
        text = extract_qr_text(msg.data, self.qr_json_key)
        if not text or not self.gate.observe(text):
            return
        req = parse_text_request(text, self.qr_prefix, self.stations, self.presets)
        if req is None:
            self.get_logger().info(f"QR bỏ qua (không phải mã điểm đến): {text!r}")
            return
        why = self._qr_blocker()
        if why:
            self._reject(f"QR {text!r} bị bỏ qua: {why}")
            return
        if "steps" in req:
            req["wait"] = self.qr_wait_s
            req["return_home"] = True
        self.get_logger().info(f"QR nhận: {text!r} -> {req}")
        self._submit(req, source=f"qr:{text}")

    def _on_inbound(self, msg):
        try:
            obj = json.loads(msg.data)
        except ValueError:
            obj = msg.data
        ok, why, rid = inbound_check(obj, datetime.now(timezone.utc), self.inbound_max_age_s,
                                     self._inbound_seen)
        if not ok:
            self.get_logger().info(f"inbound bỏ qua: {why}")
            return
        if rid:
            self._inbound_seen = (self._inbound_seen + [rid])[-200:]
        pids = extract_pallet_ids(msg.data, self.pallet_regex)
        if not pids:
            return
        pid = pids[0]
        now = time.monotonic()
        last = self._pallet_last.get(pid)
        if last is not None and now - last < self.pallet_cooldown_s:
            self.get_logger().info(
                f"pallet {pid} bỏ qua: vừa xử lý {now - last:.0f} s trước (cooldown {self.pallet_cooldown_s:.0f} s)")
            return
        station = pallet_to_station(pid, self.pallet_map, self.pallet_regex, self.stations)
        if station is None:
            self._reject(f"pallet {pid}: không suy ra được trạm (thêm vào mục pallets trong stations.yaml)",
                         pallet=pid, result_id=rid or None)
            return
        why = self._qr_blocker()
        if why:
            self._reject(f"pallet {pid} bị bỏ qua: {why}", pallet=pid, result_id=rid or None)
            return
        self._pallet_last[pid] = now
        self.get_logger().info(f"nhập kho: pallet {pid} -> trạm {station}")
        self._pub_event("pallet_dispatch", pallet=pid, station=station, result_id=rid or None)
        self._submit({"steps": [station], "wait": self.qr_wait_s, "return_home": True,
                      "id": f"pallet:{pid}"}, source=f"pallet:{pid}")

    def _on_save_station(self, msg):
        name = msg.data.strip()
        if not NAME_RE.match(name):
            self._reject(f"tên trạm '{name}' chỉ dùng chữ, số, '_'", station=name)
            return
        if self.core.busy:
            self._reject("không lưu trạm khi đang chạy mission", station=name)
            return
        pose = self._robot_pose()
        if pose is None:
            self._reject(f"không lấy được TF {self.map_frame}->{self.base_frame}", station=name)
            return
        try:
            cfg = self._read_yaml()
            cfg.setdefault("stations", {})[name] = {
                "x": round(pose.x, 3), "y": round(pose.y, 3), "yaw": round(pose.yaw, 3)}
            d = os.path.dirname(os.path.abspath(self.stations_file))
            fd, tmp = tempfile.mkstemp(dir=d, suffix=".tmp")
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)
            os.replace(tmp, self.stations_file)
        except (OSError, yaml.YAMLError) as e:
            self._reject(f"ghi file lỗi: {e}", station=name)
            return
        ok, info = self._reload()
        self._pub_event("station_saved", station=name, x=round(pose.x, 3), y=round(pose.y, 3),
                        yaw=round(pose.yaw, 3))
        self.get_logger().info(
            f"đã lưu trạm '{name}': x={pose.x:.3f} y={pose.y:.3f} "
            f"yaw={math.degrees(pose.yaw):.1f}deg | reload: {info}")

    # ------------------------------------------------------------ loop
    def _tick(self):
        while self._dead_timers:
            self.destroy_timer(self._dead_timers.pop())
        self.core.tick()
        if self.mode_pending is not None and not self.core.busy and not self.core.queue:
            self._apply_mode(self.mode_pending, "đã xong nhiệm vụ")
        if self.core.state == State.PAUSED and not self._external_active:
            if self._external_clear_t is None:
                self._external_clear_t = time.monotonic()
            elif time.monotonic() - self._external_clear_t >= self.resume_delay_s:
                self._external_clear_t = None
                self.core.resume()
        now = time.monotonic()
        st = self.core.status()
        st["dry_run"] = self.dry_run
        st["mode"] = self.mode
        st["mode_pending"] = self.mode_pending
        st["last_reject"] = self._last_reject
        st["stations"] = list(self.stations)
        st["missions"] = list(self.presets)
        st["home_station"] = self.home
        st["station_poses"] = {k: {"x": v.x, "y": v.y, "yaw": v.yaw} for k, v in self.stations.items()}
        d = self._dist_to_home()
        st["dist_home"] = round(d, 2) if d is not None else None
        st["at_home"] = d is not None and (self.home_radius <= 0 or d <= self.home_radius)
        st["external_goal_active"] = self._external_active
        st["ready_for_scan"] = (not self._qr_blocker()) and self.core.state in (State.IDLE, State.STOPPED)
        if st["state"] in (State.NAVIGATING.value, State.RETURNING.value):
            st.update(self._feedback)
        js = json.dumps(st, ensure_ascii=False, sort_keys=True)
        key = (st["state"], st["target"], st["mission_id"], st["last_reject"], st["ready_for_scan"],
               st["mode"], st["mode_pending"], tuple(st["stations"]))
        changed = js != self._last_status_json
        # tối đa 2 Hz (web dùng useRosTopic), đổi trạng thái thì gửi ngay, im lặng thì 2 s gửi 1 lần
        if (changed and (key != self._last_status_key or now - self._last_status_t >= 0.5)) \
                or now - self._last_status_t > 2.0:
            self.pub_status.publish(String(data=js))
            self._last_status_json, self._last_status_t, self._last_status_key = js, now, key


def main():
    rclpy.init()
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Nav2 keeps driving the last goal if this node dies -> try to cancel it.
        try:
            if node.core.busy and rclpy.ok():
                node._cancel_goal()
                rclpy.spin_once(node, timeout_sec=0.5)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()