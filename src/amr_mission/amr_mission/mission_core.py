"""Mission state machine for the AMR, independent of ROS so it can be tested with pytest.

The ROS node (mission_manager.py) only provides:
  - send_goal(seq, station, pose) / cancel_goal()  -> Nav2 NavigateToPose
  - on_nav_result(seq, ok, reason) when Nav2 returns a result
  - tick() called periodically to handle dwell timers
"""
from __future__ import annotations

import json
import re
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Deque, Dict, List, Optional, Tuple

WAIT_CONFIRM = -1.0          # wait < 0  -> wait for /mission/confirm
NAME_RE = re.compile(r"^[A-Za-z0-9_]+$")   # no '-' : it separates "A-B-C" in QR codes
SPLIT_RE = re.compile(r"[\s,;>\-]+")


class State(str, Enum):
    IDLE = "IDLE"                  # free (usually at home)
    NAVIGATING = "NAVIGATING"      # driving to a station of the mission
    DWELLING = "DWELLING"          # waiting N s at a station
    WAIT_CONFIRM = "WAIT_CONFIRM"  # waiting for operator confirm
    RETURNING = "RETURNING"        # driving back to home
    PAUSED = "PAUSED"              # nhường goal ngoài giữa chuyến giao (AUTO), sẽ làm tiếp
    STOPPED = "STOPPED"            # cancelled by operator, robot stays in place
    ERROR = "ERROR"                # nav failed after retries, robot stays in place


BUSY_STATES = (State.NAVIGATING, State.DWELLING, State.WAIT_CONFIRM, State.RETURNING, State.PAUSED)
DELIVERY_STATES = (State.NAVIGATING, State.DWELLING, State.WAIT_CONFIRM)


class MissionError(ValueError):
    pass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float  # rad, map frame


@dataclass(frozen=True)
class Step:
    station: str
    wait: float = 0.0  # s ; <0 = wait for confirm


@dataclass
class Mission:
    name: str
    steps: List[Step]
    return_home: bool = True
    repeat: int = 1
    source: str = "local"
    mid: str = ""        # id do bên gửi đặt (web), trả lại trong status/event


# ---------------------------------------------------------------- parsing

def match_name(name: str, table: dict) -> Optional[str]:
    """Case-insensitive lookup, returns the canonical key or None."""
    if not isinstance(name, str):
        return None
    if name in table:
        return name
    low = name.strip().lower()
    for k in table:
        if k.lower() == low:
            return k
    return None


def parse_stations(raw) -> Dict[str, Pose2D]:
    if not isinstance(raw, dict) or not raw:
        raise MissionError("stations: trong file YAML phải có ít nhất 1 trạm")
    out = {}
    for name, v in raw.items():
        name = str(name)
        if not NAME_RE.match(name):
            raise MissionError(f"tên trạm '{name}' chỉ được dùng chữ, số, '_' (không dùng '-')")
        try:
            out[name] = Pose2D(float(v["x"]), float(v["y"]), float(v.get("yaw", 0.0)))
        except (KeyError, TypeError, ValueError) as e:
            raise MissionError(f"trạm '{name}' sai định dạng, cần {{x, y, yaw}}: {e}") from e
    return out


def parse_wait(v) -> float:
    if isinstance(v, str) and v.strip().lower() == "confirm":
        return WAIT_CONFIRM
    w = float(v)
    if w > 3600:
        raise MissionError(f"wait {w}s quá lớn (tối đa 3600)")
    return w


def build_mission(req: dict, stations: Dict[str, Pose2D], presets: dict,
                  default_wait: float, source: str = "local") -> Mission:
    """Build a Mission from a request dict.

    Accepted forms:
      {"name": "abc_loop"}                       preset from YAML (other keys override it)
      {"goto": "B", "wait": 5}                   single station
      {"steps": ["A", "B", "C"], "wait": 5, "repeat": 2, "return_home": true}
      {"steps": [{"station": "A", "wait": 5}, {"station": "B", "wait": "confirm"}]}
    """
    if not isinstance(req, dict):
        raise MissionError("request phải là JSON object")
    req = dict(req)
    name = req.get("name")
    preset_key = match_name(name, presets) if name is not None else None

    if preset_key is not None:
        base = dict(presets[preset_key])
        base.update({k: v for k, v in req.items() if k != "name"})
        name = preset_key
    elif "goto" in req:
        base = dict(req)
        base["steps"] = [req["goto"]]
        name = name or f"goto_{req['goto']}"
    elif "steps" in req:
        base = req
        name = name or "custom"
    elif name is not None:
        raise MissionError(f"không có mission tên '{name}' (có: {', '.join(presets) or 'trống'})")
    else:
        raise MissionError("request cần 'name', 'goto' hoặc 'steps'")

    try:
        wait_default = parse_wait(base.get("wait", default_wait))
    except (TypeError, ValueError) as e:
        raise MissionError(f"wait sai: {e}") from e

    raw_steps = base.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise MissionError("steps phải là danh sách không rỗng")
    if len(raw_steps) > 50:
        raise MissionError("tối đa 50 điểm mỗi mission")

    steps: List[Step] = []
    for s in raw_steps:
        if isinstance(s, str):
            st, w = s, wait_default
        elif isinstance(s, dict) and "station" in s:
            st = s["station"]
            try:
                w = parse_wait(s.get("wait", wait_default))
            except (TypeError, ValueError) as e:
                raise MissionError(f"wait của '{st}' sai: {e}") from e
        else:
            raise MissionError(f"step sai định dạng: {s!r}")
        key = match_name(str(st), stations)
        if key is None:
            raise MissionError(f"không có trạm '{st}' (có: {', '.join(stations)})")
        steps.append(Step(key, w))

    try:
        repeat = int(base.get("repeat", 1))
    except (TypeError, ValueError) as e:
        raise MissionError(f"repeat sai: {e}") from e
    if not 1 <= repeat <= 100:
        raise MissionError("repeat phải trong khoảng 1..100")

    return Mission(name=str(name), steps=steps,
                   return_home=bool(base.get("return_home", True)),
                   repeat=repeat, source=source, mid=str(req.get("id", ""))[:64])


def parse_text_request(text: str, prefix: str, stations: dict, presets: dict) -> Optional[dict]:
    """Turn a QR / plain-text payload into a request dict, or None if it is not a destination code.

    "A"          -> {"steps": ["A"]}
    "A-B-C"      -> {"steps": ["A", "B", "C"]}
    "abc_loop"   -> {"name": "abc_loop"}   (preset)
    With prefix "AMR:", only "AMR:..." payloads are accepted (pallet QRs are ignored).
    """
    t = (text or "").strip()
    if prefix:
        if not t.upper().startswith(prefix.upper()):
            return None
        t = t[len(prefix):].strip()
    if not t:
        return None
    key = match_name(t, presets)
    if key is not None:
        return {"name": key}
    key = match_name(t, stations)
    if key is not None:
        return {"steps": [key]}
    parts = [p for p in SPLIT_RE.split(t) if p]
    names = [match_name(p, stations) for p in parts]
    if len(parts) < 2 or any(n is None for n in names):
        return None
    return {"steps": names}


def extract_qr_text(data: str, json_key: str = "") -> str:
    """Get the QR string out of a std_msgs/String payload (raw text or JSON)."""
    s = (data or "").strip()
    if not s:
        return ""
    if not json_key and not s.startswith(("{", '"')):
        return s
    try:
        obj = json.loads(s)
    except ValueError:
        return "" if json_key else s
    if isinstance(obj, str):
        return obj.strip()
    if not isinstance(obj, dict):
        return ""
    if json_key:
        cur = obj
        for part in json_key.split("."):
            if not isinstance(cur, dict) or part not in cur:
                return ""
            cur = cur[part]
        return str(cur).strip() if isinstance(cur, (str, int, float)) else ""
    for k in ("text", "qr", "data", "value", "payload", "content", "code"):
        if isinstance(obj.get(k), str):
            return obj[k].strip()
    return ""


# ---------------------------------------------------------------- pallet -> station

def find_matching_strings(obj, regex) -> List[str]:
    """Mọi chuỗi trong JSON (lồng nhau tùy ý) khớp regex, theo thứ tự gặp."""
    out: List[str] = []
    if isinstance(obj, str):
        if regex.fullmatch(obj.strip()):
            out.append(obj.strip())
    elif isinstance(obj, dict):
        for v in obj.values():
            out.extend(find_matching_strings(v, regex))
    elif isinstance(obj, (list, tuple)):
        for v in obj:
            out.extend(find_matching_strings(v, regex))
    return out


def extract_pallet_ids(data: str, regex) -> List[str]:
    """Lấy mã pallet từ payload /amr_vision/inbound (JSON bất kỳ hoặc text thô)."""
    s = (data or "").strip()
    try:
        obj = json.loads(s)
    except ValueError:
        obj = s
    seen, out = set(), []
    for pid in find_matching_strings(obj, regex):
        if pid not in seen:
            seen.add(pid)
            out.append(pid)
    return out


def inbound_check(obj, now_utc, max_age_s: float, seen_ids) -> Tuple[bool, str, str]:
    """Lọc 1 message /amr_vision/inbound (schema amr_vision/inbound/v1).

    Trả về (nhận?, lý do nếu bỏ, result_id). Topic này latched và vision gửi lại
    kết quả cuối mỗi lần kết nối lại, nên phải bỏ: event khác PALLET_COUNTED,
    qr_status khác CONFIRMED, result_id đã xử lý, committed_at quá cũ.
    Trường nào không có trong message thì không kiểm tra trường đó.
    """
    from datetime import datetime, timezone
    if not isinstance(obj, dict):
        return True, "", ""
    ev = obj.get("event")
    if ev is not None and ev != "PALLET_COUNTED":
        return False, f"event={ev}", ""
    qs = obj.get("qr_status")
    if qs is not None and qs != "CONFIRMED":
        return False, f"qr_status={qs}", ""
    rid = str(obj.get("result_id") or "")
    if rid and rid in seen_ids:
        return False, f"result_id {rid} đã xử lý", rid
    ts = obj.get("committed_at")
    if isinstance(ts, str) and max_age_s > 0:
        try:
            t = datetime.fromisoformat(ts.replace("Z", "+00:00"))
            if t.tzinfo is None:
                t = t.replace(tzinfo=timezone.utc)
            age = (now_utc - t).total_seconds()
            if age > max_age_s:
                return False, f"kết quả cũ {age:.0f} s (latched/gửi lại)", rid
        except ValueError:
            pass
    return True, "", rid


def pallet_to_station(pid: str, pallet_map: dict, regex, stations: dict) -> Optional[str]:
    """Bảng `pallets` trong YAML ưu tiên; không có thì lấy nhóm cuối của regex (PAL-001-B -> B)."""
    key = match_name(pid, pallet_map)
    if key is not None:
        return match_name(str(pallet_map[key]), stations)
    m = regex.fullmatch(pid)
    if m and m.groups():
        return match_name(m.groups()[-1], stations)
    return None


# ---------------------------------------------------------------- QR gate

class QrGate:
    """Accept a QR only when it newly appears.

    A QR seen continuously (camera publishing every frame, or a QR fixed at the
    home position) must disappear for `gap_s` before it can trigger again.
    `block(s)` ignores every QR for s seconds (used right after returning home).
    """

    def __init__(self, gap_s: float = 2.0, clock: Callable[[], float] = time.monotonic):
        self.gap_s = gap_s
        self.clock = clock
        self._last_seen: Dict[str, float] = {}
        self._blocked_until = 0.0

    def block(self, seconds: float) -> None:
        self._blocked_until = max(self._blocked_until, self.clock() + seconds)

    def observe(self, payload: str) -> bool:
        now = self.clock()
        prev = self._last_seen.get(payload)
        self._last_seen[payload] = now
        if len(self._last_seen) > 200:     # keep memory bounded
            cutoff = now - 10 * self.gap_s
            self._last_seen = {k: t for k, t in self._last_seen.items() if t >= cutoff}
        if now < self._blocked_until:
            return False
        return prev is None or (now - prev) >= self.gap_s


# ---------------------------------------------------------------- core

class MissionCore:
    def __init__(self, stations: Dict[str, Pose2D], home: str,
                 send_goal: Callable[[int, str, Pose2D], None],
                 cancel_goal: Callable[[], None],
                 clock: Callable[[], float] = time.monotonic,
                 log: Callable[[str], None] = print,
                 on_event: Callable[[str, dict], None] = lambda e, d: None,
                 nav_retries: int = 1, max_queue: int = 10):
        self.stations = stations
        self.home = home
        if home not in stations:
            raise MissionError(f"home_station '{home}' không có trong stations")
        self._send_goal_cb = send_goal
        self._cancel_goal_cb = cancel_goal
        self.clock = clock
        self.log = log
        self.on_event = on_event
        self.nav_retries = max(0, int(nav_retries))
        self.max_queue = max_queue

        self.state = State.IDLE
        self.mission: Optional[Mission] = None
        self.step_idx = 0
        self.round = 0
        self.attempt = 0
        self.target: Optional[str] = None
        self.dwell_until: Optional[float] = None
        self.queue: Deque[Mission] = deque()
        self.nav_seq = 0
        self.last_error = ""
        self.last_event = ""
        self.done_count = 0

    # ---- public API -------------------------------------------------
    @property
    def busy(self) -> bool:
        return self.state in BUSY_STATES

    def update_stations(self, stations: Dict[str, Pose2D], home: str) -> None:
        if self.busy:
            raise MissionError("không đổi stations khi đang chạy mission")
        if home not in stations:
            raise MissionError(f"home_station '{home}' không có trong stations")
        self.stations, self.home = stations, home

    def submit(self, m: Mission, replace: bool = False) -> Tuple[bool, str]:
        for s in m.steps:
            if s.station not in self.stations:
                return False, f"không có trạm '{s.station}'"
        if replace and (self.busy or self.queue):
            old = self.mission.name if self.mission else ""
            if self.busy:
                self._cancel_goal_cb()
                self.nav_seq += 1
            self.queue.clear()
            self._event("preempted", old_mission=old, new_mission=m.name)
            self._begin(m)
            return True, f"thay '{old}' bằng '{m.name}'"
        if not self.busy:
            self._begin(m)
            return True, f"bắt đầu mission '{m.name}'"
        if len(self.queue) >= self.max_queue:
            return False, f"hàng đợi đầy ({self.max_queue})"
        self.queue.append(m)
        self._event("queued", mission=m.name, mission_id=m.mid, position=len(self.queue))
        return True, f"xếp hàng '{m.name}' (vị trí {len(self.queue)})"

    def on_nav_result(self, seq: int, ok: bool, reason: str = "") -> None:
        if seq != self.nav_seq or self.state not in (State.NAVIGATING, State.RETURNING):
            return  # stale result from a cancelled / preempted goal
        if ok:
            if self.state == State.RETURNING:
                self._event("returned_home", mission=self.mission.name if self.mission else "")
                self._finish()
                return
            step = self.mission.steps[self.step_idx]
            self._event("arrived", station=step.station, step=self.step_idx + 1)
            if step.wait < 0:
                self.state = State.WAIT_CONFIRM
            elif step.wait > 0:
                self.state = State.DWELLING
                self.dwell_until = self.clock() + step.wait
            else:
                self._advance()
            return
        # failure
        if self.attempt <= self.nav_retries:
            self._event("nav_retry", station=self.target, attempt=self.attempt, reason=reason)
            self._send()
            return
        self.last_error = f"không tới được '{self.target}' sau {self.attempt} lần ({reason})"
        dropped = len(self.queue)
        self.queue.clear()
        self.state = State.ERROR
        self._event("error", error=self.last_error, queue_dropped=dropped)

    def tick(self) -> None:
        if self.state == State.DWELLING and self.dwell_until is not None \
                and self.clock() >= self.dwell_until:
            self.dwell_until = None
            self._advance()

    def confirm(self) -> Tuple[bool, str]:
        if self.state != State.WAIT_CONFIRM:
            return False, f"không ở trạng thái chờ xác nhận (đang {self.state.value})"
        self._event("confirmed", station=self.target)
        self._advance()
        return True, "đã xác nhận"

    def cancel(self) -> Tuple[bool, str]:
        """Stop in place, drop the queue."""
        if not self.busy and not self.queue:
            return False, f"không có gì để hủy (đang {self.state.value})"
        self._cancel_goal_cb()
        self.nav_seq += 1  # invalidate pending results
        self.queue.clear()
        name = self.mission.name if self.mission else ""
        mid = self.mission.mid if self.mission else ""
        self.mission, self.target, self.dwell_until = None, None, None
        self.state = State.STOPPED
        self._event("cancelled", mission=name, mission_id=mid)
        return True, "đã dừng, xe đứng tại chỗ"

    def external_override(self, reason: str) -> bool:
        """Có goal Nav2 không phải của mình (web click bản đồ, RViz): nhường quyền, dừng nhiệm vụ.

        Không gửi cancel: goal của mình đã bị Nav2 bỏ, goal mới là của người khác.
        """
        if not self.busy and not self.queue:
            return False
        self.nav_seq += 1
        self.queue.clear()
        name = self.mission.name if self.mission else ""
        mid = self.mission.mid if self.mission else ""
        self.mission, self.target, self.dwell_until = None, None, None
        self.state = State.STOPPED
        self.last_error = reason
        self._event("overridden", mission=name, mission_id=mid, reason=reason)
        return True

    def pause_external(self, reason: str) -> bool:
        """Goal ngoài chen vào giữa chuyến giao: nhường Nav2, giữ nguyên bước hiện tại để làm tiếp."""
        if self.state not in DELIVERY_STATES:
            return False
        self.nav_seq += 1          # bỏ kết quả của goal mình vừa bị Nav2 bỏ
        self.dwell_until = None
        self.state = State.PAUSED
        self._event("paused", station=self.target, reason=reason)
        return True

    def resume(self) -> bool:
        """Goal ngoài đã xong: quay lại trạm của bước đang dở và làm lại bước đó."""
        if self.state != State.PAUSED or self.mission is None:
            return False
        station = self.mission.steps[self.step_idx].station
        self._event("resumed", station=station)
        self._go(station, State.NAVIGATING)
        return True

    def go_home(self) -> Tuple[bool, str]:
        """Abort everything and drive back to home."""
        if self.busy:
            self._cancel_goal_cb()
            self.nav_seq += 1
        self.queue.clear()
        self._begin(Mission(name="go_home", steps=[], return_home=True, source="operator"))
        return True, "đang về vị trí chờ"

    def status(self) -> dict:
        m = self.mission
        st = {
            "state": self.state.value,
            "mission": m.name if m else None,
            "mission_id": m.mid if m else None,
            "source": m.source if m else None,
            "step": (self.step_idx + 1) if m and m.steps and self.state != State.RETURNING else None,
            "steps": [s.station for s in m.steps] if m else [],
            "round": (self.round + 1) if m else None,
            "repeat": m.repeat if m else None,
            "target": self.target,
            "attempt": self.attempt if self.target else None,
            "dwell_left": round(max(0.0, self.dwell_until - self.clock()), 1)
            if self.dwell_until is not None else None,
            "queue": [q.name for q in self.queue],
            "last_error": self.last_error,
            "last_event": self.last_event,
            "done_count": self.done_count,
        }
        return st

    # ---- internals --------------------------------------------------
    def _event(self, name: str, **data) -> None:
        if self.mission is not None and "mission_id" not in data:
            data["mission_id"] = self.mission.mid
        self.last_event = name
        self.log(f"[mission] {name} {data if data else ''}".rstrip())
        self.on_event(name, data)

    def _begin(self, m: Mission) -> None:
        self.mission = m
        self.step_idx = 0
        self.round = 0
        self.last_error = ""
        self._event("mission_started", mission=m.name, source=m.source,
                    steps=[s.station for s in m.steps], repeat=m.repeat)
        self._next()

    def _next(self) -> None:
        m = self.mission
        if self.step_idx >= len(m.steps):
            self.round += 1
            if m.steps and self.round < m.repeat:
                self.step_idx = 0
            elif m.return_home:
                self._go(self.home, State.RETURNING)
                return
            else:
                self._finish()
                return
        self._go(m.steps[self.step_idx].station, State.NAVIGATING)

    def _advance(self) -> None:
        self.step_idx += 1
        self._next()

    def _go(self, station: str, state: State) -> None:
        self.state = state
        self.target = station
        self.attempt = 0
        self._send()

    def _send(self) -> None:
        self.nav_seq += 1
        self.attempt += 1
        self._send_goal_cb(self.nav_seq, self.target, self.stations[self.target])

    def _finish(self) -> None:
        name = self.mission.name if self.mission else ""
        mid = self.mission.mid if self.mission else ""
        self.done_count += 1
        self.mission, self.target, self.dwell_until = None, None, None
        self._event("mission_done", mission=name, mission_id=mid)
        if  self.queue:
            self._begin(self.queue.popleft())
        else:
            self.state = State.IDLE