import os
import sys

import pytest
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from amr_mission.mission_core import (  # noqa: E402
    MissionCore, MissionError, QrGate, State, build_mission,
    extract_qr_text, parse_stations, parse_text_request,
)

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "stations.yaml")))
STATIONS = parse_stations(CFG["stations"])
PRESETS = CFG["missions"]


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeNav:
    def __init__(self):
        self.goals = []        # (seq, station)
        self.cancels = 0

    def send(self, seq, station, pose):
        self.goals.append((seq, station))

    def cancel(self):
        self.cancels += 1

    @property
    def last(self):
        return self.goals[-1]


def make(retries=1):
    clk, nav, events = Clock(), FakeNav(), []
    core = MissionCore(STATIONS, "home", nav.send, nav.cancel, clock=clk,
                       log=lambda s: None, on_event=lambda e, d: events.append(e),
                       nav_retries=retries)
    return core, nav, clk, events


def ok(core, nav):
    core.on_nav_result(nav.last[0], True)


def test_qr_single_station_then_home():
    core, nav, clk, ev = make()
    req = parse_text_request("a", "", STATIONS, PRESETS)
    req.update(wait=5.0, return_home=True)
    core.submit(build_mission(req, STATIONS, PRESETS, 5.0))
    assert core.state == State.NAVIGATING and nav.last[1] == "A"
    ok(core, nav)
    assert core.state == State.DWELLING
    clk.t += 4.9; core.tick()
    assert core.state == State.DWELLING
    clk.t += 0.2; core.tick()
    assert core.state == State.RETURNING and nav.last[1] == "home"
    ok(core, nav)
    assert core.state == State.IDLE and "returned_home" in ev and core.done_count == 1


def test_abc_loop_repeat_2():
    core, nav, clk, _ = make()
    core.submit(build_mission({"name": "abc_loop", "repeat": 2}, STATIONS, PRESETS, 5.0))
    visited = []
    for _ in range(6):
        visited.append(nav.last[1]); ok(core, nav)
        clk.t += 5.0; core.tick()
    assert visited == ["A", "B", "C", "A", "B", "C"]
    assert nav.last[1] == "home"
    ok(core, nav)
    assert core.state == State.IDLE


def test_confirm_wait():
    core, nav, _, _ = make()
    core.submit(build_mission({"name": "nhap_kho"}, STATIONS, PRESETS, 5.0))
    ok(core, nav)
    assert core.state == State.WAIT_CONFIRM
    assert core.confirm()[0]
    assert nav.last[1] == "C"


def test_retry_then_error_clears_queue():
    core, nav, _, _ = make(retries=1)
    core.submit(build_mission({"goto": "B"}, STATIONS, PRESETS, 0))
    core.submit(build_mission({"goto": "C"}, STATIONS, PRESETS, 0))
    assert len(core.queue) == 1
    core.on_nav_result(nav.last[0], False, "ABORTED")
    assert core.state == State.NAVIGATING and len(nav.goals) == 2   # retried
    core.on_nav_result(nav.last[0], False, "ABORTED")
    assert core.state == State.ERROR and not core.queue and "B" in core.last_error
    core.go_home()
    assert core.state == State.RETURNING


def test_cancel_ignores_stale_result():
    core, nav, _, _ = make()
    core.submit(build_mission({"goto": "A"}, STATIONS, PRESETS, 0))
    seq = nav.last[0]
    assert core.cancel()[0] and core.state == State.STOPPED and nav.cancels == 1
    core.on_nav_result(seq, True)          # late result must be ignored
    assert core.state == State.STOPPED


def test_queue_runs_after_home():
    core, nav, _, _ = make()
    core.submit(build_mission({"goto": "A", "wait": 0}, STATIONS, PRESETS, 0))
    core.submit(build_mission({"goto": "B", "wait": 0}, STATIONS, PRESETS, 0))
    ok(core, nav); assert nav.last[1] == "home"
    ok(core, nav); assert nav.last[1] == "B" and core.state == State.NAVIGATING


def test_no_return_home():
    core, nav, _, _ = make()
    core.submit(build_mission({"steps": ["A"], "wait": 0, "return_home": False}, STATIONS, PRESETS, 0))
    ok(core, nav)
    assert core.state == State.IDLE and nav.last[1] == "A"


def test_parse_text_request():
    P = PRESETS
    assert parse_text_request("B", "", STATIONS, P) == {"steps": ["B"]}
    assert parse_text_request("a-b-c", "", STATIONS, P) == {"steps": ["A", "B", "C"]}
    assert parse_text_request("A > C", "", STATIONS, P) == {"steps": ["A", "C"]}
    assert parse_text_request("ABC_LOOP", "", STATIONS, P) == {"name": "abc_loop"}
    assert parse_text_request("PL-0001", "", STATIONS, P) is None      # pallet QR
    assert parse_text_request("A", "AMR:", STATIONS, P) is None         # prefix required
    assert parse_text_request("amr:A-B", "AMR:", STATIONS, P) == {"steps": ["A", "B"]}


def test_extract_qr_text():
    assert extract_qr_text("A") == "A"
    assert extract_qr_text('{"text": "A-B"}') == "A-B"
    assert extract_qr_text('{"qr": {"text": "C"}}', "qr.text") == "C"
    assert extract_qr_text('{"foo": 1}') == ""
    assert extract_qr_text('"B"') == "B"


def test_build_mission_errors():
    with pytest.raises(MissionError):
        build_mission({"steps": ["X"]}, STATIONS, PRESETS, 5)
    with pytest.raises(MissionError):
        build_mission({"name": "khong_co"}, STATIONS, PRESETS, 5)
    with pytest.raises(MissionError):
        build_mission({"steps": []}, STATIONS, PRESETS, 5)
    m = build_mission({"steps": [{"station": "a", "wait": "confirm"}, "b"], "wait": 2},
                      STATIONS, PRESETS, 5)
    assert [(s.station, s.wait) for s in m.steps] == [("A", -1.0), ("B", 2.0)]


def test_qr_gate():
    clk = Clock()
    g = QrGate(gap_s=2.0, clock=clk)
    assert g.observe("A")                 # first sight
    clk.t += 0.1; assert not g.observe("A")   # still in view
    clk.t += 3.0; assert g.observe("A")   # disappeared > gap, shown again
    g.block(5.0)
    for _ in range(10):                   # QR fixed at home, robot just came back
        clk.t += 0.5; assert not g.observe("A")
    clk.t += 0.5; assert not g.observe("A")   # block over but QR never left view
    clk.t += 2.5; assert g.observe("A")   # removed then shown again
