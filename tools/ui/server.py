#!/usr/bin/env python3
"""Local UI for the ToF/LoRa tank demo.

Talks to node A over USB serial, keeps the latest state of every B node, and
serves a single page on 127.0.0.1. Standard library plus pyserial only, as
SPEC 0.1.7 requires - no web framework.

Run it from the ESP-IDF environment so pyserial is importable:

    . /Users/hanjiin/.espressif/v5.4.4/esp-idf/export.sh
    python3 tools/ui/server.py --port /dev/cu.usbserial-0001

Close `idf.py monitor` first: only one program can hold the serial port.
"""

import argparse
import glob
import statistics
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

try:
    import serial  # type: ignore
except ImportError:  # pragma: no cover - environment problem, not logic
    raise SystemExit(
        "pyserial not found. Activate the ESP-IDF environment first:\n"
        "  . /Users/hanjiin/.espressif/v5.4.4/esp-idf/export.sh"
    )

HERE = Path(__file__).resolve().parent
STATE_FILE = HERE / "ui_state.json"
ZONE_COUNT = 64
NODE_IDS = (1, 2)

# A single frame carries per-zone noise and the odd dropout, so the baseline is
# the per-zone median of the last few frames instead. Averaging N samples cuts
# random error by sqrt(N); past a point the error is dominated by mounting angle
# and surface, which more frames cannot fix. 30 is what the floor survey uses.
BASELINE_FRAMES = 30
# A zone needs this many valid readings before its baseline is trusted - a third
# of the window, the same proportion as before. Zones that stay dark (black
# floor, out of range) are left unset rather than given a made-up number.
BASELINE_MIN_VALID = 10

# Matches components/geometry/geometry.c so the UI and the firmware agree.
DEFAULT_FOV_DEG = 45.0

# The demo profile has the node take ten measurements inside one wake window,
# and the headline number is their mean. Frames are grouped by the node's own
# wake_count rather than by arrival time, so a window that ran short is never
# silently topped up with readings from the previous one.
FILL_FRAMES = 10
# Below this the newest window is still filling, so the previous complete one is
# a better answer than a single frame.
FILL_MIN_FRAMES = 3

# One definition of the four levels, served to both pages so they cannot drift
# apart. `min` is the inclusive lower bound in percent.
#
# The names describe one thing - how much the drain has filled up - on a single
# scale. An earlier draft mixed axes (a reference level, then a safety judgment,
# then a cause, then a consequence), which reads oddly when the four sit side by
# side in a legend.
# `color` fills dots and bars and reads well on the admin page's dark panels.
# `ink` is the same hue darkened enough to be legible as text on white, which
# the two public-facing pages need - #eab308 headline text on white is not
# readable. Both travel together so a level can never be half-recoloured.
LEVELS = [
    {"key": "clear",  "name": "비움", "color": "#3b82f6", "ink": "#1d4ed8",
     "min": 0,  "note": "바닥이 드러남. 배수 정상"},
    {"key": "ok",     "name": "양호", "color": "#22c55e", "ink": "#15803d",
     "min": 15, "note": "소량 적치. 배수에 지장 없음"},
    {"key": "warn",   "name": "주의", "color": "#eab308", "ink": "#a16207",
     "min": 40, "note": "적치 진행 중. 청소 권장"},
    {"key": "danger", "name": "위험", "color": "#ef4444", "ink": "#b91c1c",
     "min": 70, "note": "배수 불가 임박. 즉시 조치"},
]


def level_for(fill_pct):
    chosen = LEVELS[0]
    for level in LEVELS:
        if fill_pct >= level["min"]:
            chosen = level
    return chosen


# The demo shows one real drain among a city's worth of them. Only the first
# entry is measured; the other nine are fixed sample values, not simulations -
# they are here to give the map something to look like. They stay put across
# reloads on purpose: numbers that reshuffle every few seconds read as a bug.
SEOUL_SITES = [
    {"id": "uos", "name": "서울시립대학교",
     "addr": "서울특별시 동대문구 서울시립대로 163",
     "lon": 127.0582, "lat": 37.5834, "live": True},
    {"id": "gangnam", "name": "강남역 사거리",
     "addr": "서울특별시 강남구 테헤란로 152",
     "lon": 127.0364, "lat": 37.5006, "fill_pct": 82.0, "age_min": 4},
    {"id": "mapo", "name": "월드컵북로 입구",
     "addr": "서울특별시 마포구 월드컵북로 21",
     "lon": 126.9088, "lat": 37.5563, "fill_pct": 22.0, "age_min": 11},
    {"id": "songpa", "name": "올림픽공원 남측",
     "addr": "서울특별시 송파구 올림픽로 300",
     "lon": 127.1059, "lat": 37.5145, "fill_pct": 51.0, "age_min": 7},
    {"id": "ydp", "name": "여의대로 버스정류장",
     "addr": "서울특별시 영등포구 여의대로 108",
     "lon": 126.9270, "lat": 37.5258, "fill_pct": 6.0, "age_min": 2},
    {"id": "jongno", "name": "세종대로 시청 앞",
     "addr": "서울특별시 종로구 세종대로 175",
     "lon": 126.9769, "lat": 37.5720, "fill_pct": 44.0, "age_min": 19},
    {"id": "seongbuk", "name": "안암로 고려대 앞",
     "addr": "서울특별시 성북구 안암로 145",
     "lon": 127.0323, "lat": 37.5889, "fill_pct": 31.0, "age_min": 6},
    {"id": "eunpyeong", "name": "통일로 구파발",
     "addr": "서울특별시 은평구 통일로 1050",
     "lon": 126.9227, "lat": 37.6176, "fill_pct": 11.0, "age_min": 25},
    {"id": "gangseo", "name": "공항대로 발산역",
     "addr": "서울특별시 강서구 공항대로 376",
     "lon": 126.8360, "lat": 37.5586, "fill_pct": 74.0, "age_min": 9},
    {"id": "nowon", "name": "동일로 노원역",
     "addr": "서울특별시 노원구 동일로 1414",
     "lon": 127.0568, "lat": 37.6542, "fill_pct": 18.0, "age_min": 14},
]


def sites_payload(store):
    """The map's ten points. The live one carries whatever the nodes measured;
    an unmeasured live site keeps its dot grey rather than inventing a level."""
    snapshot = store.snapshot()
    measured = [n["assessment"]["fill_pct"] for n in snapshot["nodes"]
                if n.get("assessment") and n["assessment"].get("fill_pct") is not None]
    now = time.time()

    out = []
    for site in SEOUL_SITES:
        entry = {k: site[k] for k in ("id", "name", "addr", "lon", "lat")}
        entry["live"] = bool(site.get("live"))
        if entry["live"]:
            # Two sensors look at the same tank from opposite sides, so the
            # site's number is their mean.
            fill = sum(measured) / len(measured) if measured else None
            entry["nodes"] = len(measured)
            entry["updated"] = max(
                (n["last_seen"] for n in snapshot["nodes"] if n["last_seen"]),
                default=None)
        else:
            fill = site["fill_pct"]
            entry["updated"] = now - site["age_min"] * 60
        entry["fill_pct"] = None if fill is None else round(fill, 1)
        entry["level"] = None if fill is None else level_for(fill)
        out.append(entry)
    return {"now": now, "levels": LEVELS, "sites": out,
            "serial_ok": snapshot["serial_ok"]}


def default_node_settings():
    return {
        # Applied to the 8x8 before display. Raw frames are always kept as
        # received; SPEC 0.1.8 requires the per-node original to survive.
        "rotation": 0,      # 0, 90, 180, 270
        "mirror_x": False,
        "mirror_y": False,
        # Mount description used for the 3D projection.
        "pitch_deg": 0.0,
        "roll_deg": 0.0,
        "fov_deg": DEFAULT_FOV_DEG,
        "baseline": None,   # 64 distances captured over an empty tank
        "baseline_at": None,
        "baseline_frames": 0,
        "baseline_covered": 0,
        # One sensor can survey a tank from several mounting points by being
        # moved between them. Each slot keeps the floor and the loaded state
        # captured from one position, so they can be compared afterwards.
        "slots": {name: default_slot() for name in SLOT_NAMES},
    }


SLOT_NAMES = ("A", "B")


def default_slot():
    return {
        "baseline": None, "baseline_at": None, "baseline_covered": 0,
        "current": None, "current_at": None, "current_covered": 0,
        "rotation": 0, "mirror_x": False, "mirror_y": False,
        "label": "",
    }


class Store:
    """Everything the page needs, guarded by one lock."""

    def __init__(self):
        self.lock = threading.Lock()
        self.nodes = {
            nid: {
                "node": nid,
                "last_seen": None,
                "last_type": None,
                "seq": None,
                "rssi": None,
                "snr": None,
                "config": None,
                "telemetry": None,
                "status": None,
                "planned_sleep_ms": None,
                "frame": None,        # raw 64 distances, exactly as received
                # Same frame with the zones the node marked invalid set to
                # None. The mask is 64 bits and reaches the page as a JSON
                # number, which JavaScript rounds - it cannot do this itself
                # without silently mis-masking the low zones.
                "frame_valid": None,
                "valid_mask": None,
                "frame_at": None,
                "config_state": None,  # what the bridge reports
            }
            for nid in NODE_IDS
        }
        self.recent_frames = {nid: [] for nid in NODE_IDS}
        # (wake_count, distances, valid_mask) so a window can be reconstructed.
        self.wake_history = {nid: [] for nid in NODE_IDS}
        self.settings = {nid: default_node_settings() for nid in NODE_IDS}
        self.log = []          # recent lines from A, for the page
        self.serial_ok = False
        self.load()

    # ---- persistence -------------------------------------------------
    def load(self):
        if not STATE_FILE.exists():
            return
        try:
            saved = json.loads(STATE_FILE.read_text())
        except (OSError, ValueError):
            return
        for nid in NODE_IDS:
            stored = saved.get(str(nid))
            if isinstance(stored, dict):
                self.settings[nid].update(stored)
            slots = self.settings[nid].setdefault("slots", {})
            for name in SLOT_NAMES:
                merged = default_slot()
                merged.update(slots.get(name) or {})
                slots[name] = merged

    def save(self):
        payload = {str(nid): self.settings[nid] for nid in NODE_IDS}
        try:
            STATE_FILE.write_text(json.dumps(payload, indent=1))
        except OSError as exc:
            print(f"could not save settings: {exc}")

    # ---- ingest ------------------------------------------------------
    def add_log(self, text, echo=False):
        if echo:
            print(text, flush=True)
        with self.lock:
            self.log.append({"t": time.time(), "text": text})
            del self.log[:-200]

    def apply_event(self, event):
        node_id = event.get("node")
        if node_id not in self.nodes:
            return
        kind = event.get("ev")
        with self.lock:
            node = self.nodes[node_id]
            if kind == "config":
                node["config_state"] = event
                return

            node["last_seen"] = time.time()
            node["last_type"] = kind
            node["seq"] = event.get("seq")
            node["rssi"] = event.get("rssi")
            node["snr"] = event.get("snr")
            node["config"] = {
                "revision": event.get("rev"),
                "sleep_s": event.get("sleep_s"),
                "wake_s": event.get("wake_s"),
                "measure": event.get("measure"),
            }
            node["telemetry"] = {
                "awake_ms": event.get("awake_ms"),
                "last_sleep_ms": event.get("last_sleep_ms"),
                "total_awake_ms": event.get("total_awake_ms"),
                "total_sleep_ms": event.get("total_sleep_ms"),
                "wakes": event.get("wakes"),
                "retries": event.get("retries"),
                "vbat_mv": event.get("vbat_mv"),
            }
            if kind == "STATUS":
                node["status"] = event.get("status_name")
            elif kind == "SLEEPING":
                node["planned_sleep_ms"] = event.get("planned_sleep_ms")
            elif kind == "FRAME":
                distances = event.get("d")
                if isinstance(distances, list) and len(distances) == ZONE_COUNT:
                    node["frame"] = distances
                    node["valid_mask"] = event.get("valid_mask")
                    node["frame_valid"] = [
                        d if (event.get("valid_mask") or 0) & (1 << z) else None
                        for z, d in enumerate(distances)]
                    node["frame_at"] = time.time()
                    mask = event.get("valid_mask") or 0
                    history = self.recent_frames[node_id]
                    history.append((distances, mask))
                    del history[:-BASELINE_FRAMES]
                    windows = self.wake_history[node_id]
                    windows.append((event.get("wakes"), distances, mask))
                    del windows[:-(FILL_FRAMES * 4)]

    # ---- reads -------------------------------------------------------
    def snapshot(self):
        with self.lock:
            nodes = []
            for nid in NODE_IDS:
                node = dict(self.nodes[nid])
                node["assessment"] = self._assess_locked(nid)
                node["buffered"] = len(self.recent_frames[nid])
                node["baseline_window"] = BASELINE_FRAMES
                nodes.append(node)
            return {
                "serial_ok": self.serial_ok,
                "now": time.time(),
                "levels": LEVELS,
                "nodes": nodes,
                "settings": {str(nid): dict(self.settings[nid])
                             for nid in NODE_IDS},
                "log": list(self.log[-60:]),
            }

    def _median_of_recent(self, node_id):
        """Per-zone median over the frames where that zone actually reported a
        valid reading. Mixing in the distances that came with an invalid status
        would poison the result, and those are common over a dark floor."""
        history = list(self.recent_frames[node_id])
        if not history:
            return None, 0, 0
        values, covered = [], 0
        for zone in range(ZONE_COUNT):
            samples = [d[zone] for d, mask in history if mask & (1 << zone)]
            if len(samples) >= BASELINE_MIN_VALID:
                values.append(int(statistics.median(samples)))
                covered += 1
            else:
                values.append(None)
        return values, covered, len(history)

    def _latest_window(self, node_id):
        """Frames from the node's most recent wake window, newest last."""
        history = self.wake_history[node_id]
        if not history:
            return [], None
        newest = history[-1][0]
        group = [(d, m) for w, d, m in history if w == newest]
        if len(group) < FILL_MIN_FRAMES:
            earlier = [w for w, _, _ in history if w != newest]
            if earlier:
                previous = earlier[-1]
                older = [(d, m) for w, d, m in history if w == previous]
                if len(older) > len(group):
                    return older[-FILL_FRAMES:], previous
        return group[-FILL_FRAMES:], newest

    def _assess_locked(self, node_id):
        """How full the tank is, from the mean of one wake window's frames.

        Call with self.lock held. Returns None when there is nothing to say
        yet; otherwise always carries `frames` and `zones` so the page can show
        what the number was actually computed from rather than implying ten
        measurements when only two arrived."""
        window, wake = self._latest_window(node_id)
        if not window:
            return None

        settings = self.settings[node_id]
        baseline = settings.get("baseline")
        report = {"frames": len(window), "wake": wake, "zones": 0,
                  "fill_pct": None, "mean_height_mm": None, "level": None,
                  "baseline_mm": None, "reason": None}

        if not baseline:
            report["reason"] = "baseline 없음"
            return report

        # Full means the debris has reached the sensor, so each zone is scored
        # against its own baseline - that reading IS the distance from the
        # sensor to the floor along that zone's ray. Dividing by a single depth
        # for all 64 would be wrong here: the sensor looks down at an angle, so
        # the near column sits ~305 mm away and the far one ~450 mm over the
        # same flat floor. A ratio of two distances measured along the same ray
        # cancels that tilt exactly.
        fractions, heights = [], []
        for zone in range(ZONE_COUNT):
            floor = baseline[zone]
            if floor is None or floor <= 0:
                continue
            samples = [d[zone] for d, mask in window if mask & (1 << zone)]
            if not samples:
                continue
            mean_d = sum(samples) / len(samples)
            heights.append(floor - mean_d)
            # Deliberately unclamped per zone: clamping each one at 0 would push
            # the mean above zero over an empty tank, where noise scatters both
            # ways. Only the final figure is clamped.
            fractions.append(1.0 - mean_d / floor)
        report["zones"] = len(fractions)
        if not fractions:
            report["reason"] = "유효한 zone 없음"
            return report

        fill = max(0.0, min(100.0, sum(fractions) / len(fractions) * 100.0))
        floors = [v for v in baseline if v is not None]
        report.update({"mean_height_mm": round(sum(heights) / len(heights), 1),
                       "baseline_mm": round(sum(floors) / len(floors), 1),
                       "fill_pct": round(fill, 1),
                       "level": level_for(fill)})
        return report

    def assess(self, node_id):
        with self.lock:
            return self._assess_locked(node_id)

    def capture_baseline(self, node_id):
        with self.lock:
            values, covered, frames = self._median_of_recent(node_id)
            if values is None:
                return False, "아직 프레임을 받지 못했습니다"
            if covered == 0:
                return False, "유효한 zone이 없습니다. 바닥 반사율과 각도를 확인하세요"
            self.settings[node_id].update({
                "baseline": values, "baseline_at": time.time(),
                "baseline_frames": frames, "baseline_covered": covered,
            })
        self.save()
        return True, f"{frames}개 프레임으로 baseline 확보: {covered}/{ZONE_COUNT} zone"

    def capture_slot(self, node_id, slot_name, kind):
        """kind is 'baseline' (empty tank) or 'current' (tank as it is now)."""
        with self.lock:
            slot = self.settings[node_id]["slots"].get(slot_name)
            if slot is None:
                return False, "알 수 없는 위치"
            values, covered, frames = self._median_of_recent(node_id)
            if values is None:
                return False, "아직 프레임을 받지 못했습니다"
            if covered == 0:
                return False, "유효한 zone이 없습니다"
            slot[kind] = values
            slot[f"{kind}_at"] = time.time()
            slot[f"{kind}_covered"] = covered
        self.save()
        label = "바닥" if kind == "baseline" else "현재"
        return True, f"위치 {slot_name} {label} 기록: {covered}/{ZONE_COUNT} zone ({frames}프레임)"

    def clear_slot(self, node_id, slot_name):
        with self.lock:
            if slot_name not in self.settings[node_id]["slots"]:
                return False, "알 수 없는 위치"
            self.settings[node_id]["slots"][slot_name] = default_slot()
        self.save()
        return True, f"위치 {slot_name} 초기화"

    def update_slot_settings(self, node_id, slot_name, changes):
        allowed = {"rotation", "mirror_x", "mirror_y", "label"}
        with self.lock:
            slot = self.settings[node_id]["slots"].get(slot_name)
            if slot is None:
                return
            for key, value in changes.items():
                if key in allowed:
                    slot[key] = value
        self.save()

    def clear_baseline(self, node_id):
        with self.lock:
            self.settings[node_id]["baseline"] = None
            self.settings[node_id]["baseline_at"] = None
            self.settings[node_id]["baseline_frames"] = 0
            self.settings[node_id]["baseline_covered"] = 0
        self.save()

    def update_settings(self, node_id, changes):
        allowed = {"rotation", "mirror_x", "mirror_y", "pitch_deg", "roll_deg",
                   "fov_deg"}
        with self.lock:
            for key, value in changes.items():
                if key in allowed:
                    self.settings[node_id][key] = value
        self.save()


class SerialLink:
    """Reader thread plus a writer queue for commands going to A."""

    def __init__(self, port, baud, store):
        self.port = port
        self.baud = baud
        self.store = store
        self.outbox = queue.Queue()
        self.stop = threading.Event()

    def send(self, line):
        self.outbox.put(line.strip() + "\n")

    def run(self):
        while not self.stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as link:
                    self.store.serial_ok = True
                    self.store.add_log(f"connected to {self.port}", echo=True)
                    # The bridge boots in human-readable mode; the UI wants JSON.
                    time.sleep(0.3)
                    link.write(b"MODE JSON\n")
                    link.write(b"STATUS\n")
                    self._pump(link)
            except (serial.SerialException, OSError) as exc:
                self.store.add_log(f"serial error: {exc}", echo=True)
            self.store.serial_ok = False
            time.sleep(1.0)

    def _pump(self, link):
        buffer = b""
        while not self.stop.is_set():
            while not self.outbox.empty():
                command = self.outbox.get()
                link.write(command.encode())
                self.store.add_log(f"> {command.strip()}")

            chunk = link.read(4096)
            if chunk:
                buffer += chunk
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    self._handle_line(raw.decode("utf-8", "replace").strip())

    def _handle_line(self, text):
        if not text:
            return
        if text.startswith("{"):
            try:
                event = json.loads(text)
            except ValueError:
                self.store.add_log(text)
                return
            self.store.apply_event(event)
            if event.get("ev") == "config":
                self.store.add_log(
                    f"config B-{event.get('node')}: {event.get('state')}"
                    f" rev={event.get('rev')} ({event.get('note')})")
            return
        self.store.add_log(text)


class Handler(BaseHTTPRequestHandler):
    store: Store = None      # set in main()
    link: SerialLink = None

    def log_message(self, fmt, *args):
        pass  # the default handler spams stderr for every poll

    def _send(self, code, body, content_type="application/json"):
        payload = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    PAGES = {
        "/": "home.html",
        "/home": "home.html",
        "/user": "user.html",
        "/user.html": "user.html",
        "/admin": "index.html",
        "/admin.html": "index.html",
        "/index.html": "index.html",
        "/survey": "survey.html",
        "/survey.html": "survey.html",
    }

    def do_GET(self):
        page = self.PAGES.get(self.path)
        if page:
            self._send(200, (HERE / page).read_bytes(),
                       "text/html; charset=utf-8")
        elif self.path == "/api/state":
            self._send(200, json.dumps(self.store.snapshot()))
        elif self.path == "/api/sites":
            self._send(200, json.dumps(sites_payload(self.store)))
        else:
            self._send(404, json.dumps({"error": "not found"}))

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(length) or b"{}")
        except ValueError:
            self._send(400, json.dumps({"error": "bad json"}))
            return

        if self.path == "/api/command":
            command = str(body.get("line", "")).strip()
            if not command:
                self._send(400, json.dumps({"error": "empty command"}))
                return
            self.link.send(command)
            self._send(200, json.dumps({"ok": True}))
            return

        node_id = body.get("node")
        if node_id not in NODE_IDS:
            self._send(400, json.dumps({"error": "bad node"}))
            return

        if self.path == "/api/baseline":
            if body.get("clear"):
                self.store.clear_baseline(node_id)
                self._send(200, json.dumps({"ok": True, "msg": "baseline cleared"}))
                return
            ok, msg = self.store.capture_baseline(node_id)
            self._send(200 if ok else 409, json.dumps({"ok": ok, "msg": msg}))
            return

        if self.path == "/api/slot":
            slot = str(body.get("slot", ""))
            action = str(body.get("action", ""))
            if action in ("baseline", "current"):
                ok, msg = self.store.capture_slot(node_id, slot, action)
            elif action == "clear":
                ok, msg = self.store.clear_slot(node_id, slot)
            elif action == "settings":
                self.store.update_slot_settings(node_id, slot,
                                                body.get("changes", {}))
                ok, msg = True, "적용됨"
            else:
                ok, msg = False, "알 수 없는 동작"
            self._send(200 if ok else 409, json.dumps({"ok": ok, "msg": msg}))
            return

        if self.path == "/api/settings":
            self.store.update_settings(node_id, body.get("changes", {}))
            self._send(200, json.dumps({"ok": True}))
            return

        self._send(404, json.dumps({"error": "not found"}))


def guess_port():
    candidates = sorted(glob.glob("/dev/cu.usbserial*"))
    return candidates[0] if candidates else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None,
                        help="serial port of node A (default: first usbserial)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--http-port", type=int, default=8765)
    args = parser.parse_args()

    port = args.port or guess_port()
    if not port:
        raise SystemExit("no /dev/cu.usbserial* found; pass --port")

    store = Store()
    link = SerialLink(port, args.baud, store)
    threading.Thread(target=link.run, daemon=True).start()

    Handler.store = store
    Handler.link = link
    server = ThreadingHTTPServer(("127.0.0.1", args.http_port), Handler)
    print(f"serial : {port} @ {args.baud}")
    print(f"open   : http://127.0.0.1:{args.http_port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        link.stop.set()


if __name__ == "__main__":
    main()
