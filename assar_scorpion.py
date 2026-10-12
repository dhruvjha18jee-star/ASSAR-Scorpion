"""
╔══════════════════════════════════════════════════════════════╗
║   A.S.S.A.R  —  PROJECT SCORPION  —  Python Bridge          ║
╠══════════════════════════════════════════════════════════════╣
║  HARDWARE THIS CODE IS BUILT FOR (UNCHANGED FROM ASSAR v1):  ║
║    HC-05 TXD → D7  |  HC-05 RXD → D10 (voltage divider)    ║
║    IN1=D9  IN2=D8  IN3=D6  IN4=D5                           ║
║    ENA/ENB via jumper caps on L298N                          ║
║    No wiring, no new sensors, no new components.             ║
║    Chassis may be rebuilt in 5mm foam board — electronics    ║
║    and pin map are 100% identical to the original build.     ║
╠══════════════════════════════════════════════════════════════╣
║  EVERYTHING BELOW THIS BLOCK THAT ALSO EXISTED IN THE        ║
║  ORIGINAL assar_rover.py (YOLO tracking math, evasion,       ║
║  search-scan, RoverSerial reconnect logic, VideoStream       ║
║  buffering) IS UNCHANGED. This file only ADDS the pieces      ║
║  that were missing or buggy — see CHANGELOG below.            ║
╠══════════════════════════════════════════════════════════════╣
║  CHANGELOG  (assar_rover.py → assar_scorpion.py):            ║
║  • NEW  Real dashboard UI at "/" (was API-only, no page)     ║
║  • NEW  Works from phone browser too (same LAN, same server) ║
║  • NEW  Boot gate: dashboard waits for BT before unlocking   ║
║  • NEW  Manual drive is now smooth (sticky cmd + 0.5s        ║
║         fail-safe timeout) instead of one-shot-per-frame     ║
║  • NEW  Voice can speak from the BROWSER (phone or laptop    ║
║         speaker — whichever has the dashboard open), not     ║
║         only from the laptop running the .py                 ║
║  • FIX  pyttsx3 "only speaks once" bug (Windows SAPI5 loop)  ║
║  • FIX  evasion_duration was truncated to an int by the      ║
║         /config endpoint (1.5s → 1s) — now stays a float     ║
║  • FIX  Directional states (TRACKING_LEFT/RIGHT, CLEARING,   ║
║         mode switches) were silently un-announced — fixed    ║
║  • NEW  /api/reconnect_camera, /api/reconnect_bt — editing   ║
║         camera_url / com_port from the dashboard now         ║
║         actually reconnects (previously had no effect        ║
║         until the old connection happened to fail)           ║
║  • NEW  /api/net_info + optional /qr.png for phone pairing   ║
╠══════════════════════════════════════════════════════════════╣
║  INSTALL:  pip install -r requirements.txt                   ║
║  RUN:      python assar_scorpion.py                          ║
║  OPEN:     http://localhost:8888        (on the laptop)      ║
║            http://<laptop-LAN-ip>:8888  (on the phone)       ║
╚══════════════════════════════════════════════════════════════╝
"""

import cv2
import serial
import serial.tools.list_ports
import time
import threading
import queue
import random
import socket
import io
import pyttsx3
from typing import Optional
from flask import Flask, Response, jsonify, request, render_template

# qrcode is optional — the dashboard works fine without it, it just won't
# render a scannable QR code on the boot screen (the LAN URL is always
# shown as plain text either way). Install with: pip install qrcode
try:
    import qrcode
    QR_AVAILABLE = True
except ImportError:
    QR_AVAILABLE = False

from flask_cors import CORS
from ultralytics import YOLO
import torch

# ══════════════════════════════════════════════════════════════
# 1. CONFIGURATION  (all values editable via web dashboard)
# ══════════════════════════════════════════════════════════════
config = {
    # Set to "AUTO" to scan for HC-05 automatically,
    # or set to exact port e.g. "COM7" / "/dev/rfcomm0"
    "com_port":              "COM5",

    # IP Webcam app URL — must be on same Wi-Fi as laptop
    # UPDATE THIS every session if your phone IP changes
    "camera_url":            "http://192.168.1.13:8080/video",

    # YOLO class IDs
    "target_class_id":       0,              # 0 = person
    "obstacle_classes":      [56, 57, 60],   # chair, couch, dining table

    # Detection sensitivity
    "confidence_threshold":  0.45,           # 0.1 – 0.95

    # ── LATENCY FIX: frame is now 320x240 so margin is scaled down ──
    "center_margin":         60,             # px band around center before turning (320px wide frame)

    # Distance thresholds (bounding-box area in pixels²)
    # Frame is 320x240 = 76800 px total.  ~65% = 50000,  ~40% = 30000
    "stop_area_threshold":   50000,          # triggers evasion when target too close
    "approach_slow_area":    30000,          # future: slow approach when target this close

    # Timing — tightened for lower latency
    "command_throttle":      0.08,           # min seconds between Bluetooth sends (was 0.12)
    "evasion_duration":      1.5,            # seconds of reverse during evasion
    "post_evasion_turn_ms":  600,            # ms of turning after reversing
    "target_grace_ms":       300,            # ms to keep last command if target flickers (was 500)
    "search_rotate_ms":      900,            # ms to rotate each direction during search (was 1800)
    "ping_interval":         1.0,            # seconds between Arduino watchdog pings

    # ── NEW (Scorpion) ──────────────────────────────────────────
    # Manual mode now holds the last command "sticky" instead of firing
    # once per frame, so the frontend just re-sends it every ~150ms while
    # a button is held. If no fresh command arrives within this window
    # (dropped wifi, closed tab, phone locked) it fails safe to Stop.
    "manual_command_timeout": 0.5,           # seconds — manual drive fail-safe

    # Local (laptop) pyttsx3 narration. OFF by default in Scorpion because
    # the dashboard now speaks from the BROWSER (phone or laptop speaker,
    # whichever tab is open) via the Web Speech API, which is more
    # reliable and works from either device. Turn this on if you want the
    # laptop to also narrate out loud even with no browser tab open at
    # all — just know you'll hear things twice if a browser dashboard is
    # also open on that same laptop with its voice unmuted.
    "local_speech_enabled":  False,

    # The old cv2.imshow() debug window only ever appears on the laptop's
    # own screen — it is not visible from the phone or the web dashboard.
    # Keep it for at-the-desk debugging, or turn off for headless /
    # phone-only runs.
    "show_local_window":     True,
}
config_lock = threading.Lock()

# ══════════════════════════════════════════════════════════════
# 2. SHARED STATE  (written by control loop, read by Flask)
# ══════════════════════════════════════════════════════════════
rover_state = {
    "state":            "OFFLINE",
    "mode":             "autonomous",   # "autonomous" | "manual"
    "evading":          False,
    "target_found":     False,
    "fps":              0.0,
    "connected":        False,
    "serial_connected": False,
    "bt_port":          "—",
    "targets_detected": 0,
    "link_confirmed":   False,   # NEW (Scorpion) — see RoverSerial.link_confirmed
}
state_lock = threading.Lock()

activity_log  = []
log_lock       = threading.Lock()

# ── Manual-drive state (Scorpion: sticky value, not one-shot) ──────────
# manual_command[0]      — the LAST direction the dashboard asked for.
# manual_command_time[0] — wall-clock time it was set, for the fail-safe
#                          timeout in the main loop (see MANUAL MODE below).
manual_command      = [b"S"]
manual_command_time = [0.0]
manual_lock         = threading.Lock()

# NEW (Scorpion): lets /api/shutdown (a different thread than the main
# loop) request a clean stop. Pressing 'q' in the optional local window
# still just breaks the loop directly (same thread, no cross-thread
# hazard) — this Event only exists for the cross-thread dashboard case.
shutdown_event = threading.Event()

# Maps the byte actually sent to the Arduino -> a human-readable state,
# used for both the dashboard telemetry and voice narration.
MANUAL_STATE_MAP = {
    b"F": "MANUAL_FORWARD",
    b"B": "MANUAL_BACKWARD",
    b"L": "MANUAL_LEFT",
    b"R": "MANUAL_RIGHT",
    b"S": "MANUAL_STOPPED",
}


_log_seq = [0]   # monotonic counter so the dashboard can fetch only new entries


def log_event(kind: str, msg: str, announce: bool = False,
               speech: Optional[str] = None) -> None:
    """
    Record an activity-log entry. If announce=True, the entry is flagged
    for voice narration in the dashboard (via /logs -> Web Speech API in
    the browser) using `speech` as the exact phrase — falling back to
    `msg` if `speech` isn't given. announce=False entries (most system/
    boot chatter) are still visible in the log panel, just not spoken.
    """
    with log_lock:
        _log_seq[0] += 1
        entry = {
            "seq":       _log_seq[0],
            "timestamp": time.strftime("%H:%M:%S"),
            "type":      kind,
            "message":   msg,
            "announce":  announce,
            "speech":    (speech if speech is not None else msg) if announce else None,
        }
        activity_log.append(entry)
        if len(activity_log) > 400:
            activity_log.pop(0)
    print(f"[{entry['timestamp']}][{kind.upper():8s}] {msg}")


# ══════════════════════════════════════════════════════════════
# 3. BLUETOOTH PORT DETECTION
# ══════════════════════════════════════════════════════════════
def find_bluetooth_port() -> Optional[str]:
    """
    Try to find the HC-05 virtual COM port automatically.
    Checks description/HWID keywords first, then lists all ports.
    """
    ports    = serial.tools.list_ports.comports()
    keywords = ["bluetooth", "bthenum", "hc-05", "hc05",
                "rfcomm", "wireless", "serial over bt"]

    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(kw in desc or kw in hwid for kw in keywords):
            log_event("system", f"BT port auto-detected: {p.device} — {p.description}")
            return p.device

    if ports:
        names = [p.device for p in ports]
        log_event("system", f"No BT port matched. Available ports: {names}")
        log_event("system", "Set com_port manually in config or via dashboard.")
    else:
        log_event("system", "No serial ports found at all. Is the device paired?")
    return None


# ══════════════════════════════════════════════════════════════
# 4. BLUETOOTH SERIAL MANAGER  (with auto-reconnect)
# ══════════════════════════════════════════════════════════════
class RoverSerial:
    # NEW (Scorpion): how long without a valid ack before we stop calling
    # the link "confirmed". The Arduino's own ping cadence guarantees an
    # ack at least once a second when idle (more often while driving), so
    # 3x that gives comfortable margin against a single missed byte
    # without taking long to notice a genuinely dead link.
    ACK_TIMEOUT = 3.0
    _ACK_BYTES  = (b"F", b"B", b"L", b"R", b"S", b"P")

    def __init__(self):
        self._ser         = None
        self._lock        = threading.Lock()
        self._port        = None
        self._reconnecting = False

        # NEW (Scorpion) — see link_confirmed / _ack_listener_worker below.
        # Requires ASSAR_Arduino_Scorpion.ino (optional). With the original
        # ASSAR_Arduino_Final.ino this simply never receives an ack and
        # link_confirmed stays False forever — everything else (sending
        # commands, driving the rover) is completely unaffected either way.
        self._ack_lock      = threading.Lock()
        self._last_ack_time = 0.0
        self._last_ack_byte = b""
        threading.Thread(target=self._ack_listener_worker, daemon=True).start()

    def connect(self, port: str, baud: int = 9600) -> bool:
        with self._lock:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
                self._ser  = serial.Serial(port, baud, timeout=0.1)
                self._port = port
                time.sleep(2.0)   # HC-05 needs ~2s to stabilise after RFCOMM connect
                log_event("system", f"Bluetooth connected: {port} @ {baud} baud")
                return True
            except Exception as e:
                log_event("system", f"Bluetooth connect FAILED on {port}: {e}")
                self._ser = None
                return False

    def send(self, cmd: bytes) -> bool:
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(cmd)
                    return True
                except Exception as e:
                    log_event("system", f"Bluetooth write error: {e} — scheduling reconnect")
                    self._ser = None
        # Trigger background reconnect if we lost the port
        if not self._reconnecting and self._port:
            self._reconnecting = True
            threading.Thread(target=self._reconnect_worker, daemon=True).start()
        return False

    def _reconnect_worker(self):
        """Background thread: retry Bluetooth connection every 5 seconds."""
        log_event("system", "Bluetooth reconnect thread started...")
        while True:
            time.sleep(5)
            with config_lock:
                port = config["com_port"]
            if port == "AUTO":
                port = find_bluetooth_port()
            if port and self.connect(port, 9600):
                log_event("system", "Bluetooth reconnected successfully")
                with state_lock:
                    rover_state["serial_connected"] = True
                    rover_state["bt_port"]          = port
                self._reconnecting = False
                return
            log_event("system", "Bluetooth reconnect attempt failed — retrying in 5s")

    def close(self):
        with self._lock:
            if self._ser and self._ser.is_open:
                try:
                    self._ser.write(b"S")
                    time.sleep(0.1)
                    self._ser.close()
                except Exception:
                    pass

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._ser is not None and self._ser.is_open

    def _ack_listener_worker(self):
        """
        NEW (Scorpion). Watches for the Arduino echoing a recognised
        command byte back (see ASSAR_Arduino_Scorpion.ino — optional,
        not required). is_open being True only proves the Windows COM
        port opened; it says nothing about whether an Arduino is
        actually alive on the other end of that link. An ack seen here
        is real evidence the Arduino received and acted on a byte.

        Deliberately does NOT hold self._lock for the blocking read —
        only to grab a reference to the current serial object — so a
        slow/absent ack can never stall send()/connect()/close() on
        other threads. If close()/connect() swap or close the
        underlying port while a read is in flight, pyserial raises,
        we swallow it and loop back to pick up whatever self._ser is
        now on the next pass.
        """
        while True:
            with self._lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(0.2)
                continue
            try:
                byte = ser.read(1)   # port opened with timeout=0.1, see connect()
            except Exception:
                byte = b""
            if byte in self._ACK_BYTES:
                with self._ack_lock:
                    self._last_ack_time = time.time()
                    self._last_ack_byte = byte

    @property
    def link_confirmed(self) -> bool:
        """True if the Arduino has ack'd a command within ACK_TIMEOUT
        seconds. Requires the optional Scorpion firmware — see above."""
        with self._ack_lock:
            return (time.time() - self._last_ack_time) < self.ACK_TIMEOUT

    @property
    def last_ack_age(self) -> float:
        """Seconds since the last valid ack, or a large sentinel if none yet."""
        with self._ack_lock:
            if self._last_ack_time == 0.0:
                return 9999.0
            return time.time() - self._last_ack_time


rover_serial = RoverSerial()


# ══════════════════════════════════════════════════════════════
# 5. ARDUINO WATCHDOG PING
# ══════════════════════════════════════════════════════════════
def _ping_worker():
    """
    Sends a 'P' (ping) byte to Arduino every ping_interval seconds.
    This resets the Arduino's 1500ms safety watchdog so it does not
    stop motors while rover is legitimately in SEARCHING/STOPPED state.
    Without this, the watchdog would kill motor state after 1.5s of 'S'.
    """
    while True:
        with config_lock:
            interval = config["ping_interval"]
        rover_serial.send(b"P")
        time.sleep(interval)


threading.Thread(target=_ping_worker, daemon=True).start()


# ══════════════════════════════════════════════════════════════
# 6. SPEECH ENGINE  (isolated thread — never blocks vision loop)
# ══════════════════════════════════════════════════════════════
# BUG FIX (Scorpion): the original code created ONE pyttsx3 engine at
# thread startup and called engine.say()+runAndWait() on it repeatedly
# in a loop. On Windows, pyttsx3's SAPI5 driver very commonly only
# produces audio on the FIRST runAndWait() call in that pattern — every
# call after that silently no-ops (this is a long-documented pyttsx3
# issue, not specific to this project). That matches exactly what was
# reported: only the boot phrase "Systems active..." was ever heard.
#
# The fix used here is the standard workaround: initialise a fresh
# engine for every single phrase instead of reusing one across the
# whole session. It costs a small amount of extra latency per phrase
# (engine init), which is irrelevant here because announcements are
# already decoupled from the vision loop by this background thread and
# rate-limited by _SPEECH_COOLDOWN below.
_speech_q       = queue.Queue()
_last_phrase    = ""
_last_phrase_t  = 0.0
_SPEECH_COOLDOWN = 5.0    # seconds before repeating same phrase


def _speech_worker():
    while True:
        try:
            text = _speech_q.get(timeout=1)
            if text == "__QUIT__":
                break
            try:
                engine = pyttsx3.init()          # fresh engine per phrase — see note above
                engine.setProperty("rate", 158)
                engine.setProperty("volume", 1.0)
                engine.say(text)
                engine.runAndWait()
                try:
                    engine.stop()
                except Exception:
                    pass
                del engine
            except Exception as e:
                log_event("system", f"TTS phrase failed: {e}")
            _speech_q.task_done()
        except queue.Empty:
            continue
        except Exception as e:
            log_event("system", f"TTS runtime error: {e}")


threading.Thread(target=_speech_worker, daemon=True).start()


def speak(text: str) -> None:
    """
    Queue a phrase for the LAPTOP's own speaker via pyttsx3. Gated by
    config["local_speech_enabled"] (default False in Scorpion — see the
    config block for why). This has no effect on the dashboard's own
    browser-based voice, which runs independently on whichever device
    (phone or laptop) has the page open and is not affected by this flag.
    """
    global _last_phrase, _last_phrase_t
    with config_lock:
        if not config["local_speech_enabled"]:
            return
    now = time.time()
    if text == _last_phrase and (now - _last_phrase_t) < _SPEECH_COOLDOWN:
        return
    if _speech_q.qsize() < 2:
        _speech_q.put(text)
        _last_phrase   = text
        _last_phrase_t = now


# ══════════════════════════════════════════════════════════════
# 7. ZERO-LATENCY VIDEO STREAM  (always-fresh frame buffer)
# ══════════════════════════════════════════════════════════════
class VideoStream:
    """
    Decodes frames in a background thread and keeps only the latest one.
    This eliminates the built-in buffer lag of cv2.VideoCapture.
    Also monitors for stream death and attempts reconnect.
    """
    def __init__(self, src: str):
        self._src         = src
        self._stream       = None
        self.frame         = None
        self.stopped       = False
        self._lock         = threading.Lock()   # guards self.frame
        self._stream_lock  = threading.Lock()   # guards self._stream / self._src
        self._open_stream()

    def _open_stream(self):
        if self._stream:
            self._stream.release()
        self._stream = cv2.VideoCapture(self._src)
        self._stream.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def start(self):
        ok, frame = self._stream.read()
        if ok:
            with self._lock:
                self.frame = frame
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        fail_count = 0
        while not self.stopped:
            with self._stream_lock:
                ret, frame = self._stream.read()
            if ret:
                fail_count = 0
                with self._lock:
                    self.frame = frame
            else:
                fail_count += 1
                if fail_count > 30:
                    # Stream dead — try to reconnect
                    log_event("system", "Camera stream lost — attempting reconnect...")
                    with self._stream_lock:
                        self._open_stream()
                    fail_count = 0
                    time.sleep(2)
            time.sleep(0.01)

    def read(self) -> Optional[object]:
        with self._lock:
            return self.frame.copy() if self.frame is not None else None

    def change_source(self, new_src: str) -> None:
        """
        NEW (Scorpion): switch to a different camera URL on a running
        session. Closes a real gap in the original code — editing
        camera_url through the dashboard's /config endpoint updated the
        stored value, but nothing ever reopened cv2.VideoCapture with it,
        so the change silently had no effect until the old stream
        happened to drop on its own. Wired to POST /api/reconnect_camera.
        """
        with self._stream_lock:
            self._src = new_src
            self._open_stream()
        log_event("system", f"Camera source switched to {new_src}")

    def stop(self):
        self.stopped = True
        if self._stream:
            self._stream.release()


def get_lan_ip() -> str:
    """
    NEW (Scorpion): best-effort discovery of this laptop's LAN IP address,
    so the dashboard can tell you the exact URL to open on your phone
    instead of you having to run ipconfig yourself. Opens a UDP "connection"
    to a public IP — this sends no actual packets (UDP has no handshake),
    it just asks the OS to pick which local interface/IP would be used to
    reach that address, which is a standard, harmless trick for this.
    Falls back to "127.0.0.1" (localhost-only) if it can't be determined,
    e.g. no network interface is up at all.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


DASHBOARD_PORT = 8888

# ══════════════════════════════════════════════════════════════
# 8. FLASK DASHBOARD BRIDGE  (http://localhost:8888)
# ══════════════════════════════════════════════════════════════
flask_app  = Flask(__name__)
CORS(flask_app)
_frame_jpg = None
_frame_lk  = threading.Lock()


@flask_app.route("/")
def dashboard_page():
    """
    NEW (Scorpion): this is the piece that was actually missing. The
    original file had a complete API (/status, /logs, /command, /mode,
    /config, /video_feed) and its own docstring called it a "dashboard",
    but there was no @flask_app.route("/") anywhere in it — nothing ever
    served an HTML page, so the "dashboard" was API-only. Flask looks for
    templates in a "templates/" folder next to this script by default,
    so dashboard.html must sit at templates/dashboard.html alongside
    assar_scorpion.py for this to find it.
    """
    return render_template("dashboard.html", dashboard_port=DASHBOARD_PORT)


@flask_app.route("/api/net_info")
def api_net_info():
    """NEW (Scorpion): tells the dashboard its own LAN URL, so it can show
    you the exact address to open on your phone instead of you needing to
    run ipconfig yourself."""
    ip = get_lan_ip()
    return jsonify({
        "lan_ip":         ip,
        "port":           DASHBOARD_PORT,
        "dashboard_url":  f"http://{ip}:{DASHBOARD_PORT}",
        "qr_available":   QR_AVAILABLE,
    })


@flask_app.route("/qr.png")
def api_qr():
    """NEW (Scorpion, optional): a scannable QR code pointing at this
    dashboard's LAN URL, for fast phone pairing. Degrades gracefully —
    if the `qrcode` package isn't installed, returns 404 and the
    dashboard just shows the URL as plain text instead (see /api/net_info)."""
    if not QR_AVAILABLE:
        return jsonify({"error": "qrcode package not installed — pip install qrcode"}), 404
    ip = get_lan_ip()
    url = f"http://{ip}:{DASHBOARD_PORT}"
    img = qrcode.make(url)
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    buf.seek(0)
    return Response(buf.getvalue(), mimetype="image/png")


@flask_app.route("/status")
def api_status():
    with state_lock:
        return jsonify(dict(rover_state))


@flask_app.route("/logs")
def api_logs():
    with log_lock:
        return jsonify(list(activity_log[-80:]))


@flask_app.route("/command", methods=["POST"])
def api_command():
    """
    Set the current manual drive direction. Scorpion note: this is now a
    STICKY value (see MANUAL MODE in the main loop) — the dashboard is
    expected to call this repeatedly (~every 150ms) while a direction
    button is held, and call it once more with cmd="S" on release. If no
    call arrives for manual_command_timeout seconds the rover stops on
    its own regardless of what was last sent.
    """
    data = request.get_json(silent=True) or {}
    cmd  = data.get("cmd", "").upper()
    if cmd not in ("F", "B", "L", "R", "S"):
        return jsonify({"error": "invalid command — use F/B/L/R/S"}), 400
    with state_lock:
        if rover_state["mode"] != "manual":
            return jsonify({"error": "switch to MANUAL mode first"}), 400
    with manual_lock:
        manual_command[0]      = cmd.encode()
        manual_command_time[0] = time.time()
    return jsonify({"ok": True, "cmd": cmd})


@flask_app.route("/mode", methods=["POST"])
def api_mode():
    data = request.get_json(silent=True) or {}
    mode = data.get("mode", "").lower()
    if mode not in ("autonomous", "manual"):
        return jsonify({"error": "mode must be 'autonomous' or 'manual'"}), 400
    with state_lock:
        rover_state["mode"] = mode
    if mode == "manual":
        # Always enter manual mode stopped, with a fresh timestamp, so a
        # stale sticky command from a previous manual session can never
        # carry over.
        with manual_lock:
            manual_command[0]      = b"S"
            manual_command_time[0] = time.time()
        rover_serial.send(b"S")
    phrase = "Manual control engaged." if mode == "manual" else "Autonomous mode engaged."
    speak(phrase)
    log_event("system", f"Mode switched → {mode.upper()}", announce=True, speech=phrase)
    return jsonify({"ok": True, "mode": mode})


@flask_app.route("/api/estop", methods=["POST"])
def api_estop():
    """
    Emergency stop. Works regardless of current mode: forces manual mode
    (so autonomous logic can't immediately override it next frame), zeros
    the sticky command, and sends S directly and immediately — all in one
    atomic server-side call, rather than the frontend sequencing two
    separate requests that could theoretically interleave with something
    else.
    """
    with state_lock:
        rover_state["mode"] = "manual"
    with manual_lock:
        manual_command[0]      = b"S"
        manual_command_time[0] = time.time()
    rover_serial.send(b"S")
    speak("Emergency stop.")
    log_event("system", "EMERGENCY STOP triggered from dashboard",
              announce=True, speech="Emergency stop.")
    return jsonify({"ok": True, "mode": "manual"})


@flask_app.route("/config", methods=["GET"])
def api_config_get():
    with config_lock:
        return jsonify(dict(config))


@flask_app.route("/config", methods=["POST"])
def api_config_set():
    data = request.get_json(silent=True) or {}
    with config_lock:
        str_keys  = ("com_port", "camera_url")

        # BUG FIX (Scorpion): evasion_duration used to live in int_keys
        # below, so posting it from a dashboard would silently truncate
        # 1.5s -> 1s (int(1.5) == 1). It's consumed in the main loop as
        # `now + ev_dur` seconds, so it must stay a float.
        float_keys = ("confidence_threshold", "ping_interval", "command_throttle",
                      "evasion_duration", "manual_command_timeout")

        int_keys  = ("center_margin", "stop_area_threshold",
                     "post_evasion_turn_ms", "target_grace_ms", "search_rotate_ms",
                     "target_class_id")   # NEW: was declared in config but never
                                          # actually accepted by this endpoint

        bool_keys = ("show_local_window", "local_speech_enabled")   # NEW

        for k in str_keys:
            if k in data:
                config[k] = str(data[k])
        for k in float_keys:
            if k in data:
                config[k] = round(max(0.05, float(data[k])), 3)
        for k in int_keys:
            if k in data:
                config[k] = int(data[k])
        for k in bool_keys:
            if k in data:
                config[k] = bool(data[k])

        # NEW: obstacle_classes is a list of COCO class ids — was declared
        # in config but, like target_class_id, this endpoint had no code
        # path that ever accepted it.
        if "obstacle_classes" in data:
            try:
                config["obstacle_classes"] = [int(c) for c in data["obstacle_classes"]]
            except (TypeError, ValueError):
                return jsonify({"error": "obstacle_classes must be a list of integers"}), 400

        if "confidence_threshold" in data:
            config["confidence_threshold"] = max(0.1, min(0.95, config["confidence_threshold"]))
        if "manual_command_timeout" in data:
            config["manual_command_timeout"] = max(0.15, config["manual_command_timeout"])
    log_event("system", "Configuration updated via dashboard")
    return jsonify({"ok": True})


@flask_app.route("/api/reconnect_camera", methods=["POST"])
def api_reconnect_camera():
    """
    NEW (Scorpion): reopen the camera stream using the CURRENTLY
    configured camera_url, on demand. Gap this closes: editing
    camera_url through /config only ever updated the stored string —
    nothing reopened cv2.VideoCapture with it, so a new phone IP each
    session had no effect until the old stream happened to drop on its
    own. See VideoStream.change_source().
    """
    with config_lock:
        url = config["camera_url"]
    try:
        cap.change_source(url)
        return jsonify({"ok": True, "camera_url": url})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500


@flask_app.route("/api/reconnect_bt", methods=["POST"])
def api_reconnect_bt():
    """
    NEW (Scorpion): actually attempt a Bluetooth reconnect using the
    CURRENTLY configured com_port (or AUTO-detect) right now, instead of
    waiting for the old connection to fail on its own and the background
    reconnect thread to eventually pick up a changed value. connect()
    blocks ~2s for HC-05 stabilisation, so this runs in a background
    thread — poll /status (serial_connected) for the result.
    """
    with config_lock:
        port = config["com_port"]
    if port == "AUTO":
        port = find_bluetooth_port()
    if not port:
        return jsonify({"ok": False, "error": "no Bluetooth port found/configured"}), 400

    def _do_reconnect(p):
        ok = rover_serial.connect(p, 9600)
        with state_lock:
            rover_state["serial_connected"] = ok
            rover_state["bt_port"]          = p if ok else "—"

    threading.Thread(target=_do_reconnect, args=(port,), daemon=True).start()
    return jsonify({"ok": True, "attempting_port": port})


@flask_app.route("/api/shutdown", methods=["POST"])
def api_shutdown():
    """
    NEW (Scorpion): clean stop, triggerable from the dashboard — useful
    if you're driving from your phone and the laptop isn't within reach
    to press Q or Ctrl+C. Sets shutdown_event, which the main loop checks
    once per frame; the SAME finally: block that Ctrl+C hits still runs
    (motors stopped, Bluetooth port closed, camera released), so it's a
    real shutdown, not a forceful kill.
    """
    log_event("system", "Shutdown requested from dashboard")
    shutdown_event.set()
    return jsonify({"ok": True})


def _mjpeg_generator():
    """Yields a continuous MJPEG stream for the browser dashboard."""
    while True:
        with _frame_lk:
            jpg = _frame_jpg
        if jpg:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + jpg
                + b"\r\n"
            )
        time.sleep(0.04)   # ~25 fps ceiling


@flask_app.route("/video_feed")
def api_video_feed():
    return Response(
        _mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def _start_flask():
    flask_app.run(host="0.0.0.0", port=DASHBOARD_PORT, threaded=True, use_reloader=False)


# ══════════════════════════════════════════════════════════════
# 9. BOOT SEQUENCE
# ══════════════════════════════════════════════════════════════
print("\n" + "═"*60)
print("   A.S.S.A.R  —  PROJECT SCORPION")
print("   AUTONOMOUS SEMANTIC SEARCH AND ACTION ROVER")
print("   BOOT SEQUENCE")
print("═"*60)

# Hardware acceleration
device = "cuda" if torch.cuda.is_available() else "cpu"
log_event("system", f"Vision compute: {'GPU — CUDA active' if device=='cuda' else 'CPU — consider GPU for better FPS'}")

# Neural network
log_event("system", "Loading YOLOv8n weights (downloads ~6MB on first run)...")
model = YOLO("yolov8n.pt")
log_event("system", "YOLOv8n ready")

# Bluetooth serial
with config_lock:
    _port = config["com_port"]

if _port == "AUTO":
    _port = find_bluetooth_port()

if _port:
    ok = rover_serial.connect(_port, 9600)
    with state_lock:
        rover_state["serial_connected"] = ok
        rover_state["bt_port"]          = _port if ok else "FAILED"
    if not ok:
        log_event("system", "WARNING: Bluetooth connect failed")
        log_event("system", "  → Check that HC-05 is paired in Windows Bluetooth settings")
        log_event("system", "  → Change com_port in config or set via dashboard")
        log_event("system", "  → Will auto-retry every 5 seconds in background")
        # Start reconnect loop automatically
        threading.Thread(target=rover_serial._reconnect_worker, daemon=True).start()
else:
    log_event("system", "WARNING: No Bluetooth port found — running vision-only mode")

# Camera
with config_lock:
    _cam = config["camera_url"]
log_event("system", f"Connecting to camera: {_cam}")
log_event("system", "  → Make sure phone and laptop are on the SAME Wi-Fi network")
cap = VideoStream(_cam).start()
time.sleep(2.0)

# Flask bridge — this is deliberately the LAST thing that starts, so the
# dashboard only comes online once Bluetooth has at least been attempted
# (see "starts after bluetooth connection"). The dashboard's own boot
# screen additionally polls /status and waits for serial_connected
# before unlocking full controls — see templates/dashboard.html.
threading.Thread(target=_start_flask, daemon=True).start()
time.sleep(0.4)

_lan_ip = get_lan_ip()
log_event("system", f"Dashboard running at http://localhost:{DASHBOARD_PORT}  (this laptop)")
log_event("system", f"Dashboard running at http://{_lan_ip}:{DASHBOARD_PORT}  (phone — same Wi-Fi)")
if QR_AVAILABLE:
    log_event("system", f"Scan the QR code on the dashboard boot screen to open it on your phone")
else:
    log_event("system", "Tip: pip install qrcode to get a scannable QR code on the boot screen")

speak("Systems active. Target detection online.")
log_event("system", "ASSAR Scorpion fully operational", announce=True,
          speech="Systems active. Target detection online.")
print("═"*60)
print(f"   LAPTOP:  http://localhost:{DASHBOARD_PORT}")
print(f"   PHONE:   http://{_lan_ip}:{DASHBOARD_PORT}   (same Wi-Fi network)")
print("═"*60 + "\n")


# ══════════════════════════════════════════════════════════════
# 10. MAIN CONTROL LOOP
# ══════════════════════════════════════════════════════════════
current_state   = "SEARCHING"
last_state      = ""
last_cmd_time   = 0.0
last_ping_time  = 0.0

# Evasion state
evading              = False
evasion_end_t        = 0.0
post_evasion_turn    = False        # True during the turn-after-reverse phase
post_evasion_end_t   = 0.0
post_evasion_dir     = b"L"         # random L or R chosen at evasion trigger

# Target grace period (prevents jitter when target flickers for 1-2 frames)
last_target_time     = 0.0          # last time we actually saw a target
last_target_command  = b"S"         # last command issued while tracking

# Search scan state (rover rotates to find lost target)
search_phase         = 0            # 0 = stop, 1 = rotate L, 2 = rotate R
search_phase_end_t   = 0.0

# FPS counter
fps_count  = 0
fps_timer  = time.time()

target_found = False

try:
    while not shutdown_event.is_set():
        frame = cap.read()
        if frame is None:
            time.sleep(0.05)
            continue

        # ── LATENCY FIX: smaller frame = faster YOLO inference ────
        frame_w = cv2.resize(frame, (320, 240))
        frame_h = frame_w.shape[0]
        cx      = frame_w.shape[1] // 2      # horizontal center line
        now          = time.time()
        command      = b"S"
        target_count = 0   # NEW: reset each frame, only YOLO branch sets it > 0

        # ── Snapshot config (thread-safe, once per frame) ─────────
        with config_lock:
            conf          = config["confidence_threshold"]
            obs_cl        = config["obstacle_classes"]
            tgt_cl        = config["target_class_id"]
            margin        = config["center_margin"]
            stop_area     = config["stop_area_threshold"]
            throttle      = config["command_throttle"]
            ev_dur        = config["evasion_duration"]
            pe_turn_ms    = config["post_evasion_turn_ms"]
            grace_ms      = config["target_grace_ms"]
            search_rot_ms = config["search_rotate_ms"]
            ping_ivl      = config["ping_interval"]
            manual_timeout = config["manual_command_timeout"]

        with state_lock:
            mode = rover_state["mode"]

        # ══════════════════════════════════════════════════════════
        # MANUAL MODE
        # ══════════════════════════════════════════════════════════
        if mode == "manual":
            # NEW (Scorpion): sticky command + fail-safe timeout.
            # The old code consumed manual_command[0] once and reset it to
            # None, so it relied on the browser re-POSTing faster than the
            # vision loop iterates — in practice the loop usually wins
            # that race, and driving would stutter (forward-brake-forward)
            # instead of moving smoothly. Now the dashboard just keeps
            # re-sending the held direction (~every 150ms) and we hold
            # that value here until it goes stale.
            with manual_lock:
                mcmd     = manual_command[0]
                mcmd_age = now - manual_command_time[0]

            if mcmd_age > manual_timeout:
                # No fresh command in time — closed tab, dropped Wi-Fi,
                # locked phone, etc. Fail safe to Stop.
                command = b"S"
            else:
                command = mcmd

            # The actual Bluetooth send happens once, below, in the
            # shared "SEND COMMAND (throttled)" block — see the note
            # there for why this used to be duplicated per-branch.
            current_state = MANUAL_STATE_MAP.get(command, "MANUAL_STOPPED")
            target_found  = False

        # ══════════════════════════════════════════════════════════
        # AUTONOMOUS MODE
        # ══════════════════════════════════════════════════════════
        else:
            target_found  = False
            obstacle_hit  = False

            # ── PRIORITY 1: POST-EVASION TURN ─────────────────────
            # After reversing, rover turns to clear the obstacle path.
            if post_evasion_turn:
                if now < post_evasion_end_t:
                    command       = post_evasion_dir
                    current_state = "CLEARING"
                else:
                    post_evasion_turn = False
                    current_state     = "SEARCHING"
                    log_event("state", "Path cleared — resuming search")

            # ── PRIORITY 2: ACTIVE EVASION (reverse) ──────────────
            elif evading:
                if now < evasion_end_t:
                    command       = b"B"
                    current_state = "EVADING"
                else:
                    # Evasion reverse done → start post-evasion turn
                    evading           = False
                    post_evasion_turn = True
                    post_evasion_dir  = random.choice([b"L", b"R"])
                    post_evasion_end_t = now + (pe_turn_ms / 1000.0)
                    command           = post_evasion_dir
                    current_state     = "CLEARING"
                    log_event("state", f"Reverse done — turning {post_evasion_dir.decode()} to clear path")

            # ── PRIORITY 3: YOLO INFERENCE + TRACKING ─────────────
            else:
                results = model(frame_w, verbose=False, device=device, conf=conf)

                # ── Find best (closest = largest) target ──────────
                best_target  = None   # (area, x1, y1, x2, y2, ocx)
                best_area    = 0
                target_count = 0     # NEW: how many target-class boxes this frame

                for result in results:
                    if obstacle_hit:
                        break
                    for box in result.boxes:
                        cls            = int(box.cls[0])
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        ocx            = (x1 + x2) // 2
                        barea          = (x2 - x1) * (y2 - y1)

                        # ── Obstacle detection ─────────────────────
                        if cls in obs_cl:
                            cv2.rectangle(frame_w, (x1,y1),(x2,y2), (0,130,255), 2)
                            cv2.putText(frame_w, "OBSTACLE",
                                        (x1, max(y1-6, 12)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,130,255), 1)
                            if barea > stop_area:
                                obstacle_hit      = True
                                evading           = True
                                evasion_end_t     = now + ev_dur
                                command           = b"B"
                                current_state     = "EVADING"
                                speak("Obstacle detected. Evading.")
                                log_event("obstacle", f"Obstacle evasion triggered (area={barea})",
                                          announce=True, speech="Obstacle detected. Evading.")
                                break

                        # ── Target candidate — pick largest later ──
                        elif cls == tgt_cl:
                            target_count += 1
                            cv2.rectangle(frame_w, (x1,y1),(x2,y2), (0,220,0), 2)
                            conf_val = float(box.conf[0])
                            cv2.putText(frame_w,
                                        f"TARGET {conf_val:.2f}",
                                        (x1, max(y1-6, 12)),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0,220,0), 1)
                            if barea > best_area:
                                best_area   = barea
                                best_target = (barea, x1, y1, x2, y2, ocx)

                # ── Process best target (closest person) ──────────
                if best_target and not obstacle_hit:
                    target_found    = True
                    last_target_time = now
                    barea, x1, y1, x2, y2, ocx = best_target

                    # Draw targeting reticle on best target
                    tcx = (x1+x2)//2
                    tcy = (y1+y2)//2
                    cv2.drawMarker(frame_w, (tcx, tcy), (0,255,0),
                                   cv2.MARKER_CROSS, 20, 2)

                    if barea > stop_area:
                        # Too close — trigger evasion
                        evading           = True
                        obstacle_hit      = True
                        evasion_end_t     = now + ev_dur
                        command           = b"B"
                        current_state     = "EVADING"
                        speak("Target proximity warning. Reversing.")
                        log_event("state", f"Too close to target (area={barea}) — reversing",
                                  announce=True, speech="Target proximity warning. Reversing.")
                    elif ocx < cx - margin:
                        command       = b"L"
                        current_state = "TRACKING_LEFT"
                        last_target_command = command
                    elif ocx > cx + margin:
                        command       = b"R"
                        current_state = "TRACKING_RIGHT"
                        last_target_command = command
                    else:
                        command       = b"F"
                        current_state = "APPROACHING"
                        last_target_command = command

                elif not obstacle_hit:
                    # No target detected this frame
                    grace_elapsed = (now - last_target_time) * 1000

                    if grace_elapsed < grace_ms:
                        # ── Grace period: hold last command briefly ─
                        command       = last_target_command
                        current_state = "TRACKING_GRACE"
                    else:
                        # ── Smart search: rotate to scan ───────────
                        search_elapsed = (now - search_phase_end_t) * 1000

                        if search_phase == 0:
                            # Just lost target — start rotating left
                            search_phase      = 1
                            search_phase_end_t = now + (search_rot_ms / 1000.0)
                            command            = b"L"
                            current_state      = "SCANNING_LEFT"
                        elif search_phase == 1:
                            if now < search_phase_end_t:
                                command       = b"L"
                                current_state = "SCANNING_LEFT"
                            else:
                                # Rotate right for twice as long (covers L side + center + R side)
                                search_phase      = 2
                                search_phase_end_t = now + (search_rot_ms * 2 / 1000.0)
                                command            = b"R"
                                current_state      = "SCANNING_RIGHT"
                        elif search_phase == 2:
                            if now < search_phase_end_t:
                                command       = b"R"
                                current_state = "SCANNING_RIGHT"
                            else:
                                # Full scan done — stop and wait
                                search_phase  = 0
                                command       = b"S"
                                current_state = "SEARCHING"

                # Reset search phase when target is found
                if target_found:
                    search_phase = 0

        # ══════════════════════════════════════════════════════════
        # SEND COMMAND (throttled)  —  NEW: now truly shared by both
        # modes. In the original file this block was duplicated once
        # inside MANUAL and once inside AUTONOMOUS (indented under the
        # `else:`), which never double-sent in practice but meant the
        # announcement dispatcher below could never run for manual-mode
        # states. Living here once, after the if/else, fixes both.
        # ══════════════════════════════════════════════════════════
        if (now - last_cmd_time) > throttle:
            rover_serial.send(command)
            last_cmd_time = now

        # ══════════════════════════════════════════════════════════
        # ANNOUNCE STATE CHANGES  —  shared by both modes (Scorpion)
        # Every phrase here is spoken (a) locally via pyttsx3 if
        # local_speech_enabled, through speak(), and (b) from whichever
        # browser has the dashboard open, via /logs -> Web Speech API,
        # because log_event(..., announce=True) tags the entry.
        # ══════════════════════════════════════════════════════════
        if current_state != last_state:
            announcements = {
                "SEARCHING":       "Target lost. Scanning area.",
                "SCANNING_LEFT":   "Scanning left.",
                "SCANNING_RIGHT":  "Scanning right.",
                "APPROACHING":     "Target acquired. Moving forward.",
                "TRACKING_LEFT":   "Turning left to track target.",
                "TRACKING_RIGHT":  "Turning right to track target.",
                "CLEARING":        "Clearing path.",
                "MANUAL_FORWARD":  "Moving forward.",
                "MANUAL_BACKWARD": "Moving backward.",
                "MANUAL_LEFT":     "Turning left.",
                "MANUAL_RIGHT":    "Turning right.",
                # Deliberately silent (not a gap — a design choice):
                #   EVADING          — speaks a more specific phrase
                #                      ("proximity warning" vs "obstacle
                #                      detected") at the trigger site
                #                      instead of a generic one here.
                #   TRACKING_GRACE   — transient hold (<~300ms) while a
                #                      target flickers; announcing it
                #                      would mean chattering on every
                #                      brief flicker.
                #   MANUAL_STOPPED   — the resting state between button
                #                      presses; would narrate "stopped"
                #                      after every single tap.
            }
            if current_state in announcements:
                phrase = announcements[current_state]
                speak(phrase)
                log_event("state", f"State → {current_state}",
                          announce=True, speech=phrase)
            last_state = current_state

        # ══════════════════════════════════════════════════════════
        # UPDATE SHARED STATE  (read by Flask /status)
        # ══════════════════════════════════════════════════════════
        fps_count += 1
        if now - fps_timer >= 1.0:
            fps_val   = fps_count / (now - fps_timer)
            fps_timer = now
            fps_count = 0
            with state_lock:
                rover_state["fps"] = round(fps_val, 1)

        with state_lock:
            rover_state["state"]            = current_state
            rover_state["evading"]          = evading
            rover_state["target_found"]     = target_found
            rover_state["connected"]        = True
            rover_state["serial_connected"] = rover_serial.is_open
            rover_state["link_confirmed"]   = rover_serial.link_confirmed
            rover_state["targets_detected"] = target_count

        # ══════════════════════════════════════════════════════════
        # RENDER HUD ON FRAME
        # ══════════════════════════════════════════════════════════
        # State colour
        if current_state == "EVADING":
            hud_col = (0, 0, 255)    # red
        elif current_state.startswith("SCANNING"):
            hud_col = (0, 200, 255)  # amber
        elif current_state == "APPROACHING":
            hud_col = (0, 255, 0)    # green
        else:
            hud_col = (200, 200, 200)

        cv2.putText(frame_w, f"STATE: {current_state}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, hud_col, 2)
        cv2.putText(frame_w, f"MODE: {mode.upper()}", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.50, (160,160,160), 1)
        cv2.putText(frame_w, f"BT: {'OK' if rover_serial.is_open else 'OFFLINE'}",
                    (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0,255,0) if rover_serial.is_open else (0,0,255), 1)

        # Center aim-line
        cv2.line(frame_w, (cx - margin, 0), (cx - margin, frame_h), (40,40,40), 1)
        cv2.line(frame_w, (cx + margin, 0), (cx + margin, frame_h), (40,40,40), 1)
        cv2.line(frame_w, (cx, 0), (cx, frame_h), (60,60,60), 1)

        # ══════════════════════════════════════════════════════════
        # PUSH ANNOTATED FRAME TO MJPEG STREAM
        # ══════════════════════════════════════════════════════════
        _, jpg = cv2.imencode(".jpg", frame_w, [cv2.IMWRITE_JPEG_QUALITY, 78])
        with _frame_lk:
            _frame_jpg = jpg.tobytes()

        # ══════════════════════════════════════════════════════════
        # LOCAL WINDOW DISPLAY  (press Q to quit) — optional (Scorpion)
        # ══════════════════════════════════════════════════════════
        # This window only ever appears on the laptop's own screen — not
        # in the dashboard, not on the phone. Off by default is NOT the
        # setting here (show_local_window defaults True, so behaviour is
        # unchanged unless you turn it off), but for phone-only or
        # headless runs you can disable it from the config panel.
        with config_lock:
            _show_win = config["show_local_window"]
        if _show_win:
            cv2.imshow("ASSAR Terminal", frame_w)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                log_event("system", "Keyboard shutdown initiated")
                break

# ══════════════════════════════════════════════════════════════
# 11. SAFE SHUTDOWN
# ══════════════════════════════════════════════════════════════
finally:
    print("\n" + "═"*60)
    log_event("system", "Safe shutdown in progress...")
    speak("Powering down.")
    log_event("system", "Shutting down", announce=True, speech="Powering down.")
    rover_serial.close()
    cap.stop()
    cv2.destroyAllWindows()
    _speech_q.put("__QUIT__")
    time.sleep(0.6)
    log_event("system", "ASSAR offline")
    print("═"*60 + "\n")
