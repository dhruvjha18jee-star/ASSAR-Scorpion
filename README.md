# ASSAR-Scorpion
This code is an upgrade to the previous version of the code for the Rover ASSAR
A web dashboard + control-logic upgrade for the ASSAR rover (Autonomous
Semantic Search and Action Rover). No electronics changed — same
Arduino Nano, same L298N motor driver, same HC-05 Bluetooth module, same
2-wheel drive, same phone-as-camera setup. `ASSAR\_Arduino\_Final.ino` does
not need to be re-flashed to use anything in this zip; it's included here
byte-for-byte as you already had it. There is also now one optional
firmware upgrade, `ASSAR\_Arduino\_Scorpion.ino` — see Optional upgrade
below — which you can skip entirely and everything still works.
Everything in this folder replaces `assar\_rover.py` with `assar\_scorpion.py`
plus one new file, `templates/dashboard.html` — the actual control page
that didn't exist before (see What changed below).
---
1. Install
```
pip install -r requirements.txt
```
If you get an "externally managed environment" error, either add
`--break-system-packages` to that command, or use a virtual environment
(recommended long-term, not required to get this working today).
GPU note (RTX 4050): if `torch.cuda.is\_available()` prints `False` on
boot, you likely have the CPU-only build of torch. Install the CUDA build
first — see the note at the bottom of `requirements.txt`.
2. Run
```
python assar\_scorpion.py
```
Boot order is unchanged from the original: neural network loads →
Bluetooth connects (or starts retrying in the background if it can't) →
camera connects → then the dashboard server starts. This is what
"starts after Bluetooth connection" means in practice — the dashboard
literally cannot come online before Bluetooth has at least been attempted.
3. Open the dashboard
On the laptop: `http://localhost:8888`
On your phone (same Wi-Fi as the laptop): the exact address is
printed in the terminal on boot, and shown on the dashboard's own boot
screen as text + a QR code (if you installed `qrcode`).
The boot screen shows live Bluetooth/camera connection status and moves
into the main dashboard automatically once Bluetooth is connected (or
after ~6 seconds you can enter anyway, e.g. for a vision-only test with
no Arduino attached).
---
What changed vs. the original `assar\_rover.py`
The original file already had a real Flask API — `/status`, `/logs`,
`/command`, `/mode`, `/config`, `/video\_feed` all existed and worked. What
it didn't have was any `@flask\_app.route("/")` — nothing ever served an
actual page, so there was no dashboard to open, on a phone or otherwise,
even though the code's own docstring called it one. That's the main gap
this fills. Everything else below fell out of actually building and
testing a UI on top of that API.
New
The dashboard page itself (`templates/dashboard.html`) — video feed,
live telemetry, mode toggle, manual D-pad, activity log, settings panel,
emergency stop, boot/connect screen with LAN URL + QR code.
Works from a phone browser, not just the laptop — same server, same LAN.
Voice narration now speaks from the browser (Web Speech API) —
whichever device has the dashboard open, phone or laptop, uses that
device's own speaker. Independent of the fix below.
`/api/estop` — one-call emergency stop from any mode.
`/api/reconnect\_camera`, `/api/reconnect\_bt` — make editing
`camera\_url`/`com\_port` from the dashboard actually reconnect on the
spot (previously these values could be stored via `/config` but
nothing ever re-opened the camera or serial port with the new value —
it silently had no effect until the old connection happened to drop on
its own).
`/api/net\_info`, `/qr.png` — LAN IP + optional QR code for fast phone
pairing, so you don't need to run `ipconfig` yourself.
`/api/shutdown` — clean stop from the dashboard, for when you're
driving from your phone and the laptop isn't within reach.
`target\_class\_id` and `obstacle\_classes` are now actually editable from
the dashboard. They were already sitting in the `config` dict in the
original file, but `/config`'s POST handler had no code path for either
of them, so changing them was never actually possible before.
`targets\_detected` in `/status` is now a real number (count of
target-class detections in the current frame). It was declared in
`rover\_state` in the original file but nothing ever wrote a value into
it, so it always read `0`.
Fixed
Voice only ever said one phrase. pyttsx3 on Windows very commonly
only produces audio on the first `engine.say()+runAndWait()` call
when you reuse one engine across a session — every call after that
silently no-ops. That's a known pyttsx3/SAPI5 issue, not something
specific to this code, and it matches exactly what was reported (only
"Systems active..." was ever heard). Fixed by creating a fresh engine
per phrase instead of reusing one.
Several states never had a phrase to begin with — `TRACKING\_LEFT`,
`TRACKING\_RIGHT`, `CLEARING`, and mode switches were never in the
announcement table at all, and manual-mode driving had no per-direction
states, so the "which direction it's going" narration genuinely
couldn't have worked before. See Voice below for the full list of
what's now spoken.
`evasion\_duration` was declared as `1.5` (seconds) but the `/config`
POST handler had it listed as an integer field, so posting a value from
a dashboard would have silently truncated `1.5` → `1`. Never surfaced
before because there was no dashboard to post from — now that there is,
it's fixed (moved to the float fields).
Manual driving would have stuttered. The original manual-mode logic
consumed one queued command and reset it to `None` — it relied on the
browser re-POSTing a held button faster than the vision loop iterates,
which the loop usually wins, so commands would flicker back to the
default Stop between presses. Also never surfaced before because there
was no frontend to exercise it. Now the dashboard just keeps re-sending
the held direction (~every 150ms) and the backend holds that value
until it's stale — see Manual mode & safety below.
Unchanged (verified, not just assumed) — the YOLO detection/tracking
math, evasion behaviour, search-scan pattern, Bluetooth auto-reconnect,
camera auto-reconnect/buffering, and the Arduino command protocol
(`F`/`B`/`L`/`R`/`S`/`P`) are all exactly as they were.
---
Manual mode & safety
Tap MANUAL to unlock the D-pad (or press W/A/S/D or the arrow keys on
a laptop). Hold a direction to drive; release to stop. Under the hood:
Holding a button re-sends that command roughly every 150ms.
If the dashboard doesn't send anything for `manual\_command\_timeout`
seconds (0.5s by default, tunable in Settings), the rover stops on its
own — closed browser tab, locked phone, dropped Wi-Fi, anything.
Space bar or the on-screen ⚠ EMERGENCY STOP button forces Stop
and switches to manual mode immediately, regardless of what mode you
were in, in a single request.
The Arduino's own 1.5s no-signal watchdog (unchanged, in the `.ino`) is
still there underneath all of this as a last line of defence.
Voice
Two independent voice paths, both start on by default:
Browser voice (new) — speaks from whichever device has the
dashboard open, via the Web Speech API. Mute it with the 🔊 toggle on
the dashboard (saved per-browser).
Laptop voice (pyttsx3, now fixed) — off by default in Settings →
System. Turn it on if you want the laptop to narrate out loud even
with no browser open at all. If you turn it on and you're viewing
the dashboard on that same laptop, you'll hear each phrase twice —
mute one of the two.
What's actually narrated: target lost/scanning left/scanning right/target
acquired moving forward/turning left or right to track/clearing path
(autonomous), moving forward/backward/turning left/right (manual, only on
a direction change, not every tap), mode switches, emergency stop, and
boot/shutdown. Deliberately not narrated: the brief target flicker
hold state and "stopped" between manual taps — both would just be
chatter.
iPhone/iPad: Safari requires one on-screen tap before it allows any
programmatic speech. The dashboard does this automatically the moment you
tap anything on the boot screen — you shouldn't need to think about it,
but if voice seems silent on iOS, tap the 🔊 icon once.
---
Settings panel reference
Setting	What it does
COM port / Camera URL	Same fields as before, now with working Reconnect buttons
Confidence threshold	YOLO detection confidence cutoff
Target class	What ASSAR tracks — person by default, or a curated list of other COCO classes
Obstacle classes	What autonomous mode steers around
Center margin, stop area threshold	Tracking-box tolerance
Command throttle	Min. time between Bluetooth writes
Evasion duration	How long it reverses when evading
Manual drive fail-safe	The 0.5s timeout described above
Local window	The old `cv2.imshow` debug window — only visible on the laptop itself
Laptop speaker narration	The pyttsx3 path described above
All of these already existed as `config` dict values that were "editable
via web dashboard" per the original file's own comments — the two class
fields and the two new toggles are the only genuinely new settings; the
rest are the same knobs, just now actually reachable.
---
Optional upgrade: real Arduino link confirmation
By default, "BT: PORT OPEN" only means the Windows COM port opened — it
doesn't prove an Arduino is actually alive and receiving on the other end.
`ASSAR\_Arduino\_Scorpion.ino` closes that gap: after acting on a valid
command, the Arduino echoes that exact byte back over Bluetooth, and
Python watches for it in a background thread. Once seen, the dashboard's
BT indicator advances from PORT OPEN to LINKED, and drops back to
PORT OPEN if no ack arrives for 3 seconds (the Arduino's own ping keeps
this fed roughly once a second even when idle, so 3s gives comfortable
margin without taking long to notice a real problem).
This is entirely optional and fully backward compatible. Everything
in this zip — dashboard, manual mode, voice, all of it — works completely
normally with your existing `ASSAR\_Arduino\_Final.ino`. If you don't
re-flash, the indicator simply stays at "PORT OPEN" forever, which is
normal and does not mean anything is broken. To enable it: flash
`ASSAR\_Arduino\_Scorpion.ino` instead (same wiring, same board, same
upload process you already know) — no Python-side setup needed, it
detects acks automatically once they start arriving.
How this was verified, honestly: I compiled this sketch for real —
installed `gcc-avr`/`avr-libc` and the actual Arduino AVR core, and built
it against an ATmega328P target at 16MHz (the Nano's real MCU/clock),
producing a clean `.hex` with zero warnings and zero errors. I also
tested the Python side for real, against a simulated serial link that
behaves like the Arduino would (echoing valid commands, going silent to
simulate a dead board, coming back) — confirm/timeout/recovery all
behaved correctly with no false positives across a long run. What I
can't verify without your actual board is real-world Bluetooth timing —
the header comment in the .ino explains the specific timing reasoning
(SoftwareSerial's ~1ms write blocking a receive window that's normally
80ms+ between commands) and why it should be fine, but "should be fine"
on paper isn't the same as your actual HC-05 link. Worth confirming once
before you rely on it for anything important like a live demo.
---

PWM speed control. ENA/ENB are currently jumpered to always-on, so
the rover is full-speed-or-stop. Real speed control means moving ENA/ENB
off the jumper caps onto two PWM pins (e.g. D3/D11) and updating the
Arduino sketch to use `analogWrite`. This is what would make the
`approach\_slow\_area` value in `config` (already present, currently
inert) actually do something — it's clearly a placeholder for exactly
this, and I've left it as-is rather than wire up fake behaviour on
motors that can't actually go slow yet.
Ultrasonic obstacle sensing (HC-SR04 or similar) — your presentation
already lists this as planned future work, as a complement to (not
replacement for) the current vision-based obstacle avoidance.
Battery voltage monitoring — a simple resistor voltage divider into
a spare analog pin, also already on your own future-upgrades list.
Chassis rebuild notes — foam board (5mm)
Since you mentioned the rebuild will use 5mm foam board (like the
Scorpion Eye reference image) instead of cardboard, a few practical notes
since foam board behaves differently under a cutting blade and glue than
cardboard does:
Score, don't fold, for the tail curve. Foam board cracks the foam
core if you bend it sharply like cardboard. For a curved tail, either
cut it from a single piece following the curve (wasteful but strongest),
or score a shallow V-groove partway through the top paper layer along
the fold line and bend gently along that groove — it folds cleanly
without cracking.
Don't mount hardware directly into bare foam. Screws and standoffs
pull straight out of foam core under vibration. Use a small square of
thin plywood/acrylic (or even doubled-up foam board) as a mounting pad
under the Arduino, L298N, and breadboard, glued to the foam board, and
screw into that instead.
Weight distribution matters more with 2-wheel drive. With only two
driven wheels and (presumably) a passive caster/skid at the back or
front, keep the battery and the heaviest single item (usually the
breadboard+wiring bundle) centered low over the drive axle, not out
over the tail — the reference image's tail is hollow/lightweight by
design for exactly this reason, it's not meant to carry weight.
Phone slot ("eye"): keep it vertical and roughly centered, angled
back a few degrees rather than perfectly vertical — a slight backward
tilt keeps a standing/seated tracked person centered in frame instead
of the camera looking at their feet from a low chassis.
Hot glue over white glue for foam board — it bonds faster and holds
better on the paper facing; white glue can warp the paper skin as it
dries.
I'm intentionally not giving exact cut dimensions — I don't have your
actual breadboard/L298N/battery footprint measured, and guessing specific
numbers would be more likely to waste a sheet of foam board than help.
---
