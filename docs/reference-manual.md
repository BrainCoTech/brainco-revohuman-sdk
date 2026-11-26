# RevoHuman SDK Reference Manual

English | [简体中文](zh-CN/RevoHuman%20SDK%20参考手册.md)

> For installation and getting started, the [README](../README.md) at the root of the source bundle is authoritative; this manual is the complete reference.
> Paths in this manual (`examples/`, `ros2/`, `simulation/`, etc.) are relative to the source bundle root `revohuman-<version>/`;
> where SDK code comments and error messages mention "README section N", they mean the section with the same number in this manual.

Data capture SDK for the RevoHuman data glove. Two product models are supported:

| Product model | Composition |
|---|---|
| **P1** | Encoder glove (21 joints per hand) + Vive Trackers on both hands |
| **PTV1** | P1 + full-finger tactile (241 taxels) + wrist cameras |

The SDK automatically identifies the left and right hands and the product model from the USB serial numbers and the hardware present,
then assembles the matching sensor set for that product model, behind a single interface. There are two entry points; you can use just one of them:

| Entry point | Usage | Dependencies |
|---|---|---|
| Native Python | `from revohuman import open_kit` | `pyserial` + `PyYAML` (install extras for the rest as needed) |
| ROS 2 | `ros2 launch revohuman_bringup revohuman.launch.py` | Also requires ROS 2 Humble |

`import revohuman` does not load numpy, rclpy, opencv or libsurvive;
submodules such as Tracker, camera and recording are imported only when used.

---

## 1. Installation

The core SDK and the Tracker driver ship as per-platform wheels; everything else is one platform-independent source bundle.

| File | Contents |
|---|---|
| `revohuman-<version>-cp310-cp310-linux_x86_64.whl` | Core SDK, x86_64 |
| `revohuman-<version>-cp310-cp310-linux_aarch64.whl` | Core SDK, aarch64 (i.e. arm64 on Linux) |
| `pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_x86_64.whl` | Tracker driver libsurvive, x86_64 |
| `pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_aarch64.whl` | Tracker driver libsurvive, aarch64 |
| `revohuman-<version>-bundle.tar.gz` | Source bundle: ROS 2 packages, hand models (URDF), examples, docs, simulation, Docker image build files, one-step install script |
| `SHA256SUMS` | Checksums |

Runtime environment: Linux, CPython 3.10 (the version shipped with Ubuntu 22.04). The core SDK is provided as compiled modules;
a wheel installs only on the Python version and architecture named in its file name, and pip rejects a mismatched one.

```bash
sha256sum -c SHA256SUMS --ignore-missing
WHL=revohuman-0.1.4-cp310-cp310-linux_$(uname -m).whl
pip install "$WHL"                   # core: glove capture, identification, command line
pip install "$WHL[recording]"        # MCAP recording and playback (mcap, lz4)
pip install "$WHL[camera]"           # wrist camera image decoding (opencv; not needed for capture and timestamping themselves)
pip install "$WHL[ble]"              # base station scan / pairing / channel change (bleak; required when using the Tracker)
pip install "$WHL[viewer]"           # MuJoCo 3D visualization tool
tar xzf revohuman-0.1.4-bundle.tar.gz   # → revohuman-0.1.4/: ros2/ models/ examples/ docs/ simulation/ docker/ install_venv.sh calibrate_lighthouses.sh README.md README.zh-CN.md LICENSE NOTICE
```

- **One-step install**: when not using ROS 2, `install_venv.sh` at the root of the source bundle installs all of the above, together with the bundled pysurvive and the 3D visualization dependencies, into a separate virtual environment
  (default `~/.venvs/revohuman_sdk`), and as needed installs system dependencies and the Tracker device rules and adds the user to the `dialout` group; see *01 Installation and System Requirements* section 11.
  Requires CPython 3.10 (`--python` selects the interpreter).
- **Tracker** additionally needs the bundled libsurvive Python binding pysurvive 1.1.21+revohuman.3: `python3 -m pip install --user --no-deps pysurvive-1.1.21+revohuman.3-cp310-cp310-linux_$(uname -m).whl` (the bundled version contains driver fixes this SDK needs; do not install the version from PyPI; `--no-deps` must not be omitted, see step ② of the installation in the [README](../README.md)).
- **Serial port permission**: the current user must be able to read and write `/dev/ttyACM*` (usually by joining the `dialout` group).
- **Single-process exclusive access**: a glove, and a libsurvive setup, can each be opened by only one process at a time.
  A second process gets `PortBusyError` (with the holder's pid) or `TrackerBusyError`.

After installing, run a health check first:

```bash
python3 -m revohuman doctor            # read-only health check: glove units present and serial ports, camera port position on the same HUB, handedness, USB topology, leftover sessions
python3 -m revohuman doctor --probe    # also briefly opens the serial port to read the handedness stored in firmware and the firmware info (do not use on a glove that is capturing)
```

`doctor` does not check the Tracker; check the Tracker with `python3 -m revohuman tracker check --live` (section 4).

---

## 2. Hardware identification

### 2.1 Left and right hands

With firmware 0.20.3 and later, the handedness is stored in the firmware; on earlier gloves the USB serial number indicates the handedness (L = left hand, R = right hand):

| Serial number | Product model | Handedness |
|---|---|---|
| `BCRH1PL…` / `BCRH1PR…` | P1 | 7th character |
| `BCRH1TL…` / `BCRH1TR…` | PTV1 | 7th character |
| `BCRH1L…` / `BCRH1R…` (early batches) | Not indicated | 6th character |

The SDK identifies the hand in the following order, entirely read-only:

1. **Handedness stored in firmware**: read once by briefly opening the serial port on connect, `scan(probe=True)` and `doctor --probe`; the result is cached in
   `/tmp/revohuman/glove_hand_register.json` (re-read after replugging or reboot). The serial numbers of a later batch of gloves (`BCE1M…`) carry no handedness and rely on this alone;
2. **Serial number**: trusted when it carries L/R and no other MCU present claims the same hand;
3. Not identifiable (no handedness stored in firmware, or both gloves of an early batch have the same serial number): connecting by handedness is refused, and **the SDK does not guess from enumeration order**; specify by serial port instead, see 2.3.

If the handedness stored in an MCU's firmware contradicts that MCU's serial number, the connection is refused (`hand_source == "conflict"`; `doctor` reports fail).

Tracker serial number for each hand: taken from the Tracker actually plugged into the same USB HUB; when it is not on the glove's HUB, the caller gives it explicitly (2.3).
The wrist camera works the same way and follows the HUB the glove is on.

### 2.2 Product model (P1 / PTV1)

Three pieces of evidence are collected for each hand, and they must agree:

| Evidence | Indicates PTV1 | Indicates P1 |
|---|---|---|
| Serial number (`BCRH1T…` / `BCRH1P…`, or `products.yaml` rules) | Points to PTV1 | Points to P1 |
| Tactile module reported by the firmware | Tactile present | No tactile |
| Wrist camera on the same USB HUB | Present | Absent |

- Contradictory evidence, or different conclusions for the left and right hands: `ProductUnidentified` is raised, listing every conflict.
- A product model declared explicitly (`product="PTV1"`) that the evidence contradicts: `ProductMismatch` is raised.
- The declared value is used only when there is no usable evidence at all (`source="declared"`).

The SDK has two built-in serial number rules: `BCRH1P[LR]` → P1, `BCRH1T[LR]` → PTV1; serial numbers of early batches do not indicate the product model
and are judged by hardware evidence only. To supply more rules, write them in `~/.config/revohuman/products.yaml` (or point to the file with the environment variable
`REVOHUMAN_PRODUCTS`); each regex is matched from the start of the serial number, and these rules are checked before the built-in ones:

```yaml
rules:
  - match: "BCRH2"     # regex, re.match
    variant: PTV1
    note: 可选说明       # optional note
```

To view the identification result:

```bash
python3 -m revohuman product identify            # automatic identification (briefly opens the serial port to read the tactile model)
python3 -m revohuman product identify --no-probe # USB topology and serial numbers only; does not open the serial port
python3 -m revohuman product identify --product PTV1 --json
```

`--no-probe` does not open the serial port and so cannot read the tactile model: on a product model with tactile (PTV1) it reports the hand as missing tactile (`未探测`, not probed), with exit code `1`.
Use it only to check the USB topology and serial numbers; the product model is whatever the run without `--no-probe` reports.

To get the hand mechanism model name for a product model in code:

```python
from revohuman.product import variant_hand_model
variant_hand_model("P1")     # "p"
variant_hand_model("PTV1")   # "dv1"
```

The product model is written the same way as for `--product` (case-insensitive); you can also read `P1.hand_model` / `PTV1.hand_model` directly.
An unrecognized product model, `"auto"` and the empty string raise `ValueError` rather than falling back to a default model: identify the product model first, then look it up.

### 2.3 When handedness cannot be identified

Handedness is taken only from the MCU firmware (stored handedness, serial number). When the firmware has no stored handedness, both gloves of an early batch have the same serial number, or both MCUs present claim the same hand,
connecting by handedness raises `UnregisteredGloveError` with the reason, and **the SDK does not guess from the port or enumeration order**. In that case the caller specifies by serial port:

```python
from revohuman import GloveManager
mgr = GloveManager()
pair = mgr.connect_pair(left_port="/dev/ttyACM0", right_port="/dev/ttyACM1")  # left/right labeled by argument position
glove = mgr.connect(port="/dev/ttyACM0")                                     # single hand: frames carry no handedness
```

Serial ports and serial numbers come from a scan:

```python
for d in GloveManager().scan():
    print(d.usb_topology_id, d.port, d.usb_serial, d.handedness, d.hand_source, d.tracker_serial)
```

When the Tracker is not plugged into the glove HUB (wireless receiver), the SDK has no way of knowing which hand it belongs to; give it explicitly with `--tracker left=LHR-... --tracker right=LHR-...`
(in Python, `SurviveTrackerSource(registry={serial: hand})`).
When the hand to be opened is not present and the serial numbers of the gloves present all identify the other hand, `GloveNotPresent` is raised.
When the handedness stored in an MCU's firmware contradicts that MCU's own serial number, `HandConflictError` (error code 1403) is raised; again no guess is made, and the remedy is the same as above.

### 2.4 Firmware upgrade

Command line only, one glove at a time. Before writing starts, the firmware file and the device state are checked, and you must type `yes` to confirm:

```bash
python3 -m revohuman firmware info                                      # read-only: firmware version and upgrade state of the gloves present
python3 -m revohuman firmware upgrade app.bin --version 1.3.0 --hand left
python3 -m revohuman firmware upgrade app.bin --version 1.3.0 --port /dev/ttyACM0 --yes   # --yes is required in non-interactive environments
```

- Only raw `.bin` files are accepted. A `.bin` carries no reliable version information, so `--version` must be written exactly as given by the firmware publisher; a wrong value makes the final version check report failure.
- If the serial port is in use (most likely by capture), the upgrade is refused; stop capture first. Downgrades are refused by default; to really downgrade, add `--allow-downgrade`;
  this option only lifts the check on the SDK side, and the glove firmware itself may still refuse the downgrade (reporting `设备不接受这份固件`, the device does not accept this firmware).
- If `--version` equals the glove's current version, the firmware is reflashed as usual (transfer, verification and activation all run), and the glove firmware itself may still refuse.
  With the same version number, the final version check cannot tell the previous firmware from the flashed one: the result only says that the device has verified and activated it, the JSON includes `"same_version": true`,
  and `"rebooted"` indicates whether the glove was observed to reboot (dropped off and reappeared, or the serial port number changed).
- If the transfer fails partway or you press Ctrl+C, the transfer is cancelled and the glove keeps running the old firmware.
- After activation the glove reboots automatically; the command finds the same glove again by its physical USB position and checks the version (the ttyACM number may change after reboot),
  waiting 45 seconds by default (`--reconnect-timeout`). **Do not unplug or power off before the command reports completion**; if it times out without confirmation, check with `firmware info`.

### 2.5 Encoder zero pose calibration

Gloves leave the factory with the encoder zero calibrated; everyday use does not require recalibration. Calibration records the glove's **current posture** as each encoder channel's zero and writes it to the glove,
**replacing the current zero (including the factory zero); this cannot be undone**. It is unrelated to the Tracker zero pose (4.3).

```bash
python3 -m revohuman glove calibrate --hand left                          # all online encoders; type yes to confirm
python3 -m revohuman glove calibrate --hand right --index 3 --index 4     # only channels 3 and 4 (numbered 0–20)
python3 -m revohuman glove calibrate --port /dev/ttyACM0 --yes            # --yes is required in non-interactive environments
```

- The zero pose is five fingers straight and the hand laid flat; after calibration, each channel of `joints_deg` reads about 180° in this posture.
- First put the glove in the zero pose and hold it still, then run the command; do not move until calibration completes. Calibrate one hand at a time.
- If the serial port is in use (most likely by capture), calibration is refused; stop capture first. If the target glove is not present, it is refused, and no guess is made from port order.
- On success it prints the raw zero value of each encoder channel (`zero_raw` in `--json`), with exit code `0`.
- If any encoder failed to calibrate, or it was not confirmed that the zero was saved to the glove, the exit code is `1` and the succeeded and failed encoders are listed;
  do not treat partial success as done: put the glove in the posture and calibrate again.
- The same operation can also be started from Python (`glove.calibrate()`, see 5.1) or ROS 2 (the `calibrate` service, not created by default, see 7.4).

---

## 3. Quick start

```python
import queue
from revohuman import open_kit

with open_kit() as kit:                        # identify the product model and assemble all sensors
    print(kit)                                 # RevoHumanKit(PTV1, hands=['left', 'right'], sensors=[...])

    # the zero pose was recorded automatically at startup (put both hands in the zero pose before starting, see 4.3); call kit.set_reference() to record it again
    sub = kit.synchronizer().subscribe()       # bimanual paired frames
    pair = sub.recv(timeout=1.0)
    print(pair.skew_ns, pair.left.joints_deg[:3])

    delta = kit.tracker.delta("right")         # pose of the right-hand tracker relative to the zero pose
    if kit.has("wrist_camera"):
        frame = kit.camera("right").latest()   # PTV1: latest camera frame

    rec = kit.recorder()                       # already wired to all data streams of this kit
    handle = rec.start("episode_001.mcap")
    ...
    print(handle.stop())                       # {"frames": ..., "per_channel": {...}}
    rec.close()
```

A runnable version: [`examples/kit_quickstart.py`](../examples/kit_quickstart.py).
Before using the Tracker, the base stations must first be put into the allowlist and the geometry calibrated once; see section 4.

### `open_kit()` parameters

| Parameter | Default | Description |
|---|---|---|
| `product` | `"auto"` | `"auto"` identifies from evidence; `"P1"` / `"PTV1"` is a declaration, refused if the hardware contradicts it |
| `hands` | `"both"` | `"both"` (bimanual hardware sync) / `"left"` / `"right"` |
| `sensors` | All of the product model's | Enable only some kinds, e.g. `{"encoder", "tactile"}`; must include `encoder` and cannot go beyond the product model |
| `strict` | `True` | Sensors the product model should have but whose hardware is missing: `True` raises `ProductMismatch`; `False` skips them and records them in `kit.missing` |
| `rate_hz` | `200` | Glove data rate |
| `sync` | `True` | Bimanual hardware sync: `True` (raise on failure) / `"best-effort"` / `False` |
| `tracker` | `None` | Arguments passed to `TrackerRuntime`, e.g. `{"gate_mode": "flag"}` |
| `camera` | `None` | Arguments passed to `CameraConfig` (resolution, frame rate, exposure, etc.); the device path is filled in by the kit per glove unit |
| `rules_path` | Default path | Path to `products.yaml` |
| `on_fault` | `None` | Runtime fault callback `cb(exc)` (4.4) |

Sensor names: `encoder`, `tracker`, `tactile`, `wrist_camera`.

### `RevoHumanKit`

| Member | Description |
|---|---|
| `product` | Identification result: `variant`, `source` (`sn` / `hardware` / `declared`), the hardware present for each hand |
| `variant` / `sensors` / `hands` / `missing` | Product model / sensors enabled this time / hands opened / sensors missing in non-strict mode |
| `has(sensor)` | Whether that sensor is enabled |
| `glove(hand)` / `camera(hand)` | A single glove (`Glove`) / wrist camera (`WristCamera`) |
| `synchronizer()` | Bimanual pairer (`BimanualSynchronizer`); repeated calls return the same one |
| `tracker` | `TrackerRuntime` (4.6); `None` when the Tracker is not enabled |
| `set_reference()` | Record the zero pose again (already recorded automatically at startup); same as `kit.tracker.set_reference()` |
| `geometry()` | Base station geometry gate verdict; do not capture when `ok` is false |
| `calibrate_geometry(timeout_s=None)` | Calibrate the base station geometry in a subprocess; the runtime session is reopened automatically afterwards |
| `recorder(...)` | Returns a `TopicRecorder` wired to all data streams (5.5) |
| `status()` | Dict of component states, including `fatal` |
| `fatal` / `raise_if_fatal()` | Latched unrecoverable fault (4.4); `None` / does not raise when there is none |
| `close()` | Close all components; supports the `with` statement |

Assembly order: Tracker static check (S0 in 4.2, reads files only; if it blocks, the gloves are never opened) → gloves → product model identification → Tracker → cameras; if any step fails, the parts already opened are closed before the exception is raised.

---

## 4. Tracker and base stations

Tracker poses are referenced to the base stations. A site must first be prepared: put your own base stations into the allowlist → scan to confirm and rule out same-channel conflicts → calibrate the base station geometry.
The geometry is saved to disk and reused automatically at later startups; recalibration is needed only after base stations are moved, replaced, added or removed, or have their channel changed. The zero pose is recorded automatically at every startup (see 4.3).
Calibration also fixes the coordinate frame the Tracker poses are expressed in; see 4.7 for its definition.

**Bluetooth required.** Base station serial numbers and channels can only be read over Bluetooth. Scanning, changing channels and blinking all require a working Bluetooth adapter on this machine and the `ble` extra installed.
Bluetooth is used only by the `revohuman-lighthouses` command; SDK startup and runtime never touch Bluetooth.

### 4.1 Deployment procedure

```bash
# ① Scan nearby base stations: serial number / advertised name / channel / signal (a base station seen for the first time is connected once, only to read its serial number)
revohuman-lighthouses scan

# ② Write the config file from the example (left untouched if it already exists; its path, <上一步打印的路径> below, is printed), then, checking the labels
#    on the back of the base stations, replace the placeholders with the 12-character serial numbers (starting with FA) of your own base stations
#    (${EDITOR:-nano} uses nano when EDITOR is not set; if neither is available, use any text editor, or first sudo apt install nano)
revohuman-lighthouses init
${EDITOR:-nano} <上一步打印的路径>

# ③ Scan again: every station in the allowlist must show ✓, with no same-channel conflict (a pass also refreshes the channel preflight check, valid for 5 minutes)
revohuman-lighthouses scan

# ③′ Only if ③ reports a same-channel conflict: review the automatic plan (computed, not written) → apply after confirming → back to ③
revohuman-lighthouses plan
revohuman-lighthouses apply --yes

# ④ Calibrate the base station geometry (the first time, or after base stations have moved or channels have changed)
python3 -m revohuman tracker calibrate

# ⑤ Startup self-check (optional): reads files only by default; --live actually starts a capture and runs both stages of the checks in 4.2
python3 -m revohuman tracker check --live

# Offline view at any time: allowlist, preflight check, geometry, zero pose, verdict of the last startup check
python3 -m revohuman tracker status
```

Once the allowlist is filled in, the scan in ③, the calibration in ④ and `check --live` in ⑤ can be done with one command, `calibrate_lighthouses.sh` at the root of the source bundle
(it only reads the base stations, stops on a conflict without running ③′, and runs inside the container when `--docker` is given on the host); see *03 Base Stations and Pairing* section 5.4.

`revohuman-lighthouses` and `python3 -m revohuman tracker lighthouses` are the same command.

**Config file**: the base station allowlist and the calibration thresholds are in the same YAML file, written by hand; the SDK only reads it. Only the first file found in the following order is used,
with no merging across files; at startup each command prints, as its first line, which file is used this time and which layer it comes from:

1. `--whitelist <path>`
2. Environment variable `$REVOHUMAN_LIGHTHOUSES`
3. `revohuman_tracker.yaml` at the root of the source tree (this layer exists only when running from source; it does not exist after a pip install, and startup says so)
4. `~/.config/revohuman/lighthouses.yaml`

Files that exist but are not read (for example the one in the source tree once the environment variable is set, or the old file at 4 once a newer config exists) are named at startup.
`init` writes to the file that will be read afterwards (when running from source and neither location has a file yet, it writes 3), and does not overwrite an existing one.
For a complete example see `examples/revohuman_tracker.example.yaml`:

```yaml
lighthouses:
  - FAXXXXXXXXXX          # placeholder: replace with the serial number on the label on the back of the first base station
  - serial: FAYYYYYYYYYY  # second station
    place: 右后上方        # optional (here "upper rear right"); used in error messages to identify which station

calibration:              # optional: view coverage thresholds for calibration; command-line arguments take precedence
  coverage_cells: 2       # minimum number of direction cells covered (0 disables this criterion)
  coverage_span_deg: 5    # minimum angular span seen by the base stations, in degrees (0–120)
```

- Do not commit a file containing real serial numbers into git (the one at the source tree root is already in `.gitignore`).
- If the file is missing or the allowlist is empty, the startup check refuses outright (`whitelist_missing`) and suggests running `init` first.
- The `calibration:` section accepts only `coverage_cells` and `coverage_span_deg`; a misspelled key name, a non-numeric value or a span above 120° is refused with the line number.
  When this section is omitted, the code defaults (2 cells / 5°) apply, and the example shows exactly the defaults; raise them as needed after adding base stations.
  A single position already occupies 1 horizontal direction cell and 1 vertical direction cell at once, so with at most 2 cells this criterion is met from the start,
  and only the angular span actually matters. The span is computed over all Trackers combined; with the two hands placed apart left and right, it may be enough from the start.

- Do not enter the `LHB-xxxxxxxx` advertised name, and do not enter channel numbers; a wrong entry is reported with its line. At most 4 stations, no duplicates.
- The mapping between serial number and advertised name is read by `scan` and kept in the identity table in the state directory; each base station only needs to be read once per machine.
  If you do not want to connect to neighbors' base stations, use `scan --no-connect`: passive scan only; stations whose serial number has never been read show as `?`.

**Same-channel conflict**: two base stations on the same channel cannot be told apart, and base station poses are lost repeatedly. `plan` / `apply` only move stations in the allowlist:

- The channels occupied by other base stations seen in this scan (including those whose serial number could not be read), and by base stations seen optically at the most recent startup, are all avoided;
  optical records do not expire by time (the output notes how long ago they were seen); they are disregarded only when they have no overlap at all with the current allowlist (as if a whole different set of base stations were in use),
  and a line explaining this is printed;
- When you have confirmed that the site has changed and the base stations last seen optically are gone, add `--ignore-optical` to judge by this round's Bluetooth scan only;
  conflicts found by this round's Bluetooth scan are not affected by this switch and are still refused;
- Nothing is moved unless necessary, only the colliding stations; `--pin FA…=CH` sends a given station to a given channel, `--avoid CH` additionally avoids certain channels;
- By default the plan is only printed; only `apply --yes` writes; `apply` rescans and recomputes on its own and does not reuse an earlier `plan` result;
- Before writing, the serial number and current channel are checked within the same connection; after writing, the value is read back; once all writes are done, a rescan verifies again;
- After a successful move the channel preflight check becomes invalid: run `scan` again, and `calibrate` again.

Other operations:

- `set-channel FA… CH [--yes] [--ignore-optical]`: changes only the given station, and the target channel must be free; when refused, it says which station occupies the channel,
  and whether that station was seen in this round's Bluetooth scan or optically, and how long ago.
- `identify FA…|LHB-… [--seconds 5] [--scan-seconds 6]`: makes a base station blink for a few seconds (1–30 s) so you can tell which one it is on site;
  `--scan-seconds` is how many seconds to scan for this station's address before blinking.
- Base stations not in the allowlist: their channel is never changed; blinking one requires `--foreign` every time and is recorded in the write log.
- For centralized deployments, set `REVOHUMAN_LIGHTHOUSE_WRITE=0` to disable all write operations (channel changes and blinking); only plans are produced then.
- `python3 -m revohuman tracker preflight` is a quick version that does only the channel preflight check: passive scan only, no connection to any base station.

`revohuman-lighthouses` exit codes: `0` allowlist complete with no conflicts, or operation succeeded; `1` gaps, conflicts, a plan waiting to be applied, or a write failure;
`2` Bluetooth unavailable, another Bluetooth operation in progress, or a usage error.

**Calibration procedure**: place the Tracker in an area that all base stations can see, **move it to a different position and orientation → hold still for about 1 second → move again**,
and repeat until the command reports completion. Waving at a steady speed or staying still all the time yields no valid samples.

**Criteria for calibration completion** (held continuously for `--stable-sec` seconds):

- Every base station in the allowlist has received data, has a solved pose, the pose is non-zero, and the variance is within the limit (one station is allowed zero variance as the reference station);
- View coverage is sufficient: at least `--coverage-cells` direction cells and a span of at least `--coverage-span-deg`; each is taken, in order of precedence, from the command-line arguments,
  the `calibration:` section of the config file, and the code defaults (2 cells / 5°); the values used and their sources are printed when calibration starts;
- With multiple stations, the gravity directions solved for the base stations differ pairwise by no more than 10° (more than that is a bad solution and counts as failure);
- Base stations outside the allowlist must not be solved along with them: as soon as one appears, calibration fails without waiting for the timeout and rolls back immediately (the solved geometry has already been pulled off by it).

On success an "accepted geometry" record is written, and the startup check goes by it. On timeout (default 300 seconds), failure or abort, the geometry configuration is automatically rolled back to what it was before calibration.

**Foreign base station fence**: by default only the base stations in the allowlist are used and all others are masked. During capture, serial number and channel are also checked, and only allowlisted, calibrated base stations are accepted;
other people's base stations nearby, or base stations whose channel has changed, are not merged into the current coordinate frame. `--no-fence` disables the fence;
`--no-preflight` both skips the channel preflight check and disables the fence; for troubleshooting only.

### 4.2 Startup check

`open_kit()` (with the Tracker enabled) and `TrackerRuntime.start()` run the checks in two stages before handing over the handle, and raise
`TrackerNotReady` (code 1905) if they are not met. The exception carries `kind`, `reasons` (listed one by one, base stations referred to by serial number) and `remedy`
(a fix command chosen by `kind`, ready to copy and run); an ERROR log line is written before it is raised.

**Stage 1: static check.** Reads files only, takes milliseconds. `open_kit()` runs it before connecting the gloves, so if it blocks, the gloves are never started at all.

| `kind` | Meaning | Fix |
|---|---|---|
| `whitelist_missing` | Allowlist does not exist or is empty | `scan` → fill in the allowlist → `calibrate` |
| `whitelist_invalid` | Allowlist has a format error, duplicates, or too many stations | Fix the line indicated |
| `lighthouse_unresolved` | A station in the allowlist has never been seen in a scan on this machine | Run `scan` with that station powered on and someone nearby |
| `config_missing` | The base station geometry has not been calibrated yet (a first run lands here) | `scan` → `calibrate` |
| `config_unreadable` | Geometry file unreadable or corrupted | Check permissions; recalibrate if corrupted |
| `geometry_unsolved` | A base station in the allowlist has no solved pose | `calibrate` |
| `geometry_unaccepted` | The current geometry is not the one accepted by `calibrate` (calibration interrupted, edited by hand, or copied) | `calibrate` |
| `floor_offset` | Floor offset exceeds 1 m | `calibrate` |
| `geometry_inconsistent` | With multiple stations, the angle between base station gravity directions exceeds 10° (bad solution) | `calibrate` |

- A base station whose current channel differs from the one at calibration (`channel_changed`) only produces a warning, not a block, with a hint to `scan` again and `calibrate`.
- If geometry exists but there is no "accepted geometry" record, and all other checks pass, the geometry is adopted automatically once at the first startup, with a warning.

**Stage 2: check after streaming starts.** After capture has started and before the handle is handed over:

| `kind` | Meaning |
|---|---|
| `geometry_unavailable` | Within `ootx_wait_s` (default 25 seconds) an allowlisted base station produced no data (not powered, blocked), or the base station state could not be read |
| `lighthouse_wiped` / `lighthouse_moved` | A base station pose was cleared at startup: when the gravity direction broadcast by a base station changes by 2° or more, the driver clears that station's position and rewrites the base station configuration. The base station may have been knocked askew or moved, or it may just be reading jitter (this happens even when nobody touches it); the angles of the two cases overlap and cannot be told apart. With the bundled driver `lighthouse_moved` is usually reported, which does not necessarily mean anyone touched it; both are handled the same way: restarting does not help; `scan` again and `calibrate` |
| `tracker_no_pose` | A Tracker produced no pose within 5 seconds; the base stations it cannot see are attached |
| `reference_failed` | The automatic zero pose was not recorded |

Every startup logs one line `[tracker-gate] mode=... registered=... config=...`; the verdict is saved to `last_gate.json` in the state directory,
and `tracker status` shows the result of the "last startup check".

`gate_mode`: `error` (default, blocks) / `flag` (only writes to the log and `status()`, returns as usual) / `off` (no checks).

Fix commands are not subject to these checks: `tracker calibrate` does not run the startup check; otherwise, once the base stations had moved, they could never be calibrated back.

### 4.3 Zero pose

The zero pose is the reference for Tracker poses: `delta` is the change of the current pose relative to the moment the zero pose was taken (definition in 4.6, coordinate frame in 4.7).

- After the startup check passes, the zero pose is recorded once automatically from the pose at that moment, so **put both hands in the zero pose before starting**.
- When needed, call `kit.set_reference()` / `rt.set_reference()` or `tracker set-reference` to record it again. Re-recording it in the middle of a recording invalidates the current segment.
- By default the zero pose saved to disk last time is not reused: after the base stations as a whole are moved, an old zero pose would offset all the data, and this cannot be noticed from the data.
  If you really need to reuse it, pass `restore=True` (it still requires the base station geometry fingerprint and the Tracker serial numbers to match, and an age of no more than 8 hours); `auto_reference=False` disables automatic recording.
- If the base station geometry is abnormal, zero pose calibration is refused (`CalibrationRejected`, `stage="geometry"`); when `tracker set-reference` is blocked by the startup check, the exit code is 3.

### 4.4 Runtime protection

After the handle is handed over, the SDK checks itself once per second (`guard_interval_s`):

| ID | Condition | Severity | Exception |
|---|---|---|---|
| R1 | The pose of an allowlisted base station was cleared (the gravity direction broadcast by that station changed by 2° or more: knocked askew, or reading jitter, see 4.2) | Unrecoverable | `GeometryLost` (1906) |
| R1 | The full pose of any allowlisted base station differs from the calibration acceptance record by more than 1 mm or 0.1° (compared station by station, also for a single station; stops on that very tick without waiting 3 seconds; also stops under `gate_mode="flag"`) | Unrecoverable | `GeometryLost` (1906, `kind="changed"`) |
| R1 | Not enough usable base stations (occlusion), or the spacing between base stations differs from calibration by more than ±10 mm, for 3 seconds | Recoverable | `GeometryLost` (1906) |
| R2 | A base station is moved during capture (for the detection scope see the end of this section), or a base station outside the allowlist is solved into the coordinate frame | Unrecoverable | `WorldFrameChanged` (1902) |
| R3 | Frame-to-frame pose jump | Counted and warned only | — |
| R4 | All Trackers stop streaming for 10 seconds | Unrecoverable | `TrackerStreamDead` (1901) |
| R4' | A single Tracker stops streaming | Flagged only (occlusion is normal) | — |

**Recoverable**: the current recording segment is invalidated; until the condition clears, `start()` raises the exception; after recovery you can continue recording without restarting the process.

**Unrecoverable**: the whole kit is interrupted.

1. Tracker capture stops, and `recv` / `async for` on subscriptions raise the exception;
2. The recorder invalidates the current episode, and every later `start()` raises the exception (`kit.recorder()` is hooked up automatically; for a recorder you created yourself, call `rec.guard_tracker(kit.tracker)`);
3. The kit latches the fault: `kit.fatal`, `kit.status()["fatal"]`, `kit.raise_if_fatal()`; after that `set_reference()` / `geometry()` / `recorder()` all raise the exception;
4. The glove and camera hardware streams are not stopped actively and wait for `close()`; **only restarting the process helps**.

Faults of both severities invoke `cb(exc)` from `open_kit(on_fault=cb)`. Under `gate_mode="flag"` they are only recorded, without interruption; `"off"` does not check.
There are two exceptions: once "base station pose does not match the acceptance record" holds, every subsequent frame is in a wrong coordinate frame, so under `"flag"` capture is stopped as well and zero pose calibration is refused, and only `"off"` skips the check;
the R2 world-frame gate ignores `gate_mode` and is controlled separately by `world_gate_mode` below, default `error`, so it stops capture even under `gate_mode="flag"`.

The **geometry gate** can also be queried actively: `kit.geometry()` / `TrackerRuntime.geometry()` return a dict containing `ok`, `errors`, `flags` and `pending`,
with criteria `wipe:<id>` (that station's pose was cleared), `changed:<id>` (that station's pose differs from the acceptance record by more than 1 mm / 0.1°),
`unavailable` (geometry cannot be read), `drift` (spacing change beyond ±10 mm, for 3 seconds).
The `accepted` field gives each station's currently measured difference from the acceptance record (`stations[<id>] = {dpos_mm, drot_deg}`); without an acceptance record it is `None` and this comparison is not done.

**World-frame gate** (`world_gate_mode` of `SurviveTrackerSource`, default `error`): stops capture when a base station is detected as moved during capture,
and `stats()["world"]` records the event. Options are `error` / `flag` (only record, keep capturing) / `off` (no detection), set via `TrackerRuntime(source_kwargs={"world_gate_mode": ...})`
(for `open_kit()`, inside `tracker={"source_kwargs": {...}}`). If `stats()["world"]["pose_watch"]` is not `"ok"`, the detector is not working,
and "no alarm" then does not mean the base station has not moved; even when it is `"ok"`, changes are visible only when the base station poses are re-solved; see the next paragraph.

**Undetectable moves**: at runtime the SDK does not re-solve base station positions; it relies on the gravity direction broadcast by each base station to judge whether it has been moved (a change of 2° or more makes the driver clear that station's pose, triggering R1).
Therefore when a base station is translated, or rotated about the vertical axis (gravity direction unchanged), neither R1 nor R2 triggers, with one base station or several alike; tilts of less than 2° are not detected either.
After base stations have moved you must recalibrate; do not wait for an error.

### 4.5 State files

State directory: `$REVOHUMAN_TRACKER_STATE_DIR` > `$XDG_STATE_HOME/revohuman/tracker` > `~/.local/state/revohuman/tracker`.

| File | Contents |
|---|---|
| `lighthouse_identities.json` | Base station identity table: serial number ↔ advertised name, channel seen in the most recent scan (written by `scan`; do not edit by hand) |
| `lighthouse_channel_preflight.json` | Most recent channel preflight check |
| `geometry_accepted.json` | Accepted base station geometry; also the baseline for runtime spacing drift and the per-station pose comparison |
| `last_gate.json` | Verdict of the most recent startup check |
| `lighthouse_write_log.jsonl` | Write log of channel changes and blinking |
| `calibration.json` | Zero pose |
| `coverage.json` | View coverage of the most recent calibration |
| `survive_mode` | Current mode (runtime / calibration) |
| `last_exit.json` / `alive.json` | Process exit reason / heartbeat |

The base station geometry is stored by libsurvive in `~/.config/libsurvive/config.json`; to recalibrate, use `tracker calibrate`, and do not delete or copy this file by hand.

### 4.6 Python interface: `TrackerRuntime`

`open_kit()` creates it automatically; for standalone use:

```python
from revohuman.tracker.calibration import TrackerRuntime

with TrackerRuntime({"left": "LHR-AAAAAAAA", "right": "LHR-BBBBBBBB"}).start() as rt:
    print(rt.geometry())              # geometry gate
    frame, age_s = rt.latest("left")  # latest TrackerFrame and its age (seconds), after occlusion recovery smoothing
    raw, _ = rt.latest("left", raw=True)  # raw frame (not smoothed)
    d = rt.delta("left")              # TrackerPose: increment relative to the zero pose (start() already recorded the zero pose)
    print(d.position, d.rotation)     # meters; quaternion (w, x, y, z)
    rt.set_reference()                # record the zero pose again when needed
    print(rt.status())                # includes gate (most recent startup check), fatal
```

Increment definition: `delta.position = R_refᵀ (p − p_ref)`, `delta.rotation = R_refᵀ R`, i.e. expressed in the Tracker's own frame at the moment the zero pose was taken, independent of the origin and orientation of the base station frame (4.7).

**Occlusion recovery smoothing**: when a Tracker is fully occluded (or faces away from all base stations) and then reappears, the poses of the first few frames after recovery may jump several centimeters in one step.
By default `latest()` / `delta()` spread this jump into a smooth transition of a few hundred milliseconds during "the 1 second right after recovery", and output poses unchanged at all other times;
`status()["roles"][hand]["recovery"]` records the current offset and the number of occurrences. The zero pose (`set_reference`) is recorded from the raw pose;
subscribing directly to the capture source (recording, ROS node) always yields raw frames. Pass `recovery_smoothing=False` if you do not need it.

Main parameters:

- Checks: `gate_mode` (`error`/`flag`/`off`), `whitelist`, `registered` (resolved from the allowlist by default), `ootx_wait_s` (25; released early once the base stations are ready, so a normal startup does not wait the full time), `pose_wait_s` (5);
- Zero pose: `auto_reference` (True), `restore` (False), `restore_max_age_s` (8 hours);
- Runtime: `guard_interval_s` (1), `stall_s` (10), `drift_tol_m` (0.01), `hold_sec` (3), `fence` (True);
- Occlusion recovery smoothing: `recovery_smoothing` (True), `recovery_params` (`RecoveryParams` or a dict with the same field names: `dark_s` 0.2, `window_s` 1.0, `jump_mps` 1.0, `jump_dps` 180, `tau_s` 0.15, `glide_mps` 0.5, `glide_dps` 90);
- Other: `adopt_geometry` (True; pass False for a read-only self-check so that old geometry is not adopted), `state_dir`, `config_path`.

Calibration and runtime cannot be switched within the same process (a libsurvive limitation); for calibration use the command line `tracker calibrate`.

### 4.7 Coordinate frame

`TrackerFrame.pose`, `/tracker/<hand>` in recording files and the ROS 2 `tracker_pose` are all the absolute pose of the Tracker in the **base station frame**.
The base station frame is fixed by `tracker calibrate`:

| Item | Definition |
|---|---|
| Type | Right-handed Cartesian frame |
| Units | Position in meters; orientation as a unit quaternion, (w, x, y, z) in the SDK and recording files, (x, y, z, w) in ROS 2 messages |
| +Z | Vertical, up. Constrained by the accelerometers built into the base stations and Trackers; close to, but not exactly, the plumb direction |
| +Y | Horizontal, from the origin toward the reference base station (the reference base station has x ≈ 0, y > 0) |
| +X | X = Y × Z: points to your right-hand side when standing at the origin facing the reference base station |
| Horizontal origin | A position where the Tracker was held still during calibration, chosen automatically by the solver; not the position of a base station |
| Height zero | About the lowest height at which the Tracker was held still during calibration; z is not the height above the floor |
| What the pose describes | The Tracker's own frame (defined by the Tracker's factory calibration data), not including the Tracker's mounting offset on the glove. For the palm pose, use `hand_fk.palm_from_tracker()` to convert using the product model's nominal mounting relationship (6.7); it is not calibrated per unit, so for higher accuracy measure the mounting relationship yourself |

- **Reference base station**: with only one, it is that one; with several, the last line of the `tracker calibrate` output,
  `Using LH … (xxxxxxxx) as reference lighthouse`, is authoritative; the parentheses hold the reference base station's 8-character id.
- **Each calibration rebuilds the frame**: the origin, the +Y direction and the height zero depend on where the Tracker went during this calibration;
  even if the base stations did not move and you only recalibrate, the frame changes (measured differences of nearly 20 cm and about 5°). Absolute poses from different calibrations cannot be compared directly;
  the zero pose saved to disk carries a base station geometry fingerprint and is not reused after recalibration (4.3).
- **Fixed at runtime**: base station positions are not re-solved at runtime, so absolute poses under the same calibration can be compared directly.
  The frame becomes invalid once base stations are moved; for what can and cannot be detected, see 4.4.
- **Increments**: `delta` is expressed in the Tracker's own frame at the moment the zero pose was taken (4.6) and does not change with the origin or orientation of the base station frame.
- **palm_link in the hand model**: the origin of the palm_link frame is the CAD assembly datum of the hand model (`models/`), at the back-of-hand layer, beyond the proximal end of the back-of-hand plate;
  it is not on any joint, nor is it the anatomical base of the palm or the wrist joint center; do not use it directly as the wrist point. In the model, the origin of `{hand}_tracker_Link`
  is the origin of `TrackerPose` (on the axis of the 1/4" screw hole on the underside of the Tracker).
- The ROS 2 node only writes `frame_id` in the message header and does not publish tf (7.3).

### 4.8 Orientation jump gate (optional, off by default)

At sites with poor lighting conditions (for example with only one base station), the optical solve occasionally produces a frame whose orientation is clearly discontinuous with the frames before and after it,
and libsurvive's pose estimate accepts it as is.
The bundled pysurvive 1.1.21+revohuman.3 has an optional gate: before each optical pose is merged into the pose estimate, it is compared once with the orientation predicted from earlier data for the same moment, and the frame is dropped if they differ too much.

- **Off by default**; the SDK never turns it on by itself. When it is off, behavior is the same as in the previous version.
- Only orientation is checked, not position. Tolerance = threshold + current angular velocity × the time difference between the two + the deviation when the previous frame was accepted, so normal fast rotations are not blocked.
- Frames are dropped consecutively for at most `--kalman-obs-rot-gate-release-sec` seconds (default 0.1), after which frames are accepted however large the difference, so it never gets stuck;
  the first 16 frames after a Tracker starts tracking or after a tracking reset, and the first frame back in view after occlusion, are not compared.
- There is no universally recommended threshold; the 5° / 0.1 s below is only a starting point. While it is on, libsurvive prints one statistics line starting with `T5` per Tracker every 60 seconds
  (how many frames were compared, how many were dropped, deviation distribution); tune by it.

The two parameters are passed to libsurvive as is:

```python
GATE = ["--kalman-obs-rot-gate-deg", "5", "--kalman-obs-rot-gate-release-sec", "0.1"]
kit = open_kit(tracker={"survive_args": GATE})                 # open_kit / TrackerRuntime
src = SurviveTrackerSource(registry=reg, args=GATE)             # low-level interface (5.3)
print(src.stats()["obs_rot_gate"])                              # {"mode": "on", "deg": 5.0, "release_s": 0.1, ...}
```

```bash
# ROS 2 (tracker_survive_args in 7.5)
ros2 launch revohuman_bringup revohuman.launch.py \
    tracker_survive_args:=--kalman-obs-rot-gate-deg,5,--kalman-obs-rot-gate-release-sec,0.1
```

There is no need to repeat the SDK's default arguments in `survive_args` / `args`; any that are left out are filled in automatically.
The gate is really on only when `stats()["obs_rot_gate"]["mode"]` is `"on"`; `"unsupported"` means the installed pysurvive does not recognize this parameter
(older versions silently ignore it, and the SDK logs a warning at startup); install the bundled version instead. To turn it off, remove the two parameters and restart.

---

## 5. Using the modules

`open_kit()` covers the common use cases. When you need finer control, use the individual modules directly.

### 5.1 Single glove

```python
import queue
from revohuman import GloveManager

with GloveManager().connect(handedness="left") as glove:
    sub = glove.subscribe()                  # 200 Hz by default
    for _ in range(200):
        try:
            f = sub.recv(timeout=1.0)
        except queue.Empty:
            continue                         # warm-up, timestamp gate closed, or reconnecting (§6.3)
        print(f.seq, f.sample_wall_ns, f.joints_deg[:3])
    print(glove.stats().to_dict())
```

`connect()` also accepts `port=`, `unit_id=`, `rate_hz=` and `require_tactile=`.
`glove.selftest()` runs a read-only self-test. The self-test has to stop this hand's data stream first and restart it after reading: this hand produces no new frames for about 50 ms, the paired two-hand stream is likewise empty for about 50 ms (hardware sync is kept), and `f.generation` increases by 1 afterwards. **Do not call it while recording.**
`glove.calibrate(target=255, timeout=15.0)` performs encoder zero-pose calibration (written to the glove, irreversible; before calibrating, straighten all five fingers and lay the hand flat, see §2.5): `target=255` calibrates all online encoders,
`0`–`20` calibrates a single channel; the result carries `succeeded_indices`, `failed_indices`, `zero_raw` and `persisted`.
It raises `CalibrationError` (error code 1300) when an encoder fails, the result is not saved, or it times out; in the first two cases the exception's `result` attribute holds the same partial result.
`glove.clear_calibration()` clears the encoder calibration (written to the glove). Like the self-test, both have to stop this hand's data stream first; **do not call them while recording**.
For tactile tare use `glove.tactile_tare()`; for split tactile use `connect(..., split_tactile=True)`; see §6.4.

### 5.2 Two-hand pairing

```python
from revohuman import GloveManager

pair = GloveManager().connect_pair()         # aligns both gloves to the same USB frame clock before streaming starts
sync = pair.synchronizer()
pf = sync.subscribe().recv(timeout=1.0)      # PairedFrame
print(pf.skew_ns)                            # right − left sampling time; always 0 under hardware sync
```

Pairing accuracy budget: `pair.synchronizer(pair_accuracy_ms=1.0)` (default 1.0 ms); pairs that do not meet it are not output.
When hardware sync is lost, the SDK switches to software pairing and rebuilds sync in the background; it does not stop recording automatically. Recordings that need strict sync can install a callback with `sync.on_sync_lost(callback)` (e.g. `rec.abort`: stops the current segment and latches the recorder); after that, calling `rec.start()` raises `StrictSyncLost`.

**Two-hand tactile gate**: on by default when both gloves' product models have tactile (`pair.synchronizer(tactile_gate="auto")`; off when the product model cannot be determined).
When tactile on either hand is unusable for the current tick (no tactile, or `tactile.usable` is false, §6.4), **the whole pair is withheld, joints included**,
and counted in `sync.stats()["tactile_gated_pairs"]`; output resumes automatically once tactile recovers on both sides. Gate closing and gate opening each report one notice, N2207 / N2208 (§6.6);
you can also receive `callback(is_open, reason)` via `pair.synchronizer(on_tactile_gate=callback)`. For the current state see `tactile_gate` (whether enabled), `tactile_gate_open`,
`tactile_gate_reason` and `tactile_gate_closures` in `stats()`.
To keep joints flowing when tactile fails, pass `tactile_gate=False`. The ROS 2 two-hand node has the same gate (§7.3).

### 5.3 Low-level Tracker interface

```python
from revohuman import GloveManager, Handedness
from revohuman.tracker import SurviveTrackerSource, BimanualTrackerSynchronizer

reg = GloveManager().tracker_registry()                  # {"LHR-...": Handedness.LEFT, ...}
with SurviveTrackerSource(registry=reg, lighthouses=["xxxxxxxx", "yyyyyyyy"]) as src:
    tsync = BimanualTrackerSynchronizer(src.hand(Handedness.LEFT), src.hand(Handedness.RIGHT))
    pair = tsync.subscribe().recv(timeout=1.0)           # PairedTrackerFrame
    print(pair.left.sample_wall_ns, pair.left.pose.position)
```

`lighthouses` takes the 8-character ids of the base stations on the allowlist. Closing either Tracker ends the paired stream; a world-frame change raises `WorldFrameChanged` (§4.4).

#### Polling mode (`poll_mode`)

`SurviveTrackerSource(poll_mode=...)` decides on which side the tracker data callbacks and the polling loop run:

| Value | Meaning |
|---|---|
| `"auto"` (default) | Uses `native` if the package contains a prebuilt accelerator for this machine's architecture; otherwise (or if loading or its self-check fails) silently falls back to `python` |
| `"native"` | Callbacks and the polling loop run in the C accelerator without holding the Python interpreter lock, so however busy other Python threads are, tracker capture is not slowed down; `start()` raises if the accelerator is unavailable |
| `"python"` | Pure Python path |

- If the constructor argument is not given, the environment variable `REVOHUMAN_TRACKER_POLL_MODE` is read (same values as above).
  `REVOHUMAN_SURVIVE_SHIM=off` disables the accelerator (equivalent to `python`); setting it to a file path loads the specified accelerator.
- The package ships two accelerators, for aarch64 and x86_64. Both paths output identical pose frames; the only difference is that timestamping is no longer affected by busy Python threads.
- Which path is actually taken, and why, is in `src.stats()`: `poll_mode` / `poll_mode_requested` / `poll_mode_reason`;
  `native` holds the accelerator's raw counters (`None` if `native` was never used). On `native`, the process has two threads named
  `rh-survive-poll` / `rh-survive-disp` (visible in `/proc/<pid>/task/*/comm`).

#### IMU and light windows (`native` only)

```python
tr = src.hand(Handedness.LEFT)
imu = tr.subscribe_imu()                 # TrackerImuSample, about 250 Hz
light = tr.subscribe_light()             # TrackerLightWindow, one per base station channel per sweep
s = imu.recv(timeout=1.0)
print(s.sample_wall_ns, s.accel, s.gyro, s.saturated)
```

- `TrackerImuSample`: `accel` (g) and `gyro` (rad/s) are in the IMU chip's own axes and are **not** rotated into the tracker pose frame;
  `raw_counts` are the raw counts, and bits 0..5 of `saturated` flag full-scale saturation of ax ay az gx gy gz, in that order.
  `sample_wall_ns` is on the same time axis and behind the same timestamp gate as the pose frames, so it can be subtracted directly from glove frames.
- `TrackerLightWindow`: a summary of hits for one tracker on one base station channel between two consecutive sweeps —
  `sensors_axis0/1` are sensor bitmasks, `hits_axis0/1` are hit counts, and `start_wall_ns` / `sample_wall_ns` are the start and end of the window.
  No windows on a channel for some time = that base station was not seen during that time.
- Nothing is processed when nobody subscribes; on the `python` path subscribing still succeeds but delivers no data. Whether data is available, and why, is in
  `src.stats()["imu_light"]`; per-tracker counts are in `stats()["trackers"][serial]["source"]["imu_light"]`.
- `TopicRecorder` does not record these two kinds of data.

#### Gyro orientation fusion (optional, off by default, `native` only)

The accelerator runs a complementary fusion of the IMU gyro with the optical pose and attaches a `frame.fused` to every frame (`FusedOrientation`: `rotation` (wxyz),
`state`, `reason`, `bias`, `err_rad`, `imu_used`, `pose_seq`, `resets`, `params_hash`). **Only the orientation is corrected**:
`fused.rotation` is in the same frame and component order as `pose.rotation`; position, timestamps and `pose` itself are unchanged. `fused` is not included in `to_dict()` or in recording files.

```python
ok, why = src.gyro_fuse_available                              # False + reason when unavailable
applied = src.load_gyro_fuse_constants("gyro_fuse.yaml")       # {serial: params_hash}
src.enable_gyro_fuse()                                         # if not enabled, frame.fused is always None
```

- None of the entry points `gyro_fuse_available` / `configure_gyro_fuse(serial, params)` / `load_gyro_fuse_constants(path)` /
  `enable_gyro_fuse(serial=None, on=True)` / `clear_gyro_fuse(serial=None)` / `gyro_fuse_stats()` raise;
  when unavailable they return the reason, and the caller keeps using the raw orientation.
- The constants file is YAML (`version: 1`, `frame: libsurvive_object`), keyed by tracker serial number, giving each tracker's mounting rotation `Rb`,
  `scale` and `bias_rad_s`; an entry that fails validation is rejected, with the reason in `stats()["gyro_fuse"]["constants"]["rejected"]`.
  For parameter validation and hashing see `revohuman.tracker.gyro_fuse` (`FuseParams` / `GyroFuseConstants` / `rb_raw_from_delta`).
- Every time constants are submitted, convergence restarts from the next frame, so the SDK skips the submission automatically when the constants have not changed (same `params_hash`).
- The configuration lasts only for the current capture: a freshly started `SurviveTrackerSource` must be configured and enabled again after `start()` succeeds; the SDK does not replay it.
- `stats()["gyro_fuse"]` gives availability and frame pairing counts; `gyro_fuse_stats()` gives each tracker's convergence state and bias estimate,
  where `err_max_rad` is cleared on every read, so there should be only one reader.

### 5.4 Wrist cameras (PTV1)

```python
from revohuman.camera import WristCamera, CameraConfig

cfg = CameraConfig(usb_hub="3-3", handedness="right")     # usb_hub = usb_topology_id of that hand's glove unit; auto exposure by default
with WristCamera(cfg) as cam:                # entering with = start()
    f = cam.subscribe(queue_size=8).recv(timeout=1.0)
    print(f.sample_wall_ns - glove_frame.sample_wall_ns)   # same time axis as the glove; subtract directly
    img = f.decode()                         # needs the camera extra
```

- Device: give `device="/dev/videoN"`, or give `usb_hub=` to auto-discover under that HUB; if neither is given, the first discovered camera is used (with a warning if there are several).
  `open_kit()` fills this in automatically per glove unit.
- **Product model**: supported wrist cameras are listed in `revohuman.camera.models`:

  | Model name | USB id |
  |---|---|
  | `uvc_28c4` | `1bcf:28c4` |
  | `uvc_3080` | `0edc:3080` |

  Auto-discovery only recognizes listed USB ids. `start()` identifies the model from the device's USB id and applies the controls listed for that model (11 items, including auto exposure,
  auto white balance and 50 Hz; `uvc_3080` (the wrist camera built into the glove) is managed at that model's factory values and keeps continuous autofocus on); `cam.model` / `cam.vid_pid` and `model` / `vid_pid` in `stats().stream`
  record which model it actually is. `CameraConfig(model="uvc_3080")` pins the model: auto-discovery looks only for that model and raises `CameraNotFound` if none is found (see "Camera not found" below);
  if `device=` is given explicitly and it is not that model, `start()` raises `RuntimeError`.
  For an unrecognized camera (`device` given explicitly but the USB id is not listed), no default controls are applied, only the ones you give explicitly.
- **Controls**: `controls=` replaces the whole table (`{}` = manage nothing, keep all of the device's current values); to change just a few use
  `control_overrides={"gamma": 120, "power_line_frequency": None}` (layered on top of the model's table; `None` = leave this item alone).
  The table actually applied is in `cam.controls_wanted`, and the read-back values are in `stats().stream["controls"]`.
- **Exposure**: auto exposure (aperture priority) by default, bright enough for indoor scenes. For fixed exposure use
  `control_overrides={"auto_exposure": 1, "exposure_time_absolute": 333}` (in units of 100 µs; 333 = 33.3 ms, the upper limit at 30 fps).
  Under auto exposure the per-frame exposure time is unknown (see "Timestamps" below); switch to fixed exposure when you need sub-frame alignment.
- **Camera not found**: raises `CameraNotFound` (a subclass of `FileNotFoundError`; the error code is still `E1000`). When `usb_hub` is given,
  the exception text describes what is actually under that HUB (HUB absent / no video device / camera present but model not listed / model does not match `model` /
  device present but no node under `/dev`), and `exc.probe["verdict"]` and `exc.probe["ports"]` give per-port details.
- Default 640×480@30 MJPG. Capture, timestamping and recording use only the standard library (V4L2); only `decode()` needs opencv.
- **Timestamps**: the device PTS (start of frame capture) is mapped to the host clock via the USB frame clock. The mapping itself is accurate to microseconds
  (jitter p95 3 µs / max 5 µs, plus a fixed bias of at most 125 µs), so the times of consecutive frames from the same camera are very accurate relative to each other.
  Between PTS and the actual exposure time there is a device-constant phase. Each of the two listed models has an exposure timing measured with an on-board LED (at 640×480@30 MJPG,
  the exposure end and the readout span are fixed relative to PTS); by default (`CameraConfig(model_timing=True)`) the SDK uses it to place the sampling time at the exposure center of the middle row
  (`stats().stream["stamp_model"]` is `model_led`). Other stream formats and unlisted models automatically fall back to the "readout midpoint" model:
  sampling time = PTS + frame period/2; `stamp_fallback` states the reason, and `sample_phase_bound_ms` gives the phase bound under this model,
  `(T − t_exp)/2` (8.35 ms at exposure 166, about 0 at exposure 333).
  For sub-frame alignment with the gloves, measure this constant and pass it in `CameraConfig(pts_offset_ns=...)` (the measurement tool is not shipped with the SDK).
  In that case switch to fixed exposure (e.g. 333, see "Exposure" above): with fixed exposure this phase is constant and only needs to be measured once; under auto exposure it changes with the scene.
  Model timing is about 16–18 ms later than the readout midpoint; to compare against recordings made with earlier versions, set `model_timing=False` to fall back to the readout midpoint.
  An explicit `pts_offset_ns` always takes precedence (`calibrated`).
- Under auto exposure (the default) the device does not report the actual exposure: `exposure_us` is `None` and `stats().stream["exposure_mode"]` is `auto`.
  Model timing timestamps using the auto exposure upper limit measured indoors (about 30 ms); when the scene is brighter and auto exposure shortens the exposure, the true exposure center is later than the timestamp, by up to about 15 ms.
  `sample_phase_bound_ms` reports exactly this one-sided worst case: under model timing it is the uncertainty of the timing constant plus half the auto exposure upper limit
  (about 15.3 ms for `uvc_28c4`, about 15.9 ms for `uvc_3080`); with fixed exposure only the uncertainty of the timing constant remains (0.3 / 0.7 ms);
  under the readout-midpoint model with unknown exposure it is half the frame period.
- **Missing metadata**: when the metadata node is open and frames are arriving but the time mapping has not converged for `meta_missing_s` (default 2 s) in a row, frames are still not delivered,
  `stats().stream["meta_missing"]` is `True`, and `meta_missing_reason` says whether no metadata arrived at all (`no_meta`),
  metadata arrived without timestamps (`meta_without_pts`), or the mapping does not converge (`clock_unconverged`).
- The mapping is fitted against the host's CLOCK_MONOTONIC_RAW and is **not affected by host NTP frequency adjustment** (see §6.1). Mapping status is in `cam.stats().clock`:
  `converged`, `model_domain` (`"raw"`), `usb_ppm_vs_raw` (frequency offset of the USB frame clock relative to the host clock),
  `dev_ppm_vs_raw` (frequency offset of the camera clock relative to the host clock). Both are frequency offsets between hardware clocks and are essentially constant under normal conditions.
- Per-row times for the rolling shutter: `f.row_sample_wall_ns(row)`.
- **Zero-copy**: `cam.subscribe(loaned=True)` gives read-only views onto driver buffers, which must be returned with `with frame:` or `frame.release()`;
  when the quota runs out it automatically falls back to copying (counted in `stats().stream["loan"]["starved"]`), and no data is lost. Zero-copy and normal subscriptions can coexist.
- Frames whose time mapping has not yet converged are not delivered by default (`deliver_unmapped=True` lets them through; their `stamp_source` is `first_payload`, accuracy ±2 ms).

`CameraFrame` fields: `sample_wall_ns`, `sample_mono_ns`, `recv_wall_ns` (arrival time, tens of milliseconds after sampling; do not use it for alignment),
`pts`, `buffer_mono_ns`, `pts_offset_ns`, `exposure_us`, `readout_ns`, `stamp_source`, `flags`, `data`, `nbytes`.

Examples: [`examples/wrist_camera_async.py`](../examples/wrist_camera_async.py),
[`examples/wrist_camera_record.py`](../examples/wrist_camera_record.py).

### 5.5 Recording and playback (MCAP)

`kit.recorder()` comes with all data streams already wired up. Topics:

| Topic | Content |
|---|---|
| `/glove/<hand>` | Single-hand raw frames (`GloveFrame`) |
| `/paired` | Paired two-hand frames (`PairedFrame`) |
| `/tracker/<hand>` | Tracker frames (`TrackerFrame`) |
| `/camera/<hand>` | Camera frames (raw JPEG bytes, not transcoded) |
| `/tactile/<hand>` | Split tactile samples (`TactileSample`, §6.4); present only when you manually call `rec.record(glove.subscribe_tactile())` |

Manual assembly:

```python
from revohuman import TopicRecorder, recording

rec = TopicRecorder(compression="lz4")
rec.record(pair.left.subscribe())
rec.record(pair.right.subscribe())
rec.record(pair.synchronizer().subscribe())
rec.guard_capture(pair)                      # glove timestamp gate closes → current episode is invalidated
rec.guard_world_frame(src)                   # base station moved → current segment invalidated and locked

h = rec.start("ep_001.mcap")                 # calling start again first finalizes the previous segment
...
print(h.stop())                              # blocks until the file is fully written; cannot be cancelled
rec.close()

with recording.open("ep_001.mcap") as reader:
    for msg in reader:
        print(msg.topic, msg.log_time_ns, msg.frame)   # frames read back have their original types
```

- MCAP `log_time` is the arrival time (`recv_wall_ns`, monotonic in write order within a topic); for cross-modal alignment use the `sample_wall_ns` inside the frame (`left.sample_wall_ns` for paired two-hand frames).
- The recorder does not accept ungated subscriptions (`gated=False`, see §6.3); passing one raises `ValueError`.
- Recordings always store raw joint angles: passing a subscription with joint-angle smoothing (`subscribe(joint_filter=...)`, see §6.3) raises `ValueError`; open a separate smoothed subscription for display / control.
- With split tactile enabled (§6.4), joint frames carry no tactile data, so you must also record `glove.subscribe_tactile()` (one per hand), or no tactile is recorded;
  when reading back, pair the samples with joint frames by nearest `tactile.sample_wall_ns`. `kit.recorder()` does not enable split tactile.
- `start()` raises `CaptureGateClosed` while the timestamp gate is closed.
- Data volume for reference: two hands + two cameras is about 11 GB/hour (varies with image detail).

Example: [`examples/pair_and_record.py`](../examples/pair_and_record.py).

### 5.6 Glove button

The button on the glove toggles between "start" and "end": each valid press toggles once, holding it down does not repeat, and the state is "end" at power-up.
The button state is reported in the same frame as the joint data, with no extra connection and no change to the sampling rate. The button is off by default; when needed, pass `button=True` explicitly when connecting:

```python
import queue
from revohuman import ButtonMode, GloveManager

with GloveManager().connect(handedness="right", button=True) as glove:
    presses = glove.button.subscribe()
    while True:
        try:
            ev = presses.recv(timeout=1.0)   # ButtonEvent
        except queue.Empty:
            continue
        print(ev.mode, ev.press_count, ev.sample_wall_ns)
        if ev.mode is ButtonMode.END:
            break
    presses.close()
```

For two hands use `connect_pair(..., button=True)`; each glove's button is at `pair.left.button` / `pair.right.button`.
The button does not take part in two-hand pairing. `open_kit()` does not enable the button by default; when needed pass `glove_kwargs={"button": True}`, and the button is at `kit.glove(h).button`.
The ROS 2 two-hand node enables it with the parameter `button:=true` and publishes one `button_event` per press (§7.3); the single-hand node does not enable the button.

- **Availability**: `glove.button.supported` is determined only after the stream first starts. Stream start is asynchronous, so check it after you get the first frame.
  - `None`: the stream has not been started yet.
  - `True`: the glove firmware supports reporting the button during capture.
  - `False`: the firmware does not support it. Connection and capture proceed as usual, open button subscriptions are closed (`recv()` raises `UnsupportedFeatureError`), and one `N2406` is logged.
- **When not enabled**: without `button=True`, `glove.button.enabled` is `False`. `subscribe()` / `read()` raise `UnsupportedFeatureError`, and the data stream is exactly the same as without the button.
- **Current state**: `glove.button.read()` returns the `ButtonState` of the most recent frame, with `mode`, `pressed` (whether the button is held down in this frame) and `press_sequence` (the glove's cumulative count of valid presses). It is `None` while there is no valid state yet.
- **Event logic**:
  - The first valid state after each stream start serves only as a baseline and produces no event. If the button was pressed before connecting, the first press may report "end"; press once more for "start".
  - After that, each increase of n in the cumulative press count reports one `ButtonEvent` with `press_count=n`.
  - `press_count > 1` (i.e. `ev.missed_presses > 0`) means some presses in between were not reported individually; in that case `ev.mode` is authoritative.
- **After reconnecting**: after a reconnect, or after a self-test / calibration restarts the data stream, the button state is initialized from scratch and `ev.generation` changes accordingly. The state from before the disconnect is not carried over, and presses made while disconnected are not reported afterwards.
- **Timing**: an event's `recv_wall_ns` / `sample_wall_ns` / `seq` are taken from the first frame in which the press is seen.
  - Press times are accurate only to the frame: suitable for episode start/end, not for precise time alignment.
  - While the timestamp gate is closed, events are still reported, with `sample_wall_ns` set to `None`.
- **Recording**: button events are not written to MCAP. If you need to keep a record of them, save the event fields yourself.
- **Counters**: `glove.button.events_total` is the number of events; `glove.button.missed_presses_total` is the number of presses not reported individually.
- **Requires glove firmware that supports reporting the button during capture.** A glove can be opened by only one process at a time; do not start a separate program to read the button.

Example: [`examples/button_record.py`](../examples/button_record.py) (press once to start recording a segment, press again to end it).

### 5.7 Wrist camera intrinsics and extrinsics (PTV1)

`revohuman.calibration` provides the wrist cameras' intrinsics and mounting pose (extrinsics). It covers only the two wrist cameras on the gloves; P1 has no wrist cameras.

```python
from revohuman import calibration as cal

ext, why = cal.wrist_camera_extrinsic("uvc_3080", "left", product="PTV1")              # model default
ext, why = cal.wrist_camera_extrinsic("uvc_3080", "left", "PTV1", glove_sn=serial)     # per-unit value when a per-unit calibration exists
intr, why = cal.wrist_camera_intrinsics("uvc_3080", None, "left")                      # model default
intr, why = cal.wrist_camera_intrinsics("uvc_3080", serial, "left")                    # per-unit value when a per-unit calibration exists
print(intr.scope)                                                                      # "unit" / "model"
```

Camera model names are the names listed in `revohuman.camera.models` (§5.4). Every function returns `(value, reason)`; when the value is `None`, check the reason:

| Reason | Meaning |
|---|---|
| `ok` | Found |
| `no_calibration` | This camera has neither a per-unit calibration for this glove nor a listed model default |
| `calibration_unreadable` | A calibration file exists but cannot be read (bad format, missing fields); does not silently fall back to the model default |
| `bad_argument` | Invalid argument (handedness not `left` / `right`, unknown product, or a serial number or camera model name containing path characters) |
| `no_mount_for_camera_model` | This product has no mounting pose listed for this camera model (always this value for P1) |

| Function | Description |
|---|---|
| `wrist_camera_intrinsics(camera_model, glove_sn, side, product="PTV1")` | Intrinsics `Intrinsics` (ROS camera_info convention, OpenCV distortion): `width` / `height`, `distortion_model`, `k` (3×3, row-major), `d`, `p` (= [K \| 0]), `calibration_id`, `image_transform`, `rms_px`, `calibrated_at`, `calib_focus`, `board`, `scope`, `source`; `as_dict()` converts it to a dict. `glove_sn` may be `None` (returns the model default directly) |
| `wrist_camera_extrinsic(camera_model, side, product="PTV1", glove_sn=None)` | Mounting pose `Extrinsic`: `{side}_palm_link` → `{side}_camera_Link`, `pose` = (x, y, z in meters, qx, qy, qz, qw), plus `method`, `source`, `extra` |
| `wrist_camera_calibrations()` / `wrist_camera_extrinsic_calibrations()` | Iterate over the per-unit intrinsics / per-unit extrinsics present in the data directory, yielding (camera model, serial number, hand, value, reason) for each entry |
| `load_intrinsics(directory)` / `load_wrist_extrinsic(directory)` | Read one calibration directory directly |
| `rpy_to_quat(rpy)` | URDF rpy → quaternion (qx, qy, qz, qw) |

**Extrinsics: model default and per-unit value.** `ext.extra["scope"]` tells which one you got:

- `"model"`: the model default, taken from `wrist_cameras` in the model package's `models/<model>/model.yaml`. Returned when `glove_sn` is not given, or when the data directory has no per-unit extrinsics for this glove.
- `"unit"`: the per-unit value, taken from this glove's `extrinsic.yaml` in the data directory; `extra` additionally carries `calibration_id`, `palm_frame` and the uncertainty.

`extra` always carries `optical_inplane_deg` (rotation of the optical frame relative to camera_Link about +Z) and `upright_transform` (the transform that uprights that hand's raw image).

The PTV1 `uvc_3080` model default comes from calibration-target measurements on **one pair of gloves** (`method` = `calibrated`), with an uncertainty of 1σ ≤0.45° / ≤3.8 mm.
These values have not been validated on multiple pairs of gloves, and assembly variation between gloves is not included in this uncertainty; when a per-unit calibration exists, the per-unit value takes precedence.

**Palm convention.** The extrinsics are "recorded palm → camera": palm = tracker pose · the tracker extrinsics in the URDF (i.e. `hand_fk.palm_from_tracker`, §6.7).
Apply them directly on top of this palm; **do not stack any additional origin correction**.

**Uprighting the image.** The intrinsics and the optical frame both refer to the **uprighted** image; the camera module outputs the raw image, and `frame.decode()` does not upright it.
Before use, upright it according to `Intrinsics.image_transform` (or the extrinsics' `extra["upright_transform"]`): left hand `rotate_180` (rotate 180°),
right hand `flip_vertical` (flip top to bottom). The right hand's raw image is mirrored, so the pinhole model cannot be applied without uprighting it.

**Cameras in the URDF.** The PTV1 URDF (`models/dv1/urdf/`) has, for each hand, two fixed joints generated from the model default:

- `{side}_palm_link → {side}_camera_Link`: xyz / rpy are the model default.
- `{side}_camera_Link → {side}_camera_optical_frame`: rotation by `optical_inplane_deg` (270°) about +Z. In the optical frame z points forward (optical axis), x to the right of the image and y to the bottom of the image, matching the uprighted image.

Running forward kinematics on the URDF gives the model default; when a pair of gloves has a per-unit calibration, the result looked up with the glove serial number takes precedence. The P1 URDF has no cameras.

**Intrinsics: model default and per-unit value.** `intr.scope` tells which one you got:

- `"unit"`: the per-unit value, taken from this glove's `intrinsics.yaml` in the data directory.
- `"model"`: the model default, taken from `wrist_cameras.<camera_model>.intrinsics` in the model package's `models/<model>/model.yaml`. Returned when `glove_sn` is not given,
  or when the data directory has no per-unit intrinsics for this glove; `source` states where it came from, and `calibration_id` is that of the original calibration, so it can be traced.

The PTV1 `uvc_3080` intrinsics model default comes from calibration-target measurements on **one pair of gloves** (one set each for the left and right hand). Units of the same camera model differ considerably:
compared against the calibration of another pair of gloves, the left-hand image differs by about 9 px across the image (about 1.5° in ray direction) and the right hand by about 4 px (about 0.6°), whereas the reprojection error
of a per-unit calibration is about 0.1 px. The model default is only suitable for uses such as rough projection or drawing guide lines; for accurate reconstruction, do a per-unit calibration first. With samples from only two pairs of gloves, the spread across the whole batch is unknown.
Continuous autofocus was off during calibration (recorded in `calib_focus`), while `uvc_3080` keeps continuous autofocus on during recording (§5.4), so for recorded images the values are an approximation.

**Shipped data and your own data.** Per-unit calibration data is **not shipped with the package**: the shipped `revohuman/calibration/data/` contains only a layout note (`README.md`; Chinese version `README.zh-CN.md`),
and the intrinsics and extrinsics model defaults are in the model packages. Put per-unit data in a directory using the layout below and point the environment variable `REVOHUMAN_CALIBRATION_DIR` at it;
if it is not set, both intrinsics and extrinsics return the model defaults:

```
<dir>/wrist/<camera_model>/<glove_sn>/<left|right>/
    intrinsics.yaml           intrinsics (calibration tool output as is; calibration_id = first 16 characters of this file's sha256)
    calibration_report.json   calibration report (upright transform, reprojection error, calibration time, calibration target, focus setting)
    extrinsic.yaml            optional: this unit's extrinsics (same format as wrist_cameras in model.yaml, palm_frame: recorded)
```

**Glove serial numbers are not guaranteed to be unique.** Two different pairs of gloves may share a number, yet per-unit data is keyed by serial number: with a shared number you get the other pair's calibration.
Before putting data into the data directory, confirm that the serial number really belongs to the pair of gloves in hand; when using two pairs of gloves with the same number at the same time, do not provide per-unit data for them.

---

## 6. Data semantics

### 6.1 Timestamps

All `*_wall_ns` values are CLOCK_REALTIME nanoseconds and share one process-level time base, so values from different modalities can be subtracted directly. Every frame type carries the two times below:

| Field | Meaning |
|---|---|
| `sample_wall_ns` | **Sampling time**: when the sensor actually sampled. Use it for cross-modal alignment |
| `recv_wall_ns` | **Arrival time**: when the SDK received the data, including link latency; do not use it for alignment |

For P1 alignment methods, synchronization metrics, and handling of time-related exceptions, see the user manual *09 Timestamp Synchronization*.

Each modality also has its own time fields:

| Frame | Field | Meaning |
|---|---|---|
| `GloveFrame` | `device_tick` | Raw device value of the encoder sampling time (1 ms counter); `sample_wall_ns` is converted from it |
| `TactileData` | `sample_wall_ns` / `device_tick` | The tactile data's own sampling time / its raw device value (same 1 ms time base as the encoders); see below |
| `TactileData` | `age_ms` | Tactile data age reported by the device; for diagnostics only, not used for timestamping |
| `TrackerFrame` | `timecode` / `tc_unwrapped` | Device 48 MHz counter (wraps around about every 89.5 seconds) / unwrapped value; use the latter for comparisons across frames |
| `CameraFrame` | `pts` | Device "frame-capture start" counter; `sample_wall_ns` is converted from it, see 5.4 |
| `CameraFrame` | `sample_mono_ns` | The same instant as `sample_wall_ns`, expressed in CLOCK_MONOTONIC |
| `CameraFrame` | `buffer_mono_ns` | Driver buffer time (CLOCK_MONOTONIC), for troubleshooting only |
| `CameraFrame` | `stamp_source` | `"pts"` (normal) or `"first_payload"` (fallback, derived back from the buffer time, ±2 ms) |
| `PairedFrame` / `PairedTrackerFrame` | `skew_ns` | Difference between the sampling times of the left and right units |

- **Frames have no quality field**: frames whose time mapping cannot be trusted are not delivered, pairs that miss the pairing accuracy are not output, and pairs are also not output while the bimanual tactile gate is closed (5.2). Every frame you receive is within the accuracy budget.
- **Tactile has its own sampling time**: tactile is sampled on its own cadence, not on the same beat as the encoders. `frame.tactile.sample_wall_ns` is about 3–9 ms later than the encoders in the same frame;
  this difference varies by hand and by session and is **not a constant**. For millisecond-level analysis, use the tactile time directly; do not estimate it as the encoder time plus a constant.
- **`skew_ns`**: always 0 for gloves under hardware sync; the two Trackers sample independently, so it is not 0 even in the best tier (up to about ±2 ms).
- **Absolute phase**: the sampling times of the Tracker and the wrist camera are both mapped from device counters, and the mapping itself is accurate; between "device timestamping → actual sampling"
  there is an additional device constant. For the Tracker it is the packing latency inside the device, configured per serial number with `SurviveTrackerSource(phi_pack_ms={"LHR-…": 毫秒})`
  (`毫秒` = milliseconds; ROS parameter `tracker_phi_pack_ms:=LHR-…=毫秒,LHR-…=毫秒`, live broadcast `revohuman-live --tracker-phi-pack LHR-…=毫秒`;
  the measurement tool is not shipped with the SDK);
  for the camera it is the phase from PTS to the exposure time: for listed models in the measured stream formats it is compensated by default using the model timing (`model_timing`; for the error under auto exposure see 5.4);
  in other cases configure `CameraConfig(pts_offset_ns=...)` (ROS parameter `camera_pts_offset_ms`, see 5.4). The constant stays the same throughout a modality and does not affect that modality's own timing;
  configure it for absolute cross-modal alignment. With neither model timing nor a configured value, it shows up as a fixed offset.
- Accuracy budgets are given as physical quantities: bimanual pairing `synchronizer(pair_accuracy_ms=...)` (default 1.0 ms);
  Tracker sampling time `SurviveTrackerSource(stamp_accuracy_ms=...)` (not required by default). Frames or pairs that miss the budget are not delivered.
- Judge run quality from statistics, not from frames: `frames_by_tier`, `capture_gate["transitions"]` and `frames_blocked` in `glove.stats().stream`;
  for the pairer, `sync.stats()["pairs_by_tier"]`.
- **Host clock**: the time mappings of all three streams (glove, Tracker, wrist camera) are fitted against the host's CLOCK_MONOTONIC_RAW (a hardware clock whose frequency NTP does not adjust),
  and the output is converted back to CLOCK_MONOTONIC and CLOCK_REALTIME. Therefore **frequency adjustment** (slew) by the host NTP client does not affect sampling times:
  in simulation, with a sudden 0.07% frequency adjustment applied to the host clock, the sampling times of all three streams differed by no more than 2 µs from those without the adjustment.
  Host wall-clock **jumps** (step: the first NTP sync, a manual time change, catch-up after waking from sleep) are detected on the glove side: the process-level time base is refreshed accordingly,
  the `*_wall_ns` of all three streams shift by the same amount and remain comparable with each other, and the glove pauses frame delivery and resumes after automatically re-syncing its time
  (counted in `capture_gate["transitions"]` and `frames_blocked`). When only Trackers or cameras are open, without a glove, the time base is not refreshed:
  the `*_wall_ns` of the streams remain consistent with each other, but differ from the host's current wall clock by the jump amount.
  Such pauses are undesirable during recording; it is recommended that the host steps only at boot and only slews while running (see 10).

### 6.2 Frame structures

**`GloveFrame`**

| Field | Description |
|---|---|
| `seq` / `generation` | Sequence number; a change in `generation` means a reconnect happened in between |
| `sample_wall_ns` / `recv_wall_ns` | See 6.1 |
| `device_tick` | Device-side 1 ms counter |
| `joints_deg` | 21 joint angles (degrees); names from `revohuman.joint_map.joint_names()` |
| `joints_valid_mask` / `all_joints_valid` | Per-channel valid bits |
| `tactile` | `TactileData`; `None` when the product model has no tactile. Still attached when this beat's tactile data is invalid, with `tactile.usable` set to `False`. With tactile split, it holds the latest tactile state, and is also `None` before the first packet arrives (6.4) |
| `handedness` | `Handedness.LEFT` / `RIGHT` |
| `sync_key` | Key for exact bimanual pairing |
| `gate_open` | Whether the timestamp gate is open; only ungated subscriptions (6.3) receive frames with `False` |
| `skipped_snapshots` / `dropped_local` | Count of snapshots skipped on the device side / frames dropped from the local queue |

**`TactileData`**: `values`, `point_count`, `valid`, `link_ok`, `age_ms`, `unit`, `calibrated`, `device_tick`, `sample_wall_ns`,
the health fields `link_state`, `fault_reason`, `health_flags`, `reconnect_attempts`, `sequence`, and the criterion property `usable` (6.4).
`unit` is carried with each frame: `"mN"` means calibrated force values (millinewtons); `"device_native"` means raw counts, not force.

**`PairedFrame`**: `left`, `right`, `skew_ns`.

**`TrackerFrame`**: `serial`, `seq`, `handedness`, `pose` (`TrackerPose`: pose in the base station frame, `position` in meters, `rotation` as a quaternion w,x,y,z; see 4.7),
`sample_wall_ns`, `recv_wall_ns`, `timecode` (raw device 48 MHz value, wraps around about every 89.5 seconds), `tc_unwrapped`.
**`PairedTrackerFrame`**: `left`, `right`, `skew_ns`.

All frames have `to_dict()` / `from_dict()` (the format used in recording files). `from_dict()` parses strictly and raises on any missing field,
so that data from another version is not silently read as default values; the exceptions are `calibrated` / `unit` (when missing, read back as uncalibrated raw counts) and the tactile health fields (absent in older recordings; when missing they read back as `None`, and `usable` falls back to `valid` and `link_ok`).

### 6.3 How to read subscriptions

All SDK interfaces are synchronous functions; capture runs in the SDK's own background threads, and the caller does not need to poll the devices.
There are two kinds of interfaces for getting data: a **subscription** delivers every frame in order without skipping any; a **latest value** does not block and immediately returns the most recent one.

**Which calls block**

| Call | Blocks | Notes |
|---|---|---|
| `open_kit()`, `connect()`, `connect_pair()` | Yes | Opens the devices and runs the startup checks; `open_kit()` waits for the base stations and Trackers to be ready, about 15 s. Returns without waiting for the first frame |
| `close()`, leaving `with` | Yes | Stops capture and waits for the background threads to exit |
| `sub.recv(timeout=t)` | Up to `t` seconds | Raises `queue.Empty` on timeout; `timeout=0` does not wait; omitting `timeout` waits indefinitely |
| `await sub.recv_async(timeout=t)` | No (suspends the coroutine) | The capture thread wakes the event loop directly; no thread pool is used |
| `rt.latest(role)` / `rt.delta(role)` (`kit.tracker`, 4.6) | No | Returns `(None, None)` / `None` when there is no pose yet |
| `cam.latest()` (5.4) | No | Returns `None` when there is no frame yet |
| `glove.button.read()` (5.6) | No | Returns the latest button state |
| `rec.start()` / `h.stop()` (5.5) | Yes | `stop()` returns only after the file has been completely written |
| `stats()` / `status()` | No | Snapshot; `status()` does not raise |
| `selftest()` / `calibrate()` | Yes | This hand stops streaming in the meantime (5.1) |

Opening, closing, and starting/stopping recording have no asyncio versions; in an asyncio program, call them with something like `await asyncio.to_thread(open_kit, "P1")`.

**Reading a subscription** (the same for glove, bimanual pairing, button, Tracker and camera subscriptions)

| Pattern | Code |
|---|---|
| Synchronous, one frame | `f = sub.recv(timeout=1.0)` |
| Synchronous loop | `for f in sub:` |
| Callback | `x.subscribe(callback=on_frame)` |
| Asynchronous, one frame | `f = await sub.recv_async(timeout=1.0)` |
| Asynchronous loop | `async for f in sub:` |

- **Queue**: each subscription is an independent bounded queue, and every frame is delivered to every subscription. Default capacity: glove and bimanual pairing 64, button 16, Tracker (`src.tracker(serial).subscribe()`) 128,
  Tracker pairing 64, camera 8; adjust with `subscribe(queue_size=...)`. When the queue is full, **the oldest frame is dropped**, without affecting capture or other subscriptions; the drop count is in `sub.dropped_count`.
- **One subscription, one reader**: when several threads read the same subscription, each frame goes to only one of them. For multiple consumers, call `subscribe()` once for each.
- **Callbacks**: each callback subscription has its own background thread that invokes the callback frame by frame, in order; a slow callback likewise causes the oldest frames to be dropped, as above. An exception raised by the callback does not interrupt capture and is **not printed**;
  only the most recent one is kept in `sub.last_error`. Do not call `recv()` on a callback subscription, and do not close a callback's own subscription from inside the callback.
- **Latest value**: gloves have no "latest frame" interface; when you only need the latest value, drain the backlog with `recv(timeout=0)` and keep the last frame.
  `rt.latest()` caches only frames that carry a pose: after occlusion, a fault, or close, it still returns the last pose, and `age_s` (seconds since it was received, host monotonic clock) keeps growing; use it to judge freshness.
- **The glove's timestamp gate**: by default a subscription delivers only frames with qualified timestamps. During startup warm-up, while timestamp quality is degraded (`N2102`), and during disconnect/reconnect, no frames are produced, and the reader only sees `queue.Empty`;
  the gate state is in `glove.capture_ok` / `glove.capture_cause` (6.6). After a reconnect, the frames' `generation` increases by 1.

**Close and exceptions**

- After a subscription, device or kit is closed, `recv()` / `recv_async()` raise `Disconnected` (1101). `async for` ends accordingly;
  a synchronous `for f in sub:` that is blocked waiting at the time of close raises `Disconnected`, so wrap it in `try`, or use a `recv(timeout=...)` loop and check `sub.closed`.
- The close notification is delivered only once. After getting `Disconnected`, do not call `recv()` on the same subscription again: without `timeout` it waits forever, with `timeout` it raises `queue.Empty`.
- For a subscription closed because of an unrecoverable fault, `recv()` raises the corresponding exception, and `async for` does not swallow it: `WorldFrameChanged` (1902), `GeometryLost` (1906),
  `TrackerStreamDead` (1901) for Tracker subscriptions, `FirmwareIncompatible` (1205) for glove subscriptions, and so on. Catch them all with `RevoHumanError` (6.5).
- `rt.latest()` / `rt.delta()` / `status()` do not raise. Check for unrecoverable Tracker faults with `kit.fatal` (`None` means normal) or `kit.raise_if_fatal()`;
  after such a fault the Tracker stops while glove data keeps flowing as usual, and `kit.glove()` / `kit.camera()` / `kit.synchronizer()` raise that fault instead.

**Ungated subscription**: `glove.subscribe(gated=False)` keeps delivering frames while the timestamp gate is closed (`frame.gate_open == False`; the `sample_wall_ns` of these frames is not guaranteed to be accurate). It suits display-only use with no cross-source alignment; the recorder does not accept such subscriptions.

**Joint angle smoothing** (off by default): smooths joint angles for display and control.
`glove.subscribe(joint_filter=...)` applies causal smoothing only to the frames received by that one subscription (requires numpy); the original frames and other subscriptions are not affected:

```python
view = glove.subscribe(joint_filter="default", gated=False)   # display: smoothing + ungated; default latency 20 ms @200 Hz
fast = glove.subscribe(joint_filter="savgol:window=7,order=1") # lower-latency alternative (15 ms)
print(view.joint_filter.describe(200.0))                       # window length, nominal latency, white-noise reduction ratio
```

- Spec strings: `default` (= `gaussian:sigma=2.5,truncate=1.5`, 9 samples), `gaussian:sigma=,truncate=`,
  `savgol:window=,order=[,pos=center|end]`, `ma:window=`, `none`; you can also pass a `revohuman.JointFilter` instance (one instance can be attached to only one subscription).
- Each channel is independent: smoothing starts from the first frame (constant input comes out unchanged); it restarts on reconnect (`generation` changes) or when the frames dropped exceed the window length; channels whose `joints_valid_mask` bit is 0 pass through unchanged.
- The cost is pure lag: during fast motion the filtered value trails the raw value by about the "nominal latency". The recorder rejects smoothed subscriptions (5.5).

For complete examples (blocking reads, callbacks, and asyncio) see *04 Recording*, section 9.

### 6.4 Tactile

- When the product model has no tactile, `frame.tactile` is `None`.
- To require tactile: `connect(..., require_tactile=True)`; it raises `TactileUnsupported` if the model has no tactile, and `TactileUndetermined` if this cannot be determined.
- If the two hands have different tactile variants, `VariantMismatch` is raised.
- **Whether a tactile reading is usable is given by `tactile.usable`** (= `valid`, link online, and `fault_reason == 0`); do not look only at `link_ok`:
  `valid` can still be false while the link is online, in which case the point values are the stale values of the last good sample and `sequence` does not advance; it is not new data.
  `revohuman.tactile_health.block_reason(tactile)` gives the reason it is unusable (`None` when usable).
- Tactile health statistics are in `glove.stats().stream["tactile_health"]` (`None` when the model has no tactile and the diagnostic counters have never been read):
  `frames` / `usable` / `invalid` / `not_online` / `valid_not_online` (normal transition at startup and recovery) / `online_invalid` / `fault_nonzero`,
  the maximum run of consecutive unusable readings, the `by_link_state` / `by_fault` distributions, `recent_unusable` (about the last 1 second), and `last` (raw fields of the most recent reading and why it was unusable).
  A pair opened with `connect_pair()` additionally carries `f4`: diagnostic counters of the tactile link, read once before the stream starts as a baseline, then once each time bimanual sync is rebuilt (while the stream is stopped) and once after `close()` stops the stream;
  `last_window` is the increment between two readings. When older firmware does not support them, `supported` is `False` (not treated as zero faults); read failures are counted in `errors` / `last_error`;
  no difference is taken for a window in which the glove lost power and restarted (`restarts_detected` increases by 1). They are not read while the stream is running.

**Tactile split** (off by default): `connect(..., split_tactile=True)` (bimanual: `connect_pair(..., split_tactile=True)`)
makes tactile data report sample by sample at the sensor module's own rate (about 215 Hz), instead of one reading being attached to each joint frame.

```python
with GloveManager().connect(handedness="right", split_tactile=True) as glove:
    print(glove.tactile_transport)           # "split"; "combined" when the firmware does not support it
    joints = glove.subscribe()               # joint frames still at 200 Hz; f.tactile is only the latest tactile state
    touch = glove.subscribe_tactile()        # one TactileSample per tactile sample
    s = touch.recv(timeout=1.0)
    if s.usable:
        print(s.tactile.sample_wall_ns, s.tactile.values[:5])
```

- When `glove.tactile_transport` is `"split"`, the `tactile` on joint frames holds **the most recently received tactile state**, not a sample from this beat:
  if no new tactile packet has arrived for more than 250 ms it is marked invalid (`valid` is false); it is `None` only before the first tactile packet of this connection (or after a reconnect) arrives.
  It is only for judging "is tactile usable right now" (the bimanual tactile gate uses it, 5.2); per-sample tactile data is available only from `subscribe_tactile()`.
  When the firmware does not support split, the connection still succeeds, the value is `"combined"`, tactile stays on the joint frames as before, and `subscribe_tactile()` receives nothing.
- `TactileSample`: `tactile` (tactile data of the same shape as on joint frames; `sample_wall_ns` is this sample's sampling time), `generation`,
  `packet_seq` (+1 per packet), `cycle_sequence` (the joint cycle number in effect when the sample was received), `dropped_samples` (samples dropped on the device side
  since this stream was started, cumulative), `overflow`, `recv_wall_ns` (arrival time), `handedness`. A packet that only reports health status has `tactile.valid` false,
  and its point values may be empty; use `s.usable` to judge whether it is usable.
- Time alignment: split tactile and joints are not sampled in the same cycle; pair them by nearest `tactile.sample_wall_ns` and joint frame `sample_wall_ns`.
  The sampling time of a split sample is taken at the moment "this sample's acquisition completed", which differs from how the tactile sampling time is taken in combined frames; do not align data from the two modes with the same fixed offset.
- `gated` for `subscribe_tactile()` works as for `subscribe()` (6.3); callbacks run in the dispatch thread and must not block.

**Tactile tare**: `glove.tactile_tare(timeout=None)` makes the glove perform a tare; this is the same as long-pressing the glove button for about 2 seconds.
It blocks until finished (about 3.5 s, during which the glove shows a blue light; `timeout` defaults to the wait time reported by the device + 2 s) and returns `glove.tactile_tare_state()`.

- Success means the tare command was sent to the tactile module (there is no acknowledgement of whether the module completed it). Rejection, failure or timeout raises `TactileTareError` (1805); it is **not retried automatically**.
- Taring is done inside the glove: tactile values read afterwards are already tared; the host does not subtract a baseline.
- When the firmware does not support triggering from the host, the request is rejected outright, and taring can only be done by a long press on the glove; when the product model has no tactile, `TactileTareError` is raised as well.
- Before calling, keep the glove still and not touching anything. Do not call during recording.
- `glove.tactile_tare_state()` describes **the most recent tare request**, whether it came from a program or from a long press on the glove:
  `available` (whether the model has tactile), `supported` (whether the firmware supports triggering from the host; `None` = not yet detected since connecting),
  `tared` (the most recent tare command was sent and there has been no reconnect / reset since), `in_progress`,
  `origin` (`this_host` this program / `glove_button` long press on the glove / `other_host` another host / `unknown` before this program started),
  `outcome`, `message`, `stamp_ns`, `stale` (reconnected since), `mcu_reset`.

### 6.5 Exceptions and error codes

All exceptions inherit from `RevoHumanError` (**not** `RuntimeError`; catch `RevoHumanError` or the specific type).
`revohuman.error_code(exc)` gives a machine-checkable error code, and `format_error(exc)` gives readable text that includes the code.

| Code | Exception | Scenario |
|---:|---|---|
| 1000 | `RevoHumanError` | General |
| 1101 | `Disconnected` | Connection lost / subscription closed |
| 1102 | `PortBusyError` | Serial port in use by another process (holder pid attached) |
| 1201 | — | Abnormal device response |
| 1202 | `UnsupportedFeatureError` | Firmware does not support the feature |
| 1203 / 1204 | Subclasses of `ProtocolError` (not exported separately) | Firmware does not support / is incompatible with the time sync feature; check for 1203 / 1204 with `error_code(exc)`, or catch `ProtocolError` |
| 1205 | `FirmwareIncompatible` | Incompatible firmware version (unrecoverable) |
| 1300 | `CalibrationError` | Encoder calibration failed |
| 1400 | `UnregisteredGloveError` | Neither the handedness recorded in the MCU firmware nor the serial number identifies the hand; specify it by serial port with `port=` instead |
| 1401 | `GloveNotPresent` | The hand to be opened is not present (`present` lists which hands are present) |
| 1403 | `HandConflictError` | The handedness recorded in an MCU's firmware contradicts its serial number; confirm which unit is this hand, then use `port=` |
| 1500 | `StreamRateConflict` | Repeated subscriptions at different rates on the same connection |
| 1600 | `CaptureGateClosed` | Recording started while the timestamp gate is closed |
| 1700 | `StrictSyncLost` | Bimanual hardware sync lost |
| 1800 / 1801 | `TactileUnsupported` / `TactileUndetermined` | Tactile required but the model has none / cannot be determined |
| 1802 | `VariantMismatch` | Left and right hands have different tactile variants |
| 1803 | `ProductUnidentified` | Product model cannot be identified (conflicting or insufficient evidence) |
| 1804 | `ProductMismatch` | The declared product model does not match the hardware, or a sensor is missing in strict mode |
| 1805 | `TactileTareError` | Tactile tare failed (model has no tactile / firmware does not support triggering from the host / rejected / timed out) |
| 1900 | `TrackerBusyError` | libsurvive is already in use by another process |
| 1901 | `TrackerStreamDead` | No Tracker data after startup; or all Trackers stopped streaming for 10 seconds while running (4.4 R4) |
| 1902 | `WorldFrameChanged` | A base station was detected as moved during capture, or a base station outside the allowlist was solved into the coordinate frame (see 4.4) |
| 1903 | `LighthouseError` | Base station allowlist, channel preflight check, or a base station write operation did not pass (`reasons` lists each item) |
| 1904 | `CalibrationRejected` | Calibration rejected or not completed (`stage`: `preflight` / `geometry` / `timeout` / `reference` / `state`) |
| 1905 | `TrackerNotReady` | Tracker startup check did not pass (`kind` / `reasons` / `remedy`, see 4.2) |
| 1906 | `GeometryLost` | Base station geometry lost during capture (`recoverable` distinguishes the two severities, see 4.4) |

### 6.6 Runtime notices (N codes) and logging

Error codes (6.5) answer "which exception was raised"; runtime notices answer "what happened while running". SDK notice logs always begin with the code:

```
N2102 [/dev/ttyACM0] 录制门关：时间基准质量下降，帧暂停派发（门开后自动恢复）
```

The `录制门` (recording gate) in notices and errors is what this manual calls the timestamp gate.

- Identify the code by the first 5 characters; there is no need to parse the Chinese text. Structured fields are attached to the `LogRecord`: `record.revohuman_code` (e.g. `"N2102"`),
  `record.revohuman_fields` (a dict with port, handedness, count, duration, etc.); log handlers can convert them to JSON directly.
- The full table is `revohuman.notices.Notice`; `revohuman.notices.notice_spec(code)` gives the default severity and text template.
- The command-line tools (`python3 -m revohuman`, `revohuman-live`, `revohuman-lighthouses`) and the 3D visualization color log lines in the terminal by severity:
  DEBUG dim, INFO uncolored, WARNING yellow, ERROR red, CRITICAL bold red. This is controlled by the environment variable `REVOHUMAN_LOG_COLOR`:
  `auto` (default; colored when the output is a terminal, `NO_COLOR` is not set, and `TERM` is not `dumb`) / `always` / `never`.
  When redirected to a file or a pipe, the output has no color and is byte-for-byte identical to uncolored output. When the SDK is used as a library it does not configure log handlers, so there is no coloring.

| Code range | Category | Examples |
|---|---|---|
| 21xx | Single-hand timing and recording gate | 2101 recording gate open · 2102 recording gate closed · 2103 recording gate reopened · 2104 / 2105 timestamp quality degraded / restored · 2107 abnormal transport latency |
| 22xx | Bimanual sync and pairing | 2201 fell back to software pairing · 2202 hardware sync lost · 2203 rebuilt · 2204 rebuild failed · 2205 / 2206 alignment deviation over budget / restored · 2207 / 2208 tactile pairing paused / resumed |
| 23xx | USB link | 2301 link unstable · 2302 resetting · 2303 recovery failed · 2304 recovered |
| 24xx | Stream | 2401 reconnecting · 2402 recovered · 2403 serial port name changed · 2404 stream interrupted · 2405 firmware incompatible · 2406 firmware does not support button reports during capture · 2407 SDK program error during connect |
| 25xx | Recording | 2501 entire episode discarded · 2502 recording aborted because bimanual sync was lost |

Time quality problems are reported only by **cause category** (`revohuman.notices.SyncCause`), which is enough to decide what to do:

| `cause` | Meaning | Action |
|---|---|---|
| `clock_not_ready` | Time base not yet established or expired; being rebuilt | Usually recovers by itself within seconds; no action needed |
| `clock_degraded` | Time base quality degraded | If it persists, check USB cables, HUBs and host load |
| `host_clock_event` | Host clock jump (wake from sleep / manual time change / time-sync step) | Recovers automatically; avoid during recording, see 10 |
| `device_reset` | Device reset | Recovers automatically; if frequent, check the power supply |
| `link_unstable` | USB link unstable | Check cables, HUBs and power supply |

Where to read it: `glove.capture_cause` (`None` while the gate is open), `glove.stats().stream["capture_gate"]["cause"]`,
`glove.stats().mapper_quality["cause"]`, `detail["cause"]` in the `glove.on_capture_gate` callback (`(glove, ok, detail)`),
and the field of the same name in the ROS `status`.

### 6.7 Hand forward kinematics and keypoints

`revohuman.hand_fk` computes the pose of each finger segment from the bundled hand model, and `revohuman.hand_keypoints` builds on it to give the nominal positions of human hand keypoints;
both depend only on the Python standard library, and their results match the same-named interfaces of the C SDK. Get the model name with `variant_hand_model()` (2.2):

```python
from revohuman import hand_fk, hand_keypoints, joint_map
from revohuman.product import variant_hand_model

model = variant_hand_model("PTV1")                                # "dv1"
q = joint_map.to_radians(frame.joints_deg)                        # firmware angles (deg) → joint angles (rad)
links = hand_fk.forward_kinematics(q, frame.handedness, model=model)          # 21 segments, palm_link frame
palm = hand_fk.palm_from_tracker(tf.pose.position, tf.pose.rotation, tf.handedness, model=model)
kp = hand_keypoints.hand_keypoints(q, frame.handedness, model=model, valid_mask=frame.joints_valid_mask)
```

| Interface | Description |
|---|---|
| `hand_fk.forward_kinematics(q, hand, base=None, model=)` | 21 joint angles → 21 `LinkPose` (names from `link_names(hand)`), in the palm_link frame by default; when `base` is given, the result is `base ∘ result` |
| `hand_fk.palm_from_tracker(position, rotation_wxyz, hand, model=)` | `TrackerPose` → pose of palm_link in the same coordinate frame (nominal mounting relation, not calibrated per unit) |
| `hand_fk.hand_points(q, hand, model=, valid_mask=None)` | 21 segments + 5 fingertips + palm_link origin, 27 points in total, in the tracker_link frame, with position / orientation valid bits |
| `hand_keypoints.hand_knuckles(q, hand, model=, valid_mask=None)` | Nominal centers of the MCP joints of the index, middle, ring and little fingers (4 poses) |
| `hand_keypoints.hand_keypoints(q, hand, model=, valid_mask=None)` | Positions of 22 keypoints (wrist point, thumb CMC/MCP/IP/TIP, MCP/PIP/DIP/TIP of the four fingers, palm center) and the palm frame `palm_r`; names from `keypoint_names(hand)` |

- `LinkPose` = `position` (meters) + `rotation` (3×3, stored row by row; column j is the j-th axis); `palm_r` is also stored row by row,
  with Z the palm normal pointing out of the palm surface and X pointing toward the middle finger.
- The points from `hand_points` are the origins of the exoskeleton segments, on the back-of-hand layer; keypoints are moved into the fingers by nominal constants and are not calibrated values for each glove.
  The wrist point is a nominal reference point, not the anatomical center of the wrist joint.
- When a joint angle is non-finite or the corresponding bit of `valid_mask` is 0, the affected points are NaN and their valid bits are 0 (valid bit 1 ⇔ the point is entirely finite).
- The joint tables and axis conventions of OpenXR and other skeletons are not in the SDK; users convert from these points according to their own conventions.
- `hand` takes `"left"` / `"right"` or a `Handedness`; `model` takes `"dv1"` / `"p"`; other values, or a joint angle count other than 21, raise `ValueError`.
- The wrist camera is not among these 21 segments: get the camera mounting pose relative to palm_link (the same as `{side}_camera_Link` / `{side}_camera_optical_frame` in the URDF)
  with `revohuman.calibration.wrist_camera_extrinsic`; for its conventions and caveats see 5.7.

---

## 7. ROS 2

### 7.1 Build and launch

```bash
mkdir -p ~/ros2_ws/src && ln -s $PWD/ros2/* ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch revohuman_bringup revohuman.launch.py                 # both hands, product model detected automatically
ros2 launch revohuman_bringup revohuman.launch.py mode:=left      # single hand
ros2 launch revohuman_bringup revohuman.launch.py product:=PTV1   # declare the product model
ros2 launch revohuman_bringup revohuman.launch.py --show-args     # all tunable parameters
```

Three packages: `revohuman_msgs` (`TactileArray`, `Calibrate`), `revohuman_ros` (nodes), `revohuman_bringup` (launch files and parameters).
Run the commands above in the `revohuman-<版本>/` directory (`<版本>` = version) extracted from the source bundle. The ROS nodes need to be able to import `revohuman`:
first install the wheel from §1 into the Python 3.10 environment that ROS uses.

| `mode` | Node | Namespace |
|---|---|---|
| `pair` (default) | `bimanual_node`, 42 joints | `/revohuman/pair` |
| `left` / `right` | `glove_node`, 21 joints | `/revohuman/left` / `/revohuman/right` |

Parameter precedence: command line > `params_file` (default `config/revohuman.yaml`) > node defaults.
Do not launch `mode:=pair` and `mode:=left|right` at the same time (a glove can be opened by only one process).

A node exiting with exit code 5 means it is switching Tracker mode (runtime ↔ base station calibration), and launch restarts it automatically; after more than 10 restarts within 60 seconds, it stops restarting.

### 7.2 Product models and sensors

At startup the node identifies the product model as described in 2.2 and decides from it which sensors to enable:

- `product`: `auto` (default) / `P1` / `PTV1` / `off` (no identification; entirely governed by the parameters below).
- Tactile: if the model has tactile but it cannot be read on one of the hands → the node refuses to start (with `product_strict:=false`, it only warns). `publish_tactile:=off` turns it off.
- Camera: when `camera_mode` is left empty, it is `required` for PTV1 and `off` for P1; an explicit value takes precedence.
- Tracker: both models have it; controlled by `tracker_mode` (default `required`).

### 7.3 Topics

Single hand (`<ns>` = `/revohuman/left` or `/revohuman/right`):

| Topic | Type | Description |
|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | 21 joints, in **radians** |
| `tactile` | `revohuman_msgs/TactileArray` | Present only when the glove has tactile |
| `tactile_tare_state` | `std_msgs/String` (JSON) | Tactile tare state `{"stamp_ns", "<hand>": tactile_tare_state()}` (6.4); published at startup and on every change; same QoS as the status topics |
| `tracker_pose` | `geometry_msgs/PoseStamped` | Absolute pose in the base station frame (4.7); `frame_id` is set by `tracker_frame_id` (default `world`) |
| `tracker_delta` | `geometry_msgs/PoseStamped` | Increment relative to the zero pose, expressed in the Tracker's own frame at the zero-pose moment (4.6); `frame_id` = `revohuman_<hand>_tracker_zero`; published only after zero-pose calibration |
| `wrist_camera/image/compressed` | `sensor_msgs/CompressedImage` | Present only when the camera is enabled; raw MJPG bytes |
| `status` / `tracker_status` / `wrist_camera/status` | `std_msgs/String` (JSON) | 1 Hz status |

Both hands (`/revohuman/pair`): `joint_states` (21 left + 21 right), `tactile_left|right`, `tracker_pose_left|right`,
`tracker_delta_left|right`, `wrist_camera_left|right/image/compressed`, `status`, `tracker_status`, `wrist_camera_status`,
`tactile_tare_state` (two entries, `left` and `right`); with `button:=true` there is also `button_event`.

`button_event` (`std_msgs/String`, JSON): one message per valid glove press, with the fields `hand`, `mode` (`start` / `end`), `press_sequence`,
`press_count`, `missed_presses`, `recv_wall_ns`, `sample_wall_ns`, `seq`, `generation`, `event_id` (monotonically increasing within the node, for deduplication).
QoS is RELIABLE + **VOLATILE**, depth 10: a subscriber that connects late does not receive earlier presses, so it never mistakes an old press for a new start / end.
The node only forwards presses and does not control recording; if the firmware does not support the button, capture continues as usual and `button.<hand>.supported` in `status` is `false`.

- Data topic QoS: BEST_EFFORT, depth 10.
- **Status topic QoS: RELIABLE + TRANSIENT_LOCAL, depth 1**. A subscriber with mismatched QoS receives nothing at all.
- Every `header.stamp` is a sampling time (same as the native SDK). Arrival times are not published.
- `tracker_pose` / `tracker_delta` are raw poses (without the occlusion-recovery smoothing of 4.6); quaternions follow the ROS convention (x, y, z, w).
  The node does not publish tf; it only writes `frame_id` in the header. To connect to a tf tree, broadcast the transforms yourself.
- For bimanual `joint_states`, `header.stamp` is the left hand's sampling time; under hardware sync both hands share the same time, otherwise the right hand differs by no more than `pair_accuracy_ms`.
- `TactileArray`: `header.stamp` is the tactile data's own sampling time, and `frame_stamp` equals the `header.stamp` of the `joint_states` of the same cycle (pair them by equal values of it);
  when `calibrated` is false, `data` is raw counts.
  The exception is when `split_tactile` is on (tactile split; on by default in `revohuman_bringup`, see 7.5): tactile is published sample by sample at the sensor module's own rate (about 215Hz),
  `frame_stamp` equals `header.stamp` and does not correspond to any joint frame, so pair by nearest `header.stamp`.
- Bimanual node: when both gloves have tactile and `publish_tactile` is `auto`, the bimanual tactile gate is enabled (5.2). While the gate is closed,
  `joint_states` and `tactile_left|right` are **withheld for the whole cycle** (per-sample split tactile is not published either), and publishing resumes automatically after recovery;
  see the current state in `pairing.tactile_gate_open` / `tactile_gate_reason` of `status`. With `publish_tactile:=off` the gate is not enabled and joints are published regardless.

### 7.4 Services

Glove services (single hand under `<ns>/`, both hands under `<ns>/left/` and `<ns>/right/`):

| Service | Type | Description |
|---|---|---|
| `selftest` | `std_srvs/Trigger` | Read-only self-test; this hand stops streaming for about 50 ms; do not call during recording |
| `get_sn` | `std_srvs/Trigger` | Reads the serial number |
| `tactile_tare` | `std_srvs/Trigger` | Tactile tare (6.4; only sends the tare command and does not touch the encoder calibration); always provided; blocks about 3.5 s; on success `message` is the tare state JSON, on failure it begins with `E1805` |
| `calibrate` | `revohuman_msgs/Calibrate` | Encoder calibration (writes to the device); provided only with `allow_calibration:=true` |
| `clear_calibration` | `std_srvs/Trigger` | Clears the encoder calibration (writes to the device); same condition as above |

The ROS interface does not provide serial number writing.

Tracker services (under `<ns>/`):

| Service | Type | Description |
|---|---|---|
| `tracker/set_reference` | `std_srvs/Trigger` | Zero-pose calibration; on success `message` is the result JSON |
| `tracker/get_status` | `std_srvs/Trigger` | Returns the Tracker status JSON |
| `tracker/set_calibration_mode` | `std_srvs/SetBool` | `true`: enter base station calibration; `false` during calibration: abort and roll back |

**Calibrating base stations in ROS**: first complete steps ①–③ of 4.1 (allowlist and scan) on the command line, then

```bash
ros2 service call /revohuman/pair/tracker/set_calibration_mode std_srvs/srv/SetBool "{data: true}"
```

If the allowlist or the channel preflight check does not pass, the request is rejected outright. Once they pass, the node exits and restarts in calibration mode (gloves are not opened in this mode),
publishing `{"mode": "calibrate", "state", "coverage", "hint", "trackers"}` on `tracker_status` so that a UI can guide the operator;
after completion, failure or abort it returns to runtime mode automatically. Then call `tracker/set_reference` for zero-pose calibration.

ROS nodes do not perform the startup checks of 4.2 or the runtime protection of 4.4 (except the world-frame gate, which is controlled by `tracker_world_gate_mode` and applies as usual):
at startup they do not block and do not record a zero pose automatically; instead they reuse the zero pose last saved to disk
(same conditions as `restore=True` in 4.3). If there is no zero pose to reuse, call `tracker/set_reference`.
Whether the base station geometry meets the requirements is shown in the `geometry` field of `tracker_status`; when `tracker_geometry_gate_mode` is `error`, zero-pose calibration is refused while the geometry does not meet the requirements.

### 7.5 Parameters

For the full list with comments, see [`ros2/revohuman_bringup/config/revohuman.yaml`](../ros2/revohuman_bringup/config/revohuman.yaml). Common parameters:

| Parameter | Default | Description |
|---|---|---|
| `product` / `product_strict` / `product_rules_path` | `auto` / `true` / empty | Product model identification (7.2) |
| `rate_hz` | `200.0` | Glove rate |
| `publish_tactile` / `require_tactile` | `auto` / `false` | Tactile publishing |
| `split_tactile` | `true` | Tactile split: publish sample by sample at the sensor module's own rate (for pairing see 7.3); falls back to combined frames automatically when the firmware does not support it; set `false` if you need to pair by equal `frame_stamp` |
| `allow_calibration` | `false` | Whether to provide the encoder calibration services |
| `button` | `false` | Bimanual node: enable the glove button; presses are published on `button_event` (7.3); capture continues as usual if the firmware does not support it |
| `pair_mode` | `strict` | `strict`: output only pairs from the same hardware sync cycle; `best_effort`: allow pairs within 1 ms |
| `pair_accuracy_ms` / `tracker_pair_accuracy_ms` | `-1.0` / `1.0` | Accuracy budget (ms) for glove / Tracker left-right pairing; a negative glove value follows `pair_mode` (0 for `strict`, 1.0 for `best_effort`); a negative Tracker value sets no budget |
| `tracker_mode` | `required` | `required` / `auto` / `off` |
| `tracker_left_serial` / `tracker_right_serial` / `tracker_serial` | empty | Override the automatically resolved Tracker serial numbers |
| `tracker_lighthouses` | empty | 8-digit ids of the enrolled base stations; if empty, resolved from the base station allowlist (4.1) |
| `tracker_world_gate_mode` / `tracker_geometry_gate_mode` | `error` / `error` | World-frame gate / geometry gate (7.4) |
| `tracker_stamp_accuracy_ms` / `tracker_phi_pack_ms` | `-1.0` / empty | Absolute accuracy budget for sampling times (negative = not required) / per-serial-number packing latency `LHR-…=毫秒` (milliseconds) (6.1); meeting the former requires filling in the latter first |
| `tracker_liveness_gate_s` | `-1.0` | Window (seconds) of the Tracker startup liveness gate; negative = chosen automatically by the SDK, `0` = off |
| `tracker_fence` / `tracker_require_preflight` | `true` / `true` | Foreign base station fence / require a preflight check before calibration |
| `tracker_calibration_timeout_s` | `300.0` | Base station calibration timeout |
| `tracker_state_dir` / `tracker_config_path` | empty | State directory / libsurvive configuration path |
| `tracker_survive_args` | empty | Extra libsurvive arguments, such as the orientation jump gate (4.8, off by default); the SDK's default arguments are added automatically |
| `tracker_frame_id` | `world` | `frame_id` of `tracker_pose`, i.e. the name of the base station frame (4.7) |
| `camera_mode` | empty (per model) | `required` / `auto` / `off` |
| `camera_device` / `camera_usb_hub` | empty | Camera device node / HUB restriction for a single-hand node; if empty, the camera is discovered automatically under the HUB of that hand's glove unit. Bimanual nodes use `camera_left_*` / `camera_right_*` |
| `camera_model` | empty | Wrist camera model (`uvc_28c4` / `uvc_3080`, 5.4); empty = identified automatically from the device; once set it is pinned, and the camera fails to start if the plugged-in camera is not that model |
| `camera_width` / `camera_height` / `camera_fps` / `camera_fourcc` | `640` / `480` / `30` / `MJPG` | Camera format |
| `camera_exposure` | `-1` | Exposure: ≤0 means auto exposure; a positive number means fixed exposure (in 100 µs steps, e.g. `333`, 5.4) |
| `camera_controls` | empty | Extra controls `name=value`, layered on top of that model's control table; a misspelled name makes startup fail |
| `camera_pts_offset_ms` | empty | Constant offset of PTS relative to the sampling time (milliseconds; `pts_offset_ns` in 5.4); an empty string means not calibrated; `0` and negative numbers are both valid values |
| `camera_frame_id` | empty | Image `frame_id`; if empty, `revohuman_<hand>_wrist_camera` |
| `camera_zero_copy` | `true` | Zero-copy publishing |

When parameters are passed through `ros2 launch`, launch converts the types automatically; list parameters are comma-separated (`tracker_lighthouses:=xxxxxxxx,yyyyyyyy`).
With `ros2 run` directly, or in yaml, the type must match the default value: write floats as `200.0`, not `200`;
the string `off` must be quoted, `-p "tracker_mode:='off'"` (otherwise it is parsed as a boolean).

### 7.6 Status JSON

- `status`: `handedness`, `effective_rate_hz`, `frames_by_tier`, `capture_gate`, `frames_blocked`, `mapper_quality`,
  `pairing` (bimanual), `product` (product model identification result), `node`, `tactile_tare` (6.4), `tactile_transport` (`split` / `combined`),
  `tactile_health` (tactile usability statistics and diagnostic counters, 6.4),
  `button` (bimanual: `enabled`, `published`, `publish_errors`, and for each of `left` / `right`: `supported`, `events_total`, `missed_presses_total`, `subscribed`).
- `tracker_status` (runtime mode): `mode` (`runtime`), `running`, `dead_reason`, `world` (world-frame gate state), `lighthouses`, `trackers`, `pairing`,
  `session` (`registered_lighthouses`, per-hand `online` / `pose_age_s` / `referenced`, `reference`, `geometry`, `startup`).
- `wrist_camera/status`: `running`, `resolved` (device path, `model`, `vid_pid`), `stream` (frame counts, `effective_fps`, `sample_phase_bound_ms`,
  `loan`, `stalled`, `fatal_error`, `model`, `vid_pid`, `usb_bcd_device` (camera firmware version), `exposure_mode`, `stamp_model`, `pts_offset_ms`,
  `meta_missing` / `meta_missing_reason`), `clock`, `config`, `node`. To check liveness, look at `stalled` / `fatal_error` and whether the frame counts keep increasing;
  if images never arrive, check `meta_missing` first.

---

## 8. Command-line reference

`python3 -m revohuman [-v] <命令>` (`<命令>` = command); every command except `live` supports `--json`.

| Command | Description |
|---|---|
| `doctor [--probe]` | Health check |
| `live [--hands both\|left\|right] [--product auto\|P1\|PTV1] [--no-tracker] [--tracker-gate flag\|error] [--tracker-phi-pack SERIAL=毫秒] [--sync best-effort\|on\|off] [--socket P]` | Local live broadcast for the 3D visualization and your own programs to subscribe to (`毫秒` = milliseconds); same as `revohuman-live`; for the options see *04 Recording* |
| `firmware info [--hands both\|left\|right] [--port P]` | Firmware version and upgrade status of the gloves present (read-only) |
| `firmware upgrade IMAGE.bin --version X.Y.Z (--hand left\|right \| --port P) [--allow-downgrade] [--yes] [--reconnect-timeout 45]` | Upgrades the firmware of one glove (2.4) |
| `glove calibrate (--hand left\|right \| --port P) [--index N]… [--yes] [--timeout 15]` | Encoder zero calibration, written to the glove and irreversible (2.5) |
| `product identify [--product auto\|P1\|PTV1] [--hands both\|left\|right] [--no-probe] [--rules P]` | Identifies the product model |
| `tracker lighthouses <子命令>` | Same as `revohuman-lighthouses <子命令>` (`<子命令>` = subcommand; see the table below) |
| `tracker preflight [--seconds 10] [--whitelist P] [--state-dir D]` | Quick version that only runs the channel preflight check: passive scan, does not connect to base stations |
| `tracker check [--live]` | Startup self-check: by default reads files only; `--live` starts one capture and also checks base station data and Tracker poses |
| `tracker calibrate [--timeout 300] [--min-sec 3] [--stable-sec 3] [--coverage-cells N] [--coverage-span-deg DEG] [--no-fence] [--no-preflight] [--quiet]` | Base station geometry calibration |
| `tracker set-reference [--wait 5]` | Zero-pose calibration |
| `tracker status [--state-dir D] [--config P] [--whitelist P] [--registered ID]…` | Offline view of the allowlist, preflight check, geometry, zero pose and the last startup check |

`check`, `calibrate` and `set-reference` also accept `--tracker left=LHR-... --tracker right=LHR-...`, `--hands`,
`--whitelist`, `--state-dir`, `--config`, and `--registered ID` (8-digit id of an enrolled base station, repeatable; when given, the base station allowlist is not read);
when these are not given, they are resolved automatically from the glove units and the base station allowlist.

Exit codes: `0` success; `1` failure, not ready, or blocked by the startup check; `2` usage error, or an error in the tool itself; `3` rejected or product model mismatch (`set-reference` blocked by the startup check also gives `3`). For the exit codes of `live`, see *07 Logging and Error Codes*, section 5.

Base station commands `revohuman-lighthouses [-v] <子命令>` (requires the `ble` extra); all accept `--whitelist`, `--state-dir` and `--json`:

| Subcommand | Description |
|---|---|
| `scan [--seconds 10] [--no-connect]` | Scans nearby base stations, checks them against the allowlist, and looks for same-channel conflicts; refreshes the channel preflight check when it passes |
| `show` | Offline view of the allowlist resolution result (no Bluetooth needed) |
| `init` | Writes the configuration file from the example (left untouched if it already exists) |
| `plan [--pin FA…=CH] [--avoid CH] [--seconds 10] [--ignore-optical]` | Computes and prints a channel change plan without writing anything |
| `apply --yes [--pin FA…=CH] [--avoid CH] [--seconds 10] [--ignore-optical]` | Rescans, recomputes and executes the plan (without `--yes` it is the same as `plan`) |
| `set-channel FA… CH [--yes] [--seconds 10] [--ignore-optical]` | Moves one base station in the allowlist to the specified free channel (without `--yes` it only prints) |
| `identify FA…\|LHB-… [--seconds 5] [--foreign] [--scan-seconds 6]` | Makes a base station blink; a foreign station requires `--foreign` and is recorded in the write log |

For exit codes, see 4.1.

Scripts in the source bundle (all support `--help`):

| Script | Description |
|---|---|
| `install_venv.sh [--venv DIR] [--wheels DIR] [--python PY] [--mirror cn\|official] [--no-tracker] [--no-system] [-y] [--force] [--dry-run]` | One-step install into a virtual environment (without ROS 2); see *01 Installation and System Requirements*, section 11 |
| `calibrate_lighthouses.sh [--hands both\|left\|right] [--tracker 手=序列号]… [--whitelist FILE] [--state-dir DIR] [--config FILE] [--scan-seconds S] [--timeout S] [--no-check] [-y] [--venv DIR\|none] [--docker [--image IMG] [--name NAME] [--restart] [--data DIR] [--state DIR] [-e K=V]…] [-- calibrate 参数…]` | (`手=序列号` = hand=serial number; `calibrate 参数` = arguments for calibrate.) Check allowlist → identify Trackers → scan → calibrate → `check --live`: base station calibration done in one command; it has the additional exit code `4` (the allowlist or the scan needs to be dealt with first). See *03 Base Stations and Pairing*, section 5.4, and *08 Running in Containers*, section 7.3 |
| `simulation/run_sim_host.sh [--venv DIR\|none] [--hands …] [--product …] [--no-tracker] [--demo [P1\|PTV1]] [--log FILE] [--socket PATH] [可视化参数…] [-- live 参数…]` | (`可视化参数` = visualization options; `live 参数` = live options.) Starts `live` in the background and opens the 3D visualization in the foreground; closing the window also stops the broadcast; if a broadcast is already running, only the window is opened. See *05 Simulation and Visualization*, section 4.1 |
| `simulation/run_sim_docker.sh [--image IMG] [--name NAME] [--restart] [--data DIR] [--state DIR] [--ble] [-e K=V]… [其余同 run_sim_host.sh]` | (`其余同 run_sim_host.sh` = other options as for run_sim_host.sh.) Same as above, with both the broadcast and the window running in a container; container options are passed unchanged to `docker/run.sh`. See *08 Running in Containers*, section 8 |
| `simulation/run_viewer.sh [可视化参数…]` | Starts only the 3D visualization (builds its dependency environment on first run); see *05 Simulation and Visualization*, section 6 |
| `docker/install.sh` / `docker/run.sh` | One-step image build / start and enter the container; see *08 Running in Containers* |

---

## 9. Directory layout and examples

Layout of the release files: the core SDK is in the wheel (installed into site-packages by `pip install`), and the source bundle extracts to:

```
revohuman-<版本>/
  ├ README.md           Install and quick start (English; Chinese: README.zh-CN.md)
  ├ LICENSE / NOTICE    License (Apache-2.0) and copyright notice
  ├ install_venv.sh     One-step install into a virtual environment (without ROS 2)
  ├ calibrate_lighthouses.sh  One-step base station calibration (scan → calibrate → startup self-check)
  ├ ros2/               ROS 2 packages (see below)
  ├ models/             Hand models (one model package per product: URDF and meshes)
  ├ examples/           Examples
  ├ docs/               User manual (from 00-overview.md) and this reference manual (reference-manual.md); Chinese copies in docs/zh-CN/
  ├ simulation/         Simulation and visualization (run_sim_host.sh / run_sim_docker.sh start the broadcast and window in one step)
  └ docker/             Docker image build files (one-step build script install.sh, Dockerfile, and launch script run.sh)
```

Module layout:

```
revohuman/              SDK (core; compiled modules in the wheel)
  ├ kit.py              open_kit / RevoHumanKit
  ├ product.py          Product model identification
  ├ manager.py          GloveManager: scanning, handedness resolution, connection
  ├ identity.py         Glove unit scanning and handedness determination (firmware-recorded handedness / serial number)
  ├ device_regs.py      Decoding of firmware identity info (handedness, product type, SN, structural part version) and tactile diagnostic counters
  ├ tactile_health.py   Tactile usability criteria, per-frame health tally, diagnostic counter windows
  ├ hand_fk.py          Hand forward kinematics and tracker→palm_link conversion
  ├ hand_keypoints.py   Nominal positions of human hand keypoints and finger-base points
  ├ glove.py            Single glove
  ├ joint_filter.py     Consumer-side joint angle smoothing (off by default, subscribe(joint_filter=...))
  ├ pairing.py          Bimanual pairing
  ├ frames.py           Frame types
  ├ recording.py        MCAP recording and playback
  ├ doctor.py           Health check
  ├ tracker/            Vive Tracker: capture, time mapping, pairing, world-frame detection
  │  └ calibration/     Base station allowlist, channel preflight check, geometry calibration, startup checks, zero pose, runtime session
  ├ calibration/        Wrist camera intrinsics and extrinsics (5.7; data/ contains only a layout description, per-unit data is supplied by you)
  └ camera/             Wrist camera
ros2/
  ├ revohuman_msgs/     Message and service definitions
  ├ revohuman_ros/      glove_node / bimanual_node
  └ revohuman_bringup/  Launch files and parameters
docs/                   User manual (00–09) and reference manual (English; Chinese copies in docs/zh-CN/)
examples/               Examples
simulation/             Glove visualization: subscribes to the local revohuman-live broadcast (see simulation/README.md)
```

Examples (`examples/`): `kit_quickstart.py` (full kit), `read_frames.py` / `read_frames_async.py` (single hand),
`pair_and_record.py` (bimanual recording), `button_record.py` (button-controlled recording), `tracker_bimanual.py`, `wrist_camera_record.py`, `wrist_camera_async.py`.

---

## 10. Usage notes

- **Handedness and product model are never guessed**: when they cannot be identified, the SDK refuses to start and explains why. For handedness, follow the message and specify the serial port; for the product model, follow the message and declare it.
- **A glove, or a libsurvive instance, can be opened by only one process at a time.**
- **After base stations are moved, replaced, added or removed**, update the allowlist, rerun `revohuman-lighthouses scan`, and then run `tracker calibrate`.
  Translation or horizontal rotation of a base station cannot be detected at runtime (4.4); if one has been moved, recalibrate instead of waiting for an error.
- **Every calibration rebuilds the base station frame** (4.7); absolute Tracker poses recorded under different calibrations cannot be compared directly.
- **After changing the venue or the lighting**, check the camera exposure again.
- **Host time sync**: when data from multiple hosts must be aligned, have them all sync to the same NTP server, configured to step only at boot and only slew while running
  (e.g. chrony's `makestep 1 3`). Stepping while running makes the glove briefly pause frame delivery (see 6.1).
- Serial numbers are written at the factory; the SDK only reads them and never writes them.
