# 02 Hardware Connection

English | [简体中文](zh-CN/02-硬件连接.md)

This chapter connects the two P1 gloves to the host and confirms that the host recognizes them. Base station placement, pairing and calibration are covered in *03 Base Stations and Pairing*.

## 1. Components

A P1 set consists of the following parts:

| Part | Quantity | Description |
|---|---|---|
| Glove unit | 2 (one left, one right) | Each unit contains the encoder glove, a Vive Tracker mounted on the back of the hand, and a USB HUB inside the unit; **one USB cable** connects it to the host |
| Base station | Depends on the site | SteamVR base stations; the Tracker relies on them for positioning. For placement and pairing see *03 Base Stations and Pairing* |
| Host | 1 | Meets section 1 of *01 Installation and System Requirements*, with all software installed |

Each glove unit appears on the host as two USB devices, both under the unit's own HUB:

| Device | USB ID | How it appears on the host |
|---|---|---|
| Glove | `0483:5710` | Serial port `/dev/ttyACM*` |
| Tracker | `28de:2300` | USB device, read by libsurvive |

The SDK can tell which Tracker belongs to which glove: the two are under the same HUB.

Each Tracker has a power button and a status light. Power it on before use; see section 2.1.

## 2. Wiring

1. Plug the USB cables of the two glove units into USB ports on the host. **Plug them directly into the host**; do not go through a passive HUB or a USB extension cable.
2. Either hand can go into any port. The SDK tells the left and right hands apart by the handedness stored in the glove firmware; for earlier gloves whose firmware does not store it, it uses the handedness character in the USB serial number
   (`BCRH1PL…` / `BCRH1TL…` is a left hand, `BCRH1PR…` / `BCRH1TR…` is a right hand). It does not depend on which port is used or which glove is plugged in first.
3. After plugging in, wait a few seconds for the system to finish enumerating the devices.
4. Briefly press the power button on each Tracker to power it on, and confirm that the light is green (see section 2.1).

Notes:

- Do not plug or unplug the gloves while they are in use (during recording, live broadcast, or while ROS nodes are running). To change ports, first close all programs that are using the gloves.
- The serial port numbers `/dev/ttyACM0` and `/dev/ttyACM1` are determined by plug-in order and may swap after replugging. The SDK and the ROS nodes both find the gloves by handedness;
  **do not hard-code serial port numbers in your own programs**.
- Connecting the two hands to different USB controllers on the host also works. The health check shows a notice (see section 3.2), and the time alignment between the two hands is slightly less precise than on the same controller.

### 2.1 Tracker power button and status light

P1 uses the VIVE Tracker (3.0). Each Tracker has **one power button** (the only button on the Tracker) and **one status light**.
This power button is not the same as the button on the glove: the glove button controls recording, see section 8 of *04 Recording*.

The Trackers in this kit pass data directly to the host over the glove unit's USB cable. They **do not use a wireless receiver and do not need to be paired in SteamVR**.
While connected over USB, a Tracker can be used while it charges.

Power button operations:

| Action | Effect |
|---|---|
| Press briefly once | Power on |
| Hold for about 5 seconds | Power off |
| Hold for about 2 seconds | Enters wireless pairing mode; the light blinks blue. **Not used with this kit**; if entered by accident, handle it as in the table below |

Status light colors:

| Light | Meaning | What to do |
|---|---|---|
| Green | Normal mode: powered on and working normally | You can continue |
| Solid blue | Connecting | Wait a few seconds and continue once it turns green; if it never turns green, hold the power button for about 5 seconds to power off, then press briefly to power on |
| Blinking blue | Wireless pairing mode (entered by holding the power button for about 2 seconds) | This kit does not need wireless pairing: hold the power button for about 5 seconds to power off, then press briefly to power on |
| Blinking red | Low battery | Keep the USB cable connected to charge it; if it has already powered off automatically, charge it for a while, then press briefly to power on |
| Off | Powered off | Press the power button briefly to power on |

There are two more light colors related to charging: orange while charging; when fully charged, white if powered off and green if powered on.

According to the official documentation, the Tracker powers off automatically when the battery is depleted, when it has been idle in wireless pairing mode for more than 30 seconds, or when it has not moved for more than 5 minutes.
The light then goes out; press the power button briefly to power it on again.

The light reflects only the Tracker's own state. Whether the Tracker is recognized by the host: see section 3.3. Whether it can see the base stations and whether its pose is usable:
go by the result of `tracker check --live` in section 6 of *03 Base Stations and Pairing*.

The power button and status light descriptions above are based on HTC's official documentation:
[Status light](https://www.vive.com/us/support/tracker3/category_howto/checking-the-status-light.html),
[Turning on or off](https://www.vive.com/us/support/tracker3/category_howto/turning-on-or-off.html),
[Pairing](https://www.vive.com/us/support/tracker3/category_howto/pairing-vive-tracker.html).

## 3. Checking the connection

### 3.1 Serial ports

```bash
ls -l /dev/ttyACM*
```

Sample output:

```text
crw-rw---- 1 root dialout 166, 0 Sep 27 23:51 /dev/ttyACM0
crw-rw---- 1 root dialout 166, 1 Sep 27 23:51 /dev/ttyACM1
```

You should see two serial ports with group `dialout`. If there are none, check the USB cables; if there is only one, the other glove is not connected properly.

### 3.2 Health check

```bash
python3 -m revohuman doctor --probe
```

Sample output (two gloves connected to two different USB controllers on the same host):

```text
⚠️  RevoHuman 手套体检 — 6 项检查：fail=0 warn=1
  ✅ 在位手套单元 — 2 个单元
    ✅ 1-1 — /dev/ttyACM0，无同 HUB 相机
    ✅ 3-1 — /dev/ttyACM1，无同 HUB 相机
  ✅ 手别认定 — 2/2 单元已认定手别
    ✅ 1-1 — left（按 MCU 固件登记的手别）
    ✅ 3-1 — right（按 MCU 固件登记的手别）
  ⚠️  USB 拓扑与电源管理 — 2 个单元完成拓扑解析
    ✅ 1-1 — 12Mbps @ 控制器 0000:05:00.3，autosuspend 已关
    ✅ 3-1 — 12Mbps @ 控制器 0000:05:00.4，autosuspend 已关
    ⚠️  双手拓扑分档 — cross-controller：……
  ✅ 固件版本与能力 — 2 个单元
    ✅ 1-1 — fw 0.20.3，ENCODER_ONLY，编码器 21，不带触觉型号（A 态）
    ✅ 3-1 — fw 0.20.3，ENCODER_ONLY，编码器 21，不带触觉型号（A 态）
  ✅ 流会话残留 — 2 个单元
    ✅ 1-1 — 无残留会话
    ✅ 3-1 — 无残留会话
  ⏭️  时间戳映射质量 — 需要活动数据流……
```

The unit names `1-1` and `3-1` are USB port locations and differ depending on which port is used. The only `warn` above is the `cross-controller` notice; the gloves can be used normally.

`--probe` briefly opens each glove, reads the handedness stored in the firmware and the firmware information, and checks whether a data stream left by a previous program is still running. **Do not add `--probe` while recording or another program is using the gloves**;
without it, only the checks that do not open the devices are run. Handedness can then come only from the serial number or from results already read since this boot, and newer gloves are shown as `未认定手别` (handedness not determined).

What each item means:

| Check | When it passes | When it does not pass |
|---|---|---|
| `在位手套单元` (glove units present) | Lists 2 units and their serial ports | Fewer units than expected, or `未发现手套单元` (no glove units found): the gloves are not connected properly, see section 6 |
| `手别认定` (handedness) | `2/2 单元已认定手别` (2/2 units have handedness determined); each unit is labeled `left` / `right` with the basis (`按 MCU 固件登记的手别` (handedness stored in the MCU firmware) or `按 MCU 序列号 …` (from the MCU serial number)) | `未认定手别` (handedness not determined) with the hint `加 --probe 读` (add --probe to read): rerun with `--probe`; if both gloves claim the same hand (neither is trusted) or it still cannot be determined: specify the gloves by serial port instead (reference manual section 2.3); `手别冲突` (handedness conflict; fail, the firmware-stored handedness and the serial number disagree): contact technical support |
| `USB 拓扑与电源管理` (USB topology and power management) | Each unit shows `autosuspend 已关` (autosuspend off) | `cross-controller` is only a notice (see section 2); the gloves can be used normally |
| `固件版本与能力` (firmware version and capabilities) | Reads the firmware version and product model of both gloves | `读取失败` (read failed): the glove is being used by another program; or the next item reports a leftover session (see the next row) |
| `流会话残留` (leftover stream session) | `无残留会话` (no leftover session) | Reports a running stream session: the previous program that used the glove did not exit normally, and the glove is still sending data. The SDK stops it automatically the next time the glove is opened; no action needed |
| `时间戳映射质量` (timestamp mapping quality) | Requires a data stream; not done in the health check | — |

How to judge: the first line shows `fail=0`, **and** `在位手套单元` lists 2 units, and `手别认定` shows `2/2`.
Checking only `fail=0` is not enough — with no glove connected at all, the health check also shows `fail=0` (`未发现手套单元` counts only as a `warn`).

### 3.3 Product model and left/right hands

```bash
python3 -m revohuman product identify
```

Sample output:

```text
型号 P1（依据：sn）  传感器：encoder, tracker
  left  手套 BCRH1PLXXXXXXXXXXX @ /dev/ttyACM0  tracker LHR-XXXXXXXX  触觉 none  相机 无
  right 手套 BCRH1PRXXXXXXXXXXX @ /dev/ttyACM1  tracker LHR-XXXXXXXX  触觉 none  相机 无
```

You should see `型号 P1` (product model P1), sensors `encoder, tracker`, and both the `left` and the `right` line should have a glove serial number and a Tracker serial number (starting with `LHR-`).
If a line has no Tracker serial number, the Tracker of that glove unit was not recognized: first confirm that this Tracker is powered on and its light is green (section 2.1),
then replug the unit; if it still does not work, check that the Tracker device rules from section 6 of
*01 Installation and System Requirements* are installed.

## 4. Wearing

- The Tracker is mounted on the back of the hand, and faces outward when worn. **During use, do not cover the Tracker with the other hand or an object**; otherwise it cannot receive light from the base stations and the pose is interrupted.
- Secure the USB cable along the upper limb and leave enough slack, so that raising your hands or turning around does not pull on the connector.
- The zero pose of the glove's joint angles is calibrated before shipment; put the glove on and it is ready to use. Recalibrate per section 5 only after replacing encoder magnets, after the glove has been repaired, or when the zero pose is clearly offset.
- The Tracker zero pose (the pose of both hands held in a fixed posture) is unrelated to the glove joint zero pose; record it as needed during recording or in ROS, see *04 Recording* and *06 ROS 2*.

## 5. Glove joint zero-pose calibration

The zero posture of the glove joints is **five fingers straight, palm flat**. With a correct zero pose, every joint reads around 180° when the glove is in this posture.
Recalibrate only in the following cases:

- Encoder magnets have been replaced, or the glove has been repaired;
- With five fingers straight and the hand flat, a few joints read clearly away from 180° while all other joints read around 180°.
  The readings can be viewed in the joint-angle table at the top left of the fixed-base view in *05 Simulation and Visualization* (columns `L deg` / `R deg`).

Calibration takes the glove's **current posture** as the new zero pose and writes it into the glove, replacing the previous zero pose (including the factory zero pose). This **cannot be undone**; if calibrated wrongly, the only fix is to hold the correct posture and calibrate again.
With a wrong zero pose the glove still outputs data as usual, and the error cannot be seen in the data, so always hold the correct posture before calibrating.

Calibrate one hand at a time, as follows:

1. Close all programs that are using the gloves (`revohuman-live`, recording scripts, ROS nodes, visualization). If the glove is in use, the command refuses outright.
2. Put on, or lay flat, the glove to calibrate: **five fingers straight, palm flat**, and hold still.
3. Run the following command (left hand shown; for the right hand, replace `left` with `right`):

```bash
python3 -m revohuman glove calibrate --hand left
```

4. The command first prints the glove to be calibrated (hand, USB location, serial port, serial number). After checking it, type `yes` and press Enter to start calibration. Do not move the glove until calibration is complete.
5. `标定完成：…已保存到手套` (calibration complete: … saved to the glove) with exit code 0 means success.

Exit codes:

| Exit code | Meaning | Glove zero pose |
|---|---|---|
| 0 | Calibration succeeded; the new zero pose has been saved to the glove | Replaced with the new zero pose |
| 1 | Calibration failed: only some encoders succeeded (the succeeded and failed indices are listed), the device did not report completion within `--timeout` (default 15 seconds), or it was interrupted with `Ctrl+C` | May be partially replaced: hold the correct posture and calibrate again |
| 1 | Cannot connect to the glove (no serial port permission, or it was just unplugged), or the glove currently has no online encoders | Unchanged |
| 2 | Invalid argument (e.g. `--index` not within 0..20) | Unchanged |
| 3 | Refused: this glove cannot be found, the glove is in use by another program, or not confirmed | Unchanged |

Other usage:

- When running in a non-interactive environment such as a script, you must add `--yes` to skip the confirmation; otherwise the command refuses outright (exit code 3) and does not modify the glove.
- You can also use `--port /dev/ttyACM0` instead of `--hand` to specify the glove by serial port (it must be the serial port of a glove that is present). Serial port numbers may swap after replugging; before proceeding, check the hand and serial number printed by the command.
- All options:

```bash
python3 -m revohuman glove calibrate --help
```

## 6. Common issues

| Symptom | Action |
|---|---|
| `ls /dev/ttyACM*` reports `No such file or directory` | The glove is not connected: check that both ends of the USB cable are firmly plugged in, and try a different USB port on the host |
| A program reports `Permission denied` for the serial port | The current user is not in the `dialout` group, or has not logged in again after being added: see section 5 of *01 Installation and System Requirements* |
| The health check lists only 1 unit | The other glove is not connected properly, or is connected through a passive HUB: plug it directly into the host |
| A hand has no Tracker serial number in `product identify` | Confirm that the Tracker is powered on (section 2.1); replug that glove unit; check that the Tracker device rules (chapter 01, section 6) are installed and that the unit was replugged afterwards |
| A program reports error code `1102` (glove in use) | `revohuman-live`, a recording script, a ROS node or the visualization is using the glove; close it first; see *07 Logging and Error Codes* |
| With five fingers straight and the hand flat, a few joints read clearly away from 180° | The zero pose is offset; recalibrate this glove per section 5 |
| The Tracker light is off, or goes out during use | The Tracker has powered off (it powers off automatically when the battery is depleted, when idle in pairing mode, or when it has not moved for a long time): press the power button briefly to power it on, see section 2.1 |
| The Tracker light blinks blue | The power button was held too long and the Tracker entered wireless pairing mode. This kit does not need wireless pairing: hold for about 5 seconds to power off, then press briefly to power on |
| The Tracker light blinks red | Low battery: keep the USB cable connected to charge it; it can be used while charging |
