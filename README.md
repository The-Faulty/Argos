# Argos

Argos is a low-cost search-and-rescue quadruped built for Georgia Tech Senior
Design (ECE 4014, Spring 2026). The robot is tethered — there is no battery
or wireless control loop. Power and ethernet run to the chassis from the
operator station.

The stack is intentionally small:

- **ESP32 + Arduino sketch** drives 12 servos through a PCA9685 over I²C and
  reports a gas reading on its ADC.
- **Raspberry Pi** runs a Node.js dashboard server and a Python sensor
  sidecar. The Node server speaks JSON-over-serial @921600 to the ESP32, runs
  the gait planner and IK in JavaScript, persists user settings, records
  telemetry, and serves the React dashboard.
- **React dashboard** renders 14 panels for joint/leg/foot control, IMU,
  gas, thermal, RealSense camera, stance presets, animations, and recording.
  It opens one WebSocket to the Pi server.
- **Python sidecar** reads the Intel RealSense (color stream + IMU) and the
  MLX90640 thermal grid, exposes them over MJPEG + WebSocket on `:8788`.

```
Browser ──HTTP/WS──▶ Pi Node server (8787) ──serial JSON──▶ ESP32 + PCA9685
                          │
                          └──HTTP+WS──▶ Pi Python sidecar (8788) ──▶ RealSense + MLX90640
```

There is no ROS, no SLAM, no autonomous navigation, no LiDAR.

---

## Hardware

| Component         | Notes                                                   |
| ----------------- | ------------------------------------------------------- |
| Raspberry Pi 4/5  | Ubuntu 22.04 or Raspberry Pi OS 64-bit                  |
| ESP32 (any variant) | Flashed with Arduino IDE; USB serial to the Pi        |
| PCA9685            | I²C servo driver, 12 channels used                     |
| 12× hobby servos   | Three per leg (hip / thigh / calf)                     |
| Intel RealSense D435i or D455 | D435 (no `i`) has no IMU — see notes        |
| MLX90640           | I²C thermal grid (24 × 32)                              |
| MQ-x gas sensor    | Wired to one ESP32 ADC pin                             |

---

## Repo layout

```
Argos/
├── dashboard/              # React UI + Node Pi server
│   ├── pi_server/          # serial bridge, gait planner, mode controller
│   ├── shared/             # robot config, kinematics, protocol, servo cal
│   ├── src/                # React components and hooks
│   └── tests/              # node --test unit tests
├── pi_sensors/             # Python sidecar (RealSense + MLX90640)
└── firmware/argos_servo/   # Arduino sketch for the ESP32
```

---

## Setup — Pi

Install once:

```bash
# Node 20 + npm
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs

# Python sidecar deps
sudo apt install -y python3-venv python3-pip libusb-1.0-0
cd pi_sensors
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# Dashboard build
cd ../dashboard
npm install
npm run build

# Serial port permissions (one-time)
sudo usermod -aG dialout $USER
# log out and back in
```

A udev rule keeps the ESP32 at a stable name:

```
# /etc/udev/rules.d/99-argos.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", SYMLINK+="ttyESP32"
```

Reload with `sudo udevadm control --reload && sudo udevadm trigger`.

---

## Setup — ESP32

1. Open `firmware/argos_servo/argos_servo.ino` in Arduino IDE 2.x.
2. Install board support for ESP32 and the Adafruit PCA9685 library.
3. Select the correct board, set baud to 921600.
4. Flash. The serial monitor should print `{"type":"hello",...}` on boot.

---

## Run

Two processes on the Pi. From the repo root:

```bash
./scripts/start_pi.sh
```

That launches:

- `pi_sensors/.venv/bin/python sensor_server.py` (port 8788)
- `node dashboard/pi_server/server.js` (port 8787)

Open `http://<pi-ip>:8787/` in a browser. The header pill shows three
indicators: **pi**, **esp**, **sensors**.

For autostart on boot, install the systemd units:

```bash
sudo cp scripts/argos-dashboard.service /etc/systemd/system/
sudo cp pi_sensors/argos-sensors.service /etc/systemd/system/
sudo systemctl enable --now argos-dashboard argos-sensors
```

---

## Tests

```bash
cd dashboard
npm test
```

---

## Troubleshooting

- **Header shows `esp ✕`.** The Pi can't reach the ESP32 over serial. Check
  `ls -l /dev/ttyUSB0` (or `/dev/ttyESP32`), `dmesg | tail` after plugging
  the cable, and confirm your user is in `dialout`. Override the device or
  baud with `ARGOS_SERIAL_PORT` / `ARGOS_SERIAL_BAUD`.
- **Header shows `sensors ✕`.** The Python sidecar isn't responding. Run
  `systemctl status argos-sensors` (or check the foreground console). Most
  failures are a missing RealSense (`pyrealsense2` import), or the MLX90640
  not on I²C — both cases will show in `/health` at `:8788`.
- **No camera image but `sensors ✓`.** D435 (no `i`) has no IMU; the camera
  still works. If the IMU pill is missing too, you have the wrong model.
- **Joints jitter / move the wrong way.** Use the dashboard's settings drawer
  to adjust per-servo offsets, then the joint-limit fields. They persist to
  `~/.argos/`.
- **`gas` value pinned at 0.** Confirm the ADC pin in `argos_servo.ino`
  matches your wiring (default `GAS_ADC_PIN = 1`).

---

## Notes

- Argos is **tethered**. Do not add battery telemetry or wireless-link UI.
- The dashboard's `useRosbridge` hook is a historical name; the WebSocket
  target is the same-origin Node server, not rosbridge.
- All persistence lives under `~/.argos/` (servo overrides, joint limits,
  rotate settings, stance presets, telemetry recordings).
