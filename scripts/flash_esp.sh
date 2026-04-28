#!/usr/bin/env bash
# Flash the Argos ESP32 firmware from the Pi using arduino-cli.
#
# Usage:
#   scripts/flash_esp.sh                        # ESP32-C6 (current default)
#   scripts/flash_esp.sh --board classic        # classic ESP32 / HUZZAH32 Feather
#   scripts/flash_esp.sh --port /dev/ttyACM0    # override auto-detect
#   scripts/flash_esp.sh --setup                # one-time install of cli + cores
#
# Pre-reqs (handled by --setup):
#   - arduino-cli installed at /usr/local/bin/arduino-cli
#   - esp32 board core (Espressif) installed via board manager
#   - Adafruit_PWMServoDriver library installed
#   - User in `dialout` group (for /dev/tty* without sudo). If not:
#       sudo usermod -aG dialout $USER && newgrp dialout
#
# After a successful flash this stops the dashboard service (so the new
# firmware connects on a fresh serial), then restarts it.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

BOARD="c6"
PORT=""
DO_SETUP=0
SKIP_RESTART=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --board)        BOARD="$2"; shift 2;;
    --port)         PORT="$2"; shift 2;;
    --setup)        DO_SETUP=1; shift;;
    --no-restart)   SKIP_RESTART=1; shift;;
    -h|--help)
      grep '^#' "$0" | sed 's/^# \?//'
      exit 0;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

case "$BOARD" in
  c6)      SKETCH_DIR="$ROOT/firmware/argos_servo_c6";  FQBN="esp32:esp32:esp32c6";;
  classic) SKETCH_DIR="$ROOT/firmware/argos_servo";     FQBN="esp32:esp32:featheresp32";;
  *) echo "--board must be 'c6' or 'classic'" >&2; exit 2;;
esac

# ── Setup (one-time) ─────────────────────────────────────────────────────
if [[ "$DO_SETUP" -eq 1 ]]; then
  echo "[setup] installing arduino-cli + esp32 core + libraries"
  if ! command -v arduino-cli >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
      | BINDIR=/usr/local/bin sh
  fi
  arduino-cli config init --overwrite >/dev/null 2>&1 || true
  arduino-cli config set board_manager.additional_urls \
    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  arduino-cli core update-index
  arduino-cli core install esp32:esp32
  arduino-cli lib install "Adafruit PWM Servo Driver Library"
  echo "[setup] done. If this is your first flash, log out and back in"
  echo "        (or run: newgrp dialout) so the serial port is accessible."
  echo
fi

# ── Auto-detect port ─────────────────────────────────────────────────────
if [[ -z "$PORT" ]]; then
  for cand in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [[ -e "$cand" ]]; then PORT="$cand"; break; fi
  done
fi
if [[ -z "$PORT" ]]; then
  echo "[flash] no /dev/ttyACM* or /dev/ttyUSB* found; pass --port explicitly" >&2
  exit 1
fi

if [[ ! -d "$SKETCH_DIR" ]]; then
  echo "[flash] sketch directory missing: $SKETCH_DIR" >&2; exit 1
fi
if ! command -v arduino-cli >/dev/null 2>&1; then
  echo "[flash] arduino-cli not on PATH; rerun with --setup first" >&2; exit 1
fi

# ── Stop the dashboard so it lets go of the serial port ──────────────────
DASHBOARD_WAS_RUNNING=0
if systemctl is-active --quiet argos-dashboard 2>/dev/null; then
  DASHBOARD_WAS_RUNNING=1
  echo "[flash] stopping argos-dashboard.service to free $PORT"
  sudo systemctl stop argos-dashboard
fi

echo "[flash] board=$FQBN  port=$PORT  sketch=$SKETCH_DIR"

# ── Compile + upload ─────────────────────────────────────────────────────
arduino-cli compile --fqbn "$FQBN" "$SKETCH_DIR"
arduino-cli upload  --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo "[flash] done."

# ── Restart dashboard ────────────────────────────────────────────────────
if [[ "$SKIP_RESTART" -eq 0 ]] && [[ "$DASHBOARD_WAS_RUNNING" -eq 1 ]]; then
  echo "[flash] restarting argos-dashboard.service"
  sudo systemctl start argos-dashboard
fi
