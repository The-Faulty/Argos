#!/usr/bin/env bash
# Boot the two Argos services in the foreground. Ctrl-C kills both.
#
# For autostart on boot use the systemd units instead:
#   scripts/argos-dashboard.service
#   pi_sensors/argos-sensors.service
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

cleanup() {
  echo "[start_pi] shutting down"
  kill 0 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Python sensor sidecar (RealSense + MLX90640) on :8788.
(
  cd "$ROOT/pi_sensors"
  if [ -d ".venv" ]; then
    exec .venv/bin/python sensor_server.py
  else
    exec python3 sensor_server.py
  fi
) &

# Node dashboard server on :8787.
(
  cd "$ROOT/dashboard"
  exec node pi_server/server.js
) &

wait
