#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASHBOARD_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTROLLER_ROOT="$SCRIPT_DIR"
PORT="${PORT:-8787}"

echo "Installing dashboard dependencies..."
cd "$DASHBOARD_ROOT"
if [ ! -d node_modules ]; then
  npm install
fi

echo "Building dashboard for Pi hosting..."
npm run build

echo "Installing Pi controller dependencies..."
cd "$CONTROLLER_ROOT"
if [ ! -d node_modules ]; then
  npm install
fi

echo "Starting Pi controller on port $PORT"
echo "Frontend will be served from: http://$(hostname -I 2>/dev/null | awk '{print $1}'):$PORT"
npm start
