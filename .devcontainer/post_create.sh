#!/bin/bash
# Post-create script for Argos Dev Container
# This runs once after the container is first built.

set -e

echo "========================================="
echo "  Argos Dev Container — Post-Create Setup"
echo "========================================="

# --- 1. Update rosdep ---
echo "[1/5] Updating rosdep..."
sudo apt-get update -qq
rosdep update --rosdistro=humble

# --- 2. Install workspace dependencies ---
echo "[2/5] Installing workspace dependencies via rosdep..."
if [ -d /workspace/ros2_ws/src ]; then
  rosdep install --from-paths /workspace/ros2_ws/src --ignore-src -r -y 2>/dev/null || true
fi

# --- 3. Build the workspace ---
echo "[3/5] Building workspace..."
if [ -d /workspace/ros2_ws/src ]; then
  cd /workspace/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install 2>&1 || echo "WARNING: colcon build had issues (may be fine on first run with no packages yet)"
fi

# --- 4. Add shell convenience to bashrc ---
echo "[4/5] Configuring shell..."
BASHRC_MARKER="# --- Argos Dev Container ---"
if ! grep -q "$BASHRC_MARKER" ~/.bashrc 2>/dev/null; then
  cat >> ~/.bashrc << 'BASHRC_BLOCK'

# --- Argos Dev Container ---
source /opt/ros/humble/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
export ROS_DOMAIN_ID=42
# Useful aliases
alias cb='cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash'
alias cs='source /workspace/ros2_ws/install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'
alias piconnect='ssh argos@argos-pi.local'
BASHRC_BLOCK
fi

# --- 5. Done ---
echo "[5/5] Post-create complete."
echo ""
echo "  To build:   cd /workspace/ros2_ws && colcon build --symlink-install"
echo "  To source:   source install/setup.bash"
echo "  Alias 'cb':  build + source in one command"
echo "  Alias 'piconnect': SSH to the Raspberry Pi"
echo ""
