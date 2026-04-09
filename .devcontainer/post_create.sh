#!/bin/bash
# Post-create script for the Argos dev container.

set -e

echo "========================================="
echo "  Argos Dev Container - Post-Create Setup"
echo "========================================="

if command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
else
  SUDO=""
fi

echo "[1/5] Preparing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  ${SUDO} rosdep init
fi
${SUDO} apt-get update -qq
rosdep update --rosdistro=humble

echo "[2/5] Installing workspace dependencies..."
if [ -d /workspace/ros2_ws/src ]; then
  rosdep install --from-paths /workspace/ros2_ws/src --ignore-src -r -y || true
fi

echo "[3/5] Building the workspace..."
if [ -d /workspace/ros2_ws/src ] && find /workspace/ros2_ws/src -name package.xml -print -quit | grep -q .; then
  cd /workspace/ros2_ws
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install || echo "WARNING: colcon build reported issues."
fi

echo "[4/5] Configuring the shell..."
BASHRC_MARKER="# --- Argos Dev Container ---"
if ! grep -q "$BASHRC_MARKER" ~/.bashrc 2>/dev/null; then
  cat >> ~/.bashrc << 'BASHRC_BLOCK'

# --- Argos Dev Container ---
source /opt/ros/humble/setup.bash
if [ -f /workspace/ros2_ws/install/setup.bash ]; then
  source /workspace/ros2_ws/install/setup.bash
fi
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
alias cb='cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash'
alias cs='source /workspace/ros2_ws/install/setup.bash'
alias rt='ros2 topic list'
alias rn='ros2 node list'
alias piconnect='ssh argos@argos-pi.local'
BASHRC_BLOCK
fi

echo "[5/5] Post-create complete."
echo
echo "  Build:   cd /workspace/ros2_ws && colcon build --symlink-install"
echo "  Source:  source /workspace/ros2_ws/install/setup.bash"
echo "  Alias:   cb"
echo
