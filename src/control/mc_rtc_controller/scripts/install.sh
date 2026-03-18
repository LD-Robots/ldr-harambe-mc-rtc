#!/usr/bin/env bash
set -euo pipefail

# Base = folder where script lives
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"

TARGET_HOME="${HOME}"
if [ "${SUDO_USER-}" != "" ] && [ "${HOME}" = "/root" ]; then
  TARGET_HOME="/home/${SUDO_USER}"
fi

link_file() {
  local src_rel="$1"
  local dst_rel="$2"
  local src="$BASE_DIR/$src_rel"
  local dst="$dst_rel"

  if [ ! -f "$src" ]; then
    echo "Missing source file: $src"
    exit 1
  fi

  mkdir -p "$(dirname "$dst")"

  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    local ts
    ts="$(date +%Y%m%d-%H%M%S)"
    cp "$dst" "${dst}.bak.${ts}"
    echo "Backup: ${dst}.bak.${ts}"
  fi

  ln -sf "$src" "$dst"
  echo "Linked $dst -> $src"
}

# Remove old G1 symlinks from ld-robots-humanoid-robot-demo
echo "Cleaning up old G1 configs..."
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1CoMPosture.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1LIPMStabilizer.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/robots/g1.json"
rm -rf "$TARGET_HOME/.config/mc_rtc/robots/rsdf/G1"
rm -f "/usr/local/share/mc_mujoco/G1.yaml" 2>/dev/null || true

echo "Installing Harambe configs..."
link_file "../config/mc_rtc.yaml" "$TARGET_HOME/.config/mc_rtc/mc_rtc.yaml"
link_file "../config/mc_mujoco.yaml" "$TARGET_HOME/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml"
link_file "../config/HarambeCoMPosture.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/HarambeCoMPosture.yaml"
link_file "../config/BaselineWalkingController.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/BaselineWalkingController.yaml"
link_file "../config/Harambe.yaml" "/usr/local/share/mc_mujoco/Harambe.yaml"
link_file "../config/harambe.json" "$TARGET_HOME/.config/mc_rtc/robots/harambe.json"
link_file "../config/harambe.rsdf" "$TARGET_HOME/.config/mc_rtc/robots/rsdf/Harambe/harambe.rsdf"

echo ""
echo "All configs linked. Build the controller with:"
echo "  cd $(dirname "$BASE_DIR") && cmake -S . -B build && cmake --build build"
