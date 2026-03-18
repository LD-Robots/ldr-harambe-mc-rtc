#!/usr/bin/env bash
set -euo pipefail

# Base = folder where script lives
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$(cd "$BASE_DIR/../config" && pwd)"
URDF_DIR="$(cd "$BASE_DIR/../urdf" && pwd)"
REPO_ROOT="$(cd "$BASE_DIR/../../../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build/mc_rtc_controller"

TARGET_HOME="${HOME}"
if [ "${SUDO_USER-}" != "" ] && [ "${HOME}" = "/root" ]; then
  TARGET_HOME="/home/${SUDO_USER}"
fi
MC_RTC_DIR="$TARGET_HOME/.config/mc_rtc"

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

# Copy a file, replacing @CONFIG_DIR@ and @URDF_DIR@ placeholders
install_file() {
  local src_rel="$1"
  local dst="$2"
  local src="$BASE_DIR/$src_rel"

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

  # Remove existing symlink if present
  rm -f "$dst"

  sed -e "s|@CONFIG_DIR@|${CONFIG_DIR}|g" \
      -e "s|@URDF_DIR@|${URDF_DIR}|g" \
      -e "s|@MC_RTC_DIR@|${MC_RTC_DIR}|g" \
      -e "s|@BUILD_DIR@|${BUILD_DIR}|g" \
      "$src" > "$dst"
  echo "Installed $dst (from $src)"
}

# Remove old G1 symlinks from ld-robots-humanoid-robot-demo
echo "Cleaning up old G1 configs..."
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1CoMPosture.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1LIPMStabilizer.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/robots/g1.json"
rm -rf "$TARGET_HOME/.config/mc_rtc/robots/rsdf/G1"
rm -f "/usr/local/share/mc_mujoco/G1.yaml" 2>/dev/null || true

echo "Installing Harambe configs..."
# Files with placeholders — copy with sed substitution
install_file "../config/mc_rtc.yaml" "$TARGET_HOME/.config/mc_rtc/mc_rtc.yaml"
install_file "../config/harambe.json" "$TARGET_HOME/.config/mc_rtc/robots/harambe.json"
install_file "../config/Harambe.yaml" "/usr/local/share/mc_mujoco/Harambe.yaml"

# Files without placeholders — symlink
link_file "../config/mc_mujoco.yaml" "$TARGET_HOME/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml"
link_file "../config/HarambeCoMPosture.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/HarambeCoMPosture.yaml"
link_file "../config/BaselineWalkingController.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/BaselineWalkingController.yaml"
link_file "../config/harambe.rsdf" "$TARGET_HOME/.config/mc_rtc/robots/rsdf/Harambe/harambe.rsdf"

echo ""
echo "All configs installed. Build the controller with:"
echo "  cd $(dirname "$BASE_DIR") && cmake -S . -B build && cmake --build build"
