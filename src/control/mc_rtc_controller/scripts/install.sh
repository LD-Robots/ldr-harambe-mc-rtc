#!/usr/bin/env bash
set -euo pipefail

# Base = folder where script lives
BASE_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="$(cd "$BASE_DIR/../config" && pwd)"
URDF_DIR="$(cd "$BASE_DIR/../urdf" && pwd)"
REPO_ROOT="$(cd "$BASE_DIR/../../../.." && pwd)"
BUILD_DIR="$REPO_ROOT/build/mc_rtc_controller"
GENERATED_DIR="$REPO_ROOT/build/generated"

TARGET_HOME="${HOME}"
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

# Copy a file, replacing placeholders with resolved paths
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
      -e "s|@GENERATED_DIR@|${GENERATED_DIR}|g" \
      "$src" > "$dst"
  echo "Installed $dst (from $src)"
}

# Resolve package:// URIs to real filesystem paths using ros2 pkg prefix
resolve_package_uris() {
  local src="$1"
  local dst="$2"
  local content
  content="$(cat "$src")"

  # Find all unique package:// references
  local packages
  packages="$(echo "$content" | grep -oP 'package://\K[^/]+' | sort -u)"

  for pkg in $packages; do
    local pkg_prefix
    pkg_prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null || true)"
    if [ -z "$pkg_prefix" ]; then
      echo "WARNING: Could not resolve package '$pkg'"
      continue
    fi
    local pkg_share="$pkg_prefix/share/$pkg"
    content="$(echo "$content" | sed "s|package://${pkg}/|${pkg_share}/|g")"
    echo "  Resolved package://$pkg/ -> $pkg_share/"
  done

  mkdir -p "$(dirname "$dst")"
  echo "$content" > "$dst"
  echo "Generated $dst (resolved package:// URIs)"
}

# Remove old G1 symlinks from ld-robots-humanoid-robot-demo
echo "Cleaning up old G1 configs..."
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1CoMPosture.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/controllers/G1LIPMStabilizer.yaml"
rm -f "$TARGET_HOME/.config/mc_rtc/robots/g1.json"
rm -rf "$TARGET_HOME/.config/mc_rtc/robots/rsdf/G1"
sudo rm -f "/usr/local/share/mc_mujoco/G1.yaml" 2>/dev/null || true

echo "Installing Harambe configs..."
# MuJoCo XML — symlink (mesh paths are already absolute, no resolve needed)
mkdir -p "$GENERATED_DIR"
rm -f "$GENERATED_DIR/harambe_mujoco.xml"
ln -sf "$URDF_DIR/harambe_mujoco.xml" "$GENERATED_DIR/harambe_mujoco.xml"
echo "Linked $GENERATED_DIR/harambe_mujoco.xml -> $URDF_DIR/harambe_mujoco.xml"

# Files with placeholders — copy with sed substitution
install_file "../config/mc_rtc.yaml" "$TARGET_HOME/.config/mc_rtc/mc_rtc.yaml"
install_file "../config/harambe.json" "$TARGET_HOME/.config/mc_rtc/robots/harambe.json"
# Harambe.yaml goes to /usr/local — needs sudo
TMP_HARAMBE="$(mktemp)"
sed -e "s|@CONFIG_DIR@|${CONFIG_DIR}|g" \
    -e "s|@URDF_DIR@|${URDF_DIR}|g" \
    -e "s|@MC_RTC_DIR@|${MC_RTC_DIR}|g" \
    -e "s|@BUILD_DIR@|${BUILD_DIR}|g" \
    -e "s|@GENERATED_DIR@|${GENERATED_DIR}|g" \
    "$BASE_DIR/../config/Harambe.yaml" > "$TMP_HARAMBE"
sudo mkdir -p /usr/local/share/mc_mujoco
sudo cp "$TMP_HARAMBE" /usr/local/share/mc_mujoco/Harambe.yaml
rm -f "$TMP_HARAMBE"
echo "Installed /usr/local/share/mc_mujoco/Harambe.yaml"

# Files without placeholders — symlink
link_file "../config/mc_mujoco.yaml" "$TARGET_HOME/.config/mc_rtc/mc_mujoco/mc_mujoco.yaml"
link_file "../config/HarambeCoMPosture.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/HarambeCoMPosture.yaml"
link_file "../config/BaselineWalkingController.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/BaselineWalkingController.yaml"
link_file "../config/HarambeWalking.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/HarambeWalking.yaml"
link_file "../config/HarambeOnnxWalking.yaml" "$TARGET_HOME/.config/mc_rtc/controllers/HarambeOnnxWalking.yaml"
link_file "../config/harambe.rsdf" "$TARGET_HOME/.config/mc_rtc/robots/rsdf/Harambe/harambe.rsdf"

echo ""
echo "All configs installed. Build the controller with:"
echo "  cd $(dirname "$BASE_DIR") && cmake -S . -B build && cmake --build build"
