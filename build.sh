#!/bin/bash
# build.sh — UR5e RT Controller Build Script
#
# Usage:
#   ./build.sh                         # build (auto-detect workspace)
#   ./build.sh --mujoco /opt/mujoco-3.2.4  # build with MuJoCo support
#   ./build.sh --help                  # show help

set -e

# ── Colors ─────────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# ── Helper functions ───────────────────────────────────────────────────────────
info()    { echo -e "${BLUE}▶ $*${NC}"; }
success() { echo -e "${GREEN}✔ $*${NC}"; }
warn()    { echo -e "${YELLOW}⚠ $*${NC}"; }
error()   { echo -e "${RED}✘ $*${NC}"; exit 1; }

# ── Argument parsing ───────────────────────────────────────────────────────────
MJ_DIR=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mujoco)
      MJ_DIR="$2"
      shift 2
      ;;
    -h|--help|help)
      echo ""
      echo "Usage: $0 [OPTIONS]"
      echo ""
      echo "Options:"
      echo "  --mujoco <path>   Path to MuJoCo install dir (e.g. /opt/mujoco-3.2.4)"
      echo "                    Enables ur5e_mujoco_sim package in the build"
      echo "  --help            Show this help"
      echo ""
      echo "Examples:"
      echo "  ./build.sh"
      echo "  ./build.sh --mujoco /opt/mujoco-3.2.4"
      echo ""
      exit 0
      ;;
    *)
      error "Unknown option: '$1'  (run $0 --help for usage)"
      ;;
  esac
done

# ── Workspace detection ────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(dirname "$SCRIPT_DIR")"
DETECTED_WS="$(dirname "$SRC_DIR")"

if [[ "$(basename "$SRC_DIR")" == "src" && -d "$DETECTED_WS" ]]; then
  WORKSPACE="$DETECTED_WS"
  info "Workspace auto-detected: $WORKSPACE"
else
  WORKSPACE=~/ur_ws
  warn "Script is not under <workspace>/src/<repo>/ — using default: $WORKSPACE"
fi

if [[ ! -d "$WORKSPACE" ]]; then
  error "Workspace not found: $WORKSPACE"
fi

# ── Build ──────────────────────────────────────────────────────────────────────
info "Building all ur5e packages..."
cd "$WORKSPACE"

CMAKE_ARGS=()
if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
  CMAKE_ARGS+=("-Dmujoco_DIR=${MJ_DIR}/lib/cmake/mujoco")
  info "MuJoCo cmake path: ${MJ_DIR}/lib/cmake/mujoco"
fi

PACKAGES=(ur5e_rt_base ur5e_rt_controller ur5e_hand_udp ur5e_tools)
if [[ -n "$MJ_DIR" && -d "$MJ_DIR" ]]; then
  PACKAGES+=(ur5e_mujoco_sim)
fi

if [[ ${#CMAKE_ARGS[@]} -gt 0 ]]; then
  colcon build \
      --packages-select "${PACKAGES[@]}" \
      --symlink-install \
      --cmake-args "${CMAKE_ARGS[@]}"
else
  colcon build \
      --packages-select "${PACKAGES[@]}" \
      --symlink-install
fi

source install/setup.bash
grep -qF "$WORKSPACE/install/setup.bash" ~/.bashrc || \
    echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

success "All packages built and sourced"
