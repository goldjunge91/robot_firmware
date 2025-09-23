#!/usr/bin/env bash
# Lightweight build helper for lpico_pio using PlatformIO
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Preferred venv to use (set by user request)
PREFERRED_VENV="/home/ros/.platformio/penv"
if [ -f "$PREFERRED_VENV/bin/activate" ]; then
  # shellcheck source=/dev/null
  source "$PREFERRED_VENV/bin/activate"
  echo "Sourced venv: $PREFERRED_VENV"
fi

if ! command -v pio >/dev/null 2>&1; then
  echo "PlatformIO CLI (pio) not found in PATH or venv."
  echo "Install it in the venv with: python3 -m pip install --user platformio" 
  exit 2
fi

if [ "${1-}" = "--check" ]; then
  echo "PlatformIO version: $(pio --version)"
  echo "Project dir: $PROJECT_DIR"
  exit 0
fi

echo "Running PlatformIO build in $PROJECT_DIR"
pio run "$@"

echo "Build finished. Build artifacts are in .pio/build"
