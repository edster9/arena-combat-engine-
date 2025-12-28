#!/bin/bash
# Run Car Wars client
# OS-aware: handles WSL2, Linux, and MINGW64
# Usage: ./run.sh [--build] [args...]
# Output is always logged to ../last_run.log

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LOG_FILE="$PROJECT_DIR/last_run.log"

# Check for --build flag
if [[ "$1" == "--build" ]] || [[ "$1" == "-b" ]]; then
    shift
    "$SCRIPT_DIR/build.sh" || exit 1
fi

BUILD_DIR="$SCRIPT_DIR/build"

# Run and log output (both stdout and stderr)
run_with_log() {
    echo "=== Run started: $(date) ===" > "$LOG_FILE"
    "$@" 2>&1 | tee -a "$LOG_FILE"
    echo "" >> "$LOG_FILE"
    echo "=== Run ended: $(date) ===" >> "$LOG_FILE"
}

case "$(uname -s)" in
    MINGW*|MSYS*)
        # Windows/MINGW64 - run from build directory
        cd "$BUILD_DIR" || exit 1
        run_with_log ./carwars.exe "$@"
        ;;
    Linux)
        cd "$BUILD_DIR"
        if grep -qi microsoft /proc/version 2>/dev/null; then
            # WSL2 - use D3D12 GPU acceleration
            run_with_log env GALLIUM_DRIVER=d3d12 ./carwars "$@"
        else
            # Native Linux
            run_with_log ./carwars "$@"
        fi
        ;;
    *)
        # Fallback
        cd "$BUILD_DIR"
        run_with_log ./carwars "$@"
        ;;
esac
