#!/bin/bash
# Run Car Wars client
# OS-aware: handles WSL2, Linux, and MINGW64
# Usage: ./run.sh [--build] [args...]

SCRIPT_DIR="$(dirname "$0")"

# Check for --build flag
if [[ "$1" == "--build" ]] || [[ "$1" == "-b" ]]; then
    shift
    "$SCRIPT_DIR/build.sh" || exit 1
fi

cd "$SCRIPT_DIR/build"

case "$(uname -s)" in
    MINGW*|MSYS*)
        # Windows/MINGW64 - native execution
        ./carwars.exe "$@"
        ;;
    Linux)
        if grep -qi microsoft /proc/version 2>/dev/null; then
            # WSL2 - use D3D12 GPU acceleration
            GALLIUM_DRIVER=d3d12 ./carwars "$@"
        else
            # Native Linux
            ./carwars "$@"
        fi
        ;;
    *)
        # Fallback
        ./carwars "$@"
        ;;
esac
