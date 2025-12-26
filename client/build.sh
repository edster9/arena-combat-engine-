#!/bin/bash
# Build Car Wars client
# OS-aware: handles WSL2, Linux, and MINGW64

set -e

cd "$(dirname "$0")"

# Create build directory if needed
mkdir -p build
cd build

case "$(uname -s)" in
    MINGW*|MSYS*)
        # Windows/MINGW64
        echo "Building for Windows (MINGW64)..."
        cmake -G "MinGW Makefiles" ..
        mingw32-make -j$(nproc)
        echo "Build complete: carwars.exe"
        ;;
    Linux|*)
        # Linux / WSL2
        echo "Building for Linux..."
        cmake ..
        make -j$(nproc)
        echo "Build complete: carwars"
        ;;
esac
