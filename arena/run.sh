#!/bin/bash
# Run Arena with hardware GPU acceleration on WSL2
# GALLIUM_DRIVER=d3d12 forces Mesa to use the GPU via DirectX 12

cd "$(dirname "$0")/build"
GALLIUM_DRIVER=d3d12 ./arena "$@"
