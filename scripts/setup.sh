#!/usr/bin/env bash
# Install the build dependencies for drone_sim on Linux or macOS.
#
# This installs system packages ONLY. Configuring, building, and testing the
# project (cmake --preset linux && cmake --build --preset linux && ctest
# --preset linux) are documented in the README -- this script deliberately
# stops at the deps so it stays a thin, predictable wrapper around the
# package manager.
#
# Safe to re-run: apt and brew both skip anything already installed.
set -euo pipefail

if command -v apt-get >/dev/null 2>&1; then
    echo "Detected apt (Debian/Ubuntu) -- installing dependencies..."
    sudo apt-get update
    sudo apt-get install -y cmake g++ libglfw3-dev libopencv-dev
elif command -v brew >/dev/null 2>&1; then
    echo "Detected Homebrew (macOS) -- installing dependencies..."
    brew install cmake glfw opencv
else
    echo "error: no supported package manager found (need apt-get or brew)." >&2
    echo "Install cmake, a C++ compiler, GLFW, and OpenCV by hand, then build per the README." >&2
    exit 1
fi

echo "Done. Dependencies are installed -- now build the project per the README."
