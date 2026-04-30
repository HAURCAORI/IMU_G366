# toolchain-rpi4.cmake
# Cross-compilation toolchain for Raspberry Pi 4B (64-bit, aarch64)
# Targets: Raspberry Pi OS 64-bit / Ubuntu 64-bit on RPi 4
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=../toolchain-rpi4.cmake
#   cmake --build build

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ── Compiler ──────────────────────────────────────────────────────────────────
# Install on Ubuntu/WSL:  sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# ── CPU flags for Cortex-A72 (RPi 4) ─────────────────────────────────────────
set(CMAKE_C_FLAGS_INIT   "-mcpu=cortex-a72 -march=armv8-a")
set(CMAKE_CXX_FLAGS_INIT "-mcpu=cortex-a72 -march=armv8-a")

# ── Sysroot (optional — set if you have an RPi rootfs for proper linking) ─────
# set(CMAKE_SYSROOT /opt/rpi-sysroot)

# ── Search path rules — don't mix host and target libraries ───────────────────
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
