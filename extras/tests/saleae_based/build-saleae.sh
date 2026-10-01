#!/bin/sh
# build-saleae.sh — Build script for Saleae test firmware.
#
# Usage: ./build-saleae.sh [targets]
# Targets: arduino (default), espidf, all
#
# Integrates with existing build-pio-dirs.sh and build-idf-platformio.sh.
# Copies files into build directories (no symbolic links).

set -e

ROOT=$(git rev-parse --show-toplevel 2>/dev/null || echo ".")
TARGETS=${1:-arduino}

# Create build directory (no symlinks — copy files)
rm -fR saleae_build
mkdir -p saleae_build

if [ "$TARGETS" = "arduino" ] || [ "$TARGETS" = "all" ]; then
    echo "Building Arduino PlatformIO targets..."
    # Arduino PlatformIO: copy firmware into pio_dirs/saleae/
    mkdir -p saleae_build/pio_dirs/saleae/src
    cp firmware/platformio.ini saleae_build/pio_dirs/saleae/ 2>/dev/null || true
    cp firmware/src/*.cpp saleae_build/pio_dirs/saleae/src/ 2>/dev/null || true
    cp firmware/adapters/*.cpp saleae_build/pio_dirs/saleae/src/ 2>/dev/null || true
    cp -r firmware/lib/saleae_test_cases saleae_build/pio_dirs/saleae/lib/ 2>/dev/null || true
    # Copy library source (no symlinks)
    if [ -d "$ROOT/src" ]; then
        cp -r "$ROOT/src" saleae_build/pio_dirs/saleae/FastAccelStepper 2>/dev/null || true
    fi
    echo "Arduino build directories created."
    ls -al saleae_build/pio_dirs/ 2>/dev/null || true
fi

if [ "$TARGETS" = "espidf" ] || [ "$TARGETS" = "all" ]; then
    echo "Building ESP-IDF PlatformIO targets..."
    # ESP-IDF PlatformIO: copy firmware into pio_espidf/saleae/
    mkdir -p saleae_build/pio_espidf/saleae/src
    cp firmware/platformio_idf.ini saleae_build/pio_espidf/saleae/ 2>/dev/null || true
    cp firmware/CMakeLists.txt saleae_build/pio_espidf/saleae/ 2>/dev/null || true
    cp firmware/src/*.cpp saleae_build/pio_espidf/saleae/src/ 2>/dev/null || true
    cp firmware/adapters/*.cpp saleae_build/pio_espidf/saleae/src/ 2>/dev/null || true
    cp -r firmware/lib/saleae_test_cases saleae_build/pio_espidf/saleae/lib/ 2>/dev/null || true
    # Copy library source (no symlinks)
    if [ -d "$ROOT/src" ]; then
        cp -r "$ROOT/src" saleae_build/pio_espidf/saleae/FastAccelStepper 2>/dev/null || true
        cp "$ROOT/CMakeLists.txt" saleae_build/pio_espidf/saleae/FastAccelStepper 2>/dev/null || true
    fi
    echo "ESP-IDF build directories created."
    ls -al saleae_build/pio_espidf/ 2>/dev/null || true
fi

echo ""
echo "Build directories created in saleae_build/"
ls -al saleae_build/ 2>/dev/null || true