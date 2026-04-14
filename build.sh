#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

# Parse -jN parameter
JOBS=$(nproc)
while [[ $# -gt 0 ]]; do
    case "$1" in
        -j*)
            JOBS="${1#-j}"
            shift
            ;;
        --clean)
            echo "Cleaning build directories..."
            rm -rf build/
            shift
            ;;
        --clean-all)
            echo "Cleaning all build directories including deps..."
            rm -rf build/ deps/build-default/
            shift
            ;;
        *)
            echo "Usage: $0 [-jN] [--clean] [--clean-all]"
            echo "  -jN         Number of parallel jobs (default: $(nproc))"
            echo "  --clean     Remove build/ before building"
            echo "  --clean-all Remove build/ and deps/build/"
            exit 1
            ;;
    esac
done

echo "=== PrusaSlicer Linux Build ==="
echo "Using $JOBS parallel jobs"
echo ""

# Check basic prerequisites
for cmd in cmake make g++; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "ERROR: '$cmd' not found. Install build prerequisites:"
        echo ""
        echo "  sudo apt-get install -y git build-essential autoconf cmake \\"
        echo "    libglu1-mesa-dev libgtk-3-dev libdbus-1-dev \\"
        echo "    libwebkit2gtk-4.1-dev texinfo"
        exit 1
    fi
done

# Step 1: Build dependencies (skip if already built)
DEPS_BUILD="$SCRIPT_DIR/deps/build-default"
DEPS_DESTDIR="$DEPS_BUILD/destdir"
if [ -d "$DEPS_DESTDIR" ]; then
    echo "=== Dependencies already built, skipping (use --clean-all to rebuild) ==="
else
    echo "=== Building dependencies ==="
    mkdir -p "$DEPS_BUILD"
    cd "$DEPS_BUILD"
    cmake .. -DDEP_WX_GTK3=ON
    make -j"$JOBS"
    cd "$SCRIPT_DIR"
fi
echo ""

# Step 2: Build PrusaSlicer
echo "=== Building PrusaSlicer ==="
mkdir -p build
cd build
cmake .. \
    -DSLIC3R_STATIC=1 \
    -DSLIC3R_GTK=3 \
    -DSLIC3R_PCH=OFF \
    -DCMAKE_PREFIX_PATH="$DEPS_DESTDIR/usr/local"
make -j"$JOBS"

echo ""
echo "=== Build complete ==="
echo "Binary: $SCRIPT_DIR/build/src/prusa-slicer"
echo "Run:    ./build/src/prusa-slicer"
