#!/bin/bash
# Performance Test Harness for AprilTag CUDA Detector
# Usage: ./test_performance.sh [--video <path>] [--output <file>] [--baseline]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VIDEO_PATH="${1:-input/Moving.avi}"
OUTPUT_FILE="${2:-test_results/test_results_$(date +%Y%m%d_%H%M%S).txt}"
BASELINE_MODE="${3:-}"

# Ensure test_results directory exists
mkdir -p "${SCRIPT_DIR}/test_results"

BINARY="$SCRIPT_DIR/src/apriltags_cuda/build/video_visualize_fixed"
OPENCV_LIB="$SCRIPT_DIR/src/apriltags_cuda/build/OpenCV-install/lib"

if [ ! -f "$BINARY" ]; then
    echo "Error: Binary not found at $BINARY"
    echo "Please build the project first: cd src/apriltags_cuda/build && cmake --build . --target video_visualize_fixed"
    exit 1
fi

if [ ! -f "$VIDEO_PATH" ]; then
    echo "Error: Video not found at $VIDEO_PATH"
    exit 1
fi

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OPENCV_LIB

echo "=========================================="
echo "AprilTag CUDA Performance Test"
echo "=========================================="
echo "Video: $VIDEO_PATH"
echo "Output: $OUTPUT_FILE"
echo "Date: $(date)"
echo "=========================================="
echo ""

# Run the test and capture output
"$BINARY" --video "$VIDEO_PATH" 2>&1 | grep -v "retrieveFrame Unknown/unsupported picture format" | tee "$OUTPUT_FILE"

echo ""
echo "=========================================="
echo "Test complete. Results saved to: $OUTPUT_FILE"
echo "=========================================="

