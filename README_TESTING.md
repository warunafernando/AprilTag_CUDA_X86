# AprilTag CUDA Performance Testing

This directory contains tools for performance testing and benchmarking the AprilTag CUDA detector.

## Test Harness

### Quick Start

Run a performance test:
```bash
./test_performance.sh [video_path] [output_file]
```

Example:
```bash
./test_performance.sh input/Moving.avi my_test_results.txt
```

### Test Script Details

**`test_performance.sh`** - Automated performance testing script

- Runs the video visualization tool with full timing statistics
- Captures all output to a text file
- Automatically sets up library paths
- Provides error checking for missing binaries/videos

**Usage:**
```bash
./test_performance.sh [video_path] [output_file]
```

**Arguments:**
- `video_path`: Path to input video (default: `input/Moving.avi`)
- `output_file`: Path to save test results (default: `test_results_YYYYMMDD_HHMMSS.txt`)

### Analysis Tool

**`analyze_timing.py`** - Parse and format test results

Extracts key metrics from test output files and generates formatted summaries.

**Usage:**
```bash
./analyze_timing.py <test_output_file>
```

**Output:**
- Prints formatted summary to console
- Saves detailed report to `<filename>_summary.txt`

**Extracted Metrics:**
- Total frames processed and processing time
- Pipeline performance (avg ms/frame, theoretical FPS)
- Detection statistics (before/after filtering, filter rate)
- Component timing breakdown with percentages
- Filtering parameters used

## Test Results Format

Test output files contain:
1. Configuration information (video path, resolution, FPS)
2. Filtering parameters (if tuned)
3. Processing statistics (frames, detections)
4. Detailed timing breakdown for each component:
   - Frame Read
   - Color Conversion
   - GPU Detection (Total)
     - CUDA Operations
     - CPU Decode
   - Coordinate Scaling
   - Filter Duplicates
   - Display (imshow)
5. Total pipeline timing

## Example Workflow

```bash
# Run performance test
./test_performance.sh input/Moving.avi baseline_results.txt

# Analyze results
./analyze_timing.py baseline_results.txt

# Run with tuned parameters (modify code), then test again
./test_performance.sh input/Moving.avi tuned_results.txt
./analyze_timing.py tuned_results.txt

# Compare results manually or create comparison script
```

## Current Performance (After Tuning)

**Configuration:**
- Video: 1280x1024 @ 211 FPS
- Filtering parameters tuned for early false positive rejection
  - min_cluster_pixels: 8 (default: 5)
  - max_line_fit_mse: 6.0 (default: 10.0)
  - critical_angle: 7° (default: 10°)
  - min_white_black_diff: 8 (default: 5)

**Typical Results:**
- **Pipeline FPS**: ~107-109 FPS (theoretical)
- **Processing FPS**: ~26-31 FPS (actual video processing)
- **GPU Detection**: ~7.4 ms/frame
  - CUDA Operations: ~3.6 ms (49%)
  - CPU Decode: ~2.9 ms (39%)
- **Filter Rate**: ~75% of detections filtered as false positives

## Future Improvements

1. **Automated Comparison**: Script to compare two test results side-by-side
2. **Regression Testing**: Baseline comparison with thresholds
3. **Multi-video Testing**: Batch processing multiple test videos
4. **Performance Tracking**: Store results over time to track improvements
5. **Visualization**: Generate graphs/charts from timing data

