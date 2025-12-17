# AprilTag CUDA Detector - Performance Test Report

**Date**: December 16, 2025  
**Test Video**: `input/Moving.avi`  
**Resolution**: 1280x1024 @ 211 FPS  
**Total Frames**: 1916

## Configuration

### Filtering Parameters (Tuned)
- **min_cluster_pixels**: 8 (default: 5) - Increased to filter smaller false positives
- **max_nmaxima**: 10 (FIXED - compiled into CUDA kernel)
- **max_line_fit_mse**: 6.0 (default: 10.0) - Stricter line fit requirement
- **critical_angle**: 7° (default: 10°) - More aggressive angle filtering
- **min_white_black_diff**: 8 (default: 5) - Requires stronger contrast

## Performance Summary

### Overall Performance
- **Total Processing Time**: 72.31 seconds
- **Processing FPS**: 26.50 FPS (actual video processing rate)
- **Pipeline FPS**: 107.49 FPS (theoretical maximum)
- **Average Pipeline Time**: 9.303 ms per frame

### Detection Statistics
- **Detections Before Filtering**: 7,412 (3.87 per frame)
- **Detections After Filtering**: 1,888 (0.99 per frame)
- **Filter Rate**: 74.5% (5,524 false positives filtered)

## Detailed Timing Breakdown

### Component Performance (ms per frame)

| Component | Time (ms) | % of Pipeline | Status |
|-----------|-----------|---------------|--------|
| **GPU Detection (Total)** | **7.363** | **79.2%** | Main bottleneck |
| ├─ CUDA Operations | 3.634 | 39.1% | Largest component |
| └─ CPU Decode | 2.914 | 31.3% | Second largest |
| **Display (imshow)** | **0.853** | **9.2%** | Medium |
| **Frame Read** | **0.775** | **8.3%** | Small |
| **Color Conversion** | 0.307 | 3.3% | Negligible |
| **Filter/Scale** | 0.005 | <0.1% | Negligible |
| **Draw 3D/Text** | N/A | - | Async (not blocking) |

### Timing Details

#### Frame Read
- Average: 0.775 ms
- Min: 0.425 ms
- Max: 9.954 ms
- Total: 1.484 s

#### Color Conversion
- Average: 0.307 ms
- Min: 0.128 ms
- Max: 7.878 ms
- Total: 0.589 s

#### GPU Detection (Total)
- Average: 7.363 ms
- Min: 1.689 ms
- Max: 273.029 ms
- Total: 14.107 s

##### CUDA Operations
- Average: 3.634 ms (49.4% of GPU detection)
- Min: 1.496 ms
- Max: 69.708 ms
- Total: 6.930 s

##### CPU Decode
- Average: 2.914 ms (39.6% of GPU detection)
- Min: 0.122 ms
- Max: 28.891 ms
- Total: 5.580 s

#### Coordinate Scaling
- Average: 0.001 ms
- Total: 0.002 s

#### Filter Duplicates
- Average: 0.004 ms
- Total: 0.008 s

#### Display (imshow)
- Average: 0.853 ms
- Min: 0.297 ms
- Max: 9.990 ms
- Total: 1.632 s

## Performance Analysis

### Bottlenecks

1. **GPU Detection (79.2%)** - Main performance bottleneck
   - CUDA Operations: 3.634 ms (49.4% of detection time)
   - CPU Decode: 2.914 ms (39.6% of detection time)
   - Combined: 6.548 ms out of 7.363 ms total

2. **Display (9.2%)** - Secondary bottleneck
   - OpenCV imshow operations: 0.853 ms
   - Can be eliminated for headless/production use

3. **Frame Read (8.3%)** - Minor contributor
   - Video I/O: 0.775 ms
   - Could potentially be optimized with prefetching

### Optimization Impact

Compared to baseline (before tuning filtering parameters):
- **CPU Decode**: Improved by ~27% (from ~3.5ms to ~2.9ms)
- **Total Pipeline**: Improved by ~18% (from ~10.8ms to ~9.3ms)
- **Pipeline FPS**: Increased from ~92 FPS to ~107 FPS

The tuning successfully filtered false positives earlier, reducing expensive CPU decoding operations.

## Recommendations for Further Optimization

### High Impact (Easy)
1. **Remove Display for Production**: Saves ~0.85 ms (9.2%)
   - Pipeline: 9.30 ms → 8.45 ms
   - Expected FPS: 107 → 118 FPS

2. **Increase Decimation**: 2.0 → 2.5 or 3.0
   - Potential savings: ~0.5-1.0 ms in CUDA operations
   - Trade-off: Slightly reduced detection accuracy for distant tags

### High Impact (Medium Effort)
3. **Prefetch Frames**: Background thread for frame reading
   - Potential savings: ~0.3-0.4 ms
   - Complexity: Medium (threading)

### Highest Impact (High Effort)
4. **Port CPU Decode to GPU**: 2-3 weeks estimated
   - Potential savings: ~2.9 ms (31% of pipeline)
   - Pipeline: 9.30 ms → 6.40 ms
   - Expected FPS: 107 → 156 FPS

## Test Harness

The test harness includes:
- `test_performance.sh` - Automated testing script
- `analyze_timing.py` - Results analysis tool
- `README_TESTING.md` - Testing documentation

Run tests:
```bash
./test_performance.sh input/Moving.avi results.txt
./analyze_timing.py results.txt
```

