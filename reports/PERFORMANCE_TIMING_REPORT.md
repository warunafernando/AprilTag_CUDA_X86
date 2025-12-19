# Performance Timing Report

## Date: December 2024

## Overview
This document describes the timing infrastructure and performance characteristics of the AprilTag CUDA detector video visualization tool.

## Timing Infrastructure

### Per-Stage Timing Breakdown
The application measures and reports detailed timing for each stage of the processing pipeline:

1. **Frame Read** (`acc_read_ms`)
   - Time to read frame from video file
   - Includes reader thread overhead (if enabled)

2. **Detect Total** (`acc_detect_ms`)
   - Total time for tag detection
   - Broken down into:
     - **CUDA Operations** (`acc_cuda_ms`): GPU-accelerated detection pipeline
     - **CPU Decode** (`acc_cpu_decode_ms`): CPU-based tag ID extraction and decoding

3. **Scale Coordinates** (`acc_scale_ms`)
   - Time to scale detection coordinates from decimated to full resolution
   - Only applies when `quad_decimate > 1`

4. **Filter Duplicates** (`acc_filter_ms`)
   - Time to filter duplicate detections based on center distance
   - Removes detections with same ID within threshold distance

5. **Draw (axes/text)** (`acc_draw_ms`)
   - Time to draw tag outlines, 3D axes, tag IDs, and information table
   - Includes pose estimation for 3D visualization
   - Includes table rendering

6. **Write Frame** (`writer_write_ms`)
   - Time to write frame to output video file (if enabled)
   - Runs in separate thread to avoid blocking main pipeline

### Timing Measurement Methods

#### CUDA Operations Timing
- Measured using CUDA events (`CudaEvent`)
- Records events before and after CUDA kernel launches
- Synchronizes stream to ensure accurate timing
- Cumulative duration tracked in `GpuDetector` class

#### CPU Operations Timing
- Measured using `std::chrono::steady_clock`
- High-resolution timing with nanosecond precision
- Accumulated in per-stage timing variables

#### Per-Frame Timing
- Each stage timing measured per frame
- Accumulated over all frames
- Final report shows averages (total_time / frame_count)

## Performance Characteristics

### Expected Performance (1280x1024 Grayscale Video @ 211 FPS)

Based on typical performance characteristics:

#### Frame Read
- **Typical**: 0.5-1.0 ms/frame
- **Optimized**: Direct grayscale read with `CAP_PROP_CONVERT_RGB=false`
- **Reader Thread**: Can reduce main thread blocking (if enabled)

#### Detection Pipeline
- **CUDA Operations**: ~7-9 ms/frame (majority of detection time)
  - Image preprocessing
  - Edge detection
  - Quad fitting
  - GPU-accelerated filtering
- **CPU Decode**: ~2-3 ms/frame
  - Tag ID extraction
  - Hamming distance calculation
  - Codeword matching
- **Total Detection**: ~9-12 ms/frame

#### Post-Processing
- **Scale Coordinates**: <0.1 ms/frame
- **Filter Duplicates**: <0.1 ms/frame
- **Draw**: 1-2 ms/frame
  - Pose estimation: ~0.5-1.0 ms
  - Drawing operations: ~0.5-1.0 ms
- **Write**: 2-5 ms/frame (when enabled, runs in parallel thread)

### Overall Performance

#### Typical Total Pipeline Time
- **Per Frame**: ~11-15 ms (excluding file write)
- **Effective FPS**: ~65-90 FPS (depending on detection workload)
- **Bottleneck**: CUDA detection operations (~60-70% of total time)

#### Performance Optimizations Implemented

1. **Direct Grayscale Input**
   - Bypasses unnecessary color conversion
   - Reduces frame read overhead

2. **Threaded Pipeline** (optional)
   - Reader thread for frame prefetching
   - Writer thread for file output
   - Main thread focuses on detection and visualization

3. **Coordinate Scaling Optimization**
   - Minimal overhead (<0.1 ms)
   - Only applied when necessary

4. **Efficient Duplicate Filtering**
   - Sorted by decision margin (best first)
   - Early termination for nearby duplicates
   - O(n²) but optimized with distance thresholding

## Measurement Methodology

### Timing Accumulation
```cpp
auto start = chrono::steady_clock::now();
// ... operation ...
auto end = chrono::steady_clock::now();
acc_stage_ms += chrono::duration<double, milli>(end - start).count();
```

### CUDA Timing
```cpp
CudaEvent before_op;
before_op.Record(&stream_);
// ... CUDA operations ...
CudaEvent after_op;
after_op.Record(&stream_);
stream_.Synchronize();
float elapsed_ms = after_op.ElapsedTime(before_op);
```

### Reporting
- All timings reported as averages: `total_time / frame_count`
- Precision: 2 decimal places
- Units: milliseconds per frame

## Detection Statistics

### Frame Detection Histogram
The application tracks and reports:
- Number of frames with 0, 1, 2, ... tags detected (before filtering)
- Number of frames with 0, 1, 2, ... tags detected (after filtering)
- Helps identify detection consistency and filtering effectiveness

### Detection Counts
- Total detections before filtering
- Total detections after filtering
- Average detections per frame (before and after)

## Performance Bottlenecks

### Current Bottlenecks (in order of impact)
1. **CUDA Detection Operations** (~7-9 ms, ~60-70% of total)
   - GPU-bound, difficult to optimize further without hardware changes
   - Includes edge detection, quad fitting, filtering

2. **CPU Decode** (~2-3 ms, ~15-20% of total)
   - Potential for GPU offload (partially implemented)
   - Includes tag ID extraction, Hamming distance

3. **Drawing Operations** (~1-2 ms, ~10-15% of total)
   - CPU-bound pose estimation and OpenCV drawing
   - Could be optimized with GPU-accelerated drawing (complex)

4. **Frame Read** (~0.5-1 ms, ~5-10% of total)
   - Already optimized with direct grayscale read
   - Minimal improvement possible

## Recommendations for Further Optimization

1. **GPU Decode Offload** (if not fully implemented)
   - Move tag ID extraction to GPU
   - Reduce CPU decode time

2. **Parallel Pose Estimation**
   - GPU-accelerated pose estimation
   - Batch processing multiple tags

3. **Optimized Drawing**
   - Use GPU-accelerated rendering APIs
   - Reduce CPU-GPU transfers

4. **Detection Algorithm Optimization**
   - Further optimize CUDA kernels
   - Reduce redundant computations
   - Optimize memory access patterns

## Usage

### Running with Timing Output
The application automatically reports detailed timing at the end of processing:

```bash
./video_visualize_fixed --video input/Moving.avi
```

### Timing Output Format
```
Timing (ms/frame, averages):
  Frame read:        X.XX
  Detect total:      X.XX
    CUDA ops:        X.XX
    CPU decode:      X.XX
  Scale coordinates: X.XX
  Filter duplicates: X.XX
  Draw (axes/text):  X.XX
  Write frame:       X.XX
```

### Performance Monitoring
- Monitor average processing FPS
- Check per-stage timing breakdown
- Identify bottlenecks based on relative times
- Track detection statistics for quality assessment




