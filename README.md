# Standalone CUDA AprilTag Detector

A high-performance CUDA-based AprilTag detection system with coordinate scaling fixes, duplicate filtering, and 3D visualization capabilities.

## Overview

This project provides a standalone implementation of the CUDA AprilTag detector based on the [Team766/apriltags_cuda](https://github.com/Team766/apriltags_cuda) repository. It includes several enhancements:

- **Coordinate Scaling Fix**: Corrects GPU detector coordinates from decimated space to full resolution
- **Duplicate Detection Filtering**: Removes false positives and duplicate detections
- **3D Visualization**: Real-time 3D pose visualization with distance measurement
- **Performance Benchmarking**: Comprehensive performance analysis tools

## Features

- GPU-accelerated AprilTag detection using CUDA
- Real-time video processing with 3D visualization
- Coordinate transformation fixes for accurate tag localization
- Duplicate detection filtering for reliable results
- Distance measurement from camera to tag
- Frame-by-frame performance metrics
- Support for multiple tag families (tag36h11, tag25h9, etc.)

## Requirements

- **CUDA Toolkit**: 12.2 or compatible
- **CMake**: 3.15 or higher
- **Ninja**: Build system
- **OpenCV**: Built with CUDA support (automatically built during setup)
- **System Libraries**:
  - `libapriltag` (installed system-wide at `/usr/local/lib/libapriltag.so`)
  - Headers at `/usr/local/include/apriltag/`

## Directory Structure

```
StandAlone/
├── src/
│   ├── apriltags_cuda/          # Main CUDA AprilTag detector
│   │   ├── src/                  # Core detection code
│   │   ├── tools/                # Utility tools
│   │   │   ├── video_visualize_fixed.cu  # Visualization with fixes
│   │   │   ├── compare_detectors.cu      # CPU vs GPU comparison
│   │   │   └── debug_coordinates.cu      # Coordinate debugging
│   │   └── CMakeLists.txt
│   └── apriltag_cgpadwick/       # CPU AprilTag library (dependency)
├── input/                        # Input video files
│   ├── Stable.avi
│   └── Moving.avi
├── output/                       # Generated video outputs (excluded from git)
├── reports/                      # Performance reports and analysis
│   ├── VISUALIZATION_UPDATE_SUMMARY.md
│   ├── PERFORMANCE_TIMING_REPORT.md
│   └── [other analysis reports]
├── config.txt                    # Configuration file for detector parameters
├── setup_env.sh                  # Environment setup script
└── README.md                     # This file
```

## Setup

1. **Set up environment** (required before building):
```bash
source setup_env.sh
```

2. **Build the project**:
```bash
cd src/apriltags_cuda/build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CUDA_ARCHITECTURES=86 \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.2/bin/nvcc \
  -DCMAKE_CUDA_STANDARD=17 ..
ninja -j4
```

**Note**: Adjust `CMAKE_CUDA_ARCHITECTURES` for your GPU:
- RTX 2050/2060/2070/2080: `86`
- RTX 3050/3060/3070/3080: `86`
- RTX 3090/4090: `89`
- Check your GPU compute capability: `nvidia-smi --query-gpu=compute_cap --format=csv`

## Usage

### Video Visualization with 3D Tags

Display or generate output videos with 3D tag visualization, real-time information table, and detailed timing:

```bash
cd src/apriltags_cuda/build
# Display only (default)
./video_visualize_fixed --video /home/nav/Apriltag/StandAlone/input/Stable.avi

# With output file
./video_visualize_fixed \
  --video /home/nav/Apriltag/StandAlone/input/Stable.avi \
  --output /home/nav/Apriltag/StandAlone/output/Stable_3d_fixed.avi \
  --family tag36h11 \
  --tag_size 0.305
```

**Options**:
- `--video`: Input video path (required)
- `--output`: Output video path (optional, if omitted, only displays on screen)
- `--family`: Tag family (default: `tag36h11`)
- `--tag_size`: Tag size in meters (default: `0.305` for 1-foot tags)
- `--min_distance`: Minimum distance for duplicate filtering in pixels (default: `50.0`)

**Visualization Features**:
- **3D Axes**: Color-coded axes showing tag orientation (X=red, Y=green, Z=blue)
- **Tag Outlines**: Yellow rectangles around detected tags
- **Tag IDs**: Green text showing tag ID numbers
- **Information Table**: Compact table in top-left corner displaying:
  - Current FPS
  - Per-tag data: ID, X, Y, Z coordinates, Distance, Probability
  - All values with 2 decimal precision

### Compare CPU vs GPU Detectors

Compare detection accuracy between CPU and GPU implementations:

```bash
./compare_detectors \
  --video /home/nav/Apriltag/input/Stable.avi \
  --family tag36h11 \
  --max_frames 100
```

### Debug Coordinate Transformations

Debug coordinate scaling issues:

```bash
./debug_coordinates \
  --video /home/nav/Apriltag/input/Stable.avi \
  --family tag36h11 \
  --max_frames 10
```

## Key Fixes and Enhancements

### 1. Coordinate Scaling Fix

The GPU detector processes images at decimated resolution (typically 2x decimation) for performance. The original implementation had issues with coordinate scaling, causing detections to appear at incorrect locations.

**Fix**: Added coordinate scaling in `video_visualize_fixed.cu` to convert from decimated space to full resolution:
```cpp
void scale_detection_coordinates(apriltag_detection_t *det, double decimate_factor)
```

### 2. Duplicate Detection Filtering

The GPU detector can produce multiple detections for the same physical tag. Added filtering to keep only the best detection per tag ID based on decision margin.

**Implementation**: `filter_duplicates()` function filters by:
- Tag ID (keeps best decision margin per ID)
- Coordinate validity (removes detections outside image bounds)
- Minimum distance threshold

### 3. Distance Measurement

Added real-time distance calculation from camera to tag using pose estimation:
- Calculates translation vector magnitude
- Displays distance in meters above each tag
- Format: `ID:X Dist:Y.XXXm`

### 4. Tag Size Configuration

Default tag size set to 0.305m (1 foot) for accurate distance measurements. Adjust with `--tag_size` parameter if your tags are different sizes.

### 5. Enhanced Visualization

Recent improvements to the visualization system:

- **Information Table**: Compact table in top-left corner displaying:
  - Current processing FPS
  - Per-tag information: ID, X, Y, Z coordinates, Distance (meters), Probability (normalized 0-1)
  - All numeric values with 2 decimal precision
- **Optimized Layout**: Smaller fonts, compact table design
- **3D Visualization**: Color-coded axes (X=red, Y=green, Z=blue) showing tag orientation
- **Real-time Display**: Live video display with all visualizations overlaid

See `reports/VISUALIZATION_UPDATE_SUMMARY.md` for detailed information about visualization improvements.

## Performance

### Typical Performance (1280x1024 Grayscale Video)

Performance varies based on number of detected tags and scene complexity:
- **Processing FPS**: ~84-86 FPS (depending on detection workload)
- **Per-frame timing breakdown** (typical averages):
  - Frame read: 0.56-0.58 ms (Reader thread, non-blocking)
  - Detection total: 2.25-2.28 ms
    - CUDA operations: 1.51-1.55 ms (~67-68% of detection)
    - CPU decode: 0.69-0.70 ms (~30-31% of detection)
  - Scale coordinates: <0.01 ms
  - Filter duplicates: <0.01 ms
  - Draw (axes/text): 8.81-9.09 ms (largest component, includes visualization)
  - Write frame: 2-5 ms (when enabled, runs in parallel Writer thread)

### Multi-Threaded Architecture

The application uses a three-thread architecture to maximize performance:

1. **Reader Thread** (Frame Prefetching)
   - Prefetches frames from video file in parallel
   - Maintains a bounded queue (default: 10 frames)
   - Eliminates blocking I/O from main detection loop
   - Thread-safe timing accumulation

2. **Main Detection Thread**
   - Performs GPU-accelerated AprilTag detection
   - Scales coordinates, filters duplicates
   - Draws 3D visualization and information table
   - Displays frames on screen
   - Optionally enqueues frames for writing

3. **Writer Thread** (Video Output, optional)
   - Writes processed frames to output file in parallel
   - Only active when `--output` is specified
   - Maintains a bounded queue (default: 5 frames)
   - Prevents I/O from blocking detection

**Threading Benefits**:
- Non-blocking frame reading (0.56-0.58 ms in parallel)
- Parallel video writing capability (when enabled)
- Overall throughput: ~84-86 FPS for 1280x1024 video
- Detection pipeline alone: ~440-450 FPS potential

See `reports/THREADING_ARCHITECTURE_AND_TEST_RESULTS.md` for detailed threading analysis and test results.

### Detection Quality
- **Detection accuracy**: ~99%+ with coordinate fixes
- **Coordinate accuracy**: <1 pixel average error vs CPU detector
- **False positive rate**: Significantly reduced with duplicate filtering

See `reports/PERFORMANCE_TIMING_REPORT.md` for detailed timing analysis.

## Configuration

All detector parameters can be configured via `config.txt`. Key settings include:

- **Detector parameters**: `quad_decimate`, filtering thresholds, etc.
- **Camera intrinsics**: Focal length, principal point
- **Distortion coefficients**: Radial and tangential distortion
- **Tag size**: Physical tag size in meters
- **Filtering parameters**: Minimum distance for duplicate filtering
- **Threading parameters**:
  - `[prefetching]`: Frame prefetching queue size and policy
  - `[writer]`: Video output queue size and policy

See `config.txt` for all available options and default values.

## Troubleshooting

### Build Issues

1. **CUDA not found**: Ensure CUDA is installed and `CUDA_HOME` is set:
   ```bash
   export CUDA_HOME=/usr/local/cuda-12.2
   ```

2. **OpenCV not found**: OpenCV is built automatically during CMake configuration. If issues occur, check build logs in `src/apriltags_cuda/build/`.

3. **apriltag library not found**: Ensure the CPU apriltag library is installed:
   ```bash
   # Check if installed
   pkg-config --exists apriltag && echo "OK" || echo "Not found"
   ```

### Runtime Issues

1. **Segmentation fault**: Ensure GPU detector is warmed up before processing. The visualization tool includes automatic warmup.

2. **Incorrect distances**: Verify tag size matches your actual tag size:
   ```bash
   # For 1-foot tags (default)
   --tag_size 0.305
   
   # For 6-inch tags
   --tag_size 0.152
   ```

3. **No detections**: Check that:
   - Video path is correct
   - Tag family matches your tags (`tag36h11`, `tag25h9`, etc.)
   - Tags are visible and properly lit in the video

## Development

### Adding New Tools

Tools are located in `src/apriltags_cuda/tools/`. To add a new tool:

1. Create `.cu` file in `tools/`
2. Add executable to `CMakeLists.txt`:
   ```cmake
   add_executable(my_tool tools/my_tool.cu src/apriltag_utils.cu)
   target_link_libraries(my_tool apriltag_cuda ...)
   ```

### Modifying Detection Pipeline

Core detection code is in `src/apriltags_cuda/src/apriltag_detect.cu`. Key functions:
- `GpuDetector::Detect()`: Main detection entry point
- `GpuDetector::DecodeTags()`: Tag decoding and coordinate transformation
- `AdjustPixelCenters()`: Coordinate scaling from decimated to full resolution

## License

This project is based on:
- [Team766/apriltags_cuda](https://github.com/Team766/apriltags_cuda) - CUDA AprilTag detector
- [cgpadwick/apriltag](https://github.com/cgpadwick/apriltag) - CPU AprilTag library

See respective repositories for license information.

## Contributing

When contributing, please:
1. Test with both Stable.avi and Moving.avi
2. Verify coordinate accuracy using `compare_detectors`
3. Update this README if adding new features
4. Follow existing code style and conventions

## Acknowledgments

- Team766 for the CUDA AprilTag implementation
- cgpadwick for the enhanced CPU AprilTag library
- OpenCV team for computer vision libraries
