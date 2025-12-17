# Configuration File Documentation

## Overview

All detector settings and tuning parameters are now controlled via a configuration file (`config.txt`). This makes it easy to adjust parameters without recompiling the code.

## Configuration File: `config.txt`

The configuration file uses a simple key-value format with sections:

```
[section]
key: value
```

### Sections

#### `[detector]`
Detector basic parameters:
- `family`: Tag family name (default: `tag36h11`)
- `quad_decimate`: Image decimation factor (default: `2.0`)
- `quad_sigma`: Gaussian blur sigma (default: `0.0`)
- `nthreads`: Number of threads (default: `1`)
- `refine_edges`: Enable edge refinement (default: `true`)
- `debug`: Enable debug mode (default: `false`)

#### `[filtering]`
Early false positive filtering parameters (tuned for performance):
- `min_cluster_pixels`: Minimum pixels per cluster (default: `6`)
- `max_line_fit_mse`: Maximum line fit error (default: `8.0`)
- `critical_angle_degrees`: Critical angle for filtering in degrees (default: `7.0`)
- `min_white_black_diff`: Minimum contrast difference (default: `6`)
- `min_distance_for_duplicates`: Minimum distance for duplicate filtering (default: `50.0`)

#### `[camera]`
Camera intrinsic parameters:
- `fx`: Focal length X (default: `905.495617`)
- `fy`: Focal length Y (default: `609.916016`)
- `cx`: Principal point X (default: `907.909470`)
- `cy`: Principal point Y (default: `352.682645`)

#### `[distortion]`
Camera distortion coefficients:
- `k1`, `k2`, `k3`: Radial distortion coefficients
- `p1`, `p2`: Tangential distortion coefficients

#### `tag_size_meters`
Physical tag size in meters (default: `0.305` = 1 foot)

#### `[data_collection]`
Data collection settings:
- `enabled`: Enable data collection features (default: `false`)
- `save_lost_frames`: Save list of frames that lost all detections (default: `true`)
- `lost_frames_output`: Output filename for lost frames list (default: `lost_frames_list.txt`)

## Usage

### Default Configuration
```bash
./src/apriltags_cuda/build/video_visualize_fixed --video input/Moving.avi
```
Uses `config.txt` in the current directory.

### Custom Configuration
```bash
./src/apriltags_cuda/build/video_visualize_fixed --video input/Moving.avi --config my_config.txt
```

### Command Line Options
- `--video, -v <file>`: Input video file (required)
- `--output, -o <file>`: Output video file (optional)
- `--config, -c <file>`: Configuration file (default: `config.txt`)
- `--help, -h`: Show help message

## Current Default Values

These are the tuned values currently saved in `config.txt`:

**Detector:**
- Family: `tag36h11`
- Quad decimate: `2.0`
- Quad sigma: `0.0`
- Threads: `1`
- Refine edges: `true`

**Filtering (Less Aggressive):**
- `min_cluster_pixels`: `6` (allows smaller quads)
- `max_line_fit_mse`: `8.0` (less strict line fit)
- `critical_angle_degrees`: `7.0` (moderate angle filtering)
- `min_white_black_diff`: `6` (moderate contrast requirement)
- `min_distance_for_duplicates`: `50.0` pixels

**Camera & Distortion:**
- Camera matrix and distortion coefficients are set for the specific camera setup
- Tag size: `0.305` meters (1 foot)

**Data Collection:**
- Enabled: `true` (saves lost frames list)
- Output: `lost_frames_list.txt`

## Modifying Configuration

To change parameters:

1. Edit `config.txt` with your desired values
2. Run the detector - it will automatically use the new values
3. No recompilation needed!

### Example: More Aggressive Filtering

To filter more false positives:
```
[filtering]
min_cluster_pixels: 8
max_line_fit_mse: 6.0
min_white_black_diff: 8
```

### Example: Less Aggressive Filtering

To preserve more detections:
```
[filtering]
min_cluster_pixels: 5
max_line_fit_mse: 10.0
min_white_black_diff: 5
```

### Example: Disable Data Collection

To disable data collection features:
```
[data_collection]
enabled: false
```

## Testing

After modifying the configuration, test with:
```bash
./test_performance.sh input/Moving.avi test_results.txt
./analyze_timing.py test_results.txt
```

This will show you the impact of your parameter changes.

## Notes

- All parameters use the default values shown in the config file if not specified
- The configuration file is read at startup - changes require restarting the program
- If the config file is missing, the program will warn but continue with hardcoded defaults
- Comments (lines starting with `#`) are supported in the config file

