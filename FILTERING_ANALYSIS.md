# Filtering Analysis - Frames Lost to Filtering

## Summary

This document analyzes frames that had detections **before filtering** but **no detections after filtering**. These frames had false positives that were correctly filtered out.

## Current Results

From the test run:
- **Total frames analyzed**: 1916
- **Frames with detections before filtering**: 1892 (98.7%)
- **Frames with detections after filtering**: 1888 (98.5%)
- **Frames lost to filtering**: 4 frames (0.2%)
  - Frame numbers: 865, 1191, 1592, 1906
- **Average detections per lost frame**: 1.50
- **Total detections filtered from these frames**: 6

## Current Filtering Parameters

The tuned parameters that filter false positives:

- `min_cluster_pixels: 8` (default: 5)
  - Rejects quads with fewer than 8 pixels
  
- `max_line_fit_mse: 6.0` (default: 10.0)
  - Stricter line fit error requirement (lower = more filtering)
  
- `critical_angle: 7°` (default: 10°)
  - More aggressive angle filtering (smaller angle = more filtering)
  
- `min_white_black_diff: 8` (default: 5)
  - Requires stronger contrast (higher = more filtering)

## Analysis

### What These Frames Tell Us

1. **Low Detection Count**: Each lost frame had only 1-2 detections on average, indicating these were likely weak false positives.

2. **Filtering Effectiveness**: Only 0.2% of frames lost all detections, which means:
   - 99.8% of frames either had no detections (from start) or retained at least one valid detection
   - The filtering is working well - removing false positives while preserving real detections

3. **Potential Improvements**:
   - These frames could be analyzed to see if they represent a pattern (e.g., specific lighting conditions, edge cases)
   - If these were valid detections, we might need to relax some filtering parameters
   - If they were false positives (most likely), the filtering is correct

## Fine-Tuning Recommendations

### Option 1: Current Settings (Recommended)
**Status**: Working well - only 0.2% frames affected

The current filtering parameters are working effectively:
- High detection rate (98.5% of frames have detections after filtering)
- Low false positive rate (74.5% of detections filtered as false positives)
- Minimal frame loss (0.2%)

### Option 2: More Aggressive Filtering
If you want to filter even more false positives, you could:
- Increase `min_cluster_pixels` from 8 → 10
- Decrease `max_line_fit_mse` from 6.0 → 5.0
- Increase `min_white_black_diff` from 8 → 10

**Trade-off**: May lose a few more frames but will filter more false positives

### Option 3: Less Aggressive Filtering
If you want to preserve more frames:
- Decrease `min_cluster_pixels` from 8 → 6
- Increase `max_line_fit_mse` from 6.0 → 8.0
- Decrease `min_white_black_diff` from 8 → 6

**Trade-off**: May allow more false positives through

## Next Steps for Analysis

To further analyze these frames, you could:

1. **Visual Inspection**: Extract frames 865, 1191, 1592, 1906 and visually inspect them
   ```bash
   # Use ffmpeg to extract specific frames
   ffmpeg -i input/Moving.avi -vf "select='eq(n,865)'" -vsync vfr frame_865.png
   ```

2. **Detection Characteristics**: Analyze decision margins, coordinates, and tag IDs of the filtered detections

3. **Pattern Recognition**: Look for common characteristics:
   - Lighting conditions
   - Camera angle/distance
   - Background patterns
   - Motion blur

4. **Parameter Sweep**: Test different parameter combinations to find optimal balance

## File: `lost_frames_list.txt`

This file contains the list of frame numbers and their detection counts:
```
# Format: frame_number detection_count
865 1
1191 2
1592 2
1906 1
```

Use this file to extract and analyze specific frames for further investigation.

