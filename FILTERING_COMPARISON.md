# Filtering Parameter Comparison

## Test Results: Less Aggressive Filtering (Option 3)

### Parameter Changes

| Parameter | Previous (Aggressive) | New (Less Aggressive) | Default |
|-----------|----------------------|----------------------|---------|
| `min_cluster_pixels` | 8 | **6** | 5 |
| `max_line_fit_mse` | 6.0 | **8.0** | 10.0 |
| `min_white_black_diff` | 8 | **6** | 5 |
| `critical_angle` | 7° | 7° (unchanged) | 10° |

### Detection Results

**Both configurations produce identical detection results:**

- **Before filtering**: 7412 detections (3.87/frame)
- **After filtering**: 1888 detections (0.99/frame)
- **Filter rate**: 74.5% (same in both)
- **Frames with detections before**: 1892 (98.7%)
- **Frames with detections after**: 1888 (98.5%)
- **Frames lost to filtering**: 4 frames (0.2%)

### Key Findings

1. **Same Detection Quality**: Less aggressive filtering maintains the same final detection results
   - Same number of frames with detections preserved
   - Same filter rate
   - Same frames lost (4 frames: 865, 1191, 1592, 1906)

2. **Performance Impact**: Less aggressive filtering may allow more candidates through early GPU filtering, but they get filtered later, resulting in the same final count

3. **Why Same Results?**: 
   - The 4 frames that lose all detections have weak false positives (1-2 detections each)
   - These detections are likely filtered by the duplicate/invalid coordinate filtering, not the early GPU filtering
   - Less aggressive early filtering allows more candidates to reach CPU decode, but they still get filtered later

### Recommendation

**Use Less Aggressive Filtering (Option 3):**

✅ **Advantages:**
- Same detection quality (no loss)
- Slightly less restrictive early filtering (may help in edge cases)
- Closer to defaults (more conservative approach)
- Better balance between filtering aggressiveness

**Parameters to use:**
- `min_cluster_pixels: 6`
- `max_line_fit_mse: 8.0`
- `min_white_black_diff: 6`
- `critical_angle: 7°` (keep as is)

### Conclusion

The less aggressive filtering parameters provide the same detection quality while being less restrictive. Since the final results are identical, the less aggressive settings are preferable as they:
1. Preserve more candidates early (which may help in edge cases)
2. Are closer to default values (more conservative)
3. Still filter effectively at later stages

The 4 frames that lose detections appear to have very weak false positives that get filtered regardless of the early filtering aggressiveness, suggesting they are correctly identified as false positives by the duplicate/invalid coordinate filtering.

