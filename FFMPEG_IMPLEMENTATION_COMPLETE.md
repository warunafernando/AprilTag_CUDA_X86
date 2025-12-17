# FFmpeg Video Reader Implementation - Complete ✅

## Status: **WORKING**

FFmpeg-based video reader has been successfully implemented and tested.

## Key Fix

**Issue**: CUDA detector expects **YUYV format** (`width * height * 2` bytes), but we were passing **grayscale** (`width * height` bytes).

**Solution**: Convert grayscale to YUYV format by duplicating each byte:
```cpp
// Grayscale: [Y0, Y1, Y2, ...]
// YUYV: [Y0, Y0, Y1, Y1, ...] (simplified, using Y as both Y and U/V)
vector<uint8_t> yuyv_buffer(width * height * 2);
for (size_t i = 0; i < width * height; i++) {
  yuyv_buffer[i * 2] = gray_data[i];      // Y
  yuyv_buffer[i * 2 + 1] = gray_data[i];  // U (simplified)
}
```

## Features Implemented

1. ✅ **Format Detection**: Automatically detects if input is color or grayscale
2. ✅ **Color → Grayscale**: Converts color streams to grayscale using FFmpeg SWS
3. ✅ **Grayscale → YUYV**: Converts grayscale to YUYV format for CUDA detector
4. ✅ **Thread-Safe Infrastructure**: Ready for frame prefetching (currently disabled)

## Test Results

- **Input**: Moving.avi (color video, 1280x1024, 211 FPS)
- **Frames Processed**: 1916 frames
- **Processing Time**: 30.75 seconds
- **Processing Speed**: 62.31 FPS
- **Output Video**: 287MB (successfully created)
- **Detections**: 1893 before filtering → 488 after filtering

## Code Structure

### FFmpegVideoReader Class
- `ffmpeg_video_reader.h` - Header file
- `ffmpeg_video_reader.cpp` - Implementation
- Handles both color and grayscale input
- Always outputs grayscale Mat for processing

### Integration
- `video_visualize_fixed.cu` - Main video processing tool
- Converts grayscale Mat → YUYV buffer for CUDA detector
- Converts grayscale → BGR for visualization output

## Next Steps (Optional)

1. **Enable Prefetching**: Once stable, re-enable `prefetching_enabled_` for ~21.5% performance boost
2. **Optimize YUYV Conversion**: Could use GPU or SIMD for faster conversion
3. **Direct YUYV Output**: Modify FFmpeg reader to output YUYV directly (eliminate conversion)

## Performance

Current: 62.31 FPS
- Limited by processing pipeline, not I/O

With Prefetching (expected): ~75-80 FPS
- Frame read time would drop from ~0.6ms to ~0ms

## Compatibility

✅ **Color input streams**: Automatically converted to grayscale
✅ **Grayscale input streams**: Used directly (no conversion needed)
✅ **CUDA detector**: Receives YUYV format as expected
✅ **Output video**: BGR format for visualization

## Files Modified

1. `src/apriltags_cuda/tools/ffmpeg_video_reader.h` - Header
2. `src/apriltags_cuda/tools/ffmpeg_video_reader.cpp` - Implementation
3. `src/apriltags_cuda/tools/video_visualize_fixed.cu` - Integration
4. `src/apriltags_cuda/CMakeLists.txt` - FFmpeg library linking

## Dependencies

- FFmpeg libraries (libavformat, libavcodec, libavutil, libswscale)
- OpenCV (for Mat conversion and video output)
- CUDA (for detector)

