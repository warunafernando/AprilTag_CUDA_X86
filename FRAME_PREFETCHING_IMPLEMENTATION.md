# Frame Prefetching Implementation

## Summary

Frame prefetching has been successfully implemented to improve video processing performance by overlapping I/O operations with computation.

## Changes Made

### 1. FramePrefetcher Class (`src/apriltags_cuda/tools/video_visualize_fixed.cu`)

Added a new `FramePrefetcher` class that:
- Runs a background I/O thread to read frames from the video file
- Maintains a thread-safe queue of prefetched frames (max 2 frames ahead)
- Provides a non-blocking `getNextFrame()` method
- Automatically cleans up threads on destruction

**Key Features**:
- Thread-safe queue using `mutex` and `condition_variable`
- Queue size limited to 2 frames to prevent memory buildup
- Graceful shutdown when video ends
- Non-blocking frame retrieval

### 2. Modified Main Loop

**Before** (blocking):
```cpp
do {
    cap.read(frame);  // Blocks for ~0.616ms
    // Process frame...
} while (cap.read(frame));
```

**After** (non-blocking with prefetching):
```cpp
FramePrefetcher prefetcher(cap);
Mat frame;
while (prefetcher.getNextFrame(frame)) {  // Returns immediately (~0ms)
    // Process frame...
}
prefetcher.stop();
```

### 3. Video Initialization

Modified to read the first frame separately to get video dimensions, then rewind for the prefetcher:
- Read first frame to get width/height
- Rewind video to position 0
- Prefetcher starts reading from the beginning

## Expected Performance Impact

### Current Performance (without prefetching)
- Frame Read: **0.616 ms** (17.5% of pipeline)
- Processing: **2.908 ms** (82.5% of pipeline)
- Total Pipeline: **3.524 ms/frame** → **283.73 FPS**

### Expected Performance (with prefetching)
- Frame Read: **~0.0 ms** (overlapped with processing)
- Processing: **2.908 ms** (100% of visible pipeline)
- Effective Pipeline: **~2.908 ms/frame** → **~344 FPS**

### Improvement
- **~0.616 ms saved per frame** (17.5% improvement)
- **~61 FPS increase** (283 → 344 FPS)

## Implementation Details

### Thread Safety
- Uses `std::mutex` for queue access protection
- Uses `std::condition_variable` for efficient waiting
- Atomic `running_` flag for clean shutdown

### Memory Management
- Queue limited to 2 frames maximum
- Oldest frames dropped if queue is full (prevents memory buildup)
- Frames cloned when added to queue (safe for multithreading)

### Error Handling
- Prefetcher stops gracefully when video ends
- Main loop exits cleanly when no more frames available
- Destructor ensures thread cleanup

## Testing

The implementation has been compiled successfully. To test:

```bash
# Run performance test
./test_performance.sh --video input/Moving.avi

# Compare timing statistics
python analyze_timing.py test_results/test_results_*.txt
```

## Verification

Check the timing statistics in the output:
- **Frame Read time** should drop from ~0.616ms to ~0.0ms
- **Total Pipeline time** should decrease from ~3.524ms to ~2.908ms
- **Effective FPS** should increase from ~283 to ~344 FPS

## Code Location

All changes are in:
- `src/apriltags_cuda/tools/video_visualize_fixed.cu`
  - Lines 270-336: `FramePrefetcher` class definition
  - Lines 549-564: Modified video initialization
  - Lines 748-753: Modified main loop to use prefetcher

## Notes

- The prefetcher maintains a maximum of 2 frames in the queue
- This prevents excessive memory usage while ensuring frames are always ready
- Frame reading now happens in parallel with GPU processing
- The visualization thread continues to run independently

