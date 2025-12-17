# Frame Prefetching Implementation Status

## Current Status: **DISABLED** (Threading Issue)

Frame prefetching was attempted but resulted in segmentation faults due to threading issues with OpenCV's `VideoCapture`.

## Issue

OpenCV's `VideoCapture` is **not thread-safe**. Attempts to use it from multiple threads (even with separate instances) caused segmentation faults. 

### Attempted Solutions

1. **Separate VideoCapture instances**: Created separate `VideoCapture` objects for main thread and prefetcher thread - still caused segfaults
2. **Proper thread synchronization**: Used mutexes and condition variables for thread safety - still had issues
3. **VideoCapture reference vs. path**: Tried passing video path instead of VideoCapture reference - still unstable

## Root Cause

The segfaults suggest that OpenCV's video decoding backend (likely FFmpeg) may have internal state that conflicts when multiple `VideoCapture` instances access the same video file concurrently, even from different threads.

## Future Solutions

### Option 1: Synchronized Single VideoCapture (Medium Effort)
- Use a single `VideoCapture` instance
- Lock access with mutex (defeats the purpose of prefetching)
- **Not recommended** - would add overhead without benefit

### Option 2: Memory-Mapped Frame Buffer (High Effort)
- Read entire video to memory buffer
- Access frames directly from memory
- **Benefit**: No I/O blocking, true prefetching
- **Cost**: High memory usage, longer startup time

### Option 3: Frame Caching Without Threading (Low Effort)
- Use OpenCV's built-in buffer (`CAP_PROP_BUFFERSIZE`)
- Limited to OpenCV's internal buffering
- **Benefit**: Simple, some I/O overlap
- **Cost**: Limited improvement

### Option 4: Async I/O with Separate Process (High Effort)
- Use a separate process to read frames
- Communicate via shared memory or IPC
- **Benefit**: True isolation, no threading issues
- **Cost**: Complex implementation

## Current Performance

With blocking reads (current implementation):
- Frame Read: **0.616 ms** (17.5% of pipeline)
- Total Pipeline: **3.524 ms/frame** → **283.73 FPS**

Theoretical with prefetching (if working):
- Frame Read: **~0.0 ms** (overlapped)
- Total Pipeline: **~2.908 ms/frame** → **~344 FPS**
- Improvement: **~17.5% faster**

## Recommendation

For now, **keep using blocking reads**. The current performance (283 FPS) is already excellent, and the threading complexity outweighs the benefit until a more robust solution can be implemented.

If higher performance is critical, consider:
1. **Option 3** (CAP_PROP_BUFFERSIZE) - simplest, low risk
2. **Option 2** (memory buffer) - best performance, higher memory cost
3. Wait for OpenCV improvements or use a different video library

## Code Location

The prefetching code was temporarily added to:
- `src/apriltags_cuda/tools/video_visualize_fixed.cu` (lines 270-336: FramePrefetcher class)

**Status**: Reverted to original blocking read implementation.

