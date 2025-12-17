# Video Capture Alternatives to OpenCV VideoCapture

## Current Issue

OpenCV's `VideoCapture` is **not thread-safe**, preventing efficient frame prefetching. Multiple `VideoCapture` instances accessing the same video file cause segmentation faults.

## Alternative Options

### 1. **FFmpeg (libavformat/libavcodec)** ⭐ Recommended

**Pros:**
- ✅ **Thread-safe**: Can create multiple `AVFormatContext` instances safely
- ✅ **Performance**: Same backend OpenCV uses, but with full control
- ✅ **Prefetching**: Native support for frame buffering and async I/O
- ✅ **Flexibility**: Full control over decoding pipeline
- ✅ **Widely available**: FFmpeg is typically already installed
- ✅ **Format support**: Excellent codec and container support

**Cons:**
- ❌ **Complexity**: More complex API than OpenCV
- ❌ **Integration**: Need to convert AVFrames to OpenCV Mat format
- ❌ **Build dependency**: May need to link against libavformat/libavcodec

**Effort**: Medium (2-3 hours to implement)
**Performance Gain**: ~17.5% improvement (0.616ms frame read → ~0ms with prefetching)

### 2. **GStreamer**

**Pros:**
- ✅ **Thread-safe**: Built with threading in mind
- ✅ **Powerful**: Very flexible pipeline system
- ✅ **Good performance**: Well-optimized

**Cons:**
- ❌ **Complexity**: More complex than FFmpeg
- ❌ **Dependency**: Additional library to install
- ❌ **Integration**: More setup required

**Effort**: High (4-6 hours)
**Performance Gain**: Similar to FFmpeg

### 3. **Thread-Safe Wrapper Around OpenCV** (Not Recommended)

**Approach**: Serialize all VideoCapture access with mutexes

**Pros:**
- ✅ **Simple**: Minimal changes to existing code
- ✅ **No new dependencies**

**Cons:**
- ❌ **No prefetching benefit**: Mutex serialization defeats the purpose
- ❌ **No performance gain**: Still blocks on I/O

**Effort**: Low (30 minutes)
**Performance Gain**: None

### 4. **Memory-Map Entire Video** (For Small Videos Only)

**Approach**: Read entire video to memory, then access frames directly

**Pros:**
- ✅ **Simple**: Direct memory access
- ✅ **Fast**: No I/O blocking during processing
- ✅ **Thread-safe**: Can have multiple readers

**Cons:**
- ❌ **Memory**: Requires video to fit in RAM
- ❌ **Startup time**: Must decode entire video first
- ❌ **Not scalable**: Won't work for large videos

**Effort**: Medium (2 hours)
**Performance Gain**: Similar to prefetching, but limited by video size

## Recommendation

**Use FFmpeg directly** (Option 1). Here's why:

1. **OpenCV already uses FFmpeg** - Same backend, so same codec support
2. **Thread-safe by design** - Multiple AVFormatContext instances work independently
3. **Best performance potential** - Full control over I/O and decoding
4. **Proven technology** - FFmpeg is the industry standard

## Implementation Approach

Create a `FFmpegVideoReader` class that:

1. **Thread-safe frame prefetching**:
   - Background thread reads and decodes frames
   - Thread-safe queue for prefetched frames
   - Non-blocking `readFrame()` method

2. **OpenCV Mat conversion**:
   - Convert AVFrame (FFmpeg) → Mat (OpenCV) when needed
   - Minimal overhead (just memory copy + format conversion)

3. **Same interface as VideoCapture**:
   - `open()`, `read()`, `get()`, `release()` methods
   - Easy drop-in replacement

4. **Performance optimizations**:
   - Reuse AVFrame buffers
   - Zero-copy when possible
   - Queue size limit (2-3 frames)

## Code Structure

```cpp
class FFmpegVideoReader {
public:
    bool open(const std::string& video_path);
    bool read(cv::Mat& frame);
    double get(int prop_id);  // CAP_PROP_FPS, CAP_PROP_FRAME_WIDTH, etc.
    void release();
    
private:
    AVFormatContext* format_ctx_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVPacket* packet_;
    int video_stream_idx_;
    
    // Thread-safe prefetching
    std::thread reader_thread_;
    std::queue<cv::Mat> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
};
```

## Migration Path

1. **Phase 1**: Implement FFmpeg reader alongside OpenCV (ifdef switch)
2. **Phase 2**: Test and benchmark both implementations
3. **Phase 3**: Replace OpenCV with FFmpeg if performance is better
4. **Phase 4**: Remove OpenCV VideoCapture dependency

## Dependencies

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED
    libavformat
    libavcodec
    libavutil
    libswscale
)
```

Or using system libraries:
```bash
sudo apt-get install libavformat-dev libavcodec-dev libavutil-dev libswscale-dev
```

## Expected Performance

**Current (OpenCV blocking)**:
- Frame read: 0.616 ms
- Total pipeline: 3.524 ms/frame → 283 FPS

**With FFmpeg prefetching**:
- Frame read: ~0.0 ms (overlapped)
- Total pipeline: ~2.908 ms/frame → ~344 FPS
- **Improvement: +61 FPS (+21.5%)**

## Next Steps

1. ✅ Create FFmpegVideoReader class
2. ✅ Implement basic frame reading
3. ✅ Add thread-safe prefetching
4. ✅ Add OpenCV Mat conversion
5. ✅ Integrate into video_visualize_fixed.cu
6. ✅ Benchmark and compare with OpenCV version

