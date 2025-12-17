#ifndef FFMPEG_VIDEO_READER_H
#define FFMPEG_VIDEO_READER_H

#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <memory>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "opencv2/opencv.hpp"

namespace frc971 {
namespace apriltag {

// Thread-safe FFmpeg-based video reader with frame prefetching
class FFmpegVideoReader {
public:
    FFmpegVideoReader();
    ~FFmpegVideoReader();
    
    // Open video file
    bool open(const std::string& video_path);
    
    // Read next frame (non-blocking if prefetching enabled)
    bool read(cv::Mat& frame);
    
    // Get video property (similar to VideoCapture::get)
    double get(int prop_id);
    
    // Release resources
    void release();
    
    // Check if video is opened
    bool isOpened() const { return format_ctx_ != nullptr; }
    
    // Check if input is grayscale (true) or color (false)
    bool isInputGrayscale() const { 
        return (input_pix_fmt_ == AV_PIX_FMT_GRAY8 || 
                input_pix_fmt_ == AV_PIX_FMT_GRAY16LE ||
                input_pix_fmt_ == AV_PIX_FMT_GRAY16BE);
    }
    
    // Enable/disable prefetching (default: enabled)
    void setPrefetching(bool enable);
    
    // Set queue size for prefetching (default: 4)
    void setQueueSize(size_t queue_size);

private:
    // FFmpeg structures
    AVFormatContext* format_ctx_;
    AVCodecContext* codec_ctx_;
    AVFrame* frame_;
    AVFrame* frame_gray_;  // Grayscale frame for detector (always output grayscale)
    AVPacket* packet_;
    SwsContext* sws_ctx_;
    uint8_t* gray_buffer_;  // Store buffer pointer for cleanup
    AVPixelFormat input_pix_fmt_;  // Input pixel format (may be grayscale or color)
    
    int video_stream_idx_;
    int width_;
    int height_;
    double fps_;
    int64_t total_frames_;
    
    // Thread-safe prefetching
    std::thread reader_thread_;
    std::queue<cv::Mat> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
    std::atomic<bool> prefetching_enabled_;
    size_t max_queue_size_;
    
    // Internal methods
    void readerThreadFunc();
    bool decodeFrame();
    cv::Mat avFrameToMat(AVFrame* frame);
    void cleanup();
};

}  // namespace apriltag
}  // namespace frc971

// Note: Use OpenCV's CAP_PROP_* enum values directly (included via opencv.hpp)

#endif  // FFMPEG_VIDEO_READER_H

