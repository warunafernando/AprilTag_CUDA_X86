#include "ffmpeg_video_reader.h"
#include <iostream>
#include <cmath>
#include <cstring>

namespace frc971 {
namespace apriltag {

FFmpegVideoReader::FFmpegVideoReader()
    : format_ctx_(nullptr)
    , codec_ctx_(nullptr)
    , frame_(nullptr)
    , frame_gray_(nullptr)
    , packet_(nullptr)
    , sws_ctx_(nullptr)
    , gray_buffer_(nullptr)
    , input_pix_fmt_(AV_PIX_FMT_NONE)
    , video_stream_idx_(-1)
    , width_(0)
    , height_(0)
    , fps_(0.0)
    , total_frames_(0)
    , running_(false)
    , prefetching_enabled_(true)  // Enabled for better performance
    , max_queue_size_(4)  // Keep 4 frames ahead for better I/O overlap
{
}

FFmpegVideoReader::~FFmpegVideoReader() {
    release();
}

bool FFmpegVideoReader::open(const std::string& video_path) {
    // Clean up any existing resources
    release();
    
    // Allocate format context
    format_ctx_ = avformat_alloc_context();
    if (!format_ctx_) {
        std::cerr << "FFmpegVideoReader: Failed to allocate format context" << std::endl;
        return false;
    }
    
    // Open input file
    if (avformat_open_input(&format_ctx_, video_path.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "FFmpegVideoReader: Failed to open video: " << video_path << std::endl;
        cleanup();
        return false;
    }
    
    // Find stream info
    if (avformat_find_stream_info(format_ctx_, nullptr) < 0) {
        std::cerr << "FFmpegVideoReader: Failed to find stream info" << std::endl;
        cleanup();
        return false;
    }
    
    // Find video stream
    video_stream_idx_ = -1;
    for (unsigned int i = 0; i < format_ctx_->nb_streams; i++) {
        if (format_ctx_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_idx_ = i;
            break;
        }
    }
    
    if (video_stream_idx_ == -1) {
        std::cerr << "FFmpegVideoReader: No video stream found" << std::endl;
        cleanup();
        return false;
    }
    
    // Get codec parameters
    AVCodecParameters* codecpar = format_ctx_->streams[video_stream_idx_]->codecpar;
    
    // Find decoder
    const AVCodec* codec = avcodec_find_decoder(codecpar->codec_id);
    if (!codec) {
        std::cerr << "FFmpegVideoReader: Unsupported codec" << std::endl;
        cleanup();
        return false;
    }
    
    // Allocate codec context
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        std::cerr << "FFmpegVideoReader: Failed to allocate codec context" << std::endl;
        cleanup();
        return false;
    }
    
    // Copy codec parameters to context
    if (avcodec_parameters_to_context(codec_ctx_, codecpar) < 0) {
        std::cerr << "FFmpegVideoReader: Failed to copy codec parameters" << std::endl;
        cleanup();
        return false;
    }
    
    // Open codec
    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
        std::cerr << "FFmpegVideoReader: Failed to open codec" << std::endl;
        cleanup();
        return false;
    }
    
    // Get video properties
    width_ = codec_ctx_->width;
    height_ = codec_ctx_->height;
    input_pix_fmt_ = codec_ctx_->pix_fmt;
    
    // Calculate FPS
    AVRational fps_rational = format_ctx_->streams[video_stream_idx_]->avg_frame_rate;
    if (fps_rational.num > 0 && fps_rational.den > 0) {
        fps_ = static_cast<double>(fps_rational.num) / fps_rational.den;
    } else {
        // Fallback: use r_frame_rate
        fps_rational = format_ctx_->streams[video_stream_idx_]->r_frame_rate;
        if (fps_rational.num > 0 && fps_rational.den > 0) {
            fps_ = static_cast<double>(fps_rational.num) / fps_rational.den;
        } else {
            fps_ = 30.0;  // Default
        }
    }
    
    // Get total frame count
    total_frames_ = format_ctx_->streams[video_stream_idx_]->nb_frames;
    if (total_frames_ <= 0) {
        // Estimate from duration
        int64_t duration = format_ctx_->streams[video_stream_idx_]->duration;
        AVRational time_base = format_ctx_->streams[video_stream_idx_]->time_base;
        if (duration > 0 && time_base.num > 0) {
            double duration_sec = static_cast<double>(duration) * time_base.num / time_base.den;
            total_frames_ = static_cast<int64_t>(duration_sec * fps_ + 0.5);
        }
    }
    
    // Allocate frame and packet
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    if (!frame_ || !packet_) {
        std::cerr << "FFmpegVideoReader: Failed to allocate frame/packet" << std::endl;
        cleanup();
        return false;
    }
    
    // Determine if input is already grayscale or effectively grayscale (YUV420P)
    // YUV420P is commonly used for grayscale videos (U and V planes are constant/empty)
    bool is_grayscale = (input_pix_fmt_ == AV_PIX_FMT_GRAY8 || 
                        input_pix_fmt_ == AV_PIX_FMT_GRAY16LE ||
                        input_pix_fmt_ == AV_PIX_FMT_GRAY16BE ||
                        // AV_PIX_FMT_YUV400P not available in this FFmpeg version, using YUV420P check instead
                        // input_pix_fmt_ == AV_PIX_FMT_YUV400P ||
                        input_pix_fmt_ == AV_PIX_FMT_YUV420P); // YUV420P: extract Y plane directly
    
    // Allocate grayscale frame for detector (always output grayscale)
    frame_gray_ = av_frame_alloc();
    if (!frame_gray_) {
        std::cerr << "FFmpegVideoReader: Failed to allocate grayscale frame" << std::endl;
        cleanup();
        return false;
    }
    
    int num_bytes = av_image_get_buffer_size(AV_PIX_FMT_GRAY8, width_, height_, 1);
    gray_buffer_ = static_cast<uint8_t*>(av_malloc(num_bytes * sizeof(uint8_t)));
    if (!gray_buffer_) {
        std::cerr << "FFmpegVideoReader: Failed to allocate grayscale buffer" << std::endl;
        cleanup();
        return false;
    }
    int ret = av_image_fill_arrays(frame_gray_->data, frame_gray_->linesize, gray_buffer_,
                                   AV_PIX_FMT_GRAY8, width_, height_, 1);
    if (ret < 0) {
        std::cerr << "FFmpegVideoReader: Failed to fill image arrays" << std::endl;
        av_freep(&gray_buffer_);
        cleanup();
        return false;
    }
    
    // Initialize SWS context for format conversion (to grayscale)
    // YUV420P: Extract Y plane directly (much faster, no SWS needed)
    // True grayscale: Copy directly (no SWS needed)
    // Color: Need SWS for conversion
    if (is_grayscale || input_pix_fmt_ == AV_PIX_FMT_YUV420P) {
        // Input is grayscale or YUV420P - no SWS context needed
        // YUV420P: Will extract Y plane directly (fast memcpy)
        // True grayscale: Will copy directly
        sws_ctx_ = nullptr;
        if (input_pix_fmt_ == AV_PIX_FMT_YUV420P) {
            std::cout << "FFmpegVideoReader: Input is YUV420P (will extract Y plane directly)" << std::endl;
        } else {
            std::cout << "FFmpegVideoReader: Input is grayscale (no conversion needed)" << std::endl;
        }
    } else {
        // Input is color - need SWS context to convert to grayscale
        sws_ctx_ = sws_getContext(width_, height_, codec_ctx_->pix_fmt,
                                 width_, height_, AV_PIX_FMT_GRAY8,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!sws_ctx_) {
            std::cerr << "FFmpegVideoReader: Failed to create SWS context" << std::endl;
            cleanup();
            return false;
        }
        std::cout << "FFmpegVideoReader: Input is color (will convert to grayscale)" << std::endl;
    }
    
    // Start prefetching thread if enabled (disabled for initial testing)
    // TODO: Re-enable prefetching once synchronous reading is verified
    if (prefetching_enabled_) {
        running_ = true;
        reader_thread_ = std::thread(&FFmpegVideoReader::readerThreadFunc, this);
    }
    
    return true;
}

bool FFmpegVideoReader::read(cv::Mat& frame) {
    if (!isOpened()) {
        return false;
    }
    
    if (prefetching_enabled_) {
        // Get frame from prefetch queue
        std::unique_lock<std::mutex> lock(queue_mutex_);
        cv_.wait(lock, [this]() { return !frame_queue_.empty() || !running_; });
        
        if (frame_queue_.empty() && !running_) {
            return false;  // End of video
        }
        
        if (!frame_queue_.empty()) {
            frame = frame_queue_.front();
            frame_queue_.pop();
            cv_.notify_one();  // Notify reader thread that space is available
            return true;
        }
        
        return false;
    } else {
        // Synchronous reading (no prefetching)
        if (decodeFrame()) {
            frame = avFrameToMat(frame_gray_);
            return !frame.empty();
        }
        return false;
    }
}

double FFmpegVideoReader::get(int prop_id) {
    if (!isOpened()) {
        return 0.0;
    }
    
    // Use OpenCV's CAP_PROP_* constants (they're enum values, same as int)
    switch (prop_id) {
        case cv::CAP_PROP_FRAME_WIDTH:
            return static_cast<double>(width_);
        case cv::CAP_PROP_FRAME_HEIGHT:
            return static_cast<double>(height_);
        case cv::CAP_PROP_FPS:
            return fps_;
        case cv::CAP_PROP_FRAME_COUNT:
            return static_cast<double>(total_frames_);
        case cv::CAP_PROP_POS_FRAMES:
            // TODO: Track current frame position
            return 0.0;
        default:
            return 0.0;
    }
}

void FFmpegVideoReader::release() {
    // Stop prefetching thread
    if (reader_thread_.joinable()) {
        running_ = false;
        cv_.notify_all();
        reader_thread_.join();
    }
    
    // Clear queue
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!frame_queue_.empty()) {
            frame_queue_.pop();
        }
    }
    
    cleanup();
}

void FFmpegVideoReader::setPrefetching(bool enable) {
    if (enable == prefetching_enabled_) {
        return;  // Already in desired state
    }
    
    prefetching_enabled_ = enable;
    
    if (enable && isOpened() && !reader_thread_.joinable()) {
        // Start prefetching thread
        running_ = true;
        reader_thread_ = std::thread(&FFmpegVideoReader::readerThreadFunc, this);
    } else if (!enable && reader_thread_.joinable()) {
        // Stop prefetching thread
        running_ = false;
        cv_.notify_all();
        reader_thread_.join();
        
        // Clear queue
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!frame_queue_.empty()) {
            frame_queue_.pop();
        }
    }
}

void FFmpegVideoReader::setQueueSize(size_t queue_size) {
    max_queue_size_ = queue_size;
}

void FFmpegVideoReader::readerThreadFunc() {
    while (running_) {
        // Check if queue is full
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            if (frame_queue_.size() >= max_queue_size_) {
                // Wait until space is available
                cv_.wait(lock, [this]() {
                    return frame_queue_.size() < max_queue_size_ || !running_;
                });
            }
        }
        
        if (!running_) {
            break;
        }
        
        // Decode frame
        if (decodeFrame()) {
            cv::Mat frame = avFrameToMat(frame_gray_);
            
            // Add to queue
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (frame_queue_.size() < max_queue_size_) {
                    frame_queue_.push(frame);
                    cv_.notify_one();
                }
            }
        } else {
            // End of video
            break;
        }
    }
    
    // Signal end of video
    running_ = false;
    cv_.notify_all();
}

bool FFmpegVideoReader::decodeFrame() {
    if (!format_ctx_ || !codec_ctx_ || !frame_ || !frame_gray_ || !packet_) {
        std::cerr << "FFmpegVideoReader::decodeFrame: Invalid state - missing required structures" << std::endl;
        return false;
    }
    
    // Determine if input is already grayscale
    bool is_grayscale = (input_pix_fmt_ == AV_PIX_FMT_GRAY8 || 
                        input_pix_fmt_ == AV_PIX_FMT_GRAY16LE ||
                        input_pix_fmt_ == AV_PIX_FMT_GRAY16BE);
    
    while (av_read_frame(format_ctx_, packet_) >= 0) {
        if (packet_->stream_index == video_stream_idx_) {
            // Send packet to decoder
            int ret = avcodec_send_packet(codec_ctx_, packet_);
            if (ret < 0 && ret != AVERROR(EAGAIN)) {
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
                std::cerr << "FFmpegVideoReader::decodeFrame: avcodec_send_packet failed: " << errbuf << std::endl;
                av_packet_unref(packet_);
                continue;
            }
            
            // Receive frame from decoder
            ret = avcodec_receive_frame(codec_ctx_, frame_);
            av_packet_unref(packet_);
            
            if (ret == 0) {
                // Convert to grayscale if input is color, or copy directly if already grayscale
                bool is_grayscale_now = (input_pix_fmt_ == AV_PIX_FMT_GRAY8 || 
                                        input_pix_fmt_ == AV_PIX_FMT_GRAY16LE ||
                                        input_pix_fmt_ == AV_PIX_FMT_GRAY16BE ||
                                        // AV_PIX_FMT_YUV400P not available in this FFmpeg version
                                        false); // YUV400P check removed
                
                // Special case: YUV420P - extract Y plane directly (much faster for grayscale videos)
                if (input_pix_fmt_ == AV_PIX_FMT_YUV420P) {
                    // YUV420P: Y plane is frame_->data[0], U is data[1], V is data[2]
                    // For grayscale, just copy Y plane directly (no conversion needed)
                    if (!frame_gray_->data[0] || !frame_->data[0]) {
                        std::cerr << "FFmpegVideoReader::decodeFrame: frame data is null" << std::endl;
                        return false;
                    }
                    // Copy Y plane directly - it's already grayscale!
                    // frame_->data[0] contains the Y (luma) plane at full resolution
                    int y_linesize = frame_->linesize[0];
                    int gray_linesize = frame_gray_->linesize[0];
                    uint8_t* src = frame_->data[0];
                    uint8_t* dst = frame_gray_->data[0];
                    
                    // Copy row by row (linesize might differ due to alignment)
                    for (int y = 0; y < height_; y++) {
                        memcpy(dst + y * gray_linesize, src + y * y_linesize, width_);
                    }
                } else if (is_grayscale_now) {
                    // Input is already grayscale - copy directly
                    if (!frame_gray_->data[0] || !frame_->data[0]) {
                        std::cerr << "FFmpegVideoReader::decodeFrame: frame data is null" << std::endl;
                        return false;
                    }
                    // Copy frame data to our grayscale buffer
                    av_image_copy(frame_gray_->data, frame_gray_->linesize,
                                 (const uint8_t**)frame_->data, frame_->linesize,
                                 AV_PIX_FMT_GRAY8, width_, height_);
                } else {
                    // Input is color - convert to grayscale using SWS
                    if (!sws_ctx_) {
                        std::cerr << "FFmpegVideoReader::decodeFrame: SWS context is null but input is color" << std::endl;
                        return false;
                    }
                    if (!frame_gray_->data[0]) {
                        std::cerr << "FFmpegVideoReader::decodeFrame: frame_gray_->data[0] is null" << std::endl;
                        return false;
                    }
                    sws_scale(sws_ctx_,
                             frame_->data, frame_->linesize, 0, height_,
                             frame_gray_->data, frame_gray_->linesize);
                }
                return true;
            } else if (ret == AVERROR(EAGAIN)) {
                // Need more packets - continue loop
                continue;
            } else if (ret == AVERROR_EOF) {
                // End of stream
                return false;
            } else {
                // Other error
                char errbuf[AV_ERROR_MAX_STRING_SIZE];
                av_strerror(ret, errbuf, AV_ERROR_MAX_STRING_SIZE);
                std::cerr << "FFmpegVideoReader::decodeFrame: avcodec_receive_frame error: " << errbuf << std::endl;
                return false;
            }
        } else {
            av_packet_unref(packet_);
        }
    }
    
    // End of file reached - try to flush decoder (may return a final frame)
    int ret = avcodec_send_packet(codec_ctx_, nullptr);
    if (ret < 0 && ret != AVERROR_EOF) {
        return false;
    }
    
    ret = avcodec_receive_frame(codec_ctx_, frame_);
    if (ret == 0) {
        // Determine if input is already grayscale
        bool is_grayscale = (input_pix_fmt_ == AV_PIX_FMT_GRAY8 || 
                            input_pix_fmt_ == AV_PIX_FMT_GRAY16LE ||
                            input_pix_fmt_ == AV_PIX_FMT_GRAY16BE);
        
        if (is_grayscale) {
            // Input is already grayscale - copy directly (no conversion needed)
            if (!frame_gray_->data[0] || !frame_->data[0]) {
                return false;
            }
            if (input_pix_fmt_ == AV_PIX_FMT_GRAY8) {
                // Direct copy - same format
                av_image_copy(frame_gray_->data, frame_gray_->linesize,
                             (const uint8_t**)frame_->data, frame_->linesize,
                             AV_PIX_FMT_GRAY8, width_, height_);
            } else {
                // GRAY16 not supported
                return false;
            }
        } else {
            // Input is color - convert to grayscale using SWS
            if (!sws_ctx_) {
                return false;
            }
            if (!frame_gray_->data[0]) {
                return false;
            }
            sws_scale(sws_ctx_,
                     frame_->data, frame_->linesize, 0, height_,
                     frame_gray_->data, frame_gray_->linesize);
        }
        return true;
    }
    
    return false;
}

cv::Mat FFmpegVideoReader::avFrameToMat(AVFrame* frame) {
    if (!frame || !frame->data[0]) {
        std::cerr << "FFmpegVideoReader::avFrameToMat: Invalid frame or data" << std::endl;
        return cv::Mat();
    }
    
    // Create OpenCV Mat from AVFrame data (grayscale format)
    // Note: linesize[0] is the stride (width in bytes, may be padded)
    if (frame->linesize[0] < width_) {
        std::cerr << "FFmpegVideoReader::avFrameToMat: Invalid linesize " << frame->linesize[0] << " < width " << width_ << std::endl;
        return cv::Mat();
    }
    
    // If stride equals width, data is already contiguous - create Mat directly
    if (frame->linesize[0] == width_) {
        // Data is contiguous, create Mat directly and copy
        cv::Mat mat(height_, width_, CV_8UC1, frame->data[0]);
        return mat.clone();  // Clone to ensure ownership and CUDA compatibility
    } else {
        // Stride is larger than width (padding), need to handle stride
        // Create Mat with stride first, then clone to remove padding
        cv::Mat mat_with_stride(height_, width_, CV_8UC1, frame->data[0], static_cast<size_t>(frame->linesize[0]));
        return mat_with_stride.clone();  // Clone removes stride, makes contiguous
    }
    
    // Fallback (should never reach here)
    return cv::Mat();
}

void FFmpegVideoReader::cleanup() {
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }
    
    if (gray_buffer_) {
        av_freep(&gray_buffer_);
        gray_buffer_ = nullptr;
    }
    
    if (frame_gray_) {
        av_frame_free(&frame_gray_);
        frame_gray_ = nullptr;
    }
    
    if (frame_) {
        av_frame_free(&frame_);
        frame_ = nullptr;
    }
    
    if (packet_) {
        av_packet_free(&packet_);
        packet_ = nullptr;
    }
    
    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
        codec_ctx_ = nullptr;
    }
    
    if (format_ctx_) {
        avformat_close_input(&format_ctx_);
        format_ctx_ = nullptr;
    }
    
    video_stream_idx_ = -1;
    width_ = 0;
    height_ = 0;
    fps_ = 0.0;
    total_frames_ = 0;
}

}  // namespace apriltag
}  // namespace frc971

