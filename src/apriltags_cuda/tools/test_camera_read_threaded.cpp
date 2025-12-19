// Optimized camera reading program with two threads: read and display
// Reads at 1280x1024 @ 160 FPS with separate threads for reading and display

#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

using namespace cv;
using namespace std;
using namespace chrono;

const int TARGET_WIDTH = 1280;
const int TARGET_HEIGHT = 1024;
const int TARGET_FPS = 160;

// Frame data structure for queue
struct FrameData {
  Mat frame;
  steady_clock::time_point timestamp;
  int frame_number;
  double reading_fps;
};

// Thread-safe queue
template<typename T>
class ThreadSafeQueue {
public:
  void push(const T& item) {
    lock_guard<mutex> lock(mutex_);
    queue_.push(item);
    condition_.notify_one();
  }
  
  bool try_pop(T& item, int timeout_ms = 0) {
    unique_lock<mutex> lock(mutex_);
    if (timeout_ms > 0) {
      if (!condition_.wait_for(lock, milliseconds(timeout_ms), [this] { return !queue_.empty(); })) {
        return false;
      }
    } else {
      condition_.wait(lock, [this] { return !queue_.empty(); });
    }
    item = queue_.front();
    queue_.pop();
    return true;
  }
  
  size_t size() const {
    lock_guard<mutex> lock(mutex_);
    return queue_.size();
  }
  
  void clear() {
    lock_guard<mutex> lock(mutex_);
    while (!queue_.empty()) {
      queue_.pop();
    }
  }

private:
  mutable mutex mutex_;
  queue<T> queue_;
  condition_variable condition_;
};

// Global variables
ThreadSafeQueue<FrameData> frame_queue;
atomic<bool> running{true};
atomic<int> frame_count{0};
atomic<int> display_count{0};
const size_t MAX_QUEUE_SIZE = 5;  // Limit queue size to prevent memory buildup

// Statistics
vector<steady_clock::time_point> read_times;
mutex read_times_mutex;

void ProcessFrameStats(steady_clock::time_point read_time) {
  lock_guard<mutex> lock(read_times_mutex);
  read_times.push_back(read_time);
  if (read_times.size() > 1000) {
    read_times.erase(read_times.begin());
  }
}

void ReaderThread(CameraHandle camera_handle, unsigned char* rgb_buffer, 
                  bool is_mono, steady_clock::time_point start_time) {
  int local_frame_count = 0;
  vector<steady_clock::time_point> local_read_times;
  local_read_times.reserve(1000);
  
  // Target frame interval for 160 FPS (6.25ms)
  const auto target_frame_interval = microseconds(static_cast<int>(1000000.0 / TARGET_FPS));
  auto last_frame_time = steady_clock::now();
  
  while (running.load()) {
    tSdkFrameHead frame_info;
    BYTE *pby_buffer = nullptr;
    
    auto read_start = steady_clock::now();
    
    // Get frame with timeout
    CameraSdkStatus status = CameraGetImageBuffer(camera_handle, &frame_info, &pby_buffer, 100);
    
    if (status != CAMERA_STATUS_SUCCESS) {
      if (status == CAMERA_STATUS_TIME_OUT) {
        continue;
      } else {
        cerr << "Error: CameraGetImageBuffer failed with status " << status << endl;
        break;
      }
    }
    
    auto read_end = steady_clock::now();
    
    // Verify frame info
    if (frame_info.iWidth != TARGET_WIDTH || frame_info.iHeight != TARGET_HEIGHT) {
      CameraReleaseImageBuffer(camera_handle, pby_buffer);
      continue;
    }
    
    // Convert frame to Mat
    Mat frame;
    if (is_mono) {
      // Direct grayscale - create Mat from buffer data (copy)
      frame = Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC1, pby_buffer).clone();
    } else {
      // Process color if needed
      unsigned char* rgb_buffer_display = rgb_buffer ? rgb_buffer : (unsigned char*)pby_buffer;
      if (frame_info.uiMediaType != CAMERA_MEDIA_TYPE_BGR8 && 
          frame_info.uiMediaType != CAMERA_MEDIA_TYPE_RGB8) {
        CameraImageProcess(camera_handle, pby_buffer, rgb_buffer_display, &frame_info);
      }
      frame = Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC3, rgb_buffer_display).clone();
    }
    
    // Calculate reading FPS
    local_read_times.push_back(read_end);
    if (local_read_times.size() > 1000) {
      local_read_times.erase(local_read_times.begin());
    }
    double reading_fps = 0.0;
    if (local_read_times.size() >= 2) {
      auto total_duration = duration<double>(local_read_times.back() - local_read_times.front()).count();
      if (total_duration > 0.0) {
        reading_fps = (local_read_times.size() - 1) / total_duration;
      }
    }
    
    // Release buffer immediately after copying frame data
    CameraReleaseImageBuffer(camera_handle, pby_buffer);
    
    // Store statistics
    local_frame_count++;
    frame_count.store(local_frame_count);
    ProcessFrameStats(read_end);
    
    // Prepare frame data
    FrameData frame_data;
    frame_data.frame = frame;
    frame_data.timestamp = read_end;
    frame_data.frame_number = local_frame_count;
    frame_data.reading_fps = reading_fps;
    
    // Add to queue (drop oldest if queue is full)
    if (frame_queue.size() >= MAX_QUEUE_SIZE) {
      FrameData dummy;
      frame_queue.try_pop(dummy, 0);  // Remove oldest frame
    }
    frame_queue.push(frame_data);
    
    // Throttle to target FPS if reading too fast
    auto current_time = steady_clock::now();
    auto elapsed = duration_cast<microseconds>(current_time - last_frame_time);
    if (elapsed < target_frame_interval) {
      this_thread::sleep_for(target_frame_interval - elapsed);
    }
    last_frame_time = steady_clock::now();
  }
}

void DisplayThread(steady_clock::time_point start_time) {
  namedWindow("MindVision Camera @ 1280x1024", WINDOW_AUTOSIZE);
  FrameData frame_data;
  int last_displayed_frame = 0;
  const int STATS_INTERVAL = 100;
  
  while (running.load()) {
    // Try to get frame from queue (with timeout to check running flag)
    if (!frame_queue.try_pop(frame_data, 10)) {
      continue;  // Timeout - check running flag and try again
    }
    
    display_count++;
    
    // Calculate display FPS
    auto elapsed = duration<double>(steady_clock::now() - start_time).count();
    double display_fps = display_count.load() / elapsed;
    
    // Add FPS text overlay
    Mat display_frame = frame_data.frame;
    string fps_text = "Read FPS: " + to_string(static_cast<int>(frame_data.reading_fps));
    string display_fps_text = "Display FPS: " + to_string(static_cast<int>(display_fps));
    string frame_text = "Frame: " + to_string(frame_data.frame_number);
    
    Scalar text_color = (display_frame.channels() == 1) ? Scalar(255) : Scalar(0, 255, 0);
    putText(display_frame, fps_text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    putText(display_frame, display_fps_text, Point(10, 70), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    putText(display_frame, frame_text, Point(10, 110), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    
    // Display frame
    imshow("MindVision Camera @ 1280x1024", display_frame);
    int key = waitKey(1) & 0xFF;
    if (key == 27 || key == 'q') {  // ESC or 'q' to quit
      running.store(false);
      break;
    }
    
    // Print statistics periodically
    if (frame_data.frame_number % STATS_INTERVAL == 0 && frame_data.frame_number > last_displayed_frame) {
      last_displayed_frame = frame_data.frame_number;
      cout << fixed << setprecision(2);
      cout << "Frame " << frame_data.frame_number 
           << " | Read FPS: " << frame_data.reading_fps
           << " | Display FPS: " << display_fps
           << " | Queue size: " << frame_queue.size() << endl;
    }
  }
  
  destroyAllWindows();
}

int main(int argc, char *argv[]) {
  cout << "=== Threaded MindVision Camera Reader ===" << endl;
  cout << "Target: " << TARGET_WIDTH << "x" << TARGET_HEIGHT << " @ " << TARGET_FPS << " FPS\n" << endl;
  
  // Step 1: Initialize SDK
  cout << "Initializing MindVision SDK..." << endl;
  CameraSdkStatus status = CameraSdkInit(1);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Error: CameraSdkInit failed with status " << status << endl;
    return 1;
  }
  
  // Step 2: Enumerate cameras
  tSdkCameraDevInfo camera_list[16];
  INT camera_count = 16;
  status = CameraEnumerateDevice(camera_list, &camera_count);
  
  if (status != CAMERA_STATUS_SUCCESS || camera_count <= 0) {
    cerr << "Error: No MindVision cameras found!" << endl;
    return 1;
  }
  
  cout << "Found " << camera_count << " MindVision camera(s)" << endl;
  for (int i = 0; i < camera_count; ++i) {
    string name = camera_list[i].acFriendlyName[0] 
                  ? camera_list[i].acFriendlyName 
                  : camera_list[i].acProductName;
    cout << "  [" << i << "] " << name << endl;
  }
  
  // Step 3: Open camera
  int cam_index = 0;
  if (argc > 1) {
    cam_index = atoi(argv[1]);
  }
  
  if (cam_index < 0 || cam_index >= camera_count) {
    cerr << "Error: Invalid camera index " << cam_index << endl;
    return 1;
  }
  
  CameraHandle camera_handle = 0;
  tSdkCameraDevInfo dev_info = camera_list[cam_index];
  status = CameraInit(&dev_info, -1, -1, &camera_handle);
  
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Error: CameraInit failed with status " << status << endl;
    return 1;
  }
  
  cout << "Camera initialized successfully" << endl;
  
  // Step 4: Get camera capability
  tSdkCameraCapbility capability;
  CameraGetCapability(camera_handle, &capability);
  
  cout << "Camera max resolution: " << capability.sResolutionRange.iWidthMax 
       << "x" << capability.sResolutionRange.iHeightMax << endl;
  
  // Allocate RGB buffer for processing if needed
  unsigned char *rgb_buffer = nullptr;
  int max_width = capability.sResolutionRange.iWidthMax;
  int max_height = capability.sResolutionRange.iHeightMax;
  bool is_mono = capability.sIspCapacity.bMonoSensor;
  if (!is_mono) {
    rgb_buffer = static_cast<unsigned char *>(
        malloc(static_cast<size_t>(max_width) * static_cast<size_t>(max_height) * 3));
  }
  
  // Step 5: Set output format
  if (is_mono) {
    CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_MONO8);
    cout << "Set output format to MONO8" << endl;
  } else {
    CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_BGR8);
    cout << "Set output format to BGR8" << endl;
  }
  
  // Step 6: Set resolution
  cout << "Setting resolution to " << TARGET_WIDTH << "x" << TARGET_HEIGHT << "..." << endl;
  status = CameraSetImageResolutionEx(camera_handle, 0xff, 0, 0, 0, 0,
                                      TARGET_WIDTH, TARGET_HEIGHT, 0, 0);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Warning: CameraSetImageResolutionEx returned status " << status << endl;
  } else {
    cout << "Resolution set successfully" << endl;
  }
  
  // Step 7: Set frame speed
  int max_frame_speed = capability.iFrameSpeedDesc - 1;
  if (max_frame_speed < 0) max_frame_speed = 0;
  if (max_frame_speed > FRAME_SPEED_SUPER) max_frame_speed = FRAME_SPEED_SUPER;
  cout << "Setting frame speed to index " << max_frame_speed << "..." << endl;
  CameraSetFrameSpeed(camera_handle, max_frame_speed);
  
  // Step 8: Check and set trigger mode to continuous
  int trigger_mode = -1;
  CameraGetTriggerMode(camera_handle, &trigger_mode);
  if (trigger_mode != 0) {
    CameraSetTriggerMode(camera_handle, 0);  // 0 = continuous mode
  }
  
  // Step 9: Disable auto exposure and set exposure for 160 FPS
  BOOL ae_state = FALSE;
  CameraGetAeState(camera_handle, &ae_state);
  if (ae_state) {
    cout << "Disabling auto exposure..." << endl;
    CameraSetAeState(camera_handle, FALSE);
  }
  
  // Set exposure time for 160 FPS (should be < 6.25ms for 160 FPS)
  // Use slightly longer exposure to achieve closer to 160 FPS
  double target_exposure_us = 5500.0;  // 5.5ms exposure to get closer to 160 FPS
  double current_exposure = 0.0;
  CameraGetExposureTime(camera_handle, &current_exposure);
  cout << "Setting exposure time to " << target_exposure_us << " microseconds..." << endl;
  CameraSetExposureTime(camera_handle, target_exposure_us);
  CameraGetExposureTime(camera_handle, &current_exposure);
  cout << "Actual exposure time: " << current_exposure << " microseconds" << endl;
  
  // Step 10: Start camera streaming
  status = CameraPlay(camera_handle);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Error: CameraPlay failed with status " << status << endl;
    CameraUnInit(camera_handle);
    if (rgb_buffer) free(rgb_buffer);
    return 1;
  }
  
  cout << "Camera streaming started" << endl;
  cout << "Waiting for stream to stabilize..." << endl;
  this_thread::sleep_for(milliseconds(500));
  
  // Step 11: Start threads
  cout << "\n=== Starting threaded frame capture ===" << endl;
  cout << "Press ESC or 'q' to stop\n" << endl;
  
  read_times.reserve(2000);
  auto start_time = steady_clock::now();
  
  thread reader_thread(ReaderThread, camera_handle, rgb_buffer, is_mono, start_time);
  thread display_thread(DisplayThread, start_time);
  
  // Wait for threads
  reader_thread.join();
  display_thread.join();
  
  // Final statistics
  auto end_time = steady_clock::now();
  auto total_duration = duration<double>(end_time - start_time).count();
  int final_frame_count = frame_count.load();
  int final_display_count = display_count.load();
  double final_read_fps = final_frame_count / total_duration;
  double final_display_fps = final_display_count / total_duration;
  
  cout << "\n=== Final Statistics ===" << endl;
  cout << "Total frames read: " << final_frame_count << endl;
  cout << "Total frames displayed: " << final_display_count << endl;
  cout << "Total time: " << fixed << setprecision(2) << total_duration << " seconds" << endl;
  cout << "Average read FPS: " << final_read_fps << " (target: " << TARGET_FPS << ")" << endl;
  cout << "Average display FPS: " << final_display_fps << endl;
  
  // Cleanup
  CameraStop(camera_handle);
  CameraUnInit(camera_handle);
  if (rgb_buffer) {
    free(rgb_buffer);
  }
  
  cout << "\nCamera closed successfully" << endl;
  return 0;
}
