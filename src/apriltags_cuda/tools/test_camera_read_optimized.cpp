// Optimized camera reading program based on SDK best practices
// Reads at 1280x1024 @ 211 FPS with minimal processing

#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <thread>
#include <atomic>

using namespace cv;
using namespace std;
using namespace chrono;

const int TARGET_WIDTH = 1280;
const int TARGET_HEIGHT = 1024;
const int TARGET_FPS = 211;

// Statistics
atomic<int> frame_count{0};
vector<steady_clock::time_point> read_times;
mutex read_times_mutex;

void ProcessFrameStats(steady_clock::time_point read_time) {
  lock_guard<mutex> lock(read_times_mutex);
  read_times.push_back(read_time);
  if (read_times.size() > 1000) {
    read_times.erase(read_times.begin());
  }
}

int main(int argc, char *argv[]) {
  cout << "=== Optimized MindVision Camera Reader ===" << endl;
  cout << "Target: " << TARGET_WIDTH << "x" << TARGET_HEIGHT << " @ " << TARGET_FPS << " FPS\n" << endl;
  
  // Step 1: Initialize SDK (must be called first!)
  cout << "Initializing MindVision SDK..." << endl;
  CameraSdkStatus status = CameraSdkInit(1);  // 1 = Chinese, 0 = English
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
  
  // Allocate RGB buffer for processing if needed (for display)
  unsigned char *rgb_buffer = nullptr;
  int max_width = capability.sResolutionRange.iWidthMax;
  int max_height = capability.sResolutionRange.iHeightMax;
  if (!capability.sIspCapacity.bMonoSensor) {
    rgb_buffer = static_cast<unsigned char *>(
        malloc(static_cast<size_t>(max_width) * static_cast<size_t>(max_height) * 3));
  }
  
  // Step 5: Set output format (do this before setting resolution)
  if (capability.sIspCapacity.bMonoSensor) {
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
  
  // Verify resolution was set correctly
  tSdkImageResolution resolution_info;
  status = CameraGetImageResolution(camera_handle, &resolution_info);
  if (status == CAMERA_STATUS_SUCCESS) {
    cout << "Current resolution: " << resolution_info.iWidth << "x" << resolution_info.iHeight << endl;
  }
  
  // Step 7: Set frame speed - try HIGH first, then SUPER
  // Check capability for available frame speeds
  cout << "Available frame speed modes: " << capability.iFrameSpeedDesc << endl;
  
  // Camera only supports limited modes - use highest available
  int max_frame_speed = capability.iFrameSpeedDesc - 1;
  if (max_frame_speed < 0) max_frame_speed = 0;
  if (max_frame_speed > FRAME_SPEED_SUPER) max_frame_speed = FRAME_SPEED_SUPER;
  
  const char* speed_names[] = {"LOW", "NORMAL", "HIGH", "SUPER"};
  const char* target_speed_name = (max_frame_speed >= 0 && max_frame_speed < 4) 
                                   ? speed_names[max_frame_speed] : "UNKNOWN";
  
  cout << "Setting frame speed to " << target_speed_name 
       << " (index=" << max_frame_speed << ", max available=" << (capability.iFrameSpeedDesc-1) << ")..." << endl;
  status = CameraSetFrameSpeed(camera_handle, max_frame_speed);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Warning: CameraSetFrameSpeed returned status " << status << endl;
    // Fall back to NORMAL if HIGH/SUPER not supported
    cout << "Falling back to NORMAL mode..." << endl;
    CameraSetFrameSpeed(camera_handle, FRAME_SPEED_NORMAL);
  } else {
    cout << "Frame speed set successfully" << endl;
  }
  
  // Verify frame speed was set correctly
  int current_frame_speed = -1;
  status = CameraGetFrameSpeed(camera_handle, &current_frame_speed);
  if (status == CAMERA_STATUS_SUCCESS) {
    const char* speed_name = (current_frame_speed >= 0 && current_frame_speed < 4) 
                             ? speed_names[current_frame_speed] : "UNKNOWN";
    cout << "Current frame speed: index=" << current_frame_speed 
         << " (" << speed_name << ")" << endl;
  }
  
  cout << "\nNOTE: Camera reports " << capability.iFrameSpeedDesc 
       << " frame speed modes available. Maximum supported mode may be less than SUPER (211 FPS)." << endl;
  cout << "Actual FPS will depend on camera model and resolution settings." << endl;
  
  // Step 8: Check and set trigger mode to continuous (not triggered)
  int trigger_mode = -1;
  status = CameraGetTriggerMode(camera_handle, &trigger_mode);
  if (status == CAMERA_STATUS_SUCCESS) {
    cout << "Current trigger mode: " << trigger_mode << " (0=continuous)" << endl;
    if (trigger_mode != 0) {
      cout << "Setting trigger mode to continuous (0)..." << endl;
      status = CameraSetTriggerMode(camera_handle, 0);  // 0 = continuous mode
      if (status != CAMERA_STATUS_SUCCESS) {
        cerr << "Warning: Failed to set trigger mode to continuous" << endl;
      }
    }
  }
  
  // Step 9: Check and set exposure settings for maximum frame rate
  BOOL ae_state = FALSE;
  status = CameraGetAeState(camera_handle, &ae_state);
  if (status == CAMERA_STATUS_SUCCESS) {
    cout << "Auto exposure state: " << (ae_state ? "ON" : "OFF") << endl;
    if (ae_state) {
      cout << "Disabling auto exposure for maximum frame rate..." << endl;
      CameraSetAeState(camera_handle, FALSE);
    }
  }
  
  // Set a very short exposure time for high frame rates
  // For 211 FPS, exposure should be < 4.7ms (1/211 seconds)
  double target_exposure_us = 3000.0;  // 3ms exposure for high FPS
  double current_exposure = 0.0;
  status = CameraGetExposureTime(camera_handle, &current_exposure);
  if (status == CAMERA_STATUS_SUCCESS) {
    cout << "Current exposure time: " << current_exposure << " microseconds" << endl;
    if (current_exposure > target_exposure_us) {
      cout << "Setting exposure time to " << target_exposure_us << " microseconds for high FPS..." << endl;
      status = CameraSetExposureTime(camera_handle, target_exposure_us);
      if (status != CAMERA_STATUS_SUCCESS) {
        cerr << "Warning: Failed to set exposure time, status=" << status << endl;
      } else {
        cout << "Exposure time set successfully" << endl;
        // Verify
        CameraGetExposureTime(camera_handle, &current_exposure);
        cout << "Actual exposure time: " << current_exposure << " microseconds" << endl;
      }
    }
  }
  
  // Step 10: Start camera streaming
  status = CameraPlay(camera_handle);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Error: CameraPlay failed with status " << status << endl;
    CameraUnInit(camera_handle);
    return 1;
  }
  
  cout << "Camera streaming started" << endl;
  cout << "Waiting for stream to stabilize..." << endl;
  this_thread::sleep_for(milliseconds(500));  // Give camera time to start
  
  // Step 11: Read frames with optimized loop
  cout << "\n=== Starting frame read test ===" << endl;
  cout << "Reading frames with video display..." << endl;
  cout << "Press ESC or 'q' to stop\n" << endl;
  
  // Create display window
  namedWindow("MindVision Camera @ 1280x1024", WINDOW_AUTOSIZE);
  
  read_times.reserve(2000);
  auto start_time = steady_clock::now();
  const int STATS_INTERVAL = 100;
  int last_frame_count = 0;
  
  // Optimized reading loop: minimal processing, immediate release
  int timeout_count = 0;
  int dropped_frames = 0;
  
  while (true) {
    tSdkFrameHead frame_info;
    BYTE *pby_buffer = nullptr;
    
    auto read_start = steady_clock::now();
    
    // Use very short timeout (10ms) to avoid blocking - we want to catch frames as fast as possible
    status = CameraGetImageBuffer(camera_handle, &frame_info, &pby_buffer, 10);
    
    if (status != CAMERA_STATUS_SUCCESS) {
      if (status == CAMERA_STATUS_TIME_OUT) {
        timeout_count++;
        // If we're getting many timeouts, camera might not be producing frames
        if (timeout_count > 1000) {
          cerr << "Warning: Many timeouts, camera may not be producing frames at expected rate" << endl;
          timeout_count = 0;
        }
        continue;
      } else {
        cerr << "Error: CameraGetImageBuffer failed with status " << status << endl;
        break;
      }
    }
    
    timeout_count = 0;  // Reset timeout counter on successful read
    
    auto read_end = steady_clock::now();
    
    // Check for frame drops (uiTimeStamp increment should be consistent)
    static UINT last_timestamp = 0;
    static bool first_frame = true;
    if (!first_frame && frame_info.uiTimeStamp > last_timestamp + 100) {
      UINT diff = frame_info.uiTimeStamp - last_timestamp;
      if (diff > 200) {  // Significant gap indicates dropped frames
        dropped_frames++;
      }
    }
    last_timestamp = frame_info.uiTimeStamp;
    first_frame = false;
    
    // Verify frame info
    if (frame_info.iWidth != TARGET_WIDTH || frame_info.iHeight != TARGET_HEIGHT) {
      cerr << "Warning: Frame size " << frame_info.iWidth << "x" << frame_info.iHeight 
           << " (expected " << TARGET_WIDTH << "x" << TARGET_HEIGHT << ")" << endl;
    }
    
    // Store statistics
    frame_count++;
    ProcessFrameStats(read_end);
    
    // Convert frame to Mat for display (grayscale - no conversion needed)
    Mat display_frame;
    if (frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) {
      // Direct grayscale display - no conversion needed
      display_frame = Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC1, pby_buffer);
    } else {
      // Color sensor - process if needed (BGR/RGB)
      unsigned char* rgb_buffer_display = rgb_buffer ? rgb_buffer : (unsigned char*)pby_buffer;
      if (frame_info.uiMediaType != CAMERA_MEDIA_TYPE_BGR8 && 
          frame_info.uiMediaType != CAMERA_MEDIA_TYPE_RGB8) {
        CameraImageProcess(camera_handle, pby_buffer, rgb_buffer_display, &frame_info);
      }
      display_frame = Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC3, rgb_buffer_display);
    }
    
    // Add FPS text overlay
    double recent_fps = 0.0;
    {
      lock_guard<mutex> lock(read_times_mutex);
      if (read_times.size() >= 2) {
        auto total_duration = duration<double>(read_times.back() - read_times.front()).count();
        if (total_duration > 0.0) {
          recent_fps = (read_times.size() - 1) / total_duration;
        }
      }
    }
    auto elapsed = duration<double>(steady_clock::now() - start_time).count();
    double overall_fps = frame_count.load() / elapsed;
    
    string fps_text = "FPS: " + to_string(static_cast<int>(recent_fps)) + " / " 
                     + to_string(static_cast<int>(overall_fps));
    string frame_text = "Frame: " + to_string(frame_count.load());
    string dropped_text = "Dropped: " + to_string(dropped_frames);
    
    // Use white color for grayscale (255) or green for color (0,255,0)
    Scalar text_color = (frame_info.uiMediaType == CAMERA_MEDIA_TYPE_MONO8) 
                        ? Scalar(255) : Scalar(0, 255, 0);
    putText(display_frame, fps_text, Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    putText(display_frame, frame_text, Point(10, 70), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    putText(display_frame, dropped_text, Point(10, 110), FONT_HERSHEY_SIMPLEX, 1, text_color, 2);
    
    // Display frame
    imshow("MindVision Camera @ 1280x1024", display_frame);
    int key = waitKey(1) & 0xFF;
    if (key == 27 || key == 'q') {  // ESC or 'q' to quit
      cout << "\nStopping capture..." << endl;
      break;
    }
    
    // Print statistics periodically
    int current_frame = frame_count.load();
    if (current_frame % STATS_INTERVAL == 0 && current_frame > last_frame_count) {
      last_frame_count = current_frame;
      
      // Calculate recent FPS
      double recent_fps = 0.0;
      {
        lock_guard<mutex> lock(read_times_mutex);
        if (read_times.size() >= 2) {
          auto total_duration = duration<double>(read_times.back() - read_times.front()).count();
          if (total_duration > 0.0) {
            recent_fps = (read_times.size() - 1) / total_duration;
          }
        }
      }
      
      // Calculate overall FPS
      auto elapsed = duration<double>(steady_clock::now() - start_time).count();
      double overall_fps = current_frame / elapsed;
      
      cout << fixed << setprecision(2);
      cout << "Frame " << current_frame 
           << " | Recent FPS: " << recent_fps
           << " | Overall FPS: " << overall_fps
           << " | Dropped: " << dropped_frames << endl;
    }
    
    // Immediately release buffer - don't hold onto frames
    CameraReleaseImageBuffer(camera_handle, pby_buffer);
  }
  
  // Final statistics
  auto end_time = steady_clock::now();
  auto total_duration = duration<double>(end_time - start_time).count();
  int final_frame_count = frame_count.load();
  double final_fps = final_frame_count / total_duration;
  
  cout << "\n=== Final Statistics ===" << endl;
  cout << "Total frames read: " << final_frame_count << endl;
  cout << "Total time: " << fixed << setprecision(2) << total_duration << " seconds" << endl;
  cout << "Average FPS: " << final_fps << " (target: " << TARGET_FPS << ")" << endl;
  
  {
    lock_guard<mutex> lock(read_times_mutex);
    if (read_times.size() >= 2) {
      double min_interval = 1.0;
      double max_interval = 0.0;
      double sum_intervals = 0.0;
      
      for (size_t i = 1; i < read_times.size(); ++i) {
        double interval = duration<double>(read_times[i] - read_times[i-1]).count();
        if (interval < min_interval) min_interval = interval;
        if (interval > max_interval) max_interval = interval;
        sum_intervals += interval;
      }
      
      double avg_interval = sum_intervals / (read_times.size() - 1);
      cout << "\nFrame interval statistics:" << endl;
      cout << "  Min: " << (min_interval * 1000) << " ms" << endl;
      cout << "  Max: " << (max_interval * 1000) << " ms" << endl;
      cout << "  Avg: " << (avg_interval * 1000) << " ms" << endl;
      cout << "  Expected (for " << TARGET_FPS << " FPS): " << (1000.0 / TARGET_FPS) << " ms" << endl;
    }
  }
  
  // Cleanup
  destroyAllWindows();
  CameraStop(camera_handle);
  CameraUnInit(camera_handle);
  if (rgb_buffer) {
    free(rgb_buffer);
  }
  
  cout << "\nCamera closed successfully" << endl;
  return 0;
}
