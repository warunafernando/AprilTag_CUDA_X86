// Simple program to read from MindVision camera at fixed resolution and FPS
// Reads at 1280x1024 @ 211 FPS and measures actual read FPS

#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstring>
#include <thread>

using namespace cv;
using namespace std;
using namespace chrono;

int main(int argc, char *argv[]) {
  const int TARGET_WIDTH = 1280;
  const int TARGET_HEIGHT = 1024;
  const int TARGET_FPS = 211;
  
  cout << "Initializing MindVision SDK..." << endl;
  
  // Enumerate cameras
  tSdkCameraDevInfo camera_list[16];
  INT camera_count = 16;
  CameraSdkStatus status = CameraEnumerateDevice(camera_list, &camera_count);
  
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
  
  // Open first camera
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
  
  // Get camera capability
  tSdkCameraCapbility capability;
  CameraGetCapability(camera_handle, &capability);
  
  int max_width = capability.sResolutionRange.iWidthMax;
  int max_height = capability.sResolutionRange.iHeightMax;
  
  cout << "Camera max resolution: " << max_width << "x" << max_height << endl;
  
  // Allocate buffer for RGB conversion if needed
  unsigned char *rgb_buffer = nullptr;
  if (!capability.sIspCapacity.bMonoSensor) {
    rgb_buffer = static_cast<unsigned char *>(
        malloc(static_cast<size_t>(max_width) * static_cast<size_t>(max_height) * 3));
    cout << "Allocated RGB buffer for color sensor" << endl;
  }
  
  // Set output format
  if (capability.sIspCapacity.bMonoSensor) {
    CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_MONO8);
    cout << "Set output format to MONO8" << endl;
  } else {
    CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_BGR8);
    cout << "Set output format to BGR8" << endl;
  }
  
  // Set resolution to 1280x1024
  cout << "Setting resolution to " << TARGET_WIDTH << "x" << TARGET_HEIGHT << "..." << endl;
  status = CameraSetImageResolutionEx(camera_handle, 0xff, 0, 0, 0, 0,
                                      TARGET_WIDTH, TARGET_HEIGHT, 0, 0);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Warning: CameraSetImageResolutionEx returned status " << status << endl;
  }
  
  // Set frame speed to SUPER (211 FPS)
  cout << "Setting frame speed to SUPER (211 FPS)..." << endl;
  status = CameraSetFrameSpeed(camera_handle, FRAME_SPEED_SUPER);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Warning: CameraSetFrameSpeed returned status " << status << endl;
  }
  
  // Start camera
  status = CameraPlay(camera_handle);
  if (status != CAMERA_STATUS_SUCCESS) {
    cerr << "Error: CameraPlay failed with status " << status << endl;
    CameraUnInit(camera_handle);
    if (rgb_buffer) free(rgb_buffer);
    return 1;
  }
  
  cout << "Camera started, waiting for stream to stabilize..." << endl;
  this_thread::sleep_for(milliseconds(200));
  
  // Read frames and measure FPS
  cout << "\nStarting frame read test..." << endl;
  cout << "Press ESC or 'q' to stop\n" << endl;
  
  vector<steady_clock::time_point> read_times;
  read_times.reserve(1000);
  int frame_count = 0;
  const int STATS_INTERVAL = 100;  // Print stats every N frames
  
  auto start_time = steady_clock::now();
  
  while (true) {
    tSdkFrameHead frame_info;
    BYTE *pby_buffer = nullptr;
    
    auto read_start = steady_clock::now();
    
    // Get frame with 100ms timeout (longer for high FPS)
    status = CameraGetImageBuffer(camera_handle, &frame_info, &pby_buffer, 100);
    
    if (status != CAMERA_STATUS_SUCCESS) {
      if (status == CAMERA_STATUS_TIME_OUT) {
        continue;  // Timeout, try again
      }
      cerr << "Error: CameraGetImageBuffer failed with status " << status << endl;
      break;
    }
    
    auto read_end = steady_clock::now();
    
    // Store read time
    read_times.push_back(read_end);
    if (read_times.size() > 1000) {
      read_times.erase(read_times.begin());
    }
    
    frame_count++;
    
    // Verify resolution directly from frame_info (skip Mat conversion for speed)
    if (frame_info.iWidth != TARGET_WIDTH || frame_info.iHeight != TARGET_HEIGHT) {
      cerr << "Warning: Frame size mismatch: " << frame_info.iWidth << "x" << frame_info.iHeight 
           << " (expected " << TARGET_WIDTH << "x" << TARGET_HEIGHT << ")" << endl;
    }
    
    // Calculate and display FPS periodically
    if (frame_count % STATS_INTERVAL == 0 && read_times.size() >= 2) {
      auto total_duration = duration<double>(read_times.back() - read_times.front()).count();
      double avg_fps = (read_times.size() - 1) / total_duration;
      
      auto elapsed = duration<double>(steady_clock::now() - start_time).count();
      double overall_fps = frame_count / elapsed;
      
      cout << fixed << setprecision(2);
      cout << "Frame " << frame_count 
           << " | Recent FPS: " << avg_fps
           << " | Overall FPS: " << overall_fps
           << " | Size: " << TARGET_WIDTH << "x" << TARGET_HEIGHT << endl;
    }
    
    // Optional: Display frame (comment out for pure read performance test)
    // imshow("Camera", frame);
    // int key = waitKey(1) & 0xFF;
    // if (key == 27 || key == 'q') {
    //   break;
    // }
    
    // Release frame buffer
    CameraReleaseImageBuffer(camera_handle, pby_buffer);
  }
  
  // Final statistics
  auto end_time = steady_clock::now();
  auto total_duration = duration<double>(end_time - start_time).count();
  double final_fps = frame_count / total_duration;
  
  cout << "\n=== Final Statistics ===" << endl;
  cout << "Total frames read: " << frame_count << endl;
  cout << "Total time: " << fixed << setprecision(2) << total_duration << " seconds" << endl;
  cout << "Average FPS: " << final_fps << " (target: " << TARGET_FPS << ")" << endl;
  
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
    cout << "Frame interval stats:" << endl;
    cout << "  Min: " << (min_interval * 1000) << " ms" << endl;
    cout << "  Max: " << (max_interval * 1000) << " ms" << endl;
    cout << "  Avg: " << (avg_interval * 1000) << " ms" << endl;
  }
  
  // Cleanup
  CameraStop(camera_handle);
  CameraUnInit(camera_handle);
  if (rgb_buffer) {
    free(rgb_buffer);
  }
  
  cout << "Camera closed successfully" << endl;
  return 0;
}
