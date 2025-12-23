#include "CameraApi.h"
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/workerpool.h>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <numeric>

using namespace cv;
using namespace std;

// Simple function to draw detection outlines
void draw_detection_outlines(cv::Mat& img, zarray_t* detections) {
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    
    // Draw the tag outline
    line(img, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]), Scalar(0, 0xff, 0), 2);
    line(img, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0, 0, 0xff), 2);
    line(img, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]), Scalar(0xff, 0, 0), 2);
    line(img, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0, 0), 2);
    
    // Draw tag ID
    string text = to_string(det->id);
    int baseline;
    Size text_size = getTextSize(text, FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
    putText(img, text, Point(det->c[0] - text_size.width / 2, det->c[1] + text_size.height / 2),
            FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0xff, 0xff, 0), 2);
  }
}

int main(int argc, char** argv) {
  int camera_index = 0;
  bool use_mvsdk = true;  // Default to MindVision SDK
  string video_file = "";
  bool test_mirror = false;  // Test horizontal mirror as well
  
  // Parse command line arguments
  if (argc > 1) {
    string arg1 = argv[1];
    if (arg1.find(".avi") != string::npos || arg1.find(".mp4") != string::npos || 
        arg1.find(".mov") != string::npos) {
      video_file = arg1;
      use_mvsdk = false;
    } else {
      camera_index = atoi(argv[1]);
    }
  }
  if (argc > 2 && string(argv[2]) == "--opencv") {
    use_mvsdk = false;
  }
  // Check for mirror testing flag
  for (int i = 1; i < argc; i++) {
    if (string(argv[i]) == "--mirror" || string(argv[i]) == "-m") {
      test_mirror = true;
    }
  }
  
  // Initialize AprilTag detector
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  
  td->quad_decimate = 1.0;  // 1.0 for better detection (slower but detects smaller tags), 2.0 is faster
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;
  td->nthreads = 4;  // Use more threads for faster processing
  td->wp = workerpool_create(4);
  
  cout << "AprilTag detector initialized:" << endl;
  cout << "  Family: tag36h11" << endl;
  cout << "  Quad decimate: " << td->quad_decimate << endl;
  cout << "  Refine edges: " << td->refine_edges << endl;
  cout << "  Mirror testing: " << (test_mirror ? "ENABLED" : "DISABLED") << endl;
  
  VideoCapture cap;
  Mat frame, gray;
  CameraHandle mv_handle = 0;
  bool mv_mono = true;
  bool use_video = !video_file.empty();
  
  if (use_video) {
    cap.open(video_file);
    if (!cap.isOpened()) {
      cerr << "Error: Could not open video file " << video_file << endl;
      return 1;
    }
    cout << "Video file opened: " << video_file << endl;
  } else if (use_mvsdk) {
    // Initialize MindVision SDK
    CameraSdkStatus status = CameraSdkInit(1);  // 1 = Chinese, 0 = English
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: CameraSdkInit failed with status " << status << endl;
      return 1;
    }
    
    // Enumerate cameras
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
    
    if (camera_index < 0 || camera_index >= camera_count) {
      cerr << "Error: Invalid camera index " << camera_index << endl;
      return 1;
    }
    
    // Initialize camera
    status = CameraInit(&camera_list[camera_index], -1, -1, &mv_handle);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: CameraInit failed with status " << status << endl;
      return 1;
    }
    
    // Get capability
    tSdkCameraCapbility capability;
    CameraGetCapability(mv_handle, &capability);
    mv_mono = capability.sIspCapacity.bMonoSensor;
    
    // Set output format to grayscale
    if (mv_mono) {
      CameraSetIspOutFormat(mv_handle, CAMERA_MEDIA_TYPE_MONO8);
    } else {
      CameraSetIspOutFormat(mv_handle, CAMERA_MEDIA_TYPE_MONO8);
    }
    
    // Set resolution to 1280x1024
    status = CameraSetImageResolutionEx(mv_handle, 0xff, 0, 0, 0, 0,
                                        1280, 1024, 0, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Warning: CameraSetImageResolutionEx returned status " << status << endl;
    }
    
    // Set frame speed to highest available
    int max_frame_speed = capability.iFrameSpeedDesc - 1;
    if (max_frame_speed < 0) max_frame_speed = 0;
    CameraSetFrameSpeed(mv_handle, max_frame_speed);
    
    // Disable auto exposure and set fixed exposure (longer for better visibility)
    CameraSetAeState(mv_handle, FALSE);
    // Load settings from GUI config file (try build directory first, then current directory)
    int config_exposure = -1, config_brightness = -1, config_gain = -1;
    string config_path = "gui_config.txt";
    ifstream config_file(config_path);
    if (!config_file.is_open()) {
      // Try in parent directory (if running from tools/)
      config_path = "../gui_config.txt";
      config_file.open(config_path);
    }
    if (!config_file.is_open()) {
      // Try build directory
      config_path = "build/gui_config.txt";
      config_file.open(config_path);
    }
    if (config_file.is_open()) {
      string line;
      bool in_camera_controls = false;
      while (getline(config_file, line)) {
        if (line.find("[camera_controls]") != string::npos) {
          in_camera_controls = true;
          continue;
        }
        if (in_camera_controls && line[0] == '[') {
          break;  // Next section
        }
        if (in_camera_controls) {
          // Trim whitespace
          size_t colon_pos = line.find(":");
          if (colon_pos != string::npos) {
            string key = line.substr(0, colon_pos);
            string value_str = line.substr(colon_pos + 1);
            // Trim whitespace from value
            value_str.erase(0, value_str.find_first_not_of(" \t"));
            value_str.erase(value_str.find_last_not_of(" \t") + 1);
            if (key == "exposure" && !value_str.empty()) {
              config_exposure = stoi(value_str);
            } else if (key == "brightness" && !value_str.empty()) {
              config_brightness = stoi(value_str);
            } else if (key == "gain" && !value_str.empty()) {
              config_gain = stoi(value_str);
            }
          }
        }
      }
      config_file.close();
      if (config_exposure >= 0 || config_brightness >= 0 || config_gain >= 0) {
        cout << "Loaded settings from " << config_path << ":" << endl;
      }
    }
    
    // Get exposure range and set exposure from config (or use default)
    double exposure_min, exposure_max, exposure_def;
    CameraGetExposureTimeRange(mv_handle, &exposure_min, &exposure_max, &exposure_def);
    cout << "Exposure range: " << exposure_min << " - " << exposure_max << " us (default: " << exposure_def << ")" << endl;
    
    double exposure_time;
    if (config_exposure >= 0) {
      // Use GUI mapping: exposure = max_exposure - (value / 100.0) * (max_exposure - min_exposure)
      // Where value 0 = max exposure, 100 = min exposure
      double min_exposure = 1000.0;
      double max_exposure = 100000.0;
      exposure_time = max_exposure - (config_exposure / 100.0) * (max_exposure - min_exposure);
      cout << "Using exposure from config: slider=" << config_exposure << " -> " << exposure_time << " us (" << (exposure_time/1000.0) << " ms)" << endl;
    } else {
      exposure_time = 25000.0;  // Default 25ms
      cout << "Using default exposure: " << exposure_time << " us (" << (exposure_time/1000.0) << " ms)" << endl;
    }
    if (exposure_time > exposure_max) exposure_time = exposure_max;
    if (exposure_time < exposure_min) exposure_time = exposure_min;
    CameraSetExposureTime(mv_handle, exposure_time);
    
    // Set analog gain from brightness config (or use default)
    if (config_brightness >= 0) {
      // GUI mapping: analog_gain = (brightness * 100) / 255
      INT analog_gain = (config_brightness * 100) / 255;
      if (CameraSetAnalogGain(mv_handle, analog_gain) == CAMERA_STATUS_SUCCESS) {
        cout << "Using analog gain from config brightness: brightness=" << config_brightness << " -> analog_gain=" << analog_gain << endl;
      }
    }
    
    // Set digital gain from config (or use default)
    if (config_gain >= 0) {
      if (CameraSetGain(mv_handle, config_gain, config_gain, config_gain) == CAMERA_STATUS_SUCCESS) {
        cout << "Using digital gain from config: gain=" << config_gain << endl;
      }
    }
    
    // Start capture
    CameraPlay(mv_handle);
    
    cout << "MindVision camera opened successfully" << endl;
  } else {
    cap.open(camera_index);
    if (!cap.isOpened()) {
      cerr << "Error: Could not open camera " << camera_index << endl;
      return 1;
    }
    
    // Set camera properties for better performance
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1024);
    cap.set(CAP_PROP_CONVERT_RGB, false);
    
    cout << "OpenCV camera opened successfully" << endl;
  }
  
  cout << "Press 'q' or ESC to quit" << endl;
  
  auto last_time = chrono::steady_clock::now();
  auto start_time = chrono::steady_clock::now();
  int frame_count = 0;
  int total_frames_processed = 0;  // Keep separate counter for progress
  double fps = 0.0;
  
  // Statistics collection
  vector<double> detection_times;  // in microseconds
  vector<int> detection_counts;
  int total_detections = 0;
  
  while (true) {
    if (use_mvsdk && mv_handle != 0) {
      // Read frame from MindVision camera
      BYTE *pbyBuffer = nullptr;
      tSdkFrameHead sFrameInfo;
      CameraSdkStatus status = CameraGetImageBuffer(mv_handle, &sFrameInfo, &pbyBuffer, 100);
      
      if (status != CAMERA_STATUS_SUCCESS) {
        if (status == CAMERA_STATUS_TIME_OUT) {
          continue;  // Skip timeout frames
        }
        cerr << "Error: CameraGetImageBuffer failed with status " << status << endl;
        break;
      }
      
      // Create Mat from buffer (grayscale)
      frame = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1, pbyBuffer).clone();
      
      // Release buffer
      CameraReleaseImageBuffer(mv_handle, pbyBuffer);
      
      gray = frame;  // Already grayscale
    } else {
      // Read frame from OpenCV camera or video file
      cap >> frame;
      if (frame.empty()) {
        if (use_video) {
          cout << "End of video file" << endl;
        } else {
          cerr << "Error: Failed to read frame" << endl;
        }
        break;
      }
      
      // Convert to grayscale if needed
      if (frame.channels() == 3) {
        cvtColor(frame, gray, COLOR_BGR2GRAY);
      } else if (frame.channels() == 2) {
        cvtColor(frame, gray, COLOR_YUV2GRAY_YUY2);
      } else {
        gray = frame;
      }
    }
    
    // Ensure gray frame is contiguous
    if (!gray.isContinuous()) {
      gray = gray.clone();
    }
    
    // STEP-BY-STEP DEBUGGING
    static int debug_counter = 0;
    static bool saved_test_frame = false;
    static bool detailed_debug_done = false;
    
    // Step 1: Validate frame format
    if (debug_counter == 0) {
      cout << "\n========== STEP 1: FRAME VALIDATION ==========" << endl;
      cout << "Frame dimensions: " << gray.cols << "x" << gray.rows << endl;
      cout << "Frame channels: " << gray.channels() << " (expected: 1)" << endl;
      cout << "Frame type: " << gray.type() << " (CV_8UC1=" << CV_8UC1 << ")" << endl;
      cout << "Data pointer: " << (void*)gray.data << endl;
      cout << "Is contiguous: " << gray.isContinuous() << endl;
      cout << "Total pixels: " << gray.total() << endl;
    }
    
    // Step 2: Analyze image statistics
    if (debug_counter == 0) {
      cout << "\n========== STEP 2: IMAGE STATISTICS ==========" << endl;
      Scalar mean_val, stddev_val;
      meanStdDev(gray, mean_val, stddev_val);
      cout << "Mean pixel value: " << mean_val[0] << " (ideal: 60-180)" << endl;
      cout << "Std deviation: " << stddev_val[0] << " (higher = more contrast)" << endl;
      
      double min_val, max_val;
      minMaxLoc(gray, &min_val, &max_val);
      cout << "Pixel range: " << min_val << " - " << max_val << endl;
      cout << "Dynamic range: " << (max_val - min_val) << " (ideal: >100)" << endl;
    }
    
    // Step 3: Validate image_u8_t structure
    image_u8_t im = {
      .width = gray.cols,
      .height = gray.rows,
      .stride = gray.cols,
      .buf = gray.data
    };
    
    if (debug_counter == 0) {
      cout << "\n========== STEP 3: IMAGE_U8_T STRUCTURE ==========" << endl;
      cout << "im.width: " << im.width << endl;
      cout << "im.height: " << im.height << endl;
      cout << "im.stride: " << im.stride << " (should equal width for contiguous)" << endl;
      cout << "im.buf pointer: " << (void*)im.buf << " (should match gray.data)" << endl;
      cout << "Buffer size check: " << (im.width * im.height) << " bytes" << endl;
      
      // Validate buffer accessibility
      if (im.buf != nullptr) {
        cout << "First pixel: " << (int)im.buf[0] << endl;
        cout << "Last pixel: " << (int)im.buf[im.width * im.height - 1] << endl;
        cout << "Center pixel: " << (int)im.buf[im.width * (im.height/2) + (im.width/2)] << endl;
      } else {
        cout << "ERROR: im.buf is NULL!" << endl;
      }
    }
    
    // Step 4: Save test frame for visual inspection
    if (!saved_test_frame) {
      imwrite("/tmp/test_frame_raw.png", gray);
      
      // Also save enhanced versions for comparison
      Mat enhanced;
      equalizeHist(gray, enhanced);
      imwrite("/tmp/test_frame_enhanced.png", enhanced);
      
      // Save contrast-enhanced version
      Mat contrast;
      gray.convertTo(contrast, -1, 2.0, 50); // Increase contrast and brightness
      imwrite("/tmp/test_frame_contrast.png", contrast);
      
      cout << "\n========== STEP 4: SAVED TEST FRAMES ==========" << endl;
      cout << "Saved frames to:" << endl;
      cout << "  /tmp/test_frame_raw.png (original)" << endl;
      cout << "  /tmp/test_frame_enhanced.png (histogram equalized)" << endl;
      cout << "  /tmp/test_frame_contrast.png (contrast enhanced)" << endl;
      saved_test_frame = true;
    }
    
    // Step 5: Try detection with original image
    if (debug_counter == 0) {
      cout << "\n========== STEP 5: DETECTION ATTEMPT (ORIGINAL) ==========" << endl;
      cout << "Detector parameters:" << endl;
      cout << "  quad_decimate: " << td->quad_decimate << endl;
      cout << "  quad_sigma: " << td->quad_sigma << endl;
      cout << "  refine_edges: " << td->refine_edges << endl;
      cout << "  decode_sharpening: " << td->decode_sharpening << endl;
      cout << "  nthreads: " << td->nthreads << endl;
    }
    
    auto detect_start = chrono::steady_clock::now();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    
    // If mirror testing is enabled, also test horizontally mirrored image
    zarray_t* detections_mirrored = nullptr;
    if (test_mirror) {
      Mat gray_mirrored;
      flip(gray, gray_mirrored, 1);  // 1 = horizontal flip
      
      image_u8_t im_mirrored = {
        .width = gray_mirrored.cols,
        .height = gray_mirrored.rows,
        .stride = gray_mirrored.cols,
        .buf = gray_mirrored.data
      };
      
      detections_mirrored = apriltag_detector_detect(td, &im_mirrored);
      
      // Merge detections from mirrored image (adjust x coordinates back)
      int num_orig = zarray_size(detections);
      int num_mirrored = zarray_size(detections_mirrored);
      
      if (num_mirrored > 0) {
        // Create new array with combined detections
        zarray_t* combined = zarray_create(sizeof(apriltag_detection_t*));
        
        // Add original detections (need to copy pointers)
        for (int i = 0; i < num_orig; i++) {
          apriltag_detection_t* det;
          zarray_get(detections, i, &det);
          zarray_add(combined, &det);
        }
        
        // Add mirrored detections (mirror x coordinates back)
        for (int i = 0; i < num_mirrored; i++) {
          apriltag_detection_t* det_mirr;
          zarray_get(detections_mirrored, i, &det_mirr);
          
          // Create a copy and mirror x coordinates
          apriltag_detection_t* det_copy = (apriltag_detection_t*)calloc(1, sizeof(apriltag_detection_t));
          memcpy(det_copy, det_mirr, sizeof(apriltag_detection_t));
          
          // Mirror x coordinates: x_new = width - x_old
          int width = gray.cols;
          det_copy->c[0] = width - det_copy->c[0];
          for (int j = 0; j < 4; j++) {
            det_copy->p[j][0] = width - det_copy->p[j][0];
          }
          
          zarray_add(combined, &det_copy);
        }
        
        // Destroy old detections array structure (but not the detections themselves - they're in combined now)
        zarray_destroy(detections);
        
        // Clean up mirrored detections array (but not the detections - original ones moved, copies created)
        zarray_destroy(detections_mirrored);
        
        detections = combined;
      } else {
        // No mirrored detections, just clean up
        zarray_destroy(detections_mirrored);
      }
    }
    
    auto detect_end = chrono::steady_clock::now();
    auto detect_time = chrono::duration_cast<chrono::microseconds>(detect_end - detect_start).count();
    
    // Collect statistics (skip first frame which has debugging)
    if (debug_counter > 1 || !detailed_debug_done) {
      detection_times.push_back(detect_time / 1000.0);  // Convert to milliseconds
      int num_dets = zarray_size(detections);
      detection_counts.push_back(num_dets);
      total_detections += num_dets;
      total_frames_processed++;
    }
    
    // Step 6: Analyze detection results
    int num_detections = zarray_size(detections);
    if (debug_counter == 0 || num_detections > 0) {
      if (debug_counter == 0) {
        cout << "\n========== STEP 6: DETECTION RESULTS ==========" << endl;
        cout << "Detection time: " << detect_time << " microseconds";
        if (test_mirror) {
          cout << " (including mirror test)";
        }
        cout << endl;
      }
      cout << "Detections: " << num_detections;
      if (test_mirror) {
        cout << " (original + mirrored)";
      }
      cout << endl;
      for (int i = 0; i < num_detections; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        cout << "  Tag ID: " << det->id 
             << ", center: (" << det->c[0] << ", " << det->c[1] << ")"
             << ", decision_margin: " << det->decision_margin << endl;
      }
      
      if (debug_counter == 0 && num_detections == 0) {
        cout << "\n========== STEP 7: TROUBLESHOOTING (NO DETECTIONS) ==========" << endl;
        Scalar mean_val, stddev_val;
        meanStdDev(gray, mean_val, stddev_val);
        
        if (mean_val[0] < 30) {
          cout << "WARNING: Image is too dark (mean < 30). Try increasing exposure/brightness." << endl;
        } else if (mean_val[0] > 220) {
          cout << "WARNING: Image is too bright (mean > 220). Try decreasing exposure." << endl;
        }
        
        if (stddev_val[0] < 20) {
          cout << "WARNING: Low contrast (stddev < 20). Image may be too uniform." << endl;
        }
        
        // Try with enhanced image
        Mat enhanced;
        equalizeHist(gray, enhanced);
        image_u8_t im_enhanced = {
          .width = enhanced.cols,
          .height = enhanced.rows,
          .stride = enhanced.cols,
          .buf = enhanced.data
        };
        zarray_t* detections_enhanced = apriltag_detector_detect(td, &im_enhanced);
        int num_enhanced = zarray_size(detections_enhanced);
        cout << "  Detection with histogram equalization: " << num_enhanced << " tags" << endl;
        for (int i = 0; i < num_enhanced; i++) {
          apriltag_detection_t* det;
          zarray_get(detections_enhanced, i, &det);
          apriltag_detection_destroy(det);
        }
        zarray_destroy(detections_enhanced);
        
        // Try with contrast enhancement
        Mat contrast;
        gray.convertTo(contrast, -1, 2.0, 50);
        image_u8_t im_contrast = {
          .width = contrast.cols,
          .height = contrast.rows,
          .stride = contrast.cols,
          .buf = contrast.data
        };
        zarray_t* detections_contrast = apriltag_detector_detect(td, &im_contrast);
        int num_contrast = zarray_size(detections_contrast);
        cout << "  Detection with contrast enhancement: " << num_contrast << " tags" << endl;
        for (int i = 0; i < num_contrast; i++) {
          apriltag_detection_t* det;
          zarray_get(detections_contrast, i, &det);
          apriltag_detection_destroy(det);
        }
        zarray_destroy(detections_contrast);
        
        // Try with gamma correction (brighten dark images)
        Mat gamma;
        double gamma_val = 0.5;  // Gamma < 1 brightens the image
        Mat lookup_table(1, 256, CV_8U);
        uchar* p = lookup_table.ptr();
        for (int i = 0; i < 256; i++) {
          p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma_val) * 255.0);
        }
        LUT(gray, lookup_table, gamma);
        image_u8_t im_gamma = {
          .width = gamma.cols,
          .height = gamma.rows,
          .stride = gamma.cols,
          .buf = gamma.data
        };
        zarray_t* detections_gamma = apriltag_detector_detect(td, &im_gamma);
        int num_gamma = zarray_size(detections_gamma);
        cout << "  Detection with gamma correction (gamma=0.5): " << num_gamma << " tags" << endl;
        for (int i = 0; i < num_gamma; i++) {
          apriltag_detection_t* det;
          zarray_get(detections_gamma, i, &det);
          apriltag_detection_destroy(det);
        }
        zarray_destroy(detections_gamma);
        
        // Try with quad_decimate = 2.0 for comparison (faster processing, less sensitive to small tags)
        cout << "\n  Trying with quad_decimate=2.0 (faster, less sensitive):" << endl;
        // Try with quad_decimate = 2.0 for comparison (faster, less sensitive)
        td->quad_decimate = 2.0;
        zarray_t* detections_coarse = apriltag_detector_detect(td, &im);
        int num_coarse = zarray_size(detections_coarse);
        cout << "  Detection with quad_decimate=2.0: " << num_coarse << " tags" << endl;
        for (int i = 0; i < num_coarse; i++) {
          apriltag_detection_t* det;
          zarray_get(detections_coarse, i, &det);
          cout << "    Tag ID: " << det->id << ", center: (" << det->c[0] << ", " << det->c[1] << ")" << endl;
          apriltag_detection_destroy(det);
        }
        zarray_destroy(detections_coarse);
        td->quad_decimate = 1.0;  // Restore original value
      }
      
      if (debug_counter == 0) {
        cout << "\n==========================================\n" << endl;
        detailed_debug_done = true;
      }
    }
    debug_counter++;
    
    // Convert to BGR for display
    Mat display_frame;
    if (frame.channels() == 1) {
      cvtColor(frame, display_frame, COLOR_GRAY2BGR);
    } else if (frame.channels() == 2) {
      cvtColor(frame, display_frame, COLOR_YUV2BGR_YUY2);
    } else {
      display_frame = frame.clone();
    }
    
    // Draw detections
    draw_detection_outlines(display_frame, detections);
    
    // Calculate FPS
    frame_count++;
    auto current_time = chrono::steady_clock::now();
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(current_time - last_time).count();
    if (elapsed >= 1000) {
      fps = (frame_count * 1000.0) / elapsed;
      frame_count = 0;
      last_time = current_time;
    }
    
    // Draw info
    string info_text = "FPS: " + to_string(fps).substr(0, 4) + 
                       "  Tags: " + to_string(zarray_size(detections));
    putText(display_frame, info_text, Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
    
    // Display (skip for video files to speed up processing)
    if (!use_video) {
      imshow("AprilTag Detection", display_frame);
    }
    
    // Clean up detections
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);
    
    // Check for exit (skip for video files)
    if (!use_video) {
      int key = waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) {  // 'q' or ESC
        break;
      }
    }
    
    // Progress indicator for video processing
    if (use_video && total_frames_processed % 100 == 0 && total_frames_processed > 0) {
      cout << "Processed " << total_frames_processed << " frames..." << endl;
    }
  }
  
  // Calculate final statistics
  auto end_time = chrono::steady_clock::now();
  auto total_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
  
  // Cleanup
  if (use_mvsdk && mv_handle != 0) {
    CameraStop(mv_handle);
    CameraUnInit(mv_handle);
  } else {
    cap.release();
  }
  destroyAllWindows();
  
  // Print statistics summary
  if (use_video && detection_times.size() > 0) {
    cout << "\n========================================" << endl;
    cout << "TIMING STATISTICS SUMMARY" << endl;
    cout << "========================================" << endl;
    cout << "Total frames processed: " << detection_times.size() << endl;
    cout << "Total processing time: " << total_time << " ms (" << (total_time / 1000.0) << " seconds)" << endl;
    cout << "Average processing FPS: " << (detection_times.size() * 1000.0 / total_time) << endl;
    cout << "\nDetection timing (ms/frame):" << endl;
    
    // Calculate statistics
    double sum = 0.0;
    for (double t : detection_times) sum += t;
    double mean = sum / detection_times.size();
    
    sort(detection_times.begin(), detection_times.end());
    double median = detection_times[detection_times.size() / 2];
    double p90 = detection_times[static_cast<size_t>(detection_times.size() * 0.90)];
    double p99 = detection_times[static_cast<size_t>(detection_times.size() * 0.99)];
    double min_time = detection_times[0];
    double max_time = detection_times.back();
    
    cout << "  Mean:   " << fixed << setprecision(3) << mean << " ms" << endl;
    cout << "  Median: " << median << " ms" << endl;
    cout << "  P90:    " << p90 << " ms" << endl;
    cout << "  P99:    " << p99 << " ms" << endl;
    cout << "  Min:    " << min_time << " ms" << endl;
    cout << "  Max:    " << max_time << " ms" << endl;
    
    cout << "\nDetection counts:" << endl;
    cout << "  Total detections: " << total_detections << endl;
    cout << "  Average per frame: " << (total_detections / (double)detection_counts.size()) << endl;
    
    int frames_with_tags = 0;
    int frames_without_tags = 0;
    for (int count : detection_counts) {
      if (count > 0) frames_with_tags++;
      else frames_without_tags++;
    }
    cout << "  Frames with tags: " << frames_with_tags << endl;
    cout << "  Frames without tags: " << frames_without_tags << endl;
    
    cout << "========================================" << endl;
  }
  
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  
  return 0;
}

