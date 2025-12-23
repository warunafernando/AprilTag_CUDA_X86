#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>

#if defined(__CUDACC__)
#pragma diag_suppress 611
#endif

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
}
#include "g2d.h"

// MindVision SDK support
#include "CameraApi.h"

using namespace std;
using namespace cv;

// External CPU decode helper
namespace frc971::apriltag {
void DecodeTagsFromQuads(const std::vector<QuadCorners> &quad_corners,
                         const uint8_t *gray_buf, int width, int height,
                         apriltag_detector_t *td,
                         const CameraMatrix &camera_matrix,
                         const DistCoeffs &distortion_coefficients,
                         zarray_t *detections,
                         zarray_t *poly0,
                         zarray_t *poly1);
}

int main(int argc, char **argv) {
  string family = "tag36h11";
  double tag_size = 0.1; // Tag size in meters
  bool horizontal_mirror = false;
  int camera_index = 0;
  bool use_mvsdk = false;
  
  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    string arg(argv[i]);
    if ((arg == "--camera" || arg == "-c") && i + 1 < argc) {
      camera_index = atoi(argv[++i]);
    } else if (arg == "--mvsdk") {
      use_mvsdk = true;
    } else if (arg == "--mirror" || arg == "-m") {
      horizontal_mirror = true;
    } else if (arg == "--help" || arg == "-h") {
      cout << "Usage: camera_gpu_detect [--camera <index>] [--mvsdk] [--mirror]\n";
      cout << "  --camera/-c <index>  Camera index (default: 0)\n";
      cout << "  --mvsdk              Use MindVision SDK camera\n";
      cout << "  --mirror/-m          Enable horizontal mirroring\n";
      return 0;
    }
  }
  
  // Initialize camera
  int width, height;
  double fps = 30.0;
  Mat frame;
  VideoCapture cap;
  CameraHandle mv_camera_handle = 0;
  
  if (use_mvsdk) {
    // MindVision SDK camera
    cout << "Initializing MindVision SDK..." << endl;
    CameraSdkStatus status = CameraSdkInit(1);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: CameraSdkInit failed" << endl;
      return 1;
    }
    
    tSdkCameraDevInfo camera_list[16];
    INT camera_count = 16;
    status = CameraEnumerateDevice(camera_list, &camera_count);
    if (status != CAMERA_STATUS_SUCCESS || camera_count <= 0 || camera_index >= camera_count) {
      cerr << "Error: MindVision camera not found!" << endl;
      return 1;
    }
    
    status = CameraInit(&camera_list[camera_index], -1, -1, &mv_camera_handle);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: CameraInit failed" << endl;
      return 1;
    }
    
    tSdkCameraCapbility tCapability;
    CameraGetCapability(mv_camera_handle, &tCapability);
    CameraSetIspOutFormat(mv_camera_handle, CAMERA_MEDIA_TYPE_MONO8);
    
    // Set resolution (1280x1024)
    width = 1280;
    height = 1024;
    status = CameraSetImageResolutionEx(mv_camera_handle, 0xff, 0, 0, 0, 0, width, height, 0, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Warning: CameraSetImageResolutionEx returned status " << status << endl;
    }
    // Set frame speed to highest available
    int max_frame_speed = tCapability.iFrameSpeedDesc - 1;
    if (max_frame_speed < 0) max_frame_speed = 0;
    if (max_frame_speed > FRAME_SPEED_SUPER) max_frame_speed = FRAME_SPEED_SUPER;
    CameraSetFrameSpeed(mv_camera_handle, max_frame_speed);
    
    // Disable auto exposure and set fixed exposure
    CameraSetAeState(mv_camera_handle, FALSE);
    CameraSetExposureTime(mv_camera_handle, 3000);
    CameraSetAnalogGain(mv_camera_handle, 1);
    CameraPlay(mv_camera_handle);
    
    fps = 211.0;
    cout << "MindVision camera initialized: " << width << "x" << height << endl;
  } else {
    // V4L2 camera
    cap.open(camera_index, CAP_V4L2);
    if (!cap.isOpened()) {
      cerr << "Failed to open camera index " << camera_index << endl;
      return 1;
    }
    
    cap.set(CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CAP_PROP_FRAME_HEIGHT, 1024);
    cap.set(CAP_PROP_CONVERT_RGB, false);
    
    width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    
    cout << "Camera initialized: " << width << "x" << height << " @ " << fps << " FPS" << endl;
  }
  
  // Ensure even dimensions for CUDA (required by GpuDetector)
  if (width % 2 != 0) width = (width / 2) * 2;
  if (height % 2 != 0) height = (height / 2) * 2;
  
  cout << "Final resolution: " << width << "x" << height << endl;
  
  // Camera parameters (from fisheye calibration)
  frc971::apriltag::CameraMatrix cam{865.077, 868.827, 651.002, 454.253};
  frc971::apriltag::DistCoeffs dist{0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Create tag detector
  apriltag_family_t *tf = tag36h11_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->nthreads = 4;
  td->wp = workerpool_create(4);
  
  cout << "Creating GPU detector..." << endl;
  // Create GPU detector
  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
  cout << "GPU detector created successfully" << endl;
  
  cout << "Starting detection... Press 'q' to quit" << endl;
  
  zarray_t *poly0 = g2d_polygon_create_zeros(4);
  zarray_t *poly1 = g2d_polygon_create_zeros(4);
  
  auto t_start = chrono::steady_clock::now();
  size_t frame_num = 0;
  double acc_detect_ms = 0.0;
  double acc_decode_ms = 0.0;
  double acc_display_ms = 0.0;
  
  while (true) {
    // Read frame
    bool ok = false;
    if (use_mvsdk && mv_camera_handle != 0) {
      BYTE *pbyBuffer = nullptr;
      tSdkFrameHead sFrameInfo;
      CameraSdkStatus status = CameraGetImageBuffer(mv_camera_handle, &sFrameInfo, &pbyBuffer, 100);
      if (status == CAMERA_STATUS_SUCCESS) {
        if (sFrameInfo.iWidth > 0 && sFrameInfo.iHeight > 0) {
          frame = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1, pbyBuffer, sFrameInfo.iWidth);
          frame = frame.clone();
          CameraReleaseImageBuffer(mv_camera_handle, pbyBuffer);
          ok = !frame.empty() && frame.cols > 0 && frame.rows > 0;
        } else {
          CameraReleaseImageBuffer(mv_camera_handle, pbyBuffer);
        }
      } else if (status == CAMERA_STATUS_TIME_OUT) {
        continue; // Timeout is normal, try again
      } else {
        cerr << "CameraGetImageBuffer error: " << status << endl;
        break;
      }
    } else {
      ok = cap.read(frame);
      if (ok && frame.type() != CV_8UC1) {
        if (frame.channels() == 3) {
          cvtColor(frame, frame, COLOR_BGR2GRAY);
        }
      }
    }
    
    if (!ok || frame.empty() || frame.cols <= 0 || frame.rows <= 0) {
      if (frame_num == 0) {
        cerr << "Failed to read first frame" << endl;
      }
      continue; // Skip this frame, try next
    }
    
    // Convert to grayscale if needed
    if (frame.channels() == 2) {
      cvtColor(frame, frame, COLOR_YUV2GRAY_YUY2);
    } else if (frame.channels() == 3) {
      cvtColor(frame, frame, COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
      cvtColor(frame, frame, COLOR_BGRA2GRAY);
    }
    
    // Ensure single channel
    if (frame.type() != CV_8UC1) {
      if (frame.channels() > 1) {
        cvtColor(frame, frame, COLOR_BGR2GRAY);
      }
    }
    
    // Resize to expected dimensions if needed
    if (frame.cols != width || frame.rows != height) {
      resize(frame, frame, Size(width, height));
    }
    
    // Ensure frame is correct size and type
    if (frame.cols != width || frame.rows != height || frame.type() != CV_8UC1) {
      cerr << "Frame size/type mismatch: " << frame.cols << "x" << frame.rows 
           << " type=" << frame.type() << " expected " << width << "x" << height << " CV_8UC1" << endl;
      continue;
    }
    
    auto detect_start = chrono::steady_clock::now();
    
    // GPU detection
    try {
      if (frame.data == nullptr) {
        cerr << "Frame data is null!" << endl;
        continue;
      }
      detector.DetectGpuOnly(frame.data);
      auto quads_fullres = detector.FitQuads();
      
      // Scale quads from full resolution to decimated space
    const double quad_decimate = td->quad_decimate;
    std::vector<frc971::apriltag::QuadCorners> quads(quads_fullres.size());
    int gray_width = detector.Width();
    int gray_height = detector.Height();
    
    for (size_t i = 0; i < quads_fullres.size(); i++) {
      quads[i] = quads_fullres[i];
      if (quad_decimate > 1.0) {
        if (quad_decimate != 1.5) {
          for (int j = 0; j < 4; j++) {
            quads[i].corners[j][0] = (quads[i].corners[j][0] - 0.5) / quad_decimate + 0.5;
            quads[i].corners[j][1] = (quads[i].corners[j][1] - 0.5) / quad_decimate + 0.5;
          }
        } else {
          for (int j = 0; j < 4; j++) {
            quads[i].corners[j][0] /= quad_decimate;
            quads[i].corners[j][1] /= quad_decimate;
          }
        }
      }
    }
    
    // Mirror if requested
    if (horizontal_mirror) {
      detector.MirrorGrayImageOnGpu();
      for (auto& quad : quads) {
        for (int i = 0; i < 4; i++) {
          quad.corners[i][0] = gray_width - 1 - quad.corners[i][0];
        }
        float temp[2];
        temp[0] = quad.corners[0][0]; temp[1] = quad.corners[0][1];
        quad.corners[0][0] = quad.corners[1][0]; quad.corners[0][1] = quad.corners[1][1];
        quad.corners[1][0] = temp[0]; quad.corners[1][1] = temp[1];
        temp[0] = quad.corners[2][0]; temp[1] = quad.corners[2][1];
        quad.corners[2][0] = quad.corners[3][0]; quad.corners[2][1] = quad.corners[3][1];
        quad.corners[3][0] = temp[0]; quad.corners[3][1] = temp[1];
      }
    }
    
      vector<uint8_t> gray_host(gray_width * gray_height);
      detector.CopyGrayHostTo(gray_host);
      
      auto decode_start = chrono::steady_clock::now();
      acc_detect_ms += chrono::duration<double, milli>(decode_start - detect_start).count();
      
      // Decode tags
      zarray_t *detections = zarray_create(sizeof(apriltag_detection_t *));
      frc971::apriltag::DecodeTagsFromQuads(quads, gray_host.data(), gray_width, gray_height,
                                            td, cam, dist, detections, poly0, poly1);
      
      // Scale coordinates to full resolution
      if (quad_decimate > 1.0) {
        for (int i = 0; i < zarray_size(detections); i++) {
          apriltag_detection_t *det;
          zarray_get(detections, i, &det);
          if (quad_decimate != 1.5) {
            for (int j = 0; j < 4; j++) {
              det->p[j][0] = (det->p[j][0] - 0.5) * quad_decimate + 0.5;
              det->p[j][1] = (det->p[j][1] - 0.5) * quad_decimate + 0.5;
            }
            det->c[0] = (det->c[0] - 0.5) * quad_decimate + 0.5;
            det->c[1] = (det->c[1] - 0.5) * quad_decimate + 0.5;
          } else {
            for (int j = 0; j < 4; j++) {
              det->p[j][0] *= quad_decimate;
              det->p[j][1] *= quad_decimate;
            }
            det->c[0] *= quad_decimate;
            det->c[1] *= quad_decimate;
          }
        }
      }
      
      auto display_start = chrono::steady_clock::now();
      acc_decode_ms += chrono::duration<double, milli>(display_start - decode_start).count();
      
      // Draw detections
      Mat display;
      if (frame.channels() == 1) {
        cvtColor(frame, display, COLOR_GRAY2BGR);
      } else {
        display = frame.clone();
      }
      
      if (horizontal_mirror) {
        flip(display, display, 1);
      }
      
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        
        // Validate coordinates before drawing
        bool valid = true;
        for (int j = 0; j < 4; j++) {
          if (det->p[j][0] < 0 || det->p[j][0] >= display.cols ||
              det->p[j][1] < 0 || det->p[j][1] >= display.rows) {
            valid = false;
            break;
          }
        }
        if (!valid) continue;
        
        // Draw quad
        line(display, Point((int)det->p[0][0], (int)det->p[0][1]),
             Point((int)det->p[1][0], (int)det->p[1][1]), Scalar(0, 255, 0), 2);
        line(display, Point((int)det->p[1][0], (int)det->p[1][1]),
             Point((int)det->p[2][0], (int)det->p[2][1]), Scalar(0, 255, 0), 2);
        line(display, Point((int)det->p[2][0], (int)det->p[2][1]),
             Point((int)det->p[3][0], (int)det->p[3][1]), Scalar(0, 255, 0), 2);
        line(display, Point((int)det->p[3][0], (int)det->p[3][1]),
             Point((int)det->p[0][0], (int)det->p[0][1]), Scalar(0, 255, 0), 2);
        
        // Draw ID
        putText(display, to_string(det->id), Point((int)det->c[0], (int)det->c[1]),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 255, 0), 2);
      }
      
      // Draw stats
      stringstream ss;
      ss << "FPS: " << fixed << setprecision(1) << (1000.0 / (acc_detect_ms + acc_decode_ms + acc_display_ms) * (frame_num + 1));
      ss << " | Detections: " << zarray_size(detections);
      putText(display, ss.str(), Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 255), 2);
      
      try {
        imshow("AprilTag Detection (GPU)", display);
      } catch (const cv::Exception& e) {
        cerr << "OpenCV error: " << e.what() << endl;
        break;
      }
      
      auto display_end = chrono::steady_clock::now();
      acc_display_ms += chrono::duration<double, milli>(display_end - display_start).count();
      
      // Cleanup
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        apriltag_detection_destroy(det);
      }
      zarray_destroy(detections);
      
    } catch (const std::exception& e) {
      cerr << "Error in GPU detection: " << e.what() << endl;
      continue;
    }
    
    frame_num++;
    
    // Print stats every 100 frames
    if (frame_num % 100 == 0) {
      double avg_detect = acc_detect_ms / frame_num;
      double avg_decode = acc_decode_ms / frame_num;
      double avg_display = acc_display_ms / frame_num;
      double total_fps = 1000.0 / (avg_detect + avg_decode + avg_display);
      cout << "Frame " << frame_num << ": Detect=" << fixed << setprecision(2) << avg_detect 
           << "ms, Decode=" << avg_decode << "ms, Display=" << avg_display 
           << "ms, Total FPS=" << total_fps << "\r" << flush;
    }
    
    char key = waitKey(1) & 0xFF;
    if (key == 'q' || key == 27) break; // 'q' or ESC to quit
  }
  
  // Cleanup
  if (use_mvsdk && mv_camera_handle != 0) {
    CameraStop(mv_camera_handle);
    CameraUnInit(mv_camera_handle);
  } else {
    cap.release();
  }
  
  workerpool_destroy(td->wp);
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  zarray_destroy(poly0);
  zarray_destroy(poly1);
  
  auto t_end = chrono::steady_clock::now();
  double total_s = chrono::duration<double>(t_end - t_start).count();
  double avg_fps = frame_num / total_s;
  
  cout << "\n\nProcessed " << frame_num << " frames in " << fixed << setprecision(2) << total_s 
       << " seconds\n";
  cout << "Average FPS: " << avg_fps << "\n";
  
  return 0;
}

