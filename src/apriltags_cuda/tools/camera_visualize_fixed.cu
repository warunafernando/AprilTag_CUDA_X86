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
#pragma diag_suppress 611  // Suppress MultiBandBlender override warning from OpenCV when compiling with nvcc
#endif

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"
#include "config_parser.h"  // For reading config.txt

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
}
#include "g2d.h"

// MindVision SDK support
#include "CameraApi.h"

// Shared memory IPC support
#include "TagDetectionIPC.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
using namespace cv;

struct DrawItem {
  Mat color;  // pre-drawn BGR frame ready to display/write
};

struct FrameItem {
  Mat gray;
  size_t idx;
};

// External CPU decode helper from apriltag_detect.cu, reused here for the
// 2-stage pipeline (GPU quads -> CPU decode).
namespace frc971::apriltag {
void DecodeTagsFromQuads(const std::vector<QuadCorners> &quad_corners,
                         const uint8_t *gray_buf, int width, int height,
                         apriltag_detector_t *td,
                         const CameraMatrix &camera_matrix,
                         const DistCoeffs &distortion_coefficients,
                         zarray_t *detections,
                         zarray_t *poly0,
                         zarray_t *poly1);
}  // namespace frc971::apriltag

struct TagPoseInfo {
  int id;
  double x, y, z;  // translation components (in meters)
  double decision_margin;
};

// Calculate distance between two detection centers
double detection_distance(apriltag_detection_t *det1, apriltag_detection_t *det2) {
  double dx = det1->c[0] - det2->c[0];
  double dy = det1->c[1] - det2->c[1];
  return sqrt(dx * dx + dy * dy);
}

// Scale detection coordinates from decimated space to full resolution
void scale_detection_coordinates(apriltag_detection_t *det, double decimate_factor) {
  if (decimate_factor <= 1.0) return;
  
  if (decimate_factor != 1.5) {
    // Standard scaling: (x - 0.5) * factor + 0.5
    for (int j = 0; j < 4; j++) {
      det->p[j][0] = (det->p[j][0] - 0.5) * decimate_factor + 0.5;
      det->p[j][1] = (det->p[j][1] - 0.5) * decimate_factor + 0.5;
    }
    det->c[0] = (det->c[0] - 0.5) * decimate_factor + 0.5;
    det->c[1] = (det->c[1] - 0.5) * decimate_factor + 0.5;
  } else {
    // For 1.5x decimation, simple multiplication
    for (int j = 0; j < 4; j++) {
      det->p[j][0] *= decimate_factor;
      det->p[j][1] *= decimate_factor;
    }
    det->c[0] *= decimate_factor;
    det->c[1] *= decimate_factor;
  }
}

// Check if detection coordinates are valid (within image bounds)
bool is_valid_detection(apriltag_detection_t *det, int width, int height) {
  // Check center coordinates
  if (det->c[0] < 0 || det->c[0] >= width || det->c[1] < 0 || det->c[1] >= height) {
    return false;
  }
  // Check all corner coordinates
  for (int i = 0; i < 4; i++) {
    if (det->p[i][0] < 0 || det->p[i][0] >= width || 
        det->p[i][1] < 0 || det->p[i][1] >= height) {
      return false;
    }
  }
  return true;
}

// Mirror detection coordinates horizontally (flip x coordinates)
void mirror_detection_coordinates(apriltag_detection_t *det, int width) {
  // Mirror center coordinates
  det->c[0] = width - 1 - det->c[0];
  
  // Mirror all corner coordinates and swap to maintain correct orientation
  // After horizontal flip: top-left becomes top-right, top-right becomes top-left
  // So we swap p[0] with p[1], and p[2] with p[3]
  double temp[2];
  
  // First mirror all x coordinates
  for (int i = 0; i < 4; i++) {
    det->p[i][0] = width - 1 - det->p[i][0];
  }
  
  // Then swap corners to maintain correct orientation after horizontal mirror
  // Swap top corners: p[0] <-> p[1]
  temp[0] = det->p[0][0]; temp[1] = det->p[0][1];
  det->p[0][0] = det->p[1][0]; det->p[0][1] = det->p[1][1];
  det->p[1][0] = temp[0]; det->p[1][1] = temp[1];
  
  // Swap bottom corners: p[2] <-> p[3]
  temp[0] = det->p[2][0]; temp[1] = det->p[2][1];
  det->p[2][0] = det->p[3][0]; det->p[2][1] = det->p[3][1];
  det->p[3][0] = temp[0]; det->p[3][1] = temp[1];
}

// Filter duplicate detections - keep the one with best decision margin
// For same ID, keep only the best one that is also valid (within image bounds)
vector<apriltag_detection_t*> filter_duplicates(const zarray_t *detections, int width, int height, double min_distance = 50.0) {
  vector<apriltag_detection_t*> filtered;
  vector<apriltag_detection_t*> all_dets;
  
  // Extract all detections
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(const_cast<zarray_t *>(detections), i, &det);
    all_dets.push_back(det);
  }
  
  // Group by tag ID and keep only the best VALID one per ID
  map<int, apriltag_detection_t*> best_by_id;
  
  for (auto *det : all_dets) {
    // Only consider valid detections (within image bounds)
    if (!is_valid_detection(det, width, height)) {
      continue;
    }
    
    auto it = best_by_id.find(det->id);
    if (it == best_by_id.end()) {
      // First valid detection with this ID
      best_by_id[det->id] = det;
    } else {
      // Compare decision margins - keep the one with higher margin
      if (det->decision_margin > it->second->decision_margin) {
        best_by_id[det->id] = det;
      }
    }
  }
  
  // Convert map to vector
  for (auto &pair : best_by_id) {
    filtered.push_back(pair.second);
  }
  
  return filtered;
}

// Draw 3D axes on detected tag and return pose information
// Returns true if pose estimation succeeded, false otherwise
bool draw_3d_axes(Mat &im, apriltag_detection_t *det, 
                  const frc971::apriltag::CameraMatrix &cam,
                  const frc971::apriltag::DistCoeffs &dist,
                  double tag_size,
                  TagPoseInfo &pose_info) {
  pose_info.id = det->id;
  pose_info.decision_margin = det->decision_margin;
  pose_info.x = pose_info.y = pose_info.z = 0.0;

  // Convert camera matrix and distortion to OpenCV format
  Mat camera_matrix = (Mat_<double>(3, 3) <<
    cam.fx, 0, cam.cx,
    0, cam.fy, cam.cy,
    0, 0, 1);
  
  Mat dist_coeffs = (Mat_<double>(5, 1) <<
    dist.k1, dist.k2, dist.p1, dist.p2, dist.k3);

  // Object points in tag coordinate frame (centered, z = 0)
  double s = tag_size * 0.5;
  vector<Point3f> object_points = {
      Point3f(-s, -s, 0),
      Point3f( s, -s, 0),
      Point3f( s,  s, 0),
      Point3f(-s,  s, 0)};

  // Image points from detection
  vector<Point2f> image_points = {
      Point2f(det->p[0][0], det->p[0][1]),
      Point2f(det->p[1][0], det->p[1][1]),
      Point2f(det->p[2][0], det->p[2][1]),
      Point2f(det->p[3][0], det->p[3][1])};

  Mat rvec, tvec;
  bool ok = solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);
  if (!ok) {
    // If pose estimation failed, just draw outline
    line(im, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[3][0], det->p[3][1]),
         Point(det->p[0][0], det->p[0][1]), Scalar(0, 255, 255), 2);
    // Draw tag ID only
    stringstream ss;
    ss << "ID:" << det->id;
    putText(im, ss.str(), Point(det->c[0] - 30, det->c[1] - 10),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
    return false;
  }

  // Use OpenCV's drawFrameAxes to draw 3D axes
  drawFrameAxes(im, camera_matrix, dist_coeffs, rvec, tvec, tag_size * 0.5, 3);

  // Draw tag outline
  line(im, Point(det->p[0][0], det->p[0][1]),
       Point(det->p[1][0], det->p[1][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[1][0], det->p[1][1]),
       Point(det->p[2][0], det->p[2][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[2][0], det->p[2][1]),
       Point(det->p[3][0], det->p[3][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[3][0], det->p[3][1]),
       Point(det->p[0][0], det->p[0][1]), Scalar(0, 255, 255), 2);

  // Draw tag ID only (no distance/margin text)
  stringstream ss;
  ss << "ID:" << det->id;
  putText(im, ss.str(), Point(det->c[0] - 30, det->c[1] - 10),
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

  // Extract pose information
  pose_info.x = tvec.at<double>(0);
  pose_info.y = tvec.at<double>(1);
  pose_info.z = tvec.at<double>(2);

  return true;
}

// Draw information table in top-left corner
void draw_info_table(Mat &im, const vector<TagPoseInfo> &tags, double fps) {
  const int table_width = 220;
  const int table_x = 10;  // Position on left side
  const int start_y = 10;
  const int line_height = 25;
  const double font_scale = 0.38;
  const int thickness = 1;
  const Scalar text_color(255, 255, 255);
  const Scalar bg_color(0, 0, 0);

  // Calculate table height
  int num_rows = 2 + tags.size();  // Header + FPS + tags
  int table_height = num_rows * line_height + 10;
  
  // Draw background rectangle with border
  rectangle(im, Point(table_x - 5, start_y - 5), 
            Point(table_x + table_width, start_y + table_height), 
            bg_color, -1);
  rectangle(im, Point(table_x - 5, start_y - 5), 
            Point(table_x + table_width, start_y + table_height), 
            text_color, 1);

  // Draw FPS header
  int y = start_y + 20;
  stringstream fps_ss;
  fps_ss << "FPS: " << fixed << setprecision(1) << fps;
  putText(im, fps_ss.str(), Point(table_x, y), 
          FONT_HERSHEY_SIMPLEX, 0.45, text_color, thickness);

  // Draw column headers
  y = start_y + 45;
  putText(im, "ID", Point(table_x, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  putText(im, "X", Point(table_x + 35, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  putText(im, "Y", Point(table_x + 70, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  putText(im, "Z", Point(table_x + 105, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  putText(im, "Dist", Point(table_x + 140, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
  putText(im, "Prob", Point(table_x + 175, y), 
          FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);

  // Draw horizontal line under headers
  line(im, Point(table_x - 5, y + 12), Point(table_x + table_width, y + 12), text_color, 1);

  // Draw each tag's information
  y = start_y + 70;
  for (const auto &tag : tags) {
    // Calculate distance
    double distance = sqrt(tag.x * tag.x + tag.y * tag.y + tag.z * tag.z);
    
    stringstream ss;
    ss << tag.id;
    putText(im, ss.str(), Point(table_x, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    ss.str("");
    ss << fixed << setprecision(2) << tag.x;
    putText(im, ss.str(), Point(table_x + 35, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    ss.str("");
    ss << fixed << setprecision(2) << tag.y;
    putText(im, ss.str(), Point(table_x + 70, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    ss.str("");
    ss << fixed << setprecision(2) << tag.z;
    putText(im, ss.str(), Point(table_x + 105, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    ss.str("");
    ss << fixed << setprecision(2) << distance;
    putText(im, ss.str(), Point(table_x + 140, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    ss.str("");
    // Normalize decision margin to 0-1 range (divide by 150 as typical max for good detections)
    double normalized_prob = tag.decision_margin / 150.0;
    if (normalized_prob > 1.0) normalized_prob = 1.0;  // Cap at 1.0
    ss << fixed << setprecision(2) << normalized_prob;
    putText(im, ss.str(), Point(table_x + 175, y), 
            FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    
    y += line_height;
  }
}

int main(int argc, char **argv) {
  string video_path;
  string output_path;
  string family = "tag36h11";
  double tag_size = 0.305; // Tag size in meters (1 foot = 0.305m, adjust as needed)
  double min_distance = 50.0; // Minimum distance between detections to consider them different
  bool horizontal_mirror = true; // DEFAULT: Enable horizontal mirror/flip for detection (required for tag detection)
  // DEFAULT: Use MindVision camera (index 0) by default
  int camera_index = 0;
  bool use_mvsdk = true;  // DEFAULT: Use MindVision SDK
  bool no_display = false;
  bool use_frames_shm = false;

  for (int i = 1; i < argc; ++i) {
    string arg(argv[i]);
    if ((arg == "--video" || arg == "-v") && i + 1 < argc) {
      video_path = argv[++i];
    } else if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
      output_path = argv[++i];
    } else if ((arg == "--family" || arg == "-f") && i + 1 < argc) {
      family = argv[++i];
    } else if (arg == "--tag_size" && i + 1 < argc) {
      tag_size = atof(argv[++i]);
    } else if (arg == "--min_distance" && i + 1 < argc) {
      min_distance = atof(argv[++i]);
    } else if ((arg == "--camera" || arg == "-c") && i + 1 < argc) {
      camera_index = atoi(argv[++i]);
    } else if (arg == "--mvsdk") {
      use_mvsdk = true;
    } else if (arg == "--no-display") {
      no_display = true;
    } else if (arg == "--frames-shm") {
      use_frames_shm = true;
    } else if (arg == "--mirror" || arg == "-m") {
      horizontal_mirror = true;
    } else if (arg == "--help" || arg == "-h") {
      cout << "Usage: camera_visualize_fixed [--video <input.avi> | --camera <index> [--mvsdk] | --frames-shm] "
              "[--output <output.avi>] [--no-display] [--mirror] "
              "[--family <tag36h11>] [--tag_size <0.1>] [--min_distance <50.0>]\n";
      cout << "\nDefaults: --camera 0 --mvsdk (MindVision camera)\n";
      return 0;
    }
  }

  // No error check needed - we default to camera_index = 0 and use_mvsdk = true
  // If user explicitly wants video or frames-shm, they can still specify it

  // Load configuration from config.txt
  ConfigParser config("config.txt");
  // Config-driven overrides with sensible defaults
  family = config.get_string("detector.family", family);
  tag_size = config.get_double("tag_size_meters", tag_size);
  min_distance = config.get_double("filtering.min_distance_for_duplicates", min_distance);
  const double cfg_fx = config.get_double("camera.fx", 905.495617);
  const double cfg_fy = config.get_double("camera.fy", 609.916016);
  const double cfg_cx = config.get_double("camera.cx", 907.909470);
  const double cfg_cy = config.get_double("camera.cy", 352.682645);
  const double cfg_k1 = config.get_double("distortion.k1", 0.059238);
  const double cfg_k2 = config.get_double("distortion.k2", -0.075154);
  const double cfg_p1 = config.get_double("distortion.p1", -0.003801);
  const double cfg_p2 = config.get_double("distortion.p2", 0.001113);
  const double cfg_k3 = config.get_double("distortion.k3", 0.0);
  const bool show_before_filter =
      config.get_bool("reporting.show_before_filter", false);
  const bool prefetch_enabled = config.get_bool("prefetching.enabled", false);
  const int cfg_prefetch_q = std::max(1, config.get_int("prefetching.queue_size", 2));
  const bool prefetch_drop_oldest = config.get_bool("prefetching.drop_oldest", true);
  const int cfg_writer_q = std::max(1, config.get_int("writer.queue_size", 5));
  const bool writer_drop_oldest = config.get_bool("writer.drop_oldest", true);

  // Camera handle for MindVision SDK
  CameraHandle mv_camera_handle = 0;
  unsigned char *mv_rgb_buffer = nullptr;
  tSdkCameraCapbility mv_capability;
  bool is_mv_mono = false;
  
  // Shared memory for frames (when use_frames_shm is true)
  FrameIPCData* frame_shm = nullptr;
  int frame_shm_fd = -1;
  uint32_t last_frame_id = 0;
  
  // VideoCapture for video files and regular cameras
  VideoCapture cap;
  
  int width, height;
  double fps = 30.0;
  Mat frame;
  
  if (!video_path.empty()) {
    // Use OpenCV VideoCapture for video file
    cap.open(video_path, CAP_ANY);
    if (!cap.isOpened()) {
      cerr << "Failed to open video: " << video_path << endl;
      return 1;
    }

    // CRITICAL: Don't convert to RGB - read grayscale directly for better performance
    cap.set(CAP_PROP_CONVERT_RGB, false);

    // Get video properties
    width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) {
      fps = 30.0; // Default if FPS not available
      cout << "FPS not available, using default: 30.0 FPS\n";
    } else {
      cout << "Using video FPS from metadata: " << fixed << setprecision(2) << fps << "\n";
    }

    // Read first frame to verify (backend should give us grayscale)
    if (!cap.read(frame)) {
      cerr << "Could not read first frame\n";
      return 1;
    }
    
    if (frame.empty()) {
      cerr << "First frame is empty\n";
      return 1;
    }
    
    // Ensure grayscale
    if (frame.type() != CV_8UC1) {
      if (frame.channels() == 2) {
        cvtColor(frame, frame, COLOR_YUV2GRAY_YUY2);
      } else if (frame.channels() == 3) {
        cvtColor(frame, frame, COLOR_BGR2GRAY);
      }
    }
    cout << "First frame size: " << frame.cols << "x" << frame.rows 
         << ", type: " << frame.type() << " (grayscale)" << endl;
  } else if (use_mvsdk && camera_index >= 0) {
    // Use MindVision SDK for camera
    cout << "Initializing MindVision SDK..." << endl;
    CameraSdkStatus status = CameraSdkInit(1);
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
    
    if (camera_index >= camera_count) {
      cerr << "Error: Invalid camera index " << camera_index << " (found " << camera_count << " cameras)" << endl;
      return 1;
    }
    
    cout << "Opening MindVision camera index " << camera_index << "..." << endl;
    status = CameraInit(&camera_list[camera_index], -1, -1, &mv_camera_handle);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: CameraInit failed with status " << status << endl;
      return 1;
    }
    
    // Get capability
    CameraGetCapability(mv_camera_handle, &mv_capability);
    is_mv_mono = mv_capability.sIspCapacity.bMonoSensor;
    
    // Allocate RGB buffer only if not monochrome
    int max_width = mv_capability.sResolutionRange.iWidthMax;
    int max_height = mv_capability.sResolutionRange.iHeightMax;
    if (!is_mv_mono) {
      mv_rgb_buffer = static_cast<unsigned char *>(
          malloc(static_cast<size_t>(max_width) * static_cast<size_t>(max_height) * 3));
    }
    
    // Set output format to MONO8 (grayscale) - force grayscale for AprilTag
    cout << "Setting camera to MONO8 (grayscale) format..." << endl;
    CameraSetIspOutFormat(mv_camera_handle, CAMERA_MEDIA_TYPE_MONO8);
    
    // Set resolution (use 1280x1024 for high FPS)
    width = 1280;
    height = 1024;
    cout << "Setting resolution to " << width << "x" << height << "..." << endl;
    status = CameraSetImageResolutionEx(mv_camera_handle, 0xff, 0, 0, 0, 0, width, height, 0, 0);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Warning: CameraSetImageResolutionEx returned status " << status << endl;
    }
    
    // Set frame speed to highest available
    int max_frame_speed = mv_capability.iFrameSpeedDesc - 1;
    if (max_frame_speed < 0) max_frame_speed = 0;
    if (max_frame_speed > FRAME_SPEED_SUPER) max_frame_speed = FRAME_SPEED_SUPER;
    CameraSetFrameSpeed(mv_camera_handle, max_frame_speed);
    
    // Load camera settings from file if available
    // Defaults: exposure slider=50 (maps to ~50000us), gain=0, brightness=128
    int exposure_slider = 50; // Slider value 0-100, will be converted to exposure time
    int gain = 0; // Slider value 0-100
    int brightness = 128; // Slider value 0-255
    
    FILE *settings_file = fopen("camera_settings.txt", "r");
    if (settings_file) {
        cout << "Loading camera settings from camera_settings.txt (exposure, gain, brightness only)..." << endl;
        char line[256];
        bool in_settings = false;
        while (fgets(line, sizeof(line), settings_file)) {
            // Skip comments and empty lines
            if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
            
            // Check for [Settings] section
            if (strstr(line, "[Settings]")) {
                in_settings = true;
                continue;
            }
            if (strstr(line, "[")) {
                in_settings = false;
                continue;
            }
            
            if (in_settings) {
                if (strncmp(line, "exposure=", 9) == 0) {
                    exposure_slider = atoi(line + 9);
                    cout << "  Loaded exposure slider value: " << exposure_slider << " (0-100)" << endl;
                } else if (strncmp(line, "gain=", 5) == 0) {
                    gain = atoi(line + 5);
                    cout << "  Loaded gain: " << gain << " (0-100)" << endl;
                } else if (strncmp(line, "brightness=", 11) == 0) {
                    brightness = atoi(line + 11);
                    cout << "  Loaded brightness: " << brightness << " (0-255)" << endl;
                }
                // Ignore contrast, saturation, sharpness
            }
        }
        fclose(settings_file);
        cout << "Camera settings loaded successfully." << endl;
    } else {
        cout << "No camera_settings.txt found, using default settings" << endl;
    }
    
    // Apply camera settings (matching GUI implementation)
    CameraSetAeState(mv_camera_handle, FALSE);
    
    // Exposure: Map slider (0-100) to exposure time (100000 to 1000 microseconds)
    double min_exposure = 1000.0;
    double max_exposure = 100000.0;
    double exposure_time = max_exposure - (exposure_slider / 100.0) * (max_exposure - min_exposure);
    CameraSetExposureTime(mv_camera_handle, exposure_time);
    cout << "Applied exposure: " << exposure_time << " microseconds (from slider value " << exposure_slider << ")" << endl;
    
    // Gain: RGB gains (same value for all channels)
    if (gain >= 0) {
        CameraSetGain(mv_camera_handle, gain, gain, gain);
        cout << "Applied gain: " << gain << endl;
    }
    
    // Brightness: Map slider (0-255) to analog gain (0-100)
    if (brightness >= 0) {
        INT analogGain = (brightness * 100) / 255;
        CameraSetAnalogGain(mv_camera_handle, analogGain);
        cout << "Applied brightness (analog gain): " << analogGain << " (from slider value " << brightness << ")" << endl;
    }
    
    // Start capture
    CameraPlay(mv_camera_handle);
    
    // Allow camera to start streaming
    this_thread::sleep_for(chrono::milliseconds(200));
    
    // Read first frame
    BYTE *pbyBuffer = nullptr;
    tSdkFrameHead sFrameInfo;
    status = CameraGetImageBuffer(mv_camera_handle, &sFrameInfo, &pbyBuffer, 100);
    if (status != CAMERA_STATUS_SUCCESS) {
      cerr << "Error: Failed to get first frame from camera (status " << status << ")" << endl;
      CameraUnInit(mv_camera_handle);
      return 1;
    }
    
    // Create Mat from frame data (MONO8 format)
    width = sFrameInfo.iWidth;
    height = sFrameInfo.iHeight;
    frame = Mat(height, width, CV_8UC1, pbyBuffer, sFrameInfo.iWidth);
    frame = frame.clone(); // Clone to own the data
    
    CameraReleaseImageBuffer(mv_camera_handle, pbyBuffer);
    
    cout << "Camera initialized: " << width << "x" << height 
         << ", format: MONO8 (grayscale)" << endl;
    fps = 211.0; // Approximate FPS for this mode
  } else if (camera_index >= 0) {
    // Use OpenCV VideoCapture for regular camera
    cap.open(camera_index, CAP_ANY);
    if (!cap.isOpened()) {
      cerr << "Failed to open camera index " << camera_index << endl;
      return 1;
    }
    
    cap.set(CAP_PROP_CONVERT_RGB, false);
    
    width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
    height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30.0;
    
    if (!cap.read(frame)) {
      cerr << "Could not read first frame from camera\n";
      return 1;
    }
    
    // Ensure grayscale
    if (frame.type() != CV_8UC1) {
      if (frame.channels() == 2) {
        cvtColor(frame, frame, COLOR_YUV2GRAY_YUY2);
      } else if (frame.channels() == 3) {
        cvtColor(frame, frame, COLOR_BGR2GRAY);
      }
    }
    
    cout << "Camera opened: " << width << "x" << height 
         << ", type: " << frame.type() << " (grayscale)" << endl;
  } else if (use_frames_shm) {
    // Read frames from shared memory (GUI writes frames here)
    cout << "Reading frames from shared memory..." << endl;
    
    // Open shared memory for frames
    frame_shm_fd = shm_open(FRAME_IPC_KEY, O_RDONLY, 0666);
    if (frame_shm_fd < 0) {
      cerr << "Error: Failed to open shared memory for frames: " << FRAME_IPC_KEY << endl;
      return 1;
    }
    
    frame_shm = static_cast<FrameIPCData*>(
        mmap(0, sizeof(FrameIPCData), PROT_READ, MAP_SHARED, frame_shm_fd, 0));
    if (frame_shm == MAP_FAILED) {
      cerr << "Error: Failed to map shared memory for frames" << endl;
      ::close(frame_shm_fd);
      return 1;
    }
    
    // Wait for first frame to get dimensions and valid frame_id
    cout << "Waiting for first frame from GUI..." << endl;
    int wait_count = 0;
    uint32_t initial_frame_id = 0;
    while (frame_shm->width == 0 || frame_shm->height == 0 || frame_shm->frame_id == 0) {
      this_thread::sleep_for(chrono::milliseconds(100));
      wait_count++;
      if (wait_count > 100) {  // 10 seconds timeout
        cerr << "Error: Timeout waiting for first frame from GUI" << endl;
        munmap(frame_shm, sizeof(FrameIPCData));
        ::close(frame_shm_fd);
        return 1;
      }
    }
    
    width = frame_shm->width;
    height = frame_shm->height;
    fps = 30.0; // Default FPS for shared memory frames
    initial_frame_id = frame_shm->frame_id;
    
    // Wait a bit more to ensure frame data is fully written
    this_thread::sleep_for(chrono::milliseconds(100));
    
    // Create Mat from shared memory data - ensure we have a valid frame_id
    if (frame_shm->frame_id == initial_frame_id && frame_shm->frame_id > 0) {
      frame = Mat(height, width, CV_8UC1, (void*)frame_shm->data, width);
      frame = frame.clone(); // Clone to own the data
      last_frame_id = frame_shm->frame_id;
      
      // Validate the frame before proceeding
      if (frame.empty() || frame.data == nullptr || frame.cols != width || frame.rows != height) {
        cerr << "Error: Invalid initial frame from shared memory" << endl;
        munmap(frame_shm, sizeof(FrameIPCData));
        ::close(frame_shm_fd);
        return 1;
      }
    } else {
      cerr << "Error: Frame ID changed before we could read initial frame" << endl;
      munmap(frame_shm, sizeof(FrameIPCData));
      ::close(frame_shm_fd);
      return 1;
    }
    
    cout << "Shared memory initialized: " << width << "x" << height 
         << " (grayscale from GUI), frame_id=" << initial_frame_id << endl;
  }

  // Setup detector
  apriltag_family_t *tf = nullptr;
  setup_tag_family(&tf, family.c_str());
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  // Respect config but enforce kernel constraint on quad_decimate (must be 2.0)
  const double cfg_decimate = config.get_double("detector.quad_decimate", 2.0);
  if (fabs(cfg_decimate - 2.0) > 1e-6) {
    cout << "Warning: quad_decimate is forced to 2.0 (config requested " << cfg_decimate
         << ") due to CUDA kernel constraint.\n";
  }
  td->quad_decimate = 2.0;
  td->quad_sigma = config.get_double("detector.quad_sigma", 0.0);
  td->nthreads = std::max(1, config.get_int("detector.nthreads", 1));
  td->debug = config.get_bool("detector.debug", false);
  td->refine_edges = config.get_bool("detector.refine_edges", true);
  td->wp = workerpool_create(td->nthreads);

  frc971::apriltag::CameraMatrix cam{cfg_fx, cfg_fy, cfg_cx, cfg_cy};
  frc971::apriltag::DistCoeffs dist{cfg_k1, cfg_k2, cfg_p1, cfg_p2, cfg_k3};

  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
  // Leave GPU decode debug disabled by default; enable manually if needed.
  // detector.SetGpuDecodeDebug(true);

  const bool write_enabled = !output_path.empty();
  VideoWriter writer;
  if (write_enabled) {
    writer.open(output_path, VideoWriter::fourcc('X', 'V', 'I', 'D'),
                fps, Size(width, height), true);
    if (!writer.isOpened()) {
      cerr << "Failed to open output video: " << output_path << endl;
      return 1;
    }
  }

  // Shared memory for tag detections (write to GUI)
  TagDetectionData* tag_shm = nullptr;
  int tag_shm_fd = -1;
  // Shared memory for annotated frames (write to GUI)
  AnnotatedFrameIPCData* annotated_frame_shm = nullptr;
  int annotated_frame_shm_fd = -1;
  std::atomic<uint32_t> annotated_frame_id_counter{0};
  
  if (!no_display || use_frames_shm) {
    // Create shared memory for tag detections
    tag_shm_fd = shm_open(TAG_IPC_KEY, O_CREAT | O_RDWR, 0666);
    if (tag_shm_fd >= 0) {
      if (ftruncate(tag_shm_fd, sizeof(TagDetectionData)) == 0) {
        void* mem = mmap(0, sizeof(TagDetectionData), PROT_READ | PROT_WRITE, MAP_SHARED, tag_shm_fd, 0);
        if (mem != MAP_FAILED) {
          tag_shm = static_cast<TagDetectionData*>(mem);
          tag_shm->clear();
        } else {
          ::close(tag_shm_fd);
          tag_shm_fd = -1;
        }
      } else {
        ::close(tag_shm_fd);
        tag_shm_fd = -1;
      }
    }
    
    // Create shared memory for annotated frames (when using --frames-shm)
    if (use_frames_shm) {
      annotated_frame_shm_fd = shm_open(ANNOTATED_FRAME_IPC_KEY, O_CREAT | O_RDWR, 0666);
      if (annotated_frame_shm_fd >= 0) {
        if (ftruncate(annotated_frame_shm_fd, sizeof(AnnotatedFrameIPCData)) == 0) {
          void* mem = mmap(0, sizeof(AnnotatedFrameIPCData), PROT_READ | PROT_WRITE, MAP_SHARED, annotated_frame_shm_fd, 0);
          if (mem != MAP_FAILED) {
            annotated_frame_shm = static_cast<AnnotatedFrameIPCData*>(mem);
            annotated_frame_shm->frame_id = 0;
            annotated_frame_shm->width = 0;
            annotated_frame_shm->height = 0;
            annotated_frame_shm->channels = 3;
          } else {
            ::close(annotated_frame_shm_fd);
            annotated_frame_shm_fd = -1;
          }
        } else {
          ::close(annotated_frame_shm_fd);
          annotated_frame_shm_fd = -1;
        }
      }
    }
  }

  size_t frame_num = 0;
  auto t_start = chrono::steady_clock::now();
  vector<double> frame_times;
  int total_detections_before = 0;
  int total_detections_after = 0;
  // Histogram of detections per frame (before/after filtering)
  std::map<int, int64_t> det_hist_before;
  std::map<int, int64_t> det_hist_after;
  // Timing accumulators (milliseconds)
  double acc_read_ms = 0.0;
  double acc_detect_ms = 0.0;
  double acc_cuda_ms = 0.0;
  double acc_mirror_ms = 0.0;
  double acc_cpu_decode_ms = 0.0;
  double acc_scale_ms = 0.0;
  double acc_filter_ms = 0.0;
  double acc_draw_ms = 0.0;
  // Track detector cumulative timings to get per-frame deltas
  double prev_cuda_total = 0.0;
  double prev_cpu_total = 0.0;
  // Writer thread timing (ms)
  double writer_write_ms = 0.0;
  size_t writer_frames = 0;
  std::atomic<double> acc_read_ms_atomic{0.0};

  if (!video_path.empty()) {
    cout << "Processing video: " << video_path << endl;
  } else if (use_frames_shm) {
    cout << "Processing frames from shared memory (GUI)" << endl;
  } else if (use_mvsdk) {
    cout << "Processing from MindVision camera (index " << camera_index << ")" << endl;
  } else {
    cout << "Processing from camera (index " << camera_index << ")" << endl;
  }
  if (!output_path.empty()) {
    cout << "Output: " << output_path << endl;
  }
  cout << "Resolution: " << width << "x" << height << " @ " << fps << " FPS\n";
  cout << "Min distance for duplicate filtering: " << min_distance << " pixels\n";

  // Thread-safe queue for draw/display/write (final rendered frames)
  const size_t writer_queue_size = static_cast<size_t>(cfg_writer_q);
  std::deque<DrawItem> draw_queue;
  std::mutex draw_mtx;
  std::condition_variable draw_cv;
  bool display_done = false;

  // Thread-safe queue for frame prefetch
  const size_t prefetch_queue_size = static_cast<size_t>(cfg_prefetch_q);
  std::deque<FrameItem> frame_queue;
  std::mutex fq_mtx;
  std::condition_variable fq_cv;
  bool reader_done = false;

  // Thread-safe queue for GPU->CPU decode stage (2-stage pipeline)
  struct DecodeJob {
    std::vector<frc971::apriltag::QuadCorners> quads;
    // Grayscale image used for CPU decode (GPU-produced gray buffer).
    std::vector<uint8_t> gray;
    int width;
    int height;
    // Original input grayscale frame used only for visualization, so the user
    // always sees the real video frame rather than any GPU-internal layout.
    std::vector<uint8_t> display_gray;
    int disp_width;
    int disp_height;
    size_t frame_index;
    double fps_estimate;
  };
  const size_t decode_queue_size =
      static_cast<size_t>(std::max(1, config.get_int("decode.queue_size", 4)));
  const bool decode_drop_oldest =
      config.get_bool("decode.drop_oldest", true);
  std::deque<DecodeJob> decode_queue;
  std::mutex dq_mtx;
  std::condition_variable dq_cv;
  bool decode_done = false;

  // Display/write thread: owns imshow/waitKey and optionally VideoWriter
  std::thread display_thread([&]() {
    while (true) {
      DrawItem item;
      {
        std::unique_lock<std::mutex> lk(draw_mtx);
        draw_cv.wait(lk, [&]() { return display_done || !draw_queue.empty(); });
        if (draw_queue.empty()) {
          if (display_done) {
            break;
          } else {
            continue;  // wait again
          }
        }
        item = std::move(draw_queue.front());
        draw_queue.pop_front();
      }

      // Display on screen (unless disabled)
      if (!no_display) {
        imshow("AprilTags", item.color);
        waitKey(1);
      }

      // Optional video writing (non-blocking for detector thread)
      if (write_enabled) {
        auto write_start = chrono::steady_clock::now();
        writer.write(item.color);
        auto write_end = chrono::steady_clock::now();

        writer_write_ms +=
            chrono::duration<double, milli>(write_end - write_start).count();
        writer_frames++;
      }
    }
  });

  // Decode + draw thread: consumes GPU-produced quads/gray images and produces
  // fully rendered color frames, then enqueues them to the display/write
  // thread. This is Stage 2 of the 2-stage pipeline.
  std::thread decode_thread([&]() {
    // Local APRILTAG detector and family for CPU decode stage.
    apriltag_family_t *tf_cpu = nullptr;
    setup_tag_family(&tf_cpu, family.c_str());
    apriltag_detector_t *td_cpu = apriltag_detector_create();
    apriltag_detector_add_family(td_cpu, tf_cpu);
    td_cpu->quad_decimate = td->quad_decimate;
    td_cpu->quad_sigma = td->quad_sigma;
    td_cpu->refine_edges = td->refine_edges;
    td_cpu->debug = td->debug;
    td_cpu->nthreads = td->nthreads;
    td_cpu->wp = workerpool_create(td_cpu->nthreads);

    zarray_t *poly0 = g2d_polygon_create_zeros(4);
    zarray_t *poly1 = g2d_polygon_create_zeros(4);
    zarray_t *detections = zarray_create(sizeof(apriltag_detection_t *));

    while (true) {
      DecodeJob job;
      {
        std::unique_lock<std::mutex> lk(dq_mtx);
        dq_cv.wait(lk, [&]() { return decode_done || !decode_queue.empty(); });
        if (decode_queue.empty()) {
          if (decode_done) break;
          else continue;
        }
        job = std::move(decode_queue.front());
        decode_queue.pop_front();
      }

      // Mirroring is now done on GPU (in main thread after DetectGpuOnly, before decode thread)
      // Gray image and quads are already mirrored when they reach here
      
      // Debug: print quad count periodically
      static int debug_frame_count = 0;
      if (++debug_frame_count % 100 == 0) {
        cout << "Frame " << job.frame_index << ": " << job.quads.size() << " quads found" << endl;
        cout << "  Gray image: " << job.width << "x" << job.height << ", size: " << job.gray.size() << endl;
        if (!job.quads.empty()) {
          cout << "  First quad corners: (" << job.quads[0].corners[0][0] << "," << job.quads[0].corners[0][1] << "), "
               << "(" << job.quads[0].corners[1][0] << "," << job.quads[0].corners[1][1] << "), "
               << "(" << job.quads[0].corners[2][0] << "," << job.quads[0].corners[2][1] << "), "
               << "(" << job.quads[0].corners[3][0] << "," << job.quads[0].corners[3][1] << ")" << endl;
          // Check if quad coordinates are in bounds
          bool in_bounds = true;
          for (int i = 0; i < 4; i++) {
            if (job.quads[0].corners[i][0] < 0 || job.quads[0].corners[i][0] >= job.width ||
                job.quads[0].corners[i][1] < 0 || job.quads[0].corners[i][1] >= job.height) {
              in_bounds = false;
              break;
            }
          }
          cout << "  Quad in bounds: " << (in_bounds ? "yes" : "no") << endl;
          // Check gray image sample at first quad corner
          if (in_bounds && job.gray.size() >= job.width * job.height) {
            int x = static_cast<int>(job.quads[0].corners[0][0]);
            int y = static_cast<int>(job.quads[0].corners[0][1]);
            if (x >= 0 && x < job.width && y >= 0 && y < job.height) {
              uint8_t pixel = job.gray[y * job.width + x];
              cout << "  Gray pixel at first corner: " << static_cast<int>(pixel) << endl;
            }
          }
        }
        cout << "  Decimate factor: " << td->quad_decimate << endl;
      }
      
      auto cpu_start = chrono::steady_clock::now();
      frc971::apriltag::DecodeTagsFromQuads(
          job.quads, job.gray.data(), job.width, job.height,
          td_cpu, cam, dist, detections, poly0, poly1);
      auto cpu_end = chrono::steady_clock::now();
      acc_cpu_decode_ms +=
          chrono::duration<double, milli>(cpu_end - cpu_start).count();

      const zarray_t *detections_const = detections;
      int det_before = zarray_size(detections_const);
      if (det_before > 0) {
        // Print tag IDs for each detection
        cout << "  -> " << det_before << " detections: Tag IDs: ";
        for (int i = 0; i < det_before; i++) {
          apriltag_detection_t *det;
          zarray_get(const_cast<zarray_t *>(detections_const), i, &det);
          cout << det->id;
          if (i < det_before - 1) cout << ", ";
        }
        cout << endl;
      } else if (debug_frame_count % 100 == 0) {
        cout << "  -> 0 detections after decode" << endl;
      }
      total_detections_before += det_before;
      det_hist_before[det_before]++;

      auto scale_start = chrono::steady_clock::now();
      const double gpu_decimate = td->quad_decimate;
      if (gpu_decimate > 1.0) {
        for (int i = 0; i < zarray_size(detections_const); i++) {
          apriltag_detection_t *det;
          zarray_get(const_cast<zarray_t *>(detections_const), i, &det);
          scale_detection_coordinates(det, gpu_decimate);
        }
      }
      
      // Note: Mirroring is done in CPU thread before decode (above), so coordinates are already correct
      // No need to mirror coordinates again here
      
      auto scale_end = chrono::steady_clock::now();
      acc_scale_ms +=
          chrono::duration<double, milli>(scale_end - scale_start).count();

      auto filt_start = chrono::steady_clock::now();
      std::vector<apriltag_detection_t *> filtered =
          filter_duplicates(detections_const, job.width, job.height,
                            min_distance);
      int det_after = static_cast<int>(filtered.size());
      total_detections_after += det_after;
      det_hist_after[det_after]++;
      auto filt_end = chrono::steady_clock::now();
      acc_filter_ms +=
          chrono::duration<double, milli>(filt_end - filt_start).count();

      auto draw_start = chrono::steady_clock::now();
      // For visualization, use the original input grayscale frame
      // If mirroring is enabled, flip the display frame to match the mirrored coordinates
      Mat gray(job.disp_height, job.disp_width, CV_8UC1,
               job.display_gray.data());
      Mat gray_for_display;
      if (horizontal_mirror) {
        cv::flip(gray, gray_for_display, 1);  // 1 = horizontal flip
      } else {
        gray_for_display = gray;
      }
      Mat color_frame;
      cvtColor(gray_for_display, color_frame, COLOR_GRAY2BGR);

      vector<TagPoseInfo> tag_poses;
      for (auto *det : filtered) {
        TagPoseInfo pose_info;
        if (draw_3d_axes(color_frame, det, cam, dist, tag_size, pose_info)) {
          tag_poses.push_back(pose_info);
        }
      }

      // Write tag detections to shared memory for GUI
      if (tag_shm != nullptr) {
        tag_shm->timestamp = chrono::duration<double>(
            chrono::steady_clock::now().time_since_epoch()).count();
        tag_shm->num_tags = 0;
        for (const auto& pose : tag_poses) {
          if (tag_shm->num_tags < TagDetectionData::MAX_TAGS) {
            tag_shm->tags[tag_shm->num_tags].id = pose.id;
            tag_shm->tags[tag_shm->num_tags].x = pose.x;
            tag_shm->tags[tag_shm->num_tags].y = pose.y;
            tag_shm->tags[tag_shm->num_tags].z = pose.z;
            tag_shm->tags[tag_shm->num_tags].decision_margin = pose.decision_margin;
            tag_shm->num_tags++;
          }
        }
      }

      draw_info_table(color_frame, tag_poses, job.fps_estimate);
      
      // Write annotated frame to shared memory for GUI (when using --frames-shm)
      if (use_frames_shm && annotated_frame_shm != nullptr && 
          color_frame.cols > 0 && color_frame.rows > 0 &&
          color_frame.cols <= static_cast<int>(AnnotatedFrameIPCData::MAX_WIDTH) &&
          color_frame.rows <= static_cast<int>(AnnotatedFrameIPCData::MAX_HEIGHT)) {
        annotated_frame_shm->width = color_frame.cols;
        annotated_frame_shm->height = color_frame.rows;
        annotated_frame_shm->channels = 3;
        annotated_frame_shm->frame_id = ++annotated_frame_id_counter;
        annotated_frame_shm->timestamp = chrono::duration<double>(
            chrono::steady_clock::now().time_since_epoch()).count();
        
        // Copy BGR frame data (ensure it's contiguous)
        if (color_frame.isContinuous()) {
          memcpy(annotated_frame_shm->data, color_frame.data, 
                 color_frame.rows * color_frame.cols * 3);
        } else {
          // Copy row by row if not contiguous
          for (int y = 0; y < color_frame.rows; y++) {
            memcpy(annotated_frame_shm->data + y * color_frame.cols * 3,
                   color_frame.ptr(y), color_frame.cols * 3);
          }
        }
      }
      
      auto draw_end = chrono::steady_clock::now();
      acc_draw_ms +=
          chrono::duration<double, milli>(draw_end - draw_start).count();

      {
        std::lock_guard<std::mutex> lk(draw_mtx);
        if (draw_queue.size() >= writer_queue_size) {
          if (writer_drop_oldest && !draw_queue.empty()) {
            draw_queue.pop_front();
          } else {
            continue;  // drop this rendered frame
          }
        }
        DrawItem item;
        item.color = color_frame.clone();
        draw_queue.push_back(std::move(item));
      }
      draw_cv.notify_one();
    }

    // Cleanup CPU decode resources.
    for (int i = 0; i < zarray_size(detections); ++i) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);
    zarray_destroy(poly0);
    zarray_destroy(poly1);
    apriltag_detector_destroy(td_cpu);
    teardown_tag_family(&tf_cpu, family.c_str());
  });

  auto process_frame = [&](const Mat &frame_ref) {
    // Validate frame
    if (frame_ref.empty() || frame_ref.data == nullptr) {
      cerr << "Error: Invalid frame passed to process_frame" << endl;
      return;
    }
    if (frame_ref.cols != width || frame_ref.rows != height) {
      cerr << "Error: Frame size mismatch: " << frame_ref.cols << "x" << frame_ref.rows 
           << " expected " << width << "x" << height << endl;
      return;
    }
    if (!frame_ref.isContinuous()) {
      cerr << "Error: Frame is not contiguous" << endl;
      return;
    }
    
    // Always use original frame for detection (mirroring will be applied to coordinates after detection)
    // Stage 1: GPU-only detection on this frame.
    auto f_start = chrono::steady_clock::now();
    detector.DetectGpuOnly(frame_ref.data);
    auto f_end = chrono::steady_clock::now();
    double frame_ms = chrono::duration<double, milli>(f_end - f_start).count();
    frame_times.push_back(frame_ms);
    acc_detect_ms += frame_ms;

    // Approximate CUDA time using the detector's CUDA accumulator.
    double cur_cuda = detector.GetCudaOperationsDurationMs();
    acc_cuda_ms += (cur_cuda - prev_cuda_total);
    prev_cuda_total = cur_cuda;
    
    // Calculate current FPS (rolling average)
    double current_fps = 0.0;
    if (frame_times.size() >= 30) {
      double avg_ms = 0.0;
      for (size_t i = frame_times.size() - 30; i < frame_times.size(); i++) {
        avg_ms += frame_times[i];
      }
      avg_ms /= 30.0;
      current_fps = 1000.0 / avg_ms;
    } else if (frame_times.size() > 1) {
      double avg_ms = 0.0;
      for (double t : frame_times) avg_ms += t;
      avg_ms /= frame_times.size();
      current_fps = 1000.0 / avg_ms;
    }

    // Build a decode job with copies of:
    //  - GPU-produced grayscale image (gray_image_host_) for CPU decode
    //  - Original input grayscale frame for visualization
    DecodeJob job;
    job.frame_index = frame_num;
    job.width = detector.Width();
    job.height = detector.Height();
    job.disp_width = frame_ref.cols;
    job.disp_height = frame_ref.rows;
    job.fps_estimate = current_fps;
    // FitQuads returns quads in full resolution (after AdjustPixelCenters)
    // CopyGrayHostTo returns full resolution gray image
    // So we use both in full resolution, then scale detection results afterward
    job.quads = detector.FitQuads();
    
    // Mirror gray image on GPU if requested (faster than CPU)
    auto mirror_start = chrono::steady_clock::now();
    if (horizontal_mirror) {
      detector.MirrorGrayImageOnGpu();
      // Also mirror quad coordinates (on CPU, they're small)
      int gray_width = job.width;  // Full resolution width
      for (auto& quad : job.quads) {
        // Mirror x coordinates for all 4 corners
        for (int i = 0; i < 4; i++) {
          quad.corners[i][0] = gray_width - 1 - quad.corners[i][0];
        }
        // Swap corners to maintain correct orientation: 0<->1, 2<->3
        float temp[2];
        temp[0] = quad.corners[0][0]; temp[1] = quad.corners[0][1];
        quad.corners[0][0] = quad.corners[1][0]; quad.corners[0][1] = quad.corners[1][1];
        quad.corners[1][0] = temp[0]; quad.corners[1][1] = temp[1];
        
        temp[0] = quad.corners[2][0]; temp[1] = quad.corners[2][1];
        quad.corners[2][0] = quad.corners[3][0]; quad.corners[2][1] = quad.corners[3][1];
        quad.corners[3][0] = temp[0]; quad.corners[3][1] = temp[1];
      }
    }
    auto mirror_end = chrono::steady_clock::now();
    acc_mirror_ms += chrono::duration<double, milli>(mirror_end - mirror_start).count();
    
    // Copy full-resolution gray image from GPU
    detector.CopyGrayHostTo(job.gray);
    
    // Copy the original input frame for display (independent of GPU layout).
    job.display_gray.resize(static_cast<size_t>(frame_ref.cols * frame_ref.rows));
    std::memcpy(job.display_gray.data(), frame_ref.data,
                static_cast<size_t>(frame_ref.cols * frame_ref.rows));

    {
      std::lock_guard<std::mutex> lk(dq_mtx);
      if (decode_queue.size() >= decode_queue_size) {
        if (decode_drop_oldest && !decode_queue.empty()) {
          decode_queue.pop_front();
        } else {
          // Drop this frame's decode job to keep pipeline real-time.
          goto after_gpu_stage;
        }
      }
      decode_queue.push_back(std::move(job));
    }
    dq_cv.notify_one();

after_gpu_stage:
    frame_num++;
    if (frame_num % 100 == 0) {
      cout << "Processed " << frame_num << " frames... "
           << "\r" << flush;
    }
  };

  // Frame acquisition: optional prefetching via separate thread
  if (prefetch_enabled) {
    // Seed first frame into queue
    {
      std::lock_guard<std::mutex> lk(fq_mtx);
      frame_queue.push_back(FrameItem{frame.clone(), 0});
    }
    fq_cv.notify_one();

    std::thread reader_thread([&]() {
      size_t idx = 1;
      VideoCapture* cap_ptr = !video_path.empty() || (camera_index >= 0 && !use_mvsdk) ? &cap : nullptr;
      while (true) {
        auto read_start = chrono::steady_clock::now();
        Mat f;
        bool ok = false;
        
        if (use_frames_shm && frame_shm != nullptr) {
          // Read from shared memory (GUI writes frames here)
          // Wait for a new frame (check frame_id changed)
          uint32_t current_frame_id = frame_shm->frame_id;
          if (current_frame_id != last_frame_id && frame_shm->width > 0 && frame_shm->height > 0) {
            // Create Mat from shared memory data
            f = Mat(frame_shm->height, frame_shm->width, CV_8UC1, 
                   (void*)frame_shm->data, frame_shm->width);
            f = f.clone(); // Clone to own the data
            last_frame_id = current_frame_id;
            ok = !f.empty();
          } else {
            // No new frame yet, wait a bit
            this_thread::sleep_for(chrono::milliseconds(5));
            continue;
          }
        } else if (use_mvsdk && mv_camera_handle != 0) {
          // Read from MindVision camera
          BYTE *pbyBuffer = nullptr;
          tSdkFrameHead sFrameInfo;
          CameraSdkStatus status = CameraGetImageBuffer(mv_camera_handle, &sFrameInfo, &pbyBuffer, 100);
          if (status == CAMERA_STATUS_SUCCESS) {
            // Create Mat from frame data (MONO8 format)
            f = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1, pbyBuffer, sFrameInfo.iWidth);
            f = f.clone(); // Clone to own the data
            CameraReleaseImageBuffer(mv_camera_handle, pbyBuffer);
            ok = !f.empty();
          } else if (status == CAMERA_STATUS_TIME_OUT) {
            continue; // Timeout is normal, try again
          } else {
            break; // Other error, exit
          }
        } else if (cap_ptr) {
          // Read from OpenCV VideoCapture
          ok = cap_ptr->read(f);
        } else {
          break;
        }
        
        auto read_end = chrono::steady_clock::now();
        double read_ms = chrono::duration<double, milli>(read_end - read_start).count();
        // Atomic add for double (nvcc + libstdc++ may lack fetch_add specialization)
        double cur = acc_read_ms_atomic.load(std::memory_order_relaxed);
        while (!acc_read_ms_atomic.compare_exchange_weak(
            cur, cur + read_ms, std::memory_order_relaxed)) {
          // cur is updated with latest value by compare_exchange_weak
        }

        if (!ok || f.empty()) break;
        
        // Ensure grayscale
        if (f.type() != CV_8UC1) {
          if (f.channels() == 2) {
            cvtColor(f, f, COLOR_YUV2GRAY_YUY2);
          } else if (f.channels() == 3) {
            cvtColor(f, f, COLOR_BGR2GRAY);
          }
        }
        
        if (f.cols != width || f.rows != height) {
          cerr << "Unexpected frame size in reader: " << f.cols << "x" << f.rows << endl;
          break;
        }

        {
          std::lock_guard<std::mutex> lk(fq_mtx);
          if (frame_queue.size() >= prefetch_queue_size) {
            if (prefetch_drop_oldest && !frame_queue.empty()) {
              frame_queue.pop_front();
            } else {
              continue;  // drop this frame
            }
          }
          frame_queue.push_back(FrameItem{std::move(f), idx++});
        }
        fq_cv.notify_one();
      }
      {
        std::lock_guard<std::mutex> lk(fq_mtx);
        reader_done = true;
      }
      fq_cv.notify_all();
    });

    // Consume frames from queue
    while (true) {
      FrameItem item;
      {
        std::unique_lock<std::mutex> lk(fq_mtx);
        fq_cv.wait(lk, [&]() { return reader_done || !frame_queue.empty(); });
        if (frame_queue.empty()) {
          if (reader_done) break;
          else continue;  // wait again
        }
        item = std::move(frame_queue.front());
        frame_queue.pop_front();
      }
      process_frame(item.gray);
    }

    reader_thread.join();
    acc_read_ms = acc_read_ms_atomic.load(std::memory_order_relaxed);
  } else {
    // No prefetching: synchronous read
    process_frame(frame);
    VideoCapture* cap_ptr = !video_path.empty() || (camera_index >= 0 && !use_mvsdk) ? &cap : nullptr;
    
    while (true) {
      auto read_start = chrono::steady_clock::now();
      bool ok = false;
      
      if (use_frames_shm && frame_shm != nullptr) {
        // Read from shared memory (GUI writes frames here)
        // Wait for a new frame (check frame_id changed)
        uint32_t current_frame_id = frame_shm->frame_id;
        if (current_frame_id != last_frame_id && frame_shm->width > 0 && frame_shm->height > 0) {
          // Create Mat from shared memory data
          frame = Mat(frame_shm->height, frame_shm->width, CV_8UC1, 
                     (void*)frame_shm->data, frame_shm->width);
          frame = frame.clone(); // Clone to own the data
          last_frame_id = current_frame_id;
          ok = !frame.empty();
        } else {
          // No new frame yet, wait a bit
          this_thread::sleep_for(chrono::milliseconds(5));
          continue;
        }
      } else if (use_mvsdk && mv_camera_handle != 0) {
        // Read from MindVision camera
        BYTE *pbyBuffer = nullptr;
        tSdkFrameHead sFrameInfo;
        CameraSdkStatus status = CameraGetImageBuffer(mv_camera_handle, &sFrameInfo, &pbyBuffer, 100);
        if (status == CAMERA_STATUS_SUCCESS) {
          // Create Mat from frame data (MONO8 format)
          frame = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1, pbyBuffer, sFrameInfo.iWidth);
          frame = frame.clone(); // Clone to own the data
          CameraReleaseImageBuffer(mv_camera_handle, pbyBuffer);
          ok = !frame.empty();
        } else if (status == CAMERA_STATUS_TIME_OUT) {
          continue; // Timeout is normal, try again
        } else {
          break; // Other error, exit
        }
      } else if (cap_ptr) {
        // Read from OpenCV VideoCapture
        ok = cap_ptr->read(frame);
        if (ok) {
          // Ensure grayscale
          if (frame.type() != CV_8UC1) {
            if (frame.channels() == 2) {
              cvtColor(frame, frame, COLOR_YUV2GRAY_YUY2);
            } else if (frame.channels() == 3) {
              cvtColor(frame, frame, COLOR_BGR2GRAY);
            }
          }
        }
      } else {
        break;
      }
      
      if (!ok) break;
      
      auto read_end = chrono::steady_clock::now();
      acc_read_ms += chrono::duration<double, milli>(read_end - read_start).count();
      
      if (frame.cols != width || frame.rows != height) {
        cerr << "Unexpected frame size: " << frame.cols << "x" << frame.rows << endl;
        break;
      }
      process_frame(frame);
    }
  }
  
  // Cleanup MindVision camera
  if (mv_camera_handle != 0) {
    CameraStop(mv_camera_handle);
    CameraUnInit(mv_camera_handle);
    mv_camera_handle = 0;
  }
  if (mv_rgb_buffer) {
    free(mv_rgb_buffer);
    mv_rgb_buffer = nullptr;
  }
  
  // Cleanup shared memory for frames
  if (frame_shm != nullptr && frame_shm != MAP_FAILED) {
    munmap(frame_shm, sizeof(FrameIPCData));
    frame_shm = nullptr;
  }
  if (frame_shm_fd >= 0) {
    ::close(frame_shm_fd);
    frame_shm_fd = -1;
  }
  
  // Cleanup shared memory for tag detections
  if (tag_shm != nullptr && tag_shm != MAP_FAILED) {
    munmap(tag_shm, sizeof(TagDetectionData));
    tag_shm = nullptr;
  }
  if (tag_shm_fd >= 0) {
    ::close(tag_shm_fd);
    tag_shm_fd = -1;
    shm_unlink(TAG_IPC_KEY);
  }
  
  // Cleanup shared memory for annotated frames
  if (annotated_frame_shm != nullptr && annotated_frame_shm != MAP_FAILED) {
    munmap(annotated_frame_shm, sizeof(AnnotatedFrameIPCData));
    annotated_frame_shm = nullptr;
  }
  if (annotated_frame_shm_fd >= 0) {
    ::close(annotated_frame_shm_fd);
    annotated_frame_shm_fd = -1;
    shm_unlink(ANNOTATED_FRAME_IPC_KEY);
  }

  // Finish decode and display/write threads and clean up
  {
    std::lock_guard<std::mutex> lk(dq_mtx);
    decode_done = true;
  }
  dq_cv.notify_all();
  decode_thread.join();

  {
    std::lock_guard<std::mutex> lk(draw_mtx);
    display_done = true;
  }
  draw_cv.notify_all();
  display_thread.join();
  if (write_enabled) {
    writer.release();
  }
  
  // Cleanup VideoCapture if used
  if (cap.isOpened()) {
    cap.release();
  }

  auto t_end = chrono::steady_clock::now();
  double total_s = chrono::duration<double>(t_end - t_start).count();
  double avg_fps = frame_num / total_s;

  cout << "\nCompleted processing " << frame_num << " frames in "
       << fixed << setprecision(2) << total_s << " seconds\n";
  cout << "Average processing FPS: " << avg_fps << "\n";
  if (show_before_filter) {
    cout << "Total detections before filtering: " << total_detections_before
         << "\n";
  }
  cout << "Total detections after filtering: " << total_detections_after
       << "\n";
  if (show_before_filter) {
    cout << "Average per frame: " << (total_detections_before / frame_num)
         << " -> " << (total_detections_after / frame_num) << "\n";
  } else {
    cout << "Average per frame: " << (total_detections_after / frame_num)
         << "\n";
  }
  if (write_enabled) {
    cout << "Output saved to: " << output_path << endl;
  } else {
    cout << "Output not saved (display only; pass --output to write).\n";
  }

  // Per-stage timing (ms per frame)
  if (frame_num > 0) {
    double frames = static_cast<double>(frame_num);
    double writer_den = writer_frames > 0 ? static_cast<double>(writer_frames) : 1.0;
    cout << "Timing (ms/frame, averages):\n";
    cout << "  Frame read:        " << (acc_read_ms / frames) << "\n";
    cout << "  Detect total:      " << (acc_detect_ms / frames) << "\n";
    cout << "    CUDA ops:        " << (acc_cuda_ms / frames) << "\n";
    if (horizontal_mirror) {
      cout << "    Mirror (GPU):     " << (acc_mirror_ms / frames) << "\n";
    }
    cout << "    CPU decode:      " << (acc_cpu_decode_ms / frames) << "\n";
    cout << "  Scale coordinates: " << (acc_scale_ms / frames) << "\n";
    cout << "  Filter duplicates: " << (acc_filter_ms / frames) << "\n";
    cout << "  Draw (axes/text):  " << (acc_draw_ms / frames) << "\n";
    cout << "  Write frame:       " << (writer_write_ms / writer_den) << "\n";
  }

  auto print_hist = [&](const std::map<int, int64_t>& hist,
                        const std::string& title) {
    cout << title << "\n";
    for (const auto& kv : hist) {
      cout << "  " << kv.first << " tags: " << kv.second << " frames\n";
    }
  };
  if (show_before_filter) {
    print_hist(det_hist_before,
               "Frame detection histogram (before filtering):");
  }
  print_hist(det_hist_after, "Frame detection histogram (after  filtering):");

  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}

