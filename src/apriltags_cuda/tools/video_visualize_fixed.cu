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

using namespace std;
using namespace cv;

struct DrawItem {
  Mat color;  // pre-drawn BGR frame ready to display/write
};

struct FrameItem {
  Mat gray;
  size_t idx;
};

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
    } else if (arg == "--help" || arg == "-h") {
      cout << "Usage: video_visualize_fixed --video <input.avi> --output <output.avi> "
              "[--family <tag36h11>] [--tag_size <0.1>] [--min_distance <50.0>]\n";
      return 0;
    }
  }

  if (video_path.empty()) {
    cerr << "Error: --video is required\n";
    return 1;
  }

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
  const bool prefetch_enabled = config.get_bool("prefetching.enabled", false);
  const int cfg_prefetch_q = std::max(1, config.get_int("prefetching.queue_size", 2));
  const bool prefetch_drop_oldest = config.get_bool("prefetching.drop_oldest", true);
  const int cfg_writer_q = std::max(1, config.get_int("writer.queue_size", 5));
  const bool writer_drop_oldest = config.get_bool("writer.drop_oldest", true);

  // Use OpenCV VideoCapture and ask backend for grayscale frames
  VideoCapture cap(video_path, CAP_ANY);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return 1;
  }

  // CRITICAL: Don't convert to RGB - read grayscale directly for better performance
  cap.set(CAP_PROP_CONVERT_RGB, false);

  // Get video properties
  int width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
  int height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
  double fps = cap.get(CAP_PROP_FPS);
  if (fps <= 0) {
    fps = 30.0; // Default if FPS not available
    cout << "FPS not available, using default: 30.0 FPS\n";
  } else {
    cout << "Using video FPS from metadata: " << fixed << setprecision(2) << fps << "\n";
  }

  // Read first frame to verify (backend should give us grayscale)
  Mat frame;
  if (!cap.read(frame)) {
    cerr << "Could not read first frame\n";
    return 1;
  }
  
  if (frame.empty()) {
    cerr << "First frame is empty\n";
    return 1;
  }
  
  CV_Assert(frame.type() == CV_8UC1);  // Grayscale
  cout << "First frame size: " << frame.cols << "x" << frame.rows 
       << ", type: " << frame.type() << " (grayscale)" << endl;

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

  cout << "Processing video: " << video_path << endl;
  cout << "Output: " << output_path << endl;
  cout << "Resolution: " << width << "x" << height << " @ " << fps << " FPS\n";
  cout << "Min distance for duplicate filtering: " << min_distance << " pixels\n";

  // Thread-safe queue for draw/display/write
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

      // Display on screen
      imshow("AprilTags", item.color);
      waitKey(1);

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

  auto process_frame = [&](const Mat &frame_ref) {
    // Fastest path: pass grayscale directly to detector.
    auto f_start = chrono::steady_clock::now();
    detector.Detect(frame_ref.data);
    auto f_end = chrono::steady_clock::now();
    double frame_ms = chrono::duration<double, milli>(f_end - f_start).count();
    frame_times.push_back(frame_ms);
    acc_detect_ms += frame_ms;
    // CUDA vs CPU decode split
    double cur_cuda = detector.GetCudaOperationsDurationMs();
    double cur_cpu  = detector.GetCpuDecodeDurationMs();
    acc_cuda_ms += (cur_cuda - prev_cuda_total);
    acc_cpu_decode_ms += (cur_cpu - prev_cpu_total);
    prev_cuda_total = cur_cuda;
    prev_cpu_total = cur_cpu;
    
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

    // Get detections
    const zarray_t *detections = detector.Detections();
    int det_before = zarray_size(detections);
    total_detections_before += det_before;
    det_hist_before[det_before]++;
    
    auto scale_start = chrono::steady_clock::now();
    // IMPORTANT: Scale GPU detector coordinates from decimated to full resolution
    const double gpu_decimate = td->quad_decimate;
    if (gpu_decimate > 1.0) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);
        scale_detection_coordinates(det, gpu_decimate);
      }
    }
    auto scale_end = chrono::steady_clock::now();
    acc_scale_ms += chrono::duration<double, milli>(scale_end - scale_start).count();
    
    auto filt_start = chrono::steady_clock::now();
    // Filter duplicates (also filters out invalid coordinates)
    vector<apriltag_detection_t*> filtered = filter_duplicates(detections, width, height, min_distance);
    int det_after = static_cast<int>(filtered.size());
    total_detections_after += det_after;
    det_hist_after[det_after]++;
    auto filt_end = chrono::steady_clock::now();
    acc_filter_ms += chrono::duration<double, milli>(filt_end - filt_start).count();
    
    auto draw_start = chrono::steady_clock::now();
    // Draw in main thread (for display) and optionally enqueue for writing
    Mat color_frame;
    cvtColor(frame_ref, color_frame, COLOR_GRAY2BGR);
    
    // Collect tag pose information and draw axes
    vector<TagPoseInfo> tag_poses;
    for (auto *det : filtered) {
      TagPoseInfo pose_info;
      if (draw_3d_axes(color_frame, det, cam, dist, tag_size, pose_info)) {
        // Only add tags with valid pose estimation
        tag_poses.push_back(pose_info);
      }
    }
    
    // Draw information table in top-right corner
    draw_info_table(color_frame, tag_poses, current_fps);
    auto draw_end = chrono::steady_clock::now();
    acc_draw_ms += chrono::duration<double, milli>(draw_end - draw_start).count();

    // Enqueue frame for display (and optional writing) without blocking detector
    {
      std::lock_guard<std::mutex> lk(draw_mtx);
      if (draw_queue.size() >= writer_queue_size) {
        if (writer_drop_oldest && !draw_queue.empty()) {
          draw_queue.pop_front();  // drop oldest frame
        } else {
          goto after_enqueue;  // drop this frame for display/write
        }
      }
      DrawItem item;
      item.color = color_frame.clone();
      draw_queue.push_back(std::move(item));
    }
    draw_cv.notify_one();

after_enqueue:
    frame_num++;
    if (frame_num % 100 == 0) {
      cout << "Processed " << frame_num << " frames... "
           << "Avg detections: " << (total_detections_before / frame_num) 
           << " -> " << (total_detections_after / frame_num) << " (filtered)\r" << flush;
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
      while (true) {
        auto read_start = chrono::steady_clock::now();
        Mat f;
        bool ok = cap.read(f);
        auto read_end = chrono::steady_clock::now();
        double read_ms = chrono::duration<double, milli>(read_end - read_start).count();
        // Atomic add for double (nvcc + libstdc++ may lack fetch_add specialization)
        double cur = acc_read_ms_atomic.load(std::memory_order_relaxed);
        while (!acc_read_ms_atomic.compare_exchange_weak(
            cur, cur + read_ms, std::memory_order_relaxed)) {
          // cur is updated with latest value by compare_exchange_weak
        }

        if (!ok || f.empty()) break;
        CV_Assert(f.type() == CV_8UC1);
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
    while (true) {
      auto read_start = chrono::steady_clock::now();
      if (!cap.read(frame)) break;
      auto read_end = chrono::steady_clock::now();
      acc_read_ms += chrono::duration<double, milli>(read_end - read_start).count();
      CV_Assert(frame.type() == CV_8UC1);  // Grayscale
      CV_Assert(frame.cols == width && frame.rows == height);
      process_frame(frame);
    }
  }

  // Finish display/write thread and clean up
  {
    std::lock_guard<std::mutex> lk(draw_mtx);
    display_done = true;
  }
  draw_cv.notify_all();
  display_thread.join();
  if (write_enabled) {
    writer.release();
  }
  cap.release();

  auto t_end = chrono::steady_clock::now();
  double total_s = chrono::duration<double>(t_end - t_start).count();
  double avg_fps = frame_num / total_s;

  cout << "\nCompleted processing " << frame_num << " frames in " 
       << fixed << setprecision(2) << total_s << " seconds\n";
  cout << "Average processing FPS: " << avg_fps << "\n";
  cout << "Total detections before filtering: " << total_detections_before << "\n";
  cout << "Total detections after filtering: " << total_detections_after << "\n";
  cout << "Average per frame: " << (total_detections_before / frame_num) 
       << " -> " << (total_detections_after / frame_num) << "\n";
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
  print_hist(det_hist_before, "Frame detection histogram (before filtering):");
  print_hist(det_hist_after,  "Frame detection histogram (after  filtering):");

  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}

