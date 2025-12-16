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

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
}

using namespace std;
using namespace cv;

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

// Draw 3D axes on detected tag
void draw_3d_axes(Mat &im, apriltag_detection_t *det, 
                  const frc971::apriltag::CameraMatrix &cam,
                  const frc971::apriltag::DistCoeffs &dist,
                  double tag_size = 0.1) {
  // Convert camera matrix and distortion to OpenCV format
  Mat camera_matrix = (Mat_<double>(3, 3) <<
    cam.fx, 0, cam.cx,
    0, cam.fy, cam.cy,
    0, 0, 1);
  
  Mat dist_coeffs = (Mat_<double>(5, 1) <<
    dist.k1, dist.k2, dist.p1, dist.p2, dist.k3);

  // Estimate pose from homography
  apriltag_detection_info_t info;
  info.det = det;
  info.tagsize = tag_size;
  info.fx = cam.fx;
  info.fy = cam.fy;
  info.cx = cam.cx;
  info.cy = cam.cy;

  apriltag_pose_t pose;
  double err = estimate_tag_pose(&info, &pose);
  
  if (err > 0.1) {
    // If pose estimation failed, just draw outline
    line(im, Point(det->p[0][0], det->p[0][1]),
         Point(det->p[1][0], det->p[1][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[1][0], det->p[1][1]),
         Point(det->p[2][0], det->p[2][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[2][0], det->p[2][1]),
         Point(det->p[3][0], det->p[3][1]), Scalar(0, 255, 255), 2);
    line(im, Point(det->p[3][0], det->p[3][1]),
         Point(det->p[0][0], det->p[0][1]), Scalar(0, 255, 255), 2);
    return;
  }

  // Convert apriltag pose to OpenCV format
  Mat rvec, tvec;
  
  // Convert rotation matrix to rotation vector
  Mat R = (Mat_<double>(3, 3) <<
    pose.R->data[0], pose.R->data[1], pose.R->data[2],
    pose.R->data[3], pose.R->data[4], pose.R->data[5],
    pose.R->data[6], pose.R->data[7], pose.R->data[8]);
  
  Rodrigues(R, rvec);
  
  tvec = (Mat_<double>(3, 1) <<
    pose.t->data[0],
    pose.t->data[1],
    pose.t->data[2]);

  // Use OpenCV's drawFrameAxes to draw 3D axes
  drawFrameAxes(im, camera_matrix, dist_coeffs, rvec, tvec, tag_size * 0.5, 3);

  // Calculate distance from camera to tag (magnitude of translation vector)
  double distance = sqrt(tvec.at<double>(0) * tvec.at<double>(0) +
                         tvec.at<double>(1) * tvec.at<double>(1) +
                         tvec.at<double>(2) * tvec.at<double>(2));

  // Draw tag outline
  line(im, Point(det->p[0][0], det->p[0][1]),
       Point(det->p[1][0], det->p[1][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[1][0], det->p[1][1]),
       Point(det->p[2][0], det->p[2][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[2][0], det->p[2][1]),
       Point(det->p[3][0], det->p[3][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[3][0], det->p[3][1]),
       Point(det->p[0][0], det->p[0][1]), Scalar(0, 255, 255), 2);

  // Draw tag ID and distance near the tag center
  stringstream tag_info;
  tag_info << "ID:" << det->id << " Dist:" << fixed << setprecision(3) << distance << "m";
  
  // Position text above the tag center
  Point text_pos(det->c[0] - 80, det->c[1] - 20);
  putText(im, tag_info.str(), text_pos,
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

  // Draw tag ID and decision margin
  stringstream ss;
  ss << "ID:" << det->id << " M:" << fixed << setprecision(1) << det->decision_margin;
  putText(im, ss.str(), Point(det->c[0] - 40, det->c[1] - 10),
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);
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

  if (video_path.empty() || output_path.empty()) {
    cerr << "Error: --video and --output are required\n";
    return 1;
  }

  VideoCapture cap(video_path, CAP_ANY);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return 1;
  }
  cap.set(CAP_PROP_CONVERT_RGB, false);

  Mat frame;
  if (!cap.read(frame)) {
    cerr << "Could not read first frame\n";
    return 1;
  }
  CV_Assert(frame.type() == CV_8UC1);
  int width = frame.cols;
  int height = frame.rows;
  
  // Get FPS from video metadata
  double fps = cap.get(CAP_PROP_FPS);
  if (fps <= 0) {
    fps = 30.0; // Default if FPS not available
    cout << "FPS not available, using default: 30.0 FPS\n";
  } else {
    cout << "Using video FPS from metadata: " << fixed << setprecision(2) << fps << "\n";
  }

  // Setup detector
  apriltag_family_t *tf = nullptr;
  setup_tag_family(&tf, family.c_str());
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  td->debug = false;
  td->refine_edges = true;
  td->wp = workerpool_create(1);

  frc971::apriltag::CameraMatrix cam{905.495617, 609.916016, 907.909470, 352.682645};
  frc971::apriltag::DistCoeffs dist{0.059238, -0.075154, -0.003801, 0.001113, 0.0};

  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);

  // Create output video writer
  Mat color_frame;
  cvtColor(frame, color_frame, COLOR_GRAY2BGR);
  VideoWriter writer(output_path, VideoWriter::fourcc('X', 'V', 'I', 'D'), 
                     fps, Size(width, height), true);
  if (!writer.isOpened()) {
    cerr << "Failed to open output video: " << output_path << endl;
    return 1;
  }

  size_t frame_num = 0;
  auto t_start = chrono::steady_clock::now();
  vector<double> frame_times;
  int total_detections_before = 0;
  int total_detections_after = 0;

  cout << "Processing video: " << video_path << endl;
  cout << "Output: " << output_path << endl;
  cout << "Resolution: " << width << "x" << height << " @ " << fps << " FPS\n";
  cout << "Min distance for duplicate filtering: " << min_distance << " pixels\n";

  do {
    CV_Assert(frame.type() == CV_8UC1);
    
    // Convert to color for visualization
    cvtColor(frame, color_frame, COLOR_GRAY2BGR);
    
    auto f_start = chrono::steady_clock::now();
    detector.Detect(frame.data);
    auto f_end = chrono::steady_clock::now();
    double frame_ms = chrono::duration<double, milli>(f_end - f_start).count();
    frame_times.push_back(frame_ms);
    
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
    total_detections_before += zarray_size(detections);
    
    // IMPORTANT: Scale GPU detector coordinates from decimated to full resolution
    // The GPU detector returns coordinates in decimated space (quad_decimate = 2.0)
    const double gpu_decimate = td->quad_decimate;
    if (gpu_decimate > 1.0) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);
        scale_detection_coordinates(det, gpu_decimate);
      }
    }
    
    // Filter duplicates (also filters out invalid coordinates)
    vector<apriltag_detection_t*> filtered = filter_duplicates(detections, width, height, min_distance);
    total_detections_after += filtered.size();
    
    // Draw 3D visualization for each filtered detection
    for (auto *det : filtered) {
      draw_3d_axes(color_frame, det, cam, dist, tag_size);
    }

    // Draw frame number and FPS counter (top left)
    stringstream info_text;
    info_text << "Frame: " << frame_num << " | FPS: " << fixed << setprecision(1) << current_fps;
    putText(color_frame, info_text.str(), Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);
    
    // Draw detection count (top right) - show both before and after filtering
    stringstream det_text;
    det_text << "Tags: " << filtered.size() << " (from " << zarray_size(detections) << ")";
    Size det_size = getTextSize(det_text.str(), FONT_HERSHEY_SIMPLEX, 1.0, 2, nullptr);
    putText(color_frame, det_text.str(), Point(width - det_size.width - 10, 30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);

    // Write frame
    writer.write(color_frame);

    frame_num++;
    if (frame_num % 100 == 0) {
      cout << "Processed " << frame_num << " frames... "
           << "Avg detections: " << (total_detections_before / frame_num) 
           << " -> " << (total_detections_after / frame_num) << " (filtered)\r" << flush;
    }
  } while (cap.read(frame));

  writer.release();
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
  cout << "Output saved to: " << output_path << endl;

  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}

