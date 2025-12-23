#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
}

using namespace std;
using namespace cv;

// Draw 3D axes on detected tag
void draw_3d_axes(Mat &im, apriltag_detection_t *det, 
                  double fx, double fy, double cx, double cy,
                  double k1, double k2, double p1, double p2, double k3,
                  double tag_size = 0.1) {
  // Convert camera matrix and distortion to OpenCV format
  Mat camera_matrix = (Mat_<double>(3, 3) <<
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);
  
  Mat dist_coeffs = (Mat_<double>(5, 1) <<
    k1, k2, p1, p2, k3);

  // Estimate pose from homography
  apriltag_detection_info_t info;
  info.det = det;
  info.tagsize = tag_size;
  info.fx = fx;
  info.fy = fy;
  info.cx = cx;
  info.cy = cy;

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

  // Draw tag outline
  line(im, Point(det->p[0][0], det->p[0][1]),
       Point(det->p[1][0], det->p[1][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[1][0], det->p[1][1]),
       Point(det->p[2][0], det->p[2][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[2][0], det->p[2][1]),
       Point(det->p[3][0], det->p[3][1]), Scalar(0, 255, 255), 2);
  line(im, Point(det->p[3][0], det->p[3][1]),
       Point(det->p[0][0], det->p[0][1]), Scalar(0, 255, 255), 2);

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
  double tag_size = 0.1;

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
    } else if (arg == "--help" || arg == "-h") {
      cout << "Usage: video_visualize_cpu --video <input.avi> --output <output.avi> "
              "[--family <tag36h11>] [--tag_size <0.1>]\n";
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
  double fps = cap.get(CAP_PROP_FPS);
  if (fps <= 0) fps = 30.0;

  // Setup CPU detector
  apriltag_family_t *tf = nullptr;
  setup_tag_family(&tf, family.c_str());
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->nthreads = 4;  // Use multiple threads for CPU
  td->debug = false;
  td->refine_edges = true;
  td->wp = workerpool_create(4);

  // Camera calibration parameters
  double fx = 905.495617;
  double fy = 907.909470;
  double cx = 609.916016;
  double cy = 352.682645;
  double k1 = 0.059238;
  double k2 = -0.075154;
  double p1 = -0.003801;
  double p2 = 0.001113;
  double k3 = 0.0;

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
  int total_detections = 0;
  map<int, int> tag_id_counts;

  cout << "Processing video with CPU detector: " << video_path << endl;
  cout << "Output: " << output_path << endl;
  cout << "Resolution: " << width << "x" << height << " @ " << fps << " FPS\n";

  do {
    CV_Assert(frame.type() == CV_8UC1);
    
    // Convert to color for visualization
    cvtColor(frame, color_frame, COLOR_GRAY2BGR);
    
    // Create image_u8_t for CPU detector
    image_u8_t im = {
      .width = width,
      .height = height,
      .stride = width,
      .buf = frame.data
    };
    
    auto f_start = chrono::steady_clock::now();
    zarray_t *detections = apriltag_detector_detect(td, &im);
    auto f_end = chrono::steady_clock::now();
    double frame_ms = chrono::duration<double, milli>(f_end - f_start).count();
    frame_times.push_back(frame_ms);
    
    total_detections += zarray_size(detections);
    
    // Count tag IDs
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      tag_id_counts[det->id]++;
    }
    
    // Filter duplicates - keep best detection per ID
    map<int, apriltag_detection_t*> best_by_id;
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(detections, i, &det);
      auto it = best_by_id.find(det->id);
      if (it == best_by_id.end() || det->decision_margin > it->second->decision_margin) {
        best_by_id[det->id] = det;
      }
    }
    
    // Draw 3D visualization for each filtered detection
    for (auto &pair : best_by_id) {
      draw_3d_axes(color_frame, pair.second, fx, fy, cx, cy, k1, k2, p1, p2, k3, tag_size);
    }
    
    // Calculate current FPS
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

    // Draw frame number and FPS counter (top left)
    stringstream info_text;
    info_text << "Frame: " << frame_num << " | FPS: " << fixed << setprecision(1) << current_fps << " (CPU)";
    putText(color_frame, info_text.str(), Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);
    
    // Draw detection count (top right)
    stringstream det_text;
    det_text << "Tags: " << best_by_id.size() << " (from " << zarray_size(detections) << ")";
    Size det_size = getTextSize(det_text.str(), FONT_HERSHEY_SIMPLEX, 1.0, 2, nullptr);
    putText(color_frame, det_text.str(), Point(width - det_size.width - 10, 30),
            FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2);

    // Write frame
    writer.write(color_frame);

    // Clean up detections
    apriltag_detections_destroy(detections);

    frame_num++;
    if (frame_num % 100 == 0) {
      cout << "Processed " << frame_num << " frames... "
           << "Avg detections: " << (total_detections / frame_num) << "\r" << flush;
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
  cout << "Total detections: " << total_detections << "\n";
  cout << "Average per frame: " << (total_detections / frame_num) << "\n";
  cout << "Tag ID distribution:\n";
  for (auto &pair : tag_id_counts) {
    cout << "  Tag ID " << pair.first << ": " << pair.second << " detections\n";
  }
  cout << "Output saved to: " << output_path << endl;

  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}













