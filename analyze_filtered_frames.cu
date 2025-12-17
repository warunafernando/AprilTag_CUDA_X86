#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace std;
using namespace cv;
using namespace chrono;

// Forward declarations
void scale_detection_coordinates(apriltag_detection_t *det, double decimate_factor);
bool is_valid_detection(apriltag_detection_t *det, int width, int height);
vector<apriltag_detection_t*> filter_duplicates(const zarray_t *detections, int width, int height, double min_distance);

// Scale detection coordinates from decimated to full resolution
void scale_detection_coordinates(apriltag_detection_t *det, double decimate_factor) {
  if (decimate_factor > 1.0 && decimate_factor != 1.5) {
    for (int j = 0; j < 4; j++) {
      det->p[j][0] = (det->p[j][0] - 0.5) * decimate_factor + 0.5;
      det->p[j][1] = (det->p[j][1] - 0.5) * decimate_factor + 0.5;
    }
    det->c[0] = (det->c[0] - 0.5) * decimate_factor + 0.5;
    det->c[1] = (det->c[1] - 0.5) * decimate_factor + 0.5;
  } else if (decimate_factor == 1.5) {
    for (int j = 0; j < 4; j++) {
      det->p[j][0] *= decimate_factor;
      det->p[j][1] *= decimate_factor;
    }
    det->c[0] *= decimate_factor;
    det->c[1] *= decimate_factor;
  }
}

// Check if detection coordinates are within image bounds
bool is_valid_detection(apriltag_detection_t *det, int width, int height) {
  for (int i = 0; i < 4; i++) {
    if (det->p[i][0] < 0 || det->p[i][0] >= width ||
        det->p[i][1] < 0 || det->p[i][1] >= height) {
      return false;
    }
  }
  if (det->c[0] < 0 || det->c[0] >= width ||
      det->c[1] < 0 || det->c[1] >= height) {
    return false;
  }
  return true;
}

// Filter duplicate detections by center distance
vector<apriltag_detection_t*> filter_duplicates(const zarray_t *detections, int width, int height, double min_distance) {
  vector<apriltag_detection_t*> valid_dets;
  
  // First, collect valid detections
  for (int i = 0; i < zarray_size(detections); i++) {
    apriltag_detection_t *det;
    zarray_get(const_cast<zarray_t *>(detections), i, &det);
    if (is_valid_detection(det, width, height)) {
      valid_dets.push_back(det);
    }
  }
  
  if (valid_dets.empty()) {
    return valid_dets;
  }
  
  // Sort by decision_margin (best first)
  sort(valid_dets.begin(), valid_dets.end(),
       [](apriltag_detection_t *a, apriltag_detection_t *b) {
         return a->decision_margin > b->decision_margin;
       });
  
  // Remove duplicates
  vector<apriltag_detection_t*> filtered;
  for (auto *det : valid_dets) {
    bool is_duplicate = false;
    for (auto *existing : filtered) {
      if (existing->id == det->id) {
        double dx = det->c[0] - existing->c[0];
        double dy = det->c[1] - existing->c[1];
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < min_distance) {
          is_duplicate = true;
          break;
        }
      }
    }
    if (!is_duplicate) {
      filtered.push_back(det);
    }
  }
  
  return filtered;
}

struct FrameAnalysis {
  size_t frame_num;
  int detection_count;
  double avg_decision_margin;
  double min_decision_margin;
  double max_decision_margin;
  vector<int> tag_ids;
  vector<double> decision_margins;
  vector<bool> invalid_coords;
  vector<bool> duplicate_flags;
};

int main(int argc, char **argv) {
  string video_path;
  string family = "tag36h11";
  double min_distance = 50.0;
  vector<size_t> frames_to_analyze;
  
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <video_path> [--frames frame1,frame2,...]\n";
    cerr << "  If --frames is not specified, analyzes all frames\n";
    return 1;
  }
  
  video_path = argv[1];
  
  for (int i = 2; i < argc; i++) {
    string arg(argv[i]);
    if (arg == "--frames" && i + 1 < argc) {
      string frames_str = argv[++i];
      // Parse comma-separated frame numbers
      size_t pos = 0;
      while (pos < frames_str.length()) {
        size_t comma = frames_str.find(',', pos);
        string frame_str = (comma == string::npos) ? frames_str.substr(pos) : frames_str.substr(pos, comma - pos);
        frames_to_analyze.push_back(stoul(frame_str));
        if (comma == string::npos) break;
        pos = comma + 1;
      }
    } else if (arg == "--min_distance" && i + 1 < argc) {
      min_distance = atof(argv[++i]);
    }
  }
  
  VideoCapture cap(video_path, CAP_ANY);
  if (!cap.isOpened()) {
    cerr << "Failed to open video: " << video_path << endl;
    return 1;
  }
  cap.set(CAP_PROP_CONVERT_RGB, false);
  
  Mat frame;
  int width, height;
  if (!cap.read(frame)) {
    cerr << "Could not read first frame\n";
    return 1;
  }
  width = frame.cols;
  height = frame.rows;
  
  // Setup detector
  apriltag_family_t *tf = nullptr;
  setup_tag_family(&tf, family.c_str());
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  
  // Use tuned parameters
  td->qtp.min_cluster_pixels = 8;
  td->qtp.max_line_fit_mse = 6.0;
  td->qtp.cos_critical_rad = cos(7.0 * M_PI / 180.0);
  td->qtp.min_white_black_diff = 8;
  
  frc971::apriltag::GpuDetector detector(td);
  
  vector<FrameAnalysis> analyses;
  size_t frame_num = 0;
  
  do {
    // Skip frames if specific frames requested
    if (!frames_to_analyze.empty()) {
      if (find(frames_to_analyze.begin(), frames_to_analyze.end(), frame_num) == frames_to_analyze.end()) {
        frame_num++;
        continue;
      }
    }
    
    detector.Detect(frame.data);
    const zarray_t *detections = detector.Detections();
    int detections_before_count = zarray_size(detections);
    
    if (detections_before_count == 0) {
      frame_num++;
      continue;
    }
    
    // Scale coordinates
    const double gpu_decimate = td->quad_decimate;
    if (gpu_decimate > 1.0) {
      for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);
        scale_detection_coordinates(det, gpu_decimate);
      }
    }
    
    // Analyze detections before filtering
    FrameAnalysis analysis;
    analysis.frame_num = frame_num;
    analysis.detection_count = detections_before_count;
    
    double sum_margin = 0.0;
    double min_margin = 1e9;
    double max_margin = -1e9;
    
    for (int i = 0; i < zarray_size(detections); i++) {
      apriltag_detection_t *det;
      zarray_get(const_cast<zarray_t *>(detections), i, &det);
      
      analysis.tag_ids.push_back(det->id);
      analysis.decision_margins.push_back(det->decision_margin);
      analysis.invalid_coords.push_back(!is_valid_detection(det, width, height));
      
      sum_margin += det->decision_margin;
      min_margin = min(min_margin, det->decision_margin);
      max_margin = max(max_margin, det->decision_margin);
    }
    
    analysis.avg_decision_margin = sum_margin / detections_before_count;
    analysis.min_decision_margin = min_margin;
    analysis.max_decision_margin = max_margin;
    
    // Apply filtering
    vector<apriltag_detection_t*> filtered = filter_duplicates(detections, width, height, min_distance);
    
    // Check if frame lost all detections
    if (filtered.empty()) {
      // Mark which detections were duplicates
      for (size_t i = 0; i < analysis.tag_ids.size(); i++) {
        bool was_duplicate = false;
        for (int j = 0; j < zarray_size(detections); j++) {
          apriltag_detection_t *det;
          zarray_get(const_cast<zarray_t *>(detections), j, &det);
          if (j == (int)i) continue;
          
          if (det->id == analysis.tag_ids[i]) {
            double dx = analysis.tag_ids[i] - det->id; // Simplified check
            was_duplicate = true;
            break;
          }
        }
        analysis.duplicate_flags.push_back(was_duplicate);
      }
      
      analyses.push_back(analysis);
    }
    
    frame_num++;
    
    // Limit analysis to first 1000 frames or specified frames
    if (frame_num >= 1000 && frames_to_analyze.empty()) break;
    
  } while (cap.read(frame));
  
  cap.release();
  
  // Print analysis
  cout << "\n=== Analysis of Frames Lost to Filtering ===\n\n";
  cout << "Total frames analyzed that lost all detections: " << analyses.size() << "\n\n";
  
  if (analyses.empty()) {
    cout << "No frames found that lost all detections.\n";
    return 0;
  }
  
  // Statistics
  double total_avg_margin = 0.0;
  double total_min_margin = 1e9;
  double total_max_margin = -1e9;
  int total_invalid = 0;
  int total_detections = 0;
  
  for (const auto &analysis : analyses) {
    total_avg_margin += analysis.avg_decision_margin;
    total_min_margin = min(total_min_margin, analysis.min_decision_margin);
    total_max_margin = max(total_max_margin, analysis.max_decision_margin);
    
    for (bool invalid : analysis.invalid_coords) {
      if (invalid) total_invalid++;
    }
    total_detections += analysis.detection_count;
  }
  
  cout << "Statistics:\n";
  cout << "  Average detections per frame: " << fixed << setprecision(2) << (double)total_detections / analyses.size() << "\n";
  cout << "  Average decision margin: " << (total_avg_margin / analyses.size()) << "\n";
  cout << "  Min decision margin: " << total_min_margin << "\n";
  cout << "  Max decision margin: " << total_max_margin << "\n";
  cout << "  Frames with invalid coordinates: " << total_invalid << " detections (" << (100.0 * total_invalid / total_detections) << "%)\n";
  cout << "\n";
  
  // Detailed frame-by-frame analysis (first 10)
  cout << "First 10 frames with details:\n";
  for (size_t i = 0; i < min(analyses.size(), size_t(10)); i++) {
    const auto &a = analyses[i];
    cout << "\nFrame " << a.frame_num << ":\n";
    cout << "  Detections: " << a.detection_count << "\n";
    cout << "  Decision margins: avg=" << fixed << setprecision(2) << a.avg_decision_margin 
         << ", min=" << a.min_decision_margin << ", max=" << a.max_decision_margin << "\n";
    cout << "  Tag IDs: ";
    for (size_t j = 0; j < a.tag_ids.size(); j++) {
      cout << a.tag_ids[j];
      if (a.invalid_coords[j]) cout << "(invalid)";
      if (j < a.tag_ids.size() - 1) cout << ", ";
    }
    cout << "\n";
  }
  
  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}


