#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace std;
using namespace cv;

namespace {

double Percentile(const std::vector<double> &vals, double pct) {
  if (vals.empty()) return 0.0;
  auto copy = vals;
  std::sort(copy.begin(), copy.end());
  double idx = (pct / 100.0) * (copy.size() - 1);
  size_t lower = static_cast<size_t>(idx);
  size_t upper = std::min(lower + 1, copy.size() - 1);
  double frac = idx - lower;
  return copy[lower] + (copy[upper] - copy[lower]) * frac;
}

}  // namespace

int main(int argc, char **argv) {
  string video_path = "/home/nav/Apriltag/input/Stable.avi";
  string family = "tag36h11";
  string csv_path;
  int max_frames = -1;

  for (int i = 1; i < argc; ++i) {
    string arg(argv[i]);
    if ((arg == "--video" || arg == "-v") && i + 1 < argc) {
      video_path = argv[++i];
    } else if ((arg == "--family" || arg == "-f") && i + 1 < argc) {
      family = argv[++i];
    } else if (arg == "--csv" && i + 1 < argc) {
      csv_path = argv[++i];
    } else if (arg == "--max_frames" && i + 1 < argc) {
      max_frames = atoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      cout << "Usage: video_bench --video <path> --family <tag36h11> "
              "[--csv <out.csv>] [--max_frames N]\n";
      return 0;
    }
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

  frc971::apriltag::CameraMatrix cam{905.495617, 609.916016, 907.909470,
                                     352.682645};
  frc971::apriltag::DistCoeffs dist{0.059238, -0.075154, -0.003801, 0.001113,
                                    0.0};

  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);

  size_t frames = 0;
  auto t_start = chrono::steady_clock::now();
  vector<double> frame_ms;
  vector<int> detections_per_frame;
  map<int, int> tag_id_counts;  // tag_id -> count of frames it appears in
  map<int, vector<int>> tag_id_per_frame;  // frame -> vector of tag IDs detected
  int frames_with_detections = 0;
  int frames_without_detections = 0;

  do {
    CV_Assert(frame.type() == CV_8UC1);
    auto f_start = chrono::steady_clock::now();
    detector.Detect(frame.data);
    auto f_end = chrono::steady_clock::now();
    double ms = chrono::duration<double, milli>(f_end - f_start).count();
    frame_ms.push_back(ms);
    
    // Get detections for this frame
    const zarray_t *detections = detector.Detections();
    int num_detections = zarray_size(detections);
    detections_per_frame.push_back(num_detections);
    
    vector<int> frame_tag_ids;
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t *det;
      zarray_get(const_cast<zarray_t *>(detections), i, &det);
      int tag_id = det->id;
      frame_tag_ids.push_back(tag_id);
      tag_id_counts[tag_id]++;
    }
    tag_id_per_frame[frames] = frame_tag_ids;
    
    if (num_detections > 0) {
      frames_with_detections++;
    } else {
      frames_without_detections++;
    }
    
    ++frames;
    if (max_frames > 0 && static_cast<int>(frames) >= max_frames) break;
  } while (cap.read(frame));

  auto t_end = chrono::steady_clock::now();
  double total_s = chrono::duration<double>(t_end - t_start).count();
  double fps = frames / total_s;

  double p50 = Percentile(frame_ms, 50.0);
  double p90 = Percentile(frame_ms, 90.0);
  double p99 = Percentile(frame_ms, 99.0);

  cout << fixed << setprecision(2);
  cout << "Frames: " << frames << "  FPS: " << fps << "\n";
  cout << "Frame ms p50/p90/p99: " << p50 << " / " << p90 << " / " << p99
       << endl;
  
  // Detection statistics
  cout << "\n=== Detection Statistics ===\n";
  cout << "Frames with detections: " << frames_with_detections << " (" 
       << setprecision(1) << (100.0 * frames_with_detections / frames) << "%)\n";
  cout << "Frames without detections: " << frames_without_detections << " (" 
       << (100.0 * frames_without_detections / frames) << "%)\n";
  
  if (!detections_per_frame.empty()) {
    int max_detections = *max_element(detections_per_frame.begin(), detections_per_frame.end());
    int min_detections = *min_element(detections_per_frame.begin(), detections_per_frame.end());
    double avg_detections = accumulate(detections_per_frame.begin(), detections_per_frame.end(), 0.0) / detections_per_frame.size();
    cout << setprecision(2) << "Detections per frame - min: " << min_detections 
         << ", max: " << max_detections << ", avg: " << avg_detections << "\n";
  }
  
  cout << "\nTag IDs detected (frequency across frames):\n";
  if (tag_id_counts.empty()) {
    cout << "  No tags detected in any frame\n";
  } else {
    for (const auto &pair : tag_id_counts) {
      cout << "  Tag ID " << pair.first << ": appears in " << pair.second 
           << " frames (" << setprecision(1) 
           << (100.0 * pair.second / frames) << "%)\n";
    }
  }
  
  // Find frames with detection issues
  cout << "\n=== Frame-by-Frame Analysis ===\n";
  int consecutive_misses = 0;
  int max_consecutive_misses = 0;
  vector<int> frames_missing_detections;
  
  for (size_t i = 0; i < detections_per_frame.size(); i++) {
    if (detections_per_frame[i] == 0) {
      consecutive_misses++;
      frames_missing_detections.push_back(i);
      max_consecutive_misses = max(max_consecutive_misses, consecutive_misses);
    } else {
      consecutive_misses = 0;
    }
  }
  
  if (frames_missing_detections.empty()) {
    cout << "All frames have detections!\n";
  } else {
    cout << "Frames without detections: " << frames_missing_detections.size() << "\n";
    cout << "Max consecutive frames without detections: " << max_consecutive_misses << "\n";
    if (frames_missing_detections.size() <= 20) {
      cout << "Frame numbers missing detections: ";
      for (size_t i = 0; i < frames_missing_detections.size(); i++) {
        cout << frames_missing_detections[i];
        if (i < frames_missing_detections.size() - 1) cout << ", ";
      }
      cout << "\n";
    } else {
      cout << "First 10 frames missing detections: ";
      for (int i = 0; i < 10 && i < static_cast<int>(frames_missing_detections.size()); i++) {
        cout << frames_missing_detections[i];
        if (i < 9 && i < static_cast<int>(frames_missing_detections.size()) - 1) cout << ", ";
      }
      cout << "\n... and " << (frames_missing_detections.size() - 10) << " more\n";
    }
  }

  if (!csv_path.empty()) {
    FILE *f = fopen(csv_path.c_str(), "w");
    if (f) {
      fprintf(f, "frames,fps,p50_ms,p90_ms,p99_ms,frames_with_detections,frames_without_detections,unique_tag_ids\n");
      fprintf(f, "%zu,%.2f,%.3f,%.3f,%.3f,%d,%d,%zu\n", 
              frames, fps, p50, p90, p99, 
              frames_with_detections, frames_without_detections, tag_id_counts.size());
      
      // Write per-frame detection data
      string frame_csv = csv_path;
      size_t dot_pos = frame_csv.find_last_of('.');
      if (dot_pos != string::npos) {
        frame_csv.insert(dot_pos, "_per_frame");
      } else {
        frame_csv += "_per_frame";
      }
      
      FILE *f_frame = fopen(frame_csv.c_str(), "w");
      if (f_frame) {
        fprintf(f_frame, "frame_num,num_detections,tag_ids\n");
        for (size_t i = 0; i < frames; i++) {
          fprintf(f_frame, "%zu,%d,", i, detections_per_frame[i]);
          const vector<int> &tag_ids = tag_id_per_frame[i];
          for (size_t j = 0; j < tag_ids.size(); j++) {
            fprintf(f_frame, "%d", tag_ids[j]);
            if (j < tag_ids.size() - 1) fprintf(f_frame, ";");
          }
          fprintf(f_frame, "\n");
        }
        fclose(f_frame);
        cout << "\nPer-frame detection data written to: " << frame_csv << "\n";
      }
      
      fclose(f);
    } else {
      cerr << "Failed to write csv: " << csv_path << endl;
    }
  }

  apriltag_detector_destroy(td);
  teardown_tag_family(&tf, family.c_str());
  return 0;
}





