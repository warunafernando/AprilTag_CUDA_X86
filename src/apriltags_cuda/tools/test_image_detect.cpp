#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/workerpool.h>
#include <iostream>
#include <iomanip>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <image_path>" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  Mat frame = imread(image_path, IMREAD_GRAYSCALE);
  
  if (frame.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "TESTING: " << image_path << endl;
  cout << "========================================" << endl;
  cout << "Frame dimensions: " << frame.cols << "x" << frame.rows << endl;
  
  // Statistics
  Scalar mean_val, stddev_val;
  meanStdDev(frame, mean_val, stddev_val);
  cout << "Mean pixel value: " << mean_val[0] << endl;
  cout << "Std deviation: " << stddev_val[0] << endl;
  
  double min_val, max_val;
  minMaxLoc(frame, &min_val, &max_val);
  cout << "Pixel range: " << min_val << " - " << max_val << endl;
  
  // Ensure contiguous
  if (!frame.isContinuous()) {
    frame = frame.clone();
  }
  
  // Initialize detector (same as simple_camera_detect)
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  
  td->quad_decimate = 1.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;
  td->nthreads = 4;
  td->wp = workerpool_create(4);
  
  cout << "\nDetector parameters:" << endl;
  cout << "  quad_decimate: " << td->quad_decimate << endl;
  cout << "  refine_edges: " << td->refine_edges << endl;
  cout << "  nthreads: " << td->nthreads << endl;
  
  // Create image_u8_t
  image_u8_t im = {
    .width = frame.cols,
    .height = frame.rows,
    .stride = frame.cols,
    .buf = frame.data
  };
  
  // Detect
  cout << "\nRunning detection on original image..." << endl;
  zarray_t* detections = apriltag_detector_detect(td, &im);
  int num_detections = zarray_size(detections);
  
  cout << "\nDetection results:" << endl;
  cout << "  Number of detections: " << num_detections << endl;
  
  if (num_detections > 0) {
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      cout << "  Tag ID: " << det->id << ", margin: " << fixed << setprecision(2) << det->decision_margin << endl;
    }
  }
  
  // Cleanup
  for (int i = 0; i < num_detections; i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    apriltag_detection_destroy(det);
  }
  zarray_destroy(detections);
  
  // Try with preprocessing if no detections
  if (num_detections == 0) {
    cout << "\nNo detections in original. Trying preprocessing..." << endl;
    
    // Test 1: Histogram equalization
    Mat frame_eq;
    equalizeHist(frame, frame_eq);
    image_u8_t im_eq = {
      .width = frame_eq.cols,
      .height = frame_eq.rows,
      .stride = frame_eq.cols,
      .buf = frame_eq.data
    };
    zarray_t* detections_eq = apriltag_detector_detect(td, &im_eq);
    int num_eq = zarray_size(detections_eq);
    cout << "  Histogram equalized: " << num_eq << " detections" << endl;
    for (int i = 0; i < num_eq; i++) {
      apriltag_detection_t* det;
      zarray_get(detections_eq, i, &det);
      cout << "    Tag ID: " << det->id << ", margin: " << det->decision_margin << endl;
      apriltag_detection_destroy(det);
    }
    zarray_destroy(detections_eq);
    
    // Test 2: CLAHE
    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat frame_clahe;
    clahe->apply(frame, frame_clahe);
    image_u8_t im_clahe = {
      .width = frame_clahe.cols,
      .height = frame_clahe.rows,
      .stride = frame_clahe.cols,
      .buf = frame_clahe.data
    };
    zarray_t* detections_clahe = apriltag_detector_detect(td, &im_clahe);
    int num_clahe = zarray_size(detections_clahe);
    cout << "  CLAHE: " << num_clahe << " detections" << endl;
    for (int i = 0; i < num_clahe; i++) {
      apriltag_detection_t* det;
      zarray_get(detections_clahe, i, &det);
      cout << "    Tag ID: " << det->id << ", margin: " << det->decision_margin << endl;
      apriltag_detection_destroy(det);
    }
    zarray_destroy(detections_clahe);
  }
  
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  
  return 0;
}








