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
  cout << "DEBUGGING TAG FILTERING" << endl;
  cout << "Image: " << image_path << endl;
  cout << "========================================" << endl;
  
  Scalar mean_val, stddev_val;
  meanStdDev(frame, mean_val, stddev_val);
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  cout << "Mean: " << mean_val[0] << ", Std: " << stddev_val[0] << endl;
  
  // Ensure contiguous
  if (!frame.isContinuous()) {
    frame = frame.clone();
  }
  
  // Initialize detector with detailed settings
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  
  // Default parameters
  td->quad_decimate = 1.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;
  td->nthreads = 4;
  td->wp = workerpool_create(4);
  
  // Print detector parameters
  cout << "\nDetector parameters:" << endl;
  cout << "  quad_decimate: " << td->quad_decimate << endl;
  cout << "  quad_sigma: " << td->quad_sigma << endl;
  cout << "  refine_edges: " << td->refine_edges << endl;
  cout << "  decode_sharpening: " << td->decode_sharpening << endl;
  cout << "  nthreads: " << td->nthreads << endl;
  cout << "  nthreads (wp): " << (td->wp ? 4 : 0) << endl;
  
  // Create image_u8_t
  image_u8_t im = {
    .width = frame.cols,
    .height = frame.rows,
    .stride = frame.cols,
    .buf = frame.data
  };
  
  cout << "\n========================================" << endl;
  cout << "RUNNING DETECTION..." << endl;
  cout << "========================================" << endl;
  
  // Detect tags
  zarray_t* detections = apriltag_detector_detect(td, &im);
  int num_detections = zarray_size(detections);
  
  cout << "\nDetection results:" << endl;
  cout << "  Total detections: " << num_detections << endl;
  
  if (num_detections > 0) {
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      cout << "\n  Detection " << i << ":" << endl;
      cout << "    Tag ID: " << det->id << endl;
      cout << "    Decision margin: " << fixed << setprecision(2) << det->decision_margin << endl;
      cout << "    Center: (" << det->c[0] << ", " << det->c[1] << ")" << endl;
      cout << "    Hamming: " << static_cast<int>(det->hamming) << endl;
      cout << "    Corners:" << endl;
      for (int j = 0; j < 4; j++) {
        cout << "      [" << j << "]: (" << det->p[j][0] << ", " << det->p[j][1] << ")" << endl;
      }
    }
  } else {
    cout << "  No tags detected" << endl;
    
    // Try with relaxed parameters to see if we can find anything
    cout << "\n========================================" << endl;
    cout << "TESTING WITH RELAXED PARAMETERS" << endl;
    cout << "========================================" << endl;
    
    // Try quad_decimate = 2.0 (should detect larger tags)
    td->quad_decimate = 2.0;
    zarray_t* detections2 = apriltag_detector_detect(td, &im);
    int num2 = zarray_size(detections2);
    cout << "quad_decimate=2.0: " << num2 << " detections" << endl;
    if (num2 > 0) {
      for (int i = 0; i < num2; i++) {
        apriltag_detection_t* det;
        zarray_get(detections2, i, &det);
        cout << "  Tag ID: " << det->id << ", margin: " << det->decision_margin << endl;
        apriltag_detection_destroy(det);
      }
    }
    zarray_destroy(detections2);
    
    // Try quad_decimate = 1.5
    td->quad_decimate = 1.5;
    zarray_t* detections3 = apriltag_detector_detect(td, &im);
    int num3 = zarray_size(detections3);
    cout << "quad_decimate=1.5: " << num3 << " detections" << endl;
    if (num3 > 0) {
      for (int i = 0; i < num3; i++) {
        apriltag_detection_t* det;
        zarray_get(detections3, i, &det);
        cout << "  Tag ID: " << det->id << ", margin: " << det->decision_margin << endl;
        apriltag_detection_destroy(det);
      }
    }
    zarray_destroy(detections3);
    
    td->quad_decimate = 1.0;  // Reset
  }
  
  // Cleanup
  for (int i = 0; i < num_detections; i++) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    apriltag_detection_destroy(det);
  }
  zarray_destroy(detections);
  
  // Try preprocessing
  cout << "\n========================================" << endl;
  cout << "TESTING WITH PREPROCESSING" << endl;
  cout << "========================================" << endl;
  
  // CLAHE
  Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
  Mat frame_clahe;
  clahe->apply(frame, frame_clahe);
  
  image_u8_t im_clahe = {
    .width = frame_clahe.cols,
    .height = frame_clahe.rows,
    .stride = frame_clahe.cols,
    .buf = frame_clahe.data
  };
  
  td->quad_decimate = 1.0;
  zarray_t* detections_clahe = apriltag_detector_detect(td, &im_clahe);
  int num_clahe = zarray_size(detections_clahe);
  cout << "CLAHE preprocessing: " << num_clahe << " detections" << endl;
  if (num_clahe > 0) {
    for (int i = 0; i < num_clahe; i++) {
      apriltag_detection_t* det;
      zarray_get(detections_clahe, i, &det);
      cout << "  Tag ID: " << det->id << ", margin: " << det->decision_margin << endl;
      apriltag_detection_destroy(det);
    }
  }
  zarray_destroy(detections_clahe);
  
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  
  return 0;
}

