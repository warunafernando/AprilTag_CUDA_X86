#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <image_path> [brightness] [contrast] [gamma]" << endl;
    cerr << "  brightness: -100 to 100 (default: 0)" << endl;
    cerr << "  contrast: 0.1 to 3.0 (default: 1.0)" << endl;
    cerr << "  gamma: 0.1 to 3.0 (default: 1.0)" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  double brightness = (argc >= 3) ? atof(argv[2]) : 0.0;
  double contrast = (argc >= 4) ? atof(argv[3]) : 1.0;
  double gamma = (argc >= 5) ? atof(argv[4]) : 1.0;
  
  Mat original = imread(image_path, IMREAD_GRAYSCALE);
  if (original.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  Mat processed = original.clone();
  
  // Apply brightness/contrast
  if (brightness != 0.0 || contrast != 1.0) {
    processed.convertTo(processed, -1, contrast, brightness);
  }
  
  // Apply gamma correction
  if (gamma != 1.0) {
    Mat lookup_table(1, 256, CV_8U);
    uchar* p = lookup_table.ptr();
    for (int i = 0; i < 256; i++) {
      p[i] = saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
    }
    Mat gamma_corrected;
    LUT(processed, lookup_table, gamma_corrected);
    processed = gamma_corrected;
  }
  
  // Statistics
  Scalar mean_proc, stddev_proc;
  meanStdDev(processed, mean_proc, stddev_proc);
  
  cout << "\n========================================" << endl;
  cout << "DETECTION TEST ON ADJUSTED IMAGE" << endl;
  cout << "========================================" << endl;
  cout << "Image: " << image_path << endl;
  cout << "Brightness: " << brightness << ", Contrast: " << contrast << ", Gamma: " << gamma << endl;
  cout << "Processed image - Mean: " << fixed << setprecision(2) << mean_proc[0] 
       << ", Std: " << stddev_proc[0] << endl;
  
  // Initialize detector
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  
  td->quad_decimate = 1.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;
  td->nthreads = 4;
  
  // Detect
  image_u8_t im = {
    .width = processed.cols,
    .height = processed.rows,
    .stride = processed.cols,
    .buf = processed.data
  };
  
  zarray_t* detections = apriltag_detector_detect(td, &im);
  int num_detections = zarray_size(detections);
  
  cout << "\nDetection results:" << endl;
  cout << "  Tags detected: " << num_detections << endl;
  
  if (num_detections > 0) {
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      cout << "    Tag ID: " << det->id 
           << ", margin: " << fixed << setprecision(2) << det->decision_margin << endl;
      apriltag_detection_destroy(det);
    }
  }
  
  // Cleanup
  zarray_destroy(detections);
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  
  cout << "\n========================================" << endl;
  
  return 0;
}








