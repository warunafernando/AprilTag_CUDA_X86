#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/workerpool.h>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <image_path> <output_dir>" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  string output_dir = argv[2];
  
  // Create output directory if it doesn't exist
  string mkdir_cmd = "mkdir -p " + output_dir;
  system(mkdir_cmd.c_str());
  
  Mat frame = imread(image_path, IMREAD_GRAYSCALE);
  
  if (frame.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "STAGE-BY-STAGE APRILTAG DETECTION DEBUG" << endl;
  cout << "========================================" << endl;
  cout << "Input image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  
  // Save original frame
  imwrite(output_dir + "/00_original.png", frame);
  cout << "\n[STAGE 0] Saved original frame" << endl;
  
  Scalar mean_val, stddev_val;
  meanStdDev(frame, mean_val, stddev_val);
  cout << "  Mean: " << mean_val[0] << ", Std: " << stddev_val[0] << endl;
  
  // Ensure contiguous
  if (!frame.isContinuous()) {
    frame = frame.clone();
  }
  
  // STAGE 1: Different preprocessing methods
  cout << "\n[STAGE 1] Preprocessing methods..." << endl;
  
  // 1a. Original (no preprocessing)
  Mat stage1a = frame.clone();
  imwrite(output_dir + "/01a_original.png", stage1a);
  cout << "  1a. Original saved" << endl;
  
  // 1b. Histogram equalization
  Mat stage1b;
  equalizeHist(frame, stage1b);
  imwrite(output_dir + "/01b_histogram_equalized.png", stage1b);
  Scalar mean1b, std1b;
  meanStdDev(stage1b, mean1b, std1b);
  cout << "  1b. Histogram equalized (Mean=" << mean1b[0] << ", Std=" << std1b[0] << ")" << endl;
  
  // 1c. CLAHE (various clip limits)
  for (double clip : {2.0, 3.0, 4.0, 5.0}) {
    Ptr<CLAHE> clahe = createCLAHE(clip, Size(8, 8));
    Mat stage1c;
    clahe->apply(frame, stage1c);
    stringstream ss;
    ss << output_dir << "/01c_clahe_clip" << clip << ".png";
    imwrite(ss.str(), stage1c);
    Scalar mean1c, std1c;
    meanStdDev(stage1c, mean1c, std1c);
    cout << "  1c. CLAHE clip=" << clip << " (Mean=" << mean1c[0] << ", Std=" << std1c[0] << ")" << endl;
  }
  
  // 1d. Gamma correction
  for (double gamma : {1.2, 1.5, 1.8, 2.0}) {
    double inv_gamma = 1.0 / gamma;
    Mat table(1, 256, CV_8U);
    uchar* p = table.ptr();
    for (int i = 0; i < 256; i++) {
      p[i] = saturate_cast<uchar>(pow(i / 255.0, inv_gamma) * 255.0);
    }
    Mat stage1d;
    LUT(frame, table, stage1d);
    stringstream ss;
    ss << output_dir << "/01d_gamma" << gamma << ".png";
    imwrite(ss.str(), stage1d);
    Scalar mean1d, std1d;
    meanStdDev(stage1d, mean1d, std1d);
    cout << "  1d. Gamma=" << gamma << " (Mean=" << mean1d[0] << ", Std=" << std1d[0] << ")" << endl;
  }
  
  // 1e. Contrast enhancement
  Mat stage1e;
  frame.convertTo(stage1e, -1, 1.5, 0);  // alpha=1.5 (contrast)
  imwrite(output_dir + "/01e_contrast_1.5x.png", stage1e);
  Scalar mean1e, std1e;
  meanStdDev(stage1e, mean1e, std1e);
  cout << "  1e. Contrast 1.5x (Mean=" << mean1e[0] << ", Std=" << std1e[0] << ")" << endl;
  
  // STAGE 2: Initialize detector
  cout << "\n[STAGE 2] Initializing AprilTag detector..." << endl;
  apriltag_family_t* tf = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  
  td->quad_decimate = 1.0;
  td->quad_sigma = 0.0;
  td->refine_edges = 1;
  td->decode_sharpening = 0.25;
  td->nthreads = 4;
  td->wp = workerpool_create(4);
  
  cout << "  Parameters:" << endl;
  cout << "    quad_decimate: " << td->quad_decimate << endl;
  cout << "    quad_sigma: " << td->quad_sigma << endl;
  cout << "    refine_edges: " << td->refine_edges << endl;
  cout << "    decode_sharpening: " << td->decode_sharpening << endl;
  cout << "    nthreads: " << td->nthreads << endl;
  
  // STAGE 3: Test detection on different preprocessed images
  cout << "\n[STAGE 3] Running detection on preprocessed images..." << endl;
  
  vector<pair<string, Mat>> test_images = {
    {"original", stage1a},
    {"histogram_equalized", stage1b},
    {"contrast_1.5x", stage1e}
  };
  
  // Add CLAHE versions
  for (double clip : {2.0, 3.0, 4.0, 5.0}) {
    Ptr<CLAHE> clahe = createCLAHE(clip, Size(8, 8));
    Mat clahe_img;
    clahe->apply(frame, clahe_img);
    stringstream ss;
    ss << "clahe_clip" << clip;
    test_images.push_back({ss.str(), clahe_img});
  }
  
  // Add gamma versions
  for (double gamma : {1.2, 1.5, 1.8, 2.0}) {
    double inv_gamma = 1.0 / gamma;
    Mat table(1, 256, CV_8U);
    uchar* p = table.ptr();
    for (int i = 0; i < 256; i++) {
      p[i] = saturate_cast<uchar>(pow(i / 255.0, inv_gamma) * 255.0);
    }
    Mat gamma_img;
    LUT(frame, table, gamma_img);
    stringstream ss;
    ss << "gamma" << gamma;
    test_images.push_back({ss.str(), gamma_img});
  }
  
  int stage_num = 3;
  for (const auto& test_pair : test_images) {
    string name = test_pair.first;
    Mat test_img = test_pair.second;
    
    // Ensure contiguous
    if (!test_img.isContinuous()) {
      test_img = test_img.clone();
    }
    
    // Create image_u8_t
    image_u8_t im = {
      .width = test_img.cols,
      .height = test_img.rows,
      .stride = test_img.cols,
      .buf = test_img.data
    };
    
    // Detect
    zarray_t* detections = apriltag_detector_detect(td, &im);
    int num_detections = zarray_size(detections);
    
    // Create visualization
    Mat vis;
    if (test_img.channels() == 1) {
      cvtColor(test_img, vis, COLOR_GRAY2BGR);
    } else {
      vis = test_img.clone();
    }
    
    // Draw detections
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      
      // Draw corners
      for (int j = 0; j < 4; j++) {
        int next = (j + 1) % 4;
        line(vis, Point(det->p[j][0], det->p[j][1]),
             Point(det->p[next][0], det->p[next][1]),
             Scalar(0, 255, 0), 2);
      }
      
      // Draw center
      circle(vis, Point(det->c[0], det->c[1]), 5, Scalar(0, 0, 255), -1);
      
      // Draw ID
      string text = "ID:" + to_string(det->id) + " M:" + to_string((int)det->decision_margin);
      putText(vis, text, Point(det->c[0] - 30, det->c[1] - 10),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
      
      apriltag_detection_destroy(det);
    }
    
    // Save visualization
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num << "_detection_" << name;
    if (num_detections > 0) {
      ss << "_FOUND" << num_detections << ".png";
    } else {
      ss << "_NONE.png";
    }
    imwrite(ss.str(), vis);
    
    cout << "  " << setw(2) << stage_num << ". " << name << ": " << num_detections << " detection(s)" << endl;
    zarray_destroy(detections);
    stage_num++;
  }
  
  // STAGE 4: Test with different quad_decimate values
  cout << "\n[STAGE 4] Testing different quad_decimate values..." << endl;
  Mat best_preprocessed = stage1a;  // Start with original
  
  for (double decimate : {1.0, 1.5, 2.0, 2.5, 3.0}) {
    td->quad_decimate = decimate;
    
    image_u8_t im = {
      .width = best_preprocessed.cols,
      .height = best_preprocessed.rows,
      .stride = best_preprocessed.cols,
      .buf = best_preprocessed.data
    };
    
    zarray_t* detections = apriltag_detector_detect(td, &im);
    int num_detections = zarray_size(detections);
    
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num << "_quad_decimate_" << decimate;
    if (num_detections > 0) {
      ss << "_FOUND" << num_detections << ".png";
    } else {
      ss << "_NONE.png";
    }
    
    // Create visualization
    Mat vis;
    cvtColor(best_preprocessed, vis, COLOR_GRAY2BGR);
    for (int i = 0; i < num_detections; i++) {
      apriltag_detection_t* det;
      zarray_get(detections, i, &det);
      for (int j = 0; j < 4; j++) {
        int next = (j + 1) % 4;
        line(vis, Point(det->p[j][0], det->p[j][1]),
             Point(det->p[next][0], det->p[next][1]),
             Scalar(0, 255, 0), 2);
      }
      circle(vis, Point(det->c[0], det->c[1]), 5, Scalar(0, 0, 255), -1);
      apriltag_detection_destroy(det);
    }
    imwrite(ss.str(), vis);
    
    cout << "  quad_decimate=" << decimate << ": " << num_detections << " detection(s)" << endl;
    zarray_destroy(detections);
    stage_num++;
  }
  
  td->quad_decimate = 1.0;  // Reset
  
  // STAGE 5: Summary
  cout << "\n[STAGE 5] Summary" << endl;
  cout << "All intermediate outputs saved to: " << output_dir << endl;
  
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
  
  return 0;
}








