#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/workerpool.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace cv;
using namespace std;

// Extract and analyze tag pattern in detail
void analyzeQuadPattern(const Mat& frame, const vector<Point2f>& quad, int quad_id, const string& output_dir) {
  cout << "\n========================================" << endl;
  cout << "DETAILED ANALYSIS: Quad " << quad_id << endl;
  cout << "========================================" << endl;
  
  // Refine corners
  vector<Point2f> refined = quad;
  TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001);
  cornerSubPix(frame, refined, Size(5, 5), Size(-1, -1), criteria);
  
  cout << "\nCorner coordinates:" << endl;
  for (int i = 0; i < 4; i++) {
    cout << "  Corner " << i << ": (" << refined[i].x << ", " << refined[i].y << ")" << endl;
  }
  
  // Warp to square
  int tagSize = 36;
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  Mat H = getPerspectiveTransform(refined, dstQuad);
  Mat warped;
  warpPerspective(frame, warped, H, Size(tagSize, tagSize));
  
  // Save warped image
  stringstream ss;
  ss << output_dir << "/quad" << quad_id << "_warped_36x36.png";
  imwrite(ss.str(), warped);
  
  // Try different orientations (rotations)
  cout << "\nTesting different orientations (0°, 90°, 180°, 270°)..." << endl;
  
  for (int rotation = 0; rotation < 4; rotation++) {
    Mat warped_rot = warped.clone();
    for (int r = 0; r < rotation; r++) {
      rotate(warped_rot, warped_rot, ROTATE_90_CLOCKWISE);
    }
    
    // Extract pattern (6x6 grid, border is 2 cells on each side)
    Mat pattern(6, 6, CV_8UC1);
    int borderSize = 4;  // pixels
    int dataSize = tagSize - 2 * borderSize;
    int cellSize = dataSize / 6;
    
    for (int row = 0; row < 6; row++) {
      for (int col = 0; col < 6; col++) {
        int x_start = borderSize + col * cellSize;
        int y_start = borderSize + row * cellSize;
        int x_center = x_start + cellSize / 2;
        int y_center = y_start + cellSize / 2;
        
        if (x_center < tagSize && y_center < tagSize) {
          pattern.at<uchar>(row, col) = warped_rot.at<uchar>(y_center, x_center);
        }
      }
    }
    
    // Analyze border
    int border_black = 0;
    int border_total = 0;
    
    // Top 2 rows
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        border_total++;
        if (pattern.at<uchar>(row, col) < 128) border_black++;
      }
    }
    
    // Bottom 2 rows
    for (int row = 4; row < 6; row++) {
      for (int col = 0; col < 6; col++) {
        border_total++;
        if (pattern.at<uchar>(row, col) < 128) border_black++;
      }
    }
    
    // Left/right borders (middle rows)
    for (int row = 2; row < 4; row++) {
      border_total += 2;
      if (pattern.at<uchar>(row, 0) < 128) border_black++;
      if (pattern.at<uchar>(row, 5) < 128) border_black++;
    }
    
    double border_ratio = (double)border_black / border_total;
    
    cout << "\n  Rotation " << (rotation * 90) << "°:" << endl;
    cout << "    Border black ratio: " << fixed << setprecision(3) << border_ratio 
         << " (" << border_black << "/" << border_total << ")" << endl;
    
    // Print pattern
    cout << "    Pattern (6x6, threshold=128):" << endl;
    for (int row = 0; row < 6; row++) {
      cout << "      ";
      for (int col = 0; col < 6; col++) {
        uchar val = pattern.at<uchar>(row, col);
        char c = (val < 128) ? '1' : '0';  // 1=black, 0=white
        cout << c << " ";
      }
      cout << "  [";
      for (int col = 0; col < 6; col++) {
        uchar val = pattern.at<uchar>(row, col);
        cout << setw(3) << (int)val;
        if (col < 5) cout << ",";
      }
      cout << "]" << endl;
    }
    
    // Try to decode with AprilTag library
    cout << "    Testing with AprilTag detector..." << endl;
    
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    
    td->quad_decimate = 1.0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.25;
    td->nthreads = 4;
    td->wp = workerpool_create(4);
    
    // Create image_u8_t from warped_rot
    image_u8_t im = {
      .width = warped_rot.cols,
      .height = warped_rot.rows,
      .stride = warped_rot.cols,
      .buf = warped_rot.data
    };
    
    zarray_t* detections = apriltag_detector_detect(td, &im);
    int num_detections = zarray_size(detections);
    
    if (num_detections > 0) {
      cout << "    ✓ DETECTED: " << num_detections << " tag(s)" << endl;
      for (int i = 0; i < num_detections; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        cout << "      Tag ID: " << det->id << ", margin: " << fixed << setprecision(2) 
             << det->decision_margin << endl;
        apriltag_detection_destroy(det);
      }
    } else {
      cout << "    ✗ NO DETECTION" << endl;
    }
    
    zarray_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    
    // Save warped rotated image
    stringstream ss2;
    ss2 << output_dir << "/quad" << quad_id << "_warped_rot" << (rotation * 90) << ".png";
    imwrite(ss2.str(), warped_rot);
    
    // Save pattern visualization
    Mat pattern_vis(300, 300, CV_8UC3);
    for (int row = 0; row < 6; row++) {
      for (int col = 0; col < 6; col++) {
        uchar val = pattern.at<uchar>(row, col);
        Scalar color = (val < 128) ? Scalar(0, 0, 0) : Scalar(255, 255, 255);
        Rect cell(col * 50, row * 50, 50, 50);
        rectangle(pattern_vis, cell, color, -1);
        rectangle(pattern_vis, cell, Scalar(128, 128, 128), 1);
      }
    }
    stringstream ss3;
    ss3 << output_dir << "/quad" << quad_id << "_pattern_rot" << (rotation * 90) << ".png";
    imwrite(ss3.str(), pattern_vis);
  }
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <image_path> <output_dir>" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  string output_dir = argv[2];
  
  string mkdir_cmd = "mkdir -p " + output_dir;
  system(mkdir_cmd.c_str());
  
  Mat frame = imread(image_path, IMREAD_GRAYSCALE);
  
  if (frame.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "TAG DECODE FAILURE ANALYSIS" << endl;
  cout << "========================================" << endl;
  
  // Detect quadrilaterals (focus on quads 2 and 3)
  Mat edges;
  Canny(frame, edges, 50, 150);
  
  vector<vector<Point>> contours;
  findContours(edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
  vector<vector<Point2f>> quads;
  double tag_min_area = 500;
  double tag_max_area = 50000;
  
  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area >= tag_min_area && area <= tag_max_area) {
      vector<Point> approx;
      double epsilon = 0.02 * arcLength(contours[i], true);
      approxPolyDP(contours[i], approx, epsilon, true);
      if (approx.size() == 4) {
        vector<Point2f> quad;
        for (int j = 0; j < 4; j++) {
          quad.push_back(Point2f(approx[j].x, approx[j].y));
        }
        quads.push_back(quad);
      }
    }
  }
  
  cout << "\nFound " << quads.size() << " quadrilaterals" << endl;
  cout << "Analyzing Quads 2 and 3 (likely the correct tag)..." << endl;
  
  // Analyze quads 2 and 3
  if (quads.size() > 2) {
    analyzeQuadPattern(frame, quads[2], 2, output_dir);
  }
  
  if (quads.size() > 3) {
    analyzeQuadPattern(frame, quads[3], 3, output_dir);
  }
  
  return 0;
}



