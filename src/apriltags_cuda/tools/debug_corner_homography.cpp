#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

void refineCorners(const Mat& gray, vector<Point2f>& corners, int winSize = 5, int maxIter = 30) {
  if (corners.size() != 4) return;
  
  TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, maxIter, 0.001);
  cornerSubPix(gray, corners, Size(winSize, winSize), Size(-1, -1), criteria);
}

Mat getPerspectiveTransformFromQuad(const vector<Point2f>& srcQuad, int tagSize = 36) {
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  return getPerspectiveTransform(srcQuad, dstQuad);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cerr << "Usage: " << argv[0] << " <image_path> <output_dir>" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  string output_dir = argv[2];
  
  // Create output directory
  string mkdir_cmd = "mkdir -p " + output_dir;
  system(mkdir_cmd.c_str());
  
  Mat frame = imread(image_path, IMREAD_GRAYSCALE);
  
  if (frame.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "CORNER REFINEMENT & HOMOGRAPHY DEBUG" << endl;
  cout << "========================================" << endl;
  cout << "Input image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  
  // Save original
  imwrite(output_dir + "/00_original.png", frame);
  cout << "\n[STAGE 0] Saved original frame" << endl;
  
  int stage_num = 1;
  
  // STAGE 1: Detect quadrilaterals (same as contour detection)
  cout << "\n[STAGE 1] Detecting quadrilaterals..." << endl;
  
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
  
  cout << "  Found " << quads.size() << " quadrilaterals" << endl;
  
  // Draw initial quads
  Mat initial_quads = frame.clone();
  cvtColor(initial_quads, initial_quads, COLOR_GRAY2BGR);
  for (size_t i = 0; i < quads.size(); i++) {
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(initial_quads, quads[i][j], quads[i][next], Scalar(0, 255, 0), 2);
      circle(initial_quads, quads[i][j], 5, Scalar(0, 0, 255), -1);
      string label = to_string(j);
      putText(initial_quads, label, quads[i][j] + Point2f(5, 5),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
    }
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_initial_quads.png", initial_quads);
  cout << "  Saved initial quadrilaterals" << endl;
  
  if (quads.empty()) {
    cout << "\nNo quadrilaterals found. Cannot proceed with corner refinement." << endl;
    return 0;
  }
  
  // STAGE 2: Corner refinement
  cout << "\n[STAGE 2] Refining corners..." << endl;
  
  vector<vector<Point2f>> refined_quads;
  for (size_t i = 0; i < quads.size(); i++) {
    vector<Point2f> refined = quads[i];
    
    cout << "  Quad " << i << " before refinement:" << endl;
    for (int j = 0; j < 4; j++) {
      cout << "    Corner " << j << ": (" << refined[j].x << ", " << refined[j].y << ")" << endl;
    }
    
    // Refine corners
    refineCorners(frame, refined, 5, 30);
    refined_quads.push_back(refined);
    
    cout << "  Quad " << i << " after refinement:" << endl;
    for (int j = 0; j < 4; j++) {
      double dx = refined[j].x - quads[i][j].x;
      double dy = refined[j].y - quads[i][j].y;
      double dist = sqrt(dx*dx + dy*dy);
      cout << "    Corner " << j << ": (" << refined[j].x << ", " << refined[j].y 
           << ") - moved " << fixed << setprecision(2) << dist << " pixels" << endl;
    }
  }
  
  // Draw refined quads
  Mat refined_vis = frame.clone();
  cvtColor(refined_vis, refined_vis, COLOR_GRAY2BGR);
  for (size_t i = 0; i < refined_quads.size(); i++) {
    // Draw original (green)
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(refined_vis, quads[i][j], quads[i][next], Scalar(0, 255, 0), 1);
      circle(refined_vis, quads[i][j], 3, Scalar(0, 255, 0), -1);
    }
    
    // Draw refined (red)
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(refined_vis, refined_quads[i][j], refined_quads[i][next], Scalar(0, 0, 255), 2);
      circle(refined_vis, refined_quads[i][j], 5, Scalar(0, 0, 255), -1);
    }
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_refined_corners.png", refined_vis);
  cout << "  Saved refined corners visualization (green=original, red=refined)" << endl;
  
  // STAGE 3: Homography transformation
  cout << "\n[STAGE 3] Computing homography transformations..." << endl;
  
  vector<Mat> homographies;
  int tagSize = 36;  // Standard AprilTag size
  
  for (size_t i = 0; i < refined_quads.size(); i++) {
    Mat H = getPerspectiveTransformFromQuad(refined_quads[i], tagSize);
    homographies.push_back(H);
    cout << "  Quad " << i << " homography matrix computed" << endl;
  }
  
  // STAGE 4: Warp quadrilaterals to squares
  cout << "\n[STAGE 4] Warping quadrilaterals to squares..." << endl;
  
  for (size_t i = 0; i < refined_quads.size(); i++) {
    Mat warped;
    warpPerspective(frame, warped, homographies[i], Size(tagSize, tagSize));
    
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num 
       << "_warped_quad" << i << "_" << tagSize << "x" << tagSize << ".png";
    imwrite(ss.str(), warped);
    
    // Statistics
    Scalar mean_val, stddev_val;
    meanStdDev(warped, mean_val, stddev_val);
    double min_val, max_val;
    minMaxLoc(warped, &min_val, &max_val);
    
    cout << "  Quad " << i << " warped to " << tagSize << "x" << tagSize << ":" << endl;
    cout << "    Mean: " << mean_val[0] << ", Std: " << stddev_val[0] << endl;
    cout << "    Range: " << min_val << " - " << max_val << endl;
    cout << "    Dynamic range: " << (max_val - min_val) << endl;
    
    stage_num++;
  }
  
  // STAGE 5: Enhanced warped images
  cout << "\n[STAGE 5] Creating enhanced warped images..." << endl;
  
  for (size_t i = 0; i < refined_quads.size(); i++) {
    Mat warped;
    warpPerspective(frame, warped, homographies[i], Size(tagSize, tagSize));
    
    // Histogram equalization
    Mat warped_eq;
    equalizeHist(warped, warped_eq);
    stringstream ss1;
    ss1 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_warped_quad" << i << "_histeq.png";
    imwrite(ss1.str(), warped_eq);
    stage_num++;
    
    // CLAHE
    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat warped_clahe;
    clahe->apply(warped, warped_clahe);
    stringstream ss2;
    ss2 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_warped_quad" << i << "_clahe.png";
    imwrite(ss2.str(), warped_clahe);
    stage_num++;
    
    // Threshold (binary)
    Mat warped_thresh;
    threshold(warped, warped_thresh, 127, 255, THRESH_BINARY);
    stringstream ss3;
    ss3 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_warped_quad" << i << "_threshold.png";
    imwrite(ss3.str(), warped_thresh);
    stage_num++;
    
    // Adaptive threshold
    Mat warped_adapt;
    adaptiveThreshold(warped, warped_adapt, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    stringstream ss4;
    ss4 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_warped_quad" << i << "_adapt_thresh.png";
    imwrite(ss4.str(), warped_adapt);
    stage_num++;
    
    cout << "  Quad " << i << ": saved original, histeq, clahe, threshold, adaptive threshold" << endl;
  }
  
  // STAGE 6: Larger warped size for better visualization
  cout << "\n[STAGE 6] Creating larger warped images (10x)..." << endl;
  
  int largeSize = tagSize * 10;
  for (size_t i = 0; i < refined_quads.size(); i++) {
    Mat warped_large;
    vector<Point2f> dstQuad_large;
    dstQuad_large.push_back(Point2f(0, 0));
    dstQuad_large.push_back(Point2f(largeSize - 1, 0));
    dstQuad_large.push_back(Point2f(largeSize - 1, largeSize - 1));
    dstQuad_large.push_back(Point2f(0, largeSize - 1));
    
    Mat H_large = getPerspectiveTransform(refined_quads[i], dstQuad_large);
    warpPerspective(frame, warped_large, H_large, Size(largeSize, largeSize));
    
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num 
       << "_warped_quad" << i << "_" << largeSize << "x" << largeSize << ".png";
    imwrite(ss.str(), warped_large);
    stage_num++;
    
    cout << "  Quad " << i << " warped to " << largeSize << "x" << largeSize << " (10x scale)" << endl;
  }
  
  // STAGE 7: Overlay warped regions on original
  cout << "\n[STAGE 7] Creating overlay visualization..." << endl;
  
  Mat overlay = frame.clone();
  cvtColor(overlay, overlay, COLOR_GRAY2BGR);
  
  for (size_t i = 0; i < refined_quads.size(); i++) {
    // Draw quad
    Scalar color = Scalar(rand() % 256, rand() % 256, rand() % 256);
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(overlay, refined_quads[i][j], refined_quads[i][next], color, 2);
    }
    
    // Draw corners
    for (int j = 0; j < 4; j++) {
      circle(overlay, refined_quads[i][j], 5, Scalar(0, 0, 255), -1);
      string label = to_string(j);
      putText(overlay, label, refined_quads[i][j] + Point2f(5, 5),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
    }
    
    // Draw quad number
    Point2f center = (refined_quads[i][0] + refined_quads[i][1] + 
                      refined_quads[i][2] + refined_quads[i][3]) / 4;
    string num_label = "Quad " + to_string(i);
    putText(overlay, num_label, center, FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
  }
  
  imwrite(output_dir + "/" + to_string(stage_num++) + "_overlay_all_quads.png", overlay);
  cout << "  Saved overlay with all detected quads" << endl;
  
  // Summary
  cout << "\n========================================" << endl;
  cout << "SUMMARY" << endl;
  cout << "========================================" << endl;
  cout << "Quadrilaterals detected: " << quads.size() << endl;
  cout << "Corners refined: " << refined_quads.size() << endl;
  cout << "Homography transformations computed: " << homographies.size() << endl;
  cout << "\nAll outputs saved to: " << output_dir << endl;
  cout << "Total files created: " << stage_num << endl;
  
  return 0;
}



