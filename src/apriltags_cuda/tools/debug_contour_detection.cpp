#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

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
  cout << "CONTOUR DETECTION DEBUG" << endl;
  cout << "========================================" << endl;
  cout << "Input image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  
  // Save original
  imwrite(output_dir + "/00_original.png", frame);
  cout << "\n[STAGE 0] Saved original frame" << endl;
  
  int stage_num = 1;
  
  // STAGE 1: Edge detection for contour finding
  cout << "\n[STAGE 1] Edge detection for contours..." << endl;
  
  Mat edges;
  Canny(frame, edges, 50, 150);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_edges_canny50_150.png", edges);
  cout << "  Saved Canny edges (50, 150)" << endl;
  
  // STAGE 2: Find contours
  cout << "\n[STAGE 2] Finding contours..." << endl;
  
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
  cout << "  Found " << contours.size() << " contours" << endl;
  
  // Draw all contours
  Mat all_contours = frame.clone();
  cvtColor(all_contours, all_contours, COLOR_GRAY2BGR);
  drawContours(all_contours, contours, -1, Scalar(0, 255, 0), 1);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_all_contours.png", all_contours);
  cout << "  Saved all contours visualization" << endl;
  
  // STAGE 3: Filter contours by area
  cout << "\n[STAGE 3] Filtering contours by area..." << endl;
  
  double min_area = 100;  // Minimum contour area
  double max_area = 1000000;  // Maximum contour area
  
  vector<vector<Point>> filtered_contours;
  for (size_t i = 0; i < contours.size(); i++) {
    double area = contourArea(contours[i]);
    if (area >= min_area && area <= max_area) {
      filtered_contours.push_back(contours[i]);
    }
  }
  
  cout << "  Contours with area between " << min_area << " and " << max_area << ": " 
       << filtered_contours.size() << endl;
  
  Mat filtered_vis = frame.clone();
  cvtColor(filtered_vis, filtered_vis, COLOR_GRAY2BGR);
  drawContours(filtered_vis, filtered_contours, -1, Scalar(0, 255, 0), 2);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_filtered_contours.png", filtered_vis);
  cout << "  Saved filtered contours visualization" << endl;
  
  // STAGE 4: Approximate contours to polygons
  cout << "\n[STAGE 4] Approximating contours to polygons..." << endl;
  
  vector<vector<Point>> approx_contours;
  for (size_t i = 0; i < filtered_contours.size(); i++) {
    vector<Point> approx;
    double epsilon = 0.02 * arcLength(filtered_contours[i], true);
    approxPolyDP(filtered_contours[i], approx, epsilon, true);
    if (approx.size() >= 3) {  // At least triangle
      approx_contours.push_back(approx);
    }
  }
  
  cout << "  Approximated contours: " << approx_contours.size() << endl;
  
  Mat approx_vis = frame.clone();
  cvtColor(approx_vis, approx_vis, COLOR_GRAY2BGR);
  for (size_t i = 0; i < approx_contours.size(); i++) {
    Scalar color = Scalar(rand() % 256, rand() % 256, rand() % 256);
    drawContours(approx_vis, approx_contours, i, color, 2);
    
    // Draw vertices
    for (size_t j = 0; j < approx_contours[i].size(); j++) {
      circle(approx_vis, approx_contours[i][j], 3, Scalar(0, 0, 255), -1);
    }
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_approximated_contours.png", approx_vis);
  cout << "  Saved approximated contours visualization" << endl;
  
  // STAGE 5: Find quadrilateral contours (4 vertices)
  cout << "\n[STAGE 5] Finding quadrilateral contours (4 vertices)..." << endl;
  
  vector<vector<Point>> quad_contours;
  for (size_t i = 0; i < approx_contours.size(); i++) {
    if (approx_contours[i].size() == 4) {
      quad_contours.push_back(approx_contours[i]);
    }
  }
  
  cout << "  Quadrilateral contours found: " << quad_contours.size() << endl;
  
  Mat quad_vis = frame.clone();
  cvtColor(quad_vis, quad_vis, COLOR_GRAY2BGR);
  for (size_t i = 0; i < quad_contours.size(); i++) {
    Scalar color = Scalar(0, 255, 0);  // Green for quads
    drawContours(quad_vis, quad_contours, i, color, 2);
    
    // Draw vertices with numbers
    for (size_t j = 0; j < quad_contours[i].size(); j++) {
      circle(quad_vis, quad_contours[i][j], 5, Scalar(0, 0, 255), -1);
      string label = to_string(j);
      putText(quad_vis, label, quad_contours[i][j] + Point(5, 5),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
    }
    
    // Calculate area
    double area = contourArea(quad_contours[i]);
    Point center = (quad_contours[i][0] + quad_contours[i][1] + 
                    quad_contours[i][2] + quad_contours[i][3]) / 4;
    string area_label = "A:" + to_string((int)area);
    putText(quad_vis, area_label, center, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255), 1);
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_quadrilateral_contours.png", quad_vis);
  cout << "  Saved quadrilateral contours visualization" << endl;
  
  // STAGE 6: Filter quads by area (reasonable tag size)
  cout << "\n[STAGE 6] Filtering quadrilaterals by area (tag-like sizes)..." << endl;
  
  double tag_min_area = 500;
  double tag_max_area = 50000;
  
  vector<vector<Point>> tag_quads;
  for (size_t i = 0; i < quad_contours.size(); i++) {
    double area = contourArea(quad_contours[i]);
    if (area >= tag_min_area && area <= tag_max_area) {
      tag_quads.push_back(quad_contours[i]);
    }
  }
  
  cout << "  Tag-sized quadrilaterals (" << tag_min_area << " to " << tag_max_area 
       << " pixels): " << tag_quads.size() << endl;
  
  Mat tag_quads_vis = frame.clone();
  cvtColor(tag_quads_vis, tag_quads_vis, COLOR_GRAY2BGR);
  for (size_t i = 0; i < tag_quads.size(); i++) {
    Scalar color = Scalar(0, 255, 255);  // Cyan for tag-sized quads
    drawContours(tag_quads_vis, tag_quads, i, color, 3);
    
    for (size_t j = 0; j < tag_quads[i].size(); j++) {
      circle(tag_quads_vis, tag_quads[i][j], 5, Scalar(0, 0, 255), -1);
    }
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_tag_sized_quads.png", tag_quads_vis);
  cout << "  Saved tag-sized quadrilaterals visualization" << endl;
  
  // STAGE 7: Check convexity (tags should be convex)
  cout << "\n[STAGE 7] Checking convexity..." << endl;
  
  vector<vector<Point>> convex_quads;
  for (size_t i = 0; i < tag_quads.size(); i++) {
    vector<Point> hull;
    convexHull(tag_quads[i], hull);
    if (hull.size() == tag_quads[i].size()) {  // Already convex
      convex_quads.push_back(tag_quads[i]);
    }
  }
  
  cout << "  Convex quadrilaterals: " << convex_quads.size() << endl;
  
  Mat convex_vis = frame.clone();
  cvtColor(convex_vis, convex_vis, COLOR_GRAY2BGR);
  for (size_t i = 0; i < convex_quads.size(); i++) {
    Scalar color = Scalar(255, 0, 255);  // Magenta for convex
    drawContours(convex_vis, convex_quads, i, color, 3);
    
    for (size_t j = 0; j < convex_quads[i].size(); j++) {
      circle(convex_vis, convex_quads[i][j], 5, Scalar(0, 0, 255), -1);
    }
  }
  imwrite(output_dir + "/" + to_string(stage_num++) + "_convex_quads.png", convex_vis);
  cout << "  Saved convex quadrilaterals visualization" << endl;
  
  // STAGE 8: Test with different Canny thresholds
  cout << "\n[STAGE 8] Testing different edge thresholds..." << endl;
  
  vector<pair<int, int>> thresholds = {{30, 100}, {50, 150}, {75, 200}, {100, 250}};
  
  for (const auto& thresh : thresholds) {
    Mat test_edges;
    Canny(frame, test_edges, thresh.first, thresh.second);
    
    vector<vector<Point>> test_contours;
    findContours(test_edges, test_contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    
    // Filter by area and approximate
    vector<vector<Point>> test_quads;
    for (size_t i = 0; i < test_contours.size(); i++) {
      double area = contourArea(test_contours[i]);
      if (area >= tag_min_area && area <= tag_max_area) {
        vector<Point> approx;
        double epsilon = 0.02 * arcLength(test_contours[i], true);
        approxPolyDP(test_contours[i], approx, epsilon, true);
        if (approx.size() == 4) {
          test_quads.push_back(approx);
        }
      }
    }
    
    cout << "  Canny(" << thresh.first << "," << thresh.second << "): " 
         << test_quads.size() << " tag-sized quads" << endl;
    
    Mat test_vis = frame.clone();
    cvtColor(test_vis, test_vis, COLOR_GRAY2BGR);
    drawContours(test_vis, test_quads, -1, Scalar(0, 255, 0), 2);
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num 
       << "_quads_canny" << thresh.first << "_" << thresh.second << ".png";
    imwrite(ss.str(), test_vis);
    stage_num++;
  }
  
  // STAGE 9: Preprocessed images
  cout << "\n[STAGE 9] Testing with preprocessed images..." << endl;
  
  // CLAHE
  Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
  Mat frame_clahe;
  clahe->apply(frame, frame_clahe);
  
  Mat edges_clahe;
  Canny(frame_clahe, edges_clahe, 50, 150);
  vector<vector<Point>> contours_clahe;
  findContours(edges_clahe, contours_clahe, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
  vector<vector<Point>> quads_clahe;
  for (size_t i = 0; i < contours_clahe.size(); i++) {
    double area = contourArea(contours_clahe[i]);
    if (area >= tag_min_area && area <= tag_max_area) {
      vector<Point> approx;
      double epsilon = 0.02 * arcLength(contours_clahe[i], true);
      approxPolyDP(contours_clahe[i], approx, epsilon, true);
      if (approx.size() == 4) {
        quads_clahe.push_back(approx);
      }
    }
  }
  
  cout << "  CLAHE preprocessing: " << quads_clahe.size() << " tag-sized quads" << endl;
  
  Mat clahe_vis = frame_clahe.clone();
  cvtColor(clahe_vis, clahe_vis, COLOR_GRAY2BGR);
  drawContours(clahe_vis, quads_clahe, -1, Scalar(0, 255, 0), 2);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_quads_clahe.png", clahe_vis);
  
  // Summary
  cout << "\n========================================" << endl;
  cout << "SUMMARY" << endl;
  cout << "========================================" << endl;
  cout << "Total contours found: " << contours.size() << endl;
  cout << "Filtered by area: " << filtered_contours.size() << endl;
  cout << "Approximated polygons: " << approx_contours.size() << endl;
  cout << "Quadrilaterals: " << quad_contours.size() << endl;
  cout << "Tag-sized quadrilaterals: " << tag_quads.size() << endl;
  cout << "Convex quadrilaterals: " << convex_quads.size() << endl;
  cout << "\nAll outputs saved to: " << output_dir << endl;
  cout << "Total files created: " << stage_num << endl;
  
  return 0;
}








