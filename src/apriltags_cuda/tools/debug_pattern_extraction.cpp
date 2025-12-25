#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

// Extract tag pattern grid from warped image
Mat extractTagPattern(const Mat& warped, int tagSize = 36, int borderSize = 4) {
  // Tag36h11 has 6x6 data grid + border
  // Border is typically 2 cells on each side, so we skip borderSize pixels
  int dataSize = tagSize - 2 * borderSize;
  int cellSize = dataSize / 6;  // 6x6 grid
  
  Mat pattern(6, 6, CV_8UC1);
  
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      // Calculate region in warped image
      int x_start = borderSize + col * cellSize;
      int y_start = borderSize + row * cellSize;
      
      // Sample center of cell
      int x_center = x_start + cellSize / 2;
      int y_center = y_start + cellSize / 2;
      
      if (x_center < tagSize && y_center < tagSize) {
        pattern.at<uchar>(row, col) = warped.at<uchar>(y_center, x_center);
      }
    }
  }
  
  return pattern;
}

// Create visual representation of tag pattern
Mat visualizePattern(const Mat& pattern, int cellSize = 50) {
  Mat vis(cellSize * 6, cellSize * 6, CV_8UC3);
  
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      uchar val = pattern.at<uchar>(row, col);
      
      // Determine if cell is black or white (threshold at 128)
      Scalar color = (val < 128) ? Scalar(0, 0, 0) : Scalar(255, 255, 255);
      
      Rect cell(col * cellSize, row * cellSize, cellSize, cellSize);
      rectangle(vis, cell, color, -1);
      
      // Draw border
      rectangle(vis, cell, Scalar(128, 128, 128), 1);
      
      // Show pixel value
      stringstream ss;
      ss << (int)val;
      putText(vis, ss.str(), Point(col * cellSize + 5, row * cellSize + 15),
              FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 0, 0), 1);
    }
  }
  
  return vis;
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
  cout << "PATTERN EXTRACTION DEBUG" << endl;
  cout << "========================================" << endl;
  cout << "Input image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  
  // Save original
  imwrite(output_dir + "/00_original.png", frame);
  cout << "\n[STAGE 0] Saved original frame" << endl;
  
  int stage_num = 1;
  
  // STAGE 1: Detect and warp quadrilaterals (reuse from previous tool)
  cout << "\n[STAGE 1] Detecting and warping quadrilaterals..." << endl;
  
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
  
  // Refine corners
  vector<vector<Point2f>> refined_quads;
  for (size_t i = 0; i < quads.size(); i++) {
    vector<Point2f> refined = quads[i];
    TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001);
    cornerSubPix(frame, refined, Size(5, 5), Size(-1, -1), criteria);
    refined_quads.push_back(refined);
  }
  
  // Warp to squares
  int tagSize = 36;
  vector<Mat> warped_images;
  for (size_t i = 0; i < refined_quads.size(); i++) {
    vector<Point2f> dstQuad;
    dstQuad.push_back(Point2f(0, 0));
    dstQuad.push_back(Point2f(tagSize - 1, 0));
    dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
    dstQuad.push_back(Point2f(0, tagSize - 1));
    
    Mat H = getPerspectiveTransform(refined_quads[i], dstQuad);
    Mat warped;
    warpPerspective(frame, warped, H, Size(tagSize, tagSize));
    warped_images.push_back(warped);
    
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num 
       << "_warped_quad" << i << "_" << tagSize << "x" << tagSize << ".png";
    imwrite(ss.str(), warped);
    stage_num++;
  }
  
  if (warped_images.empty()) {
    cout << "\nNo quadrilaterals found. Cannot proceed with pattern extraction." << endl;
    return 0;
  }
  
  // STAGE 2: Extract patterns from warped images
  cout << "\n[STAGE 2] Extracting patterns from warped images..." << endl;
  
  for (size_t i = 0; i < warped_images.size(); i++) {
    cout << "\n  Quad " << i << ":" << endl;
    
    Mat warped = warped_images[i];
    
    // Statistics of warped image
    Scalar mean_val, stddev_val;
    meanStdDev(warped, mean_val, stddev_val);
    double min_val, max_val;
    minMaxLoc(warped, &min_val, &max_val);
    
    cout << "    Warped image - Mean: " << mean_val[0] << ", Std: " << stddev_val[0] << endl;
    cout << "    Range: " << min_val << " - " << max_val << endl;
    
    // Extract pattern
    Mat pattern = extractTagPattern(warped, tagSize, 4);
    
    // Visualize pattern
    Mat pattern_vis = visualizePattern(pattern, 50);
    stringstream ss1;
    ss1 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_visualization.png";
    imwrite(ss1.str(), pattern_vis);
    stage_num++;
    
    // Save raw pattern as image (6x6)
    Mat pattern_large;
    resize(pattern, pattern_large, Size(300, 300), 0, 0, INTER_NEAREST);
    stringstream ss2;
    ss2 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_6x6.png";
    imwrite(ss2.str(), pattern_large);
    stage_num++;
    
    // Print pattern values
    cout << "    Extracted pattern (6x6 grid):" << endl;
    for (int row = 0; row < 6; row++) {
      cout << "      ";
      for (int col = 0; col < 6; col++) {
        uchar val = pattern.at<uchar>(row, col);
        cout << setw(4) << (int)val << " ";
      }
      cout << endl;
    }
    
    // Binary pattern (thresholded at 128)
    cout << "    Binary pattern (threshold=128):" << endl;
    for (int row = 0; row < 6; row++) {
      cout << "      ";
      for (int col = 0; col < 6; col++) {
        uchar val = pattern.at<uchar>(row, col);
        char c = (val < 128) ? '1' : '0';  // 1=black, 0=white
        cout << c << " ";
      }
      cout << endl;
    }
    
    // STAGE 3: Different preprocessing methods
    cout << "\n    Testing different preprocessing..." << endl;
    
    // Histogram equalization
    Mat warped_eq;
    equalizeHist(warped, warped_eq);
    Mat pattern_eq = extractTagPattern(warped_eq, tagSize, 4);
    Mat pattern_eq_vis = visualizePattern(pattern_eq, 50);
    stringstream ss3;
    ss3 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_histeq.png";
    imwrite(ss3.str(), pattern_eq_vis);
    stage_num++;
    
    // CLAHE
    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat warped_clahe;
    clahe->apply(warped, warped_clahe);
    Mat pattern_clahe = extractTagPattern(warped_clahe, tagSize, 4);
    Mat pattern_clahe_vis = visualizePattern(pattern_clahe, 50);
    stringstream ss4;
    ss4 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_clahe.png";
    imwrite(ss4.str(), pattern_clahe_vis);
    stage_num++;
    
    // Adaptive threshold
    Mat warped_adapt;
    adaptiveThreshold(warped, warped_adapt, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    Mat pattern_adapt = extractTagPattern(warped_adapt, tagSize, 4);
    Mat pattern_adapt_vis = visualizePattern(pattern_adapt, 50);
    stringstream ss5;
    ss5 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_adapt_thresh.png";
    imwrite(ss5.str(), pattern_adapt_vis);
    stage_num++;
    
    // Otsu threshold
    Mat warped_otsu;
    threshold(warped, warped_otsu, 0, 255, THRESH_BINARY + THRESH_OTSU);
    Mat pattern_otsu = extractTagPattern(warped_otsu, tagSize, 4);
    Mat pattern_otsu_vis = visualizePattern(pattern_otsu, 50);
    stringstream ss6;
    ss6 << output_dir << "/" << setfill('0') << setw(2) << stage_num 
        << "_pattern_quad" << i << "_otsu.png";
    imwrite(ss6.str(), pattern_otsu_vis);
    stage_num++;
    
    // STAGE 4: Pattern analysis
    cout << "    Pattern analysis:" << endl;
    
    // Check border pattern (should be all black for valid tag)
    // For Tag36h11, border is 2 cells on each side
    int border_black_count = 0;
    int border_total = 0;
    
    // Top border (rows 0-1)
    for (int row = 0; row < 2; row++) {
      for (int col = 0; col < 6; col++) {
        border_total++;
        if (pattern.at<uchar>(row, col) < 128) border_black_count++;
      }
    }
    
    // Bottom border (rows 4-5)
    for (int row = 4; row < 6; row++) {
      for (int col = 0; col < 6; col++) {
        border_total++;
        if (pattern.at<uchar>(row, col) < 128) border_black_count++;
      }
    }
    
    // Left/right borders (middle rows)
    for (int row = 2; row < 4; row++) {
      border_total += 2;
      if (pattern.at<uchar>(row, 0) < 128) border_black_count++;
      if (pattern.at<uchar>(row, 5) < 128) border_black_count++;
    }
    
    double border_black_ratio = (double)border_black_count / border_total;
    cout << "      Border black ratio: " << fixed << setprecision(2) 
         << border_black_ratio << " (" << border_black_count << "/" << border_total << ")" << endl;
    cout << "      (Should be ~1.0 for valid tag border)" << endl;
    
    // Data region contrast
    int data_region_sum = 0;
    int data_count = 0;
    for (int row = 2; row < 4; row++) {
      for (int col = 1; col < 5; col++) {
        data_region_sum += pattern.at<uchar>(row, col);
        data_count++;
      }
    }
    double data_mean = (double)data_region_sum / data_count;
    cout << "      Data region mean: " << fixed << setprecision(2) << data_mean << endl;
  }
  
  // Summary
  cout << "\n========================================" << endl;
  cout << "SUMMARY" << endl;
  cout << "========================================" << endl;
  cout << "Quadrilaterals processed: " << warped_images.size() << endl;
  cout << "Patterns extracted: " << warped_images.size() << endl;
  cout << "\nAll outputs saved to: " << output_dir << endl;
  cout << "Total files created: " << stage_num << endl;
  
  return 0;
}








