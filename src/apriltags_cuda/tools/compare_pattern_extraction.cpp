#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace cv;
using namespace std;

vector<vector<Point2f>> detectQuads(const Mat& frame) {
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
        
        // Refine corners
        vector<Point2f> refined = quad;
        TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001);
        cornerSubPix(frame, refined, Size(5, 5), Size(-1, -1), criteria);
        
        quads.push_back(refined);
      }
    }
  }
  
  return quads;
}

void extractAndVisualizePattern(const Mat& frame, const vector<Point2f>& quad, int quad_id,
                                const string& label, const string& output_dir) {
  int tagSize = 200;
  
  // Warp to square
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  Mat H = getPerspectiveTransform(quad, dstQuad);
  Mat warped;
  warpPerspective(frame, warped, H, Size(tagSize, tagSize), INTER_CUBIC);
  
  // Extract 6x6 pattern (for Tag36h11)
  // Border is ~2 cells on each side, so data is 6x6 in center
  Mat pattern(6, 6, CV_8UC1);
  int borderSize = tagSize / 9;  // Approximately 2 cells
  
  int dataSize = tagSize - 2 * borderSize;
  int cellSize = dataSize / 6;
  
  vector<vector<int>> pattern_values(6, vector<int>(6));
  
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      int x_start = borderSize + col * cellSize;
      int y_start = borderSize + row * cellSize;
      int x_center = x_start + cellSize / 2;
      int y_center = y_start + cellSize / 2;
      
      // Bounds checking
      if (x_center >= 0 && x_center < tagSize && y_center >= 0 && y_center < tagSize) {
        uchar val = warped.at<uchar>(y_center, x_center);
        pattern.at<uchar>(row, col) = val;
        pattern_values[row][col] = (int)val;
      } else {
        // Use edge pixel if center is out of bounds
        x_center = min(max(0, x_center), tagSize - 1);
        y_center = min(max(0, y_center), tagSize - 1);
        uchar val = warped.at<uchar>(y_center, x_center);
        pattern.at<uchar>(row, col) = val;
        pattern_values[row][col] = (int)val;
      }
    }
  }
  
  // Analyze border
  int border_black = 0;
  int border_total = 0;
  
  // Top 2 rows (border)
  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 6; col++) {
      border_total++;
      if (pattern_values[row][col] < 128) border_black++;
    }
  }
  
  // Bottom 2 rows (border)
  for (int row = 4; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      border_total++;
      if (pattern_values[row][col] < 128) border_black++;
    }
  }
  
  // Left/right borders (middle rows)
  for (int row = 2; row < 4; row++) {
    border_total += 2;
    if (pattern_values[row][0] < 128) border_black++;
    if (pattern_values[row][5] < 128) border_black++;
  }
  
  double border_ratio = (double)border_black / border_total;
  
  // Data region (center 4x4)
  double data_mean = 0;
  int data_count = 0;
  for (int row = 2; row < 4; row++) {
    for (int col = 1; col < 5; col++) {
      data_mean += pattern_values[row][col];
      data_count++;
    }
  }
  data_mean /= data_count;
  
  // Create pattern visualization (300x300 for visibility)
  int vis_size = 300;
  Mat pattern_vis(vis_size, vis_size, CV_8UC3);
  int cell_vis_size = vis_size / 6;
  
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      int val = pattern_values[row][col];
      Scalar color = (val < 128) ? Scalar(0, 0, 0) : Scalar(255, 255, 255);
      
      Rect cell(col * cell_vis_size, row * cell_vis_size, cell_vis_size, cell_vis_size);
      rectangle(pattern_vis, cell, color, -1);
      rectangle(pattern_vis, cell, Scalar(128, 128, 128), 2);
      
      // Draw value
      stringstream ss_val;
      ss_val << val;
      int font_scale = (cell_vis_size > 40) ? 1 : 0;
      putText(pattern_vis, ss_val.str(), 
              Point(col * cell_vis_size + 5, row * cell_vis_size + cell_vis_size / 2),
              FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0), 1);
    }
  }
  
  // Highlight border region
  rectangle(pattern_vis, Point(0, 0), Point(vis_size-1, 2*cell_vis_size-1), Scalar(0, 255, 0), 3);
  rectangle(pattern_vis, Point(0, 4*cell_vis_size), Point(vis_size-1, vis_size-1), Scalar(0, 255, 0), 3);
  rectangle(pattern_vis, Point(0, 2*cell_vis_size), Point(cell_vis_size-1, 4*cell_vis_size-1), Scalar(0, 255, 0), 3);
  rectangle(pattern_vis, Point(5*cell_vis_size, 2*cell_vis_size), Point(vis_size-1, 4*cell_vis_size-1), Scalar(0, 255, 0), 3);
  
  // Add statistics text
  stringstream ss_stats;
  ss_stats << "Border ratio: " << fixed << setprecision(3) << border_ratio 
           << " (" << border_black << "/" << border_total << ")";
  putText(pattern_vis, ss_stats.str(), Point(10, vis_size - 40), 
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
  
  stringstream ss_data;
  ss_data << "Data mean: " << fixed << setprecision(1) << data_mean;
  putText(pattern_vis, ss_data.str(), Point(10, vis_size - 15), 
          FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
  
  // Create comprehensive visualization
  int spacing = 20;
  int label_height = 40;
  int total_width = tagSize + vis_size + spacing * 3;
  int total_height = tagSize + label_height * 2;
  
  Mat vis(total_height, total_width, CV_8UC3, Scalar(64, 64, 64));
  
  // Convert warped to color
  Mat warped_color;
  cvtColor(warped, warped_color, COLOR_GRAY2BGR);
  
  // Place warped image (ensure bounds)
  int warped_x = spacing;
  int warped_y = label_height;
  int warped_w = min(tagSize, total_width - warped_x);
  int warped_h = min(tagSize, total_height - warped_y);
  if (warped_w > 0 && warped_h > 0) {
    warped_color(Rect(0, 0, warped_w, warped_h)).copyTo(vis(Rect(warped_x, warped_y, warped_w, warped_h)));
  }
  
  // Place pattern visualization (ensure bounds)
  int pattern_x = tagSize + spacing * 2;
  int pattern_y = label_height;
  int pattern_w = min(vis_size, total_width - pattern_x);
  int pattern_h = min(vis_size, total_height - pattern_y);
  if (pattern_w > 0 && pattern_h > 0) {
    pattern_vis(Rect(0, 0, pattern_w, pattern_h)).copyTo(vis(Rect(pattern_x, pattern_y, pattern_w, pattern_h)));
  }
  
  // Add labels
  string title = label + " Quad " + to_string(quad_id);
  putText(vis, title, Point(spacing, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  putText(vis, "Warped (200x200)", Point(spacing, label_height + tagSize + 20), 
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
  putText(vis, "6x6 Pattern (Green=Border)", Point(tagSize + spacing * 2, 25), 
          FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  
  // Add warped statistics
  Scalar mean_w, stddev_w;
  meanStdDev(warped, mean_w, stddev_w);
  double min_val, max_val;
  minMaxLoc(warped, &min_val, &max_val);
  
  stringstream ss_warped;
  ss_warped << "Mean: " << fixed << setprecision(1) << mean_w[0] 
            << " Std: " << setprecision(1) << stddev_w[0]
            << " Range: " << (int)min_val << "-" << (int)max_val;
  putText(vis, ss_warped.str(), Point(spacing, label_height + tagSize + 35), 
          FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0), 1);
  
  // Save individual visualization
  stringstream ss_path;
  ss_path << output_dir << "/" << label << "_quad" << quad_id << "_pattern.png";
  imwrite(ss_path.str(), vis);
  
  // Print pattern to console
  cout << "\n" << label << " Quad " << quad_id << " Pattern (6x6):" << endl;
  cout << "  Border black ratio: " << fixed << setprecision(3) << border_ratio 
       << " (" << border_black << "/" << border_total << ")" << endl;
  cout << "  Data region mean: " << fixed << setprecision(1) << data_mean << endl;
  cout << "  Pattern values:" << endl;
  for (int row = 0; row < 6; row++) {
    cout << "    ";
    for (int col = 0; col < 6; col++) {
      char c = (pattern_values[row][col] < 128) ? '1' : '0';  // 1=black, 0=white
      cout << c << " ";
    }
    cout << "  [";
    for (int col = 0; col < 6; col++) {
      cout << setw(3) << pattern_values[row][col];
      if (col < 5) cout << ",";
    }
    cout << "]" << endl;
  }
  
  // Also save just the pattern visualization
  stringstream ss_pattern;
  ss_pattern << output_dir << "/" << label << "_quad" << quad_id << "_pattern_only.png";
  imwrite(ss_pattern.str(), pattern_vis);
  
  // Save warped image
  stringstream ss_warped_file;
  ss_warped_file << output_dir << "/" << label << "_quad" << quad_id << "_warped.png";
  imwrite(ss_warped_file.str(), warped);
}

int main(int argc, char** argv) {
  if (argc < 4) {
    cerr << "Usage: " << argv[0] << " <stable_image> <newvideo_image> <output_dir>" << endl;
    return 1;
  }
  
  string stable_path = argv[1];
  string newvideo_path = argv[2];
  string output_dir = argv[3];
  
  string mkdir_cmd = "mkdir -p " + output_dir;
  system(mkdir_cmd.c_str());
  
  Mat frame_stable = imread(stable_path, IMREAD_GRAYSCALE);
  Mat frame_newvideo = imread(newvideo_path, IMREAD_GRAYSCALE);
  
  if (frame_stable.empty() || frame_newvideo.empty()) {
    cerr << "Failed to load images" << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "PATTERN EXTRACTION COMPARISON" << endl;
  cout << "========================================" << endl;
  
  // Detect quads
  vector<vector<Point2f>> quads_stable = detectQuads(frame_stable);
  vector<vector<Point2f>> quads_newvideo = detectQuads(frame_newvideo);
  
  cout << "Stable.avi: Found " << quads_stable.size() << " quadrilaterals" << endl;
  cout << "NewVideo.avi: Found " << quads_newvideo.size() << " quadrilaterals" << endl;
  
  if (quads_stable.size() == 0 || quads_newvideo.size() < 3) {
    cerr << "Error: Need at least 1 quad from Stable and 3 quads from NewVideo" << endl;
    return 1;
  }
  
  // Extract patterns
  cout << "\nExtracting patterns..." << endl;
  extractAndVisualizePattern(frame_stable, quads_stable[0], 0, "Stable", output_dir);
  extractAndVisualizePattern(frame_newvideo, quads_newvideo[2], 2, "NewVideo", output_dir);
  
  // Create side-by-side comparison
  Mat stable_pattern = imread(output_dir + "/Stable_quad0_pattern.png");
  Mat newvideo_pattern = imread(output_dir + "/NewVideo_quad2_pattern.png");
  
  if (!stable_pattern.empty() && !newvideo_pattern.empty()) {
    int max_height = max(stable_pattern.rows, newvideo_pattern.rows);
    int total_width = stable_pattern.cols + newvideo_pattern.cols + 40;
    Mat comparison(max_height, total_width, CV_8UC3, Scalar(32, 32, 32));
    
    // Ensure we don't exceed bounds
    int stable_w = min(stable_pattern.cols, total_width - 10);
    int stable_h = min(stable_pattern.rows, max_height);
    int newvideo_w = min(newvideo_pattern.cols, total_width - stable_pattern.cols - 30);
    int newvideo_h = min(newvideo_pattern.rows, max_height);
    
    stable_pattern(Rect(0, 0, stable_w, stable_h)).copyTo(comparison(Rect(10, 0, stable_w, stable_h)));
    newvideo_pattern(Rect(0, 0, newvideo_w, newvideo_h)).copyTo(comparison(Rect(stable_pattern.cols + 30, 0, newvideo_w, newvideo_h)));
    
    putText(comparison, "PATTERN EXTRACTION COMPARISON", 
            Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
    
    string comparison_path = output_dir + "/pattern_extraction_comparison.png";
    imwrite(comparison_path, comparison);
    cout << "\n  Saved: " << comparison_path << endl;
  }
  
  // Create side-by-side pattern-only comparison
  Mat stable_pattern_only = imread(output_dir + "/Stable_quad0_pattern_only.png");
  Mat newvideo_pattern_only = imread(output_dir + "/NewVideo_quad2_pattern_only.png");
  
  if (!stable_pattern_only.empty() && !newvideo_pattern_only.empty()) {
    int pattern_size = min(stable_pattern_only.rows, stable_pattern_only.cols);
    int total_w = pattern_size * 2 + 40;
    Mat pattern_comparison(pattern_size, total_w, CV_8UC3, Scalar(64, 64, 64));
    
    // Ensure bounds
    int size = min(pattern_size, total_w - 30);
    stable_pattern_only(Rect(0, 0, size, size)).copyTo(pattern_comparison(Rect(10, 0, size, size)));
    newvideo_pattern_only(Rect(0, 0, size, size)).copyTo(pattern_comparison(Rect(pattern_size + 30, 0, size, size)));
    
    putText(pattern_comparison, "Stable Quad0", Point(10, pattern_size - 30), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
    putText(pattern_comparison, "NewVideo Quad2", Point(pattern_size + 30, pattern_size - 30), 
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 2);
    
    putText(pattern_comparison, "PATTERN COMPARISON (6x6)", 
            Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
    
    string pattern_comparison_path = output_dir + "/pattern_only_comparison.png";
    imwrite(pattern_comparison_path, pattern_comparison);
    cout << "  Saved: " << pattern_comparison_path << endl;
  }
  
  cout << "\n========================================" << endl;
  cout << "PATTERN EXTRACTION COMPARISON COMPLETE" << endl;
  cout << "========================================" << endl;
  
  return 0;
}

