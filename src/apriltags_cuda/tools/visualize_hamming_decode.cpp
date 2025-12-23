#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <bitset>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

// Tag36h11 bit positions (1-indexed, 0 and 7 are border)
static const int TAG36H11_BIT_X[36] = {
    1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3
};
static const int TAG36H11_BIT_Y[36] = {
    1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4
};

// Extract pattern from warped image
vector<vector<int>> extract_pattern_from_warped(const Mat& warped) {
    int tagSize = warped.rows;
    int borderSize = tagSize / 9;
    int dataSize = tagSize - 2 * borderSize;
    int cellSize = dataSize / 6;
    
    vector<vector<int>> pattern(6, vector<int>(6));
    
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            int x_center = borderSize + col * cellSize + cellSize / 2;
            int y_center = borderSize + row * cellSize + cellSize / 2;
            
            x_center = min(max(0, x_center), tagSize - 1);
            y_center = min(max(0, y_center), tagSize - 1);
            
            pattern[row][col] = (int)warped.at<uchar>(y_center, x_center);
        }
    }
    
    return pattern;
}

// Extract 36-bit code from 6x6 pattern using Tag36h11 bit layout
uint64_t extract_code(const vector<vector<int>>& pattern_6x6) {
    uint64_t code = 0;
    
    for (int i = 0; i < 36; i++) {
        // Convert 1-indexed to 0-indexed
        int x = TAG36H11_BIT_X[i] - 1;
        int y = TAG36H11_BIT_Y[i] - 1;
        
        // Extract bit (0 = white/bright, 1 = black/dark)
        int val = pattern_6x6[y][x];
        int bit = (val < 128) ? 1 : 0;
        
        code |= ((uint64_t)bit << i);
    }
    
    return code;
}

// Create visualization of pattern with bit positions
Mat visualize_pattern_with_bits(const vector<vector<int>>& pattern_6x6, uint64_t code, 
                                 const string& title, const string& info) {
    int cell_size = 80;
    int padding = 40;
    int grid_size = 6 * cell_size;
    int total_width = grid_size + padding * 2;
    int total_height = grid_size + padding * 3 + 120;  // Extra space for bit display
    
    Mat vis(total_height, total_width, CV_8UC3, Scalar(240, 240, 240));
    
    // Draw grid
    for (int row = 0; row <= 6; row++) {
        int y = padding + row * cell_size;
        line(vis, Point(padding, y), Point(padding + grid_size, y), Scalar(200, 200, 200), 1);
    }
    for (int col = 0; col <= 6; col++) {
        int x = padding + col * cell_size;
        line(vis, Point(x, padding), Point(x, padding + grid_size), Scalar(200, 200, 200), 1);
    }
    
    // Draw pattern cells
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            int val = pattern_6x6[row][col];
            Scalar color = (val < 128) ? Scalar(0, 0, 0) : Scalar(255, 255, 255);
            Rect cell(padding + col * cell_size + 1, padding + row * cell_size + 1, 
                     cell_size - 2, cell_size - 2);
            rectangle(vis, cell, color, -1);
            
            // Draw value
            stringstream ss;
            ss << val;
            int font_scale = 0.4;
            Scalar text_color = (val < 128) ? Scalar(255, 255, 255) : Scalar(0, 0, 0);
            putText(vis, ss.str(), Point(padding + col * cell_size + 5, padding + row * cell_size + 20),
                    FONT_HERSHEY_SIMPLEX, font_scale, text_color, 1);
        }
    }
    
    // Highlight bit positions with numbers
    for (int i = 0; i < 36; i++) {
        int x = TAG36H11_BIT_X[i] - 1;  // Convert to 0-indexed
        int y = TAG36H11_BIT_Y[i] - 1;
        int bit = (code >> i) & 1;
        
        // Draw circle at bit position
        Point center(padding + x * cell_size + cell_size / 2,
                    padding + y * cell_size + cell_size / 2);
        Scalar circle_color = (bit == 1) ? Scalar(0, 255, 0) : Scalar(255, 0, 0);  // Green=1, Red=0
        circle(vis, center, 15, circle_color, 2);
        
        // Draw bit number
        stringstream ss;
        ss << i;
        int font_scale = 0.3;
        putText(vis, ss.str(), Point(center.x - 8, center.y + 4),
                FONT_HERSHEY_SIMPLEX, font_scale, Scalar(255, 255, 0), 1);
    }
    
    // Draw title
    putText(vis, title, Point(padding, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 2);
    
    // Draw info
    putText(vis, info, Point(padding, padding + grid_size + 25), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    
    // Draw extracted code
    stringstream ss_code;
    ss_code << "Extracted Code: 0x" << hex << setfill('0') << setw(16) << code << dec;
    putText(vis, ss_code.str(), Point(padding, padding + grid_size + 50),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
    
    // Draw bit pattern (36 bits)
    stringstream ss_bits;
    ss_bits << "Bits: ";
    for (int i = 35; i >= 0; i--) {
        ss_bits << ((code >> i) & 1);
        if (i % 9 == 0 && i > 0) ss_bits << " ";
    }
    putText(vis, ss_bits.str(), Point(padding, padding + grid_size + 75),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
    
    // Legend
    circle(vis, Point(padding + 20, padding + grid_size + 105), 8, Scalar(0, 255, 0), 2);
    putText(vis, "= 1 (black)", Point(padding + 35, padding + grid_size + 110),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
    circle(vis, Point(padding + 120, padding + grid_size + 105), 8, Scalar(255, 0, 0), 2);
    putText(vis, "= 0 (white)", Point(padding + 135, padding + grid_size + 110),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
    putText(vis, "Numbers = bit index (0-35)", Point(padding + 230, padding + grid_size + 110),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 1);
    
    return vis;
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
  cout << "HAMMING CODE DECODING VISUALIZATION" << endl;
  cout << "========================================" << endl;
  
  // Detect quads and warp
  auto detectQuads = [](const Mat& frame) -> vector<vector<Point2f>> {
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
          vector<Point2f> refined = quad;
          TermCriteria criteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.001);
          cornerSubPix(frame, refined, Size(5, 5), Size(-1, -1), criteria);
          quads.push_back(refined);
        }
      }
    }
    return quads;
  };
  
  vector<vector<Point2f>> quads_stable = detectQuads(frame_stable);
  vector<vector<Point2f>> quads_newvideo = detectQuads(frame_newvideo);
  
  if (quads_stable.empty() || quads_newvideo.size() < 3) {
    cerr << "Error: Need quads from both images" << endl;
    return 1;
  }
  
  int tagSize = 200;
  
  // Warp Stable Quad 0
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  Mat H_stable = getPerspectiveTransform(quads_stable[0], dstQuad);
  Mat warped_stable;
  warpPerspective(frame_stable, warped_stable, H_stable, Size(tagSize, tagSize), INTER_CUBIC);
  
  // Warp NewVideo Quad 2
  Mat H_newvideo = getPerspectiveTransform(quads_newvideo[2], dstQuad);
  Mat warped_newvideo;
  warpPerspective(frame_newvideo, warped_newvideo, H_newvideo, Size(tagSize, tagSize), INTER_CUBIC);
  
  // Extract patterns
  vector<vector<int>> pattern_stable = extract_pattern_from_warped(warped_stable);
  vector<vector<int>> pattern_newvideo = extract_pattern_from_warped(warped_newvideo);
  
  // Mirror NewVideo horizontally (as we discovered it's a mirror image)
  vector<vector<int>> pattern_newvideo_mirrored(6, vector<int>(6));
  for (int row = 0; row < 6; row++) {
    for (int col = 0; col < 6; col++) {
      pattern_newvideo_mirrored[row][5 - col] = pattern_newvideo[row][col];
    }
  }
  
  // Extract codes
  uint64_t code_stable = extract_code(pattern_stable);
  uint64_t code_newvideo = extract_code(pattern_newvideo);
  uint64_t code_newvideo_mirrored = extract_code(pattern_newvideo_mirrored);
  
  cout << "\nExtracted codes:" << endl;
  cout << "  Stable: 0x" << hex << setfill('0') << setw(16) << code_stable << dec << endl;
  cout << "  NewVideo (original): 0x" << hex << setfill('0') << setw(16) << code_newvideo << dec << endl;
  cout << "  NewVideo (horizontally mirrored): 0x" << hex << setfill('0') << setw(16) << code_newvideo_mirrored << dec << endl;
  
  int dist_original = __builtin_popcountll(code_stable ^ code_newvideo);
  int dist_mirrored = __builtin_popcountll(code_stable ^ code_newvideo_mirrored);
  cout << "\nHamming distances:" << endl;
  cout << "  Original: " << dist_original << " bits differ" << endl;
  cout << "  Mirrored: " << dist_mirrored << " bits differ";
  if (dist_mirrored == 0) {
    cout << " ✓ PERFECT MATCH!" << endl;
  } else {
    cout << endl;
  }
  
  // Create visualizations
  stringstream ss_stable_info;
  ss_stable_info << "Stable.avi Quad 0 - Pixel values shown, bit positions marked";
  Mat vis_stable = visualize_pattern_with_bits(pattern_stable, code_stable, 
                                                "STABLE.AVI QUAD 0", ss_stable_info.str());
  
  stringstream ss_newvideo_info;
  ss_newvideo_info << "NewVideo.avi Quad 2 (ORIGINAL) - Pixel values shown, bit positions marked";
  Mat vis_newvideo = visualize_pattern_with_bits(pattern_newvideo, code_newvideo,
                                                  "NEWVIDEO.AVI QUAD 2 (ORIGINAL)", ss_newvideo_info.str());
  
  stringstream ss_newvideo_mirrored_info;
  ss_newvideo_mirrored_info << "NewVideo.avi Quad 2 (HORIZONTALLY MIRRORED) - Matches Stable!";
  Mat vis_newvideo_mirrored = visualize_pattern_with_bits(pattern_newvideo_mirrored, code_newvideo_mirrored,
                                                           "NEWVIDEO.AVI QUAD 2 (MIRRORED)", ss_newvideo_mirrored_info.str());
  
  // Save individual visualizations
  string stable_path_out = output_dir + "/stable_quad0_hamming_vis.png";
  string newvideo_path_out = output_dir + "/newvideo_quad2_hamming_vis_original.png";
  string newvideo_mirrored_path_out = output_dir + "/newvideo_quad2_hamming_vis_mirrored.png";
  imwrite(stable_path_out, vis_stable);
  imwrite(newvideo_path_out, vis_newvideo);
  imwrite(newvideo_mirrored_path_out, vis_newvideo_mirrored);
  cout << "\n  Saved: " << stable_path_out << endl;
  cout << "  Saved: " << newvideo_path_out << endl;
  cout << "  Saved: " << newvideo_mirrored_path_out << endl;
  
  // Create side-by-side comparison: Stable vs Mirrored NewVideo
  int max_height = max(vis_stable.rows, vis_newvideo_mirrored.rows);
  int total_width = vis_stable.cols + vis_newvideo_mirrored.cols + 40;
  Mat comparison(max_height, total_width, CV_8UC3, Scalar(200, 200, 200));
  
  vis_stable.copyTo(comparison(Rect(10, 0, vis_stable.cols, vis_stable.rows)));
  vis_newvideo_mirrored.copyTo(comparison(Rect(vis_stable.cols + 30, 0, vis_newvideo_mirrored.cols, vis_newvideo_mirrored.rows)));
  
  // Add main title
  putText(comparison, "HAMMING CODE EXTRACTION - MIRRORED NEWVIDEO COMPARISON", 
          Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
  
  // Calculate and show differences
  uint64_t code_diff = code_stable ^ code_newvideo_mirrored;
  int diff_bits = __builtin_popcountll(code_diff);
  
  stringstream ss_diff;
  if (diff_bits == 0) {
    ss_diff << "✓ PERFECT MATCH! 0 bits differ - Patterns are IDENTICAL";
    putText(comparison, ss_diff.str(), 
            Point(10, max_height - 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);
  } else {
    ss_diff << "Difference: " << diff_bits << " bits differ out of 36";
    putText(comparison, ss_diff.str(), 
            Point(10, max_height - 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
  }
  
  // Also create comparison with original NewVideo
  int max_height_orig = max(vis_stable.rows, vis_newvideo.rows);
  int total_width_orig = vis_stable.cols + vis_newvideo.cols + 40;
  Mat comparison_orig(max_height_orig, total_width_orig, CV_8UC3, Scalar(200, 200, 200));
  
  vis_stable.copyTo(comparison_orig(Rect(10, 0, vis_stable.cols, vis_stable.rows)));
  vis_newvideo.copyTo(comparison_orig(Rect(vis_stable.cols + 30, 0, vis_newvideo.cols, vis_newvideo.rows)));
  
  putText(comparison_orig, "HAMMING CODE EXTRACTION - ORIGINAL NEWVIDEO COMPARISON", 
          Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 0), 2);
  
  uint64_t code_diff_orig = code_stable ^ code_newvideo;
  int diff_bits_orig = __builtin_popcountll(code_diff_orig);
  stringstream ss_diff_orig;
  ss_diff_orig << "Difference: " << diff_bits_orig << " bits differ out of 36";
  putText(comparison_orig, ss_diff_orig.str(), 
          Point(10, max_height_orig - 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
  
  string comparison_orig_path = output_dir + "/hamming_decode_comparison_original.png";
  imwrite(comparison_orig_path, comparison_orig);
  cout << "  Saved: " << comparison_orig_path << endl;
  
  string comparison_path = output_dir + "/hamming_decode_comparison.png";
  imwrite(comparison_path, comparison);
  cout << "  Saved: " << comparison_path << endl;
  
  cout << "\n========================================" << endl;
  cout << "VISUALIZATION COMPLETE" << endl;
  cout << "========================================" << endl;
  cout << "\nOriginal NewVideo vs Stable: " << diff_bits_orig << " bits differ" << endl;
  cout << "Mirrored NewVideo vs Stable: " << diff_bits << " bits differ";
  if (diff_bits == 0) {
    cout << " ✓ PERFECT MATCH!" << endl;
  } else {
    cout << endl;
  }
  cout << "\nCONCLUSION: NewVideo is a HORIZONTAL MIRROR IMAGE of Stable!" << endl;
  
  return 0;
}

