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

// Mirror pattern horizontally
vector<vector<int>> mirror_horizontal(const vector<vector<int>>& pattern) {
    vector<vector<int>> mirrored(6, vector<int>(6));
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            mirrored[row][5 - col] = pattern[row][col];
        }
    }
    return mirrored;
}

// Mirror pattern vertically
vector<vector<int>> mirror_vertical(const vector<vector<int>>& pattern) {
    vector<vector<int>> mirrored(6, vector<int>(6));
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            mirrored[5 - row][col] = pattern[row][col];
        }
    }
    return mirrored;
}

// Rotate pattern 90 degrees clockwise
vector<vector<int>> rotate_90(const vector<vector<int>>& pattern) {
    vector<vector<int>> rotated(6, vector<int>(6));
    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            rotated[col][5 - row] = pattern[row][col];
        }
    }
    return rotated;
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

// Calculate Hamming distance
int hamming_distance(uint64_t a, uint64_t b) {
    return __builtin_popcountll(a ^ b);
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
  cout << "HAMMING CODE DECODING WITH MIRROR TEST" << endl;
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
  
  // Extract base codes
  uint64_t code_stable = extract_code(pattern_stable);
  
  cout << "\nStable.avi Quad 0:" << endl;
  cout << "  Code: 0x" << hex << setfill('0') << setw(16) << code_stable << dec << endl;
  
  cout << "\nNewVideo.avi Quad 2 - Testing different transformations:" << endl;
  
  // Test original
  uint64_t code_newvideo_orig = extract_code(pattern_newvideo);
  int dist_orig = hamming_distance(code_stable, code_newvideo_orig);
  cout << "  Original: 0x" << hex << setfill('0') << setw(16) << code_newvideo_orig << dec 
       << " (distance: " << dist_orig << ")" << endl;
  
  // Test horizontal mirror
  vector<vector<int>> pattern_newvideo_h = mirror_horizontal(pattern_newvideo);
  uint64_t code_newvideo_h = extract_code(pattern_newvideo_h);
  int dist_h = hamming_distance(code_stable, code_newvideo_h);
  cout << "  Horizontal mirror: 0x" << hex << setfill('0') << setw(16) << code_newvideo_h << dec 
       << " (distance: " << dist_h << ")" << endl;
  
  // Test vertical mirror
  vector<vector<int>> pattern_newvideo_v = mirror_vertical(pattern_newvideo);
  uint64_t code_newvideo_v = extract_code(pattern_newvideo_v);
  int dist_v = hamming_distance(code_stable, code_newvideo_v);
  cout << "  Vertical mirror: 0x" << hex << setfill('0') << setw(16) << code_newvideo_v << dec 
       << " (distance: " << dist_v << ")" << endl;
  
  // Test both mirrors (180° rotation)
  vector<vector<int>> pattern_newvideo_both = mirror_vertical(mirror_horizontal(pattern_newvideo));
  uint64_t code_newvideo_both = extract_code(pattern_newvideo_both);
  int dist_both = hamming_distance(code_stable, code_newvideo_both);
  cout << "  Both mirrors (180°): 0x" << hex << setfill('0') << setw(16) << code_newvideo_both << dec 
       << " (distance: " << dist_both << ")" << endl;
  
  // Test rotations of mirrored versions
  vector<vector<int>> pattern_newvideo_h_rot90 = rotate_90(pattern_newvideo_h);
  uint64_t code_newvideo_h_rot90 = extract_code(pattern_newvideo_h_rot90);
  int dist_h_rot90 = hamming_distance(code_stable, code_newvideo_h_rot90);
  cout << "  H-mirror + 90°: 0x" << hex << setfill('0') << setw(16) << code_newvideo_h_rot90 << dec 
       << " (distance: " << dist_h_rot90 << ")" << endl;
  
  vector<vector<int>> pattern_newvideo_h_rot180 = rotate_90(pattern_newvideo_h_rot90);
  uint64_t code_newvideo_h_rot180 = extract_code(pattern_newvideo_h_rot180);
  int dist_h_rot180 = hamming_distance(code_stable, code_newvideo_h_rot180);
  cout << "  H-mirror + 180°: 0x" << hex << setfill('0') << setw(16) << code_newvideo_h_rot180 << dec 
       << " (distance: " << dist_h_rot180 << ")" << endl;
  
  vector<vector<int>> pattern_newvideo_h_rot270 = rotate_90(pattern_newvideo_h_rot180);
  uint64_t code_newvideo_h_rot270 = extract_code(pattern_newvideo_h_rot270);
  int dist_h_rot270 = hamming_distance(code_stable, code_newvideo_h_rot270);
  cout << "  H-mirror + 270°: 0x" << hex << setfill('0') << setw(16) << code_newvideo_h_rot270 << dec 
       << " (distance: " << dist_h_rot270 << ")" << endl;
  
  // Find best match
  vector<pair<string, int>> distances = {
    {"Original", dist_orig},
    {"Horizontal mirror", dist_h},
    {"Vertical mirror", dist_v},
    {"Both mirrors", dist_both},
    {"H-mirror + 90°", dist_h_rot90},
    {"H-mirror + 180°", dist_h_rot180},
    {"H-mirror + 270°", dist_h_rot270}
  };
  
  auto best = min_element(distances.begin(), distances.end(), 
                         [](const pair<string, int>& a, const pair<string, int>& b) {
                             return a.second < b.second;
                         });
  
  cout << "\n========================================" << endl;
  cout << "BEST MATCH: " << best->first << " (distance: " << best->second << ")" << endl;
  cout << "========================================" << endl;
  
  if (best->second == 0) {
    cout << "✓ PERFECT MATCH! Patterns are identical after transformation." << endl;
  } else if (best->second <= 5) {
    cout << "✓ Good match within error correction range." << endl;
  } else {
    cout << "? Patterns differ significantly even after transformation." << endl;
  }
  
  return 0;
}



