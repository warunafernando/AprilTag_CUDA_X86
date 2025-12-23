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
// From tag36h11.c: bit_x and bit_y arrays define where to extract bits
static const int TAG36H11_BIT_X[36] = {
    1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3
};
static const int TAG36H11_BIT_Y[36] = {
    1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4
};

// Rotate bit coordinates 90 degrees clockwise
void rotate_bit_coords_90(int& x, int& y) {
    // Rotate around center (3.5, 3.5) in 1-indexed coordinates
    int new_x = 7 - y;
    int new_y = x;
    x = new_x;
    y = new_y;
}

// Extract 36-bit code from 6x6 pattern using Tag36h11 bit layout
uint64_t extract_code(const vector<vector<int>>& pattern_6x6, const int* bit_x, const int* bit_y) {
    uint64_t code = 0;
    
    for (int i = 0; i < 36; i++) {
        // Convert 1-indexed to 0-indexed
        int x = bit_x[i] - 1;
        int y = bit_y[i] - 1;
        
        // Extract bit (0 = white/bright, 1 = black/dark)
        int val = pattern_6x6[y][x];
        int bit = (val < 128) ? 1 : 0;
        
        code |= ((uint64_t)bit << i);
    }
    
    return code;
}

// Count set bits (Hamming weight)
int hamming_weight(uint64_t x) {
    return __builtin_popcountll(x);
}

// Hamming distance between two codes
int hamming_distance(uint64_t a, uint64_t b) {
    return hamming_weight(a ^ b);
}

// Tag36h11 valid codewords (first 10 for testing - full list has 587)
// In real implementation, we'd load all 587 from tag36h11.c
static const uint64_t TAG36H11_CODES[] = {
    0x0000000d7e00984bUL,
    0x0000000dda664ca7UL,
    0x0000000dc4a1c821UL,
    0x0000000e17b470e9UL,
    0x0000000ef91d01b1UL,
    0x0000000f429cdd73UL,
    0x000000005da29225UL,
    0x00000001106cba43UL,
    0x0000000223bed79dUL,
    0x000000021f51213cUL
};
static const int NUM_TEST_CODES = 10;
static const int TOTAL_CODES = 587;

// Find best matching codeword
pair<int, int> find_best_match(uint64_t extracted_code, const uint64_t* codes, int num_codes) {
    int best_id = -1;
    int min_distance = 1000;
    
    for (int i = 0; i < num_codes; i++) {
        int dist = hamming_distance(extracted_code, codes[i]);
        if (dist < min_distance) {
            min_distance = dist;
            best_id = i;
        }
    }
    
    return make_pair(best_id, min_distance);
}

// Rotate 6x6 pattern 90 degrees clockwise
vector<vector<int>> rotate_pattern_90(const vector<vector<int>>& pattern) {
    vector<vector<int>> rotated(6, vector<int>(6));
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            rotated[j][5-i] = pattern[i][j];
        }
    }
    return rotated;
}

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

// Decode pattern at all rotations
void decode_pattern_all_rotations(const vector<vector<int>>& pattern_6x6, 
                                   const string& label, int quad_id) {
    cout << "\n" << label << " Quad " << quad_id << " - Hamming Code Decoding:" << endl;
    cout << "  (Note: Using first " << NUM_TEST_CODES << " codes for testing)" << endl;
    
    // Try all 4 rotations
    vector<vector<int>> current_pattern = pattern_6x6;
    
    for (int rotation = 0; rotation < 4; rotation++) {
        // Extract bit coordinates for this rotation
        int bit_x[36], bit_y[36];
        for (int i = 0; i < 36; i++) {
            bit_x[i] = TAG36H11_BIT_X[i];
            bit_y[i] = TAG36H11_BIT_Y[i];
            
            // Rotate coordinates
            for (int r = 0; r < rotation; r++) {
                rotate_bit_coords_90(bit_x[i], bit_y[i]);
            }
        }
        
        // Extract code
        uint64_t extracted_code = extract_code(current_pattern, bit_x, bit_y);
        
        // Find best match
        auto match = find_best_match(extracted_code, TAG36H11_CODES, NUM_TEST_CODES);
        int best_id = match.first;
        int distance = match.second;
        
        cout << "\n  Rotation " << (rotation * 90) << "°:" << endl;
        cout << "    Extracted code: 0x" << hex << setfill('0') << setw(16) << extracted_code << dec << endl;
        cout << "    Best match: Code ID " << best_id 
             << " (distance: " << distance << ")" << endl;
        
        // Tag36h11 has Hamming distance 11, can correct up to 5 errors
        if (distance <= 5) {
            cout << "    ✓ DECODABLE (within error correction range)" << endl;
        } else if (distance <= 11) {
            cout << "    ? Possibly decodable (at Hamming distance boundary)" << endl;
        } else {
            cout << "    ✗ Not decodable (too many errors)" << endl;
        }
        
        // Show bit pattern
        cout << "    Bit pattern (36 bits): ";
        for (int i = 35; i >= 0; i--) {
            cout << ((extracted_code >> i) & 1);
            if (i % 9 == 0 && i > 0) cout << " ";
        }
        cout << endl;
        
        // Rotate pattern for next iteration
        current_pattern = rotate_pattern_90(current_pattern);
    }
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
  cout << "HAMMING CODE DECODING" << endl;
  cout << "========================================" << endl;
  cout << "Tag36h11: 36 data bits, Hamming distance 11" << endl;
  cout << "Can correct up to 5 bit errors" << endl;
  
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
  
  cout << "\n========================================" << endl;
  decode_pattern_all_rotations(pattern_stable, "Stable.avi", 0);
  
  cout << "\n========================================" << endl;
  decode_pattern_all_rotations(pattern_newvideo, "NewVideo.avi", 2);
  
  cout << "\n========================================" << endl;
  cout << "DECODING COMPLETE" << endl;
  cout << "========================================" << endl;
  cout << "\nNote: This uses a simplified decoder with " << NUM_TEST_CODES 
       << " test codes." << endl;
  cout << "Full Tag36h11 has " << TOTAL_CODES << " valid codewords." << endl;
  cout << "For complete decoding, load all codewords from tag36h11.c" << endl;
  
  return 0;
}
