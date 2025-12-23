#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Usage: " << argv[0] << " <image_path> [output_dir]" << endl;
    cerr << "  Creates multiple brightness/contrast adjusted versions" << endl;
    return 1;
  }
  
  string image_path = argv[1];
  string output_dir = (argc >= 3) ? argv[2] : "output/brightness_adjust";
  
  string mkdir_cmd = "mkdir -p " + output_dir;
  system(mkdir_cmd.c_str());
  
  Mat original = imread(image_path, IMREAD_GRAYSCALE);
  if (original.empty()) {
    cerr << "Failed to load image: " << image_path << endl;
    return 1;
  }
  
  cout << "\n========================================" << endl;
  cout << "BRIGHTNESS/CONTRAST ADJUSTMENT" << endl;
  cout << "========================================" << endl;
  cout << "Original image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  
  Scalar mean_orig, stddev_orig;
  meanStdDev(original, mean_orig, stddev_orig);
  cout << "\nOriginal statistics:" << endl;
  cout << "  Mean: " << fixed << setprecision(2) << mean_orig[0] << endl;
  cout << "  Std:  " << fixed << setprecision(2) << stddev_orig[0] << endl;
  
  double min_val, max_val;
  minMaxLoc(original, &min_val, &max_val);
  cout << "  Range: " << (int)min_val << " - " << (int)max_val << endl;
  
  // Test different brightness and contrast adjustments
  vector<double> brightness_values = {-50, -30, -20, -10, 0, 10, 20, 30};
  vector<double> contrast_values = {0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5};
  
  // Method 1: Simple brightness/contrast adjustment
  // Formula: new_pixel = alpha * pixel + beta
  // alpha = contrast, beta = brightness
  
  cout << "\nCreating brightness/contrast adjusted versions..." << endl;
  
  for (double brightness : brightness_values) {
    for (double contrast : contrast_values) {
      Mat adjusted;
      original.convertTo(adjusted, -1, contrast, brightness);
      
      Scalar mean_adj, stddev_adj;
      meanStdDev(adjusted, mean_adj, stddev_adj);
      
      stringstream ss;
      ss << output_dir << "/bright" << (int)brightness << "_contrast" << fixed << setprecision(1) << contrast << ".png";
      
      imwrite(ss.str(), adjusted);
      
      cout << "  Saved: bright=" << setw(4) << (int)brightness 
           << ", contrast=" << fixed << setprecision(1) << contrast
           << " -> Mean: " << setw(6) << fixed << setprecision(1) << mean_adj[0] << endl;
    }
  }
  
  // Method 2: Gamma correction (to darken/lighter overall)
  cout << "\nCreating gamma-corrected versions..." << endl;
  vector<double> gamma_values = {0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.5, 2.0};
  
  Mat lookup_table(1, 256, CV_8U);
  uchar* p = lookup_table.ptr();
  
  for (double gamma : gamma_values) {
    for (int i = 0; i < 256; i++) {
      p[i] = saturate_cast<uchar>(pow(i / 255.0, 1.0 / gamma) * 255.0);
    }
    
    Mat gamma_corrected;
    LUT(original, lookup_table, gamma_corrected);
    
    Scalar mean_gamma, stddev_gamma;
    meanStdDev(gamma_corrected, mean_gamma, stddev_gamma);
    
    stringstream ss;
    ss << output_dir << "/gamma" << fixed << setprecision(1) << gamma << ".png";
    imwrite(ss.str(), gamma_corrected);
    
    cout << "  Saved: gamma=" << fixed << setprecision(1) << gamma
         << " -> Mean: " << setw(6) << fixed << setprecision(1) << mean_gamma[0] << endl;
  }
  
  // Method 3: Histogram equalization
  cout << "\nCreating histogram equalized versions..." << endl;
  Mat hist_eq;
  equalizeHist(original, hist_eq);
  
  Scalar mean_he, stddev_he;
  meanStdDev(hist_eq, mean_he, stddev_he);
  
  stringstream ss_he;
  ss_he << output_dir << "/histogram_equalized.png";
  imwrite(ss_he.str(), hist_eq);
  cout << "  Saved: histogram_equalized.png -> Mean: " << fixed << setprecision(1) << mean_he[0] << endl;
  
  // Method 4: CLAHE (Contrast Limited Adaptive Histogram Equalization)
  cout << "\nCreating CLAHE versions..." << endl;
  vector<double> clip_limits = {2.0, 3.0, 4.0, 5.0};
  vector<int> tile_sizes = {4, 8, 16};
  
  for (double clip_limit : clip_limits) {
    for (int tile_size : tile_sizes) {
      Ptr<CLAHE> clahe = createCLAHE(clip_limit, Size(tile_size, tile_size));
      Mat clahe_img;
      clahe->apply(original, clahe_img);
      
      Scalar mean_clahe, stddev_clahe;
      meanStdDev(clahe_img, mean_clahe, stddev_clahe);
      
      stringstream ss;
      ss << output_dir << "/clahe_clip" << fixed << setprecision(1) << clip_limit 
         << "_tile" << tile_size << ".png";
      imwrite(ss.str(), clahe_img);
      
      cout << "  Saved: clip=" << fixed << setprecision(1) << clip_limit
           << ", tile=" << tile_size
           << " -> Mean: " << setw(6) << fixed << setprecision(1) << mean_clahe[0] << endl;
    }
  }
  
  // Method 5: Simple threshold-based darkening (make darks darker)
  cout << "\nCreating threshold-darkened versions..." << endl;
  vector<int> threshold_values = {100, 120, 140, 160, 180};
  
  for (int thresh : threshold_values) {
    Mat darkened = original.clone();
    
    // Darken pixels below threshold
    for (int y = 0; y < darkened.rows; y++) {
      for (int x = 0; x < darkened.cols; x++) {
        uchar& pixel = darkened.at<uchar>(y, x);
        if (pixel < thresh) {
          pixel = saturate_cast<uchar>(pixel * 0.5);  // Make darker pixels even darker
        }
      }
    }
    
    Scalar mean_dark, stddev_dark;
    meanStdDev(darkened, mean_dark, stddev_dark);
    
    stringstream ss;
    ss << output_dir << "/darkened_thresh" << thresh << ".png";
    imwrite(ss.str(), darkened);
    
    cout << "  Saved: threshold=" << thresh
         << " -> Mean: " << setw(6) << fixed << setprecision(1) << mean_dark[0] << endl;
  }
  
  // Create a summary comparison image
  cout << "\nCreating summary comparison image..." << endl;
  
  // Select a few best candidates for visual comparison
  vector<string> candidates = {
    "bright-30_contrast1.2.png",
    "bright-20_contrast1.1.png",
    "gamma0.7.png",
    "gamma0.8.png",
    "histogram_equalized.png",
    "clahe_clip3.0_tile8.png"
  };
  
  int img_width = original.cols;
  int img_height = original.rows;
  int cols_per_row = 3;
  int rows = (candidates.size() + 2) / cols_per_row + 1;  // +1 for original
  
  Mat summary(rows * img_height, cols_per_row * img_width, CV_8UC1, Scalar(128));
  
  // Place original
  original.copyTo(summary(Rect(0, 0, img_width, img_height)));
  putText(summary, "Original", Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(255), 2);
  
  int idx = 1;
  for (const string& cand : candidates) {
    string path = output_dir + "/" + cand;
    Mat img = imread(path, IMREAD_GRAYSCALE);
    if (!img.empty()) {
      int row = idx / cols_per_row;
      int col = idx % cols_per_row;
      img.copyTo(summary(Rect(col * img_width, row * img_height, img_width, img_height)));
      
      string label = cand.substr(0, cand.find_last_of('.'));
      putText(summary, label, Point(col * img_width + 10, row * img_height + 30),
              FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255), 1);
      idx++;
    }
  }
  
  string summary_path = output_dir + "/summary_comparison.png";
  imwrite(summary_path, summary);
  cout << "  Saved summary: " << summary_path << endl;
  
  cout << "\n========================================" << endl;
  cout << "BRIGHTNESS ADJUSTMENT COMPLETE" << endl;
  cout << "========================================" << endl;
  cout << "\nAll adjusted images saved to: " << output_dir << endl;
  cout << "Check summary_comparison.png for visual comparison." << endl;
  cout << "\nRecommended: Test detection on darker versions (negative brightness, gamma < 1.0)" << endl;
  
  return 0;
}



