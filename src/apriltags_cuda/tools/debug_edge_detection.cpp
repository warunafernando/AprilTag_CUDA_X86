#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>

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
  cout << "EDGE DETECTION DEBUG" << endl;
  cout << "========================================" << endl;
  cout << "Input image: " << image_path << endl;
  cout << "Output directory: " << output_dir << endl;
  cout << "Frame: " << frame.cols << "x" << frame.rows << endl;
  
  // Save original
  imwrite(output_dir + "/00_original.png", frame);
  cout << "\n[STAGE 0] Saved original frame" << endl;
  
  // Canny edge detection with different thresholds
  cout << "\n[STAGE 1] Canny edge detection with different thresholds..." << endl;
  
  vector<pair<int, int>> canny_thresholds = {
    {50, 150},
    {75, 200},
    {100, 200},
    {50, 100},
    {100, 250},
    {30, 100},
    {100, 300}
  };
  
  int stage_num = 1;
  for (const auto& thresh : canny_thresholds) {
    Mat edges;
    Canny(frame, edges, thresh.first, thresh.second);
    
    stringstream ss;
    ss << output_dir << "/" << setfill('0') << setw(2) << stage_num 
       << "_canny_" << thresh.first << "_" << thresh.second << ".png";
    imwrite(ss.str(), edges);
    
    int edge_pixels = countNonZero(edges);
    double edge_percentage = (edge_pixels * 100.0) / (edges.cols * edges.rows);
    
    cout << "  " << setw(2) << stage_num << ". Canny(" << thresh.first 
         << ", " << thresh.second << "): " << edge_pixels << " edge pixels (" 
         << fixed << setprecision(2) << edge_percentage << "%)" << endl;
    
    stage_num++;
  }
  
  // Sobel edge detection
  cout << "\n[STAGE 2] Sobel edge detection..." << endl;
  
  Mat sobel_x, sobel_y, sobel_combined;
  Sobel(frame, sobel_x, CV_16S, 1, 0, 3);
  Sobel(frame, sobel_y, CV_16S, 0, 1, 3);
  
  // Convert to absolute values and combine
  Mat abs_sobel_x, abs_sobel_y;
  convertScaleAbs(sobel_x, abs_sobel_x);
  convertScaleAbs(sobel_y, abs_sobel_y);
  addWeighted(abs_sobel_x, 0.5, abs_sobel_y, 0.5, 0, sobel_combined);
  
  imwrite(output_dir + "/" + to_string(stage_num++) + "_sobel_combined.png", sobel_combined);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_sobel_x.png", abs_sobel_x);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_sobel_y.png", abs_sobel_y);
  
  cout << "  Saved Sobel X, Y, and combined edges" << endl;
  
  // Laplacian edge detection
  cout << "\n[STAGE 3] Laplacian edge detection..." << endl;
  
  Mat laplacian, laplacian_abs;
  Laplacian(frame, laplacian, CV_16S, 3);
  convertScaleAbs(laplacian, laplacian_abs);
  
  imwrite(output_dir + "/" + to_string(stage_num++) + "_laplacian.png", laplacian_abs);
  cout << "  Saved Laplacian edges" << endl;
  
  // Adaptive threshold (creates binary image with edges)
  cout << "\n[STAGE 4] Adaptive threshold..." << endl;
  
  Mat adaptive;
  adaptiveThreshold(frame, adaptive, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_adaptive_threshold.png", adaptive);
  
  // Invert adaptive threshold
  Mat adaptive_inv;
  adaptiveThreshold(frame, adaptive_inv, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_adaptive_threshold_inv.png", adaptive_inv);
  
  cout << "  Saved adaptive threshold (normal and inverted)" << endl;
  
  // Edge detection on preprocessed images
  cout << "\n[STAGE 5] Edge detection on preprocessed images..." << endl;
  
  // CLAHE preprocessing
  Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
  Mat frame_clahe;
  clahe->apply(frame, frame_clahe);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_preprocessed_clahe.png", frame_clahe);
  
  Mat edges_clahe;
  Canny(frame_clahe, edges_clahe, 50, 150);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_edges_clahe_canny50_150.png", edges_clahe);
  
  int edge_pixels_clahe = countNonZero(edges_clahe);
  double edge_perc_clahe = (edge_pixels_clahe * 100.0) / (edges_clahe.cols * edges_clahe.rows);
  cout << "  CLAHE + Canny(50,150): " << edge_pixels_clahe << " edge pixels (" 
       << fixed << setprecision(2) << edge_perc_clahe << "%)" << endl;
  
  // Histogram equalization preprocessing
  Mat frame_eq;
  equalizeHist(frame, frame_eq);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_preprocessed_histeq.png", frame_eq);
  
  Mat edges_eq;
  Canny(frame_eq, edges_eq, 50, 150);
  imwrite(output_dir + "/" + to_string(stage_num++) + "_edges_histeq_canny50_150.png", edges_eq);
  
  int edge_pixels_eq = countNonZero(edges_eq);
  double edge_perc_eq = (edge_pixels_eq * 100.0) / (edges_eq.cols * edges_eq.rows);
  cout << "  HistEq + Canny(50,150): " << edge_pixels_eq << " edge pixels (" 
       << fixed << setprecision(2) << edge_perc_eq << "%)" << endl;
  
  // Overlay edges on original (for visualization)
  cout << "\n[STAGE 6] Creating edge overlays..." << endl;
  
  Mat color_frame;
  cvtColor(frame, color_frame, COLOR_GRAY2BGR);
  
  Mat edges_vis;
  Canny(frame, edges_vis, 50, 150);
  
  // Create colored edge overlay
  Mat overlay = color_frame.clone();
  overlay.setTo(Scalar(0, 255, 0), edges_vis);  // Green edges
  
  imwrite(output_dir + "/" + to_string(stage_num++) + "_overlay_edges_on_original.png", overlay);
  cout << "  Saved edge overlay on original frame" << endl;
  
  // Summary
  cout << "\n========================================" << endl;
  cout << "SUMMARY" << endl;
  cout << "========================================" << endl;
  cout << "All edge detection outputs saved to: " << output_dir << endl;
  cout << "Total files created: " << stage_num << endl;
  
  return 0;
}

