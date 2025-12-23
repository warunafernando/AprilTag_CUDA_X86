#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace cv;
using namespace std;

void detectAndWarpQuads(const Mat& frame, vector<Mat>& warped_quads, vector<vector<Point2f>>& quad_corners, const string& label) {
  Mat edges;
  Canny(frame, edges, 50, 150);
  
  vector<vector<Point>> contours;
  findContours(edges, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
  
  double tag_min_area = 500;
  double tag_max_area = 50000;
  int tagSize = 200;  // Larger size for better visualization
  
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
        
        // Warp to square
        vector<Point2f> dstQuad;
        dstQuad.push_back(Point2f(0, 0));
        dstQuad.push_back(Point2f(tagSize - 1, 0));
        dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
        dstQuad.push_back(Point2f(0, tagSize - 1));
        
        Mat H = getPerspectiveTransform(refined, dstQuad);
        Mat warped;
        warpPerspective(frame, warped, H, Size(tagSize, tagSize));
        
        warped_quads.push_back(warped);
        quad_corners.push_back(refined);
      }
    }
  }
  
  cout << label << ": Found " << warped_quads.size() << " quadrilaterals" << endl;
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
  cout << "SIDE-BY-SIDE QUAD COMPARISON" << endl;
  cout << "========================================" << endl;
  
  // Detect and warp quads from both images
  vector<Mat> warped_stable, warped_newvideo;
  vector<vector<Point2f>> corners_stable, corners_newvideo;
  
  detectAndWarpQuads(frame_stable, warped_stable, corners_stable, "Stable.avi");
  detectAndWarpQuads(frame_newvideo, warped_newvideo, corners_newvideo, "NewVideo.avi");
  
  // Find the best matching quads (based on position/area)
  int max_quads = min(warped_stable.size(), warped_newvideo.size());
  
  if (max_quads == 0) {
    cout << "No quads found in one or both images" << endl;
    return 1;
  }
  
  // Focus on the tag region quads (typically in center-right area)
  // For comparison, we'll show all quads side by side
  
  int tagSize = 200;
  int spacing = 20;
  int label_height = 30;
  
  // Create comparison images
  for (size_t i = 0; i < max_quads && i < 4; i++) {
    // Create side-by-side image
    int total_width = tagSize * 2 + spacing * 3;
    int total_height = tagSize + label_height * 2;
    
    Mat comparison(total_height, total_width, CV_8UC3, Scalar(128, 128, 128));
    
    // Convert warped images to color
    Mat stable_color, newvideo_color;
    cvtColor(warped_stable[i], stable_color, COLOR_GRAY2BGR);
    cvtColor(warped_newvideo[i], newvideo_color, COLOR_GRAY2BGR);
    
    // Place stable image
    Rect stable_roi(spacing, label_height, tagSize, tagSize);
    stable_color.copyTo(comparison(stable_roi));
    
    // Place newvideo image
    Rect newvideo_roi(tagSize + spacing * 2, label_height, tagSize, tagSize);
    newvideo_color.copyTo(comparison(newvideo_roi));
    
    // Add labels
    putText(comparison, "Stable.avi (Original/Distorted)", 
            Point(spacing, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
    putText(comparison, "NewVideo.avi (Undistorted)", 
            Point(tagSize + spacing * 2, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
    
    // Add statistics
    Scalar mean_s, stddev_s, mean_n, stddev_n;
    meanStdDev(warped_stable[i], mean_s, stddev_s);
    meanStdDev(warped_newvideo[i], mean_n, stddev_n);
    
    stringstream ss1, ss2;
    ss1 << "Mean: " << fixed << setprecision(1) << mean_s[0] << " Std: " << stddev_s[0];
    ss2 << "Mean: " << fixed << setprecision(1) << mean_n[0] << " Std: " << stddev_n[0];
    
    putText(comparison, ss1.str(), Point(spacing, tagSize + label_height + 15),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0), 1);
    putText(comparison, ss2.str(), Point(tagSize + spacing * 2, tagSize + label_height + 15),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0), 1);
    
    // Save
    stringstream ss;
    ss << output_dir << "/comparison_quad" << i << ".png";
    imwrite(ss.str(), comparison);
    cout << "  Saved: " << ss.str() << endl;
    
    // Also save individual warped images with border analysis
    Mat stable_vis = stable_color.clone();
    Mat newvideo_vis = newvideo_color.clone();
    
    // Draw border region (for Tag36h11, border is ~2 cells on each side)
    int border_pixels = tagSize / 9;  // Approximately 2 cells out of 9 total
    
    // Outer border
    rectangle(stable_vis, Point(0, 0), Point(tagSize-1, tagSize-1), Scalar(0, 255, 0), 2);
    rectangle(newvideo_vis, Point(0, 0), Point(tagSize-1, tagSize-1), Scalar(0, 255, 0), 2);
    
    // Inner border (data region boundary)
    rectangle(stable_vis, Point(border_pixels, border_pixels), 
              Point(tagSize-border_pixels-1, tagSize-border_pixels-1), Scalar(255, 0, 0), 1);
    rectangle(newvideo_vis, Point(border_pixels, border_pixels), 
              Point(tagSize-border_pixels-1, tagSize-border_pixels-1), Scalar(255, 0, 0), 1);
    
    // Analyze border pixels
    Rect border_roi(0, 0, tagSize, tagSize);
    Mat stable_border = warped_stable[i](border_roi);
    Mat newvideo_border = warped_newvideo[i](border_roi);
    
    // Count black pixels in border region
    Mat stable_border_mask = Mat::zeros(tagSize, tagSize, CV_8UC1);
    Mat newvideo_border_mask = Mat::zeros(tagSize, tagSize, CV_8UC1);
    
    rectangle(stable_border_mask, Point(0, 0), Point(tagSize-1, tagSize-1), Scalar(255), -1);
    rectangle(stable_border_mask, Point(border_pixels, border_pixels), 
              Point(tagSize-border_pixels-1, tagSize-border_pixels-1), Scalar(0), -1);
    
    rectangle(newvideo_border_mask, Point(0, 0), Point(tagSize-1, tagSize-1), Scalar(255), -1);
    rectangle(newvideo_border_mask, Point(border_pixels, border_pixels), 
              Point(tagSize-border_pixels-1, tagSize-border_pixels-1), Scalar(0), -1);
    
    Mat stable_border_only;
    warped_stable[i].copyTo(stable_border_only, stable_border_mask);
    Mat newvideo_border_only;
    warped_newvideo[i].copyTo(newvideo_border_only, newvideo_border_mask);
    
    // Count black pixels in border region
    Mat stable_border_mask_binary;
    threshold(stable_border_mask, stable_border_mask_binary, 127, 255, THRESH_BINARY);
    
    int stable_total = countNonZero(stable_border_mask_binary);
    if (stable_total == 0) stable_total = 1;  // Avoid division by zero
    
    int stable_black = 0;
    for (int y = 0; y < tagSize; y++) {
      for (int x = 0; x < tagSize; x++) {
        if (stable_border_mask.at<uchar>(y, x) > 0) {
          if (warped_stable[i].at<uchar>(y, x) < 128) {
            stable_black++;
          }
        }
      }
    }
    
    Mat newvideo_border_mask_binary;
    threshold(newvideo_border_mask, newvideo_border_mask_binary, 127, 255, THRESH_BINARY);
    
    int newvideo_total = countNonZero(newvideo_border_mask_binary);
    if (newvideo_total == 0) newvideo_total = 1;  // Avoid division by zero
    
    int newvideo_black = 0;
    for (int y = 0; y < tagSize; y++) {
      for (int x = 0; x < tagSize; x++) {
        if (newvideo_border_mask.at<uchar>(y, x) > 0) {
          if (warped_newvideo[i].at<uchar>(y, x) < 128) {
            newvideo_black++;
          }
        }
      }
    }
    
    double stable_ratio = (double)stable_black / stable_total;
    double newvideo_ratio = (double)newvideo_black / newvideo_total;
    
    // Add border ratio text
    stringstream ss3, ss4;
    ss3 << "Border: " << fixed << setprecision(1) << (stable_ratio * 100) << "% black";
    ss4 << "Border: " << fixed << setprecision(1) << (newvideo_ratio * 100) << "% black";
    
    putText(stable_vis, ss3.str(), Point(5, tagSize - 5), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    putText(newvideo_vis, ss4.str(), Point(5, tagSize - 5), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
    
    stringstream ss5, ss6;
    ss5 << output_dir << "/stable_quad" << i << "_with_border.png";
    ss6 << output_dir << "/newvideo_quad" << i << "_with_border.png";
    imwrite(ss5.str(), stable_vis);
    imwrite(ss6.str(), newvideo_vis);
    
    cout << "    Stable border ratio: " << fixed << setprecision(3) << stable_ratio << endl;
    cout << "    NewVideo border ratio: " << fixed << setprecision(3) << newvideo_ratio << endl;
  }
  
  // Create overall comparison with original frames
  Mat frame_stable_color, frame_newvideo_color;
  cvtColor(frame_stable, frame_stable_color, COLOR_GRAY2BGR);
  cvtColor(frame_newvideo, frame_newvideo_color, COLOR_GRAY2BGR);
  
  // Draw detected quads on original frames
  for (size_t i = 0; i < corners_stable.size() && i < 4; i++) {
    Scalar color = Scalar(0, 255, 0);
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(frame_stable_color, corners_stable[i][j], corners_stable[i][next], color, 2);
      circle(frame_stable_color, corners_stable[i][j], 5, Scalar(0, 0, 255), -1);
    }
    string label = "Q" + to_string(i);
    Point2f center = (corners_stable[i][0] + corners_stable[i][1] + 
                      corners_stable[i][2] + corners_stable[i][3]) / 4;
    putText(frame_stable_color, label, center, FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
  }
  
  for (size_t i = 0; i < corners_newvideo.size() && i < 4; i++) {
    Scalar color = Scalar(0, 255, 0);
    for (int j = 0; j < 4; j++) {
      int next = (j + 1) % 4;
      line(frame_newvideo_color, corners_newvideo[i][j], corners_newvideo[i][next], color, 2);
      circle(frame_newvideo_color, corners_newvideo[i][j], 5, Scalar(0, 0, 255), -1);
    }
    string label = "Q" + to_string(i);
    Point2f center = (corners_newvideo[i][0] + corners_newvideo[i][1] + 
                      corners_newvideo[i][2] + corners_newvideo[i][3]) / 4;
    putText(frame_newvideo_color, label, center, FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
  }
  
  // Resize for side-by-side (maintain aspect ratio)
  double scale = 800.0 / max(frame_stable.cols, frame_stable.rows);
  Size new_size((int)(frame_stable.cols * scale), (int)(frame_stable.rows * scale));
  
  Mat stable_resized, newvideo_resized;
  resize(frame_stable_color, stable_resized, new_size);
  resize(frame_newvideo_color, newvideo_resized, new_size);
  
  // Create side-by-side original frames
  int total_w = stable_resized.cols + newvideo_resized.cols + 20;
  int total_h = max(stable_resized.rows, newvideo_resized.rows) + 40;
  Mat original_comparison(total_h, total_w, CV_8UC3, Scalar(64, 64, 64));
  
  stable_resized.copyTo(original_comparison(Rect(10, 30, stable_resized.cols, stable_resized.rows)));
  newvideo_resized.copyTo(original_comparison(Rect(stable_resized.cols + 20, 30, 
                                                    newvideo_resized.cols, newvideo_resized.rows)));
  
  putText(original_comparison, "Stable.avi (Original/Distorted)", 
          Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  putText(original_comparison, "NewVideo.avi (Undistorted)", 
          Point(stable_resized.cols + 20, 20), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  
  string orig_path = output_dir + "/original_frames_comparison.png";
  imwrite(orig_path, original_comparison);
  cout << "\nSaved original frames comparison: " << orig_path << endl;
  
  cout << "\n========================================" << endl;
  cout << "COMPARISON COMPLETE" << endl;
  cout << "========================================" << endl;
  
  return 0;
}

