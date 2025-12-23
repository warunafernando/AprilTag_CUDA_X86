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

void visualizeHomography(const Mat& frame, const vector<Point2f>& quad, int quad_id, 
                         const string& label, Mat& output_vis, const string& output_path) {
  int tagSize = 200;
  
  // Create destination quad for homography
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  // Calculate homography
  Mat H = getPerspectiveTransform(quad, dstQuad);
  
  // Apply homography (warp perspective)
  Mat warped;
  warpPerspective(frame, warped, H, Size(tagSize, tagSize), INTER_CUBIC);
  
  // Draw quad on original frame
  Mat frame_color;
  cvtColor(frame, frame_color, COLOR_GRAY2BGR);
  
  for (int i = 0; i < 4; i++) {
    int next = (i + 1) % 4;
    line(frame_color, quad[i], quad[next], Scalar(0, 255, 0), 2);
    circle(frame_color, quad[i], 5, Scalar(0, 0, 255), -1);
    
    // Label corners
    string corner_label = to_string(i);
    putText(frame_color, corner_label, Point(quad[i].x + 10, quad[i].y), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
  }
  
  // Add quad ID label
  Point2f center = (quad[0] + quad[1] + quad[2] + quad[3]) / 4;
  string quad_label = label + " Quad " + to_string(quad_id);
  putText(frame_color, quad_label, Point(center.x - 50, center.y), 
          FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  
  // Create visualization with original and warped side by side
  int spacing = 20;
  int label_height = 40;
  int total_width = frame_color.cols + tagSize + spacing * 3;
  int total_height = max(frame_color.rows, tagSize) + label_height * 2;
  
  Mat vis(total_height, total_width, CV_8UC3, Scalar(64, 64, 64));
  
  // Place original frame
  frame_color.copyTo(vis(Rect(spacing, label_height, frame_color.cols, frame_color.rows)));
  
  // Place warped image (convert to color)
  Mat warped_color;
  cvtColor(warped, warped_color, COLOR_GRAY2BGR);
  warped_color.copyTo(vis(Rect(frame_color.cols + spacing * 2, label_height, tagSize, tagSize)));
  
  // Add labels
  putText(vis, label + " - Original Frame (Quad " + to_string(quad_id) + ")", 
          Point(spacing, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  putText(vis, "Warped Result (" + to_string(tagSize) + "x" + to_string(tagSize) + ")", 
          Point(frame_color.cols + spacing * 2, 25), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 255, 255), 2);
  
  // Draw corner labels on warped image
  vector<Point2f> warped_corners = {
    Point2f(spacing * 2 + frame_color.cols, label_height),
    Point2f(spacing * 2 + frame_color.cols + tagSize - 1, label_height),
    Point2f(spacing * 2 + frame_color.cols + tagSize - 1, label_height + tagSize - 1),
    Point2f(spacing * 2 + frame_color.cols, label_height + tagSize - 1)
  };
  
  for (int i = 0; i < 4; i++) {
    circle(vis, warped_corners[i], 5, Scalar(0, 0, 255), -1);
    string corner_label = to_string(i);
    putText(vis, corner_label, Point(warped_corners[i].x + 5, warped_corners[i].y + 20), 
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
  }
  
  // Add statistics
  Scalar mean_w, stddev_w;
  meanStdDev(warped, mean_w, stddev_w);
  double min_val, max_val;
  minMaxLoc(warped, &min_val, &max_val);
  
  stringstream ss_stats;
  ss_stats << "Mean: " << fixed << setprecision(1) << mean_w[0] 
           << ", Std: " << setprecision(1) << stddev_w[0]
           << ", Range: " << (int)min_val << "-" << (int)max_val;
  putText(vis, ss_stats.str(), 
          Point(frame_color.cols + spacing * 2, label_height + tagSize + 20), 
          FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);
  
  // Draw border region on warped image (for AprilTag border visualization)
  int border_pixels = tagSize / 9;  // ~2 cells for Tag36h11
  Rect border_rect(spacing * 2 + frame_color.cols + border_pixels, 
                   label_height + border_pixels,
                   tagSize - 2 * border_pixels, 
                   tagSize - 2 * border_pixels);
  rectangle(vis, border_rect, Scalar(255, 0, 0), 2);
  putText(vis, "Data Region", 
          Point(spacing * 2 + frame_color.cols + border_pixels + 5, 
                label_height + border_pixels + 20), 
          FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 0, 0), 1);
  
  // Save individual visualization
  imwrite(output_path, vis);
  
  // Copy to output_vis for side-by-side comparison
  output_vis = vis.clone();
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
  cout << "HOMOGRAPHY TRANSFORMATION COMPARISON" << endl;
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
  
  // Visualize Stable quad0
  Mat vis_stable;
  string stable_path_out = output_dir + "/stable_quad0_homography.png";
  visualizeHomography(frame_stable, quads_stable[0], 0, "Stable.avi", vis_stable, stable_path_out);
  cout << "  Saved: " << stable_path_out << endl;
  
  // Visualize NewVideo quad2
  Mat vis_newvideo;
  string newvideo_path_out = output_dir + "/newvideo_quad2_homography.png";
  visualizeHomography(frame_newvideo, quads_newvideo[2], 2, "NewVideo.avi", vis_newvideo, newvideo_path_out);
  cout << "  Saved: " << newvideo_path_out << endl;
  
  // Create side-by-side comparison
  int max_height = max(vis_stable.rows, vis_newvideo.rows);
  int total_width = vis_stable.cols + vis_newvideo.cols + 40;
  Mat comparison(max_height, total_width, CV_8UC3, Scalar(32, 32, 32));
  
  vis_stable.copyTo(comparison(Rect(10, 0, vis_stable.cols, vis_stable.rows)));
  vis_newvideo.copyTo(comparison(Rect(vis_stable.cols + 30, 0, vis_newvideo.cols, vis_newvideo.rows)));
  
  // Add main title
  putText(comparison, "HOMOGRAPHY TRANSFORMATION COMPARISON", 
          Point(10, 15), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 255, 0), 2);
  
  string comparison_path = output_dir + "/homography_comparison.png";
  imwrite(comparison_path, comparison);
  cout << "  Saved: " << comparison_path << endl;
  
  // Also save just the warped images side by side for easier comparison
  int tagSize = 200;
  Mat stable_warped, newvideo_warped;
  
  vector<Point2f> dstQuad;
  dstQuad.push_back(Point2f(0, 0));
  dstQuad.push_back(Point2f(tagSize - 1, 0));
  dstQuad.push_back(Point2f(tagSize - 1, tagSize - 1));
  dstQuad.push_back(Point2f(0, tagSize - 1));
  
  Mat H_stable = getPerspectiveTransform(quads_stable[0], dstQuad);
  warpPerspective(frame_stable, stable_warped, H_stable, Size(tagSize, tagSize), INTER_CUBIC);
  
  Mat H_newvideo = getPerspectiveTransform(quads_newvideo[2], dstQuad);
  warpPerspective(frame_newvideo, newvideo_warped, H_newvideo, Size(tagSize, tagSize), INTER_CUBIC);
  
  Mat warped_side_by_side(tagSize, tagSize * 2 + 20, CV_8UC1, Scalar(128));
  stable_warped.copyTo(warped_side_by_side(Rect(0, 0, tagSize, tagSize)));
  newvideo_warped.copyTo(warped_side_by_side(Rect(tagSize + 20, 0, tagSize, tagSize)));
  
  // Convert to color for labels
  Mat warped_color;
  cvtColor(warped_side_by_side, warped_color, COLOR_GRAY2BGR);
  
  putText(warped_color, "Stable Quad0", Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
  putText(warped_color, "NewVideo Quad2", Point(tagSize + 25, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
  
  // Add statistics
  Scalar mean_s, stddev_s, mean_n, stddev_n;
  meanStdDev(stable_warped, mean_s, stddev_s);
  meanStdDev(newvideo_warped, mean_n, stddev_n);
  
  stringstream ss1, ss2;
  ss1 << "Mean: " << fixed << setprecision(1) << mean_s[0] << " Std: " << setprecision(1) << stddev_s[0];
  ss2 << "Mean: " << fixed << setprecision(1) << mean_n[0] << " Std: " << setprecision(1) << stddev_n[0];
  
  putText(warped_color, ss1.str(), Point(5, tagSize - 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0), 1);
  putText(warped_color, ss2.str(), Point(tagSize + 25, tagSize - 10), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 0), 1);
  
  string warped_path = output_dir + "/warped_side_by_side.png";
  imwrite(warped_path, warped_color);
  cout << "  Saved: " << warped_path << endl;
  
  cout << "\n========================================" << endl;
  cout << "HOMOGRAPHY COMPARISON COMPLETE" << endl;
  cout << "========================================" << endl;
  
  return 0;
}



