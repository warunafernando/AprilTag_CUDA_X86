
#include <iomanip>
#include <iostream>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  TickMeter meter;
  meter.start();

  // Initialize tag detector with options
  const char *famname = "tag36h11";
  apriltag_family_t *tf = NULL;
  setup_tag_family(&tf, famname);

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (errno == ENOMEM) {
    printf(
        "Unable to add family to detector due to insufficient memory to "
        "allocate the tag-family decoder with the default maximum hamming "
        "value of 2. Try choosing an alternative tag family.\n");
    exit(-1);
  }

  td->quad_decimate = 2.0;
  td->quad_sigma = 0.0;
  td->nthreads = 1;
  td->debug = 0;
  td->refine_edges = 1;
  td->wp = workerpool_create(td->nthreads);

  meter.stop();
  cout << "Detector " << famname << " initialized in " << std::fixed
       << std::setprecision(3) << meter.getTimeSec() << " seconds" << endl;

  meter.reset();

  frc971::apriltag::CameraMatrix cam;
  cam.fx = 905.495617;
  cam.fy = 907.909470;
  cam.cx = 609.916016;
  cam.cy = 352.682645;

  frc971::apriltag::DistCoeffs dist;
  dist.k1 = 0.059238;
  dist.k2 = -0.075154;
  dist.p1 = -0.003801;
  dist.p2 = 0.001113;
  dist.k3 = 0.0;

  Mat bgr_img, yuyv_img;
  bgr_img = cv::imread("../data/colorimage.jpg", cv::IMREAD_COLOR);
  cvtColor(bgr_img, yuyv_img, COLOR_BGR2YUV_YUYV);
  int width = bgr_img.cols;
  int height = bgr_img.rows;

  frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
  detector.Detect(yuyv_img.data);
  const zarray_t *detections = detector.Detections();

  if (errno == EAGAIN) {
    printf("Unable to create the %d threads requested.\n", td->nthreads);
    exit(-1);
  }

  print_detections(const_cast<zarray_t *>(detections));
  draw_detection_outlines(bgr_img, const_cast<zarray_t *>(detections));

  imshow("Tag Detections", bgr_img);
  waitKey(0);

  cv::Mat gray_cuda(cv::Size(width, height), CV_8UC1);
  detector.CopyGrayTo(gray_cuda.data);
  imshow("gray_cuda", gray_cuda);
  waitKey(0);

  cv::Mat decimated_cuda(gray_cuda.size() / 2, CV_8UC1);
  detector.CopyDecimatedTo(decimated_cuda.data);
  imshow("decimated_cuda", decimated_cuda);
  waitKey(0);

  cv::Mat thresholded_cuda(decimated_cuda.size(), CV_8UC1);
  detector.CopyThresholdedTo(thresholded_cuda.data);
  imshow("thresholded_cuda", thresholded_cuda);
  waitKey(0);

  teardown_tag_family(&tf, famname);
  apriltag_detector_destroy(td);
  return 0;
}
