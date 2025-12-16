#include <gflags/gflags.h>

#include <iomanip>
#include <iostream>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "common/getopt.h"
#include "common/matd.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
}

using namespace std;
using namespace cv;

DEFINE_int32(camera, 0, "Camera ID");
DEFINE_bool(debug, false, "Enable debugging output (slow)");
DEFINE_bool(quiet, false, "Reduce output");
DEFINE_string(family, "tag36h11", "AprilTag family to use");
DEFINE_int32(threads, 1, "Use this many CPU threads");
DEFINE_double(decimate, 2.0, "Decimate input image by this factor");
DEFINE_double(blur, 0.0, "Apply low-pass blur to input");
DEFINE_bool(refine_edges, true,
            "Spend more time trying to align edges of tags");
DEFINE_bool(verbose, false, "Print out april tag detection results");
DEFINE_bool(cpuonly, false, "Use the CPU instead of CUDA");

int main(int argc, char *argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  TickMeter meter;
  meter.start();

  // Initialize tag detector with options
  apriltag_family_t *tf = NULL;
  setup_tag_family(&tf, FLAGS_family.c_str());

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (errno == ENOMEM) {
    printf(
        "Unable to add family to detector due to insufficient memory to "
        "allocate the tag-family decoder with the default maximum hamming "
        "value of 2. Try choosing an alternative tag family.\n");
    exit(-1);
  }

  td->quad_decimate = FLAGS_decimate;
  td->quad_sigma = FLAGS_blur;
  td->nthreads = FLAGS_threads;
  td->debug = FLAGS_debug;
  td->refine_edges = FLAGS_refine_edges;
  td->wp = workerpool_create(FLAGS_threads);

  meter.stop();
  cout << "Detector " << FLAGS_family << " initialized in " << std::fixed
       << std::setprecision(3) << meter.getTimeSec() << " seconds" << endl;

  // Initialize camera
  cout << "Enabling video capture" << endl;
  VideoCapture cap(FLAGS_camera, CAP_V4L);
  if (!cap.isOpened()) {
    cerr << "Couldn't open video capture device" << endl;
    return -1;
  }
  cap.set(CAP_PROP_CONVERT_RGB, false);
  // cap.set(CAP_PROP_MODE, CV_CAP_MODE_YUYV);
  cap.set(CAP_PROP_FRAME_WIDTH, 1920);
  cap.set(CAP_PROP_FRAME_HEIGHT, 1080);

  cout << "  " << cap.get(CAP_PROP_FRAME_WIDTH) << "x"
       << cap.get(CAP_PROP_FRAME_HEIGHT) << " @" << cap.get(CAP_PROP_FPS)
       << "FPS" << endl;

  if (FLAGS_cpuonly) {
    cout << "Running in CPU only mode" << endl;
  }
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

  int width = cap.get(CAP_PROP_FRAME_WIDTH);
  int height = cap.get(CAP_PROP_FRAME_HEIGHT);

  Mat bgr_img, bgr_img_copy, yuyv_img, gray;
  while (true) {
    errno = 0;
    cap >> yuyv_img;
    cvtColor(yuyv_img, bgr_img, COLOR_YUV2BGR_YUYV);
    bgr_img_copy = bgr_img.clone();

    if (FLAGS_cpuonly) {
      cvtColor(bgr_img, gray, COLOR_BGR2GRAY);
      image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
      zarray_t *detections = apriltag_detector_detect(td, &im);
      if (FLAGS_verbose) {
        print_detections(detections);
      }
      draw_detection_outlines(bgr_img, detections);
      apriltag_detections_destroy(detections);
    } else {
      frc971::apriltag::GpuDetector detector(width, height, td, cam, dist);
      detector.Detect(yuyv_img.data);
      const zarray_t *detections = detector.Detections();
      if (FLAGS_verbose) {
        print_detections(const_cast<zarray_t *>(detections));
      }
      draw_detection_outlines(bgr_img, const_cast<zarray_t *>(detections));
    }

    if (errno == EAGAIN) {
      printf("Unable to create the %d threads requested.\n", td->nthreads);
      exit(-1);
    }

    imshow("Tag Detections", bgr_img);
    if (waitKey(30) == 27) {
      cv::imwrite("colorimage.jpg", bgr_img_copy);
      cv::imwrite("colorimagewdetections.jpg", bgr_img);
    }
  }

  teardown_tag_family(&tf, FLAGS_family.c_str());
  apriltag_detector_destroy(td);

  gflags::ShutDownCommandLineFlags();
  return 0;
}
