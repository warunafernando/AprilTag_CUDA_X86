#include "glog/logging.h"
#include "video_processor.h"

VideoProcessor::VideoProcessor(
    const int camera_index, const std::string& tag_family_name,
    const frc971::apriltag::CameraMatrix& camera_matrix,
    const frc971::apriltag::DistCoeffs& distortion_coefficients, int nthreads)
    : camera_index_(camera_index),
      tag_family_name_(tag_family_name),
      camera_matrix_(camera_matrix),
      distortion_coefficients_(distortion_coefficients),
      nthreads_(nthreads) {
  tf_ = nullptr;
  td_ = nullptr;
  cap_ = nullptr;
  gpu_detector_ = nullptr;
  initialized_ = false;
}

VideoProcessor::~VideoProcessor() {
  /*if (cap_ != nullptr) {
    delete cap_;
  }
  if (gpu_detector_ != nullptr) {
    delete gpu_detector_;
  }*/
  teardown_tag_family(&tf_, tag_family_name_.c_str());
  if (td_ != nullptr) {
    apriltag_detector_destroy(td_);
  }
}

bool VideoProcessor::initialize() {
  setup_tag_family(&tf_, tag_family_name_.c_str());
  td_ = apriltag_detector_create();
  apriltag_detector_add_family(td_, tf_);

  if (errno == ENOMEM) {
    LOG(ERROR)
        << "Unable to add family to detector due to insufficient memory to "
           "allocate the tag-family decoder with the default maximum hamming "
           "value of 2. Try choosing an alternative tag family.";
    return false;
  }

  // Setup the tag detector.
  td_->quad_decimate = 2.0;
  td_->quad_sigma = 0.0;
  td_->nthreads = nthreads_;
  td_->debug = false;
  td_->refine_edges = true;
  td_->wp = workerpool_create(nthreads_);

  // Setup the Video Capture device.
  cap_ = std::make_unique<VideoCapture>(camera_index_, CAP_V4L);
  if (!cap_->isOpened()) {
    LOG(ERROR) << "Couldn't open video capture device " << camera_index_;
    return false;
  }
  // Frames need to be in YUYV format to work properly with gpu detector.
  cap_->set(CAP_PROP_CONVERT_RGB, false);
  // cap_.set(CAP_PROP_MODE, CV_CAP_MODE_YUYV);

  // Initialize the GPU detector.
  int width = cap_->get(CAP_PROP_FRAME_WIDTH);
  int height = cap_->get(CAP_PROP_FRAME_HEIGHT);

  gpu_detector_ = std::make_unique<frc971::apriltag::GpuDetector>(
      width, height, td_, camera_matrix_, distortion_coefficients_);

  initialized_ = true;

  return true;
}

bool VideoProcessor::process() {
  if (!initialized_) {
    LOG(ERROR)
        << "VideoProcessor is not initialized.  Call initialize() first.";
    return false;
  }

  Mat yuyv_img, bgr_img;
  while (true) {
    errno = 0;
    *cap_ >> yuyv_img;

    // TOOO: Add error checking of number of channels here.
    cvtColor(yuyv_img, bgr_img, COLOR_YUV2BGR_YUYV);

    gpu_detector_->Detect(yuyv_img.data);
    const zarray_t* detections = gpu_detector_->Detections();
    draw_detection_outlines(bgr_img, const_cast<zarray_t*>(detections));
    img_ = bgr_img.clone();
  }
}