#ifndef VIDEO_PROCESSOR_H_
#define VIDEO_PROCESSOR_H_

#include <memory>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class VideoProcessor {
public:
  VideoProcessor(const int camera_index, const std::string &tag_family_name,
                 const frc971::apriltag::CameraMatrix &camera_matrix,
                 const frc971::apriltag::DistCoeffs &distortion_coefficients,
                 int nthreads = 4);

  ~VideoProcessor();

  bool initialize();
  bool process();

private:
  const std::string &tag_family_name_;
  int camera_index_;
  const frc971::apriltag::CameraMatrix &camera_matrix_;
  const frc971::apriltag::DistCoeffs &distortion_coefficients_;
  int nthreads_;
  apriltag_family_t *tf_;
  apriltag_detector_t *td_;
  std::unique_ptr<VideoCapture> cap_;
  std::unique_ptr<frc971::apriltag::GpuDetector> gpu_detector_;
  bool initialized_;
  Mat img_;
};

#endif