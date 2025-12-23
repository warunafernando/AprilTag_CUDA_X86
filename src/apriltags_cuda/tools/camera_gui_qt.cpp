// Qt-based camera GUI with camera selection and display.
// This tool enumerates V4L2 /dev/video* devices and MindVision cameras,
// lets the user pick a camera and resolution/FPS mode from combo boxes,
// and exposes basic controls via sliders. 
// When detection is enabled, it uses the video_visualize_fixed detection pipeline
// with camera input and GUI display output.

#include <opencv2/opencv.hpp>

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/image_u8.h"
#include "common/workerpool.h"
}
#include "g2d.h"

#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QCheckBox>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QProcess>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QStandardPaths>
#include <QMap>
#include <QDateTime>

#include "CameraApi.h"
#include "TagDetectionIPC.h"
#include "apriltag_utils.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>

using namespace cv;
using namespace std;

enum class Backend {
  OpenCV = 0,
  MVSDK = 1,
};

struct CameraInfo {
  int index;
  std::string name;
  bool is_webcam;
};

static std::string ToLower(const std::string &s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return out;
}

static std::string ReadV4L2Name(int index) {
#if defined(__linux__)
  std::string path =
      "/sys/class/video4linux/video" + std::to_string(index) + "/name";
  std::ifstream f(path);
  if (!f.is_open()) {
    return "video" + std::to_string(index);
  }
  std::string line;
  std::getline(f, line);
  if (line.empty()) {
    return "video" + std::to_string(index);
  }
  return line;
#else
  (void)index;
  return "camera_" + std::to_string(index);
#endif
}

static std::vector<CameraInfo> EnumerateV4L2Cameras() {
  std::vector<CameraInfo> result;
  for (int idx = 0; idx < 10; ++idx) {
    VideoCapture cap(idx, CAP_ANY);
    if (!cap.isOpened()) {
      continue;
    }
    double w = cap.get(CAP_PROP_FRAME_WIDTH);
    double h = cap.get(CAP_PROP_FRAME_HEIGHT);
    if (w <= 0 || h <= 0) {
      continue;
    }
    cap.release();

    std::string name = ReadV4L2Name(idx);
    std::string lower = ToLower(name);
    bool is_webcam = (lower.find("camera") != std::string::npos ||
                      lower.find("webcam") != std::string::npos ||
                      lower.find("uvc") != std::string::npos ||
                      lower.find("cam") != std::string::npos);

    result.push_back(CameraInfo{idx, name, is_webcam});
  }
  return result;
}

struct Mode {
  int width;
  int height;
  double fps;
  std::string label;
};

struct MVMode {
  int width;
  int height;
  int frame_speed_index;
  std::string label;
};

static bool EnsureMVSDKInitialized() {
  static bool inited = false;
  if (!inited) {
    if (CameraSdkInit(1) != CAMERA_STATUS_SUCCESS) {
      std::cerr << "CameraSdkInit failed\n";
      return false;
    }
    inited = true;
  }
  return inited;
}

class CameraWindow : public QWidget {
 public:
  explicit CameraWindow(QWidget *parent = nullptr)
      : QWidget(parent),
        backend_combo_(new QComboBox(this)),
        camera_combo_(new QComboBox(this)),
        mode_combo_(new QComboBox(this)),
        info_label_(new QLabel(this)),
        image_label_(new QLabel(this)),
        exposure_slider_(new QSlider(Qt::Horizontal, this)),
        brightness_slider_(new QSlider(Qt::Horizontal, this)),
        gain_slider_(new QSlider(Qt::Horizontal, this)),
        contrast_slider_(new QSlider(Qt::Horizontal, this)),
        saturation_slider_(new QSlider(Qt::Horizontal, this)),
        sharpness_slider_(new QSlider(Qt::Horizontal, this)),
        save_settings_button_(new QPushButton("Save Settings", this)),
        record_button_(new QPushButton("Record 30s Video", this)),
        fisheye_undistort_checkbox_(new QCheckBox("Enable Fisheye Undistortion", this)),
        timer_(new QTimer(this)),
        backend_(Backend::OpenCV),
        mv_handle_(0),
        mv_rgb_buffer_(nullptr),
        frame_count_(0),
        fps_(0.0),
        reading_fps_(0.0),
        detection_enabled_(false),
        detection_process_(nullptr),
        apriltag_detector_(nullptr),
        apriltag_family_(nullptr),
        tag_shm_(nullptr),
        tag_shm_fd_(-1),
        frame_shm_(nullptr),
        frame_shm_fd_(-1),
        frame_id_counter_(0),
        annotated_frame_shm_(nullptr),
        annotated_frame_shm_fd_(-1),
        last_annotated_frame_id_(0) {
    setWindowTitle("Camera Selector with AprilTag Detection");
    resize(960, 720);
    setMinimumSize(800, 600);
    move(100, 100);  // Position window explicitly

    backend_combo_->addItem("OpenCV (V4L2)",
                            static_cast<int>(Backend::OpenCV));
    backend_combo_->addItem("MindVision MVSDK",
                            static_cast<int>(Backend::MVSDK));

    ocv_devices_ = EnumerateV4L2Cameras();
    devices_ = ocv_devices_;

    ocv_modes_ = {
        {640, 480, 30.0, "640x480 @30"},
        {1280, 720, 30.0, "1280x720 @30"},
        {1920, 1080, 30.0, "1920x1080 @30"},
        {640, 480, 60.0, "640x480 @60"},
    };

    for (size_t i = 0; i < devices_.size(); ++i) {
      const auto &d = devices_[i];
      QString text = QString("[%1] index=%2 %3%4")
                         .arg(i)
                         .arg(d.index)
                         .arg(QString::fromStdString(d.name))
                         .arg(d.is_webcam ? " (webcam)" : "");
      camera_combo_->addItem(text, d.index);
    }

    for (size_t i = 0; i < ocv_modes_.size(); ++i) {
      mode_combo_->addItem(QString::fromStdString(ocv_modes_[i].label));
    }

    image_label_->setMinimumSize(640, 480);
    image_label_->setAlignment(Qt::AlignCenter);

    exposure_slider_->setRange(0, 100);
    exposure_slider_->setValue(50);
    brightness_slider_->setRange(0, 255);
    brightness_slider_->setValue(128);
    gain_slider_->setRange(0, 100);
    gain_slider_->setValue(50);
    contrast_slider_->setRange(0, 100);
    contrast_slider_->setValue(50);
    saturation_slider_->setRange(0, 100);
    saturation_slider_->setValue(50);
    sharpness_slider_->setRange(0, 100);
    sharpness_slider_->setValue(50);

    auto *top_bar = new QHBoxLayout;
    top_bar->addWidget(new QLabel("Backend:", this));
    top_bar->addWidget(backend_combo_);
    top_bar->addSpacing(20);
    top_bar->addWidget(new QLabel("Camera:", this));
    top_bar->addWidget(camera_combo_, 1);
    top_bar->addSpacing(20);
    top_bar->addWidget(new QLabel("Mode:", this));
    top_bar->addWidget(mode_combo_);

    auto *controls_layout = new QFormLayout;
    controls_layout->addRow("Exposure", exposure_slider_);
    controls_layout->addRow("Brightness", brightness_slider_);
    controls_layout->addRow("Gain", gain_slider_);
    controls_layout->addRow("Contrast", contrast_slider_);
    controls_layout->addRow("Saturation", saturation_slider_);
    controls_layout->addRow("Sharpness", sharpness_slider_);
    controls_layout->addRow("", fisheye_undistort_checkbox_);

    auto *controls_group = new QGroupBox("Controls", this);
    controls_group->setLayout(controls_layout);

    // Tag information label
    tag_info_label_ = new QLabel("No detections", this);
    tag_info_label_->setWordWrap(true);
    tag_info_label_->setMinimumHeight(100);
    tag_info_label_->setStyleSheet("QLabel { background-color: #2b2b2b; color: white; padding: 5px; }");

    auto *right_layout = new QVBoxLayout;
    right_layout->addWidget(info_label_);
    right_layout->addWidget(controls_group);
    right_layout->addWidget(save_settings_button_);
    right_layout->addWidget(record_button_);
    right_layout->addWidget(new QLabel("Tag Detections:", this));
    right_layout->addWidget(tag_info_label_);
    right_layout->addStretch(1);

    auto *main_layout = new QVBoxLayout;
    main_layout->addLayout(top_bar);

    auto *center_layout = new QHBoxLayout;
    center_layout->addWidget(image_label_, 3);
    center_layout->addLayout(right_layout, 1);

    main_layout->addLayout(center_layout, 1);
    setLayout(main_layout);

    connect(backend_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int idx) { OnBackendChanged(idx); });

    connect(camera_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int idx) { OnCameraChanged(idx); });
    connect(mode_combo_,
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            [this](int idx) { OnModeChanged(idx); });

    // OpenCV/V4L2 camera controls
    auto slider_to_cap = [this](QSlider *slider, int prop) {
      connect(slider, &QSlider::valueChanged, this,
              [this, slider, prop](int value) {
                if (backend_ != Backend::OpenCV) return;
                if (!cap_.isOpened()) return;
                double v = static_cast<double>(value);
                if (prop == CAP_PROP_EXPOSURE) {
                  v = (v - 50.0) / 10.0;
                }
                cap_.set(prop, v);
              });
    };

    slider_to_cap(exposure_slider_, CAP_PROP_EXPOSURE);
    slider_to_cap(brightness_slider_, CAP_PROP_BRIGHTNESS);
    slider_to_cap(gain_slider_, CAP_PROP_GAIN);
    slider_to_cap(contrast_slider_, CAP_PROP_CONTRAST);
    slider_to_cap(saturation_slider_, CAP_PROP_SATURATION);
    slider_to_cap(sharpness_slider_, CAP_PROP_SHARPNESS);
    
    // MindVision MVSDK camera controls
    connect(exposure_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              // Map slider value (0-100) to exposure time in microseconds
              // 0 = max exposure (slow), 100 = min exposure (fast)
              // For high FPS, we want values near 100 (low exposure)
              // Exposure range: ~1000 to ~100000 microseconds (1ms to 100ms)
              double min_exposure = 1000.0;
              double max_exposure = 100000.0;
              double exposure = max_exposure - (value / 100.0) * (max_exposure - min_exposure);
              CameraSetExposureTime(mv_handle_, exposure);
            });
    
    // Brightness: MVSDK doesn't have direct brightness control
    // Use brightness slider to adjust analog gain (for monochrome cameras) or gamma
    connect(brightness_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              // For monochrome cameras, adjust analog gain to affect brightness
              // Map 0-255 to analog gain range (typically 0-100)
              int analog_gain = (value * 100) / 255;
              CameraSetAnalogGain(mv_handle_, analog_gain);
            });
    
    connect(gain_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              // Gain is RGB gains, use same value for all channels
              // Gain value is typically 0-100, where 100 = 1.0x gain
              CameraSetGain(mv_handle_, value, value, value);
            });
    
    connect(contrast_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              CameraSetContrast(mv_handle_, value);
            });
    
    connect(saturation_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              CameraSetSaturation(mv_handle_, value);
            });
    
    connect(sharpness_slider_, &QSlider::valueChanged, this,
            [this](int value) {
              if (backend_ != Backend::MVSDK || mv_handle_ == 0) return;
              CameraSetSharpness(mv_handle_, value);
            });
    
    connect(save_settings_button_, &QPushButton::clicked, this,
            [this]() { SaveSettings(); });
    
    connect(record_button_, &QPushButton::clicked, this,
            [this]() { StartRecording(); });
    
    // Initialize fisheye checkbox state based on whether calibration was loaded
    fisheye_undistort_checkbox_->setChecked(fisheye_undistort_enabled_);
    connect(fisheye_undistort_checkbox_, &QCheckBox::toggled, this,
            [this](bool checked) {
              fisheye_undistort_enabled_ = checked;
              // Reset maps if disabled to free memory
              if (!checked) {
                fisheye_map1_ = cv::Mat();
                fisheye_map2_ = cv::Mat();
              }
              qDebug() << "Fisheye undistortion" << (checked ? "enabled" : "disabled");
            });

    connect(timer_, &QTimer::timeout, this, [this]() { OnUpdateFrame(); });
    timer_->start(10);

    fps_timer_.start();
    
    // Load fisheye calibration
    QString calib_path = "/home/nav/9202/Hiru/Apriltag/calibration_data/camera_params.yaml";
    LoadFisheyeCalibration(calib_path);
    
    // Load saved settings
    LoadSettings();

    // Apply saved camera selection after enumeration
    if (saved_camera_index_ >= 0 && saved_camera_index_ < static_cast<int>(devices_.size())) {
      camera_combo_->setCurrentIndex(saved_camera_index_);
      OpenCamera(saved_camera_index_);
      if (saved_mode_index_ >= 0) {
        mode_combo_->setCurrentIndex(saved_mode_index_);
        if (backend_ == Backend::OpenCV) {
          ApplyMode(saved_mode_index_);
        } else {
          ApplyMVMode(saved_mode_index_);
        }
      }
      // Apply saved camera control settings
      ApplySavedCameraSettings();
    } else if (!devices_.empty()) {
      camera_combo_->setCurrentIndex(0);
      OpenCamera(0);
      mode_combo_->setCurrentIndex(0);
      ApplyMode(0);
    }
  }

  ~CameraWindow() override {
    // Stop recording if active
    if (is_recording_) {
      StopRecording();
    }
    // Stop detection and wait for process to finish before destruction
    StopDetection();
    // Also stop reader thread
    StopReaderThread();
    cap_.release();
    if (mv_handle_ != 0) {
      CameraUnInit(mv_handle_);
      mv_handle_ = 0;
    }
    if (mv_rgb_buffer_) {
      free(mv_rgb_buffer_);
      mv_rgb_buffer_ = nullptr;
    }
  }

 private:
  void OnBackendChanged(int combo_index) {
    QVariant data = backend_combo_->itemData(combo_index);
    if (!data.isValid()) {
      return;
    }
    Backend new_backend = static_cast<Backend>(data.toInt());
    if (new_backend == backend_) {
      return;
    }
    
    StopReaderThread();
    backend_ = new_backend;

    cap_.release();
    if (mv_handle_ != 0) {
      CameraUnInit(mv_handle_);
      mv_handle_ = 0;
    }
    if (mv_rgb_buffer_) {
      free(mv_rgb_buffer_);
      mv_rgb_buffer_ = nullptr;
    }

    camera_combo_->blockSignals(true);
    camera_combo_->clear();
    camera_combo_->blockSignals(false);

    mode_combo_->blockSignals(true);
    mode_combo_->clear();
    mode_combo_->blockSignals(false);

    devices_.clear();

    if (backend_ == Backend::OpenCV) {
      devices_ = ocv_devices_;
      for (size_t i = 0; i < devices_.size(); ++i) {
        const auto &d = devices_[i];
        QString text = QString("[%1] index=%2 %3%4")
                           .arg(i)
                           .arg(d.index)
                           .arg(QString::fromStdString(d.name))
                           .arg(d.is_webcam ? " (webcam)" : "");
        camera_combo_->addItem(text, d.index);
      }

      mode_combo_->blockSignals(true);
      for (size_t i = 0; i < ocv_modes_.size(); ++i) {
        mode_combo_->addItem(QString::fromStdString(ocv_modes_[i].label));
      }
      mode_combo_->blockSignals(false);

      if (!devices_.empty()) {
        camera_combo_->setCurrentIndex(0);
        OpenCamera(0);
        mode_combo_->setCurrentIndex(0);
        ApplyMode(0);
      }
    } else {
      if (!EnsureMVSDKInitialized()) {
        info_label_->setText("Failed to initialize MindVision SDK.");
        return;
      }
      tSdkCameraDevInfo list[16];
      INT count = 16;
      CameraSdkStatus s = CameraEnumerateDevice(list, &count);
      if (s != CAMERA_STATUS_SUCCESS || count <= 0) {
        info_label_->setText("No MindVision cameras found via MVSDK.");
        return;
      }
      mv_devices_.assign(list, list + count);
      for (int i = 0; i < count; ++i) {
        const tSdkCameraDevInfo &dev = mv_devices_[i];
        std::string name = dev.acFriendlyName[0]
                               ? dev.acFriendlyName
                               : dev.acProductName;
        devices_.push_back(CameraInfo{i, name, false});
        QString text = QString("[%1] MV index=%2 %3")
                           .arg(i)
                           .arg(i)
                           .arg(QString::fromStdString(name));
        camera_combo_->addItem(text, i);
      }

      if (!devices_.empty()) {
        camera_combo_->setCurrentIndex(0);
        OpenCamera(0);
      }
    }
  }

  void OnCameraChanged(int combo_index) {
    if (combo_index < 0 || combo_index >= static_cast<int>(devices_.size())) {
      return;
    }
    OpenCamera(combo_index);
  }

  void OnModeChanged(int combo_index) {
    if (combo_index < 0) {
      return;
    }
    if (backend_ == Backend::OpenCV) {
      if (combo_index >= static_cast<int>(ocv_modes_.size())) {
        return;
      }
      ApplyMode(combo_index);
    } else {
      if (mv_handle_ == 0) {
        return;
      }
      if (combo_index >= static_cast<int>(mv_modes_.size())) {
        return;
      }
      ApplyMVMode(combo_index);
    }
  }

  void OpenCamera(int list_index) {
    StopDetection();  // Stop any existing detection when changing camera
    StopReaderThread();
    if (list_index < 0 || list_index >= static_cast<int>(devices_.size())) {
      return;
    }
    if (backend_ == Backend::OpenCV) {
      if (cap_.isOpened()) {
        cap_.release();
      }
      int real_index = devices_[list_index].index;
      cap_.open(real_index, CAP_ANY);
      if (!cap_.isOpened()) {
        info_label_->setText(
            QString("Failed to open V4L2 camera index %1").arg(real_index));
        return;
      }
      cap_.set(CAP_PROP_CONVERT_RGB, false);
      info_label_->setText(QString("Opened V4L2 camera index %1 (%2)")
                               .arg(real_index)
                               .arg(QString::fromStdString(
                                   devices_[list_index].name)));
      mode_combo_->blockSignals(true);
      mode_combo_->clear();
      for (size_t i = 0; i < ocv_modes_.size(); ++i) {
        mode_combo_->addItem(QString::fromStdString(ocv_modes_[i].label));
      }
      mode_combo_->setCurrentIndex(0);
      mode_combo_->blockSignals(false);
      ApplyMode(0);
      
      // Start detection automatically after a short delay (using simple CPU-based detector)
      QTimer::singleShot(500, this, [this]() {
        StartDetection();
      });
    } else {
      OpenMVSDKCamera(list_index);
    }
  }

  void OpenMVSDKCamera(int list_index) {
    StopDetection();  // Stop any existing detection when changing camera
    if (list_index < 0 || list_index >= static_cast<int>(mv_devices_.size())) {
      return;
    }
    if (!EnsureMVSDKInitialized()) {
      info_label_->setText("Failed to initialize MindVision SDK.");
      return;
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
    if (mv_handle_ != 0) {
      CameraUnInit(mv_handle_);
      mv_handle_ = 0;
    }
    if (mv_rgb_buffer_) {
      free(mv_rgb_buffer_);
      mv_rgb_buffer_ = nullptr;
    }

    tSdkCameraDevInfo dev = mv_devices_[list_index];
    CameraHandle hCam = 0;
    CameraSdkStatus s = CameraInit(&dev, -1, -1, &hCam);
    if (s != CAMERA_STATUS_SUCCESS) {
      info_label_->setText("CameraInit failed for MindVision camera.");
      return;
    }
    mv_handle_ = hCam;

    CameraGetCapability(mv_handle_, &mv_capability_);

    int maxw = mv_capability_.sResolutionRange.iWidthMax;
    int maxh = mv_capability_.sResolutionRange.iHeightMax;
    mv_rgb_buffer_ =
        static_cast<unsigned char *>(malloc(static_cast<size_t>(maxw) *
                                            static_cast<size_t>(maxh) * 3));

    if (mv_capability_.sIspCapacity.bMonoSensor) {
      CameraSetIspOutFormat(mv_handle_, CAMERA_MEDIA_TYPE_MONO8);
    } else {
      CameraSetIspOutFormat(mv_handle_, CAMERA_MEDIA_TYPE_BGR8);
    }

    CameraPlay(mv_handle_);
    
    // Configure camera for high-speed reading (disable auto exposure, set exposure)
    BOOL ae_state = FALSE;
    CameraGetAeState(mv_handle_, &ae_state);
    if (ae_state) {
      CameraSetAeState(mv_handle_, FALSE);
    }
    // Set exposure time for high FPS (5ms for ~160-200 FPS)
    CameraSetExposureTime(mv_handle_, 5000.0);
    
    // Initialize slider values from camera settings
    double current_exposure = 0.0;
    if (CameraGetExposureTime(mv_handle_, &current_exposure) == CAMERA_STATUS_SUCCESS) {
      // Map exposure time to slider value (0-100)
      double min_exposure = 1000.0;
      double max_exposure = 100000.0;
      int slider_value = static_cast<int>(((max_exposure - current_exposure) / (max_exposure - min_exposure)) * 100.0);
      slider_value = std::max(0, std::min(100, slider_value));
      exposure_slider_->blockSignals(true);
      exposure_slider_->setValue(slider_value);
      exposure_slider_->blockSignals(false);
    }
    
    int current_gain_r = 0, current_gain_g = 0, current_gain_b = 0;
    if (CameraGetGain(mv_handle_, &current_gain_r, &current_gain_g, &current_gain_b) == CAMERA_STATUS_SUCCESS) {
      gain_slider_->blockSignals(true);
      gain_slider_->setValue(current_gain_r);  // Use R channel value
      gain_slider_->blockSignals(false);
    }
    
    int current_contrast = 0;
    if (CameraGetContrast(mv_handle_, &current_contrast) == CAMERA_STATUS_SUCCESS) {
      contrast_slider_->blockSignals(true);
      contrast_slider_->setValue(current_contrast);
      contrast_slider_->blockSignals(false);
    }
    
    int current_saturation = 0;
    if (CameraGetSaturation(mv_handle_, &current_saturation) == CAMERA_STATUS_SUCCESS) {
      saturation_slider_->blockSignals(true);
      saturation_slider_->setValue(current_saturation);
      saturation_slider_->blockSignals(false);
    }
    
    int current_sharpness = 0;
    if (CameraGetSharpness(mv_handle_, &current_sharpness) == CAMERA_STATUS_SUCCESS) {
      sharpness_slider_->blockSignals(true);
      sharpness_slider_->setValue(current_sharpness);
      sharpness_slider_->blockSignals(false);
    }
    
    INT current_analog_gain = 0;
    if (CameraGetAnalogGain(mv_handle_, &current_analog_gain) == CAMERA_STATUS_SUCCESS) {
      // Map analog gain (0-100 typically) to brightness slider (0-255)
      int brightness_value = (current_analog_gain * 255) / 100;
      brightness_slider_->blockSignals(true);
      brightness_slider_->setValue(brightness_value);
      brightness_slider_->blockSignals(false);
    }
    
    mv_modes_.clear();
    mv_modes_.push_back(
        MVMode{1280, 1024, FRAME_SPEED_SUPER, "1280x1024 @211 FPS"});
    mv_modes_.push_back(
        MVMode{1280, 1024, FRAME_SPEED_HIGH, "1280x1024 @106 FPS"});
    mv_modes_.push_back(
        MVMode{480, 640, FRAME_SPEED_SUPER, "480x640 @790 FPS"});

    mode_combo_->blockSignals(true);
    mode_combo_->clear();
    for (const auto &m : mv_modes_) {
      mode_combo_->addItem(QString::fromStdString(m.label));
    }
    mode_combo_->setCurrentIndex(0);
    mode_combo_->blockSignals(false);
    ApplyMVMode(0);

    std::string name = dev.acFriendlyName[0] ? dev.acFriendlyName
                                             : dev.acProductName;
    info_label_->setText(QString("Opened MindVision camera [%1] (%2)")
                             .arg(list_index)
                             .arg(QString::fromStdString(name)));
    
    // Start threaded reader for MindVision camera
    StartReaderThread();
    
    // Start detection automatically after a short delay
    QTimer::singleShot(500, this, [this]() {
      StartDetection();
    });
  }
  
  void StartReaderThread() {
    if (backend_ == Backend::MVSDK && mv_handle_ != 0 && !reader_running_.load()) {
      reader_running_.store(true);
      frame_queue_.clear();
      reader_thread_ = std::thread(&CameraWindow::ReaderThreadFunc, this);
    }
  }
  
  void StopReaderThread() {
    if (reader_running_.load()) {
      reader_running_.store(false);
      if (reader_thread_.joinable()) {
        reader_thread_.join();
      }
    }
    frame_queue_.clear();
  }
  
  void ReaderThreadFunc() {
    if (mv_handle_ == 0 || mv_rgb_buffer_ == nullptr) return;
    
    bool is_mono = mv_capability_.sIspCapacity.bMonoSensor;
    std::vector<std::chrono::steady_clock::time_point> read_times;
    read_times.reserve(1000);
    const auto target_frame_interval = std::chrono::microseconds(static_cast<int>(1000000.0 / 160.0));
    auto last_frame_time = std::chrono::steady_clock::now();
    
    while (reader_running_.load()) {
      BYTE *pbyBuffer = nullptr;
      tSdkFrameHead sFrameInfo;
      CameraSdkStatus s = CameraGetImageBuffer(mv_handle_, &sFrameInfo, &pbyBuffer, 100);
      
      if (s != CAMERA_STATUS_SUCCESS) {
        if (s == CAMERA_STATUS_TIME_OUT) {
          continue;
        }
        break;
      }
      
      auto read_time = std::chrono::steady_clock::now();
      
      // Convert frame to Mat
      Mat frame;
      if (is_mono) {
        frame = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1, pbyBuffer).clone();
      } else {
        CameraImageProcess(mv_handle_, pbyBuffer, mv_rgb_buffer_, &sFrameInfo);
        frame = Mat(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC3, mv_rgb_buffer_).clone();
      }
      
      CameraReleaseImageBuffer(mv_handle_, pbyBuffer);
      
      // Calculate reading FPS
      read_times.push_back(read_time);
      if (read_times.size() > 1000) {
        read_times.erase(read_times.begin());
      }
      double fps = 0.0;
      if (read_times.size() >= 2) {
        auto total_duration = std::chrono::duration<double>(read_times.back() - read_times.front()).count();
        if (total_duration > 0.0) {
          fps = (read_times.size() - 1) / total_duration;
        }
      }
      
      // Add to queue
      FrameData frame_data;
      frame_data.frame = frame;
      frame_data.timestamp = read_time;
      frame_data.reading_fps = fps;
      frame_queue_.push(frame_data);
      reading_fps_ = fps;
      
      // Throttle to target FPS
      auto current_time = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_frame_time);
      if (elapsed < target_frame_interval) {
        std::this_thread::sleep_for(target_frame_interval - elapsed);
      }
      last_frame_time = std::chrono::steady_clock::now();
    }
  }

  void ApplyMode(int mode_index) {
    if (backend_ != Backend::OpenCV) return;
    if (!cap_.isOpened()) return;
    if (mode_index < 0 ||
        mode_index >= static_cast<int>(ocv_modes_.size()))
      return;
    const Mode &m = ocv_modes_[mode_index];
    cap_.set(CAP_PROP_FRAME_WIDTH, m.width);
    cap_.set(CAP_PROP_FRAME_HEIGHT, m.height);
    cap_.set(CAP_PROP_FPS, m.fps);
  }

  void ApplyMVMode(int mode_index) {
    if (backend_ != Backend::MVSDK) return;
    if (mv_handle_ == 0) return;
    if (mode_index < 0 ||
        mode_index >= static_cast<int>(mv_modes_.size()))
      return;
    const MVMode &m = mv_modes_[mode_index];
    CameraSetImageResolutionEx(mv_handle_, 0xff, 0, 0, 0, 0,
                               m.width, m.height, 0, 0);
    CameraSetFrameSpeed(mv_handle_, m.frame_speed_index);
  }

  void StartDetection() {
    int camera_index = -1;
    
    if (backend_ == Backend::OpenCV) {
      // For OpenCV/V4L2 cameras, use the device index
      if (camera_combo_->currentIndex() >= 0 &&
          camera_combo_->currentIndex() < static_cast<int>(devices_.size())) {
        camera_index = devices_[camera_combo_->currentIndex()].index;
      }
    } else if (backend_ == Backend::MVSDK) {
      // For MindVision cameras, use the MindVision camera index directly
      // (the detection process will use --mvsdk flag to access via SDK)
      if (camera_combo_->currentIndex() >= 0 &&
          camera_combo_->currentIndex() < static_cast<int>(mv_devices_.size())) {
        camera_index = camera_combo_->currentIndex();  // Use MindVision camera index directly
      }
    }

    if (camera_index < 0) {
      QMessageBox::warning(this, "No Camera",
                          "Please select a camera first.");
      detection_enabled_ = false;
      return;
    }

    // For now, use AprilTag library directly in GUI (simpler for debugging)
    // Stop any existing detection process first
    StopDetection();
    
    // Initialize AprilTag detector if needed
    if (apriltag_detector_ == nullptr) {
      if (setup_tag_family(&apriltag_family_, "tag36h11")) {
        apriltag_detector_ = apriltag_detector_create();
        apriltag_detector_add_family(apriltag_detector_, apriltag_family_);
        apriltag_detector_->quad_decimate = 2.0;
        apriltag_detector_->quad_sigma = 0.0;
        apriltag_detector_->refine_edges = 1;
        apriltag_detector_->nthreads = 1;
        apriltag_detector_->wp = workerpool_create(1);
      }
    }
    
    detection_enabled_ = true;
    info_label_->setText("AprilTag detection enabled (direct in GUI, CPU-based)");
    return;  // Skip the separate process approach for now
    
    // Launch video_visualize_fixed with --camera and --no-display options
    // The detection pipeline runs unchanged, but imshow is disabled for GUI mode
    // Note: For MindVision cameras, we use --mvsdk flag to access via SDK directly
    detection_process_ = new QProcess(this);
    // Set process channel mode to avoid issues
    detection_process_->setProcessChannelMode(QProcess::MergedChannels);
    
    // Find video_visualize_fixed executable - look in same directory as GUI first
    QString program;
    QString gui_path = QCoreApplication::applicationFilePath();
    QFileInfo gui_info(gui_path);
    QString gui_dir = gui_info.absolutePath();
    QString local_path = gui_dir + "/video_visualize_fixed";
    
    if (QFile::exists(local_path)) {
      program = local_path;
    } else {
      // Fall back to PATH (user must have it in PATH or use full path)
      program = "video_visualize_fixed";
    }
    
    QStringList arguments;
    if (backend_ == Backend::MVSDK) {
      // Use shared memory for frames: GUI reads camera, writes to shared memory, detection reads from shared memory
      arguments << "--frames-shm" << "--no-display";
    } else {
      arguments << "--camera" << QString::number(camera_index) << "--no-display";
    }
    
    // Debug: show which camera index is being used
    QString debug_msg = QString("Starting detection with camera index: %1").arg(camera_index);
    if (backend_ == Backend::MVSDK && camera_combo_->currentIndex() >= 0 && 
        camera_combo_->currentIndex() < static_cast<int>(mv_devices_.size())) {
      const tSdkCameraDevInfo &mv_dev = mv_devices_[camera_combo_->currentIndex()];
      std::string mv_name = mv_dev.acFriendlyName[0] ? mv_dev.acFriendlyName : mv_dev.acProductName;
      debug_msg += QString(" (MVSDK camera: %1)").arg(QString::fromStdString(mv_name));
    }
    info_label_->setText(debug_msg);

    connect(detection_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int exitCode, QProcess::ExitStatus status) {
              if (status == QProcess::CrashExit) {
                info_label_->setText("Detection process crashed.");
              }
              if (detection_process_) {
                detection_process_->deleteLater();
                detection_process_ = nullptr;
              }
              detection_enabled_ = false;
            });

    // Set working directory to ensure video_visualize_fixed can find config.txt
    QString working_dir = QDir::currentPath();
    detection_process_->setWorkingDirectory(working_dir);
    
    // Connect to error output to capture camera access errors
    connect(detection_process_, &QProcess::readyReadStandardError, this, [this]() {
      QByteArray error_data = detection_process_->readAllStandardError();
      QString error_text = QString::fromLocal8Bit(error_data);
      if (error_text.contains("Failed to open camera") || 
          error_text.contains("No input specified")) {
        info_label_->setText("Camera access failed - camera may not be accessible via V4L2");
      }
    });

    detection_process_->start(program, arguments);
    if (!detection_process_->waitForStarted(2000)) {
      QMessageBox::critical(this, "Detection Error",
                           QString("Failed to start detection process.\n"
                                  "Make sure video_visualize_fixed is in your PATH.\n"
                                  "Program: %1").arg(program));
      detection_process_->deleteLater();
      detection_process_ = nullptr;
      detection_enabled_ = false;
    } else {
      detection_enabled_ = true;
      if (backend_ == Backend::MVSDK) {
        info_label_->setText(QString("Detection started for MindVision camera (trying V4L2 index %1). "
                                    "If camera is not accessible via V4L2, detection will fail.").arg(camera_index));
      } else {
        info_label_->setText(QString("Detection started for camera %1").arg(camera_index));
      }
      
      // For MVSDK, create shared memory for frames (GUI writes, detection reads)
      if (backend_ == Backend::MVSDK && frame_shm_fd_ < 0) {
        frame_shm_fd_ = shm_open(FRAME_IPC_KEY, O_CREAT | O_RDWR, 0666);
        if (frame_shm_fd_ >= 0) {
          if (ftruncate(frame_shm_fd_, sizeof(FrameIPCData)) == 0) {
            void* mem = mmap(0, sizeof(FrameIPCData), PROT_READ | PROT_WRITE, MAP_SHARED, frame_shm_fd_, 0);
            if (mem != MAP_FAILED) {
              frame_shm_ = static_cast<FrameIPCData*>(mem);
              frame_shm_->width = 0;
              frame_shm_->height = 0;
              frame_shm_->channels = 1;
              frame_shm_->frame_id = 0;
              frame_shm_->timestamp = 0.0;
              frame_id_counter_ = 0;
            } else {
              ::close(frame_shm_fd_);
              frame_shm_fd_ = -1;
              shm_unlink(FRAME_IPC_KEY);
            }
          } else {
            ::close(frame_shm_fd_);
            frame_shm_fd_ = -1;
            shm_unlink(FRAME_IPC_KEY);
          }
        }
      }
      
      // Open shared memory for tag detections (retry if not available yet)
      if (tag_shm_fd_ < 0) {
        tag_shm_fd_ = shm_open(TAG_IPC_KEY, O_RDONLY, 0666);
        if (tag_shm_fd_ >= 0) {
          void* mem = mmap(0, sizeof(TagDetectionData), PROT_READ, MAP_SHARED, tag_shm_fd_, 0);
          if (mem != MAP_FAILED) {
            tag_shm_ = static_cast<TagDetectionData*>(mem);
          } else {
            ::close(tag_shm_fd_);
            tag_shm_fd_ = -1;
          }
        }
      }
    }
  }

  void StopDetection() {
    if (detection_process_) {
      // Disconnect all signals first to prevent events after process is deleted
      detection_process_->disconnect();
      
      // Stop the process if it's still running
      if (detection_process_->state() != QProcess::NotRunning) {
        // Terminate gracefully first
        detection_process_->terminate();
        if (!detection_process_->waitForFinished(1000)) {
          // If still running, force kill
          detection_process_->kill();
          detection_process_->waitForFinished(500);
        }
      }
      
      // Delete the process object
      // Use deleteLater() if we're not in destructor, otherwise delete immediately
      // For safety, we'll always use deleteLater() and let Qt handle it
      detection_process_->deleteLater();
      detection_process_ = nullptr;
    }
    
    detection_enabled_ = false;
    
    // Cleanup shared memory
    if (frame_shm_ != nullptr && frame_shm_ != MAP_FAILED) {
      munmap(frame_shm_, sizeof(FrameIPCData));
      frame_shm_ = nullptr;
    }
    if (frame_shm_fd_ >= 0) {
      ::close(frame_shm_fd_);
      frame_shm_fd_ = -1;
      shm_unlink(FRAME_IPC_KEY);
    }
    if (tag_shm_ != nullptr && tag_shm_ != MAP_FAILED) {
      munmap(tag_shm_, sizeof(TagDetectionData));
      tag_shm_ = nullptr;
    }
    if (tag_shm_fd_ >= 0) {
      ::close(tag_shm_fd_);
      tag_shm_fd_ = -1;
    }
    // Cleanup annotated frame shared memory
    if (annotated_frame_shm_ != nullptr && annotated_frame_shm_ != MAP_FAILED) {
      munmap(annotated_frame_shm_, sizeof(AnnotatedFrameIPCData));
      annotated_frame_shm_ = nullptr;
    }
    if (annotated_frame_shm_fd_ >= 0) {
      ::close(annotated_frame_shm_fd_);
      annotated_frame_shm_fd_ = -1;
    }
    
    detection_enabled_ = false;
    tag_info_label_->setText("No detections");
    
    // Clean up AprilTag detector
    if (apriltag_detector_ != nullptr) {
      apriltag_detector_destroy(apriltag_detector_);
      apriltag_detector_ = nullptr;
    }
    if (apriltag_family_ != nullptr) {
      teardown_tag_family(&apriltag_family_, "tag36h11");
      apriltag_family_ = nullptr;
    }
  }

  static QImage MatToQImage(const cv::Mat &mat) {
    if (mat.empty()) return {};
    cv::Mat rgb;
    switch (mat.channels()) {
      case 1:
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
        break;
      case 2:
        cv::cvtColor(mat, rgb, cv::COLOR_YUV2RGB_YUY2);
        break;
      case 3:
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        break;
      case 4:
        cv::cvtColor(mat, rgb, cv::COLOR_BGRA2RGB);
        break;
      default:
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        break;
    }
    return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step),
                  QImage::Format_RGB888)
        .copy();
  }

  void OnUpdateFrame() {
    Mat frame;
    Mat original_frame_for_recording;  // Store original frame before undistortion for recording
    
    // If detection is enabled, display annotated frames from detection process
    // Otherwise, display raw camera feed
    if (detection_enabled_ && backend_ == Backend::MVSDK) {
      // Try to open annotated frame shared memory if not already open
      if (annotated_frame_shm_ == nullptr && annotated_frame_shm_fd_ < 0) {
        annotated_frame_shm_fd_ = shm_open(ANNOTATED_FRAME_IPC_KEY, O_RDONLY, 0666);
        if (annotated_frame_shm_fd_ >= 0) {
          void* mem = mmap(0, sizeof(AnnotatedFrameIPCData), PROT_READ, MAP_SHARED, annotated_frame_shm_fd_, 0);
          if (mem != MAP_FAILED) {
            annotated_frame_shm_ = static_cast<AnnotatedFrameIPCData*>(mem);
          } else {
            ::close(annotated_frame_shm_fd_);
            annotated_frame_shm_fd_ = -1;
          }
        }
      }
      
      // Read annotated frame from shared memory
      if (annotated_frame_shm_ != nullptr && 
          annotated_frame_shm_->width > 0 && annotated_frame_shm_->height > 0 &&
          annotated_frame_shm_->frame_id != last_annotated_frame_id_) {
        // Create Mat from annotated frame data (BGR format)
        frame = Mat(annotated_frame_shm_->height, annotated_frame_shm_->width, CV_8UC3,
                   (void*)annotated_frame_shm_->data, annotated_frame_shm_->width * 3);
        frame = frame.clone(); // Clone to own the data
        last_annotated_frame_id_ = annotated_frame_shm_->frame_id;
        // Apply fisheye undistortion if enabled
        if (fisheye_undistort_enabled_) {
          frame = UndistortFrame(frame);
        }
      } else if (has_last_frame_) {
        // Use last frame if no new annotated frame available
        frame = last_frame_;
      }
    } else {
      // Display raw camera feed
      if (backend_ == Backend::OpenCV) {
        if (cap_.isOpened()) {
          cap_ >> frame;
        }
      } else {
        // Use threaded reader for MindVision cameras
        FrameData frame_data;
        if (frame_queue_.try_pop(frame_data, 5)) {
          frame = frame_data.frame;
          reading_fps_ = frame_data.reading_fps;
        } else {
          // No frame available, use last frame if we have one
          if (has_last_frame_) {
            frame = last_frame_;
          }
        }
      }
      
      // Store original frame before undistortion for recording
      if (!frame.empty()) {
        original_frame_for_recording = frame.clone();
      }
      
      // Apply fisheye undistortion if enabled
      if (!frame.empty() && fisheye_undistort_enabled_) {
        frame = UndistortFrame(frame);
      }
      
      // Apply AprilTag detection if enabled (using apriltag library directly)
      if (detection_enabled_ && apriltag_detector_ != nullptr && !frame.empty() && frame.cols > 0 && frame.rows > 0) {
        // Convert to grayscale if needed
        Mat gray;
        if (frame.channels() == 3) {
          cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else if (frame.channels() == 2) {
          cvtColor(frame, gray, cv::COLOR_YUV2GRAY_YUY2);
        } else {
          gray = frame;
        }
        
        // Ensure gray frame is valid and contiguous
        if (!gray.empty() && gray.cols > 0 && gray.rows > 0 && gray.data != nullptr) {
          // Ensure data is contiguous
          if (!gray.isContinuous()) {
            gray = gray.clone();
          }
          
          // Create image_u8_t for apriltag detector
          image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,  // Use cols for stride, not step[0] which might be padded
            .buf = gray.data
          };
          
          // Detect tags
          zarray_t* detections = apriltag_detector_detect(apriltag_detector_, &im);
          
          // Draw detections on frame
          Mat display_frame;
          if (frame.channels() == 1) {
            cvtColor(frame, display_frame, cv::COLOR_GRAY2BGR);
          } else {
            display_frame = frame.clone();
          }
          
          // Draw detection outlines
          draw_detection_outlines(display_frame, detections);
          
          // Update tag info label
          int num_detections = zarray_size(detections);
          if (num_detections == 0) {
            tag_info_label_->setText("Waiting for detections...");
          } else {
            QString tag_text = QString("Tags detected: %1\n").arg(num_detections);
            for (int i = 0; i < num_detections; i++) {
              apriltag_detection_t* det;
              zarray_get(detections, i, &det);
              tag_text += QString("ID: %1\n").arg(det->id);
            }
            tag_info_label_->setText(tag_text);
          }
          
          // Clean up detections
          for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);
            apriltag_detection_destroy(det);
          }
          zarray_destroy(detections);
          
          frame = display_frame;
        }
      }
    }

    if (frame.empty()) {
      if (has_last_frame_) {
        frame = last_frame_;
      } else {
        QImage placeholder(640, 480, QImage::Format_RGB888);
        placeholder.fill(Qt::black);
        QPainter p(&placeholder);
        p.setPen(Qt::white);
        QString msg = detection_enabled_ ? "Waiting for detection output..." : "No frame from selected camera";
        p.drawText(placeholder.rect(), Qt::AlignCenter, msg);
        p.end();
        image_label_->setPixmap(QPixmap::fromImage(placeholder));
        return;
      }
    } else {
      frame.copyTo(last_frame_);
      has_last_frame_ = true;
    }

    ++frame_count_;
    qint64 ms = fps_timer_.elapsed();
    if (ms >= 500) {
      fps_ = (frame_count_ * 1000.0) / static_cast<double>(ms);
      frame_count_ = 0;
      fps_timer_.restart();
    }

    // Write frame to video if recording
    // Record based on fisheye checkbox state:
    // - If checkbox enabled: record UNDISTORTED frame (what user sees)
    // - If checkbox disabled: record original frame (no undistortion applied)
    Mat frame_for_recording;
    if (fisheye_undistort_enabled_) {
      // User wants undistorted video - record the undistorted frame (what's displayed)
      frame_for_recording = frame;
    } else {
      // No undistortion - record original frame
      if (!original_frame_for_recording.empty()) {
        frame_for_recording = original_frame_for_recording;
      } else {
        frame_for_recording = frame;
      }
    }
    if (is_recording_ && video_writer_.isOpened() && !frame_for_recording.empty() && frame_for_recording.cols > 0 && frame_for_recording.rows > 0) {
      // Ensure frame is in BGR format for video writer (use original frame before undistortion)
      cv::Mat frame_to_write;
      if (frame_for_recording.channels() == 1) {
        // Convert grayscale to BGR
        cv::cvtColor(frame_for_recording, frame_to_write, cv::COLOR_GRAY2BGR);
      } else if (frame_for_recording.channels() == 3) {
        // Already BGR (or RGB, but OpenCV uses BGR)
        frame_to_write = frame_for_recording.clone();  // Make a copy to ensure data is valid
      } else {
        // Unsupported format, skip this frame
        qDebug() << "Warning: Frame has unsupported channel count:" << frame_for_recording.channels();
        frame_to_write = cv::Mat();
      }
      
      if (!frame_to_write.empty() && frame_to_write.data != nullptr && 
          frame_to_write.cols > 0 && frame_to_write.rows > 0) {
        // Ensure frame is contiguous
        if (!frame_to_write.isContinuous()) {
          frame_to_write = frame_to_write.clone();
        }
        
        // Ensure frame is CV_8UC3 (8-bit, 3 channels, BGR)
        if (frame_to_write.type() != CV_8UC3) {
          if (recorded_frame_count_ < 3) {  // Only log first few
            qDebug() << "Warning: Frame type mismatch. Expected CV_8UC3, got:" << frame_to_write.type();
          }
          frame_to_write.convertTo(frame_to_write, CV_8UC3);
        }
        
        // Verify frame is valid before writing
        if (frame_to_write.empty() || frame_to_write.data == nullptr) {
          qDebug() << "ERROR: frame_to_write became invalid after conversion!";
        } else {
        
          // Write frame to video using stream operator
          try {
            video_writer_ << frame_to_write;
            recorded_frame_count_++;
            
            // Log every 30 frames (roughly once per second at 30fps)
            if (recorded_frame_count_ % 30 == 0) {
              qDebug() << "Recorded" << recorded_frame_count_ << "frames, elapsed:" << recording_timer_.elapsed() << "ms";
            }
            
            // Log first few frames for debugging
            if (recorded_frame_count_ <= 3) {
              qDebug() << "Frame" << recorded_frame_count_ << "written - size:" << frame_to_write.cols << "x" << frame_to_write.rows
                       << "type:" << frame_to_write.type() << "channels:" << frame_to_write.channels();
            }
          } catch (const cv::Exception& e) {
            qDebug() << "Exception writing frame:" << e.what();
          }
        }
      } else {
        if (recorded_frame_count_ < 5) {  // Only log first few times to avoid spam
          qDebug() << "Warning: frame_to_write invalid - empty:" << frame_to_write.empty()
                   << "data:" << (frame_to_write.data != nullptr)
                   << "size:" << frame_to_write.cols << "x" << frame_to_write.rows
                   << "input channels:" << frame.channels();
        }
      }
      
      // Check if 30 seconds have elapsed
      if (recording_timer_.elapsed() >= RECORDING_DURATION_MS) {
        StopRecording();
      }
    }
    
    QImage img = MatToQImage(frame);
    if (!img.isNull()) {
      image_label_->setPixmap(
          QPixmap::fromImage(img).scaled(image_label_->size(),
                                         Qt::KeepAspectRatio,
                                         Qt::SmoothTransformation));
    }

    int w = frame.cols;
    int h = frame.rows;
    QString status = detection_enabled_ ? " [Detection ON]" : "";
    if (backend_ == Backend::MVSDK) {
      info_label_->setText(
          QString("Res: %1x%2  FPS: Read %3 / Display %4%5")
              .arg(w)
              .arg(h)
              .arg(reading_fps_, 0, 'f', 1)
              .arg(fps_, 0, 'f', 1)
              .arg(status));
    } else {
      info_label_->setText(
          QString("Res: %1x%2  FPS: %3%4")
              .arg(w)
              .arg(h)
              .arg(fps_, 0, 'f', 1)
              .arg(status));
    }
    
    // Tag info label is updated in the detection code above (if detection is enabled)
    // Old shared memory code removed - now using direct detection in GUI
    // Removed old shared memory frame writing code - not needed for direct detection
    /*
      if (backend_ == Backend::MVSDK && frame_shm_ != nullptr && !frame.empty()) {
        // Convert to grayscale if needed
        Mat gray_frame;
        if (frame.channels() == 1) {
          gray_frame = frame;
        } else if (frame.channels() == 3) {
          cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
        } else if (frame.channels() == 2) {
          cvtColor(frame, gray_frame, cv::COLOR_YUV2GRAY_YUY2);
        } else {
          gray_frame = frame;  // Fallback
        }
        
        // Check frame size
        if (gray_frame.cols > 0 && gray_frame.rows > 0 &&
            gray_frame.cols <= static_cast<int>(FrameIPCData::MAX_WIDTH) &&
            gray_frame.rows <= static_cast<int>(FrameIPCData::MAX_HEIGHT)) {
          frame_shm_->width = gray_frame.cols;
          frame_shm_->height = gray_frame.rows;
          frame_shm_->channels = 1;
          frame_shm_->frame_id = ++frame_id_counter_;
          frame_shm_->timestamp = std::chrono::duration<double>(
              std::chrono::steady_clock::now().time_since_epoch()).count();
          
          // Copy frame data (ensure it's contiguous)
          if (gray_frame.isContinuous()) {
            memcpy(frame_shm_->data, gray_frame.data, gray_frame.rows * gray_frame.cols);
          } else {
            // Copy row by row if not contiguous
            for (int y = 0; y < gray_frame.rows; y++) {
              memcpy(frame_shm_->data + y * gray_frame.cols,
                     gray_frame.ptr(y), gray_frame.cols);
            }
          }
        }
      }
    }
    */
  }

  QComboBox *backend_combo_;
  QComboBox *camera_combo_;
  QComboBox *mode_combo_;
  QLabel *info_label_;
  QLabel *image_label_;
  QSlider *exposure_slider_;
  QSlider *brightness_slider_;
  QSlider *gain_slider_;
  QSlider *contrast_slider_;
  QSlider *saturation_slider_;
  QSlider *sharpness_slider_;
  QPushButton *save_settings_button_;
  QPushButton *record_button_;
  QCheckBox *fisheye_undistort_checkbox_;
  QTimer *timer_;
  
  // Video recording
  cv::VideoWriter video_writer_;
  bool is_recording_ = false;
  QElapsedTimer recording_timer_;
  QString current_video_path_;  // Store the current recording path
  int recorded_frame_count_ = 0;  // Track how many frames were written
  static const int RECORDING_DURATION_MS = 30000;  // 30 seconds

  Backend backend_;
  std::vector<CameraInfo> devices_;
  std::vector<CameraInfo> ocv_devices_;
  std::vector<tSdkCameraDevInfo> mv_devices_;
  std::vector<Mode> ocv_modes_;
  std::vector<MVMode> mv_modes_;

  cv::VideoCapture cap_;
  CameraHandle mv_handle_;
  tSdkCameraCapbility mv_capability_;
  unsigned char *mv_rgb_buffer_;

  cv::Mat last_frame_;
  bool has_last_frame_ = false;

  // Threaded frame reading for MindVision cameras
  struct FrameData {
    cv::Mat frame;
    std::chrono::steady_clock::time_point timestamp;
    double reading_fps;
  };
  
  template<typename T>
  class ThreadSafeQueue {
  public:
    void push(const T& item) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= MAX_SIZE) {
        queue_.pop();  // Remove oldest if full
      }
      queue_.push(item);
      condition_.notify_one();
    }
    
    bool try_pop(T& item, int timeout_ms = 0) {
      std::unique_lock<std::mutex> lock(mutex_);
      if (timeout_ms > 0) {
        if (!condition_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                 [this] { return !queue_.empty(); })) {
          return false;
        }
      } else if (queue_.empty()) {
        return false;
      }
      item = queue_.front();
      queue_.pop();
      return true;
    }
    
    size_t size() const {
      std::lock_guard<std::mutex> lock(mutex_);
      return queue_.size();
    }
    
    void clear() {
      std::lock_guard<std::mutex> lock(mutex_);
      while (!queue_.empty()) {
        queue_.pop();
      }
    }
  private:
    mutable std::mutex mutex_;
    std::queue<T> queue_;
    std::condition_variable condition_;
    static const size_t MAX_SIZE = 3;
  };
  
  ThreadSafeQueue<FrameData> frame_queue_;
  std::atomic<bool> reader_running_{false};
  std::thread reader_thread_;
  double reading_fps_ = 0.0;

  QElapsedTimer fps_timer_;
  int frame_count_;
  double fps_;

  bool detection_enabled_;
  QProcess *detection_process_;
  
  // AprilTag detector (for simple debugging - using apriltag library directly)
  apriltag_detector_t* apriltag_detector_;
  apriltag_family_t* apriltag_family_;
  
  // Shared memory for tag detections
  TagDetectionData* tag_shm_;
  int tag_shm_fd_;
  // Shared memory for frames (GUI -> Detection process)
  FrameIPCData* frame_shm_;
  int frame_shm_fd_;
  uint32_t frame_id_counter_;
  // Shared memory for annotated frames (Detection process -> GUI)
  AnnotatedFrameIPCData* annotated_frame_shm_;
  int annotated_frame_shm_fd_;
  uint32_t last_annotated_frame_id_;
  QLabel* tag_info_label_;
  
  // Fisheye undistortion
  cv::Mat fisheye_map1_, fisheye_map2_;
  cv::Mat fisheye_K_, fisheye_D_;
  bool fisheye_undistort_enabled_ = false;
  cv::Size fisheye_image_size_;
  
  // Saved settings
  int saved_camera_index_ = -1;
  int saved_mode_index_ = -1;
  int saved_exposure_ = -1;
  int saved_brightness_ = -1;
  int saved_gain_ = -1;
  int saved_contrast_ = -1;
  int saved_saturation_ = -1;
  int saved_sharpness_ = -1;
  
  // Settings save/load methods
  QString FindConfigFile();
  void SaveSettings();
  void LoadSettings();
  void ApplySavedCameraSettings();
  
  // Fisheye undistortion methods
  bool LoadFisheyeCalibration(const QString& calib_path);
  void InitFisheyeUndistortMaps(const cv::Size& image_size);
  cv::Mat UndistortFrame(const cv::Mat& frame);
  
  // Video recording methods
  void StartRecording();
  void StopRecording();
  QString GetVideoOutputPath();
};

QString CameraWindow::FindConfigFile() {
  // Find config.txt in project root (same directory as where video_visualize_fixed runs)
  QString app_dir = QCoreApplication::applicationDirPath();
  QStringList possible_paths = {
    app_dir + "/../../config.txt",  // From build directory: build -> src -> project root
    app_dir + "/../../../config.txt",  // If deeper
    QDir::currentPath() + "/config.txt",  // Current working directory
    QDir::homePath() + "/Apriltag/StandAlone/config.txt"  // Absolute path
  };
  
  for (const QString& path : possible_paths) {
    QFileInfo info(path);
    if (info.exists() && info.isFile()) {
      return path;
    }
  }
  
  // If not found, use project root relative to build directory
  return app_dir + "/../../config.txt";
}

void CameraWindow::SaveSettings() {
  QString config_path = FindConfigFile();
  
  // Read existing config.txt to preserve all non-GUI settings
  QStringList existing_lines;
  QSet<QString> gui_sections = {"window", "camera_selection", "camera_controls"};
  bool in_gui_section = false;
  QString current_section;
  
  QFile read_file(config_path);
  if (read_file.exists() && read_file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QTextStream in(&read_file);
    while (!in.atEnd()) {
      QString line = in.readLine();
      QString trimmed = line.trimmed();
      
      // Check if this is a section header
      if (trimmed.startsWith("[") && trimmed.endsWith("]")) {
        current_section = trimmed.mid(1, trimmed.length() - 2);
        in_gui_section = gui_sections.contains(current_section);
        if (!in_gui_section) {
          existing_lines.append(line);
        }
        continue;
      }
      
      // Skip lines in GUI sections
      if (!in_gui_section) {
        existing_lines.append(line);
      }
    }
    read_file.close();
  }
  
  // Write updated config.txt
  QFile file(config_path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, "Save Settings",
                         QString("Failed to save settings to %1").arg(config_path));
    return;
  }
  
  QTextStream out(&file);
  
  // Write all existing (non-GUI) lines
  for (const QString& line : existing_lines) {
    out << line << "\n";
  }
  
  // Append GUI sections at the end
  out << "\n# GUI Settings\n";
  out << "[window]\n";
  out << "width: " << width() << "\n";
  out << "height: " << height() << "\n";
  out << "x: " << pos().x() << "\n";
  out << "y: " << pos().y() << "\n\n";
  
  out << "[camera_selection]\n";
  out << "backend: " << (backend_ == Backend::OpenCV ? "opencv" : "mvsdk") << "\n";
  if (!devices_.empty() && camera_combo_->currentIndex() >= 0) {
    out << "camera_index: " << camera_combo_->currentIndex() << "\n";
  }
  if (mode_combo_->currentIndex() >= 0) {
    out << "mode_index: " << mode_combo_->currentIndex() << "\n";
  }
  out << "\n";
  
  out << "[camera_controls]\n";
  out << "exposure: " << exposure_slider_->value() << "\n";
  out << "brightness: " << brightness_slider_->value() << "\n";
  out << "gain: " << gain_slider_->value() << "\n";
  out << "contrast: " << contrast_slider_->value() << "\n";
  out << "saturation: " << saturation_slider_->value() << "\n";
  out << "sharpness: " << sharpness_slider_->value() << "\n";
  
  file.close();
  QMessageBox::information(this, "Settings Saved",
                           QString("Settings saved to %1").arg(config_path));
}

void CameraWindow::LoadSettings() {
  QString config_path = FindConfigFile();
  QFile file(config_path);
  if (!file.exists() || !file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;  // No saved settings, use defaults
  }
  
  QTextStream in(&file);
  QMap<QString, QString> settings;
  QString current_section;
  
  while (!in.atEnd()) {
    QString line = in.readLine().trimmed();
    if (line.isEmpty() || line.startsWith("#")) {
      continue;
    }
    if (line.startsWith("[") && line.endsWith("]")) {
      current_section = line.mid(1, line.length() - 2);
      continue;
    }
    
    int colon_pos = line.indexOf(":");
    if (colon_pos > 0) {
      QString key = line.left(colon_pos).trimmed();
      QString value = line.mid(colon_pos + 1).trimmed();
      settings[current_section + "." + key] = value;
    }
  }
  file.close();
  
  // Apply window settings
  if (settings.contains("window.width") && settings.contains("window.height")) {
    int w = settings["window.width"].toInt();
    int h = settings["window.height"].toInt();
    if (w > 0 && h > 0) {
      resize(w, h);
    }
  }
  if (settings.contains("window.x") && settings.contains("window.y")) {
    int x = settings["window.x"].toInt();
    int y = settings["window.y"].toInt();
    move(x, y);
  }
  
  // Apply camera selection (will be applied after cameras are enumerated)
  if (settings.contains("camera_selection.backend")) {
    QString backend_str = settings["camera_selection.backend"];
    if (backend_str == "mvsdk") {
      backend_combo_->setCurrentIndex(1);  // MVSDK
    } else {
      backend_combo_->setCurrentIndex(0);  // OpenCV
    }
  }
  
  // Store camera selection indices to apply after enumeration
  saved_camera_index_ = -1;
  saved_mode_index_ = -1;
  if (settings.contains("camera_selection.camera_index")) {
    saved_camera_index_ = settings["camera_selection.camera_index"].toInt();
  }
  if (settings.contains("camera_selection.mode_index")) {
    saved_mode_index_ = settings["camera_selection.mode_index"].toInt();
  }
  
  // Store camera control settings to apply when camera opens
  saved_exposure_ = -1;
  saved_brightness_ = -1;
  saved_gain_ = -1;
  saved_contrast_ = -1;
  saved_saturation_ = -1;
  saved_sharpness_ = -1;
  
  if (settings.contains("camera_controls.exposure")) {
    saved_exposure_ = settings["camera_controls.exposure"].toInt();
  }
  if (settings.contains("camera_controls.brightness")) {
    saved_brightness_ = settings["camera_controls.brightness"].toInt();
  }
  if (settings.contains("camera_controls.gain")) {
    saved_gain_ = settings["camera_controls.gain"].toInt();
  }
  if (settings.contains("camera_controls.contrast")) {
    saved_contrast_ = settings["camera_controls.contrast"].toInt();
  }
  if (settings.contains("camera_controls.saturation")) {
    saved_saturation_ = settings["camera_controls.saturation"].toInt();
  }
  if (settings.contains("camera_controls.sharpness")) {
    saved_sharpness_ = settings["camera_controls.sharpness"].toInt();
  }
}

void CameraWindow::ApplySavedCameraSettings() {
  if (saved_exposure_ >= 0) {
    exposure_slider_->blockSignals(true);
    exposure_slider_->setValue(saved_exposure_);
    exposure_slider_->blockSignals(false);
    // Trigger the valueChanged signal manually
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      double v = static_cast<double>(saved_exposure_);
      v = (v - 50.0) / 10.0;
      cap_.set(CAP_PROP_EXPOSURE, v);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      double min_exposure = 1000.0;
      double max_exposure = 100000.0;
      double exposure = max_exposure - (saved_exposure_ / 100.0) * (max_exposure - min_exposure);
      CameraSetExposureTime(mv_handle_, exposure);
    }
  }
  
  if (saved_brightness_ >= 0) {
    brightness_slider_->blockSignals(true);
    brightness_slider_->setValue(saved_brightness_);
    brightness_slider_->blockSignals(false);
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      cap_.set(CAP_PROP_BRIGHTNESS, saved_brightness_);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      int analog_gain = (saved_brightness_ * 100) / 255;
      CameraSetAnalogGain(mv_handle_, analog_gain);
    }
  }
  
  if (saved_gain_ >= 0) {
    gain_slider_->blockSignals(true);
    gain_slider_->setValue(saved_gain_);
    gain_slider_->blockSignals(false);
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      cap_.set(CAP_PROP_GAIN, saved_gain_);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      CameraSetGain(mv_handle_, saved_gain_, saved_gain_, saved_gain_);
    }
  }
  
  if (saved_contrast_ >= 0) {
    contrast_slider_->blockSignals(true);
    contrast_slider_->setValue(saved_contrast_);
    contrast_slider_->blockSignals(false);
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      cap_.set(CAP_PROP_CONTRAST, saved_contrast_);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      CameraSetContrast(mv_handle_, saved_contrast_);
    }
  }
  
  if (saved_saturation_ >= 0) {
    saturation_slider_->blockSignals(true);
    saturation_slider_->setValue(saved_saturation_);
    saturation_slider_->blockSignals(false);
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      cap_.set(CAP_PROP_SATURATION, saved_saturation_);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      CameraSetSaturation(mv_handle_, saved_saturation_);
    }
  }
  
  if (saved_sharpness_ >= 0) {
    sharpness_slider_->blockSignals(true);
    sharpness_slider_->setValue(saved_sharpness_);
    sharpness_slider_->blockSignals(false);
    if (backend_ == Backend::OpenCV && cap_.isOpened()) {
      cap_.set(CAP_PROP_SHARPNESS, saved_sharpness_);
    } else if (backend_ == Backend::MVSDK && mv_handle_ != 0) {
      CameraSetSharpness(mv_handle_, saved_sharpness_);
    }
  }
}

bool CameraWindow::LoadFisheyeCalibration(const QString& calib_path) {
  QFileInfo file_info(calib_path);
  if (!file_info.exists()) {
    qDebug() << "Fisheye calibration file not found:" << calib_path;
    return false;
  }
  
  std::string path = calib_path.toStdString();
  cv::FileStorage fs(path, cv::FileStorage::READ);
  
  if (!fs.isOpened()) {
    qDebug() << "Failed to open fisheye calibration file:" << calib_path;
    return false;
  }
  
  fs["camera_matrix"] >> fisheye_K_;
  fs["distortion_coefficients"] >> fisheye_D_;
  
  int img_width = 0, img_height = 0;
  if (fs["image_width"].isInt()) {
    img_width = static_cast<int>(fs["image_width"]);
  }
  if (fs["image_height"].isInt()) {
    img_height = static_cast<int>(fs["image_height"]);
  }
  
  fs.release();
  
  if (fisheye_K_.empty() || fisheye_D_.empty()) {
    qDebug() << "Failed to read camera matrix or distortion coefficients";
    return false;
  }
  
  fisheye_image_size_ = cv::Size(img_width, img_height);
  fisheye_undistort_enabled_ = true;  // Enabled by default if calibration loaded
  
  // Update checkbox state
  if (fisheye_undistort_checkbox_) {
    fisheye_undistort_checkbox_->setChecked(true);
  }
  
  qDebug() << "Fisheye calibration loaded successfully";
  qDebug() << "Camera matrix K rows:" << fisheye_K_.rows << "cols:" << fisheye_K_.cols;
  qDebug() << "Distortion coefficients D rows:" << fisheye_D_.rows << "cols:" << fisheye_D_.cols;
  qDebug() << "Image size:" << fisheye_image_size_.width << "x" << fisheye_image_size_.height;
  
  return true;
}

void CameraWindow::InitFisheyeUndistortMaps(const cv::Size& image_size) {
  if (!fisheye_undistort_enabled_ || fisheye_K_.empty() || fisheye_D_.empty()) {
    return;
  }
  
  // Check if maps are already initialized for this size
  if (!fisheye_map1_.empty() && fisheye_map1_.size() == image_size) {
    return;
  }
  
  // Initialize undistortion maps
  cv::fisheye::initUndistortRectifyMap(
    fisheye_K_,
    fisheye_D_,
    cv::Mat::eye(3, 3, CV_32F),  // Identity matrix (no rotation)
    fisheye_K_,  // Use same camera matrix for output
    image_size,
    CV_16SC2,  // map1 type
    fisheye_map1_,
    fisheye_map2_
  );
  
  qDebug() << "Fisheye undistortion maps initialized for size:" << image_size.width << "x" << image_size.height;
}

cv::Mat CameraWindow::UndistortFrame(const cv::Mat& frame) {
  if (!fisheye_undistort_enabled_ || fisheye_map1_.empty() || fisheye_map2_.empty()) {
    return frame;  // Return original frame if undistortion not enabled
  }
  
  // Initialize maps if needed (first frame or size changed)
  if (fisheye_map1_.size() != frame.size()) {
    InitFisheyeUndistortMaps(frame.size());
  }
  
  cv::Mat undistorted;
  // Use INTER_CUBIC for better quality (preserves edges better than LINEAR)
  // This helps maintain contrast needed for AprilTag detection
  cv::remap(frame, undistorted, fisheye_map1_, fisheye_map2_, cv::INTER_CUBIC, cv::BORDER_CONSTANT);
  
  return undistorted;
}

QString CameraWindow::GetVideoOutputPath() {
  // Find input directory (same directory as Stable.avi)
  QString app_dir = QCoreApplication::applicationDirPath();
  QStringList possible_paths = {
    app_dir + "/../../input",  // From build directory: build -> src -> project root -> input
    QDir::currentPath() + "/input",  // Current working directory
    QDir::homePath() + "/Apriltag/StandAlone/input"  // Absolute path
  };
  
  for (const QString& path : possible_paths) {
    QFileInfo info(path);
    if (info.exists() && info.isDir()) {
      // Generate filename with timestamp - use .avi with MJPG codec (most compatible)
      QDateTime now = QDateTime::currentDateTime();
      QString filename = QString("recorded_%1.avi").arg(now.toString("yyyyMMdd_hhmmss"));
      return path + "/" + filename;
    }
  }
  
  // Fallback to current directory
  QDateTime now = QDateTime::currentDateTime();
  QString filename = QString("recorded_%1.avi").arg(now.toString("yyyyMMdd_hhmmss"));
  return QDir::currentPath() + "/" + filename;
}

void CameraWindow::StartRecording() {
  if (is_recording_) {
    return;  // Already recording
  }
  
  if (last_frame_.empty() || !has_last_frame_) {
    QMessageBox::warning(this, "Recording Error", "No frame available to record. Please start the camera first.");
    return;
  }
  
  QString video_path = GetVideoOutputPath();
  double fps = fps_ > 0 ? fps_ : 30.0;  // Use current FPS or default to 30
  cv::Size frame_size(last_frame_.cols, last_frame_.rows);
  
  if (frame_size.width <= 0 || frame_size.height <= 0) {
    QMessageBox::warning(this, "Recording Error", "Invalid frame size. Please ensure camera is running.");
    return;
  }
  
  qDebug() << "Attempting to open video writer:";
  qDebug() << "  Path:" << video_path;
  qDebug() << "  Size:" << frame_size.width << "x" << frame_size.height;
  qDebug() << "  FPS:" << fps;
  
  // Use MJPG codec with .avi (most compatible, works without FFmpeg)
  bool opened = false;
  QString codec_used = "MJPG";
  int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  
  qDebug() << "Opening video writer with MJPG codec...";
  video_writer_.open(video_path.toStdString(), fourcc, fps, frame_size, true);
  opened = video_writer_.isOpened();
  
  if (!opened) {
    // Try XVID as fallback
    codec_used = "XVID";
    fourcc = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
    qDebug() << "MJPG failed, trying XVID...";
    video_writer_.open(video_path.toStdString(), fourcc, fps, frame_size, true);
    opened = video_writer_.isOpened();
  }
  
  if (!opened) {
    QMessageBox::warning(this, "Recording Error", 
                         QString("Failed to open video writer for: %1\nTried H264, MJPG, XVID, and MP4V codecs.").arg(video_path));
    return;
  }
  
  qDebug() << "Video writer opened successfully with codec:" << codec_used;
  qDebug() << "Video writer initialized with - FPS:" << fps << "Size:" << frame_size.width << "x" << frame_size.height;
  
  // Test write a frame immediately to verify it works
  if (!last_frame_.empty()) {
    cv::Mat test_frame;
    if (last_frame_.channels() == 1) {
      cv::cvtColor(last_frame_, test_frame, cv::COLOR_GRAY2BGR);
    } else {
      test_frame = last_frame_.clone();
    }
    if (test_frame.type() != CV_8UC3) {
      test_frame.convertTo(test_frame, CV_8UC3);
    }
    video_writer_ << test_frame;
    qDebug() << "Test frame written to verify video writer works";
  }
  
  is_recording_ = true;
  current_video_path_ = video_path;  // Store the path
  recorded_frame_count_ = 0;  // Reset frame counter
  recording_timer_.start();
  record_button_->setText("Recording...");
  record_button_->setEnabled(false);  // Disable button during recording
  
  qDebug() << "Started recording to:" << video_path;
  qDebug() << "Resolution:" << frame_size.width << "x" << frame_size.height << "FPS:" << fps;
  qDebug() << "VideoWriter isOpened:" << video_writer_.isOpened();
}

void CameraWindow::StopRecording() {
  if (!is_recording_) {
    return;
  }
  
  if (video_writer_.isOpened()) {
    video_writer_.release();
    qDebug() << "Video writer released";
  }
  
  is_recording_ = false;
  record_button_->setText("Record 30s Video");
  record_button_->setEnabled(true);
  
  qDebug() << "Recording stopped";
  qDebug() << "Total frames recorded:" << recorded_frame_count_;
  qDebug() << "Recording duration:" << recording_timer_.elapsed() << "ms";
  
  // Get the file path to check file size (use stored path)
  QString video_path = current_video_path_;
  QFileInfo file_info(video_path);
  if (file_info.exists()) {
    qint64 file_size = file_info.size();
    qDebug() << "Recorded video file size:" << file_size << "bytes (" << (file_size / 1024.0) << "KB)";
    
    QString message = QString("30-second video recording completed.\n\nSaved to: %1\nFile size: %2 KB\nFrames recorded: %3")
                      .arg(video_path)
                      .arg(file_size / 1024.0, 0, 'f', 2)
                      .arg(recorded_frame_count_);
    
    if (file_size < 100000) {  // Less than 100KB is suspicious
      message += "\n\nWARNING: File size seems too small. Recording may have failed.";
    }
    
    QMessageBox::information(this, "Recording Complete", message);
  } else {
    QMessageBox::warning(this, "Recording Warning", 
                         QString("Recording stopped but video file was not found at:\n%1").arg(video_path));
  }
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);

  CameraWindow window;
  window.setWindowState(Qt::WindowNoState);  // Ensure not minimized
  
  // Set a reasonable window size
  window.resize(1200, 800);
  window.setMinimumSize(800, 600);
  
  window.show();
  window.raise();
  window.activateWindow();
  window.setFocus();  // Give focus to the window
  
  // Print window size for debugging
  QSize size = window.size();
  qDebug() << "Window size:" << size.width() << "x" << size.height();
  qDebug() << "Window position:" << window.pos().x() << "," << window.pos().y();

  return app.exec();
}
