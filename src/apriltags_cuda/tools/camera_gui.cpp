#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

struct DropdownRegion {
  int start_y = 0;
  int line_height = 0;
  int count = 0;
};

static DropdownRegion g_cameraRegion;
static DropdownRegion g_modeRegion;
static int g_clickedCamera = -1;
static int g_clickedMode = -1;

static void OnMouse(int event, int /*x*/, int y, int /*flags*/, void* /*userdata*/) {
  if (event != EVENT_LBUTTONDOWN) {
    return;
  }
  if (g_cameraRegion.count > 0 &&
      y >= g_cameraRegion.start_y &&
      y < g_cameraRegion.start_y + g_cameraRegion.line_height * g_cameraRegion.count) {
    int idx = (y - g_cameraRegion.start_y) / g_cameraRegion.line_height;
    if (idx >= 0 && idx < g_cameraRegion.count) {
      g_clickedCamera = idx;
      return;
    }
  }
  if (g_modeRegion.count > 0 &&
      y >= g_modeRegion.start_y &&
      y < g_modeRegion.start_y + g_modeRegion.line_height * g_modeRegion.count) {
    int idx = (y - g_modeRegion.start_y) / g_modeRegion.line_height;
    if (idx >= 0 && idx < g_modeRegion.count) {
      g_clickedMode = idx;
      return;
    }
  }
}

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

// Try to read a human-readable name for a V4L2 device.
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

static std::vector<CameraInfo> EnumerateCameras() {
  std::vector<CameraInfo> result;

  // Probe a small range of indices. This is fast in practice and good enough
  // for a "small GUI" helper.
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

int main() {
  std::vector<CameraInfo> all = EnumerateCameras();
  if (all.empty()) {
    std::cerr << "No video devices found (0-9).\n";
    return 1;
  }

  // Show all detected cameras (integrated + USB). We still mark which ones
  // look like webcams, but we no longer filter the list so newly attached
  // USB cameras that don't include "webcam" in their name are visible.
  const std::vector<CameraInfo> &devices = all;

  std::cout << "Discovered cameras:\n";
  for (size_t i = 0; i < devices.size(); ++i) {
    std::cout << "  [" << i << "] index=" << devices[i].index
              << " name=\"" << devices[i].name << "\""
              << (devices[i].is_webcam ? " (webcam)\n" : "\n");
  }
  std::cout << "\nUse the \"Camera\" trackbar to select a device.\n"
            << "Adjust Exposure / Brightness with trackbars.\n"
            << "Press 'q' or ESC to quit.\n";

  int selected = 0;
  int max_index = static_cast<int>(devices.size()) - 1;
  bool has_camera_trackbar = false;

  namedWindow("Camera Selector", WINDOW_NORMAL);
  resizeWindow("Camera Selector", 960, 720);
  setMouseCallback("Camera Selector", OnMouse, nullptr);

  if (max_index > 0) {
    // Let OpenCV own the trackbar value; we will query it via getTrackbarPos.
    createTrackbar("Camera", "Camera Selector", nullptr, max_index);
    has_camera_trackbar = true;
  } else {
    selected = 0;
  }

  // Common resolution/FPS presets. Not all cameras support every mode; we
  // attempt to set them and then display whatever the driver reports.
  struct Mode {
    int width;
    int height;
    double fps;
    std::string label;
  };
  std::vector<Mode> modes = {
      {640, 480, 30.0, "640x480 @30"},
      {1280, 720, 30.0, "1280x720 @30"},
      {1920, 1080, 30.0, "1920x1080 @30"},
      {640, 480, 60.0, "640x480 @60"},
      {1024, 1024, 212.0, "1024x1024 @212"},
  };
  int max_mode = static_cast<int>(modes.size()) - 1;
  int selected_mode = 0;
  if (max_mode > 0) {
    createTrackbar("Mode", "Camera Selector", nullptr, max_mode);
  }

  // Basic controls exposed as trackbars. These map to common V4L2 properties
  // and may be ignored by some drivers, but provide an easy way to tweak
  // exposure, brightness, gain, etc.
  int exposure_slider = 50;   // mapped to a small range around 0
  int brightness_slider = 128;
  int gain_slider = 50;
  int contrast_slider = 50;
  int saturation_slider = 50;
  int sharpness_slider = 50;

  createTrackbar("Exposure", "Camera Selector", &exposure_slider, 100);
  createTrackbar("Brightness", "Camera Selector", &brightness_slider, 255);
  createTrackbar("Gain", "Camera Selector", &gain_slider, 100);
  createTrackbar("Contrast", "Camera Selector", &contrast_slider, 100);
  createTrackbar("Saturation", "Camera Selector", &saturation_slider, 100);
  createTrackbar("Sharpness", "Camera Selector", &sharpness_slider, 100);

  VideoCapture cap;
  auto apply_mode = [&](int mode_index) {
    if (!cap.isOpened()) return;
    if (mode_index < 0 || mode_index >= static_cast<int>(modes.size())) return;
    const Mode &m = modes[mode_index];
    cap.set(CAP_PROP_FRAME_WIDTH, m.width);
    cap.set(CAP_PROP_FRAME_HEIGHT, m.height);
    cap.set(CAP_PROP_FPS, m.fps);
  };

  auto open_camera = [&]() {
    if (cap.isOpened()) {
      cap.release();
    }
    int real_index = devices[selected].index;
    std::cout << "Opening camera [" << selected << "] index=" << real_index
              << " name=\"" << devices[selected].name << "\"\n";
    cap.open(real_index, CAP_ANY);
    if (!cap.isOpened()) {
      std::cerr << "Failed to open camera index " << real_index << "\n";
    } else {
      cap.set(CAP_PROP_CONVERT_RGB, false);
      apply_mode(selected_mode);
    }
  };

  open_camera();

  auto start = std::chrono::steady_clock::now();
  auto fps_window_start = start;
  int fps_frame_count = 0;
  double fps = 0.0;
  while (true) {
    // Handle mouse-based "dropdown" selection for cameras and modes.
    if (g_clickedCamera != -1) {
      int idx = g_clickedCamera;
      g_clickedCamera = -1;
      if (idx >= 0 && idx <= max_index) {
        selected = idx;
        open_camera();
        if (has_camera_trackbar) {
          setTrackbarPos("Camera", "Camera Selector", selected);
        }
      }
    }

    if (g_clickedMode != -1) {
      int idx = g_clickedMode;
      g_clickedMode = -1;
      if (idx >= 0 && idx <= max_mode) {
        selected_mode = idx;
        if (cap.isOpened()) {
          apply_mode(selected_mode);
        }
        if (max_mode > 0) {
          setTrackbarPos("Mode", "Camera Selector", selected_mode);
        }
      }
    }

    if (has_camera_trackbar) {
      int pos = getTrackbarPos("Camera", "Camera Selector");
      if (pos != selected && pos >= 0 && pos <= max_index) {
        selected = pos;
        open_camera();
      }
    }

    if (max_mode > 0 && cap.isOpened()) {
      int mode_pos = getTrackbarPos("Mode", "Camera Selector");
      if (mode_pos != selected_mode && mode_pos >= 0 &&
          mode_pos <= max_mode) {
        selected_mode = mode_pos;
        apply_mode(selected_mode);
      }
    }

    Mat frame;
    if (cap.isOpened()) {
      // Apply exposure/brightness and other controls from sliders
      // (best-effort; may be ignored by some drivers).
      double exposure = (static_cast<double>(exposure_slider) - 50.0) / 10.0;
      double brightness = static_cast<double>(brightness_slider);
      double gain = static_cast<double>(gain_slider);
      double contrast = static_cast<double>(contrast_slider);
      double saturation = static_cast<double>(saturation_slider);
      double sharpness = static_cast<double>(sharpness_slider);

      cap.set(CAP_PROP_EXPOSURE, exposure);
      cap.set(CAP_PROP_BRIGHTNESS, brightness);
      cap.set(CAP_PROP_GAIN, gain);
      cap.set(CAP_PROP_CONTRAST, contrast);
      cap.set(CAP_PROP_SATURATION, saturation);
      cap.set(CAP_PROP_SHARPNESS, sharpness);

      cap >> frame;

      // Update FPS estimate.
      ++fps_frame_count;
      auto now = std::chrono::steady_clock::now();
      double window_sec =
          std::chrono::duration<double>(now - fps_window_start).count();
      if (window_sec >= 0.5) {
        fps = fps_frame_count / window_sec;
        fps_frame_count = 0;
        fps_window_start = now;
      }
    }

    if (frame.empty()) {
      frame = Mat::zeros(Size(640, 480), CV_8UC1);
      putText(frame, "No frame from selected camera",
              Point(20, frame.rows / 2),
              FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255), 2);
    }

    Mat color;
    if (frame.channels() == 1) {
      cvtColor(frame, color, COLOR_GRAY2BGR);
    } else if (frame.channels() == 2) {
      // Many UVC webcams expose YUYV/YUY2 when CONVERT_RGB is false or on
      // some platforms; convert explicitly to BGR for display.
      cvtColor(frame, color, COLOR_YUV2BGR_YUY2);
    } else if (frame.channels() == 3) {
      color = frame;
    } else if (frame.channels() == 4) {
      cvtColor(frame, color, COLOR_BGRA2BGR);
    } else {
      // Fallback: show as grayscale.
      cvtColor(frame, color, COLOR_GRAY2BGR);
    }

    // Draw a simple "dropdown-like" menu showing all detected cameras and
    // highlight the currently selected one.
    std::string label = "Selected: [" + std::to_string(selected) +
                        "] index=" + std::to_string(devices[selected].index) +
                        " name=" + devices[selected].name;
    putText(color, label, Point(10, 30),
            FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 2);

    // Show current resolution and FPS.
    int w = color.cols;
    int h = color.rows;
    char info_buf[128];
    std::snprintf(info_buf, sizeof(info_buf), "Res: %dx%d  FPS: %.1f", w, h,
                  fps);
    putText(color, info_buf, Point(10, 55),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 1);

    int menu_y = 80;
    putText(color, "Available cameras (click to select):", Point(10, menu_y),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
    menu_y += 20;
    g_cameraRegion.start_y = menu_y;
    g_cameraRegion.line_height = 18;
    g_cameraRegion.count = static_cast<int>(devices.size());
    for (size_t i = 0; i < devices.size(); ++i) {
      Scalar col = (static_cast<int>(i) == selected)
                       ? Scalar(0, 255, 0)
                       : Scalar(200, 200, 200);
      std::string line = "[" + std::to_string(i) +
                         "] idx=" + std::to_string(devices[i].index) + " " +
                         devices[i].name +
                         (devices[i].is_webcam ? " (webcam)" : "");
      putText(color, line, Point(10, menu_y),
              FONT_HERSHEY_SIMPLEX, 0.45, col, 1);
      menu_y += 18;
    }

    // Draw a second "dropdown-style" menu for resolution/FPS presets.
    menu_y += 10;
    putText(color, "Modes (resolution @ FPS) (click to select):",
            Point(10, menu_y),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 1);
    menu_y += 20;
    g_modeRegion.start_y = menu_y;
    g_modeRegion.line_height = 18;
    g_modeRegion.count = static_cast<int>(modes.size());
    for (size_t i = 0; i < modes.size(); ++i) {
      Scalar col = (static_cast<int>(i) == selected_mode)
                       ? Scalar(0, 200, 255)
                       : Scalar(180, 180, 180);
      std::string line = "[" + std::to_string(i) + "] " + modes[i].label;
      putText(color, line, Point(10, menu_y),
              FONT_HERSHEY_SIMPLEX, 0.45, col, 1);
      menu_y += 18;
    }

    imshow("Camera Selector", color);
    int key = waitKey(1);
    if (key == 'q' || key == 27) {
      break;
    }

    // Allow direct "dropdown-like" selection of modes via number keys.
    if (key >= '0' && key <= '0' + max_mode) {
      int num = key - '0';
      if (num >= 0 && num <= max_mode) {
        selected_mode = num;
        if (cap.isOpened()) {
          apply_mode(selected_mode);
        }
        if (max_mode > 0) {
          setTrackbarPos("Mode", "Camera Selector", selected_mode);
        }
      }
    }

  }

  return 0;
}


