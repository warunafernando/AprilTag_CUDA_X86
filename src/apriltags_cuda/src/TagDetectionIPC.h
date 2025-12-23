#ifndef TAG_DETECTION_IPC_H
#define TAG_DETECTION_IPC_H

#include <vector>
#include <string>
#include <cstdint>
#include <cstring>

// Simple IPC structure for tag detection data
// Format: [timestamp(double), num_tags(uint32_t), tag_data...]
// Each tag: [id(uint32_t), x(double), y(double), z(double), margin(double)]
struct TagDetectionData {
  double timestamp;
  uint32_t num_tags;
  struct TagInfo {
    uint32_t id;
    double x, y, z;
    double decision_margin;
  } tags[32];  // Max 32 tags per frame
  
  static constexpr size_t MAX_TAGS = 32;
  
  void clear() {
    timestamp = 0.0;
    num_tags = 0;
    memset(tags, 0, sizeof(tags));
  }
  
  bool addTag(uint32_t id, double x, double y, double z, double margin) {
    if (num_tags >= MAX_TAGS) return false;
    tags[num_tags].id = id;
    tags[num_tags].x = x;
    tags[num_tags].y = y;
    tags[num_tags].z = z;
    tags[num_tags].decision_margin = margin;
    num_tags++;
    return true;
  }
};

// Shared memory structure for frame data (GUI -> Detection process)
struct FrameIPCData {
  uint32_t width;
  uint32_t height;
  uint32_t channels;
  uint32_t frame_id;
  double timestamp;
  uint8_t data[1920 * 1080];  // Max 1920x1080 grayscale frame
  static constexpr size_t MAX_WIDTH = 1920;
  static constexpr size_t MAX_HEIGHT = 1080;
  static constexpr size_t MAX_DATA_SIZE = MAX_WIDTH * MAX_HEIGHT;
};

// Shared memory structure for annotated frames (Detection process -> GUI)
struct AnnotatedFrameIPCData {
  uint32_t width;
  uint32_t height;
  uint32_t channels;  // Should be 3 for BGR
  uint32_t frame_id;
  double timestamp;
  uint8_t data[1920 * 1080 * 3];  // Max 1920x1080 BGR frame
  static constexpr size_t MAX_WIDTH = 1920;
  static constexpr size_t MAX_HEIGHT = 1080;
  static constexpr size_t MAX_DATA_SIZE = MAX_WIDTH * MAX_HEIGHT * 3;
};

// Shared memory key for tag detection data
inline const char* TAG_IPC_KEY = "/apriltag_detections_shm";
// Shared memory key for frame data (GUI -> Detection process)
inline const char* FRAME_IPC_KEY = "/apriltag_frames_shm";
// Shared memory key for annotated frames (Detection process -> GUI)
inline const char* ANNOTATED_FRAME_IPC_KEY = "/apriltag_annotated_frames_shm";

#endif

