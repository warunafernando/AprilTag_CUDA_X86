# Plan: Class-Based Architecture for Fast AprilTag Algorithm

## Overview
Create a class-based architecture to port `video_visualize_fixed.cu` algorithm to the GUI without modifying or combining with existing code. This allows adding more algorithms later in a modular way.

## Architecture Design

### Base Class: `AprilTagAlgorithm`
Abstract base class that all algorithms inherit from.

**Location**: `Tools/apriltag_algorithm.h` (new file)

**Interface**:
```cpp
class AprilTagAlgorithm {
public:
    virtual ~AprilTagAlgorithm() = default;
    
    // Initialize algorithm (called once at start)
    virtual bool initialize(int width, int height) = 0;
    
    // Process a single frame (called for each frame)
    // Input: grayscale frame (CV_8UC1)
    // Output: detections array (zarray_t*)
    virtual zarray_t* processFrame(const cv::Mat& gray_frame, bool mirror) = 0;
    
    // Cleanup (called when stopping)
    virtual void cleanup() = 0;
    
    // Get algorithm name
    virtual std::string getName() const = 0;
    
    // Check if algorithm needs CUDA
    virtual bool requiresCuda() const = 0;
};
```

### Implementation Classes

#### 1. `CpuAprilTagAlgorithm`
Wrapper for existing CPU detection.

**Location**: `Tools/cpu_apriltag_algorithm.h` and `.cu` (new files)

**Purpose**: Encapsulates the existing CPU detection logic

#### 2. `CudaAprilTagAlgorithm`  
Wrapper for existing CUDA detection (current index 1).

**Location**: `Tools/cuda_apriltag_algorithm.h` and `.cu` (new files)

**Purpose**: Encapsulates the existing CUDA detection logic

#### 3. `FastAprilTagAlgorithm`
Complete port of `video_visualize_fixed.cu` algorithm.

**Location**: `Tools/fast_apriltag_algorithm.h` and `.cu` (new files)

**Purpose**: Self-contained implementation of the Fast AprilTag algorithm from `video_visualize_fixed.cu`

## FastAprilTagAlgorithm Implementation

### Header File: `Tools/fast_apriltag_algorithm.h`

```cpp
#ifndef FAST_APRILTAG_ALGORITHM_H
#define FAST_APRILTAG_ALGORITHM_H

#include "apriltag_algorithm.h"
#include "../src/apriltags_cuda/src/apriltag_gpu.h"
#include "apriltag.h"
#include "g2d.h"
#include <vector>
#include <mutex>

// Forward declarations
namespace frc971 {
namespace apriltag {
    void DecodeTagsFromQuads(const std::vector<QuadCorners> &quad_corners,
                             const uint8_t *gray_buf, int width, int height,
                             apriltag_detector_t *td,
                             const CameraMatrix &camera_matrix,
                             const DistCoeffs &distortion_coefficients,
                             zarray_t *detections,
                             zarray_t *poly0,
                             zarray_t *poly1);
}
}

class FastAprilTagAlgorithm : public AprilTagAlgorithm {
public:
    FastAprilTagAlgorithm();
    ~FastAprilTagAlgorithm() override;
    
    bool initialize(int width, int height) override;
    zarray_t* processFrame(const cv::Mat& gray_frame, bool mirror) override;
    void cleanup() override;
    std::string getName() const override { return "Fast AprilTag"; }
    bool requiresCuda() const override { return true; }
    
private:
    // Dimensions
    int width_;
    int height_;
    bool initialized_;
    
    // GPU detector
    frc971::apriltag::GpuDetector* gpu_detector_;
    
    // CPU decode detector
    apriltag_family_t* tf_gpu_;
    apriltag_detector_t* td_gpu_;
    apriltag_detector_t* td_for_gpu_;  // Detector passed to GpuDetector
    
    // Camera calibration
    frc971::apriltag::CameraMatrix gpu_cam_;
    frc971::apriltag::DistCoeffs gpu_dist_;
    
    // Temporary arrays for decode (reused to avoid allocation overhead)
    zarray_t* poly0_;
    zarray_t* poly1_;
    
    // Helper functions from video_visualize_fixed.cu
    void filterDuplicates(zarray_t* detections, int width, int height, double min_distance = 50.0);
    bool isValidDetection(apriltag_detection_t* det, int width, int height);
    void mirrorQuadCoordinates(std::vector<frc971::apriltag::QuadCorners>& quads, int width);
    
    // Load calibration from fisheye calibration data
    void loadCalibration();
};

#endif
```

### Implementation File: `Tools/fast_apriltag_algorithm.cu`

**Key sections to port from `video_visualize_fixed.cu`:**

1. **Initialization** (from `video_visualize_fixed.cu` lines 682-703):
   - Create tag family (`tag36h11_create()`)
   - Create detector with `quad_decimate = 2.0`
   - Set up camera calibration matrices
   - Create `GpuDetector` instance

2. **processFrame()** (from `video_visualize_fixed.cu` lines 1096-1180):
   - Validate frame
   - Call `gpu_detector_->DetectGpuOnly(frame.data)`
   - Get quads: `gpu_detector_->FitQuads()`
   - Handle mirroring (GPU mirror + coordinate mirroring)
   - Copy gray image: `gpu_detector_->CopyGrayHostTo()`
   - Decode: `DecodeTagsFromQuads()`
   - Filter duplicates
   - Return detections

3. **Helper functions** (from `video_visualize_fixed.cu`):
   - `filter_duplicates()` (lines 149-187)
   - `is_valid_detection()` (lines 106-124)
   - Mirror coordinate logic (lines 1163-1180)

## Integration with GUI

### Step 1: Create Algorithm Factory
**Location**: `Tools/apriltag_algorithm_factory.h` (new file)

```cpp
class AprilTagAlgorithmFactory {
public:
    enum AlgorithmType {
        CPU = 0,
        CUDA_GPU = 1,
        FAST_APRILTAG = 2
    };
    
    static std::unique_ptr<AprilTagAlgorithm> create(AlgorithmType type);
};
```

### Step 2: Update GUI to Use Algorithm Classes

**File**: `Tools/apriltag_debug_gui.cu`

**Changes**:
1. Add member variable:
   ```cpp
   std::unique_ptr<AprilTagAlgorithm> currentAlgorithm_;
   ```

2. In `setupAlgorithmsTab()`:
   ```cpp
   algorithmCombo_->addItem("OpenCV CPU (AprilTag)");
   algorithmCombo_->addItem("CUDA GPU (AprilTag)");
   algorithmCombo_->addItem("Fast AprilTag");  // NEW
   ```

3. In `startAlgorithm()`:
   ```cpp
   int algorithmIndex = algorithmCombo_->currentIndex();
   AprilTagAlgorithmFactory::AlgorithmType algoType;
   switch (algorithmIndex) {
       case 0: algoType = AprilTagAlgorithmFactory::CPU; break;
       case 1: algoType = AprilTagAlgorithmFactory::CUDA_GPU; break;
       case 2: algoType = AprilTagAlgorithmFactory::FAST_APRILTAG; break;
       default: return;
   }
   
   // Initialize algorithm
   currentAlgorithm_ = AprilTagAlgorithmFactory::create(algoType);
   if (!currentAlgorithm_) {
       QMessageBox::warning(this, "Error", "Failed to create algorithm");
       return;
   }
   
   // Get frame dimensions (from camera)
   int width = ..., height = ...;
   if (!currentAlgorithm_->initialize(width, height)) {
       QMessageBox::warning(this, "Error", "Failed to initialize algorithm");
       return;
   }
   ```

4. In `detectionThreadFunction()` - Simplify:
   ```cpp
   while (algorithmRunning_) {
       // Get processed frame
       Mat gray = ...;  // From processedFrame_
       
       if (gray.empty()) continue;
       
       // Check if mirroring is enabled
       bool mirror = algorithmMirrorCheckbox_->isChecked();
       
       // Process frame using algorithm
       zarray_t* detections = currentAlgorithm_->processFrame(gray, mirror);
       
       // Draw detections and pass to display thread
       // ... existing drawing code ...
       
       // Clean up detections (algorithm owns them, but we need to destroy)
       if (detections) {
           for (int i = 0; i < zarray_size(detections); i++) {
               apriltag_detection_t* det;
               zarray_get(detections, i, &det);
               apriltag_detection_destroy(det);
           }
           zarray_destroy(detections);
       }
   }
   ```

5. In `stopAlgorithm()`:
   ```cpp
   if (currentAlgorithm_) {
       currentAlgorithm_->cleanup();
       currentAlgorithm_.reset();
   }
   ```

## File Structure

```
Tools/
├── apriltag_debug_gui.cu           (GUI main file - uses algorithms)
├── apriltag_algorithm.h             (NEW - base class)
├── apriltag_algorithm_factory.h     (NEW - factory for creating algorithms)
├── cpu_apriltag_algorithm.h         (NEW - CPU algorithm wrapper)
├── cpu_apriltag_algorithm.cu        (NEW - CPU implementation)
├── cuda_apriltag_algorithm.h        (NEW - CUDA algorithm wrapper)
├── cuda_apriltag_algorithm.cu       (NEW - CUDA implementation)
├── fast_apriltag_algorithm.h        (NEW - Fast AprilTag header)
└── fast_apriltag_algorithm.cu       (NEW - Fast AprilTag implementation)
```

## Implementation Steps

### Phase 1: Create Base Architecture
1. ✅ Create `apriltag_algorithm.h` with base class
2. ✅ Create `apriltag_algorithm_factory.h` with factory
3. ✅ Update CMakeLists.txt to include new files

### Phase 2: Port Existing Algorithms to Classes
1. ✅ Create `CpuAprilTagAlgorithm` class (wrap existing CPU code)
2. ✅ Create `CudaAprilTagAlgorithm` class (wrap existing CUDA code)
3. ✅ Test that existing functionality still works

### Phase 3: Implement Fast AprilTag
1. ✅ Create `fast_apriltag_algorithm.h`
2. ✅ Create `fast_apriltag_algorithm.cu`
3. ✅ Port initialization code from `video_visualize_fixed.cu`
4. ✅ Port `processFrame()` logic from `video_visualize_fixed.cu`
5. ✅ Port helper functions (filter_duplicates, etc.)
6. ✅ Test Fast AprilTag algorithm

### Phase 4: Integrate with GUI
1. ✅ Update GUI to use algorithm factory
2. ✅ Add "Fast AprilTag" to combo box
3. ✅ Simplify `detectionThreadFunction()` to use algorithm classes
4. ✅ Test all three algorithms work correctly

## Code Porting Strategy

### From `video_visualize_fixed.cu` to `fast_apriltag_algorithm.cu`

1. **Initialization** (lines 682-703):
   - Copy detector setup code
   - Copy GpuDetector creation
   - Copy camera calibration setup

2. **process_frame lambda** (lines 1096-1180):
   - Copy frame validation
   - Copy `DetectGpuOnly()` call
   - Copy `FitQuads()` call
   - Copy mirror handling
   - Copy `CopyGrayHostTo()` call
   - Copy `DecodeTagsFromQuads()` call
   - Copy coordinate scaling (if needed)
   - Copy duplicate filtering

3. **Helper functions**:
   - `filter_duplicates()` (lines 149-187) → `filterDuplicates()`
   - `is_valid_detection()` (lines 106-124) → `isValidDetection()`
   - Mirror coordinate logic (lines 1163-1180) → `mirrorQuadCoordinates()`

4. **Calibration loading**:
   - Load from fisheye calibration data (already in GUI)
   - Create CameraMatrix and DistCoeffs structs

## Key Benefits

1. ✅ **Modularity**: Each algorithm is self-contained
2. ✅ **No code mixing**: Fast AprilTag code stays separate from GUI code
3. ✅ **Extensibility**: Easy to add more algorithms later
4. ✅ **Testability**: Each algorithm can be tested independently
5. ✅ **Maintainability**: Changes to one algorithm don't affect others

## Dependencies

FastAprilTagAlgorithm needs:
- `GpuDetector` class (already available)
- `DecodeTagsFromQuads` function (already available)
- `g2d.h` (already included)
- Camera calibration data (already loaded in GUI)
- Tag family setup functions (already available)

## Testing Strategy

1. **Unit tests** (optional): Test each algorithm class independently
2. **Integration test**: Test Fast AprilTag in GUI
3. **Comparison test**: Compare results with standalone `video_visualize_fixed` program
4. **Performance test**: Ensure performance matches standalone version

## Future Algorithms

To add a new algorithm later:
1. Create new class inheriting from `AprilTagAlgorithm`
2. Implement required methods
3. Add to factory
4. Add to GUI combo box
5. Done! No need to modify existing code.

