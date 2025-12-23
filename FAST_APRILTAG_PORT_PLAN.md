# Plan: Port Fast AprilTag Algorithm to GUI

## Overview
Port the exact detection algorithm from `video_visualize_fixed.cu` to the GUI's Algorithms tab as a new option called "Fast AprilTag".

## Current State Analysis

### Existing in GUI:
- Algorithms tab with combo box for algorithm selection
- Current options: "OpenCV CPU (AprilTag)" and "CUDA GPU (AprilTag)"
- Multi-threaded pipeline: capture → process → detection → display
- CUDA detection infrastructure already partially exists (but may need updates)

### Algorithm to Port:
- Source: `src/apriltags_cuda/tools/video_visualize_fixed.cu`
- Key detection pipeline:
  1. GPU-only detection: `gpuDetector_->DetectGpuOnly(gray.data)`
  2. Fit quads: `gpuDetector_->FitQuads()`
  3. Mirror if enabled: `gpuDetector_->MirrorGrayImageOnGpu()` + mirror quad coordinates
  4. Copy gray image from GPU: `gpuDetector_->CopyGrayHostTo(gray_host)`
  5. CPU decode: `DecodeTagsFromQuads(quads, gray_host.data(), width, height, td_gpu_, gpu_cam_, gpu_dist_, detections, poly0, poly1)`
  6. Filter duplicates and validate coordinates

## Implementation Steps

### Step 1: Add "Fast AprilTag" Option to GUI
**File**: `Tools/apriltag_debug_gui.cu`
- Location: `setupAlgorithmsTab()` around line 3208
- Add: `algorithmCombo_->addItem("Fast AprilTag");`
- Result: Combo box will have 3 options (index 0=CPU, 1=CUDA GPU, 2=Fast AprilTag)

### Step 2: Add Member Variables for Fast AprilTag
**File**: `Tools/apriltag_debug_gui.cu`
- Location: Private member variables section
- Add:
  - `bool useFastApriltagAlgorithm_;`
  - Ensure `gpuDetector_`, `td_gpu_`, `tf_gpu_`, `td_for_gpu_` exist (may already exist)
  - Ensure `gpu_cam_`, `gpu_dist_` exist (CameraMatrix and DistCoeffs)

### Step 3: Update Algorithm Selection Logic
**File**: `Tools/apriltag_debug_gui.cu`
- Location: `startAlgorithm()` around line 653-660
- Update to handle index 2 (Fast AprilTag):
  ```cpp
  int algorithmIndex = algorithmCombo_->currentIndex();
  useCudaAlgorithm_ = (algorithmIndex == 1 || algorithmIndex == 2);
  useFastApriltagAlgorithm_ = (algorithmIndex == 2);
  ```

### Step 4: Port Detection Algorithm to detectionThreadFunction()
**File**: `Tools/apriltag_debug_gui.cu`
- Location: `detectionThreadFunction()` around line 980-1100
- Current state: Has CUDA detection code, but needs to match `video_visualize_fixed.cu` exactly
- Action: Replace/update the CUDA detection section to match the exact algorithm from `video_visualize_fixed.cu`

**Key code sections to port from video_visualize_fixed.cu:**
1. GPU-only detection stage:
   ```cpp
   gpuDetector_->DetectGpuOnly(gray.data);
   ```

2. Fit quads (full resolution):
   ```cpp
   auto quads_fullres = gpuDetector_->FitQuads();
   ```

3. Mirror handling (if enabled):
   ```cpp
   if (use_mirror) {
       gpuDetector_->MirrorGrayImageOnGpu();
       // Mirror quad coordinates on CPU
       int gray_width = gpuDetector_->Width();
       for (auto& quad : quads_fullres) {
           // Mirror x coordinates for all 4 corners
           for (int i = 0; i < 4; i++) {
               quad.corners[i][0] = gray_width - 1 - quad.corners[i][0];
           }
           // Swap corners to maintain orientation: 0<->1, 2<->3
           // ... swap logic ...
       }
   }
   ```

4. Copy gray image from GPU:
   ```cpp
   vector<uint8_t> gray_host(gpuDetector_->Width() * gpuDetector_->Height());
   gpuDetector_->CopyGrayHostTo(gray_host);
   ```

5. Decode tags:
   ```cpp
   zarray_t *poly0 = g2d_polygon_create_zeros(4);
   zarray_t *poly1 = g2d_polygon_create_zeros(4);
   zarray_t *temp_detections = zarray_create(sizeof(apriltag_detection_t *));
   
   frc971::apriltag::DecodeTagsFromQuads(
       quads, gray_host.data(), gpuDetector_->Width(), gpuDetector_->Height(),
       td_gpu_, gpu_cam_, gpu_dist_, temp_detections, poly0, poly1);
   
   detections = temp_detections;
   g2d_polygon_destroy(poly0);
   g2d_polygon_destroy(poly1);
   ```

6. Filter duplicates and validate (from video_visualize_fixed.cu):
   - Filter duplicate detections by ID (keep best decision margin)
   - Validate coordinates are within image bounds
   - Scale coordinates if needed (though quads should already be full resolution)

### Step 5: Ensure CUDA Detector Initialization
**File**: `Tools/apriltag_debug_gui.cu`
- Location: `initializeCudaDetector()` around line 300-430
- Ensure it works for Fast AprilTag (should already work, but verify):
  - Creates `GpuDetector` with correct dimensions
  - Sets up `td_gpu_` for CPU decode
  - Sets up `gpu_cam_` and `gpu_dist_` from fisheye calibration

### Step 6: Handle Mirror Checkbox
**File**: `Tools/apriltag_debug_gui.cu`
- Location: `detectionThreadFunction()`
- Fast AprilTag should respect `algorithmMirrorCheckbox_->isChecked()`
- Mirror logic should be exactly as in `video_visualize_fixed.cu` (mirror on GPU + mirror quad coordinates)

### Step 7: Testing Checklist
1. ✅ GUI compiles without errors
2. ✅ "Fast AprilTag" appears in algorithm combo box
3. ✅ Selecting Fast AprilTag and clicking Start initializes CUDA detector
4. ✅ Detection runs without crashes
5. ✅ Detections appear in display
6. ✅ Mirror checkbox works correctly
7. ✅ Performance is comparable to standalone program
8. ✅ Detection results match `video_visualize_fixed.cu` output

## Key Differences from Current CUDA Implementation

### Current GUI CUDA (index 1):
- May have different initialization or detection flow
- Needs verification against `video_visualize_fixed.cu`

### Fast AprilTag (index 2):
- Exact copy of `video_visualize_fixed.cu` algorithm
- Uses same GPU detector initialization
- Uses same decode function
- Uses same mirror handling
- Uses same filtering logic

## Files to Modify

1. **Tools/apriltag_debug_gui.cu**
   - Add combo box item (line ~3208)
   - Add member variable `useFastApriltagAlgorithm_` (line ~130)
   - Update algorithm selection logic (line ~653)
   - Port detection algorithm in `detectionThreadFunction()` (line ~980-1100)
   - Verify initialization in `initializeCudaDetector()` (line ~300)

## Dependencies Already Available

✅ `GpuDetector` class - already included
✅ `DecodeTagsFromQuads` function - already declared/available
✅ `g2d.h` - already included
✅ CUDA detection infrastructure - already exists
✅ Camera calibration matrices - already loaded from fisheye calibration

## Potential Issues & Solutions

### Issue 1: Frame format mismatch
- **Problem**: Camera provides frames, need to ensure grayscale format matches
- **Solution**: Already handled - camera frames are converted to grayscale in capture thread

### Issue 2: Detector initialization timing
- **Problem**: Detector needs 2 seconds to initialize
- **Solution**: Already handled - uses delayed initialization via `initializeCudaDetector()`

### Issue 3: Coordinate scaling
- **Problem**: Quads from GPU are full resolution, but need to ensure no double-scaling
- **Solution**: Use full-resolution quads directly (already handled in FitQuads)

### Issue 4: Memory management
- **Problem**: Need to properly destroy detections and polygons
- **Solution**: Follow exact pattern from `video_visualize_fixed.cu`

## Success Criteria

1. Fast AprilTag option appears and is selectable
2. Algorithm processes camera frames correctly
3. Detection results match `video_visualize_fixed.cu` for same input
4. No crashes or memory leaks
5. Performance is acceptable (similar to standalone program)

