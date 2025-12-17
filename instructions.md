# Option 3: Standalone CUDA AprilTag on RTX2050 (No Docker, No ROS)

**Goal:** Build and run a **standalone CUDA AprilTag detector** using **Team766/apriltags_cuda** (derived from FRC Team 971 CUDA detector) on your **RTX2050** laptop, starting with video input:

* `/home/nav/Apriltag/input/Stable.avi` → must pass
* then `/home/nav/Apriltag/input/Moving.avi`

Repo note: Team766 describes this as a standalone version of Team 971 CUDA AprilTag detection, originally targeting Jetson Orin processors. Porting to RTX2050 is expected via CUDA arch flags + minor fixes.

---

## 0) Hard constraints and assumptions

1. OS: Ubuntu (x86_64)
2. GPU: NVIDIA RTX2050 (CUDA-capable)
3. Input videos are **GRAYSCALE**:

   * enforce: `CV_Assert(frame.type() == CV_8UC1)`
4. Tag family: **36h11** (FRC default)

---

## 1) Prerequisites (local, no Docker)

1. Verify driver:

   * `nvidia-smi`
2. Install build tools:

   * `sudo apt-get update`
   * `sudo apt-get install -y build-essential cmake ninja-build git pkg-config`
3. Install CUDA toolkit (match your driver):

   * verify `nvcc --version`
4. Install OpenCV dev (for video harness):

   * `sudo apt-get install -y libopencv-dev`

**Pass criteria:** you can compile and run a tiny CUDA sample and OpenCV can open `Stable.avi`.

---

## 2) Get the CUDA AprilTag code

1. Work directory:

   * `cd /home/nav/`
2. Clone:

   * `git clone https://github.com/Team766/apriltags_cuda.git`
3. Create a local working branch:

   * `cd apriltags_cuda`
   * `git checkout -b rtx2050_port`

---

## 3) Configure build for RTX2050 (CUDA architecture)

RTX2050 is Ampere-class. Ensure CMake generates code for the correct SM.

### 3.1 Preferred: CMake CUDA_ARCHITECTURES

In a clean build directory:

1. `mkdir -p build && cd build`
2. Configure with Ninja:

   * `cmake -G Ninja \
     -DCMAKE_BUILD_TYPE=Release \
     -DCMAKE_CUDA_ARCHITECTURES=86 \
     ..`

If `86` fails due to project/toolkit mismatch, try:

* `-DCMAKE_CUDA_ARCHITECTURES=native`

### 3.2 If repo hardcodes Jetson/Orin arches

Search for hard-coded `-gencode` or `CMAKE_CUDA_ARCHITECTURES` in CMake files and replace with `86` or `native`.

**Hard fail rule:** If you see runtime error like "no kernel image available" it means the build did not include your SM arch; fix CMAKE_CUDA_ARCHITECTURES.

---

## 4) Build

From `build/`:

* `ninja -j$(nproc)`

**Pass criteria:** build completes and produces a library/executable.

---

## 5) Create a minimal video benchmark harness (must do)

Even if the repo has its own demo, create a deterministic harness so you can measure end-to-end FPS.

### 5.1 Harness requirements

1. Open video: `/home/nav/Apriltag/input/Stable.avi`
2. Assert grayscale:

   * `CV_Assert(frame.type() == CV_8UC1);`
3. Upload to GPU once per frame.
4. Call CUDA AprilTag detector.
5. Retrieve detections (ID + corners).
6. Print:

   * FPS (avg)
   * per-frame ms (p50/p90/p99)
   * optional detect-only timing
7. **NO GUI** in benchmark mode.

### 5.2 Harness location

* Create: `tools/video_bench.cpp`
* Add to CMake as a separate executable linking:

  * OpenCV
  * the apriltags_cuda library/objects

---

## 6) Validation gates

### Gate A: Stable.avi

Run:

* `./video_bench --video /home/nav/Apriltag/input/Stable.avi --family 36h11`

**Pass criteria:**

* ≥120 FPS sustained (avg)
* no crashes
* reasonable detections (IDs stable, corners plausible)

### Gate B: Moving.avi (only after Stable passes)

Run:

* `./video_bench --video /home/nav/Apriltag/input/Moving.avi --family 36h11`

**Pass criteria:**

* ≥100 FPS sustained
* detection recovers quickly after blur

---

## 7) Common porting fixes (Cursor checklist)

1. Replace Jetson-specific includes/flags.
2. Fix CUDA arch flags (most common failure).
3. Ensure compilation uses `-O3` and Release.
4. Remove any forced synchronizations used only for timing.
5. If code assumes NVMM/Argus camera, disable those parts (video-only mode).

---

## 8) Deliverables

1. Clean build on RTX2050: `build/`
2. `tools/video_bench` executable
3. Bench CSV output: `out/bench_stable.csv`, `out/bench_moving.csv`
4. Short report: what FPS achieved, where bottlenecks are, next optimizations.

