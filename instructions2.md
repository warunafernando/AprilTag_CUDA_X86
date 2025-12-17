# Option 3 — Standalone CUDA AprilTag (apriltags_cuda) on RTX2050

This canvas captures **what is already completed**, **why the build is currently blocked**, and **exact next steps** to finish the build and run benchmarks — written so Cursor can follow it without ambiguity.

---

## 1) Summary of work completed (DO NOT redo)

### Environment

* GPU: **RTX 2050** (verified with `nvidia-smi`)
* CUDA Toolkit: **12.2** (downgraded from 12.4 to avoid `__cudaLaunch` compilation errors)
* OpenCV: **4.9.0** (built and installed locally)

### Repository & build config

* Repository cloned: `apriltags_cuda`
* Working branch: `rtx2050_port`
* CMake configured with:

  * `CMAKE_CUDA_ARCHITECTURES=86` (Ampere / RTX2050)

### External dependencies (DONE)

* OpenCV 4.9.0 — built + installed
* `nlohmann/json` — built + installed
* `seasocks` — built + installed
* CCCL — configured (header-only)

### Benchmark harness

* `tools/video_bench.cpp` **created**
* Features:

  * Reads `/home/nav/Apriltag/input/Stable.avi`
  * Enforces grayscale input:

    * `CV_Assert(frame.type() == CV_8UC1);`
  * Measures:

    * average FPS
    * per-frame timing
* `CMakeLists.txt` updated to add `video_bench` executable target

---

## 2) Current status (expected, not an error)

### Not built yet

* **apriltag (CPU reference library)** — REQUIRED dependency
* Main `apriltags_cuda` build — blocked waiting for apriltag
* `video_bench` executable — not built yet

### Benchmarks

* Not run yet (blocked by missing apriltag library)

---

## 3) Root cause of the block

`apriltags_cuda` **still depends on the standard CPU `apriltag` library** for:

* tag family definitions
* ID decoding / verification
* shared reference logic

Until `apriltag` is built and installed:

* CUDA detector cannot link
* `video_bench` cannot build

This is **normal and expected**.

---

## 4) NEXT STEP — Build apriltag (CPU library)

### 4.1 Clone and build

```bash
cd /home/nav
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 4.2 Verify installation

```bash
pkg-config --modversion apriltag
```

**Pass condition:** a version string is printed.

---

## 5) Link apriltag into apriltags_cuda

### 5.1 Verify CMake detection

Open `apriltags_cuda/CMakeLists.txt` and ensure **one** of the following is present:

#### Preferred (pkg-config)

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

include_directories(${APRILTAG_INCLUDE_DIRS})
link_directories(${APRILTAG_LIBRARY_DIRS})
```

Link `${APRILTAG_LIBRARIES}` into:

* apriltags_cuda library target
* `video_bench` target

Expected paths:

* Headers: `/usr/local/include/apriltag`
* Library: `/usr/local/lib/libapriltag.so`

---

## 6) Build apriltags_cuda + video_bench

```bash
cd /home/nav/apriltags_cuda/build
ninja > build.log 2>&1
```

**Expected outputs:**

* apriltags_cuda library
* `video_bench` executable

### Common failure

* `no kernel image available` → CUDA architecture not set correctly

  * Fix: ensure `CMAKE_CUDA_ARCHITECTURES=86`

---

## 7) First benchmark run (MANDATORY ORDER)

### Gate A — Stable.avi

```bash
./video_bench \
  --video /home/nav/Apriltag/input/Stable.avi \
  --family 36h11
```

Rules:

* **NO GUI**
* Capture output to file if needed

**Pass criteria:**

* Program runs without crash
* Detections look reasonable
* FPS is reported (any value acceptable for first run)

### Gate B — Moving.avi (ONLY after Gate A)

```bash
./video_bench \
  --video /home/nav/Apriltag/input/Moving.avi \
  --family 36h11
```

---

## 8) Cursor stability rules (IMPORTANT)

Cursor crashes are caused by **large build logs**.

### Mandatory rules

1. Redirect all long build output:

   ```bash
   ninja > build.log 2>&1
   ```
2. Prefer running builds in a terminal, not Cursor output pane.
3. Never paste full build logs back into Cursor.

---

## 9) Definition of next checkpoint

You are successful at this stage when:

* `video_bench` builds
* `Stable.avi` runs end-to-end
* FPS + timing numbers are printed

Only **after this** do we optimize (streams, ROI, async, multi-camera).

---

## 10) If something fails

When reporting an issue, provide **only**:

1. Exact error message (first error, not entire log)
2. Which step number failed

Do NOT guess or apply random fixes.

