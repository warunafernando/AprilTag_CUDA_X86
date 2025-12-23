# Detailed Timing Analysis (Post Display-Thread Refactor)

## Date: 2024-12-17

## Executive Summary

This report summarizes the **current end-to-end timing** of the CUDA AprilTag detector after:
- Moving **frame input** to a reader thread with a prefetch queue, and  
- Moving **display and optional video writing** to a dedicated display/write thread with a bounded queue.

With this architecture, the **main detection thread** now runs at **~245–265 FPS** on 1280×1024 grayscale video, and **detection itself is the dominant cost**. Display and disk I/O are fully decoupled from the detector.

---

## Test Configuration

- **Videos**:
  - `input/Stable.avi` (static tag motion)
  - `input/Moving.avi` (camera/tag motion)
- **Resolution**: 1280×1024, grayscale
- **Tag family**: `tag36h11`
- **Tag size**: 0.305 m
- **Pose / visualization**: 3D axes, tag outlines, ID, and info table enabled
- **Threads**:
  - Reader thread → `frame_queue`
  - Main detection thread (CUDA + CPU decode + overlays)
  - Display/write thread → `draw_queue` (`imshow` + optional `VideoWriter`)

---

## Stable.avi Timing (After Display Thread Change)

**Command:**

```bash
./video_visualize_fixed --video input/Stable.avi
```

**Summary:**

```text
Completed processing 2018 frames in 8.09 seconds
Average processing FPS: 249.33
Total detections before filtering: 8068
Total detections after filtering: 2017
Average per frame: 3 -> 0
```

**Per-frame timing (main detection thread, averages):**

| Stage                | Time (ms) |
|----------------------|-----------|
| Frame read\*         | 0.52      |
| Detect total         | 2.28      |
| ├─ CUDA ops          | 1.50      |
| └─ CPU decode        | 0.75      |
| Scale coordinates    | 0.00      |
| Filter duplicates    | 0.00      |
| Draw (axes/text)\*\* | 0.74      |
| Write frame          | 0.00      |

\* Frame read is performed in the reader thread and reported per frame; it does **not** block the detector.  
\*\* Draw time now covers only overlays (3D axes, outlines, table); `imshow`/`waitKey` run in the display thread.

---

## Moving.avi Timing (After Display Thread Change)

**Command:**

```bash
./video_visualize_fixed --video input/Moving.avi
```

**Summary:**

```text
Completed processing 1916 frames in 7.28 seconds
Average processing FPS: 263.02
Total detections before filtering: 6641
Total detections after filtering: 1381
Average per frame: 3 -> 0
```

**Per-frame timing (main detection thread, averages):**

| Stage                | Time (ms) |
|----------------------|-----------|
| Frame read\*         | 0.52      |
| Detect total         | 2.31      |
| ├─ CUDA ops          | 1.55      |
| └─ CPU decode        | 0.73      |
| Scale coordinates    | 0.00      |
| Filter duplicates    | 0.00      |
| Draw (axes/text)\*\* | 0.49      |
| Write frame          | 0.00      |

The moving sequence shows slightly higher detector-thread FPS (~263 FPS) with nearly identical per-stage timings, indicating that motion and varying detection counts do not significantly affect performance.

---

## Stage-Level Observations

### 1. Frame Read

- **0.52 ms/frame** on both Stable and Moving videos.
- Runs in a dedicated **reader thread** using a bounded prefetch queue.
- Main detection thread only sees ready frames; read time does not block detection.

### 2. Detection (CUDA + CPU)

- **Total detection time**: ~**2.28–2.31 ms/frame**.
- **CUDA operations**: ~**1.50–1.55 ms/frame** (≈65–67% of detection).
- **CPU decode**: ~**0.73–0.75 ms/frame** (≈33–35% of detection).
- This is now the **primary cost per frame** on the main thread.

### 3. Coordinate Scaling and Duplicate Filtering

- Both stages are effectively **free** at this scale:
  - Scale: **0.00 ms/frame** (simple arithmetic)
  - Filter: **0.00 ms/frame** (efficient duplicate removal)

### 4. Drawing / Overlays (Main Thread)

- **0.49–0.74 ms/frame** for:
  - Grayscale → BGR conversion
  - `solvePnP` and `drawFrameAxes`
  - Tag outlines and text
  - Information table
- This timing **excludes display**; `imshow` and `waitKey` are handled in the display thread.

### 5. Display and Video Writing (Display/Writer Thread)

- Not included in the timings above.
- Operations:
  - Display (`imshow`/`waitKey`) of rendered frames.
  - Optional `VideoWriter::write()` when `--output` is specified.
- These operations no longer affect detector-thread FPS thanks to the `draw_queue` and separate display/write thread.

---

## Key Conclusions

1. **Detector-thread throughput** is now **~245–265 FPS** at 1280×1024 with full overlays enabled.
2. **Detection (CUDA + CPU decode)** at ~2.3 ms/frame is the dominant main-thread cost; visualization overlays add <1 ms/frame.
3. **Frame I/O (read/write) and display** are fully decoupled from detection via separate reader and display/write threads with bounded queues.
4. For applications that care primarily about **throughput**, the current architecture delivers very high performance without further changes.










