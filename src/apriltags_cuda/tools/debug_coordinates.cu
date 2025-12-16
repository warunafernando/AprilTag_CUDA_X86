#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    string video_path;
    string family = "tag36h11";
    int max_frames = 10;

    for (int i = 1; i < argc; ++i) {
        string arg(argv[i]);
        if ((arg == "--video" || arg == "-v") && i + 1 < argc) {
            video_path = argv[++i];
        } else if ((arg == "--family" || arg == "-f") && i + 1 < argc) {
            family = argv[++i];
        } else if (arg == "--max_frames" && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        }
    }

    if (video_path.empty()) {
        cerr << "Usage: debug_coordinates --video <input.avi> [--family <tag36h11>] [--max_frames <10>]\n";
        return 1;
    }

    VideoCapture cap(video_path, CAP_ANY);
    if (!cap.isOpened()) {
        cerr << "Failed to open video: " << video_path << endl;
        return 1;
    }
    cap.set(CAP_PROP_CONVERT_RGB, false);

    Mat frame;
    if (!cap.read(frame)) {
        cerr << "Could not read first frame\n";
        return 1;
    }
    CV_Assert(frame.type() == CV_8UC1);
    int width = frame.cols;
    int height = frame.rows;

    cout << "Image size: " << width << "x" << height << endl;
    cout << "Decimated size: " << width/2 << "x" << height/2 << endl;
    cout << "quad_decimate: 2.0\n\n";

    // Setup CPU detector
    apriltag_family_t *tf_cpu = nullptr;
    setup_tag_family(&tf_cpu, family.c_str());
    apriltag_detector_t *td_cpu = apriltag_detector_create();
    apriltag_detector_add_family(td_cpu, tf_cpu);
    td_cpu->quad_decimate = 2.0;
    td_cpu->quad_sigma = 0.0;
    td_cpu->nthreads = 4;
    td_cpu->debug = false;
    td_cpu->refine_edges = true;
    td_cpu->wp = workerpool_create(4);

    // Setup GPU detector
    apriltag_family_t *tf_gpu = nullptr;
    setup_tag_family(&tf_gpu, family.c_str());
    apriltag_detector_t *td_gpu = apriltag_detector_create();
    apriltag_detector_add_family(td_gpu, tf_gpu);
    td_gpu->quad_decimate = 2.0;
    td_gpu->quad_sigma = 0.0;
    td_gpu->nthreads = 1;
    td_gpu->debug = false;
    td_gpu->refine_edges = true;
    td_gpu->wp = workerpool_create(1);

    frc971::apriltag::CameraMatrix cam{905.495617, 609.916016, 907.909470, 352.682645};
    frc971::apriltag::DistCoeffs dist{0.059238, -0.075154, -0.003801, 0.001113, 0.0};
    frc971::apriltag::GpuDetector detector_gpu(width, height, td_gpu, cam, dist);

    int frame_num = 0;
    do {
        // CPU detection
        image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = frame.data
        };
        
        zarray_t *cpu_detections = apriltag_detector_detect(td_cpu, &im);
        
        // GPU detection
        detector_gpu.Detect(frame.data);
        const zarray_t *gpu_detections = detector_gpu.Detections();
        
        if (zarray_size(cpu_detections) > 0 && zarray_size(gpu_detections) > 0) {
            apriltag_detection_t *cpu_det, *gpu_det;
            zarray_get(cpu_detections, 0, &cpu_det);
            zarray_get(gpu_detections, 0, &gpu_det);
            
            if (cpu_det->id == gpu_det->id) {
                cout << fixed << setprecision(2);
                cout << "\n=== Frame " << frame_num << " ===\n";
                cout << "Tag ID: " << cpu_det->id << "\n\n";
                
                cout << "CPU Detector Corners (full resolution):\n";
                for (int i = 0; i < 4; i++) {
                    cout << "  Corner " << i << ": (" 
                         << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] << ")\n";
                }
                
                cout << "\nGPU Detector Corners (raw GPU output - decimated space):\n";
                for (int i = 0; i < 4; i++) {
                    cout << "  Corner " << i << ": (" 
                         << gpu_det->p[i][0] << ", " << gpu_det->p[i][1] << ")\n";
                }

                // Compute scaled GPU corners/center to match full resolution
                const double gpu_decimate = td_gpu->quad_decimate;
                double gpu_scaled_corners[4][2];
                for (int i = 0; i < 4; ++i) {
                    if (gpu_decimate != 1.5) {
                        gpu_scaled_corners[i][0] =
                            (gpu_det->p[i][0] - 0.5) * gpu_decimate + 0.5;
                        gpu_scaled_corners[i][1] =
                            (gpu_det->p[i][1] - 0.5) * gpu_decimate + 0.5;
                    } else {
                        gpu_scaled_corners[i][0] = gpu_det->p[i][0] * gpu_decimate;
                        gpu_scaled_corners[i][1] = gpu_det->p[i][1] * gpu_decimate;
                    }
                }
                
                cout << "\nGPU Detector Corners (scaled to full resolution):\n";
                for (int i = 0; i < 4; ++i) {
                    cout << "  Corner " << i << ": ("
                         << gpu_scaled_corners[i][0] << ", "
                         << gpu_scaled_corners[i][1] << ")\n";
                }
                
                // Calculate centers (CPU full-res, GPU scaled)
                double cpu_cx = 0, cpu_cy = 0;
                double gpu_cx = 0, gpu_cy = 0;
                for (int i = 0; i < 4; i++) {
                    cpu_cx += cpu_det->p[i][0];
                    cpu_cy += cpu_det->p[i][1];
                    gpu_cx += gpu_scaled_corners[i][0];
                    gpu_cy += gpu_scaled_corners[i][1];
                }
                cpu_cx /= 4.0;
                cpu_cy /= 4.0;
                gpu_cx /= 4.0;
                gpu_cy /= 4.0;
                
                double dx = cpu_cx - gpu_cx;
                double dy = cpu_cy - gpu_cy;
                double dist = sqrt(dx * dx + dy * dy);
                
                cout << "\nCenter comparison (CPU vs GPU scaled):\n";
                cout << "  CPU center: (" << cpu_cx << ", " << cpu_cy << ")\n";
                cout << "  GPU center (scaled): (" << gpu_cx << ", " << gpu_cy << ")\n";
                cout << "  Offset: (" << dx << ", " << dy << ")\n";
                cout << "  Distance: " << dist << " pixels\n";
                
                break; // Only analyze first frame with detections
            }
        }
        
        apriltag_detections_destroy(cpu_detections);
        
        frame_num++;
        if (frame_num >= max_frames) break;
    } while (cap.read(frame));

    cap.release();
    apriltag_detector_destroy(td_cpu);
    apriltag_detector_destroy(td_gpu);
    teardown_tag_family(&tf_cpu, family.c_str());
    teardown_tag_family(&tf_gpu, family.c_str());
    return 0;
}

