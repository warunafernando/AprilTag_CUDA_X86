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
    string video_path = "/home/nav/Apriltag/VPI_based/input/Stable.avi";
    string family = "tag36h11";
    vector<int> test_frames = {1, 5, 10, 15, 20};  // Will be updated with actual frames

    if (argc > 1) {
        // Parse frame numbers from command line
        test_frames.clear();
        for (int i = 1; i < argc; i++) {
            test_frames.push_back(atoi(argv[i]));
        }
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

    cout << fixed << setprecision(2);
    cout << "Image size: " << width << "x" << height << endl;
    cout << "Testing frames: ";
    for (int f : test_frames) cout << f << " ";
    cout << "\n\n";

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
    int tested = 0;
    
    // Sort test frames
    sort(test_frames.begin(), test_frames.end());
    
    do {
        // Check if this is a test frame
        if (find(test_frames.begin(), test_frames.end(), frame_num) == test_frames.end()) {
            frame_num++;
            if (!cap.read(frame)) break;
            continue;
        }

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
                cout << "=== Frame " << frame_num << " ===\n";
                cout << "Tag ID: " << cpu_det->id << "\n\n";
                
                cout << "CPU Corners:\n";
                for (int i = 0; i < 4; i++) {
                    cout << "  [" << i << "] (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] << ")\n";
                }
                
                cout << "\nGPU Corners:\n";
                for (int i = 0; i < 4; i++) {
                    cout << "  [" << i << "] (" << gpu_det->p[i][0] << ", " << gpu_det->p[i][1] << ")\n";
                }
                
                // Calculate centers
                double cpu_cx = 0, cpu_cy = 0;
                double gpu_cx = 0, gpu_cy = 0;
                for (int i = 0; i < 4; i++) {
                    cpu_cx += cpu_det->p[i][0];
                    cpu_cy += cpu_det->p[i][1];
                    gpu_cx += gpu_det->p[i][0];
                    gpu_cy += gpu_det->p[i][1];
                }
                cpu_cx /= 4.0;
                cpu_cy /= 4.0;
                gpu_cx /= 4.0;
                gpu_cy /= 4.0;
                
                double dx = cpu_cx - gpu_cx;
                double dy = cpu_cy - gpu_cy;
                double dist = sqrt(dx * dx + dy * dy);
                
                cout << "\nCenter Comparison:\n";
                cout << "  CPU: (" << cpu_cx << ", " << cpu_cy << ")\n";
                cout << "  GPU: (" << gpu_cx << ", " << gpu_cy << ")\n";
                cout << "  Offset: (" << dx << ", " << dy << ")\n";
                cout << "  Distance: " << dist << " pixels\n";
                
                // Check if GPU coords are exactly half
                cout << "\nCoordinate Analysis:\n";
                cout << "  If GPU coords are in decimated space (half):\n";
                for (int i = 0; i < 4; i++) {
                    double scaled_x = gpu_det->p[i][0] * 2.0;
                    double scaled_y = gpu_det->p[i][1] * 2.0;
                    double diff_x = fabs(scaled_x - cpu_det->p[i][0]);
                    double diff_y = fabs(scaled_y - cpu_det->p[i][1]);
                    cout << "    Corner " << i << ": GPU*2 = (" << scaled_x << ", " << scaled_y 
                         << "), CPU = (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] 
                         << "), diff = (" << diff_x << ", " << diff_y << ")\n";
                }
                
                cout << "\n";
                tested++;
            }
        }
        
        apriltag_detections_destroy(cpu_detections);
        tested++;
        frame_num++;
        if (tested >= test_frames.size()) break;
    } while (cap.read(frame));

    cap.release();
    apriltag_detector_destroy(td_cpu);
    apriltag_detector_destroy(td_gpu);
    teardown_tag_family(&tf_cpu, family.c_str());
    teardown_tag_family(&tf_gpu, family.c_str());
    return 0;
}

