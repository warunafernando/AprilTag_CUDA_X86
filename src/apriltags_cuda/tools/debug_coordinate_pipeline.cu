#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <fstream>
#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace std;
using namespace cv;

// Save frame to file for inspection
void save_frame(const Mat& frame, int frame_num, const string& prefix) {
    string filename = "/tmp/" + prefix + "_frame_" + to_string(frame_num) + ".png";
    imwrite(filename, frame);
    cout << "  Saved " << prefix << " frame to: " << filename << endl;
}

int main(int argc, char **argv) {
    string video_path = "/home/nav/Apriltag/VPI_based/input/Stable.avi";
    string family = "tag36h11";

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

    cout << fixed << setprecision(3);
    cout << "=== Coordinate Transformation Pipeline Debug ===\n";
    cout << "Image size: " << width << "x" << height << "\n\n";

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
    
    // First pass: Find frames where BOTH CPU and GPU detect tag ID 3
    cout << "Scanning video for frames where BOTH CPU and GPU detect tag ID 3...\n";
    cap.set(CAP_PROP_POS_FRAMES, 0);
    vector<int> test_frames;
    int frame_num = 0;
    int max_scan_frames = 100;  // Scan first 100 frames
    
    // Warmup GPU detector first
    Mat warmup_frame;
    if (cap.read(warmup_frame)) {
        detector_gpu.Detect(warmup_frame.data);
    }
    cap.set(CAP_PROP_POS_FRAMES, 0);
    frame_num = 0;
    
    do {
        if (!cap.read(frame)) break;
        
        image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = frame.data
        };
        
        // CPU detection
        zarray_t *cpu_detections = apriltag_detector_detect(td_cpu, &im);
        
        // GPU detection
        detector_gpu.Detect(frame.data);
        const zarray_t *gpu_detections = detector_gpu.Detections();
        
        // Check if BOTH detect tag ID 3
        bool cpu_has_3 = false;
        bool gpu_has_3 = false;
        
        for (int i = 0; i < zarray_size(cpu_detections); i++) {
            apriltag_detection_t *det;
            zarray_get(cpu_detections, i, &det);
            if (det->id == 3) {
                cpu_has_3 = true;
                break;
            }
        }
        
        if (gpu_detections != nullptr) {
            for (int i = 0; i < zarray_size(gpu_detections); i++) {
                apriltag_detection_t *det;
                zarray_get(gpu_detections, i, &det);
                if (det->id == 3) {
                    gpu_has_3 = true;
                    break;
                }
            }
        }
        
        if (cpu_has_3 && gpu_has_3) {
            test_frames.push_back(frame_num + 1);  // 1-indexed frame number
            cout << "  Found tag ID 3 in BOTH detectors: frame " << (frame_num + 1) << " (index " << frame_num << ")\n";
        }
        
        apriltag_detections_destroy(cpu_detections);
        frame_num++;
        
        if (test_frames.size() >= 5 || frame_num >= max_scan_frames) break;
    } while (true);
    
    if (test_frames.empty()) {
        cerr << "No frames with tag ID 3 found!\n";
        return 1;
    }
    
    cout << "\nFound " << test_frames.size() << " frames where BOTH detect tag ID 3: ";
    for (int f : test_frames) cout << f << " ";
    cout << "\n\n";
    
    if (test_frames.empty()) {
        cerr << "No frames found where both detectors detect tag ID 3!\n";
        cerr << "This means the GPU detector is not detecting tags in the same frames as CPU.\n";
        return 1;
    }
    
    ofstream log_file("/tmp/coordinate_debug.log");
    log_file << fixed << setprecision(3);
    
    // Second pass: Debug the identified frames
    cap.set(CAP_PROP_POS_FRAMES, 0);
    frame_num = 0;
    int tested = 0;
    
    do {
        if (!cap.read(frame)) break;
        
        // Check if this is a test frame (test_frames are 1-indexed, frame_num is 0-indexed)
        if (find(test_frames.begin(), test_frames.end(), frame_num + 1) == test_frames.end()) {
            frame_num++;
            continue;
        }

        cout << "\n" << string(80, '=') << "\n";
        cout << "FRAME " << (frame_num + 1) << " (index " << frame_num << ")\n";
        cout << string(80, '=') << "\n\n";
        
        log_file << "\n" << string(80, '=') << "\n";
        log_file << "FRAME " << (frame_num + 1) << " (index " << frame_num << ")\n";
        log_file << string(80, '=') << "\n\n";

        // Save original frame
        save_frame(frame, frame_num, "original");

        // CPU detection
        image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = frame.data
        };
        
        zarray_t *cpu_detections = apriltag_detector_detect(td_cpu, &im);
        
        cout << "  CPU detections: " << zarray_size(cpu_detections) << "\n";
        log_file << "  CPU detections: " << zarray_size(cpu_detections) << "\n";
        
        // GPU detection
        detector_gpu.Detect(frame.data);
        const zarray_t *gpu_detections = detector_gpu.Detections();
        
        cout << "  GPU detections: " << zarray_size(gpu_detections) << "\n";
        log_file << "  GPU detections: " << zarray_size(gpu_detections) << "\n";
        
        // Find tag ID 3 in CPU detections
        apriltag_detection_t *cpu_det = nullptr;
        for (int i = 0; i < zarray_size(cpu_detections); i++) {
            apriltag_detection_t *det;
            zarray_get(cpu_detections, i, &det);
            if (det->id == 3) {
                cpu_det = det;
                break;
            }
        }
        
        if (cpu_det && zarray_size(gpu_detections) > 0) {
            // Find matching tag ID in GPU detections
            apriltag_detection_t *gpu_det = nullptr;
            for (int i = 0; i < zarray_size(gpu_detections); i++) {
                apriltag_detection_t *det;
                zarray_get(gpu_detections, i, &det);
                if (det->id == 3) {
                    gpu_det = det;
                    break;
                }
            }
            
            if (gpu_det) {
                cout << "Tag ID: " << cpu_det->id << "\n\n";
                log_file << "Tag ID: " << cpu_det->id << "\n\n";
                
                // CPU coordinates
                cout << "CPU DETECTOR (Reference - Correct):\n";
                cout << "  Corners:\n";
                log_file << "CPU DETECTOR (Reference - Correct):\n";
                log_file << "  Corners:\n";
                for (int i = 0; i < 4; i++) {
                    cout << "    [" << i << "] (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] << ")\n";
                    log_file << "    [" << i << "] (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] << ")\n";
                }
                
                double cpu_cx = 0, cpu_cy = 0;
                for (int i = 0; i < 4; i++) {
                    cpu_cx += cpu_det->p[i][0];
                    cpu_cy += cpu_det->p[i][1];
                }
                cpu_cx /= 4.0;
                cpu_cy /= 4.0;
                cout << "  Center: (" << cpu_cx << ", " << cpu_cy << ")\n\n";
                log_file << "  Center: (" << cpu_cx << ", " << cpu_cy << ")\n\n";
                
                // GPU coordinates
                cout << "GPU DETECTOR (Current - Wrong):\n";
                cout << "  Corners:\n";
                log_file << "GPU DETECTOR (Current - Wrong):\n";
                log_file << "  Corners:\n";
                for (int i = 0; i < 4; i++) {
                    cout << "    [" << i << "] (" << gpu_det->p[i][0] << ", " << gpu_det->p[i][1] << ")\n";
                    log_file << "    [" << i << "] (" << gpu_det->p[i][0] << ", " << gpu_det->p[i][1] << ")\n";
                }
                
                double gpu_cx = 0, gpu_cy = 0;
                for (int i = 0; i < 4; i++) {
                    gpu_cx += gpu_det->p[i][0];
                    gpu_cy += gpu_det->p[i][1];
                }
                gpu_cx /= 4.0;
                gpu_cy /= 4.0;
                cout << "  Center: (" << gpu_cx << ", " << gpu_cy << ")\n\n";
                log_file << "  Center: (" << gpu_cx << ", " << gpu_cy << ")\n\n";
                
                // Analysis
                cout << "COORDINATE ANALYSIS:\n";
                log_file << "COORDINATE ANALYSIS:\n";
                
                double dx = cpu_cx - gpu_cx;
                double dy = cpu_cy - gpu_cy;
                double dist = sqrt(dx * dx + dy * dy);
                
                cout << "  Center offset: (" << dx << ", " << dy << ")\n";
                cout << "  Center distance: " << dist << " pixels\n\n";
                log_file << "  Center offset: (" << dx << ", " << dy << ")\n";
                log_file << "  Center distance: " << dist << " pixels\n\n";
                
                // Check if GPU coords are in decimated space
                cout << "  Testing if GPU coords are in DECIMATED space (half resolution):\n";
                log_file << "  Testing if GPU coords are in DECIMATED space (half resolution):\n";
                
                bool is_decimated = true;
                double max_diff = 0;
                for (int i = 0; i < 4; i++) {
                    double scaled_x = gpu_det->p[i][0] * 2.0;
                    double scaled_y = gpu_det->p[i][1] * 2.0;
                    double diff_x = fabs(scaled_x - cpu_det->p[i][0]);
                    double diff_y = fabs(scaled_y - cpu_det->p[i][1]);
                    double diff = sqrt(diff_x * diff_x + diff_y * diff_y);
                    if (diff > max_diff) max_diff = diff;
                    
                    cout << "    Corner " << i << ": GPU*2 = (" << scaled_x << ", " << scaled_y 
                         << "), CPU = (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] 
                         << "), diff = " << diff << " px\n";
                    log_file << "    Corner " << i << ": GPU*2 = (" << scaled_x << ", " << scaled_y 
                             << "), CPU = (" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] 
                             << "), diff = " << diff << " px\n";
                    
                    if (diff > 2.0) is_decimated = false;
                }
                
                cout << "\n  CONCLUSION: ";
                log_file << "\n  CONCLUSION: ";
                if (is_decimated && max_diff < 1.0) {
                    cout << "GPU coordinates ARE in decimated space (max diff: " << max_diff << " px)\n";
                    cout << "  → The coordinate fix is NOT being applied!\n";
                    log_file << "GPU coordinates ARE in decimated space (max diff: " << max_diff << " px)\n";
                    log_file << "  → The coordinate fix is NOT being applied!\n";
                } else {
                    cout << "GPU coordinates are NOT exactly in decimated space (max diff: " << max_diff << " px)\n";
                    log_file << "GPU coordinates are NOT exactly in decimated space (max diff: " << max_diff << " px)\n";
                }
                
                // Test the scaling formula
                cout << "\n  Testing AdjustPixelCenters formula: (x - 0.5) * 2 + 0.5\n";
                log_file << "\n  Testing AdjustPixelCenters formula: (x - 0.5) * 2 + 0.5\n";
                for (int i = 0; i < 4; i++) {
                    double decimated_x = gpu_det->p[i][0];
                    double decimated_y = gpu_det->p[i][1];
                    double expected_full_x = (decimated_x - 0.5) * 2.0 + 0.5;
                    double expected_full_y = (decimated_y - 0.5) * 2.0 + 0.5;
                    double diff_x = fabs(expected_full_x - cpu_det->p[i][0]);
                    double diff_y = fabs(expected_full_y - cpu_det->p[i][1]);
                    
                    cout << "    Corner " << i << ": decimated=(" << decimated_x << ", " << decimated_y 
                         << ") → full=(" << expected_full_x << ", " << expected_full_y 
                         << "), CPU=(" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] 
                         << "), diff=(" << diff_x << ", " << diff_y << ")\n";
                    log_file << "    Corner " << i << ": decimated=(" << decimated_x << ", " << decimated_y 
                             << ") → full=(" << expected_full_x << ", " << expected_full_y 
                             << "), CPU=(" << cpu_det->p[i][0] << ", " << cpu_det->p[i][1] 
                             << "), diff=(" << diff_x << ", " << diff_y << ")\n";
                }
                
                // Create visualization
                Mat vis = frame.clone();
                cvtColor(vis, vis, COLOR_GRAY2BGR);
                
                // Draw CPU detection (green)
                for (int i = 0; i < 4; i++) {
                    int next = (i + 1) % 4;
                    line(vis, 
                         Point2d(cpu_det->p[i][0], cpu_det->p[i][1]),
                         Point2d(cpu_det->p[next][0], cpu_det->p[next][1]),
                         Scalar(0, 255, 0), 2);
                }
                circle(vis, Point2d(cpu_cx, cpu_cy), 5, Scalar(0, 255, 0), -1);
                putText(vis, "CPU", Point2d(cpu_cx + 10, cpu_cy), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                
                // Draw GPU detection (red)
                for (int i = 0; i < 4; i++) {
                    int next = (i + 1) % 4;
                    line(vis, 
                         Point2d(gpu_det->p[i][0], gpu_det->p[i][1]),
                         Point2d(gpu_det->p[next][0], gpu_det->p[next][1]),
                         Scalar(0, 0, 255), 2);
                }
                circle(vis, Point2d(gpu_cx, gpu_cy), 5, Scalar(0, 0, 255), -1);
                putText(vis, "GPU", Point2d(gpu_cx + 10, gpu_cy), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 255), 2);
                
                // Draw line between centers
                line(vis, Point2d(cpu_cx, cpu_cy), Point2d(gpu_cx, gpu_cy), Scalar(255, 255, 0), 1);
                
                string vis_filename = "/tmp/comparison_frame_" + to_string(frame_num) + ".png";
                imwrite(vis_filename, vis);
                cout << "\n  Saved comparison visualization to: " << vis_filename << "\n";
                log_file << "\n  Saved comparison visualization to: " << vis_filename << "\n";
                
                tested++;
            } else {
                cout << "  WARNING: CPU found tag ID 3, but GPU did not!\n";
                log_file << "  WARNING: CPU found tag ID 3, but GPU did not!\n";
            }
        } else if (cpu_det) {
            cout << "  WARNING: CPU found tag ID 3, but GPU found no detections!\n";
            log_file << "  WARNING: CPU found tag ID 3, but GPU found no detections!\n";
        }
        
        apriltag_detections_destroy(cpu_detections);
        frame_num++;
        if (tested >= test_frames.size()) break;
    } while (true);

    cap.release();
    apriltag_detector_destroy(td_cpu);
    apriltag_detector_destroy(td_gpu);
    teardown_tag_family(&tf_cpu, family.c_str());
    teardown_tag_family(&tf_gpu, family.c_str());
    
    log_file.close();
    cout << "\n" << string(80, '=') << "\n";
    cout << "Debug complete. Log saved to: /tmp/coordinate_debug.log\n";
    cout << "Visualizations saved to: /tmp/comparison_frame_*.png\n";
    cout << string(80, '=') << "\n";
    
    return 0;
}
