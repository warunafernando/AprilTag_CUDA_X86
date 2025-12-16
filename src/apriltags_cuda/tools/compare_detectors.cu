#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include "apriltag_gpu.h"
#include "apriltag_utils.h"
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
}

using namespace std;
using namespace cv;

struct DetectionInfo {
    int id;
    double center_x, center_y;
    double decision_margin;
    double hamming;
    double corners[4][2];
};

void extract_detections(const zarray_t *detections, vector<DetectionInfo> &out) {
    out.clear();
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(const_cast<zarray_t *>(detections), i, &det);
        DetectionInfo info;
        info.id = det->id;
        info.center_x = det->c[0];
        info.center_y = det->c[1];
        info.decision_margin = det->decision_margin;
        info.hamming = det->hamming;
        for (int j = 0; j < 4; j++) {
            info.corners[j][0] = det->p[j][0];
            info.corners[j][1] = det->p[j][1];
        }
        out.push_back(info);
    }
}

double distance(const DetectionInfo &a, const DetectionInfo &b) {
    double dx = a.center_x - b.center_x;
    double dy = a.center_y - b.center_y;
    return sqrt(dx * dx + dy * dy);
}

int main(int argc, char **argv) {
    string video_path;
    string family = "tag36h11";
    int max_frames = 100; // Compare first 100 frames

    for (int i = 1; i < argc; ++i) {
        string arg(argv[i]);
        if ((arg == "--video" || arg == "-v") && i + 1 < argc) {
            video_path = argv[++i];
        } else if ((arg == "--family" || arg == "-f") && i + 1 < argc) {
            family = argv[++i];
        } else if (arg == "--max_frames" && i + 1 < argc) {
            max_frames = atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            cout << "Usage: compare_detectors --video <input.avi> [--family <tag36h11>] [--max_frames <100>]\n";
            return 0;
        }
    }

    if (video_path.empty()) {
        cerr << "Error: --video is required\n";
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

    // Statistics
    int frames_processed = 0;
    int cpu_total_detections = 0;
    int gpu_total_detections = 0;
    int frames_cpu_only = 0;
    int frames_gpu_only = 0;
    int frames_both = 0;
    int frames_neither = 0;
    
    vector<double> cpu_times;
    vector<double> gpu_times;
    
    map<int, int> cpu_id_counts;
    map<int, int> gpu_id_counts;
    
    vector<double> center_distances;
    vector<double> margin_differences;

    cout << "Comparing CPU vs GPU detectors on: " << video_path << endl;
    cout << "Analyzing first " << max_frames << " frames\n\n";

    do {
        // CPU detection
        image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = frame.data
        };
        
        auto cpu_start = chrono::steady_clock::now();
        zarray_t *cpu_detections = apriltag_detector_detect(td_cpu, &im);
        auto cpu_end = chrono::steady_clock::now();
        double cpu_ms = chrono::duration<double, milli>(cpu_end - cpu_start).count();
        cpu_times.push_back(cpu_ms);
        
        // GPU detection
        auto gpu_start = chrono::steady_clock::now();
        detector_gpu.Detect(frame.data);
        auto gpu_end = chrono::steady_clock::now();
        double gpu_ms = chrono::duration<double, milli>(gpu_end - gpu_start).count();
        gpu_times.push_back(gpu_ms);
        
        const zarray_t *gpu_detections = detector_gpu.Detections();
        
        // Extract detection info
        vector<DetectionInfo> cpu_dets, gpu_dets;
        extract_detections(cpu_detections, cpu_dets);
        extract_detections(gpu_detections, gpu_dets);

        // IMPORTANT: GPU detector returns coordinates in decimated space.
        // Our td_gpu->quad_decimate is 2.0, so scale GPU coords back to full
        // resolution before comparing with CPU.
        const double gpu_decimate = td_gpu->quad_decimate;
        if (gpu_decimate > 1.0 && !gpu_dets.empty()) {
            for (auto &det : gpu_dets) {
                if (gpu_decimate != 1.5) {
                    for (int j = 0; j < 4; ++j) {
                        det.corners[j][0] =
                            (det.corners[j][0] - 0.5) * gpu_decimate + 0.5;
                        det.corners[j][1] =
                            (det.corners[j][1] - 0.5) * gpu_decimate + 0.5;
                    }
                    det.center_x = (det.center_x - 0.5) * gpu_decimate + 0.5;
                    det.center_y = (det.center_y - 0.5) * gpu_decimate + 0.5;
                } else {
                    for (int j = 0; j < 4; ++j) {
                        det.corners[j][0] *= gpu_decimate;
                        det.corners[j][1] *= gpu_decimate;
                    }
                    det.center_x *= gpu_decimate;
                    det.center_y *= gpu_decimate;
                }
            }
        }
        
        cpu_total_detections += cpu_dets.size();
        gpu_total_detections += gpu_dets.size();
        
        // Count tag IDs
        for (auto &det : cpu_dets) {
            cpu_id_counts[det.id]++;
        }
        for (auto &det : gpu_dets) {
            gpu_id_counts[det.id]++;
        }
        
        // Filter GPU duplicates (keep best per ID)
        map<int, DetectionInfo> gpu_best;
        for (auto &det : gpu_dets) {
            auto it = gpu_best.find(det.id);
            if (it == gpu_best.end() || det.decision_margin > it->second.decision_margin) {
                gpu_best[det.id] = det;
            }
        }
        
        // Compare detections
        if (cpu_dets.empty() && gpu_best.empty()) {
            frames_neither++;
        } else if (cpu_dets.empty()) {
            frames_gpu_only++;
        } else if (gpu_best.empty()) {
            frames_cpu_only++;
        } else {
            frames_both++;
            
            // Match detections by ID and compare
            for (auto &cpu_det : cpu_dets) {
                auto it = gpu_best.find(cpu_det.id);
                if (it != gpu_best.end()) {
                    const DetectionInfo &gpu_det = it->second;
                    double dist = distance(cpu_det, gpu_det);
                    
                    // Only count valid detections (within image bounds and reasonable distance)
                    // Image is 1280x1024, so valid coordinates should be within bounds
                    bool gpu_valid = (gpu_det.center_x >= 0 && gpu_det.center_x < 1280 &&
                                     gpu_det.center_y >= 0 && gpu_det.center_y < 1024);
                    bool reasonable_dist = (dist < 100.0); // Only count if within 100 pixels
                    
                    if (gpu_valid && reasonable_dist) {
                        center_distances.push_back(dist);
                        margin_differences.push_back(cpu_det.decision_margin - gpu_det.decision_margin);
                    }
                    
                    // Debug first few matches to understand the offset
                    if (center_distances.size() <= 3 || !gpu_valid || !reasonable_dist) {
                        cerr << "DEBUG Frame " << frames_processed << " Tag ID " << cpu_det.id 
                             << ": CPU=(" << fixed << setprecision(2) << cpu_det.center_x << ", " << cpu_det.center_y 
                             << "), GPU=(" << gpu_det.center_x << ", " << gpu_det.center_y 
                             << "), dist=" << dist << "px, valid=" << (gpu_valid ? "Y" : "N") 
                             << ", reasonable=" << (reasonable_dist ? "Y" : "N") << "\n";
                    }
                }
            }
        }
        
        apriltag_detections_destroy(cpu_detections);
        
        frames_processed++;
        if (frames_processed >= max_frames) break;
    } while (cap.read(frame));

    cap.release();

    // Calculate statistics
    double avg_cpu_time = 0, avg_gpu_time = 0;
    for (double t : cpu_times) avg_cpu_time += t;
    for (double t : gpu_times) avg_gpu_time += t;
    avg_cpu_time /= cpu_times.size();
    avg_gpu_time /= gpu_times.size();
    
    double avg_center_dist = 0;
    for (double d : center_distances) avg_center_dist += d;
    if (!center_distances.empty()) avg_center_dist /= center_distances.size();
    
    double avg_margin_diff = 0;
    for (double d : margin_differences) avg_margin_diff += d;
    if (!margin_differences.empty()) avg_margin_diff /= margin_differences.empty();

    // Print results
    cout << fixed << setprecision(2);
    cout << "=== DETECTION COUNT COMPARISON ===\n";
    cout << "Frames analyzed: " << frames_processed << "\n";
    cout << "CPU total detections: " << cpu_total_detections 
         << " (avg: " << (double)cpu_total_detections / frames_processed << " per frame)\n";
    cout << "GPU total detections: " << gpu_total_detections 
         << " (avg: " << (double)gpu_total_detections / frames_processed << " per frame)\n";
    cout << "GPU after filtering: " << gpu_id_counts.size() * frames_processed / frames_processed
         << " (avg: " << (double)gpu_id_counts.size() / frames_processed << " per frame)\n\n";
    
    cout << "=== FRAME-BY-FRAME COMPARISON ===\n";
    cout << "Frames with both CPU and GPU detections: " << frames_both << "\n";
    cout << "Frames with CPU only: " << frames_cpu_only << "\n";
    cout << "Frames with GPU only: " << frames_gpu_only << "\n";
    cout << "Frames with neither: " << frames_neither << "\n\n";
    
    cout << "=== TAG ID DISTRIBUTION ===\n";
    cout << "CPU detector tag IDs:\n";
    for (auto &pair : cpu_id_counts) {
        cout << "  Tag ID " << pair.first << ": " << pair.second << " detections\n";
    }
    cout << "GPU detector tag IDs:\n";
    for (auto &pair : gpu_id_counts) {
        cout << "  Tag ID " << pair.first << ": " << pair.second << " detections\n";
    }
    cout << "\n";
    
    cout << "=== PERFORMANCE COMPARISON ===\n";
    cout << "CPU average time: " << avg_cpu_time << " ms\n";
    cout << "GPU average time: " << avg_gpu_time << " ms\n";
    cout << "Speedup: " << (avg_cpu_time / avg_gpu_time) << "x\n\n";
    
    if (!center_distances.empty()) {
        cout << "=== DETECTION LOCATION COMPARISON ===\n";
        cout << "Average center distance (matched detections): " << avg_center_dist << " pixels\n";
        cout << "Max center distance: " << *max_element(center_distances.begin(), center_distances.end()) << " pixels\n";
        cout << "Min center distance: " << *min_element(center_distances.begin(), center_distances.end()) << " pixels\n\n";
        
        cout << "=== DECISION MARGIN COMPARISON ===\n";
        cout << "Average margin difference (CPU - GPU): " << avg_margin_diff << "\n";
        cout << "Positive = CPU has higher margin, Negative = GPU has higher margin\n\n";
    }
    
    cout << "=== KEY FINDINGS ===\n";
    if (gpu_total_detections > cpu_total_detections * 2) {
        cout << "⚠️  GPU detector produces significantly more detections (duplicates)\n";
    }
    if (frames_cpu_only > frames_processed * 0.1) {
        cout << "⚠️  CPU detects tags in frames where GPU misses\n";
    }
    if (frames_gpu_only > frames_processed * 0.1) {
        cout << "⚠️  GPU detects tags in frames where CPU misses\n";
    }
    if (!center_distances.empty() && avg_center_dist > 10.0) {
        cout << "⚠️  Detection centers differ significantly (>10 pixels)\n";
    }
    if (center_distances.empty() && frames_both == 0) {
        cout << "⚠️  No matching detections found between CPU and GPU\n";
    }

    apriltag_detector_destroy(td_cpu);
    apriltag_detector_destroy(td_gpu);
    teardown_tag_family(&tf_cpu, family.c_str());
    teardown_tag_family(&tf_gpu, family.c_str());
    return 0;
}

