#include <iostream>
#include <vector>
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
    int max_frames = 100;
    int target_frames = 5;

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

    // Setup CPU detector
    apriltag_family_t *tf = nullptr;
    setup_tag_family(&tf, family.c_str());
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->quad_sigma = 0.0;
    td->nthreads = 4;
    td->debug = false;
    td->refine_edges = true;
    td->wp = workerpool_create(4);

    vector<int> good_frames;
    int frame_num = 0;
    
    do {
        image_u8_t im = {
            .width = width,
            .height = height,
            .stride = width,
            .buf = frame.data
        };
        
        zarray_t *detections = apriltag_detector_detect(td, &im);
        
        if (zarray_size(detections) > 0) {
            apriltag_detection_t *det;
            zarray_get(detections, 0, &det);
            if (det->id == 3) {  // Tag ID 3
                good_frames.push_back(frame_num);
                cout << "Frame " << frame_num << ": Found tag ID " << det->id 
                     << " at center (" << det->c[0] << ", " << det->c[1] << ")" << endl;
                
                if (good_frames.size() >= target_frames) {
                    break;
                }
            }
        }
        
        apriltag_detections_destroy(detections);
        frame_num++;
        if (frame_num >= max_frames) break;
    } while (cap.read(frame));

    cap.release();
    apriltag_detector_destroy(td);
    teardown_tag_family(&tf, family.c_str());

    cout << "\n=== Test Frames Selected ===" << endl;
    cout << "Found " << good_frames.size() << " frames with tag ID 3:" << endl;
    for (int f : good_frames) {
        cout << "  Frame " << f << endl;
    }
    
    return 0;
}







