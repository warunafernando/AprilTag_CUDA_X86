#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <apriltag/common/workerpool.h>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace cv;
using namespace std;

struct DetectionResult {
    int num_detections;
    vector<int> tag_ids;
    vector<double> margins;
    string method_name;
};

DetectionResult test_detection(const Mat& frame, apriltag_detector_t* td, const string& method_name) {
    DetectionResult result;
    result.method_name = method_name;
    
    // Ensure contiguous
    Mat test_frame = frame.isContinuous() ? frame : frame.clone();
    
    // Create image_u8_t
    image_u8_t im = {
        .width = test_frame.cols,
        .height = test_frame.rows,
        .stride = test_frame.cols,
        .buf = test_frame.data
    };
    
    // Detect
    zarray_t* detections = apriltag_detector_detect(td, &im);
    result.num_detections = zarray_size(detections);
    
    for (int i = 0; i < result.num_detections; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        result.tag_ids.push_back(det->id);
        result.margins.push_back(det->decision_margin);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);
    
    return result;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <image_path>" << endl;
        return 1;
    }
    
    string image_path = argv[1];
    Mat frame = imread(image_path, IMREAD_GRAYSCALE);
    
    if (frame.empty()) {
        cerr << "Failed to load image: " << image_path << endl;
        return 1;
    }
    
    cout << "\n========================================" << endl;
    cout << "DEBUGGING UNDISTORTED DETECTION" << endl;
    cout << "Image: " << image_path << endl;
    cout << "========================================" << endl;
    
    Scalar mean_val, stddev_val;
    meanStdDev(frame, mean_val, stddev_val);
    cout << "Original frame - Mean: " << mean_val[0] << ", Std: " << stddev_val[0] << endl;
    
    vector<DetectionResult> results;
    
    // Initialize detector
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    
    // Default parameters
    td->quad_decimate = 1.0;
    td->quad_sigma = 0.0;
    td->refine_edges = 1;
    td->decode_sharpening = 0.25;
    td->nthreads = 4;
    td->wp = workerpool_create(4);
    
    cout << "\n=== TESTING DEFAULT PARAMETERS ===" << endl;
    results.push_back(test_detection(frame, td, "Original (default params)"));
    
    // Test different quad_decimate values
    cout << "\n=== TESTING QUAD_DECIMATE VALUES ===" << endl;
    for (double decimate : {1.0, 1.5, 2.0, 2.5, 3.0}) {
        td->quad_decimate = decimate;
        string name = "Original (decimate=" + to_string(decimate) + ")";
        results.push_back(test_detection(frame, td, name));
    }
    td->quad_decimate = 1.0;  // Reset
    
    // Test different decode_sharpening
    cout << "\n=== TESTING DECODE_SHARPENING VALUES ===" << endl;
    for (double sharp : {0.0, 0.25, 0.5, 0.75, 1.0}) {
        td->decode_sharpening = sharp;
        string name = "Original (sharp=" + to_string(sharp) + ")";
        results.push_back(test_detection(frame, td, name));
    }
    td->decode_sharpening = 0.25;  // Reset
    
    // Test preprocessing methods
    cout << "\n=== TESTING PREPROCESSING METHODS ===" << endl;
    
    // 1. Histogram equalization
    Mat frame_eq;
    equalizeHist(frame, frame_eq);
    results.push_back(test_detection(frame_eq, td, "Histogram Equalized"));
    
    // 2. CLAHE with different clip limits
    for (double clip_limit : {2.0, 3.0, 4.0, 5.0}) {
        Ptr<CLAHE> clahe = createCLAHE(clip_limit, Size(8, 8));
        Mat frame_clahe;
        clahe->apply(frame, frame_clahe);
        string name = "CLAHE (clip=" + to_string(clip_limit) + ")";
        results.push_back(test_detection(frame_clahe, td, name));
    }
    
    // 3. Gamma correction (brighten)
    for (double gamma : {1.2, 1.5, 1.8, 2.0}) {
        double inv_gamma = 1.0 / gamma;
        Mat table(1, 256, CV_8U);
        uchar* p = table.ptr();
        for (int i = 0; i < 256; i++) {
            p[i] = saturate_cast<uchar>(pow(i / 255.0, inv_gamma) * 255.0);
        }
        Mat frame_gamma;
        LUT(frame, table, frame_gamma);
        string name = "Gamma (gamma=" + to_string(gamma) + ")";
        results.push_back(test_detection(frame_gamma, td, name));
    }
    
    // 4. Adaptive thresholding
    Mat frame_adaptive;
    adaptiveThreshold(frame, frame_adaptive, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);
    results.push_back(test_detection(frame_adaptive, td, "Adaptive Threshold"));
    
    // 5. Contrast enhancement
    Mat frame_contrast;
    frame.convertTo(frame_contrast, -1, 1.5, 0);  // alpha=1.5 (contrast), beta=0 (brightness)
    results.push_back(test_detection(frame_contrast, td, "Contrast Enhanced (1.5x)"));
    
    // 6. Brightness adjustment
    Mat frame_bright;
    frame.convertTo(frame_bright, -1, 1.0, 20);  // Increase brightness by 20
    results.push_back(test_detection(frame_bright, td, "Brightness +20"));
    
    // 7. Combine: CLAHE + Gamma
    Ptr<CLAHE> clahe = createCLAHE(3.0, Size(8, 8));
    Mat frame_clahe;
    clahe->apply(frame, frame_clahe);
    double inv_gamma = 1.0 / 1.5;
    Mat table(1, 256, CV_8U);
    uchar* p = table.ptr();
    for (int i = 0; i < 256; i++) {
        p[i] = saturate_cast<uchar>(pow(i / 255.0, inv_gamma) * 255.0);
    }
    Mat frame_combined;
    LUT(frame_clahe, table, frame_combined);
    results.push_back(test_detection(frame_combined, td, "CLAHE + Gamma"));
    
    // Print summary
    cout << "\n========================================" << endl;
    cout << "DETECTION RESULTS SUMMARY" << endl;
    cout << "========================================" << endl;
    
    bool found_any = false;
    for (const auto& result : results) {
        if (result.num_detections > 0) {
            found_any = true;
            cout << "\n✅ " << result.method_name << ": " << result.num_detections << " detection(s)" << endl;
            for (size_t i = 0; i < result.tag_ids.size(); i++) {
                cout << "   Tag ID: " << result.tag_ids[i] 
                     << ", margin: " << fixed << setprecision(2) << result.margins[i] << endl;
            }
        }
    }
    
    if (!found_any) {
        cout << "\n❌ No detections found with any method" << endl;
    } else {
        cout << "\n✅ SUCCESS: Found working method(s) above!" << endl;
    }
    
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    
    return found_any ? 0 : 1;
}



