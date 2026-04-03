#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tracker_pkg/global_parameters.h"
#include "tracker_pkg/manage_tracker.h"
#include "tracker_pkg/utils.h"

using Track2DEntry = std::tuple<double, int, cv::Rect2d>;

void test_synthetic_image_creation()
{
    // Create a synthetic image instead of using a physical camera.
    cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);

    // Draw a white rectangle as a dummy object.
    cv::rectangle(img, cv::Rect(100, 120, 80, 60), cv::Scalar(255, 255, 255), cv::FILLED);

    assert(!img.empty());
    assert(img.rows == 480);
    assert(img.cols == 640);

    std::cout << "[PASS] test_synthetic_image_creation" << std::endl;
}

void test_load_config()
{
    // Prepare a temporary config file for testing.
    const std::string config_path = "/ros_ws/src/tracker_pkg/config/default.txt";

    std::ofstream ofs(config_path);
    ofs << "display=1\n";
    ofs << "time_capture=5.0\n";
    ofs << "yolo_path=/tmp/model.torchscript\n";
    ofs << "yoloWidth=640\n";
    ofs << "yoloHeight=480\n";
    ofs << "object_index=0,1\n";
    ofs << "IoU_threshold=0.5\n";
    ofs << "conf_threshold=0.4\n";
    ofs.close();

    Config cfg = load_config(config_path);

    assert(cfg.yoloWidth == 640);
    assert(cfg.yoloHeight == 480);
    assert(cfg.conf_threshold > 0.0);

    std::cout << "[PASS] test_load_config" << std::endl;
}

void test_tracker_update2d()
{
    TrackerManager tracker_manager;

    std::vector<std::vector<Track2DEntry>> storage_2d;
    std::vector<cv::Rect2d> rois;
    std::vector<int> labels;

    // Synthetic detections
    rois.emplace_back(100.0, 100.0, 50.0, 60.0);
    rois.emplace_back(220.0, 180.0, 40.0, 40.0);
    labels.push_back(0);
    labels.push_back(1);

    double time_detect = 1.23;

    // This assumes update2D returns a vector<int>.
    std::vector<int> deleted_indices =
        tracker_manager.update2D(time_detect, rois, labels, storage_2d);

    (void)deleted_indices;

    // Basic sanity checks
    assert(!storage_2d.empty());

    std::cout << "[PASS] test_tracker_update2d" << std::endl;
}

int main()
{
    try {
        test_synthetic_image_creation();
        test_load_config();
        test_tracker_update2d();
    }
    catch (const std::exception & e) {
        std::cerr << "[FAIL] Exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...) {
        std::cerr << "[FAIL] Unknown exception" << std::endl;
        return 1;
    }

    std::cout << "All tests passed." << std::endl;
    return 0;
}