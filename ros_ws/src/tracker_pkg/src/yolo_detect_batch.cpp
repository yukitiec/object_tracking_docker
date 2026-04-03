#include "tracker_pkg/yolo_detect_batch.h"

void YOLODetect_batch::initializeDevice()
{
    // set device
    if (torch::cuda::is_available()) {
        device = new torch::Device(torch::kCUDA);
        std::cout << "[yolo] set CUDA" << std::endl;
    } else {
        device = new torch::Device(torch::kCPU);
        std::cout << "[yolo] set CPU" << std::endl;
    }
}

void YOLODetect_batch::loadModel()
{
    // read model
    mdl = torch::jit::load(_yolofilePath, *device);
    mdl.to(*device);
    mdl.eval();
    std::cout << "[yolo] model loaded" << std::endl;
}

YOLODetect_batch::YOLODetect_batch(
    int frameWidth,
    int frameHeight,
    int yoloWidth,
    int yoloHeight,
    double conf_threshold,
    double iou_threshold,
    const std::vector<size_t> & object_indices,
    const std::string & yolo_path)
{
    _yolofilePath = yolo_path;
    _object_indices = object_indices;
    _frameWidth = static_cast<double>(frameWidth);
    _frameHeight = static_cast<double>(frameHeight);
    _yoloWidth = static_cast<double>(yoloWidth);
    _yoloHeight = static_cast<double>(yoloHeight);
    _YOLOSize = {yoloWidth, yoloHeight};
    _ConfThreshold = conf_threshold;
    _IoUThreshold = iou_threshold;

    initializeDevice();
    loadModel();

    std::cout << "[yolo] constructor finished" << std::endl;
}

YOLODetect_batch::~YOLODetect_batch()
{
    delete device;
    device = nullptr;
}

void YOLODetect_batch::warmup()
{
    if (_warmed_up) {
        return;
    }

    std::cout << "[yolo] warmup started" << std::endl;

    // Warmup
    cv::Mat warmup_img = cv::Mat::zeros(
        static_cast<int>(_yoloHeight),
        static_cast<int>(_yoloWidth),
        CV_8UC3);
    int warmup_counter = -1;

    for (int i = 0; i < 5; ++i) {
        try {
            auto warmup_result = detect(warmup_img, warmup_counter, true);
            std::cout << "[yolo] warmup " << i
                      << " :: " << warmup_result.first.size()
                      << " ROIs" << std::endl;
        }
        catch (const std::exception & e) {
            std::cerr << "[yolo] warmup failed at iteration "
                      << i << ": " << e.what() << std::endl;
        }
    }

    _warmed_up = true;
    std::cout << "[yolo] warmup finished" << std::endl;
}

void YOLODetect_batch::preprocessRGB(const cv::Mat & frame, torch::Tensor & imgTensor)
{
    // run
    cv::Mat yoloimg;  // define yolo img type

    // Resize input image to YOLO input size
    cv::resize(frame, yoloimg, _YOLOSize);

    // vector to tensor
    imgTensor = torch::from_blob(
        yoloimg.data,
        {yoloimg.rows, yoloimg.cols, 3},
        torch::kByte);

    // Convert shape from (H,W,C) -> (C,H,W)
    imgTensor = imgTensor.permute({2, 0, 1});

    // convert to float type
    imgTensor = imgTensor.toType(torch::kFloat);

    // normalization
    imgTensor = imgTensor.div(255);

    // expand dims for batch input -> (1,3,H,W)
    imgTensor = imgTensor.unsqueeze(0);

    // transport data to GPU/CPU
    imgTensor = imgTensor.to(*device);
}

void YOLODetect_batch::preprocessImg(const cv::Mat & frame, torch::Tensor & imgTensor)
{
    // run
    cv::Mat yoloimg;  // define yolo img type

    // Resize input image to YOLO input size
    cv::resize(frame, yoloimg, _YOLOSize);

    // Convert grayscale image to RGB
    cv::cvtColor(yoloimg, yoloimg, cv::COLOR_GRAY2RGB);

    // vector to tensor
    imgTensor = torch::from_blob(
        yoloimg.data,
        {yoloimg.rows, yoloimg.cols, 3},
        torch::kByte);

    // Convert shape from (H,W,C) -> (C,H,W)
    imgTensor = imgTensor.permute({2, 0, 1});

    // convert to float type
    imgTensor = imgTensor.toType(torch::kFloat);

    // normalization
    imgTensor = imgTensor.div(255);

    // expand dims for Convolutional layer / batch input
    imgTensor = imgTensor.unsqueeze(0);

    // transport data to GPU/CPU
    imgTensor = imgTensor.to(*device);
}

// #(objects), (left,top,width,height)
std::pair<std::vector<cv::Rect2d>, std::vector<int>>
YOLODetect_batch::detect(const cv::Mat & frame, int & counter, const bool bool_color)
{
    if (frame.empty()) {
        throw std::runtime_error("[yolo] detect() received empty frame");
    }

    /* preprocess img */
    torch::Tensor imgTensor;
    if (bool_color) {
        preprocessRGB(frame, imgTensor);
    } else {
        preprocessImg(frame, imgTensor);
    }

    /* inference */
    torch::Tensor preds;
    auto start_inf = std::chrono::high_resolution_clock::now();

    /* wrap to disable grad calculation */
    {
        torch::NoGradGuard no_grad;
        preds = mdl.forward({imgTensor}).toTensor();  // preds shape : [1,n(=N_candidates),6]
    }

    if (_debug_yolo) {
        auto stop_inf = std::chrono::high_resolution_clock::now();
        auto duration_inf =
            std::chrono::duration_cast<std::chrono::microseconds>(stop_inf - start_inf);
        std::cout << "** YOLO inference time = "
                  << duration_inf.count() << " microseconds **" << std::endl;
    }

    // POST PROCESS
    auto start_postprocess = std::chrono::high_resolution_clock::now();

    // STEP1 :: extract the high score detections
    std::vector<torch::Tensor> rois;  // detected rois. (n,4)
    std::vector<int> labels;          // detected labels.

    torch::Tensor preds_good = preds.select(2, 4) >= _ConfThreshold;  // score threshold
    torch::Tensor x0 =
        preds.index_select(1, torch::nonzero(preds_good[0]).select(1, 0));

    // (1,n,6) -> (n,6)
    x0 = x0.squeeze(0);

    int size = 0;
    if (x0.defined() && x0.numel() > 0) {
        if (x0.dim() == 1) {
            x0 = x0.unsqueeze(0);
        }
        size = static_cast<int>(x0.size(0));  // num of detections
    }

    torch::Tensor bbox, pred;
    int label = -1;

    for (int i = 0; i < size; i++) {
        pred = x0[i].cpu();
        bbox = pred.slice(0, 0, 4).clone();  // x.slice(dim,start,end);

        // clamp bbox inside YOLO input image
        bbox[0] = std::max(bbox[0].item<double>(), 0.0);          // left
        bbox[1] = std::max(bbox[1].item<double>(), 0.0);          // top
        bbox[2] = std::min(bbox[2].item<double>(), _yoloWidth);   // right
        bbox[3] = std::min(bbox[3].item<double>(), _yoloHeight);  // bottom

        label = pred[5].item<int>();  // label
        rois.push_back(bbox);
        labels.push_back(label);
    }

    std::vector<cv::Rect2d> rois_new;
    std::vector<int> classes;

    // separate / convert detection to original image scale
    roiSetting(rois, labels, rois_new, classes);

    if (_debug_yolo) {
        auto stop_postprocess = std::chrono::high_resolution_clock::now();
        auto duration_postprocess =
            std::chrono::duration_cast<std::chrono::microseconds>(
                stop_postprocess - start_postprocess);
        std::cout << "** YOLO postprocess time = "
                  << duration_postprocess.count() << " microseconds **" << std::endl;
    }

    ++counter;
    return std::make_pair(rois_new, classes);
}

void YOLODetect_batch::roiSetting(
    std::vector<torch::Tensor> & detectedBoxes,
    std::vector<int> & labels,
    std::vector<cv::Rect2d> & newRoi,
    std::vector<int> & newClass)
{
    /* detected by Yolo */
    if (!detectedBoxes.empty()) {
        int numBboxes = static_cast<int>(detectedBoxes.size());  // num of detection

        /* convert torch::Tensor to cv::Rect2d */
        for (int i = 0; i < numBboxes; ++i) {
            // detect only designated objects
            if (std::find(_object_indices.begin(), _object_indices.end(), labels[i]) !=
                _object_indices.end()) {
                // resize bbox to fit original img size
                double expandrate_x = _frameWidth / _yoloWidth;
                double expandrate_y = _frameHeight / _yoloHeight;

                int left =
                    static_cast<int>(detectedBoxes[i][0].item().toFloat() * expandrate_x);
                int top =
                    static_cast<int>(detectedBoxes[i][1].item().toFloat() * expandrate_y);
                int right =
                    static_cast<int>(detectedBoxes[i][2].item().toFloat() * expandrate_x);
                int bottom =
                    static_cast<int>(detectedBoxes[i][3].item().toFloat() * expandrate_y);

                newRoi.emplace_back(left, top, (right - left), (bottom - top));
                newClass.push_back(labels[i]);
            }
        }
    }
    /* No object detected in Yolo -> return empty */
    else {
        /* nothing to do */
    }
}