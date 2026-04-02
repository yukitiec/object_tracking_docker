#pragma once

#ifndef YOLO_DETECT_BATCH_H
#define YOLO_DETECT_BATCH_H

#include <torch/torch.h>
#include <torch/script.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/optflow/rlofflow.hpp>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/core/hal/hal.hpp"
#include "opencv2/core/ocl.hpp"

#include <chrono>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#include "tracker_pkg/global_parameters.h"
/*  YOLO class definition  */
class YOLODetect_batch
{
private:
    const bool _debug_yolo = false;

    torch::jit::script::Module mdl;
    torch::Device * device{nullptr};

    std::string _yolofilePath;
    std::vector<size_t> _object_indices;
    double _frameWidth;
    double _frameHeight;
    double _yoloWidth;
    double _yoloHeight;
    cv::Size _YOLOSize;
    double _IoUThreshold;
    double _ConfThreshold;
    bool _warmed_up{false};
	/* initialize function with device settings (cpu, cuda)*/
    void initializeDevice();
    void loadModel();

public:
    /* constructor for YOLODetect */
    YOLODetect_batch(
        int frameWidth,
        int frameHeight,
        int yoloWidth,
        int yoloHeight,
        double conf_threshold,
        double iou_threshold,
        const std::vector<size_t> & object_indices,
        const std::string & yolo_path);
	/* deconstructor */
    ~YOLODetect_batch();

	//warmup for YOLO inference.
    void warmup();
	//detection
    std::pair<std::vector<cv::Rect2d>, std::vector<int>>
    detect(const cv::Mat & frame, int & counter, bool bool_color = true);
    //preprocess img
    void preprocessRGB(const cv::Mat & frame, torch::Tensor & imgTensor);
    void preprocessImg(const cv::Mat & frame, torch::Tensor & imgTensor);
	//organize data.
    void roiSetting(
        std::vector<torch::Tensor> & detectedBoxes,
        std::vector<int> & labels,
        std::vector<cv::Rect2d> & newRoi,
        std::vector<int> & newClass);
};

#endif