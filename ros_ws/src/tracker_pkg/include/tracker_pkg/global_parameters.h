#pragma once
#include <iostream>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>

#ifndef GLOBAL_PARAMETERS_H
#define GLOBAL_PARAMETERS_H

//Kalman filter setting
extern const double INIT_X;
extern const double INIT_Y;
extern const double INIT_Z;
extern const double INIT_VX;
extern const double INIT_VY;
extern const double INIT_VZ;
extern const double INIT_AX;
extern const double INIT_AY;
extern const double INIT_AZ;
extern const double NOISE_POS;
extern const double NOISE_VEL;
extern const double NOISE_ACC;
extern const double NOISE_SENSOR;

extern const int COUNTER_LOST;

struct Config
{
    bool display = false;
    double time_capture = 0.0;

    std::string yolo_path;
    int yoloWidth = 0;
    int yoloHeight = 0;

    std::vector<size_t> object_index;

    double IoU_threshold = 0.0;
    double conf_threshold = 0.0;
};

struct Cam2Yolo {
    cv::Mat img_raw;
    double time_img;
};

struct Yolo2Tracking {
    cv::Mat img_raw;
    std::vector<cv::Rect2d> rois;
    std::vector<int> labels;
    double time_detect;
};



extern std::queue<Cam2Yolo> q_cam2yolo;
extern std::queue<Yolo2Tracking> q_yolo2tracking;
extern std::queue<bool> q_end;

#endif