#pragma once
#ifndef KALMANFILTER_2D_H
#define KALMANFILTER_2D_H

#include <opencv2/opencv.hpp>

class KalmanFilter2D {
private:
    cv::Mat P_;   // 6x6 Estimate error covariance
    cv::Mat Q_;   // 6x6 Process noise covariance
    cv::Mat R_;   // 2x2 Measurement noise covariance
    cv::Mat A_;   // 6x6 State transition matrix
    cv::Mat H_;   // 2x6 Measurement matrix
    cv::Mat K_;   // 6x2 Kalman gain

    int counter_notUpdate_ = 0;
    double dt_ = 1.0;
    double time_last_update_ = 0.0;

public:
    int counter_update = 0;
    double frame_last = -0.1;

    cv::Mat state_;   // 6x1: [x, y, vx, vy, ax, ay]^T

    KalmanFilter2D(double initial_x, double initial_y,
        double initial_vx, double initial_vy,
        double initial_ax, double initial_ay,
        double process_noise_pos,
        double process_noise_vel,
        double process_noise_acc,
        double measurement_noise);

    void predict(double time_current);
    void predict_only(cv::Mat& prediction, double time_current);
    void update(const cv::Mat& measurement);

    cv::Mat getState() const;
};

#endif