#include "tracker_pkg/kalmanfilter.h"
#include <stdexcept>

KalmanFilter2D::KalmanFilter2D(double initial_x, double initial_y,
    double initial_vx, double initial_vy,
    double initial_ax, double initial_ay,
    double process_noise_pos,
    double process_noise_vel,
    double process_noise_acc,
    double measurement_noise)
{
    // State: [x, y, vx, vy, ax, ay]^T
    state_ = cv::Mat::zeros(6, 1, CV_64F);
    state_.at<double>(0, 0) = initial_x;
    state_.at<double>(1, 0) = initial_y;
    state_.at<double>(2, 0) = initial_vx;
    state_.at<double>(3, 0) = initial_vy;
    state_.at<double>(4, 0) = initial_ax;
    state_.at<double>(5, 0) = initial_ay;

    // Initial estimate error covariance
    P_ = cv::Mat::eye(6, 6, CV_64F);

    // Process noise covariance
    Q_ = cv::Mat::zeros(6, 6, CV_64F);
    Q_.at<double>(0, 0) = process_noise_pos;
    Q_.at<double>(1, 1) = process_noise_pos;
    Q_.at<double>(2, 2) = process_noise_vel;
    Q_.at<double>(3, 3) = process_noise_vel;
    Q_.at<double>(4, 4) = process_noise_acc;
    Q_.at<double>(5, 5) = process_noise_acc;

    // Measurement noise covariance
    R_ = cv::Mat::eye(2, 2, CV_64F) * measurement_noise;

    // Allocate matrices
    A_ = cv::Mat::eye(6, 6, CV_64F);
    H_ = cv::Mat::zeros(2, 6, CV_64F);
    K_ = cv::Mat::zeros(6, 2, CV_64F);

    time_last_update_ = 0.0;
}

void KalmanFilter2D::predict(double time_current)
{
    if (time_last_update_ == 0.0)
        time_last_update_ = time_current;

    dt_ = time_current - time_last_update_;

    // Constant acceleration model
    A_ = cv::Mat::eye(6, 6, CV_64F);

    A_.at<double>(0, 2) = dt_;
    A_.at<double>(1, 3) = dt_;
    A_.at<double>(0, 4) = 0.5 * dt_ * dt_;
    A_.at<double>(1, 5) = 0.5 * dt_ * dt_;
    A_.at<double>(2, 4) = dt_;
    A_.at<double>(3, 5) = dt_;

    // Predict state
    state_ = A_ * state_;

    // Predict covariance
    P_ = A_ * P_ * A_.t() + Q_;

    counter_notUpdate_++;
    time_last_update_ = time_current;
}

void KalmanFilter2D::predict_only(cv::Mat& prediction, double time_current)
{
    if (time_last_update_ == 0.0)
        time_last_update_ = time_current;

    dt_ = time_current - time_last_update_;

    A_ = cv::Mat::eye(6, 6, CV_64F);

    A_.at<double>(0, 2) = dt_;
    A_.at<double>(1, 3) = dt_;
    A_.at<double>(0, 4) = 0.5 * dt_ * dt_;
    A_.at<double>(1, 5) = 0.5 * dt_ * dt_;
    A_.at<double>(2, 4) = dt_;
    A_.at<double>(3, 5) = dt_;

    prediction = A_ * state_;
}

void KalmanFilter2D::update(const cv::Mat& measurement)
{
    if (measurement.rows != 2 || measurement.cols != 1 || measurement.type() != CV_64F) {
        throw std::runtime_error("Measurement must be a 2x1 CV_64F cv::Mat.");
    }

    // Measurement matrix: we observe x and y only
    H_ = cv::Mat::zeros(2, 6, CV_64F);
    H_.at<double>(0, 0) = 1.0;
    H_.at<double>(1, 1) = 1.0;

    // Innovation covariance
    cv::Mat S = H_ * P_ * H_.t() + R_;

    // Kalman gain
    K_ = P_ * H_.t() * S.inv();

    // Update state
    state_ = state_ + K_ * (measurement - H_ * state_);

    // Update covariance
    cv::Mat I = cv::Mat::eye(6, 6, CV_64F);
    P_ = (I - K_ * H_) * P_;

    counter_notUpdate_ = 0;
    counter_update++;
}

cv::Mat KalmanFilter2D::getState() const
{
    return state_.clone();
}