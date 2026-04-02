#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "tracker_pkg/global_parameters.h"
#include "tracker_pkg/manage_tracker.h"
#include "tracker_pkg/msg/detection2_d_array.hpp"

using Track2DEntry = std::tuple<double, int, cv::Rect2d>;
using Track2DSeq = std::vector<Track2DEntry>;

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode();
  ~TrackerNode() override;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void detection_callback(const tracker_pkg::msg::Detection2DArray::SharedPtr msg);
  void draw_tracking_results(cv::Mat & color_image);
  void finalize_and_save();
  void save_time_list(const std::string & root_dir);
  void save_track_csv(
    const std::string & path,
    const std::vector<std::vector<Track2DEntry>> & data);
  void save_video(const std::string & video_path);

  Config cfg_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<tracker_pkg::msg::Detection2DArray>::SharedPtr detection_sub_;

  TrackerManager tracker_manager_;

  cv::Mat latest_image_;
  std_msgs::msg::Header latest_header_;

  std::vector<std::vector<cv::Rect2d>> ps_2d_;
  std::vector<std::vector<Track2DEntry>> storage_2d_;
  std::vector<cv::Mat> stored_color_images_;
  std::vector<double> time_list_;

  std::chrono::steady_clock::time_point start_time_;

  std::string output_root_;
  int fps_;

  int counter_;
  double total_time_;
  int counter_deb_;
  bool roi_fixed_;
  const bool bool_kf_;
  bool finalized_{false};
};