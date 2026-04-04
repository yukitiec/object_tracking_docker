#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

// OpenCV
#include <opencv2/opencv.hpp>

// standard
#include <memory>
#include <string>
#include <vector>

// custom messages
#include "tracker_pkg/msg/detection2_d.hpp"
#include "tracker_pkg/msg/detection2_d_array.hpp"

// custom files
#include "tracker_pkg/global_parameters.h"
#include "tracker_pkg/yolo_detect_batch.h"

class YoloNode : public rclcpp::Node
{
public:
  YoloNode();
  ~YoloNode() override;

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  void publish_detections(
    const std_msgs::msg::Header & header,
    const std::vector<cv::Rect2d> & rois,
    const std::vector<int> & labels);

  void publish_annotated(
    const std_msgs::msg::Header & header,
    const cv::Mat & image);

  void draw_detections(
    cv::Mat & image,
    const std::vector<cv::Rect2d> & rois,
    const std::vector<int> & labels);

private:
  Config cfg_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<tracker_pkg::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;

  std::unique_ptr<YOLODetect_batch> yolo_detect_;
  bool yolo_initialized_{false};
  bool publish_annotated_image_{false};

  int frame_width_{0};
  int frame_height_{0};

  int counter_{0};
  double total_time_{0.0};
};