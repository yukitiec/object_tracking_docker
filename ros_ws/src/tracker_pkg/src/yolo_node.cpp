#include "tracker_pkg/yolo_node.h"

// installed files.
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

// ROS2
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// standard
#include <chrono>
#include <iostream>
#include <sstream>
#include <utility>

// custom files.
#include "tracker_pkg/utils.h"

// create custom namespace.
using std::placeholders::_1;

YoloNode::YoloNode()
: Node("yolo_node")
{
  declare_parameter<std::string>("config_path", "/ros_ws/src/tracker_pkg/config/default.txt");
  declare_parameter<std::string>("image_topic", "/camera/image_raw");
  declare_parameter<std::string>("detection_topic", "/yolo/detections");
  declare_parameter<std::string>("annotated_image_topic", "/yolo/image_annotated");
  declare_parameter<bool>("publish_annotated_image", true);

  const auto config_path = get_parameter("config_path").as_string();
  const auto image_topic = get_parameter("image_topic").as_string();
  const auto detection_topic = get_parameter("detection_topic").as_string();
  const auto annotated_image_topic = get_parameter("annotated_image_topic").as_string();
  publish_annotated_image_ = get_parameter("publish_annotated_image").as_bool();

  try
  {
    cfg_ = load_config(config_path);

    RCLCPP_INFO(this->get_logger(), "Loaded config:");
    RCLCPP_INFO(this->get_logger(), "display        : %s", cfg_.display ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "time_capture   : %.3f", cfg_.time_capture);
    RCLCPP_INFO(this->get_logger(), "yolo_path      : %s", cfg_.yolo_path.c_str());
    RCLCPP_INFO(this->get_logger(), "yoloWidth      : %d", cfg_.yoloWidth);
    RCLCPP_INFO(this->get_logger(), "yoloHeight     : %d", cfg_.yoloHeight);
    RCLCPP_INFO(this->get_logger(), "IoU_threshold  : %.3f", cfg_.IoU_threshold);
    RCLCPP_INFO(this->get_logger(), "conf_threshold : %.3f", cfg_.conf_threshold);

    std::ostringstream oss;
    oss << "object_index   : ";
    for (size_t v : cfg_.object_index) {
      oss << v << " ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(this->get_logger(), "Failed to load config: %s", e.what());
    throw;
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    image_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&YoloNode::image_callback, this, _1));

  detection_pub_ =
    this->create_publisher<tracker_pkg::msg::Detection2DArray>(detection_topic, 10);

  if (publish_annotated_image_) {
    annotated_image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(annotated_image_topic, 10);
  }

  RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing detections to: %s", detection_topic.c_str());
  if (publish_annotated_image_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing annotated images to: %s",
      annotated_image_topic.c_str());
  }
}

void YoloNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv::Mat color_image;
  try
  {
    cv_bridge::CvImagePtr cv_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    color_image = cv_ptr->image;
  }
  catch (const cv_bridge::Exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  if (color_image.empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty image.");
    return;
  }

  if (!yolo_initialized_) {
    frame_width_ = color_image.cols;
    frame_height_ = color_image.rows;

    yolo_detect_ = std::make_unique<YOLODetect_batch>(
      frame_width_,
      frame_height_,
      cfg_.yoloWidth,
      cfg_.yoloHeight,
      cfg_.conf_threshold,
      cfg_.IoU_threshold,
      cfg_.object_index,
      cfg_.yolo_path);

    // Warm up model once after the detector is created.
    yolo_detect_->warmup();
    yolo_initialized_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "YOLO initialized with frame size: %d x %d",
      frame_width_,
      frame_height_);
  }

  auto st_iteration = std::chrono::steady_clock::now();

  try
  {
    std::vector<cv::Rect2d> rois_2d;
    std::vector<int> labels;

    auto result = yolo_detect_->detect(color_image, counter_, true);
    rois_2d = result.first;
    labels = result.second;

    publish_detections(msg->header, rois_2d, labels);

    if (publish_annotated_image_) {
      cv::Mat annotated = color_image.clone();
      draw_detections(annotated, rois_2d, labels);
      publish_annotated(msg->header, annotated);
    }

    auto end_iteration = std::chrono::steady_clock::now();
    double time_inference =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        end_iteration - st_iteration).count();

    if (time_inference < 100.0) {
      total_time_ += time_inference;
    }

    if (counter_ % 50 == 0 && counter_ <= 300) {
      RCLCPP_INFO(this->get_logger(), "YOLO processing time = %.3f ms", time_inference);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error during YOLO processing: %s", e.what());
  }
}

void YoloNode::publish_detections(
  const std_msgs::msg::Header & header,
  const std::vector<cv::Rect2d> & rois,
  const std::vector<int> & labels)
{
  tracker_pkg::msg::Detection2DArray msg_out;
  msg_out.header = header;

  for (size_t i = 0; i < rois.size(); ++i) {
    tracker_pkg::msg::Detection2D det;
    det.x = rois[i].x;
    det.y = rois[i].y;
    det.width = rois[i].width;
    det.height = rois[i].height;
    det.label = labels[i];
    msg_out.detections.push_back(det);
  }

  detection_pub_->publish(msg_out);
}

void YoloNode::publish_annotated(
  const std_msgs::msg::Header & header,
  const cv::Mat & image)
{
  auto msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
  annotated_image_pub_->publish(*msg);
}

void YoloNode::draw_detections(
  cv::Mat & image,
  const std::vector<cv::Rect2d> & rois,
  const std::vector<int> & labels)
{
  for (size_t i = 0; i < rois.size(); ++i) {
    cv::rectangle(image, rois[i], cv::Scalar(0, 255, 0), 2);

    std::ostringstream label_stream;
    label_stream << "Label: " << labels[i];
    std::string label = label_stream.str();

    int baseLine = 0;
    cv::Size label_size =
      cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

    int text_x = static_cast<int>(rois[i].x);
    int text_y = std::max(static_cast<int>(rois[i].y) - label_size.height - 4, 0);

    cv::rectangle(
      image,
      cv::Point(text_x, text_y),
      cv::Point(text_x + label_size.width, text_y + label_size.height + baseLine),
      cv::Scalar(0, 255, 0),
      cv::FILLED);

    cv::putText(
      image,
      label,
      cv::Point(text_x, text_y + label_size.height),
      cv::FONT_HERSHEY_SIMPLEX,
      0.5,
      cv::Scalar(0, 0, 0),
      1);
  }
}

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);

//   try
//   {
//     auto node = std::make_shared<YoloNode>();
//     rclcpp::spin(node);
//   }
//   catch (const std::exception & e)
//   {
//     std::cerr << "Exception: " << e.what() << std::endl;
//     rclcpp::shutdown();
//     return -1;
//   }

//   rclcpp::shutdown();
//   return 0;
// }