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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// custom messages
#include "tracker_pkg/msg/detection2_d.hpp"
#include "tracker_pkg/msg/detection2_d_array.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <iostream>
#include <vector>
#include <array>
#include <ctime>
#include <algorithm>
#include <memory>
#include <string>
#include <tuple>

// custom files.
#include "tracker_pkg/global_parameters.h"
#include "tracker_pkg/manage_tracker.h"
#include "tracker_pkg/utils.h"

// create custom namespace.
using std::placeholders::_1;

using Track2DEntry = std::tuple<double, int, cv::Rect2d>;   // time, label, bbox
using Track2DSeq = std::vector<Track2DEntry>;

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode()
  : Node("tracker_node"),
    counter_(0),
    total_time_(0.0),
    counter_deb_(0),
    roi_fixed_(false),
    bool_kf_(false),
    fps_(30)
  {
    declare_parameter<std::string>("config_path", "/ros_ws/src/tracker_pkg/config/default.txt");
    declare_parameter<std::string>("output_root", "/tmp/tracker_output");
    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("detection_topic", "/yolo/detections");

    const auto config_path = get_parameter("config_path").as_string();
    output_root_ = get_parameter("output_root").as_string();
    const auto image_topic = get_parameter("image_topic").as_string();
    const auto detection_topic = get_parameter("detection_topic").as_string();

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

    start_time_ = std::chrono::steady_clock::now();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&TrackerNode::image_callback, this, _1));

    detection_sub_ = this->create_subscription<tracker_pkg::msg::Detection2DArray>(
      detection_topic,
      10,
      std::bind(&TrackerNode::detection_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribed to detection topic: %s", detection_topic.c_str());
  }

  ~TrackerNode() override
  {
    try
    {
      finalize_and_save();
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error during shutdown save: %s", e.what());
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      cv_bridge::CvImagePtr cv_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      latest_image_ = cv_ptr->image.clone();
      latest_header_ = msg->header;
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void detection_callback(const tracker_pkg::msg::Detection2DArray::SharedPtr msg)
  {
    auto now_steady = std::chrono::steady_clock::now();
    const double time_current =
      std::chrono::duration<double>(now_steady - start_time_).count();

    if (cfg_.time_capture > 0.0 && time_current > cfg_.time_capture) {
      RCLCPP_INFO(this->get_logger(), "Reached time_capture limit. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    if (counter_deb_ == 0) {
      RCLCPP_INFO(this->get_logger(), "start");
    }
    counter_deb_++;

    time_list_.push_back(time_current);

    if (latest_image_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No image received yet.");
      return;
    }

    cv::Mat color_image = latest_image_.clone();

    auto st_iteration = std::chrono::steady_clock::now();

    try
    {
      std::vector<cv::Rect2d> rois_2d;
      std::vector<int> labels;

      for (const auto & det : msg->detections) {
        rois_2d.emplace_back(det.x, det.y, det.width, det.height);
        labels.push_back(det.label);
      }

      double time_detect =
        static_cast<double>(msg->header.stamp.sec) +
        static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

      if (!rois_2d.empty()) {
        std::vector<int> index_delete_storage =
          tracker_manager_.update2D(time_detect, rois_2d, labels, storage_2d_);

        (void)index_delete_storage;
        ps_2d_.emplace_back(rois_2d);
      }

      if (cfg_.display) {
        draw_tracking_results(color_image);
        cv::imshow("PC Webcam RGB", color_image);
        cv::waitKey(1);
      }

      stored_color_images_.push_back(color_image.clone());

      auto end_iteration = std::chrono::steady_clock::now();
      double time_associate =
        std::chrono::duration_cast<std::chrono::milliseconds>(
          end_iteration - st_iteration).count();

      if (time_associate < 100.0) {
        total_time_ += time_associate;
      }

      if (counter_ % 50 == 0 && counter_ <= 300) {
        RCLCPP_INFO(this->get_logger(), "tracking processing time = %.3f ms", time_associate);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error during tracking: %s", e.what());
    }

    ++counter_;
  }

  void draw_tracking_results(cv::Mat & color_image)
  {
    for (size_t i_obj = 0; i_obj < storage_2d_.size(); ++i_obj) {
      if (storage_2d_[i_obj].empty()) {
        continue;
      }

      double t_detect = std::get<0>(storage_2d_[i_obj].back());
      int label_object = std::get<1>(storage_2d_[i_obj].back());
      cv::Rect2d bbox = std::get<2>(storage_2d_[i_obj].back());

      (void)t_detect;

      cv::rectangle(color_image, bbox, cv::Scalar(0, 0, 255), 2);

      std::ostringstream label_stream;
      label_stream << "ID: " << i_obj << " Label: " << label_object;
      std::string label = label_stream.str();

      int baseLine = 0;
      cv::Size label_size = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

      int text_x = static_cast<int>(bbox.x);
      int text_y = std::max(static_cast<int>(bbox.y) - label_size.height - 4, 0);

      cv::rectangle(
        color_image,
        cv::Point(text_x, text_y),
        cv::Point(text_x + label_size.width, text_y + label_size.height + baseLine),
        cv::Scalar(0, 0, 255),
        cv::FILLED);

      cv::putText(
        color_image,
        label,
        cv::Point(text_x, text_y + label_size.height),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 255, 255),
        1);
    }
  }

  void finalize_and_save()
  {
    if (finalized_) {
      return;
    }
    finalized_ = true;

    if (cfg_.display) {
      cv::destroyAllWindows();
    }

    double processing_speed = 0.0;
    if (total_time_ > 0.0) {
      processing_speed = static_cast<double>(counter_) / (total_time_ / 1000.0);
    }

    RCLCPP_INFO(this->get_logger(), "processing speed = %.3f Hz", processing_speed);

    auto now_time = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now_time);
    std::tm now_tm;

#ifdef _WIN32
    localtime_s(&now_tm, &time_t_now);
#else
    localtime_r(&time_t_now, &time_t_now);
#endif

    char time_str[20];
    std::snprintf(
      time_str,
      sizeof(time_str),
      "%02d_%02d_%02d_%02d",
      now_tm.tm_mday,
      now_tm.tm_hour,
      now_tm.tm_min,
      now_tm.tm_sec);

    std::string root_dir = output_root_ + "/" + time_str;
    std::filesystem::create_directories(root_dir);

    save_time_list(root_dir);

    if (!storage_2d_.empty()) {
      for (size_t i = 0; i < storage_2d_.size(); i++) {
        tracker_manager_.saved_data.push_back(storage_2d_[i]);
      }
    }

    if (!tracker_manager_.storage_2d_kf.empty()) {
      for (size_t i = 0; i < tracker_manager_.storage_2d_kf.size(); i++) {
        tracker_manager_.saved_2d_kf.push_back(tracker_manager_.storage_2d_kf[i]);
      }
    }

    save_track_csv(root_dir + "/object_2d.csv", tracker_manager_.saved_data);
    save_track_csv(root_dir + "/object_2d_kf.csv", tracker_manager_.saved_2d_kf);
    save_video(root_dir + "/output_video.mp4");
  }

  void save_time_list(const std::string & root_dir)
  {
    std::string time_list_path = root_dir + "/time_list.csv";
    std::ofstream time_list_file(time_list_path);

    if (!time_list_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open %s for writing", time_list_path.c_str());
      return;
    }

    for (size_t i = 0; i < time_list_.size(); ++i) {
      time_list_file << time_list_[i];
      if (i != time_list_.size() - 1) {
        time_list_file << ",";
      }
    }
    time_list_file << std::endl;
  }

  void save_track_csv(
    const std::string & path,
    const std::vector<std::vector<Track2DEntry>> & data)
  {
    std::ofstream file(path);

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open %s for writing", path.c_str());
      return;
    }

    for (const auto & obj_seq : data) {
      for (size_t j = 0; j < obj_seq.size(); ++j) {
        const auto & item = obj_seq[j];

        const double time = std::get<0>(item);
        const int label = std::get<1>(item);
        const cv::Rect2d & bbox = std::get<2>(item);

        file
          << time << ","
          << label << ","
          << bbox.x << ","
          << bbox.y << ","
          << bbox.width << ","
          << bbox.height;

        if (j != obj_seq.size() - 1) {
          file << ",";
        }
      }
      file << std::endl;
    }
  }

  void save_video(const std::string & video_path)
  {
    if (stored_color_images_.empty()) {
      return;
    }

    int frame_width = stored_color_images_[0].cols;
    int frame_height = stored_color_images_[0].rows;

    int fourcc = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    cv::VideoWriter video_writer(
      video_path,
      fourcc,
      fps_,
      cv::Size(frame_width, frame_height));

    if (!video_writer.isOpened()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Could not open mp4 with H.264. Falling back to MJPG.");
      fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
      video_writer.open(
        video_path,
        fourcc,
        fps_,
        cv::Size(frame_width, frame_height));
    }

    if (!video_writer.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open %s for writing video", video_path.c_str());
      return;
    }

    for (const auto & img : stored_color_images_) {
      if (img.empty()) {
        continue;
      }

      cv::Mat out_img;
      if (img.type() == CV_8UC3) {
        out_img = img;
      } else if (img.type() == CV_8UC1) {
        cv::cvtColor(img, out_img, cv::COLOR_GRAY2BGR);
      } else {
        img.convertTo(out_img, CV_8UC3);
      }

      video_writer.write(out_img);
    }

    video_writer.release();
  }

private:
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<TrackerNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception & e)
  {
    std::cerr << "Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}