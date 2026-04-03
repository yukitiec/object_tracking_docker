#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <memory>

#include "tracker_pkg/tracker_node.h"
#include "tracker_pkg/yolo_node.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto yolo_node = std::make_shared<YoloNode>();
    auto tracker_node = std::make_shared<TrackerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(yolo_node);
    executor.add_node(tracker_node);

    RCLCPP_INFO(
      rclcpp::get_logger("tracker_pipeline"),
      "Starting ROS 2 pipeline with yolo_node and tracker_node.");

    executor.spin();
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