// ROS2
#include <rclcpp/rclcpp.hpp>

// standard
#include <iostream>
#include <memory>

// custom node headers
#include "tracker_pkg/yolo_node.h"
#include "tracker_pkg/tracker_node.h"

int main(int argc, char ** argv)
{
  // Initialize ROS 2 communication for this process.
  rclcpp::init(argc, argv);

  try
  {
    // Create each node in the processing pipeline.
    //
    // yolo_node:
    //   subscribes to /camera/image_raw
    //   publishes detection results
    //
    // tracker_node:
    //   subscribes to detection results (and image if needed)
    //   updates tracker and saves outputs
    auto yolo_node = std::make_shared<YoloNode>();
    auto tracker_node = std::make_shared<TrackerNode>();

    // Use MultiThreadedExecutor so callbacks from multiple nodes
    // can be processed in the same process without blocking each other.
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(yolo_node);
    executor.add_node(tracker_node);

    RCLCPP_INFO(
      rclcpp::get_logger("tracker_pipeline"),
      "Starting ROS 2 pipeline with yolo_node and tracker_node.");

    // Spin both nodes until shutdown is requested.
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