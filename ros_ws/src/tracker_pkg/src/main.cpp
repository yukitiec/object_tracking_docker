#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

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

    std::cout << "[INFO] Starting integration test with YoloNode and TrackerNode..." << std::endl;

    std::thread spin_thread([&executor]() {
      executor.spin();
    });

    // Let the pipeline run briefly.
    std::this_thread::sleep_for(std::chrono::seconds(3));

    executor.cancel();

    if (spin_thread.joinable()) {
      spin_thread.join();
    }

    rclcpp::shutdown();

    std::cout << "[PASS] Pipeline integration test completed." << std::endl;
    return 0;
  }
  catch (const std::exception & e)
  {
    std::cerr << "[FAIL] Exception: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  catch (...)
  {
    std::cerr << "[FAIL] Unknown exception" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}