cmake_minimum_required(VERSION 3.16)
project(tracker_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

add_executable(yolo_node
  src/yolo_node.cpp
  src/yolo_detect_batch.cpp
  src/utils.cpp
  src/global_parameters.cpp
)

ament_target_dependencies(yolo_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(yolo_node
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

rosidl_target_interfaces(yolo_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(tracker_node
  src/tracker_node.cpp
  src/global_parameters.cpp
  src/hungarian.cpp
  src/kalmanfilter.cpp
  src/manage_tracker.cpp
  src/utils.cpp
)

ament_target_dependencies(tracker_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(tracker_node
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

rosidl_target_interfaces(tracker_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  yolo_node
  tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()