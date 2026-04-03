cmake_minimum_required(VERSION 3.16)
project(tracker_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
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

add_executable(tracker_pipeline
  src/main.cpp
  src/yolo_node.cpp
  src/tracker_node.cpp
  src/yolo_detect_batch.cpp
  src/manage_tracker.cpp
  src/utils.cpp
  src/global_parameters.cpp
  src/hungarian.cpp
  src/kalmanfilter.cpp
)

target_include_directories(tracker_pipeline PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

ament_target_dependencies(tracker_pipeline
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(tracker_pipeline
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
)

rosidl_target_interfaces(tracker_pipeline
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

if(BUILD_TESTING)
  enable_testing()

  add_executable(tracker_tests
    tests/test.cpp
    src/manage_tracker.cpp
    src/utils.cpp
    src/global_parameters.cpp
    src/hungarian.cpp
    src/kalmanfilter.cpp
  )

  target_include_directories(tracker_tests PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
  )

  target_link_libraries(tracker_tests
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
  )

  add_test(NAME tracker_tests COMMAND tracker_tests)
endif()

install(TARGETS
  tracker_pipeline
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