cmake_minimum_required(VERSION 3.5)
project(qr_code_position_detector)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED)

# Set C++ standard to 14
set(CMAKE_CXX_STANDARD 14)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${px4_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  /usr/include/zbar  # Add the zbar include directory here
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

# Add the executable
add_executable(qr_code_position_detector
  src/qr_code_position_detector.cpp
)

# Add dependencies
ament_target_dependencies(qr_code_position_detector
  rclcpp
  px4_msgs
  sensor_msgs
  cv_bridge
  pcl_conversions
  pcl_ros
)

target_link_libraries(qr_code_position_detector
  ${OpenCV_LIBRARIES}
)


# Link against zbar library
target_link_libraries(qr_code_position_detector
  zbar  # Link against the zbar library
)

# Install the executable
install(TARGETS
  qr_code_position_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Create the ament package
ament_package()