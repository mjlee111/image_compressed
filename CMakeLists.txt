cmake_minimum_required(VERSION 3.8)
project(image_compressed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Create executable
add_executable(image_compressed src/image_compressed.cpp)

# Specify the include directories
target_include_directories(image_compressed
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Specify the C++ standard
target_compile_features(image_compressed
  PUBLIC
    cxx_std_17
)

# Link dependencies
ament_target_dependencies(image_compressed
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(TARGETS image_compressed
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
