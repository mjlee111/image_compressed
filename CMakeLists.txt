cmake_minimum_required(VERSION 3.0.2)
project(image_compressed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_compressed_node src/image_compressed.cpp)

target_link_libraries(image_compressed_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
