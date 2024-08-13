# Image Compressed ROS Package

## Overview
The `image_compressed` package is a ROS2 node designed to subscribe to image topics, compress the images using JPEG encoding, and publish the compressed images to new topics. This package is useful for reducing the bandwidth usage when transmitting images over a ROS network.

## Features
- Subscribes to all available `sensor_msgs/Image` topics.
- Compresses images using JPEG encoding with configurable quality.
- Publishes the compressed images to new topics with a `/compressed` suffix.
- Automatically updates the list of image topics and handles new or lost topics dynamically.

## Dependencies
- ROS2 (Robot Operating System)
- OpenCV
- cv_bridge

## Installation
1. Make sure you have ROS installed on your system.
2. Install the required dependencies:
    ```bash
    sudo apt-get install ros-<ros-distro>-cv-bridge ros-<ros-distro>-image-transport
    sudo apt-get install libopencv-dev
    ```
3. Clone the `image_compressed` package into your catkin workspace:
    ```bash
    cd ~/colcon_ws/src
    git clone -b ros2 https://github.com/mjlee111/image_compressed.git
    cd ~/colcon_ws
    colcon build --symlink-install
    ```

## Usage
To run the `image_compressed` node, use the following command:
```bash
ros2 run image_compressed image_compressed
```

## Configuration
The JPEG quality for image compression can be configured through a ROS parameter. The default quality is set to 95.

