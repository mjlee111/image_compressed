#ifndef IMAGE_COMPRESSED_HPP
#define IMAGE_COMPRESSED_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <boost/bind.hpp>
#include <string>

class Converter
{
public:
  Converter(int argc, char** argv);
  ~Converter();

  bool init();
  void start();

private:
  int init_argc;
  char** init_argv;

  int quality;

  std::vector<std::string> image_topics;
  std::vector<std::string> previous_image_topics;
  std::vector<ros::Subscriber> image_subscriber;
  std::vector<ros::Publisher> image_publisher;

  void readParameters();
  void updateTopicList();
  void initializeSubscriber();
  void initializePublisher();
  void convertImageToCompressedImage(const sensor_msgs::ImageConstPtr& msg, int num);
};

#endif