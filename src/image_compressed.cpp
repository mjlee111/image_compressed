#include "../include/image_compressed/image_compressed.hpp"

int main(int argc, char** argv)
{
  Converter iC(argc, argv);
  return true;
}

Converter::Converter(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
  std::cout << "[image] Start Converting ..." << std::endl;
  init();
}

Converter::~Converter()
{
  std::cout << "[image] Ros shutdown" << std::endl;
}

bool Converter::init()
{
  ros::init(init_argc, init_argv, "image_coverter_node");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();
  updateTopicList();

  ros::NodeHandle nh;

  start();
  return true;
}

void Converter::start()
{
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    updateTopicList();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Converter::initializeSubscriber()
{
  ros::NodeHandle nh;
  for (int i = 0; i < image_topics.size(); i++)
  {
    image_subscriber.push_back(nh.subscribe<sensor_msgs::Image>(
        image_topics[i], 1, boost::bind(&Converter::convertImageToCompressedImage, this, _1, i)));
  }
}

void Converter::initializePublisher()
{
  ros::NodeHandle nh;
  for (int i = 0; i < image_topics.size(); i++)
  {
    std::string topic_index = image_topics[i] + "/compressed";
    image_publisher.push_back(nh.advertise<sensor_msgs::CompressedImage>(topic_index, 1));
  }
}

void Converter::updateTopicList()
{
  ros::master::V_TopicInfo topics;
  std::vector<std::string> new_image_topics;

  if (ros::master::getTopics(topics))
  {
    for (const auto& topic : topics)
    {
      if (topic.datatype == "sensor_msgs/Image")
      {
        new_image_topics.push_back(topic.name);
      }
    }

    for (const auto& topic : previous_image_topics)
    {
      if (std::find(new_image_topics.begin(), new_image_topics.end(), topic) == new_image_topics.end())
      {
        std::cout << "[image] Lost Topic: " << topic << " Publishing Stopped." << std::endl;
      }
    }

    for (const auto& topic : new_image_topics)
    {
      if (std::find(previous_image_topics.begin(), previous_image_topics.end(), topic) == previous_image_topics.end())
      {
        std::cout << "[image] Found New Topic: " << topic << " Publishing topic as: " << topic << "/compressed"
                  << std::endl;
      }
    }

    previous_image_topics = new_image_topics;
    image_topics = new_image_topics;

    initializeSubscriber();
    initializePublisher();
  }
  else
  {
    ROS_ERROR("[image] Failed to retrieve topics.");
  }
}

void Converter::convertImageToCompressedImage(const sensor_msgs::ImageConstPtr& msg, int num)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(95);

    sensor_msgs::CompressedImage compressed_msg;
    compressed_msg.header = msg->header;
    compressed_msg.format = "jpeg";

    cv::imencode(".jpg", cv_ptr->image, compressed_msg.data, params);
    image_publisher[num].publish(compressed_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("[image] cv_bridge exception: %s", e.what());
  }
}