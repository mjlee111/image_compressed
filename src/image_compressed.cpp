#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class ImageConverter : public rclcpp::Node
{
public:
  ImageConverter() : Node("image_converter_node")
  {
    this->declare_parameter("quality", 95);
    this->get_parameter("quality", quality_);
    RCLCPP_INFO(this->get_logger(), "cv::IMWRITE_JPEG_QUALITY: %d", quality_);

    update_topic_list();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() { this->update_topic_list(); });

    start();
  }

private:
  void start()
  {
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() { this->update_topic_list(); });
  }

  void update_topic_list()
  {
    auto topics = this->get_topic_names_and_types();
    std::vector<std::string> new_image_topics;

    for (const auto& topic : topics)
    {
      if (std::find(topic.second.begin(), topic.second.end(), "sensor_msgs/msg/Image") != topic.second.end())
      {
        new_image_topics.push_back(topic.first);
      }
    }

    for (const auto& topic : previous_image_topics_)
    {
      if (std::find(new_image_topics.begin(), new_image_topics.end(), topic) == new_image_topics.end())
      {
        RCLCPP_INFO(this->get_logger(), "Lost Topic: %s Publishing Stopped.", topic.c_str());
      }
    }

    for (const auto& topic : new_image_topics)
    {
      if (std::find(previous_image_topics_.begin(), previous_image_topics_.end(), topic) ==
          previous_image_topics_.end())
      {
        RCLCPP_INFO(this->get_logger(), "Found New Topic: %s Publishing topic as: %s/compressed", topic.c_str(),
                    topic.c_str());
      }
    }

    previous_image_topics_ = new_image_topics;
    image_topics_ = new_image_topics;

    initialize_subscriber();
    initialize_publisher();
  }

  void initialize_subscriber()
  {
    image_subscribers_.clear();
    for (size_t i = 0; i < image_topics_.size(); i++)
    {
      auto callback = [this, i](const sensor_msgs::msg::Image::SharedPtr msg) {
        this->convert_image_to_compressed_image(msg, i);
      };

      image_subscribers_.emplace_back(
          this->create_subscription<sensor_msgs::msg::Image>(image_topics_[i], 10, callback));
    }
  }

  void initialize_publisher()
  {
    image_publishers_.clear();
    for (size_t i = 0; i < image_topics_.size(); i++)
    {
      std::string topic_index = image_topics_[i] + "/compressed";
      image_publishers_.emplace_back(this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_index, 10));
    }
  }

  void convert_image_to_compressed_image(const sensor_msgs::msg::Image::SharedPtr msg, size_t num)
  {
    try
    {
      auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      std::vector<int> params;
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(quality_);

      sensor_msgs::msg::CompressedImage compressed_msg;
      compressed_msg.header = msg->header;
      compressed_msg.format = "jpeg";

      cv::imencode(".jpg", cv_ptr->image, compressed_msg.data, params);
      image_publishers_[num]->publish(compressed_msg);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  int quality_;
  std::vector<std::string> previous_image_topics_;
  std::vector<std::string> image_topics_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> image_publishers_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageConverter>());
  rclcpp::shutdown();
  return 0;
}
