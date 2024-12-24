#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>

namespace autoware_misc
{

class ImagePublisherNode : public rclcpp::Node
{
public:
  ImagePublisherNode(const rclcpp::NodeOptions & options)
  : Node("miivii_image_publisher_node", options), publish_count_(0)
  {
    // 从参数服务器获取图像路径
    image_path_ = this->declare_parameter<std::string>("image_path", "");

    // 检查图像路径是否为空
    if (image_path_.empty()) {
      RCLCPP_ERROR(
        this->get_logger(), "Image path is empty. Please set the 'image_path' parameter.");
      rclcpp::shutdown();
      return;
    }

    // 读取图像
    cv_image_ = cv::imread(image_path_, cv::IMREAD_COLOR);
    if (cv_image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read image from path: %s", image_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 将 OpenCV 图像转换为 ROS2 图像消息
    cv_image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cv_image_).toImageMsg();

    // 创建图像发布者
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("autoware_pic", 10);

    // 创建初始定时器，每秒发布一次图像
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&ImagePublisherNode::publishImage, this));

    RCLCPP_INFO(this->get_logger(), "Image loaded and ready to publish");
  }

private:
  void publishImage()
  {
    // 更新图像消息的时间戳
    cv_image_msg_->header.stamp = this->now();

    // 发布图像消息
    image_pub_->publish(*cv_image_msg_);

    // 增加发布计数
    publish_count_++;

    // 如果发布了20次，则改变定时器间隔为10秒
    if (publish_count_ == 20) {
      RCLCPP_INFO(this->get_logger(), "Changing publishing interval to every 10 seconds.");
      timer_->cancel();  // 取消当前定时器
      timer_ = this->create_wall_timer(
        std::chrono::seconds(10), std::bind(&ImagePublisherNode::publishImage, this));
    }
  }

  std::string image_path_;
  cv::Mat cv_image_;
  sensor_msgs::msg::Image::SharedPtr cv_image_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int publish_count_;
};

}  // namespace autoware_misc

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_misc::ImagePublisherNode)
