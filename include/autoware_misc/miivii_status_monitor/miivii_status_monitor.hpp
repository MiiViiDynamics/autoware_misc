#ifndef MIIVII_STATUS_MONITOR_HPP_
#define MIIVII_STATUS_MONITOR_HPP_

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <radar_msgs/msg/radar_tracks.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
namespace autoware_misc
{

class MiiviiStatusMonitorNode : public rclcpp::Node
{
public:
  explicit MiiviiStatusMonitorNode(const rclcpp::NodeOptions & options);

private:
  // 订阅者
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camerainfo_subs_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<radar_msgs::msg::RadarTracks>::SharedPtr can_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnose_sub_;

  // 发布者
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr image_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pointcloud_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr can_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr miivii_sensors_pub_;

  // 时间戳
  std::vector<rclcpp::Time> last_image_time_;
  std::vector<rclcpp::Time> last_camerainfo_time_;
  rclcpp::Time last_pointcloud_time_;
  rclcpp::Time last_can_time_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 超时时间
  std::chrono::milliseconds timeout_duration_;

  // 话题名称
  std::vector<std::string> image_topics_;
  std::vector<std::string> camerainfo_topics_;
  std::string pointcloud_topic_;
  std::string can_topic_;

  // 状态标志
  bool has_images_;
  bool has_camerainfo_;
  bool has_pointcloud_;
  bool has_can_;

  bool camera_error_flag_ = true;

  // 回调函数
  void image_callback(size_t index, const sensor_msgs::msg::Image::SharedPtr msg);
  void camerainfo_callback(size_t index, const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void can_callback(const radar_msgs::msg::RadarTracks::SharedPtr msg);
  void diagnose_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg);
  void timer_callback();

  // 状态发布函数
  void publish_image_status(bool status);
  void publish_pointcloud_status(bool status);
  void publish_can_status(bool status);
  void publish_sensors_status(std::string sensors_status);
};
}  // namespace autoware_misc

#endif  // MIIVII_STATUS_MONITOR_HPP_
