#include "autoware_misc/miivii_status_monitor/miivii_status_monitor.hpp"

namespace autoware_misc
{
MiiviiStatusMonitorNode::MiiviiStatusMonitorNode(const rclcpp::NodeOptions & options)
: Node("miivii_status_monitor_node", options), timeout_duration_(std::chrono::milliseconds(1000))
{
  // Declare and get parameters
  image_topics_ =
    this->declare_parameter<std::vector<std::string>>("image_topics", std::vector<std::string>());
  camerainfo_topics_ = this->declare_parameter<std::vector<std::string>>(
    "camerainfo_topics", std::vector<std::string>());
  pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "");
  can_topic_ = this->declare_parameter<std::string>("can_topic", "");

  // Initialize status flags
  has_images_ = !image_topics_.empty();
  has_camerainfo_ = !camerainfo_topics_.empty();
  has_pointcloud_ = !pointcloud_topic_.empty();
  has_can_ = !can_topic_.empty();

  RCLCPP_INFO(this->get_logger(), "has_images_ = %d", has_images_);

  if (has_images_) {
    for (size_t i = 0; i < image_topics_.size(); ++i) {
      image_subs_.emplace_back(this->create_subscription<sensor_msgs::msg::Image>(
        image_topics_[i], 10,
        [this, i](const sensor_msgs::msg::Image::SharedPtr msg) { image_callback(i, msg); }));
      last_image_time_.emplace_back(this->now());
    }
  }

  if (has_camerainfo_) {
    for (size_t i = 0; i < camerainfo_topics_.size(); ++i) {
      camerainfo_subs_.emplace_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camerainfo_topics_[i], 10, [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
          camerainfo_callback(i, msg);
        }));
      last_camerainfo_time_.emplace_back(this->now());
    }
  }

  if (has_pointcloud_) {
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10,
      std::bind(&MiiviiStatusMonitorNode::pointcloud_callback, this, std::placeholders::_1));
    last_pointcloud_time_ = this->now();
  }

  if (has_can_) {
    can_sub_ = this->create_subscription<radar_msgs::msg::RadarTracks>(
      can_topic_, 10,
      std::bind(&MiiviiStatusMonitorNode::can_callback, this, std::placeholders::_1));
    last_can_time_ = this->now();
  }

  diagnose_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
    "diagnostics", 10,
    std::bind(&MiiviiStatusMonitorNode::diagnose_callback, this, std::placeholders::_1));

  // Setup publishers
  image_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("camera_status", 10);
  pointcloud_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("lidar_status", 10);
  can_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("radar_status", 10);
  miivii_sensors_pub_ = this->create_publisher<std_msgs::msg::String>("miivii_sensors_status", 10);

  // Setup timer for periodic status check
  timer_ = this->create_wall_timer(
    timeout_duration_, std::bind(&MiiviiStatusMonitorNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "MiiviiStatusMonitorNode initialized successfully.");
}

// Callback for image topics
void MiiviiStatusMonitorNode::image_callback(
  size_t index, const sensor_msgs::msg::Image::SharedPtr msg)
{
  (void)msg;
  if (index < last_image_time_.size()) {
    last_image_time_[index] = this->now();
  }
}

void MiiviiStatusMonitorNode::camerainfo_callback(
  size_t index, const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  (void)msg;
  if (index < last_camerainfo_time_.size()) {
    last_camerainfo_time_[index] = this->now();
  }
}

// Callback for point cloud messages
void MiiviiStatusMonitorNode::pointcloud_callback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  (void)msg;
  last_pointcloud_time_ = this->now();
}

// Callback for CAN messages
void MiiviiStatusMonitorNode::can_callback(const radar_msgs::msg::RadarTracks::SharedPtr msg)
{
  (void)msg;
  last_can_time_ = this->now();
}

// Callback for diagnostic messages
void MiiviiStatusMonitorNode::diagnose_callback(
  const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
{
  if (
    msg->name == "MiiviiGmslCamera" &&
    msg->level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
    camera_error_flag_ = false;
  }
}

// Timer callback to evaluate and publish status
void MiiviiStatusMonitorNode::timer_callback()
{
  auto current_time = this->now();
  bool camera_status = true;
  bool camerainfo_status = true;
  bool pointcloud_status = true;
  bool can_status = true;

  std::string miivii_sensors_status = "0;0;0";

  // Evaluate image topics status
  if (has_images_) {
    camera_status = (current_time - last_image_time_[0] <= timeout_duration_);
    for (size_t i = 1; i < image_topics_.size(); ++i) {
      if (i < last_image_time_.size()) {
        camera_status = camera_status && (current_time - last_image_time_[i] <= timeout_duration_);
      }
    }
  }

  if (has_camerainfo_) {
    camerainfo_status = (current_time - last_camerainfo_time_[0] <= timeout_duration_);
    for (size_t i = 1; i < camerainfo_topics_.size(); ++i) {
      if (i < last_camerainfo_time_.size()) {
        camerainfo_status =
          camerainfo_status && (current_time - last_camerainfo_time_[i] <= timeout_duration_);
      }
    }
  }

  // Evaluate point cloud status
  if (has_pointcloud_) {
    pointcloud_status = (current_time - last_pointcloud_time_ <= timeout_duration_);
  }

  if (has_can_) {
    can_status = (current_time - last_can_time_ <= timeout_duration_);
  }

  // Publish statuses
  if (has_images_ && has_camerainfo_) {
    camera_status = camera_status && camera_error_flag_ && camerainfo_status;
  } else if (has_images_ && !has_camerainfo_) {
    camera_status = camera_status && camera_error_flag_;
  } else if (!has_images_ && has_camerainfo_) {
    camera_status = camerainfo_status;
  } else {
    camera_status = false;
  }
  camera_error_flag_ = true;
  publish_image_status(camera_status);
  publish_pointcloud_status(pointcloud_status);
  publish_can_status(can_status);

  // 根据camera_status, pointcloud_status, can_status 修改miivii_sensors_status的值
  if (camera_status) {
    miivii_sensors_status[0] = '1';
  }
  if (pointcloud_status) {
    miivii_sensors_status[2] = '1';
  }
  if (can_status) {
    miivii_sensors_status[4] = '1';
  }
  publish_sensors_status(miivii_sensors_status);
}

// Publish camera status
void MiiviiStatusMonitorNode::publish_image_status(bool status)
{
  auto msg = std_msgs::msg::Bool();
  msg.data = status;
  image_status_pub_->publish(msg);
}

// Publish point cloud status
void MiiviiStatusMonitorNode::publish_pointcloud_status(bool status)
{
  auto msg = std_msgs::msg::Bool();
  msg.data = status;
  pointcloud_status_pub_->publish(msg);
}

// Publish CAN status
void MiiviiStatusMonitorNode::publish_can_status(bool status)
{
  auto msg = std_msgs::msg::Bool();
  msg.data = status;
  can_status_pub_->publish(msg);
}

// Publish Sensor Status
void MiiviiStatusMonitorNode::publish_sensors_status(std::string sensors_status)
{
  auto msg = std_msgs::msg::String();
  msg.data = sensors_status;
  miivii_sensors_pub_->publish(msg);
}

}  // namespace autoware_misc
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_misc::MiiviiStatusMonitorNode)
