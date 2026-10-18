#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>

class ImuToTF : public rclcpp::Node
{
public:
  ImuToTF()
  : Node("imu_to_tf")
  {
    // Declare parameters
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("publish_rate", 50.0);  // Hz
    this->declare_parameter<bool>("publish_initial_tf", true);
    
    // Get parameters
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    publish_initial_tf_ = this->get_parameter("publish_initial_tf").as_bool();
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize transform to identity (will be updated when IMU messages arrive)
    latest_transform_.header.frame_id = odom_frame_;
    latest_transform_.child_frame_id = base_frame_;
    latest_transform_.transform.translation.x = 0.0;
    latest_transform_.transform.translation.y = 0.0;
    latest_transform_.transform.translation.z = 0.0;
    latest_transform_.transform.rotation.x = 0.0;
    latest_transform_.transform.rotation.y = 0.0;
    latest_transform_.transform.rotation.z = 0.0;
    latest_transform_.transform.rotation.w = 1.0;
    has_received_imu_ = false;
    
    // Create subscription to IMU topic
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      10,
      std::bind(&ImuToTF::imu_callback, this, std::placeholders::_1)
    );
    
    // Create timer to periodically publish TF (even if no IMU messages received)
    // This ensures TF tree is always available
    if (publish_tf_) {
      publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
        std::bind(&ImuToTF::publish_tf_timer, this)
      );
      
      // Publish initial identity transform immediately
      if (publish_initial_tf_) {
        publish_initial_transform();
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "IMU to TF converter started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", imu_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    if (publish_initial_tf_) {
      RCLCPP_INFO(this->get_logger(), "Publishing initial identity transform");
    }
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!publish_tf_) {
      return;
    }
    
    // Extract yaw angle from IMU orientation (for 2D SLAM, we only need yaw)
    // IMU provides full 3D orientation (roll, pitch, yaw), but for 2D SLAM
    // we should only use yaw and ignore roll/pitch to prevent drift
    tf2::Quaternion imu_quat;
    imu_quat.setX(msg->orientation.x);
    imu_quat.setY(msg->orientation.y);
    imu_quat.setZ(msg->orientation.z);
    imu_quat.setW(msg->orientation.w);
    
    // Normalize quaternion
    imu_quat.normalize();
    
    // Extract yaw (rotation around Z-axis) from the quaternion
    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);
    
    // Create a quaternion with only yaw rotation (roll=0, pitch=0)
    // This ensures 2D SLAM only uses yaw, preventing roll/pitch from affecting the transform
    tf2::Quaternion yaw_only_quat;
    yaw_only_quat.setRPY(0.0, 0.0, yaw);
    yaw_only_quat.normalize();
    
    // Update latest transform
    latest_transform_.header.frame_id = odom_frame_;
    latest_transform_.child_frame_id = base_frame_;
    
    // Position remains at origin (IMU cannot provide position)
    latest_transform_.transform.translation.x = 0.0;
    latest_transform_.transform.translation.y = 0.0;
    latest_transform_.transform.translation.z = 0.0;
    
    // Use yaw-only quaternion
    latest_transform_.transform.rotation.x = yaw_only_quat.x();
    latest_transform_.transform.rotation.y = yaw_only_quat.y();
    latest_transform_.transform.rotation.z = yaw_only_quat.z();
    latest_transform_.transform.rotation.w = yaw_only_quat.w();
    
    // Use IMU message timestamp (don't change it in timer)
    latest_transform_.header.stamp = msg->header.stamp;
    
    has_received_imu_ = true;
    
    // Transform will be published by timer with updated timestamp
  }
  
  void publish_tf_timer()
  {
    if (!publish_tf_) {
      return;
    }
    
    // Only publish if we have received IMU data
    if (!has_received_imu_) {
      return;
    }
    
    // Update timestamp to current time for TF publishing
    // (TF transforms should use current time, not message timestamp)
    latest_transform_.header.stamp = this->get_clock()->now();
    
    // Send the transform
    tf_broadcaster_->sendTransform(latest_transform_);
  }
  
  void publish_initial_transform()
  {
    if (!publish_tf_) {
      return;
    }
    
    latest_transform_.header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(latest_transform_);
    RCLCPP_INFO(this->get_logger(), "Published initial identity transform: %s -> %s", 
                odom_frame_.c_str(), base_frame_.c_str());
  }
  
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped latest_transform_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool publish_initial_tf_;
  bool has_received_imu_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToTF>());
  rclcpp::shutdown();
  return 0;
}

