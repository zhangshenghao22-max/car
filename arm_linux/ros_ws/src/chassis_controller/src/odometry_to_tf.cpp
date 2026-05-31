#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class OdometryToTF : public rclcpp::Node
{
public:
  OdometryToTF()
  : Node("odometry_to_tf")
  {
    // Declare parameters
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<double>("publish_rate", 50.0);  // Hz
    this->declare_parameter<bool>("publish_initial_tf", true);
    
    // Get parameters
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    publish_initial_tf_ = this->get_parameter("publish_initial_tf").as_bool();
    
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize transform to identity (will be updated when odom messages arrive)
    latest_transform_.header.frame_id = odom_frame_;
    latest_transform_.child_frame_id = base_frame_;
    latest_transform_.transform.translation.x = 0.0;
    latest_transform_.transform.translation.y = 0.0;
    latest_transform_.transform.translation.z = 0.0;
    latest_transform_.transform.rotation.x = 0.0;
    latest_transform_.transform.rotation.y = 0.0;
    latest_transform_.transform.rotation.z = 0.0;
    latest_transform_.transform.rotation.w = 1.0;
    has_received_odom_ = false;
    
    // Create subscription to odometry topic
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      10,
      std::bind(&OdometryToTF::odom_callback, this, std::placeholders::_1)
    );
    
    // Create timer to periodically publish TF (even if no odom messages received)
    // This ensures TF tree is always available
    if (publish_tf_) {
      publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
        std::bind(&OdometryToTF::publish_tf_timer, this)
      );
      
      // Publish initial identity transform immediately
      if (publish_initial_tf_) {
        publish_initial_transform();
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Odometry to TF converter started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    if (publish_initial_tf_) {
      RCLCPP_INFO(this->get_logger(), "Publishing initial identity transform");
    }
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!publish_tf_) {
      return;
    }
    
    // Update latest transform
    latest_transform_.header.stamp = msg->header.stamp;
    latest_transform_.header.frame_id = odom_frame_;
    latest_transform_.child_frame_id = base_frame_;
    
    // Set translation (position)
    // For base_footprint, z should always be 0 (ground level)
    // For base_link, use actual z from odometry
    latest_transform_.transform.translation.x = msg->pose.pose.position.x;
    latest_transform_.transform.translation.y = msg->pose.pose.position.y;
    // If publishing to base_footprint, ensure z=0 (ground level)
    // This is the standard ROS convention for base_footprint
    if (base_frame_.find("footprint") != std::string::npos) {
      latest_transform_.transform.translation.z = 0.0;
    } else {
      latest_transform_.transform.translation.z = msg->pose.pose.position.z;
    }
    
    // Set rotation (orientation)
    latest_transform_.transform.rotation.x = msg->pose.pose.orientation.x;
    latest_transform_.transform.rotation.y = msg->pose.pose.orientation.y;
    latest_transform_.transform.rotation.z = msg->pose.pose.orientation.z;
    latest_transform_.transform.rotation.w = msg->pose.pose.orientation.w;
    
    has_received_odom_ = true;
    
    // Transform will be published by timer
  }
  
  void publish_tf_timer()
  {
    if (!publish_tf_) {
      return;
    }
    
    // Update timestamp to current time
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
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped latest_transform_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  bool publish_initial_tf_;
  bool has_received_odom_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryToTF>());
  rclcpp::shutdown();
  return 0;
}

