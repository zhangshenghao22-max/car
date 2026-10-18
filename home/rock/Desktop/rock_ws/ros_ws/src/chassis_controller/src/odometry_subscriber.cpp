#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class OdometrySubscriber : public rclcpp::Node
{
public:
  OdometrySubscriber()
  : Node("odometry_subscriber")
  {
    // Declare parameters
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<bool>("verbose", true);
    
    // Get parameters
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    // Handle both bool and string types for verbose parameter
    if (this->get_parameter("verbose").get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
      std::string verbose_str = this->get_parameter("verbose").as_string();
      verbose_ = (verbose_str == "true" || verbose_str == "True" || verbose_str == "1");
    } else {
      verbose_ = this->get_parameter("verbose").as_bool();
    }
    
    // Create subscription to odometry topic
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic,
      10,
      std::bind(&OdometrySubscriber::odom_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(this->get_logger(), "Odometry subscriber started");
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", odom_topic.c_str());
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract pose information
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    
    // Extract orientation (quaternion)
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    
    // Extract twist (velocity) information
    double linear_x = msg->twist.twist.linear.x;
    double linear_y = msg->twist.twist.linear.y;
    double linear_z = msg->twist.twist.linear.z;
    double angular_x = msg->twist.twist.angular.x;
    double angular_y = msg->twist.twist.angular.y;
    double angular_z = msg->twist.twist.angular.z;
    
    if (verbose_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,  // Print every 1 second
        "Odometry received:\n"
        "  Pose: x=%.3f, y=%.3f, z=%.3f\n"
        "  Orientation: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f\n"
        "  Linear velocity: x=%.3f, y=%.3f, z=%.3f\n"
        "  Angular velocity: x=%.3f, y=%.3f, z=%.3f",
        x, y, z,
        qx, qy, qz, qw,
        linear_x, linear_y, linear_z,
        angular_x, angular_y, angular_z
      );
    }
    
    // Store latest odometry for potential use by other nodes
    latest_odom_ = *msg;
  }
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  nav_msgs::msg::Odometry latest_odom_;
  bool verbose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometrySubscriber>());
  rclcpp::shutdown();
  return 0;
}

