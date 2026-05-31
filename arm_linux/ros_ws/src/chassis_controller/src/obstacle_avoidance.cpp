#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// Note: nav2_msgs may not be available in all ROS2 distributions
// If not available, comment out navigation action client functionality
#ifdef HAVE_NAV2_MSGS
#include <nav2_msgs/action/navigate_to_pose.hpp>
#endif
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <cmath>
#include <algorithm>
#include <mutex>

class ObstacleAvoidance : public rclcpp::Node
{
public:
  ObstacleAvoidance()
  : Node("obstacle_avoidance"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare parameters
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("cmd_vel_input_topic", "/cmd_vel_cmd");  // Input topic for commands
    this->declare_parameter<std::string>("cmd_vel_output_topic", "/cmd_vel");    // Output topic to chassis
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<std::string>("laser_frame", "laser");
    
    // Obstacle detection parameters
    this->declare_parameter<double>("safety_distance", 0.3);  // Minimum safe distance (meters)
    this->declare_parameter<double>("detection_range", 1.0);  // Detection range (meters)
    this->declare_parameter<double>("detection_angle", M_PI / 3.0);  // Detection angle (radians, 60 degrees)
    this->declare_parameter<double>("stop_distance", 0.5);  // Distance to trigger stop (meters)
    
    // Control parameters
    this->declare_parameter<double>("check_rate", 20.0);  // Check rate (Hz)
    this->declare_parameter<bool>("enable_in_navigation", true);  // Enable in navigation mode
    this->declare_parameter<bool>("enable_in_slam", true);  // Enable in SLAM mode
    
    // Navigation integration
    this->declare_parameter<std::string>("bt_navigator_action", "navigate_to_pose");
    this->declare_parameter<bool>("trigger_replan_on_obstacle", true);  // Trigger replanning in navigation mode
    
    // Get parameters
    std::string scan_topic = this->get_parameter("scan_topic").as_string();
    std::string cmd_vel_input_topic = this->get_parameter("cmd_vel_input_topic").as_string();
    std::string cmd_vel_output_topic = this->get_parameter("cmd_vel_output_topic").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    
    safety_distance_ = this->get_parameter("safety_distance").as_double();
    detection_range_ = this->get_parameter("detection_range").as_double();
    detection_angle_ = this->get_parameter("detection_angle").as_double();
    stop_distance_ = this->get_parameter("stop_distance").as_double();
    
    double check_rate = this->get_parameter("check_rate").as_double();
    enable_in_navigation_ = this->get_parameter("enable_in_navigation").as_bool();
    enable_in_slam_ = this->get_parameter("enable_in_slam").as_bool();
    
    std::string bt_navigator_action = this->get_parameter("bt_navigator_action").as_string();
    trigger_replan_on_obstacle_ = this->get_parameter("trigger_replan_on_obstacle").as_bool();
    
    // Create subscribers
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidance::scan_callback, this, std::placeholders::_1)
    );

    // Create subscriber for velocity commands (from Nav2 or keyboard_teleop)
    // Use BEST_EFFORT QoS to match Nav2 controller's default QoS policy
    cmd_vel_input_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_input_topic,
      rclcpp::QoS(10).best_effort(),
      std::bind(&ObstacleAvoidance::cmd_vel_callback, this, std::placeholders::_1)
    );

    // Create publisher for velocity commands (to chassis or micro-ROS)
    // IMPORTANT: Use BEST_EFFORT QoS to match micro-ROS底盘的QoS
    // If QoS doesn't match, the chassis won't receive the commands!
    cmd_vel_output_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_output_topic,
      rclcpp::QoS(10).best_effort()
    );
    
    // Create action client for navigation (to trigger replanning)
    // Note: Nav2 will automatically handle replanning when cmd_vel is zero
    // We don't need to explicitly call any action here
    // The controller will detect zero velocity and trigger recovery behaviors
    
    // Create timer for periodic obstacle checking
    check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / check_rate)),
      std::bind(&ObstacleAvoidance::check_obstacles, this)
    );
    
    // State variables
    obstacle_detected_ = false;
    last_obstacle_time_ = rclcpp::Time(0);
    last_scan_time_ = rclcpp::Time(0);
    
    RCLCPP_INFO(this->get_logger(), "Obstacle avoidance node started");
    RCLCPP_INFO(this->get_logger(), "Command input topic: %s (BEST_EFFORT QoS)", cmd_vel_input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Command output topic: %s (BEST_EFFORT QoS to match chassis)", cmd_vel_output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Safety distance: %.2f m", safety_distance_);
    RCLCPP_INFO(this->get_logger(), "Detection range: %.2f m", detection_range_);
    RCLCPP_INFO(this->get_logger(), "Detection angle: %.2f deg", detection_angle_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "Stop distance: %.2f m", stop_distance_);
    RCLCPP_INFO(this->get_logger(), "Enabled in navigation: %s", enable_in_navigation_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "Enabled in SLAM: %s", enable_in_slam_ ? "true" : "false");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
    last_scan_time_ = this->get_clock()->now();
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);

    // Store the latest command
    latest_cmd_vel_ = msg;
    last_cmd_vel_time_ = this->get_clock()->now();

    // Check if we should forward this command or stop
    if (obstacle_detected_) {
      // Obstacle detected - publish stop command
      publish_stop_command();
    } else {
      // No obstacle - forward the command to output topic
      cmd_vel_output_publisher_->publish(*msg);
    }
  }

  void check_obstacles()
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    
    // Check if we have recent scan data
    if (!latest_scan_ || (this->get_clock()->now() - last_scan_time_).seconds() > 1.0) {
      // No recent scan data, don't block movement
      if (obstacle_detected_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "No recent scan data, clearing obstacle detection");
        obstacle_detected_ = false;
      }
      return;
    }
    
    // Check if we should be active (based on mode)
    // For now, we're always active if enabled
    // In the future, we could check if we're in navigation or SLAM mode
    
    // Detect obstacles in front of robot
    bool obstacle_found = detect_obstacle_in_front(*latest_scan_);

    if (obstacle_found && !obstacle_detected_) {
      // Obstacle just detected
      obstacle_detected_ = true;
      last_obstacle_time_ = this->get_clock()->now();

      RCLCPP_WARN(this->get_logger(), "OBSTACLE DETECTED! Emergency stop activated.");

      // Immediately publish stop command (highest priority)
      publish_stop_command();

      // In navigation mode, trigger replanning
      if (trigger_replan_on_obstacle_) {
        trigger_navigation_replan();
      }

    } else if (!obstacle_found && obstacle_detected_) {
      // Obstacle cleared
      auto time_since_obstacle = (this->get_clock()->now() - last_obstacle_time_).seconds();
      if (time_since_obstacle > 0.5) {  // Wait 0.5s before clearing to avoid oscillation
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared. Resuming normal operation.");
        obstacle_detected_ = false;
        // Forward the latest command (if any) to resume motion
        if (latest_cmd_vel_ && (this->get_clock()->now() - last_cmd_vel_time_).seconds() < 1.0) {
          cmd_vel_output_publisher_->publish(*latest_cmd_vel_);
        }
      }
    } else if (obstacle_found && obstacle_detected_) {
      // Obstacle still present - keep publishing stop command
      publish_stop_command();
    }
  }
  
  bool detect_obstacle_in_front(const sensor_msgs::msg::LaserScan& scan)
  {
    // Get robot orientation from TF
    double robot_yaw = 0.0;
    try {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_.lookupTransform(
        base_frame_, laser_frame_,
        tf2::TimePointZero,
        std::chrono::milliseconds(100)
      );
      
      // Get laser orientation relative to base
      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
      );
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      robot_yaw = yaw;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                          "Could not transform %s to %s: %s",
                          laser_frame_.c_str(), base_frame_.c_str(), ex.what());
      // Assume laser is forward (0 degrees)
      robot_yaw = 0.0;
    }
    
    // Calculate detection angle range in laser frame
    // Detection angle is centered on robot's forward direction
    double min_angle = -detection_angle_ / 2.0;
    double max_angle = detection_angle_ / 2.0;
    
    // Check each laser reading
    double min_distance = std::numeric_limits<double>::max();
    bool obstacle_in_range = false;
    
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      double angle = scan.angle_min + i * scan.angle_increment;
      
      // Check if this reading is in the detection angle
      if (angle >= min_angle && angle <= max_angle) {
        double range = scan.ranges[i];
        
        // Check if reading is valid and within detection range
        // Note: scan.range_max is float, detection_range_ is double, need explicit cast
        double max_range = std::min(static_cast<double>(scan.range_max), detection_range_);
        if (std::isfinite(range) && 
            range >= scan.range_min && 
            range <= max_range) {
          
          min_distance = std::min(min_distance, range);
          
          // Check if obstacle is too close
          if (range <= stop_distance_) {
            obstacle_in_range = true;
          }
        }
      }
    }
    
    if (obstacle_in_range) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Obstacle detected at distance: %.2f m (min: %.2f m)",
                           min_distance, stop_distance_);
    }
    
    return obstacle_in_range;
  }
  
  void publish_stop_command()
  {
    // Publish zero velocity command (emergency stop)
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.linear.z = 0.0;
    stop_msg.angular.x = 0.0;
    stop_msg.angular.y = 0.0;
    stop_msg.angular.z = 0.0;

    cmd_vel_output_publisher_->publish(stop_msg);

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Published emergency stop command");
  }
  
  void trigger_navigation_replan()
  {
    // In navigation mode, we can trigger replanning by canceling current goal
    // and letting Nav2 replan automatically
    // However, Nav2 should handle this automatically when cmd_vel is zero
    
    // Alternative: We could call a service to trigger recovery behavior
    // For now, we just log that replanning should occur
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "Obstacle detected in navigation mode. Nav2 should replan automatically.");
    
    // Note: Nav2's controller will detect that cmd_vel is zero and trigger recovery
    // We don't need to explicitly call any service here
  }
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_input_subscription_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_output_publisher_;

  // Action clients (reserved for future use)
  // Note: Nav2 automatically handles replanning when cmd_vel is zero

  // Timers
  rclcpp::TimerBase::SharedPtr check_timer_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Parameters
  std::string base_frame_;
  std::string laser_frame_;
  double safety_distance_;
  double detection_range_;
  double detection_angle_;
  double stop_distance_;
  bool enable_in_navigation_;
  bool enable_in_slam_;
  bool trigger_replan_on_obstacle_;

  // State
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  std::mutex scan_mutex_;
  std::mutex cmd_mutex_;
  bool obstacle_detected_;
  rclcpp::Time last_obstacle_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_cmd_vel_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleAvoidance>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

