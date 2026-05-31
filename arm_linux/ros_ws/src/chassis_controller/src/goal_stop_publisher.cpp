#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>

/**
 * @brief Node that publishes stop command when navigation goal is reached
 *
 * This node monitors the navigation action status and sends a stop command
 * immediately after the goal is reached to ensure the robot stops completely.
 *
 * Improved version with minimum goal duration check to avoid false triggers.
 */
class GoalStopPublisher : public rclcpp::Node
{
public:
  GoalStopPublisher()
  : Node("goal_stop_publisher")
  {
    // Declare parameters
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_cmd");
    this->declare_parameter<double>("stop_duration", 0.5);  // How long to send stop command (seconds)
    this->declare_parameter<int>("stop_repeats", 3);  // How many times to repeat stop command
    this->declare_parameter<std::string>("action_status_topic", "/navigate_to_pose/_action/status");
    this->declare_parameter<double>("min_goal_duration", 2.0);  // Minimum goal duration before sending stop (seconds)

    // Get parameters
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    stop_duration_ = this->get_parameter("stop_duration").as_double();
    stop_repeats_ = this->get_parameter("stop_repeats").as_int();
    std::string action_status_topic = this->get_parameter("action_status_topic").as_string();
    min_goal_duration_ = this->get_parameter("min_goal_duration").as_double();

    // Create publisher for stop command (publish to cmd_vel_cmd before obstacle_avoidance)
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      rclcpp::QoS(10).best_effort()
    );

    // Subscribe to action status to monitor goal states
    action_status_subscriber_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
      action_status_topic,
      rclcpp::QoS(10),
      std::bind(&GoalStopPublisher::action_status_callback, this, std::placeholders::_1)
    );

    // State variables
    last_goal_active_ = false;
    stop_timer_active_ = false;
    goal_start_time_ = rclcpp::Time(0);

    RCLCPP_INFO(this->get_logger(), "Goal Stop Publisher node started");
    RCLCPP_INFO(this->get_logger(), "Publishing stop commands to: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Monitoring action status: %s", action_status_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Stop duration: %.2f seconds, Repeats: %d", stop_duration_, stop_repeats_);
    RCLCPP_INFO(this->get_logger(), "Minimum goal duration: %.2f seconds", min_goal_duration_);
  }

private:
  void action_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    auto current_time = this->get_clock()->now();

    // Check if any goal is currently active
    bool goal_active = false;
    bool goal_just_completed = false;

    for (const auto& status : msg->status_list) {
      // Check if goal is active (executing or pending)
      if (status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED) {
        if (!last_goal_active_) {
          // Goal just started, record start time
          goal_start_time_ = current_time;
          RCLCPP_DEBUG(this->get_logger(), "Navigation goal started at %.3f", goal_start_time_.seconds());
        }
        goal_active = true;
        last_goal_active_ = true;
      }
      // Check if goal just succeeded
      else if (status.status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
        if (last_goal_active_) {
          goal_just_completed = true;
          RCLCPP_INFO(this->get_logger(), "Navigation goal reached! Sending stop command...");
        }
        last_goal_active_ = false;
      }
      // Check if goal failed or was aborted
      else if (status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED ||
               status.status == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
        if (last_goal_active_) {
          // Only send stop command if goal was active long enough
          auto goal_duration = (current_time - goal_start_time_).seconds();
          if (goal_duration >= min_goal_duration_) {
            goal_just_completed = true;
            RCLCPP_WARN(this->get_logger(), "Navigation goal %s after %.2f seconds. Sending stop command...",
                        status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED ? "aborted" : "canceled",
                        goal_duration);
          } else {
            RCLCPP_DEBUG(this->get_logger(), "Ignoring %s - goal duration (%.2f s) < minimum (%.2f s)",
                        status.status == action_msgs::msg::GoalStatus::STATUS_ABORTED ? "abort" : "cancel",
                        goal_duration, min_goal_duration_);
          }
        }
        last_goal_active_ = false;
      }
    }

    // If no goals are active but we previously had an active goal, send stop command
    if (!goal_active && last_goal_active_ && msg->status_list.empty()) {
      auto goal_duration = (current_time - goal_start_time_).seconds();
      if (goal_duration >= min_goal_duration_) {
        goal_just_completed = true;
        last_goal_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Navigation goal completed after %.2f seconds. Sending stop command...", goal_duration);
      } else {
        last_goal_active_ = false;
        RCLCPP_DEBUG(this->get_logger(), "Ignoring completion - goal duration (%.2f s) < minimum (%.2f s)",
                    goal_duration, min_goal_duration_);
      }
    }

    // Send stop command if goal just completed
    if (goal_just_completed && !stop_timer_active_) {
      send_stop_commands();
    }
  }

  void send_stop_commands()
  {
    stop_timer_active_ = true;

    // Send stop command multiple times to ensure it's received
    for (int i = 0; i < stop_repeats_; ++i) {
      // Create stop message
      auto stop_msg = geometry_msgs::msg::Twist();
      stop_msg.linear.x = 0.0;
      stop_msg.linear.y = 0.0;
      stop_msg.linear.z = 0.0;
      stop_msg.angular.x = 0.0;
      stop_msg.angular.y = 0.0;
      stop_msg.angular.z = 0.0;

      // Publish stop command
      cmd_vel_publisher_->publish(stop_msg);

      RCLCPP_DEBUG(this->get_logger(), "Sent stop command %d/%d", i + 1, stop_repeats_);

      // Small delay between repeats
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(this->get_logger(), "Published %d stop commands", stop_repeats_);

    // Reset flag after stop duration
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(stop_duration_ * 1000)));
      stop_timer_active_ = false;
    }).detach();
  }

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // Subscriber
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_subscriber_;

  // Parameters
  double stop_duration_;
  int stop_repeats_;
  double min_goal_duration_;

  // State
  bool last_goal_active_;
  bool stop_timer_active_;
  rclcpp::Time goal_start_time_;
  std::mutex state_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalStopPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
