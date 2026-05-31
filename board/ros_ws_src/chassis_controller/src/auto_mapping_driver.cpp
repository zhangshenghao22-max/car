#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

class AutoMappingDriver : public rclcpp::Node
{
public:
  AutoMappingDriver()
  : Node("auto_mapping_driver")
  {
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_cmd");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<double>("publish_rate_hz", 10.0);
    this->declare_parameter<double>("forward_speed", 0.04);
    this->declare_parameter<double>("avoid_forward_speed", 0.025);
    this->declare_parameter<double>("turn_speed", 0.45);
    this->declare_parameter<double>("backup_speed", -0.03);
    this->declare_parameter<double>("front_stop_distance", 0.45);
    this->declare_parameter<double>("front_clear_distance", 0.65);
    this->declare_parameter<double>("emergency_stop_distance", 0.22);
    this->declare_parameter<int>("obstacle_confirm_frames", 3);
    this->declare_parameter<int>("clear_confirm_frames", 5);
    this->declare_parameter<double>("scan_timeout", 1.0);
    this->declare_parameter<double>("stop_pause_duration", 0.35);
    this->declare_parameter<double>("commit_turn_duration", 1.2);
    this->declare_parameter<double>("avoid_commit_duration", 3.0);
    this->declare_parameter<double>("avoid_max_duration", 10.0);
    this->declare_parameter<double>("backup_duration", 0.8);
    this->declare_parameter<double>("front_sector_deg", 55.0);
    this->declare_parameter<double>("side_sector_min_deg", 35.0);
    this->declare_parameter<double>("side_sector_max_deg", 120.0);
    this->declare_parameter<double>("front_quantile", 0.20);
    this->declare_parameter<int>("min_front_points", 4);
    this->declare_parameter<bool>("return_heading_enabled", true);
    this->declare_parameter<double>("return_heading_tolerance", 0.12);
    this->declare_parameter<double>("return_heading_max_duration", 4.0);
    this->declare_parameter<double>("return_heading_abort_distance", 0.8);
    this->declare_parameter<double>("odom_timeout", 1.0);
    this->declare_parameter<double>("pass_front_distance", 1.0);
    this->declare_parameter<double>("pass_side_front_distance", 0.8);
    this->declare_parameter<double>("pass_side_distance", 0.6);
    this->declare_parameter<int>("pass_confirm_frames", 8);
    this->declare_parameter<double>("pass_min_duration", 1.0);
    this->declare_parameter<bool>("enabled", true);

    scan_topic_ = this->get_parameter("scan_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    publish_rate_hz_ = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    forward_speed_ = this->get_parameter("forward_speed").as_double();
    avoid_forward_speed_ = this->get_parameter("avoid_forward_speed").as_double();
    turn_speed_ = std::abs(this->get_parameter("turn_speed").as_double());
    backup_speed_ = this->get_parameter("backup_speed").as_double();
    front_stop_distance_ = this->get_parameter("front_stop_distance").as_double();
    front_clear_distance_ = this->get_parameter("front_clear_distance").as_double();
    emergency_stop_distance_ = this->get_parameter("emergency_stop_distance").as_double();
    obstacle_confirm_frames_ = std::max(1, static_cast<int>(this->get_parameter("obstacle_confirm_frames").as_int()));
    clear_confirm_frames_ = std::max(1, static_cast<int>(this->get_parameter("clear_confirm_frames").as_int()));
    scan_timeout_ = this->get_parameter("scan_timeout").as_double();
    stop_pause_duration_ = this->get_parameter("stop_pause_duration").as_double();
    commit_turn_duration_ = this->get_parameter("commit_turn_duration").as_double();
    avoid_commit_duration_ = this->get_parameter("avoid_commit_duration").as_double();
    avoid_max_duration_ = this->get_parameter("avoid_max_duration").as_double();
    backup_duration_ = this->get_parameter("backup_duration").as_double();
    front_sector_rad_ = deg_to_rad(this->get_parameter("front_sector_deg").as_double());
    side_sector_min_rad_ = deg_to_rad(this->get_parameter("side_sector_min_deg").as_double());
    side_sector_max_rad_ = deg_to_rad(this->get_parameter("side_sector_max_deg").as_double());
    front_quantile_ = clamp(this->get_parameter("front_quantile").as_double(), 0.05, 0.50);
    min_front_points_ = std::max(1, static_cast<int>(this->get_parameter("min_front_points").as_int()));
    return_heading_enabled_ = this->get_parameter("return_heading_enabled").as_bool();
    return_heading_tolerance_ = std::max(0.02, std::abs(this->get_parameter("return_heading_tolerance").as_double()));
    return_heading_max_duration_ = std::max(0.5, this->get_parameter("return_heading_max_duration").as_double());
    return_heading_abort_distance_ = std::max(0.1, this->get_parameter("return_heading_abort_distance").as_double());
    odom_timeout_ = std::max(0.2, this->get_parameter("odom_timeout").as_double());
    pass_front_distance_ = std::max(0.2, this->get_parameter("pass_front_distance").as_double());
    pass_side_front_distance_ = std::max(0.2, this->get_parameter("pass_side_front_distance").as_double());
    pass_side_distance_ = std::max(0.2, this->get_parameter("pass_side_distance").as_double());
    pass_confirm_frames_ = std::max(1, static_cast<int>(this->get_parameter("pass_confirm_frames").as_int()));
    pass_min_duration_ = std::max(0.0, this->get_parameter("pass_min_duration").as_double());
    enabled_ = this->get_parameter("enabled").as_bool();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10).best_effort());
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&AutoMappingDriver::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&AutoMappingDriver::odom_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AutoMappingDriver::control_loop, this));

    state_enter_time_ = this->now();
    avoid_enter_time_ = state_enter_time_;
    return_heading_enter_time_ = state_enter_time_;
    last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_odom_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    RCLCPP_INFO(this->get_logger(),
      "Auto mapping driver started: scan=%s cmd=%s odom=%s forward=%.3f avoid_forward=%.3f turn=%.3f return_heading=%s",
      scan_topic_.c_str(), cmd_vel_topic_.c_str(), odom_topic_.c_str(),
      forward_speed_, avoid_forward_speed_, turn_speed_, return_heading_enabled_ ? "true" : "false");
  }

  ~AutoMappingDriver()
  {
    publish_stop();
  }

private:
  enum class State
  {
    FORWARD,
    STOP_PAUSE,
    COMMIT_TURN,
    WALL_FOLLOW,
    RETURN_HEADING,
    RECOVERY_BACKUP
  };

  struct ScanSummary
  {
    bool valid = false;
    int front_count = 0;
    double front_min = std::numeric_limits<double>::infinity();
    double front_p20 = std::numeric_limits<double>::infinity();
    double left_score = 0.0;
    double right_score = 0.0;
    double left_front = 0.0;
    double right_front = 0.0;
    double left_side = 0.0;
    double right_side = 0.0;
  };

  static double clamp(double value, double low, double high)
  {
    return std::min(std::max(value, low), high);
  }

  static double deg_to_rad(double degrees)
  {
    return degrees * M_PI / 180.0;
  }

  static double normalize_angle(double angle)
  {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  const char * state_name(State state) const
  {
    switch (state) {
      case State::FORWARD: return "FORWARD";
      case State::STOP_PAUSE: return "STOP_PAUSE";
      case State::COMMIT_TURN: return "COMMIT_TURN";
      case State::WALL_FOLLOW: return "WALL_FOLLOW";
      case State::RETURN_HEADING: return "RETURN_HEADING";
      case State::RECOVERY_BACKUP: return "RECOVERY_BACKUP";
    }
    return "UNKNOWN";
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
    last_scan_time_ = this->now();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_heading_ = yaw_from_quaternion(msg->pose.pose.orientation);
    last_odom_time_ = this->now();
    has_odom_heading_ = true;
  }

  void control_loop()
  {
    if (!enabled_) {
      publish_stop();
      return;
    }

    sensor_msgs::msg::LaserScan::SharedPtr scan;
    rclcpp::Time scan_time(0, 0, this->get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = latest_scan_;
      scan_time = last_scan_time_;
    }

    const auto now = this->now();
    if (!scan || (now - scan_time).seconds() > scan_timeout_) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No recent /scan, auto mapping stopped");
      return;
    }

    const ScanSummary summary = summarize_scan(*scan);
    if (!summary.valid) {
      publish_stop();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No valid lidar ranges, auto mapping stopped");
      return;
    }

    update_confirm_counters(summary);
    log_diag(summary);

    if (is_emergency(summary)) {
      if (state_ == State::FORWARD && !has_return_heading_target_) {
        capture_return_heading_target(now);
      }
      enter_state(State::RECOVERY_BACKUP);
      publish_backup();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Emergency backup: state=%s front_p20=%.2f front_min=%.2f",
        state_name(state_), summary.front_p20, summary.front_min);
      return;
    }

    switch (state_) {
      case State::FORWARD:
        if (obstacle_confirm_count_ >= obstacle_confirm_frames_) {
          choose_turn_direction(summary);
          pass_confirm_count_ = 0;
          capture_return_heading_target(now);
          enter_state(State::STOP_PAUSE);
          publish_stop();
          RCLCPP_INFO(this->get_logger(),
            "Obstacle confirmed: front_p20=%.2f left=%.2f right=%.2f direction=%s",
            summary.front_p20, summary.left_score, summary.right_score,
            turn_direction_ > 0.0 ? "left" : "right");
        } else {
          publish_forward();
        }
        break;

      case State::STOP_PAUSE:
        publish_stop();
        if ((now - state_enter_time_).seconds() >= stop_pause_duration_) {
          enter_state(State::COMMIT_TURN);
        }
        break;

      case State::COMMIT_TURN:
        if ((now - state_enter_time_).seconds() >= commit_turn_duration_) {
          enter_state(State::WALL_FOLLOW);
          avoid_enter_time_ = now;
        } else {
          publish_turn();
        }
        break;

      case State::WALL_FOLLOW:
        handle_wall_follow(summary, now);
        break;

      case State::RETURN_HEADING:
        handle_return_heading(now);
        break;

      case State::RECOVERY_BACKUP:
        if ((now - state_enter_time_).seconds() >= backup_duration_) {
          choose_turn_direction(summary);
          enter_state(State::STOP_PAUSE);
        } else {
          publish_backup();
        }
        break;
    }
  }

  void handle_wall_follow(const ScanSummary& summary, const rclcpp::Time& now)
  {
    const double avoid_elapsed = (now - avoid_enter_time_).seconds();
    const bool passed = obstacle_passed(summary, now);

    if (passed) {
      if (begin_return_heading(now)) {
        RCLCPP_INFO(this->get_logger(),
          "Obstacle passed by lidar: front=%.2f side_front=%.2f side=%.2f pass=%d, returning heading",
          summary.front_p20, active_side_front(summary), active_side(summary), pass_confirm_count_);
      } else {
        enter_state(State::FORWARD);
        publish_forward();
        RCLCPP_INFO(this->get_logger(),
          "Obstacle passed by lidar: front=%.2f side_front=%.2f side=%.2f pass=%d",
          summary.front_p20, active_side_front(summary), active_side(summary), pass_confirm_count_);
      }
      return;
    }

    if (avoid_elapsed >= avoid_max_duration_) {
      enter_state(State::RECOVERY_BACKUP);
      publish_backup();
      RCLCPP_WARN(this->get_logger(),
        "Avoid timeout: front_p20=%.2f, backing up before retry", summary.front_p20);
      return;
    }

    publish_wall_follow();
  }

  double active_side_front(const ScanSummary& summary) const
  {
    return turn_direction_ > 0.0 ? summary.left_front : summary.right_front;
  }

  double active_side(const ScanSummary& summary) const
  {
    return turn_direction_ > 0.0 ? summary.left_side : summary.right_side;
  }

  bool obstacle_passed(const ScanSummary& summary, const rclcpp::Time& now)
  {
    const double avoid_elapsed = (now - avoid_enter_time_).seconds();
    if (avoid_elapsed < pass_min_duration_) {
      pass_confirm_count_ = 0;
      return false;
    }

    const bool front_clear = summary.front_p20 >= pass_front_distance_;
    const bool side_front_clear = active_side_front(summary) >= pass_side_front_distance_;
    const bool side_clear = active_side(summary) >= pass_side_distance_;
    if (front_clear && side_front_clear && side_clear) {
      pass_confirm_count_ = std::min(pass_confirm_count_ + 1, pass_confirm_frames_);
    } else {
      pass_confirm_count_ = 0;
    }

    return pass_confirm_count_ >= pass_confirm_frames_;
  }

  bool get_recent_heading(const rclcpp::Time& now, double& heading)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_heading_) {
      return false;
    }
    if ((now - last_odom_time_).seconds() > odom_timeout_) {
      return false;
    }
    heading = current_heading_;
    return true;
  }

  void capture_return_heading_target(const rclcpp::Time& now)
  {
    has_return_heading_target_ = false;
    if (!return_heading_enabled_) {
      return;
    }

    double heading = 0.0;
    if (!get_recent_heading(now, heading)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "No recent /odom heading, skip return-heading target capture");
      return;
    }

    target_heading_ = heading;
    has_return_heading_target_ = true;
  }

  bool begin_return_heading(const rclcpp::Time& now)
  {
    if (!return_heading_enabled_ || !has_return_heading_target_) {
      return false;
    }

    double heading = 0.0;
    if (!get_recent_heading(now, heading)) {
      has_return_heading_target_ = false;
      RCLCPP_WARN(this->get_logger(), "No recent /odom heading, skip return-heading stage");
      return false;
    }

    return_heading_enter_time_ = now;
    enter_state(State::RETURN_HEADING);
    handle_return_heading(now);
    return true;
  }

  void handle_return_heading(const rclcpp::Time& now)
  {
    double heading = 0.0;
    if (!get_recent_heading(now, heading)) {
      has_return_heading_target_ = false;
      enter_state(State::FORWARD);
      publish_forward();
      RCLCPP_WARN(this->get_logger(), "Lost /odom heading during return-heading stage, continue forward");
      return;
    }

    const ScanSummary summary = latest_scan_summary();
    if (summary.valid && (
        summary.front_p20 <= return_heading_abort_distance_ ||
        active_side_front(summary) <= pass_side_front_distance_ ||
        active_side(summary) <= pass_side_distance_)) {
      has_return_heading_target_ = false;
      pass_confirm_count_ = 0;
      avoid_enter_time_ = now;
      choose_turn_direction(summary);
      enter_state(State::WALL_FOLLOW);
      publish_wall_follow();
      RCLCPP_WARN(this->get_logger(),
        "Return heading aborted by lidar: front=%.2f side_front=%.2f side=%.2f, continue avoiding",
        summary.front_p20, active_side_front(summary), active_side(summary));
      return;
    }

    const double error = normalize_angle(target_heading_ - heading);
    if (std::abs(error) <= return_heading_tolerance_) {
      has_return_heading_target_ = false;
      enter_state(State::FORWARD);
      publish_forward();
      RCLCPP_INFO(this->get_logger(), "Return heading complete: error=%.3f rad", error);
      return;
    }

    if ((now - return_heading_enter_time_).seconds() >= return_heading_max_duration_) {
      has_return_heading_target_ = false;
      enter_state(State::FORWARD);
      publish_forward();
      RCLCPP_WARN(this->get_logger(), "Return heading timeout: error=%.3f rad, continue forward", error);
      return;
    }

    publish_return_heading(error);
  }

  void update_confirm_counters(const ScanSummary& summary)
  {
    if (summary.front_count < min_front_points_) {
      obstacle_confirm_count_ = 0;
      clear_confirm_count_ = std::min(clear_confirm_count_ + 1, clear_confirm_frames_);
      return;
    }

    if (summary.front_p20 <= front_stop_distance_) {
      obstacle_confirm_count_ = std::min(obstacle_confirm_count_ + 1, obstacle_confirm_frames_);
      clear_confirm_count_ = 0;
    } else if (summary.front_p20 >= front_clear_distance_) {
      clear_confirm_count_ = std::min(clear_confirm_count_ + 1, clear_confirm_frames_);
      obstacle_confirm_count_ = 0;
    } else {
      obstacle_confirm_count_ = 0;
      clear_confirm_count_ = 0;
    }
  }

  bool is_emergency(const ScanSummary& summary) const
  {
    return summary.front_count >= min_front_points_ && summary.front_p20 <= emergency_stop_distance_;
  }

  ScanSummary latest_scan_summary()
  {
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      scan = latest_scan_;
    }
    if (!scan) {
      return ScanSummary();
    }
    return summarize_scan(*scan);
  }

  void choose_turn_direction(const ScanSummary& summary)
  {
    turn_direction_ = (summary.left_score >= summary.right_score) ? 1.0 : -1.0;
  }

  ScanSummary summarize_scan(const sensor_msgs::msg::LaserScan& scan) const
  {
    ScanSummary summary;
    std::vector<double> front_ranges;
    double left_sum = 0.0;
    double right_sum = 0.0;
    double left_front_sum = 0.0;
    double right_front_sum = 0.0;
    double left_side_sum = 0.0;
    double right_side_sum = 0.0;
    int left_count = 0;
    int right_count = 0;
    int left_front_count = 0;
    int right_front_count = 0;
    int left_side_count = 0;
    int right_side_count = 0;

    const double max_usable = std::min(static_cast<double>(scan.range_max), 3.5);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > max_usable) {
        continue;
      }

      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      summary.valid = true;

      if (std::abs(angle) <= front_sector_rad_ * 0.5) {
        front_ranges.push_back(range);
        summary.front_min = std::min(summary.front_min, range);
      }

      if (angle >= side_sector_min_rad_ && angle <= side_sector_max_rad_) {
        left_sum += range;
        ++left_count;
      } else if (angle <= -side_sector_min_rad_ && angle >= -side_sector_max_rad_) {
        right_sum += range;
        ++right_count;
      }

      const double abs_angle = std::abs(angle);
      if (angle >= deg_to_rad(25.0) && angle <= deg_to_rad(70.0)) {
        left_front_sum += range;
        ++left_front_count;
      } else if (angle <= -deg_to_rad(25.0) && angle >= -deg_to_rad(70.0)) {
        right_front_sum += range;
        ++right_front_count;
      } else if (abs_angle > deg_to_rad(70.0) && abs_angle <= deg_to_rad(120.0)) {
        if (angle > 0.0) {
          left_side_sum += range;
          ++left_side_count;
        } else {
          right_side_sum += range;
          ++right_side_count;
        }
      }
    }

    summary.front_count = static_cast<int>(front_ranges.size());
    if (!front_ranges.empty()) {
      std::sort(front_ranges.begin(), front_ranges.end());
      const size_t idx = std::min(
        front_ranges.size() - 1,
        static_cast<size_t>(std::floor(front_quantile_ * static_cast<double>(front_ranges.size() - 1))));
      summary.front_p20 = front_ranges[idx];
    } else {
      summary.front_min = max_usable;
      summary.front_p20 = max_usable;
    }

    summary.left_score = left_count > 0 ? left_sum / static_cast<double>(left_count) : 0.0;
    summary.right_score = right_count > 0 ? right_sum / static_cast<double>(right_count) : 0.0;
    summary.left_front = left_front_count > 0 ? left_front_sum / static_cast<double>(left_front_count) : max_usable;
    summary.right_front = right_front_count > 0 ? right_front_sum / static_cast<double>(right_front_count) : max_usable;
    summary.left_side = left_side_count > 0 ? left_side_sum / static_cast<double>(left_side_count) : max_usable;
    summary.right_side = right_side_count > 0 ? right_side_sum / static_cast<double>(right_side_count) : max_usable;

    return summary;
  }

  void enter_state(State next)
  {
    if (state_ == next) {
      return;
    }
    state_ = next;
    state_enter_time_ = this->now();
  }

  void log_diag(const ScanSummary& summary)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1500,
      "auto_map state=%s front_p20=%.2f front_min=%.2f front_n=%d left=%.2f right=%.2f side_front=%.2f side=%.2f dir=%s obs=%d clear=%d pass=%d",
      state_name(state_), summary.front_p20, summary.front_min, summary.front_count,
      summary.left_score, summary.right_score, active_side_front(summary), active_side(summary),
      turn_direction_ > 0.0 ? "left" : "right", obstacle_confirm_count_, clear_confirm_count_, pass_confirm_count_);
  }

  void publish_forward()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = forward_speed_;
    cmd_vel_pub_->publish(msg);
  }

  void publish_turn()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.angular.z = turn_direction_ * turn_speed_;
    cmd_vel_pub_->publish(msg);
  }

  void publish_wall_follow()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = avoid_forward_speed_;
    msg.angular.z = turn_direction_ * turn_speed_ * 0.65;
    cmd_vel_pub_->publish(msg);
  }

  void publish_return_heading(double heading_error)
  {
    auto msg = geometry_msgs::msg::Twist();
    const double abs_error = std::abs(heading_error);
    const double speed = abs_error < 0.35 ? std::max(0.18, turn_speed_ * 0.55) : turn_speed_;
    msg.angular.z = std::copysign(speed, heading_error);
    cmd_vel_pub_->publish(msg);
  }

  void publish_backup()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = backup_speed_;
    cmd_vel_pub_->publish(msg);
  }

  void publish_stop()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  double publish_rate_hz_;
  double forward_speed_;
  double avoid_forward_speed_;
  double turn_speed_;
  double backup_speed_;
  double front_stop_distance_;
  double front_clear_distance_;
  double emergency_stop_distance_;
  int obstacle_confirm_frames_;
  int clear_confirm_frames_;
  double scan_timeout_;
  double stop_pause_duration_;
  double commit_turn_duration_;
  double avoid_commit_duration_;
  double avoid_max_duration_;
  double backup_duration_;
  double front_sector_rad_;
  double side_sector_min_rad_;
  double side_sector_max_rad_;
  double front_quantile_;
  int min_front_points_;
  bool return_heading_enabled_;
  double return_heading_tolerance_;
  double return_heading_max_duration_;
  double return_heading_abort_distance_;
  double odom_timeout_;
  double pass_front_distance_;
  double pass_side_front_distance_;
  double pass_side_distance_;
  int pass_confirm_frames_;
  double pass_min_duration_;
  bool enabled_;

  State state_ = State::FORWARD;
  double turn_direction_ = 1.0;
  int obstacle_confirm_count_ = 0;
  int clear_confirm_count_ = 0;
  int pass_confirm_count_ = 0;
  rclcpp::Time state_enter_time_;
  rclcpp::Time avoid_enter_time_;
  rclcpp::Time return_heading_enter_time_;
  rclcpp::Time last_scan_time_;
  rclcpp::Time last_odom_time_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;
  std::mutex odom_mutex_;
  double current_heading_ = 0.0;
  double target_heading_ = 0.0;
  bool has_odom_heading_ = false;
  bool has_return_heading_target_ = false;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutoMappingDriver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
