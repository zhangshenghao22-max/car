#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <algorithm>
#include <cmath>

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("linear_velocity_step", 0.1);
    this->declare_parameter<double>("angular_velocity_step", 0.1);
    this->declare_parameter<double>("max_linear_x", 0.9);
    this->declare_parameter<double>("max_linear_y", 0.9);
    this->declare_parameter<double>("max_angular_z", 0.9);
    this->declare_parameter<double>("publish_rate_hz", 10.0);
    this->declare_parameter<double>("linear_accel_limit", 0.12);
    this->declare_parameter<double>("angular_accel_limit", 0.20);

    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    auto get_double_param = [this](const std::string& name) -> double {
      auto param = this->get_parameter(name);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return std::stod(param.as_string());
      }
      return param.as_double();
    };

    linear_step_ = std::abs(get_double_param("linear_velocity_step"));
    angular_step_ = std::abs(get_double_param("angular_velocity_step"));
    max_linear_x_ = std::abs(get_double_param("max_linear_x"));
    max_linear_y_ = std::abs(get_double_param("max_linear_y"));
    max_angular_z_ = std::abs(get_double_param("max_angular_z"));
    publish_rate_hz_ = std::max(1.0, get_double_param("publish_rate_hz"));
    linear_accel_limit_ = std::max(0.01, std::abs(get_double_param("linear_accel_limit")));
    angular_accel_limit_ = std::max(0.01, std::abs(get_double_param("angular_accel_limit")));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 5);

    target_linear_x_ = 0.0;
    target_linear_y_ = 0.0;
    target_angular_z_ = 0.0;
    output_linear_x_ = 0.0;
    output_linear_y_ = 0.0;
    output_angular_z_ = 0.0;

    setup_terminal();

    RCLCPP_INFO(this->get_logger(), "Keyboard teleop node started");
    RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(),
      "Continuous mode: rate=%.1f Hz, linear_accel=%.3f m/s^2, angular_accel=%.3f rad/s^2",
      publish_rate_hz_, linear_accel_limit_, angular_accel_limit_);
    print_instructions();

    keyboard_thread_ = std::thread(&KeyboardTeleop::keyboard_loop, this);
    last_publish_time_ = this->now();
    auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&KeyboardTeleop::publish_cmd_vel, this));
  }

  ~KeyboardTeleop()
  {
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      target_linear_x_ = 0.0;
      target_linear_y_ = 0.0;
      target_angular_z_ = 0.0;
      output_linear_x_ = 0.0;
      output_linear_y_ = 0.0;
      output_angular_z_ = 0.0;
      publish_current_locked();
    }
    restore_terminal();
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }

private:
  void setup_terminal()
  {
    tcgetattr(STDIN_FILENO, &old_terminal_);
    new_terminal_ = old_terminal_;
    new_terminal_.c_lflag &= ~(ICANON | ECHO);
    new_terminal_.c_cc[VMIN] = 0;
    new_terminal_.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal_);
    old_flags_ = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
  }

  void restore_terminal()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_);
  }

  void print_instructions()
  {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "Keyboard Teleop Control\n";
    std::cout << "========================================\n";
    std::cout << "Mode: press once to set target velocity; robot keeps moving until x/r/Ctrl+C\n";
    std::cout << "Movement controls:\n";
    std::cout << "  w/s : Increase/Decrease linear X target (forward/backward)\n";
    std::cout << "  a/d : Increase/Decrease linear Y target (left/right)\n";
    std::cout << "  q/e : Increase/Decrease angular Z target (rotate left/right)\n";
    std::cout << "  x   : Stop all movement\n";
    std::cout << "  r   : Reset all velocities to zero\n";
    std::cout << "  i   : Print target/output velocities\n";
    std::cout << "  h   : Print this help message\n";
    std::cout << "  Ctrl+C : Exit and send zero velocity\n";
    std::cout << "========================================\n\n";
  }

  void keyboard_loop()
  {
    char key;
    while (rclcpp::ok()) {
      if (read(STDIN_FILENO, &key, 1) > 0) {
        process_key(key);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  static double approach(double current, double target, double max_delta)
  {
    const double diff = target - current;
    if (std::abs(diff) <= max_delta) {
      return target;
    }
    return current + std::copysign(max_delta, diff);
  }

  void process_key(char key)
  {
    bool target_changed = false;
    bool immediate_stop = false;

    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      switch (key) {
        case 'w':
        case 'W':
          target_linear_x_ = std::min(target_linear_x_ + linear_step_, max_linear_x_);
          target_changed = true;
          break;
        case 's':
        case 'S':
          target_linear_x_ = std::max(target_linear_x_ - linear_step_, -max_linear_x_);
          target_changed = true;
          break;
        case 'a':
        case 'A':
          target_linear_y_ = std::min(target_linear_y_ + linear_step_, max_linear_y_);
          target_changed = true;
          break;
        case 'd':
        case 'D':
          target_linear_y_ = std::max(target_linear_y_ - linear_step_, -max_linear_y_);
          target_changed = true;
          break;
        case 'q':
        case 'Q':
          target_angular_z_ = std::min(target_angular_z_ + angular_step_, max_angular_z_);
          target_changed = true;
          break;
        case 'e':
        case 'E':
          target_angular_z_ = std::max(target_angular_z_ - angular_step_, -max_angular_z_);
          target_changed = true;
          break;
        case 'x':
        case 'X':
        case 'r':
        case 'R':
          target_linear_x_ = 0.0;
          target_linear_y_ = 0.0;
          target_angular_z_ = 0.0;
          output_linear_x_ = 0.0;
          output_linear_y_ = 0.0;
          output_angular_z_ = 0.0;
          target_changed = true;
          immediate_stop = true;
          break;
        case 'i':
        case 'I':
          RCLCPP_INFO(this->get_logger(),
            "Target vx=%.2f vy=%.2f wz=%.2f | Output vx=%.2f vy=%.2f wz=%.2f",
            target_linear_x_, target_linear_y_, target_angular_z_,
            output_linear_x_, output_linear_y_, output_angular_z_);
          break;
        case 'h':
        case 'H':
          print_instructions();
          break;
        default:
          break;
      }

      if (target_changed) {
        RCLCPP_INFO(this->get_logger(),
          "Target velocity - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f",
          target_linear_x_, target_linear_y_, target_angular_z_);
        if (immediate_stop) {
          publish_current_locked();
        }
      }
    }
  }

  void publish_cmd_vel()
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);

    const auto now = this->now();
    double dt = (now - last_publish_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1.0 / publish_rate_hz_;
    }
    last_publish_time_ = now;

    output_linear_x_ = approach(output_linear_x_, target_linear_x_, linear_accel_limit_ * dt);
    output_linear_y_ = approach(output_linear_y_, target_linear_y_, linear_accel_limit_ * dt);
    output_angular_z_ = approach(output_angular_z_, target_angular_z_, angular_accel_limit_ * dt);

    publish_current_locked();
  }

  void publish_current_locked()
  {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = output_linear_x_;
    msg.linear.y = output_linear_y_;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = output_angular_z_;
    cmd_vel_publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  double target_linear_x_;
  double target_linear_y_;
  double target_angular_z_;
  double output_linear_x_;
  double output_linear_y_;
  double output_angular_z_;

  double linear_step_;
  double angular_step_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  double publish_rate_hz_;
  double linear_accel_limit_;
  double angular_accel_limit_;

  rclcpp::Time last_publish_time_;
  std::thread keyboard_thread_;
  struct termios old_terminal_;
  struct termios new_terminal_;
  int old_flags_;
  std::mutex velocity_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  // Destroy the node before shutdown so the destructor can publish one final zero Twist.
  node.reset();
  rclcpp::shutdown();
  return 0;
}
