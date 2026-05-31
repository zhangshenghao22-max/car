#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop()
  : Node("keyboard_teleop")
  {
    // Declare parameters
    // Velocity limits: linear.x/y: -0.9~0.9, angular.z: -0.9~0.9
    // Recommended: linear 0.2 or -0.2, angular 0.3 or -0.3
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<double>("linear_velocity_step", 0.1);
    this->declare_parameter<double>("angular_velocity_step", 0.1);
    this->declare_parameter<double>("max_linear_x", 0.9);
    this->declare_parameter<double>("max_linear_y", 0.9);
    this->declare_parameter<double>("max_angular_z", 0.9);
    // Get parameters (handle both string and double types from launch file)
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    
    // Helper lambda to get double parameter (handles both string and double)
    auto get_double_param = [this](const std::string& name, double /* default_val */) -> double {
      auto param = this->get_parameter(name);
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return std::stod(param.as_string());
      }
      return param.as_double();
    };
    
    linear_step_ = get_double_param("linear_velocity_step", 0.1);
    angular_step_ = get_double_param("angular_velocity_step", 0.1);
    max_linear_x_ = get_double_param("max_linear_x", 0.9);
    max_linear_y_ = get_double_param("max_linear_y", 0.9);
    max_angular_z_ = get_double_param("max_angular_z", 0.9);
    
    // Use default QoS (RELIABLE) to ensure compatibility with microROS
    // microROS typically uses RELIABLE QoS, so we must match it
    // Queue depth reduced from 10 to 5 for lower latency while maintaining compatibility
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      5  // Reduced queue depth for lower latency
    );
    
    // Initialize velocity values
    current_linear_x_ = 0.0;
    current_linear_y_ = 0.0;
    current_angular_z_ = 0.0;
    
    // Setup terminal for non-blocking input
    setup_terminal();
    
    RCLCPP_INFO(this->get_logger(), "Keyboard teleop node started");
    RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", cmd_vel_topic.c_str());
    print_instructions();
    
    // Start keyboard input thread
    keyboard_thread_ = std::thread(&KeyboardTeleop::keyboard_loop, this);
    
    // Note: Removed continuous publishing timer
    // Now only publishes when key is pressed (one-time publish)
    // This allows obstacle avoidance node to take control when needed
    RCLCPP_INFO(this->get_logger(), "Keyboard teleop: Publishing only on key press (not continuous)");
  }
  
  ~KeyboardTeleop()
  {
    restore_terminal();
    if (keyboard_thread_.joinable()) {
      keyboard_thread_.join();
    }
  }

private:
  void setup_terminal()
  {
    // Get current terminal settings
    tcgetattr(STDIN_FILENO, &old_terminal_);
    new_terminal_ = old_terminal_;
    
    // Disable canonical mode and echo
    new_terminal_.c_lflag &= ~(ICANON | ECHO);
    new_terminal_.c_cc[VMIN] = 0;
    new_terminal_.c_cc[VTIME] = 0;
    
    // Apply new terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &new_terminal_);
    
    // Set stdin to non-blocking
    old_flags_ = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
  }
  
  void restore_terminal()
  {
    // Restore terminal settings
    tcsetattr(STDIN_FILENO, TCSANOW, &old_terminal_);
    fcntl(STDIN_FILENO, F_SETFL, old_flags_);
  }
  
  void print_instructions()
  {
    std::cout << "\n";
    std::cout << "========================================\n";
    std::cout << "Keyboard Teleop Control\n";
    std::cout << "========================================\n";
    std::cout << "Movement controls:\n";
    std::cout << "  w/s : Increase/Decrease linear X velocity (forward/backward)\n";
    std::cout << "  a/d : Increase/Decrease linear Y velocity (left/right)\n";
    std::cout << "  q/e : Increase/Decrease angular Z velocity (rotate left/right)\n";
    std::cout << "  x   : Stop all movement\n";
    std::cout << "  r   : Reset all velocities to zero\n";
    std::cout << "  i   : Print current velocities\n";
    std::cout << "  h   : Print this help message\n";
    std::cout << "  Ctrl+C : Exit\n";
    std::cout << "========================================\n";
    std::cout << "\n";
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
  
  void process_key(char key)
  {
    bool velocity_changed = false;
    
    // Protect velocity modifications with mutex
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    
    switch (key) {
      case 'w':
      case 'W':
        current_linear_x_ = std::min(current_linear_x_ + linear_step_, max_linear_x_);
        RCLCPP_INFO(this->get_logger(), "Linear X: %.2f", current_linear_x_);
        velocity_changed = true;
        break;
        
      case 's':
      case 'S':
        current_linear_x_ = std::max(current_linear_x_ - linear_step_, -max_linear_x_);
        RCLCPP_INFO(this->get_logger(), "Linear X: %.2f", current_linear_x_);
        velocity_changed = true;
        break;
        
      case 'a':
      case 'A':
        current_linear_y_ = std::min(current_linear_y_ + linear_step_, max_linear_y_);
        RCLCPP_INFO(this->get_logger(), "Linear Y: %.2f", current_linear_y_);
        velocity_changed = true;
        break;
        
      case 'd':
      case 'D':
        current_linear_y_ = std::max(current_linear_y_ - linear_step_, -max_linear_y_);
        RCLCPP_INFO(this->get_logger(), "Linear Y: %.2f", current_linear_y_);
        velocity_changed = true;
        break;
        
      case 'q':
      case 'Q':
        current_angular_z_ = std::min(current_angular_z_ + angular_step_, max_angular_z_);
        RCLCPP_INFO(this->get_logger(), "Angular Z: %.2f", current_angular_z_);
        velocity_changed = true;
        break;
        
      case 'e':
      case 'E':
        current_angular_z_ = std::max(current_angular_z_ - angular_step_, -max_angular_z_);
        RCLCPP_INFO(this->get_logger(), "Angular Z: %.2f", current_angular_z_);
        velocity_changed = true;
        break;
        
      case 'x':
      case 'X':
        current_linear_x_ = 0.0;
        current_linear_y_ = 0.0;
        current_angular_z_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopped all movement");
        velocity_changed = true;
        break;
        
      case 'r':
      case 'R':
        current_linear_x_ = 0.0;
        current_linear_y_ = 0.0;
        current_angular_z_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Reset all velocities to zero");
        velocity_changed = true;
        break;
        
      case 'i':
      case 'I':
        RCLCPP_INFO(this->get_logger(), 
          "Current velocities - Linear X: %.2f, Linear Y: %.2f, Angular Z: %.2f",
          current_linear_x_, current_linear_y_, current_angular_z_);
        break;
        
      case 'h':
      case 'H':
        print_instructions();
        break;
        
      default:
        // Ignore other keys
        break;
    }
    
    // Immediately publish when velocity changes to reduce latency
    // Note: mutex is still held here, which is fine for publish
    if (velocity_changed) {
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = current_linear_x_;
      msg.linear.y = current_linear_y_;
      msg.linear.z = 0.0;
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = current_angular_z_;
      cmd_vel_publisher_->publish(msg);
    }
  }
  
  // Removed publish_cmd_vel() - now only publishes in process_key() when key is pressed
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  
  double current_linear_x_;
  double current_linear_y_;
  double current_angular_z_;
  
  double linear_step_;
  double angular_step_;
  double max_linear_x_;
  double max_linear_y_;
  double max_angular_z_;
  
  std::thread keyboard_thread_;
  struct termios old_terminal_;
  struct termios new_terminal_;
  int old_flags_;
  std::mutex velocity_mutex_;  // Protect velocity data from concurrent access
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

