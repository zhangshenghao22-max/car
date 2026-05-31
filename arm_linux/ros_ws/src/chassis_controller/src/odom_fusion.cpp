#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <cmath>
#include <mutex>

class OdomFusion : public rclcpp::Node
{
public:
  OdomFusion()
  : Node("odom_fusion")
  {
    // Declare parameters
    this->declare_parameter<std::string>("imu_topic", "/imu/data");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("publish_rate", 50.0);  // Hz
    this->declare_parameter<bool>("publish_tf", true);
    this->declare_parameter<bool>("publish_initial_tf", true);
    // Mode selection: "slam" for SLAM mode (use SLAM map->odom for fusion)
    // "navigation" for navigation mode (ignore map->odom from AMCL, avoid feedback loop)
    this->declare_parameter<std::string>("mode", "slam");
    
    // Fusion parameters
    this->declare_parameter<double>("imu_yaw_weight", 0.7);  // Weight for IMU yaw (0-1)
    this->declare_parameter<double>("slam_yaw_weight", 0.3);  // Weight for SLAM yaw (0-1)
    this->declare_parameter<double>("imu_position_weight", 0.0);  // IMU doesn't provide position
    this->declare_parameter<double>("slam_position_weight", 1.0);  // SLAM provides position
    this->declare_parameter<double>("yaw_low_pass_alpha", 0.1);  // Low-pass filter for yaw (0-1, smaller = more smoothing)
    this->declare_parameter<double>("max_yaw_rate", 2.0);  // Maximum yaw change rate (rad/s) for outlier rejection
    
    // Encoder odometry parameters (reserved for future use)
    this->declare_parameter<bool>("use_encoder_odom", false);  // Reserved: enable encoder odometry
    this->declare_parameter<std::string>("encoder_odom_topic", "/encoder/odom");  // Reserved: encoder odometry topic
    
    // Velocity integration for odometry (use cmd_vel to integrate position)
    this->declare_parameter<bool>("use_velocity_integration", true);  // Enable velocity integration from cmd_vel
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");  // Command velocity topic
    
    // Get parameters
    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    publish_tf_ = this->get_parameter("publish_tf").as_bool();
    publish_initial_tf_ = this->get_parameter("publish_initial_tf").as_bool();
    std::string mode = this->get_parameter("mode").as_string();
    use_slam_fusion_ = (mode == "slam");
    
    imu_yaw_weight_ = this->get_parameter("imu_yaw_weight").as_double();
    slam_yaw_weight_ = this->get_parameter("slam_yaw_weight").as_double();
    imu_position_weight_ = this->get_parameter("imu_position_weight").as_double();
    slam_position_weight_ = this->get_parameter("slam_position_weight").as_double();
    yaw_low_pass_alpha_ = this->get_parameter("yaw_low_pass_alpha").as_double();
    max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
    use_encoder_odom_ = this->get_parameter("use_encoder_odom").as_bool();
    use_velocity_integration_ = this->get_parameter("use_velocity_integration").as_bool();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    
    // Normalize weights
    double total_yaw_weight = imu_yaw_weight_ + slam_yaw_weight_;
    if (total_yaw_weight > 0.0) {
      imu_yaw_weight_ /= total_yaw_weight;
      slam_yaw_weight_ /= total_yaw_weight;
    }
    
    // Initialize TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Initialize state
    fused_x_ = 0.0;
    fused_y_ = 0.0;
    fused_yaw_ = 0.0;
    filtered_yaw_ = 0.0;
    last_imu_yaw_ = 0.0;
    last_slam_yaw_ = 0.0;
    rclcpp::Time init_time = this->get_clock()->now();
    last_update_time_ = init_time;
    last_imu_update_time_ = init_time;  // Initialize to avoid time source mismatch
    has_received_imu_ = false;
    has_received_slam_ = false;
    has_received_cmd_vel_ = false;
    last_cmd_vel_time_ = init_time;
    
    // Initialize transform
    latest_transform_.header.frame_id = odom_frame_;
    latest_transform_.child_frame_id = base_frame_;
    latest_transform_.transform.translation.x = 0.0;
    latest_transform_.transform.translation.y = 0.0;
    latest_transform_.transform.translation.z = 0.0;
    latest_transform_.transform.rotation.x = 0.0;
    latest_transform_.transform.rotation.y = 0.0;
    latest_transform_.transform.rotation.z = 0.0;
    latest_transform_.transform.rotation.w = 1.0;
    
    // Create subscription to IMU topic
    // Use SensorDataQoS to match IMU publisher's QoS (typically BEST_EFFORT for sensor data)
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomFusion::imu_callback, this, std::placeholders::_1)
    );
    
    // Velocity integration subscription (for odometry position calculation)
    // Use cmd_vel to integrate position: position = position + velocity * dt
    // IMPORTANT: Use BEST_EFFORT QoS to match obstacle_avoidance's output publisher
    if (use_velocity_integration_) {
      // In navigation mode, controller_server publishes to /cmd_vel with default QoS
      // We use reliable QoS to ensure we receive all commands
      cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic,
        rclcpp::QoS(10).reliable(),
        std::bind(&OdomFusion::cmd_vel_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(this->get_logger(), "Velocity integration enabled, subscribing to: %s", cmd_vel_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "Odometry position will be calculated by integrating cmd_vel with IMU yaw");
      RCLCPP_INFO(this->get_logger(), "Using reliable QoS for cmd_vel subscription");
    }
    
    // Reserved: Encoder odometry subscription (for future use)
    if (use_encoder_odom_) {
      std::string encoder_topic = this->get_parameter("encoder_odom_topic").as_string();
      encoder_odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        encoder_topic,
        10,
        std::bind(&OdomFusion::encoder_odom_callback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(this->get_logger(), "Encoder odometry enabled, subscribing to: %s", encoder_topic.c_str());
    }
    
    // Create timer to periodically publish TF and update from SLAM
    if (publish_tf_) {
      publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
        std::bind(&OdomFusion::publish_tf_timer, this)
      );
      
      // Publish initial identity transform immediately
      if (publish_initial_tf_) {
        publish_initial_transform();
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Odometry fusion node started");
    RCLCPP_INFO(this->get_logger(), "Mode: %s", use_slam_fusion_ ? "SLAM" : "Navigation");
    RCLCPP_INFO(this->get_logger(), "Position calculation: Velocity integration from cmd_vel (to avoid circular dependency with SLAM/AMCL)");
    RCLCPP_INFO(this->get_logger(), "Yaw source: IMU with low-pass filter (alpha=%.2f)", yaw_low_pass_alpha_);
    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());
    if (use_velocity_integration_) {
      RCLCPP_INFO(this->get_logger(), "Velocity integration enabled from: %s", cmd_vel_topic.c_str());
    }
    RCLCPP_INFO(this->get_logger(), "Publishing TF: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Extract yaw angle from IMU orientation
    tf2::Quaternion imu_quat;
    imu_quat.setX(msg->orientation.x);
    imu_quat.setY(msg->orientation.y);
    imu_quat.setZ(msg->orientation.z);
    imu_quat.setW(msg->orientation.w);
    imu_quat.normalize();

    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_quat).getRPY(roll, pitch, yaw);

    // Normalize yaw to [-pi, pi]
    while (yaw > M_PI) yaw -= 2.0 * M_PI;
    while (yaw < -M_PI) yaw += 2.0 * M_PI;

    // Outlier rejection: check if yaw change is too large
    // Use IMU message timestamp for dt calculation, not node clock
    if (has_received_imu_) {
      double yaw_diff = yaw - last_imu_yaw_;
      // Handle wrap-around: normalize yaw_diff to [-pi, pi]
      if (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
      if (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

      // Use IMU message timestamp instead of node clock
      rclcpp::Time msg_time(msg->header.stamp);
      double dt = 0.0;

      // Check if IMU timestamp is valid (non-zero)
      if (msg_time.nanoseconds() > 0) {
        dt = (msg_time - last_imu_msg_time_).seconds();

        // Store the message timestamp for next comparison
        last_imu_msg_time_ = msg_time;
      } else {
        // Fallback to node clock if IMU timestamp is invalid
        rclcpp::Time now = this->get_clock()->now();
        dt = (now - last_imu_update_time_).seconds();
        last_imu_update_time_ = now;
      }

      // 🔧 FIX 1: 完整的dt合理性检查
      const double dt_min = 0.001;   // 1ms (支持1000Hz IMU)
      const double dt_max = 0.5;     // 500ms (最大允许间隔)

      if (dt < dt_min) {
        // dt too small - likely callback processing delay or duplicate message
        RCLCPP_DEBUG(this->get_logger(),
                    "IMU dt too small (%.6f s), skipping update", dt);
        return;
      }

      if (dt > dt_max) {
        // dt too large - system was suspended or IMU disconnected
        RCLCPP_ERROR(this->get_logger(),
                     "IMU dt too large (%.6f s), system may have been suspended. "
                     "Resetting IMU state to prevent TF explosion. dt=%.6f, yaw_diff=%.6f",
                     dt, dt, yaw_diff);
        // 重置IMU状态，防止TF爆炸
        last_imu_yaw_ = yaw;
        filtered_yaw_ = yaw;
        rclcpp::Time reset_time = msg_time.nanoseconds() > 0 ? msg_time : this->get_clock()->now();
        last_imu_msg_time_ = reset_time;
        last_imu_update_time_ = this->get_clock()->now();
        return;
      }

      // 🔧 FIX 2: 始终检查yaw_rate，不管dt是多少（修复原来只在0.005-0.1范围内检查的BUG）
      double yaw_rate = std::abs(yaw_diff / dt);

      // 自适应阈值：根据dt动态调整阈值
      // dt越大，允许的yaw_rate越小（防止累积误差）
      double adaptive_threshold = max_yaw_rate_;
      if (dt > 0.1) {
        // 如果dt较大，使用更严格的阈值
        adaptive_threshold = max_yaw_rate_ * 0.1 / dt;
        RCLCPP_WARN(this->get_logger(),
                    "Large IMU dt detected (%.6f s), using adaptive yaw_rate threshold: %.2f rad/s",
                    dt, adaptive_threshold);
      }

      if (yaw_rate > adaptive_threshold) {
        // Reject this IMU data - do not update filtered_yaw_
        RCLCPP_WARN(this->get_logger(),
                    "IMU yaw rate too high (%.2f rad/s, threshold: %.2f), rejecting yaw update. "
                    "dt=%.6f, yaw_diff=%.6f. This may indicate IMU malfunction or system overload.",
                    yaw_rate, adaptive_threshold, dt, yaw_diff);
        last_imu_yaw_ = yaw;  // Update to prevent getting stuck on same value
        return;
      }

      // 🔧 FIX 3: 额外的合理性检查 - 检查yaw_diff是否过大
      const double max_yaw_diff = M_PI;  // 180度
      if (std::abs(yaw_diff) > max_yaw_diff) {
        RCLCPP_ERROR(this->get_logger(),
                     "IMU yaw jump too large (%.2f rad, > %.2f rad), rejecting update. "
                     "This may indicate IMU malfunction or data corruption.",
                     std::abs(yaw_diff), max_yaw_diff);
        return;
      }
    } else {
      // First IMU message - initialize time and yaw
      rclcpp::Time msg_time(msg->header.stamp);
      if (msg_time.nanoseconds() > 0) {
        last_imu_msg_time_ = msg_time;
      }
      last_imu_update_time_ = this->get_clock()->now();
      has_received_imu_ = true;
      // Initialize filtered yaw for first message
      filtered_yaw_ = yaw;
      last_imu_yaw_ = yaw;
      return;  // Skip filtered yaw update for first message (already initialized)
    }

    // Update last_imu_yaw_ for next comparison
    last_imu_yaw_ = yaw;

    // Apply low-pass filter to smooth yaw changes
    filtered_yaw_ = yaw_low_pass_alpha_ * yaw + (1.0 - yaw_low_pass_alpha_) * filtered_yaw_;
  }
  
  // Command velocity callback (for velocity integration to calculate odometry position)
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Store latest velocity command
    latest_cmd_vel_ = *msg;
    has_received_cmd_vel_ = true;
    last_cmd_vel_time_ = this->get_clock()->now();

    // Log first cmd_vel message received
    static int cmd_vel_count = 0;
    cmd_vel_count++;
    if (cmd_vel_count == 1) {
      RCLCPP_INFO(this->get_logger(), "✓ First cmd_vel message received! Velocity integration started.");
      RCLCPP_INFO(this->get_logger(), "  Initial cmd_vel: linear=(%.3f, %.3f), angular=%.3f",
                   msg->linear.x, msg->linear.y, msg->angular.z);
    } else if (cmd_vel_count % 100 == 0) {
      // Log every 100th message
      RCLCPP_INFO(this->get_logger(), "cmd_vel statistics: received %d messages, current: linear=(%.3f, %.3f), angular=%.3f",
                   cmd_vel_count, msg->linear.x, msg->linear.y, msg->angular.z);
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Received cmd_vel: linear=(%.3f, %.3f), angular=%.3f",
                   msg->linear.x, msg->linear.y, msg->angular.z);
    }
  }
  
  // Reserved: Encoder odometry callback (for future use)
  void encoder_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // TODO: Implement encoder odometry fusion
    // This is reserved for future implementation
    // For now, just store the data
    latest_encoder_odom_ = *msg;
    has_received_encoder_odom_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "Received encoder odometry (reserved for future use)");
  }
  
  void publish_tf_timer()
  {
    if (!publish_tf_) {
      return;
    }
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    rclcpp::Time now;
    rclcpp::Time prev_update_time = last_update_time_;  // Store previous update time for velocity integration
    try {
      now = this->get_clock()->now();
      // dt is calculated but not used in this branch - this is intentional
      // (void)dt;  // Suppress unused variable warning
      last_update_time_ = now;
    } catch (const std::runtime_error& e) {
      // Handle time source mismatch - reinitialize time
      RCLCPP_WARN(this->get_logger(), "Time source mismatch in timer, reinitializing: %s", e.what());
      now = this->get_clock()->now();
      prev_update_time = last_update_time_;
      last_update_time_ = now;
    }
    
    // Get SLAM pose from TF
    // SLAM publishes map->odom, which we can use to get position information
    // For 2D SLAM, we primarily use IMU for yaw and SLAM for position correction
    // IMPORTANT: In navigation mode, disable SLAM fusion to avoid feedback loop with AMCL
    double slam_x = 0.0, slam_y = 0.0, slam_yaw_correction = 0.0;
    bool slam_available = false;
    
    // Only attempt SLAM fusion if enabled (SLAM mode)
    if (!use_slam_fusion_) {
      // Navigation mode: skip SLAM fusion, only use IMU
      // This prevents feedback loop with AMCL's map->odom
      slam_available = false;
    } else {
      // SLAM mode: attempt to get SLAM data
      try {
      // Get current odom->base_footprint transform (what we're currently publishing)
      geometry_msgs::msg::TransformStamped current_odom_to_base;
      try {
        current_odom_to_base = tf_buffer_->lookupTransform(
          odom_frame_, base_frame_, tf2::TimePointZero);
      } catch (...) {
        // If not available, use identity
        current_odom_to_base.transform.translation.x = fused_x_;
        current_odom_to_base.transform.translation.y = fused_y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, fused_yaw_);
        current_odom_to_base.transform.rotation.x = q.x();
        current_odom_to_base.transform.rotation.y = q.y();
        current_odom_to_base.transform.rotation.z = q.z();
        current_odom_to_base.transform.rotation.w = q.w();
      }
      
      // Get transform from map to odom (published by SLAM)
      // Use tf2::TimePointZero to get latest available transform
      geometry_msgs::msg::TransformStamped map_to_odom = tf_buffer_->lookupTransform(
        map_frame_, odom_frame_, tf2::TimePointZero);
      
      // Get transform from map to base_footprint (if available)
      // IMPORTANT: Only use this in SLAM mode. In navigation mode (AMCL), 
      // AMCL's map->odom is based on our odom->base_footprint, so using it
      // for fusion would create a feedback loop causing coordinate explosion.
      geometry_msgs::msg::TransformStamped map_to_base;
      try {
        map_to_base = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero);
        
        // Check if this is from SLAM or AMCL by examining the transform age
        // In SLAM mode, map->base_footprint should be recent and consistent
        // For now, we'll use it if available, but we'll be cautious about position fusion
        // in navigation mode (see position fusion logic below)
        
        // Compute odom->base_footprint from map->base_footprint and map->odom
        // Manually convert geometry_msgs::Transform to tf2::Transform
        tf2::Transform map_to_odom_tf, map_to_base_tf;
        
        // Convert map_to_odom
        tf2::Quaternion q1;
        q1.setX(map_to_odom.transform.rotation.x);
        q1.setY(map_to_odom.transform.rotation.y);
        q1.setZ(map_to_odom.transform.rotation.z);
        q1.setW(map_to_odom.transform.rotation.w);
        map_to_odom_tf.setRotation(q1);
        map_to_odom_tf.setOrigin(tf2::Vector3(
          map_to_odom.transform.translation.x,
          map_to_odom.transform.translation.y,
          map_to_odom.transform.translation.z));
        
        // Convert map_to_base
        tf2::Quaternion q2;
        q2.setX(map_to_base.transform.rotation.x);
        q2.setY(map_to_base.transform.rotation.y);
        q2.setZ(map_to_base.transform.rotation.z);
        q2.setW(map_to_base.transform.rotation.w);
        map_to_base_tf.setRotation(q2);
        map_to_base_tf.setOrigin(tf2::Vector3(
          map_to_base.transform.translation.x,
          map_to_base.transform.translation.y,
          map_to_base.transform.translation.z));
        
        // odom->base_footprint = (map->odom)^-1 * (map->base_footprint)
        tf2::Transform odom_to_base_tf = map_to_odom_tf.inverse() * map_to_base_tf;
        
        // Extract position and yaw
        tf2::Vector3 translation = odom_to_base_tf.getOrigin();
        slam_x = translation.x();
        slam_y = translation.y();
        
        tf2::Quaternion rotation = odom_to_base_tf.getRotation();
        double roll, pitch, yaw;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
        
        // Normalize yaw
        while (yaw > M_PI) yaw -= 2.0 * M_PI;
        while (yaw < -M_PI) yaw += 2.0 * M_PI;
        
        slam_yaw_correction = yaw;
        // Only enable SLAM fusion if we have map->base_footprint (SLAM mode)
        // Position fusion will be handled carefully in the fusion logic below
        slam_available = true;
        has_received_slam_ = true;
        last_slam_yaw_ = slam_yaw_correction;
        
      } catch (tf2::TransformException& ex) {
        // map->base_footprint not available
        // In navigation mode (AMCL), we should NOT use map->odom from AMCL for fusion
        // because AMCL's map->odom is based on our odom->base_footprint, which would create
        // a feedback loop causing coordinate explosion.
        // Only use map->odom in SLAM mode (when slam_gmapping publishes it).
        // For navigation mode, disable SLAM fusion and only use IMU.
        slam_available = false;
      }
      
      } catch (tf2::TransformException& ex) {
        // SLAM transform not available yet, use IMU only or keep previous state
        slam_available = false;
      }
    }
    
    // IMPORTANT: In navigation mode (use_slam_fusion_=false), we completely skip
    // any attempt to get map->odom or map->base_footprint. This prevents feedback loop
    // with AMCL's map->odom. AMCL's map->odom is based on our odom->base_footprint,
    // so using it for fusion would create a feedback loop causing coordinate explosion.
    
    // Fusion logic
    // Always publish TF to ensure TF tree is available, even if no data received yet
    if (has_received_imu_ || slam_available) {
      // IMPORTANT: In SLAM mode, we should NOT fuse SLAM's yaw angle because:
      // 1. SLAM's map->odom is calculated from odom->base_footprint (which we publish)
      // 2. map->base_footprint = map->odom * odom->base_footprint
      // 3. Using map->base_footprint's yaw would create a circular dependency
      // 4. SLAM relies on our odom->base_footprint (with IMU yaw) to work correctly
      // Therefore, in SLAM mode, we should ONLY use IMU yaw, not SLAM yaw
      // We can use SLAM position (from map->base_footprint) for position fusion
      if (has_received_imu_) {
        // Always use IMU yaw (both in SLAM and navigation mode)
        // In SLAM mode: IMU yaw is used by SLAM to calculate map->odom
        // In navigation mode: IMU yaw is the only source (no SLAM fusion)
        fused_yaw_ = filtered_yaw_;
      } else if (slam_available) {
        // Only SLAM available (should not happen in normal operation, but handle edge case)
        // Use SLAM yaw only if IMU is not available
        fused_yaw_ = slam_yaw_correction;
      }
      
      // Normalize fused yaw
      while (fused_yaw_ > M_PI) fused_yaw_ -= 2.0 * M_PI;
      while (fused_yaw_ < -M_PI) fused_yaw_ += 2.0 * M_PI;
      
      // Fuse position:
      // IMPORTANT: In SLAM mode, DO NOT use SLAM position to avoid circular dependency:
      // 1. SLAM's map->odom depends on odom->base_footprint (which we publish)
      // 2. If we use SLAM position for odom->base_footprint, it creates a feedback loop
      // 3. This feedback loop causes drift and instability
      // Solution: Always use velocity integration in both SLAM and navigation modes
      // In SLAM mode: map->odom from SLAM provides the global position correction
      // In navigation mode: AMCL's map->odom provides the global position
      if (use_velocity_integration_ && has_received_cmd_vel_ && has_received_imu_) {
        // Use velocity integration for odometry position calculation
        // Position update: x = x + (vx * cos(yaw) - vy * sin(yaw)) * dt
        //                  y = y + (vx * sin(yaw) + vy * cos(yaw)) * dt
        // For mecanum wheel robot, cmd_vel.linear.x is forward, linear.y is left, angular.z is rotation
        // Note: In SLAM mode, the robot's global position in map frame is handled by SLAM's map->odom transform
        double vx = latest_cmd_vel_.linear.x;
        double vy = latest_cmd_vel_.linear.y;
        double vel_dt = (now - prev_update_time).seconds();

        // 🔧 FIX 4: 完整的vel_dt检查和异常检测
        const double vel_dt_min = 0.005;   // 5ms最小间隔
        const double vel_dt_max = 0.5;     // 500ms最大间隔（20Hz控制频率）

        if (vel_dt >= vel_dt_min && vel_dt <= vel_dt_max) {
          // dt合理，继续速度积分

          // 检查速度是否合理（防止控制器故障发送异常速度）
          const double max_linear_vel = 2.0;  // 2 m/s最大线速度
          if (std::abs(vx) > max_linear_vel || std::abs(vy) > max_linear_vel) {
            RCLCPP_ERROR(this->get_logger(),
                         "cmd_vel linear velocity too high! vx=%.3f, vy=%.3f (max=%.3f). "
                         "This may indicate controller malfunction. Skipping integration.",
                         vx, vy, max_linear_vel);
            // 不要积分这个异常的速度
          } else {
            // Transform velocity from robot frame to odom frame
            double cos_yaw = std::cos(fused_yaw_);
            double sin_yaw = std::sin(fused_yaw_);

            // For mecanum wheel: vx is forward (robot x), vy is left (robot y)
            // Transform to odom frame: rotate by yaw
            double vx_odom = vx * cos_yaw - vy * sin_yaw;
            double vy_odom = vx * sin_yaw + vy * cos_yaw;

            // 🔧 FIX 5: 检查积分后的位置是否异常
            double new_x = fused_x_ + vx_odom * vel_dt;
            double new_y = fused_y_ + vy_odom * vel_dt;

            // 检查单次积分是否过大（防止爆炸性增长）
            const double max_single_step = 1.0;  // 单次最大移动1米
            double dx = new_x - fused_x_;
            double dy = new_y - fused_y_;
            double step_size = std::sqrt(dx*dx + dy*dy);

            if (step_size > max_single_step) {
              RCLCPP_ERROR(this->get_logger(),
                           "Velocity integration step too large! step=%.3fm (max=%.3fm). "
                           "vx_odom=%.3f, vy_odom=%.3f, vel_dt=%.6f. Skipping this integration.",
                           step_size, max_single_step, vx_odom, vy_odom, vel_dt);
            } else {
              // 检查位置是否异常（防止累积误差导致位置爆炸）
              const double max_position = 1000.0;  // 1公里范围
              if (std::abs(new_x) > max_position || std::abs(new_y) > max_position) {
                RCLCPP_ERROR(this->get_logger(),
                             "Position out of bounds! x=%.3f, y=%.3f (max=%.3f). "
                             "Resetting position to origin. This may indicate TF explosion.",
                             new_x, new_y, max_position);
                fused_x_ = 0.0;
                fused_y_ = 0.0;
              } else {
                // 正常积分
                fused_x_ = new_x;
                fused_y_ = new_y;
              }
            }
          }
        } else if (vel_dt > vel_dt_max) {
          // dt太大，可能是系统卡顿
          RCLCPP_WARN(this->get_logger(),
                      "Velocity integration dt too large (%.6f s). Skipping integration to prevent drift.",
                      vel_dt);
        }
        // 如果vel_dt太小（< vel_dt_min），跳过这次积分（高频更新）
      } else {
        // No velocity integration available: keep position at origin (0, 0)
        // The robot's global position in map frame will be handled by:
        // - SLAM mode: SLAM's map->odom transform
        // - Navigation mode: AMCL's map->odom transform
        fused_x_ = 0.0;
        fused_y_ = 0.0;
      }
      // TODO: When encoder odometry is implemented, fuse encoder position here as an alternative to velocity integration
    }
    // If no data received yet, keep using identity transform (fused_x_=0, fused_y_=0, fused_yaw_=0)
    
    // Always publish transform to ensure TF tree is available
    // Create quaternion from fused yaw
    tf2::Quaternion fused_quat;
    fused_quat.setRPY(0.0, 0.0, fused_yaw_);
    fused_quat.normalize();
    
    // Update transform
    latest_transform_.header.stamp = now;
    latest_transform_.transform.translation.x = fused_x_;
    latest_transform_.transform.translation.y = fused_y_;
    latest_transform_.transform.translation.z = 0.0;
    latest_transform_.transform.rotation.x = fused_quat.x();
    latest_transform_.transform.rotation.y = fused_quat.y();
    latest_transform_.transform.rotation.z = fused_quat.z();
    latest_transform_.transform.rotation.w = fused_quat.w();
    
    // Publish transform (always publish to ensure TF tree is available)
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
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;  // For velocity integration
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_odom_subscription_;  // Reserved
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  
  geometry_msgs::msg::TransformStamped latest_transform_;
  nav_msgs::msg::Odometry latest_encoder_odom_;  // Reserved
  
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  
  // State variables
  double fused_x_, fused_y_, fused_yaw_;
  double filtered_yaw_;
  double last_imu_yaw_, last_slam_yaw_;
  rclcpp::Time last_update_time_, last_imu_update_time_;
  rclcpp::Time last_imu_msg_time_;  // Track actual IMU message timestamps
  
  // Parameters
  double imu_yaw_weight_, slam_yaw_weight_;
  double imu_position_weight_, slam_position_weight_;
  double yaw_low_pass_alpha_;
  double max_yaw_rate_;
  bool use_encoder_odom_;
  bool publish_tf_;
  bool publish_initial_tf_;
  
  // Flags
  bool has_received_imu_;
  bool has_received_slam_;
  bool has_received_cmd_vel_;  // For velocity integration
  bool has_received_encoder_odom_;  // Reserved
  bool use_slam_fusion_;  // If false, disable SLAM fusion (navigation mode)
  bool use_velocity_integration_;  // Enable velocity integration from cmd_vel
  
  // Velocity integration state
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  
  std::mutex state_mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomFusion>());
  rclcpp::shutdown();
  return 0;
}

