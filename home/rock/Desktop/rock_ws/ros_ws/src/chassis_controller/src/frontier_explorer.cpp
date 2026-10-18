#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "chassis_controller/scan_sector_summary.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <cctype>
#include <limits>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer"),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("odom_topic", "/odom");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_cmd");
    this->declare_parameter<std::string>("status_topic", "/frontier_explorer/status");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("publish_rate_hz", 5.0);
    this->declare_parameter<double>("replan_interval", 3.0);
    this->declare_parameter<double>("map_timeout", 4.0);
    this->declare_parameter<double>("scan_timeout", 1.5);
    this->declare_parameter<double>("odom_timeout", 2.0);
    this->declare_parameter<int>("free_threshold", 25);
    this->declare_parameter<int>("occupied_threshold", 65);
    this->declare_parameter<double>("obstacle_padding", 0.15);
    this->declare_parameter<int>("min_frontier_cells", 8);
    this->declare_parameter<double>("min_goal_distance", 0.45);
    this->declare_parameter<double>("max_goal_distance", 9.0);
    this->declare_parameter<double>("unknown_complete_ratio", 0.025);
    this->declare_parameter<double>("complete_confirm_duration", 8.0);
    this->declare_parameter<double>("goal_tolerance", 0.28);
    this->declare_parameter<double>("lookahead_distance", 0.65);
    this->declare_parameter<double>("max_linear_speed", 0.08);
    this->declare_parameter<double>("min_linear_speed", 0.03);
    this->declare_parameter<double>("max_angular_speed", 0.25);
    this->declare_parameter<double>("yaw_kp", 0.9);
    this->declare_parameter<double>("rotate_in_place_yaw", 0.55);
    this->declare_parameter<double>("front_stop_distance", 0.50);
    this->declare_parameter<double>("hard_stop_distance", 0.20);
    this->declare_parameter<bool>("disable_hard_stop_in_mapping", true);
    this->declare_parameter<bool>("escape_always_try_motion", true);
    this->declare_parameter<double>("front_emergency_distance", 0.28);
    this->declare_parameter<double>("front_block_distance", 0.45);
    this->declare_parameter<double>("front_clear_distance", 0.60);
    this->declare_parameter<int>("front_block_confirm_frames", 3);
    this->declare_parameter<int>("front_clear_confirm_frames", 5);
    this->declare_parameter<bool>("narrow_creep_enabled", true);
    this->declare_parameter<double>("narrow_creep_speed", 0.025);
    this->declare_parameter<double>("narrow_creep_min_front_p20", 0.35);
    this->declare_parameter<double>("narrow_creep_p50_distance", 0.55);
    this->declare_parameter<double>("angular_deadband", 0.035);
    this->declare_parameter<double>("angular_flip_suppression_yaw", 0.25);
    this->declare_parameter<double>("angular_sign_hold_duration", 0.8);
    this->declare_parameter<double>("angular_command_max_delta", 0.10);
    this->declare_parameter<double>("turn_lock_duration", 2.5);
    this->declare_parameter<double>("turn_switch_margin", 0.12);
    this->declare_parameter<double>("turn_score_ema_alpha", 0.35);
    this->declare_parameter<double>("oscillation_window", 5.0);
    this->declare_parameter<int>("oscillation_flip_limit", 3);
    this->declare_parameter<double>("anti_oscillation_reverse_duration", 1.0);
    this->declare_parameter<double>("anti_oscillation_turn_duration", 2.0);
    this->declare_parameter<double>("target_min_hold_duration", 6.0);
    this->declare_parameter<double>("target_switch_score_margin", 0.35);
    this->declare_parameter<double>("target_switch_distance_threshold", 0.55);
    this->declare_parameter<double>("target_switch_yaw_threshold", 0.75);
    this->declare_parameter<bool>("corner_turn_away_enabled", true);
    this->declare_parameter<double>("corner_front_diagonal_obstacle_p20", 0.42);
    this->declare_parameter<double>("corner_turn_block_margin", 0.08);
    this->declare_parameter<double>("corner_turn_max_toward_obstacle", 0.04);
    this->declare_parameter<double>("corner_turn_away_speed", 0.06);
    this->declare_parameter<double>("corner_creep_speed", 0.025);
    this->declare_parameter<bool>("narrow_maze_follow_enabled", true);
    this->declare_parameter<double>("maze_side_wall_max_p50", 0.55);
    this->declare_parameter<double>("maze_front_min_continue", 0.28);
    this->declare_parameter<double>("maze_front_p20_continue", 0.34);
    this->declare_parameter<double>("maze_forward_speed", 0.022);
    this->declare_parameter<double>("maze_center_kp", 0.45);
    this->declare_parameter<double>("maze_front_center_kp", 0.35);
    this->declare_parameter<double>("maze_side_error_weight", 0.55);
    this->declare_parameter<double>("maze_front_diagonal_error_weight", 0.45);
    this->declare_parameter<double>("maze_center_lookahead_min_delta", 0.04);
    this->declare_parameter<double>("maze_max_angular", 0.10);
    this->declare_parameter<double>("maze_turn_lock_duration", 1.8);
    this->declare_parameter<double>("maze_exit_clear_p20", 0.58);
    this->declare_parameter<double>("maze_inner_clearance_min_p20", 0.22);
    this->declare_parameter<double>("maze_inner_clearance_release_p20", 0.28);
    this->declare_parameter<double>("maze_front_diagonal_clearance_min_p20", 0.24);
    this->declare_parameter<double>("maze_turn_speed", 0.07);
    this->declare_parameter<double>("maze_turn_forward_speed", 0.014);
    this->declare_parameter<double>("maze_pre_turn_outer_bias_duration", 0.8);
    this->declare_parameter<double>("maze_pre_turn_outer_bias_angular", 0.05);
    this->declare_parameter<double>("maze_turn_escape_reverse_duration", 0.7);
    this->declare_parameter<double>("maze_turn_escape_reverse_speed", 0.022);
    this->declare_parameter<double>("maze_min_hold_duration", 3.0);
    this->declare_parameter<double>("maze_center_deadband", 0.025);
    this->declare_parameter<double>("maze_center_ema_alpha", 0.30);
    this->declare_parameter<double>("maze_angular_max_delta", 0.05);
    this->declare_parameter<double>("maze_turn_commit_duration", 2.4);
    this->declare_parameter<double>("maze_turn_max_continuous_duration", 2.0);
    this->declare_parameter<double>("maze_post_turn_straight_duration", 0.8);
    this->declare_parameter<double>("maze_min_turn_radius", 0.32);
    this->declare_parameter<double>("maze_corner_front_delta", 0.06);
    this->declare_parameter<bool>("maze_only_mode", false);
    this->declare_parameter<std::string>("maze_route_mode", "local_sector");
    this->declare_parameter<bool>("maze_topology_mode", false);
    this->declare_parameter<double>("maze_topology_forward_speed", 0.055);
    this->declare_parameter<double>("maze_topology_fast_speed", 0.065);
    this->declare_parameter<double>("maze_topology_turn_speed", 0.13);
    this->declare_parameter<double>("maze_topology_turn_forward_speed", 0.035);
    this->declare_parameter<int>("maze_opening_confirm_frames", 4);
    this->declare_parameter<double>("maze_junction_dwell_duration", 0.4);
    this->declare_parameter<double>("maze_node_merge_distance", 0.35);
    this->declare_parameter<double>("maze_opening_visited_distance", 0.45);
    this->declare_parameter<double>("maze_deadend_front_p20", 0.32);
    this->declare_parameter<int>("maze_deadend_confirm_frames", 3);
    this->declare_parameter<double>("maze_deadend_side_open_p20", 0.45);
    this->declare_parameter<bool>("maze_deadend_turnaround_enabled", true);
    this->declare_parameter<double>("maze_deadend_turn_speed", 0.16);
    this->declare_parameter<double>("maze_deadend_turn_yaw", 3.05);
    this->declare_parameter<double>("maze_deadend_turn_yaw_tolerance", 0.18);
    this->declare_parameter<double>("maze_deadend_turn_timeout", 7.0);
    this->declare_parameter<double>("maze_deadend_in_place_min_side_p20", 0.30);
    this->declare_parameter<double>("maze_deadend_in_place_min_front_p20", 0.26);
    this->declare_parameter<double>("maze_deadend_in_place_min_rear_p20", 0.24);
    this->declare_parameter<double>("maze_deadend_reverse_to_turn_spot_speed", 0.035);
    this->declare_parameter<double>("maze_deadend_reverse_to_turn_spot_distance", 0.25);
    this->declare_parameter<double>("maze_deadend_reverse_to_turn_spot_timeout", 3.0);
    this->declare_parameter<double>("maze_deadend_reverse_chunk_distance", 0.20);
    this->declare_parameter<double>("maze_deadend_reverse_total_max_distance", 0.75);
    this->declare_parameter<double>("maze_reverse_rear_stop_p20", 0.26);
    this->declare_parameter<double>("maze_reverse_rear_release_p20", 0.34);
    this->declare_parameter<double>("maze_deadend_front_drive_clear_p20", 0.38);
    this->declare_parameter<double>("maze_deadend_front_drive_clear_p50", 0.45);
    this->declare_parameter<int>("maze_deadend_front_drive_clear_confirm_frames", 3);
    this->declare_parameter<double>("maze_deadend_turn_search_direction_timeout", 3.0);
    this->declare_parameter<double>("maze_deadend_turn_search_max_yaw", 2.4);
    this->declare_parameter<int>("maze_deadend_turn_search_max_switches", 2);
    this->declare_parameter<bool>("maze_open_side_turn_enabled", true);
    this->declare_parameter<double>("maze_open_side_turn_min_score", 0.42);
    this->declare_parameter<double>("maze_open_side_turn_score_margin", 0.08);
    this->declare_parameter<double>("maze_open_side_front_clear_p20", 0.34);
    this->declare_parameter<double>("maze_open_side_front_clear_p50", 0.42);
    this->declare_parameter<int>("maze_open_side_confirm_frames", 2);
    this->declare_parameter<double>("maze_open_side_reselect_timeout", 2.0);
    this->declare_parameter<bool>("maze_no_reverse_mode", true);
    this->declare_parameter<double>("maze_rotate_search_speed", 0.16);
    this->declare_parameter<double>("maze_rotate_search_direction_timeout", 2.5);
    this->declare_parameter<double>("maze_rotate_search_total_timeout", 8.0);
    this->declare_parameter<int>("maze_rotate_search_max_switches", 1);
    this->declare_parameter<double>("maze_rotate_front_clear_p20", 0.34);
    this->declare_parameter<double>("maze_rotate_front_clear_p50", 0.42);
    this->declare_parameter<int>("maze_rotate_front_clear_confirm_frames", 2);
    this->declare_parameter<double>("maze_graph_front_obstacle_p20", 0.34);
    this->declare_parameter<double>("maze_graph_front_open_p20", 0.42);
    this->declare_parameter<double>("maze_graph_front_open_p50", 0.50);
    this->declare_parameter<double>("maze_graph_opening_score_threshold", 0.42);
    this->declare_parameter<double>("maze_graph_junction_center_advance", 0.34);
    this->declare_parameter<double>("maze_graph_junction_approach_speed", 0.035);
    this->declare_parameter<double>("maze_graph_junction_front_min", 0.26);
    this->declare_parameter<double>("maze_graph_junction_front_p20", 0.28);
    this->declare_parameter<double>("maze_graph_junction_front_p50", 0.40);
    this->declare_parameter<double>("maze_graph_pre_turn_shift_distance", 0.04);
    this->declare_parameter<double>("maze_graph_pre_turn_shift_speed", 0.040);
    this->declare_parameter<double>("maze_graph_pre_turn_shift_angular", 0.040);
    this->declare_parameter<double>("maze_graph_pre_turn_outer_clearance", 0.32);
    this->declare_parameter<double>("maze_graph_pre_turn_shift_timeout", 2.0);
    this->declare_parameter<double>("maze_graph_post_turn_advance", 0.18);
    this->declare_parameter<double>("maze_graph_post_turn_speed", 0.045);
    this->declare_parameter<double>("maze_graph_post_turn_front_p20", 0.28);
    this->declare_parameter<double>("maze_graph_post_turn_front_p50", 0.40);
    this->declare_parameter<double>("maze_graph_post_turn_timeout", 8.0);
    this->declare_parameter<double>("maze_graph_scan_duration", 0.35);
    this->declare_parameter<double>("maze_graph_turn_timeout", 4.0);
    this->declare_parameter<double>("maze_graph_turn_yaw_tolerance", 0.28);
    this->declare_parameter<double>("maze_complete_confirm_duration", 8.0);
    this->declare_parameter<double>("maze_corridor_center_deadband", 0.03);
    this->declare_parameter<double>("wide_maze_front_clear_p20", 0.65);
    this->declare_parameter<double>("wide_maze_side_wall_p20", 0.75);
    this->declare_parameter<int>("maze_state_confirm_frames", 3);
    this->declare_parameter<std::string>("maze_wall_follow_side", "right");
    this->declare_parameter<bool>("maze_use_robot_swept_volume", true);
    this->declare_parameter<double>("robot_length", 0.280);
    this->declare_parameter<double>("robot_width", 0.258);
    this->declare_parameter<double>("robot_safety_margin", 0.045);
    this->declare_parameter<double>("laser_x_offset", -0.13);
    this->declare_parameter<double>("maze_inner_corner_clearance", 0.18);
    this->declare_parameter<double>("maze_swept_clearance_release", 0.23);
    this->declare_parameter<double>("maze_wall_target_clearance", 0.24);
    this->declare_parameter<double>("maze_corner_preclear_duration", 0.8);
    this->declare_parameter<double>("maze_corner_preclear_reverse_speed", 0.022);
    this->declare_parameter<double>("maze_corner_preclear_outer_angular", 0.06);
    this->declare_parameter<int>("maze_corner_release_confirm_frames", 2);
    this->declare_parameter<double>("maze_swept_turn_min_commit_duration", 1.0);
    this->declare_parameter<double>("maze_swept_clearance_abort", 0.16);
    this->declare_parameter<int>("maze_corner_preclear_max_attempts", 2);
    this->declare_parameter<double>("maze_corner_preclear_total_max_distance", 0.35);
    this->declare_parameter<std::string>("maze_corner_turn_mode", "flow_arc");
    this->declare_parameter<int>("maze_corner_cycle_limit", 3);
    this->declare_parameter<double>("maze_turn_start_front_p20", 0.28);
    this->declare_parameter<double>("maze_turn_start_front_p50", 0.38);
    this->declare_parameter<double>("maze_reverse_front_min", 0.20);
    this->declare_parameter<double>("maze_reverse_front_p20", 0.24);
    this->declare_parameter<double>("maze_corner_approach_speed", 0.045);
    this->declare_parameter<double>("maze_corner_arc_speed", 0.045);
    this->declare_parameter<double>("maze_corner_arc_angular", 0.16);
    this->declare_parameter<double>("maze_corner_pivot_assist_yaw", 0.25);
    this->declare_parameter<double>("maze_corner_micro_reverse_distance", 0.07);
    this->declare_parameter<double>("maze_corner_reverse_distance", 0.14);
    this->declare_parameter<double>("maze_corner_reverse_speed", 0.040);
    this->declare_parameter<double>("maze_corner_pivot_yaw", 0.45);
    this->declare_parameter<double>("maze_corner_pivot_speed", 0.18);
    this->declare_parameter<double>("maze_corner_swing_duration", 1.4);
    this->declare_parameter<double>("maze_corner_swing_forward_speed", 0.040);
    this->declare_parameter<double>("maze_corner_swing_turn_speed", 0.14);
    this->declare_parameter<double>("maze_corner_exit_straight_duration", 0.8);
    this->declare_parameter<double>("maze_corner_retry_backout_distance", 0.25);
    this->declare_parameter<double>("maze_progress_timeout", 8.0);
    this->declare_parameter<double>("progress_timeout", 14.0);
    this->declare_parameter<double>("progress_distance", 0.12);
    this->declare_parameter<double>("recovery_duration", 2.2);
    this->declare_parameter<double>("recovery_turn_speed", 0.18);
    this->declare_parameter<double>("escape_reverse_speed", 0.025);
    this->declare_parameter<double>("min_escape_reverse_speed", 0.02);
    this->declare_parameter<double>("escape_reverse_duration", 1.0);
    this->declare_parameter<double>("escape_turn_duration", 2.0);
    this->declare_parameter<double>("emergency_turn_min_side_p20", 0.26);
    this->declare_parameter<double>("emergency_turn_min_front_diagonal_p20", 0.22);
    this->declare_parameter<double>("emergency_reverse_min_rear_p20", 0.22);
    this->declare_parameter<int>("max_emergency_escape_attempts", 8);
    this->declare_parameter<int>("max_no_frontier_recoveries", 5);
    this->declare_parameter<double>("start_search_radius", 0.60);
    this->declare_parameter<bool>("edge_fallback_enabled", true);
    this->declare_parameter<double>("fallback_obstacle_padding_1", 0.10);
    this->declare_parameter<double>("fallback_obstacle_padding_2", 0.05);
    this->declare_parameter<double>("edge_unknown_radius", 0.25);
    this->declare_parameter<double>("edge_min_goal_distance", 0.30);
    this->declare_parameter<double>("info_gain_weight", 1.0);
    this->declare_parameter<double>("distance_weight", 0.35);
    this->declare_parameter<double>("yaw_weight", 0.15);
    this->declare_parameter<double>("recent_goal_penalty", 12.0);
    this->declare_parameter<int>("max_failed_goals", 12);
    this->declare_parameter<bool>("local_escape_target_enabled", true);
    this->declare_parameter<double>("local_escape_distance", 0.35);
    this->declare_parameter<double>("local_escape_angle", 0.65);
    this->declare_parameter<bool>("enabled", true);

    map_topic_ = this->get_parameter("map_topic").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    status_topic_ = this->get_parameter("status_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_rate_hz_ = std::max(1.0, this->get_parameter("publish_rate_hz").as_double());
    replan_interval_ = std::max(0.5, this->get_parameter("replan_interval").as_double());
    map_timeout_ = std::max(0.5, this->get_parameter("map_timeout").as_double());
    scan_timeout_ = std::max(0.5, this->get_parameter("scan_timeout").as_double());
    odom_timeout_ = std::max(0.5, this->get_parameter("odom_timeout").as_double());
    free_threshold_ = static_cast<int>(this->get_parameter("free_threshold").as_int());
    occupied_threshold_ = static_cast<int>(this->get_parameter("occupied_threshold").as_int());
    obstacle_padding_ = std::max(0.05, this->get_parameter("obstacle_padding").as_double());
    min_frontier_cells_ = std::max(1, static_cast<int>(this->get_parameter("min_frontier_cells").as_int()));
    min_goal_distance_ = std::max(0.1, this->get_parameter("min_goal_distance").as_double());
    max_goal_distance_ = std::max(min_goal_distance_, this->get_parameter("max_goal_distance").as_double());
    unknown_complete_ratio_ = std::max(0.0, this->get_parameter("unknown_complete_ratio").as_double());
    complete_confirm_duration_ = std::max(1.0, this->get_parameter("complete_confirm_duration").as_double());
    goal_tolerance_ = std::max(0.05, this->get_parameter("goal_tolerance").as_double());
    lookahead_distance_ = std::max(0.1, this->get_parameter("lookahead_distance").as_double());
    max_linear_speed_ = std::max(0.0, this->get_parameter("max_linear_speed").as_double());
    min_linear_speed_ = std::max(0.0, this->get_parameter("min_linear_speed").as_double());
    max_angular_speed_ = std::max(0.0, this->get_parameter("max_angular_speed").as_double());
    yaw_kp_ = std::max(0.1, this->get_parameter("yaw_kp").as_double());
    rotate_in_place_yaw_ = std::max(0.1, this->get_parameter("rotate_in_place_yaw").as_double());
    front_stop_distance_ = std::max(0.1, this->get_parameter("front_stop_distance").as_double());
    hard_stop_distance_ = std::max(0.05, this->get_parameter("hard_stop_distance").as_double());
    disable_hard_stop_in_mapping_ = this->get_parameter("disable_hard_stop_in_mapping").as_bool();
    escape_always_try_motion_ = this->get_parameter("escape_always_try_motion").as_bool();
    front_emergency_distance_ = std::max(0.05, this->get_parameter("front_emergency_distance").as_double());
    if (hard_stop_distance_ >= front_emergency_distance_) {
      hard_stop_distance_ = std::max(0.05, front_emergency_distance_ - 0.02);
    }
    front_block_distance_ = std::max(0.1, this->get_parameter("front_block_distance").as_double());
    front_clear_distance_ = std::max(front_block_distance_, this->get_parameter("front_clear_distance").as_double());
    front_block_confirm_frames_ = std::max(1, static_cast<int>(this->get_parameter("front_block_confirm_frames").as_int()));
    front_clear_confirm_frames_ = std::max(1, static_cast<int>(this->get_parameter("front_clear_confirm_frames").as_int()));
    narrow_creep_enabled_ = this->get_parameter("narrow_creep_enabled").as_bool();
    narrow_creep_speed_ = std::max(0.0, this->get_parameter("narrow_creep_speed").as_double());
    narrow_creep_min_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("narrow_creep_min_front_p20").as_double());
    narrow_creep_p50_distance_ = std::max(
      narrow_creep_min_front_p20_, this->get_parameter("narrow_creep_p50_distance").as_double());
    angular_deadband_ = std::max(0.0, this->get_parameter("angular_deadband").as_double());
    angular_flip_suppression_yaw_ = std::max(0.0, this->get_parameter("angular_flip_suppression_yaw").as_double());
    angular_sign_hold_duration_ = std::max(0.0, this->get_parameter("angular_sign_hold_duration").as_double());
    angular_command_max_delta_ = std::max(0.0, this->get_parameter("angular_command_max_delta").as_double());
    turn_lock_duration_ = std::max(0.0, this->get_parameter("turn_lock_duration").as_double());
    turn_switch_margin_ = std::max(0.0, this->get_parameter("turn_switch_margin").as_double());
    turn_score_ema_alpha_ = clamp(this->get_parameter("turn_score_ema_alpha").as_double(), 0.01, 1.0);
    oscillation_window_ = std::max(1.0, this->get_parameter("oscillation_window").as_double());
    oscillation_flip_limit_ = std::max(1, static_cast<int>(this->get_parameter("oscillation_flip_limit").as_int()));
    anti_oscillation_reverse_duration_ = std::max(
      0.1, this->get_parameter("anti_oscillation_reverse_duration").as_double());
    anti_oscillation_turn_duration_ = std::max(
      0.1, this->get_parameter("anti_oscillation_turn_duration").as_double());
    target_min_hold_duration_ = std::max(0.0, this->get_parameter("target_min_hold_duration").as_double());
    target_switch_score_margin_ = std::max(0.0, this->get_parameter("target_switch_score_margin").as_double());
    target_switch_distance_threshold_ = std::max(
      0.05, this->get_parameter("target_switch_distance_threshold").as_double());
    target_switch_yaw_threshold_ = clamp(this->get_parameter("target_switch_yaw_threshold").as_double(), 0.05, M_PI);
    corner_turn_away_enabled_ = this->get_parameter("corner_turn_away_enabled").as_bool();
    corner_front_diagonal_obstacle_p20_ = std::max(
      0.05, this->get_parameter("corner_front_diagonal_obstacle_p20").as_double());
    corner_turn_block_margin_ = std::max(0.0, this->get_parameter("corner_turn_block_margin").as_double());
    corner_turn_max_toward_obstacle_ = std::max(
      0.0, this->get_parameter("corner_turn_max_toward_obstacle").as_double());
    corner_turn_away_speed_ = std::max(0.0, this->get_parameter("corner_turn_away_speed").as_double());
    corner_creep_speed_ = std::max(0.0, this->get_parameter("corner_creep_speed").as_double());
    narrow_maze_follow_enabled_ = this->get_parameter("narrow_maze_follow_enabled").as_bool();
    maze_side_wall_max_p50_ = std::max(0.10, this->get_parameter("maze_side_wall_max_p50").as_double());
    maze_front_min_continue_ = std::max(hard_stop_distance_, this->get_parameter("maze_front_min_continue").as_double());
    maze_front_p20_continue_ = std::max(
      maze_front_min_continue_, this->get_parameter("maze_front_p20_continue").as_double());
    maze_forward_speed_ = std::max(0.0, this->get_parameter("maze_forward_speed").as_double());
    maze_center_kp_ = std::max(0.0, this->get_parameter("maze_center_kp").as_double());
    maze_front_center_kp_ = std::max(0.0, this->get_parameter("maze_front_center_kp").as_double());
    maze_side_error_weight_ = std::max(0.0, this->get_parameter("maze_side_error_weight").as_double());
    maze_front_diagonal_error_weight_ = std::max(
      0.0, this->get_parameter("maze_front_diagonal_error_weight").as_double());
    maze_center_lookahead_min_delta_ = std::max(
      0.0, this->get_parameter("maze_center_lookahead_min_delta").as_double());
    maze_max_angular_ = std::max(0.0, this->get_parameter("maze_max_angular").as_double());
    maze_turn_lock_duration_ = std::max(0.1, this->get_parameter("maze_turn_lock_duration").as_double());
    maze_exit_clear_p20_ = std::max(maze_front_p20_continue_, this->get_parameter("maze_exit_clear_p20").as_double());
    maze_inner_clearance_min_p20_ = std::max(0.05, this->get_parameter("maze_inner_clearance_min_p20").as_double());
    maze_inner_clearance_release_p20_ = std::max(
      maze_inner_clearance_min_p20_, this->get_parameter("maze_inner_clearance_release_p20").as_double());
    maze_front_diagonal_clearance_min_p20_ = std::max(
      0.05, this->get_parameter("maze_front_diagonal_clearance_min_p20").as_double());
    maze_turn_speed_ = std::max(0.0, this->get_parameter("maze_turn_speed").as_double());
    maze_turn_forward_speed_ = std::max(0.0, this->get_parameter("maze_turn_forward_speed").as_double());
    maze_pre_turn_outer_bias_duration_ = std::max(
      0.0, this->get_parameter("maze_pre_turn_outer_bias_duration").as_double());
    maze_pre_turn_outer_bias_angular_ = std::max(
      0.0, this->get_parameter("maze_pre_turn_outer_bias_angular").as_double());
    maze_turn_escape_reverse_duration_ = std::max(
      0.0, this->get_parameter("maze_turn_escape_reverse_duration").as_double());
    maze_turn_escape_reverse_speed_ = std::max(
      0.0, this->get_parameter("maze_turn_escape_reverse_speed").as_double());
    maze_min_hold_duration_ = std::max(0.0, this->get_parameter("maze_min_hold_duration").as_double());
    maze_center_deadband_ = std::max(0.0, this->get_parameter("maze_center_deadband").as_double());
    maze_center_ema_alpha_ = clamp(this->get_parameter("maze_center_ema_alpha").as_double(), 0.01, 1.0);
    maze_angular_max_delta_ = std::max(0.0, this->get_parameter("maze_angular_max_delta").as_double());
    maze_turn_commit_duration_ = std::max(0.1, this->get_parameter("maze_turn_commit_duration").as_double());
    maze_turn_max_continuous_duration_ = std::max(
      0.2, this->get_parameter("maze_turn_max_continuous_duration").as_double());
    maze_post_turn_straight_duration_ = std::max(
      0.0, this->get_parameter("maze_post_turn_straight_duration").as_double());
    maze_min_turn_radius_ = std::max(0.05, this->get_parameter("maze_min_turn_radius").as_double());
    maze_corner_front_delta_ = std::max(0.0, this->get_parameter("maze_corner_front_delta").as_double());
    maze_only_mode_ = this->get_parameter("maze_only_mode").as_bool();
    maze_route_mode_ = this->get_parameter("maze_route_mode").as_string();
    std::transform(maze_route_mode_.begin(), maze_route_mode_.end(), maze_route_mode_.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    if (maze_route_mode_ != "local_sector" && maze_route_mode_ != "graph_dfs") {
      maze_route_mode_ = "local_sector";
    }
    maze_topology_mode_ = this->get_parameter("maze_topology_mode").as_bool();
    maze_topology_forward_speed_ = std::max(0.0, this->get_parameter("maze_topology_forward_speed").as_double());
    maze_topology_fast_speed_ = std::max(
      maze_topology_forward_speed_, this->get_parameter("maze_topology_fast_speed").as_double());
    maze_topology_turn_speed_ = std::max(0.0, this->get_parameter("maze_topology_turn_speed").as_double());
    maze_topology_turn_forward_speed_ = std::max(
      0.0, this->get_parameter("maze_topology_turn_forward_speed").as_double());
    maze_opening_confirm_frames_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_opening_confirm_frames").as_int()));
    maze_junction_dwell_duration_ = std::max(
      0.0, this->get_parameter("maze_junction_dwell_duration").as_double());
    maze_node_merge_distance_ = std::max(0.10, this->get_parameter("maze_node_merge_distance").as_double());
    maze_opening_visited_distance_ = std::max(
      0.10, this->get_parameter("maze_opening_visited_distance").as_double());
    maze_deadend_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_deadend_front_p20").as_double());
    maze_deadend_confirm_frames_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_deadend_confirm_frames").as_int()));
    maze_deadend_side_open_p20_ = std::max(
      maze_front_p20_continue_, this->get_parameter("maze_deadend_side_open_p20").as_double());
    maze_deadend_turnaround_enabled_ = this->get_parameter("maze_deadend_turnaround_enabled").as_bool();
    maze_deadend_turn_speed_ = std::max(0.0, this->get_parameter("maze_deadend_turn_speed").as_double());
    maze_deadend_turn_yaw_ = clamp(this->get_parameter("maze_deadend_turn_yaw").as_double(), 0.5, M_PI);
    maze_deadend_turn_yaw_tolerance_ = clamp(
      this->get_parameter("maze_deadend_turn_yaw_tolerance").as_double(), 0.02, 0.8);
    maze_deadend_turn_timeout_ = std::max(1.0, this->get_parameter("maze_deadend_turn_timeout").as_double());
    maze_deadend_in_place_min_side_p20_ = std::max(
      robot_width_ * 0.5, this->get_parameter("maze_deadend_in_place_min_side_p20").as_double());
    maze_deadend_in_place_min_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_deadend_in_place_min_front_p20").as_double());
    maze_deadend_in_place_min_rear_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_deadend_in_place_min_rear_p20").as_double());
    maze_deadend_reverse_to_turn_spot_speed_ = std::max(
      0.0, this->get_parameter("maze_deadend_reverse_to_turn_spot_speed").as_double());
    maze_deadend_reverse_to_turn_spot_distance_ = std::max(
      0.0, this->get_parameter("maze_deadend_reverse_to_turn_spot_distance").as_double());
    maze_deadend_reverse_to_turn_spot_timeout_ = std::max(
      0.5, this->get_parameter("maze_deadend_reverse_to_turn_spot_timeout").as_double());
    maze_deadend_reverse_chunk_distance_ = std::max(
      0.05, this->get_parameter("maze_deadend_reverse_chunk_distance").as_double());
    maze_deadend_reverse_total_max_distance_ = std::max(
      maze_deadend_reverse_chunk_distance_, this->get_parameter("maze_deadend_reverse_total_max_distance").as_double());
    maze_reverse_rear_stop_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_reverse_rear_stop_p20").as_double());
    maze_reverse_rear_release_p20_ = std::max(
      maze_reverse_rear_stop_p20_, this->get_parameter("maze_reverse_rear_release_p20").as_double());
    maze_deadend_front_drive_clear_p20_ = std::max(
      front_emergency_distance_, this->get_parameter("maze_deadend_front_drive_clear_p20").as_double());
    maze_deadend_front_drive_clear_p50_ = std::max(
      maze_deadend_front_drive_clear_p20_,
      this->get_parameter("maze_deadend_front_drive_clear_p50").as_double());
    deadend_front_drive_clear_confirm_frames_required_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_deadend_front_drive_clear_confirm_frames").as_int()));
    maze_deadend_turn_search_direction_timeout_ = std::max(
      0.5, this->get_parameter("maze_deadend_turn_search_direction_timeout").as_double());
    maze_deadend_turn_search_max_yaw_ = clamp(
      this->get_parameter("maze_deadend_turn_search_max_yaw").as_double(), 0.5, M_PI);
    maze_deadend_turn_search_max_switches_ = std::max(
      0, static_cast<int>(this->get_parameter("maze_deadend_turn_search_max_switches").as_int()));
    maze_open_side_turn_enabled_ = this->get_parameter("maze_open_side_turn_enabled").as_bool();
    maze_open_side_turn_min_score_ = std::max(
      0.0, this->get_parameter("maze_open_side_turn_min_score").as_double());
    maze_open_side_turn_score_margin_ = std::max(
      0.0, this->get_parameter("maze_open_side_turn_score_margin").as_double());
    maze_open_side_front_clear_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_open_side_front_clear_p20").as_double());
    maze_open_side_front_clear_p50_ = std::max(
      maze_open_side_front_clear_p20_, this->get_parameter("maze_open_side_front_clear_p50").as_double());
    maze_open_side_confirm_frames_required_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_open_side_confirm_frames").as_int()));
    maze_open_side_reselect_timeout_ = std::max(
      0.2, this->get_parameter("maze_open_side_reselect_timeout").as_double());
    maze_no_reverse_mode_ = this->get_parameter("maze_no_reverse_mode").as_bool();
    maze_rotate_search_speed_ = std::max(
      0.0, this->get_parameter("maze_rotate_search_speed").as_double());
    maze_rotate_search_direction_timeout_ = std::max(
      0.2, this->get_parameter("maze_rotate_search_direction_timeout").as_double());
    maze_rotate_search_total_timeout_ = std::max(
      maze_rotate_search_direction_timeout_,
      this->get_parameter("maze_rotate_search_total_timeout").as_double());
    maze_rotate_search_max_switches_ = std::max(
      0, static_cast<int>(this->get_parameter("maze_rotate_search_max_switches").as_int()));
    maze_rotate_front_clear_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_rotate_front_clear_p20").as_double());
    maze_rotate_front_clear_p50_ = std::max(
      maze_rotate_front_clear_p20_, this->get_parameter("maze_rotate_front_clear_p50").as_double());
    maze_rotate_front_clear_confirm_frames_required_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_rotate_front_clear_confirm_frames").as_int()));
    maze_graph_front_obstacle_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_graph_front_obstacle_p20").as_double());
    maze_graph_front_open_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_graph_front_open_p20").as_double());
    maze_graph_front_open_p50_ = std::max(
      maze_graph_front_open_p20_, this->get_parameter("maze_graph_front_open_p50").as_double());
    maze_graph_opening_score_threshold_ = std::max(
      0.0, this->get_parameter("maze_graph_opening_score_threshold").as_double());
    maze_graph_junction_center_advance_ = std::max(
      0.0, this->get_parameter("maze_graph_junction_center_advance").as_double());
    maze_graph_junction_approach_speed_ = std::max(
      0.0, this->get_parameter("maze_graph_junction_approach_speed").as_double());
    maze_graph_junction_front_min_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_graph_junction_front_min").as_double());
    maze_graph_junction_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_graph_junction_front_p20").as_double());
    maze_graph_junction_front_p50_ = std::max(
      maze_graph_junction_front_p20_,
      this->get_parameter("maze_graph_junction_front_p50").as_double());
    maze_graph_pre_turn_shift_distance_ = std::max(
      0.0, this->get_parameter("maze_graph_pre_turn_shift_distance").as_double());
    maze_graph_pre_turn_shift_speed_ = std::max(
      0.0, this->get_parameter("maze_graph_pre_turn_shift_speed").as_double());
    maze_graph_pre_turn_shift_angular_ = std::max(
      0.0, this->get_parameter("maze_graph_pre_turn_shift_angular").as_double());
    maze_graph_pre_turn_outer_clearance_ = std::max(
      hard_stop_distance_,
      this->get_parameter("maze_graph_pre_turn_outer_clearance").as_double());
    maze_graph_pre_turn_shift_timeout_ = std::max(
      0.2, this->get_parameter("maze_graph_pre_turn_shift_timeout").as_double());
    maze_graph_post_turn_advance_ = std::max(
      0.0, this->get_parameter("maze_graph_post_turn_advance").as_double());
    maze_graph_post_turn_speed_ = std::max(
      0.0, this->get_parameter("maze_graph_post_turn_speed").as_double());
    maze_graph_post_turn_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_graph_post_turn_front_p20").as_double());
    maze_graph_post_turn_front_p50_ = std::max(
      maze_graph_post_turn_front_p20_,
      this->get_parameter("maze_graph_post_turn_front_p50").as_double());
    maze_graph_post_turn_timeout_ = std::max(
      0.5, this->get_parameter("maze_graph_post_turn_timeout").as_double());
    maze_graph_scan_duration_ = std::max(
      0.0, this->get_parameter("maze_graph_scan_duration").as_double());
    maze_graph_turn_timeout_ = std::max(
      0.5, this->get_parameter("maze_graph_turn_timeout").as_double());
    maze_graph_turn_yaw_tolerance_ = std::max(
      0.02, this->get_parameter("maze_graph_turn_yaw_tolerance").as_double());
    maze_complete_confirm_duration_ = std::max(
      1.0, this->get_parameter("maze_complete_confirm_duration").as_double());
    maze_corridor_center_deadband_ = std::max(
      0.0, this->get_parameter("maze_corridor_center_deadband").as_double());
    if (maze_topology_mode_) {
      maze_forward_speed_ = maze_topology_forward_speed_;
      maze_turn_speed_ = maze_topology_turn_speed_;
      maze_turn_forward_speed_ = maze_topology_turn_forward_speed_;
      maze_center_deadband_ = std::max(maze_center_deadband_, maze_corridor_center_deadband_);
    }
    wide_maze_front_clear_p20_ = std::max(
      maze_front_p20_continue_, this->get_parameter("wide_maze_front_clear_p20").as_double());
    wide_maze_side_wall_p20_ = std::max(
      maze_side_wall_max_p50_, this->get_parameter("wide_maze_side_wall_p20").as_double());
    maze_state_confirm_frames_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_state_confirm_frames").as_int()));
    maze_wall_follow_side_ = normalize_wall_follow_side(this->get_parameter("maze_wall_follow_side").as_string());
    maze_use_robot_swept_volume_ = this->get_parameter("maze_use_robot_swept_volume").as_bool();
    robot_length_ = std::max(0.05, this->get_parameter("robot_length").as_double());
    robot_width_ = std::max(0.05, this->get_parameter("robot_width").as_double());
    robot_safety_margin_ = std::max(0.0, this->get_parameter("robot_safety_margin").as_double());
    laser_x_offset_ = this->get_parameter("laser_x_offset").as_double();
    maze_inner_corner_clearance_ = std::max(0.05, this->get_parameter("maze_inner_corner_clearance").as_double());
    maze_swept_clearance_release_ = std::max(
      maze_inner_corner_clearance_, this->get_parameter("maze_swept_clearance_release").as_double());
    maze_wall_target_clearance_ = std::max(
      robot_width_ * 0.5 + robot_safety_margin_ + 0.02,
      this->get_parameter("maze_wall_target_clearance").as_double());
    maze_corner_preclear_duration_ = std::max(
      0.0, this->get_parameter("maze_corner_preclear_duration").as_double());
    maze_corner_preclear_reverse_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_preclear_reverse_speed").as_double());
    maze_corner_preclear_outer_angular_ = std::max(
      0.0, this->get_parameter("maze_corner_preclear_outer_angular").as_double());
    maze_corner_release_confirm_frames_required_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_corner_release_confirm_frames").as_int()));
    maze_swept_turn_min_commit_duration_ = std::max(
      0.0, this->get_parameter("maze_swept_turn_min_commit_duration").as_double());
    maze_swept_clearance_abort_ = std::max(
      0.0, this->get_parameter("maze_swept_clearance_abort").as_double());
    maze_corner_preclear_max_attempts_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_corner_preclear_max_attempts").as_int()));
    maze_corner_preclear_total_max_distance_ = std::max(
      0.05, this->get_parameter("maze_corner_preclear_total_max_distance").as_double());
    maze_corner_turn_mode_ = this->get_parameter("maze_corner_turn_mode").as_string();
    std::transform(
      maze_corner_turn_mode_.begin(), maze_corner_turn_mode_.end(), maze_corner_turn_mode_.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (maze_corner_turn_mode_ != "flow_arc" && maze_corner_turn_mode_ != "three_point") {
      maze_corner_turn_mode_ = "legacy";
    }
    maze_corner_cycle_limit_ = std::max(
      1, static_cast<int>(this->get_parameter("maze_corner_cycle_limit").as_int()));
    maze_turn_start_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_turn_start_front_p20").as_double());
    maze_turn_start_front_p50_ = std::max(
      maze_turn_start_front_p20_, this->get_parameter("maze_turn_start_front_p50").as_double());
    maze_reverse_front_min_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_reverse_front_min").as_double());
    maze_reverse_front_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("maze_reverse_front_p20").as_double());
    maze_corner_approach_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_approach_speed").as_double());
    maze_corner_arc_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_arc_speed").as_double());
    maze_corner_arc_angular_ = std::max(
      0.0, this->get_parameter("maze_corner_arc_angular").as_double());
    maze_corner_pivot_assist_yaw_ = clamp(
      this->get_parameter("maze_corner_pivot_assist_yaw").as_double(), 0.05, 0.8);
    maze_corner_micro_reverse_distance_ = std::max(
      0.02, this->get_parameter("maze_corner_micro_reverse_distance").as_double());
    maze_corner_reverse_target_distance_ = std::max(
      0.03, this->get_parameter("maze_corner_reverse_distance").as_double());
    maze_corner_reverse_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_reverse_speed").as_double());
    maze_corner_pivot_yaw_ = clamp(this->get_parameter("maze_corner_pivot_yaw").as_double(), 0.10, 1.40);
    maze_corner_pivot_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_pivot_speed").as_double());
    maze_corner_swing_duration_ = std::max(
      0.2, this->get_parameter("maze_corner_swing_duration").as_double());
    maze_corner_swing_forward_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_swing_forward_speed").as_double());
    maze_corner_swing_turn_speed_ = std::max(
      0.0, this->get_parameter("maze_corner_swing_turn_speed").as_double());
    maze_corner_exit_straight_duration_ = std::max(
      0.0, this->get_parameter("maze_corner_exit_straight_duration").as_double());
    maze_corner_retry_backout_distance_ = std::max(
      maze_corner_reverse_target_distance_, this->get_parameter("maze_corner_retry_backout_distance").as_double());
    maze_progress_timeout_ = std::max(2.0, this->get_parameter("maze_progress_timeout").as_double());
    progress_timeout_ = std::max(2.0, this->get_parameter("progress_timeout").as_double());
    progress_distance_ = std::max(0.02, this->get_parameter("progress_distance").as_double());
    recovery_duration_ = std::max(0.5, this->get_parameter("recovery_duration").as_double());
    recovery_turn_speed_ = std::abs(this->get_parameter("recovery_turn_speed").as_double());
    escape_reverse_speed_ = std::max(0.0, std::abs(this->get_parameter("escape_reverse_speed").as_double()));
    min_escape_reverse_speed_ = std::max(0.0, std::abs(this->get_parameter("min_escape_reverse_speed").as_double()));
    escape_reverse_speed_ = std::max(escape_reverse_speed_, min_escape_reverse_speed_);
    escape_reverse_duration_ = std::max(0.2, this->get_parameter("escape_reverse_duration").as_double());
    escape_turn_duration_ = std::max(0.2, this->get_parameter("escape_turn_duration").as_double());
    emergency_turn_min_side_p20_ = std::max(0.05, this->get_parameter("emergency_turn_min_side_p20").as_double());
    emergency_turn_min_front_diagonal_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("emergency_turn_min_front_diagonal_p20").as_double());
    emergency_reverse_min_rear_p20_ = std::max(
      hard_stop_distance_, this->get_parameter("emergency_reverse_min_rear_p20").as_double());
    max_emergency_escape_attempts_ = std::max(1, static_cast<int>(this->get_parameter("max_emergency_escape_attempts").as_int()));
    max_no_frontier_recoveries_ = std::max(1, static_cast<int>(this->get_parameter("max_no_frontier_recoveries").as_int()));
    start_search_radius_ = std::max(0.10, this->get_parameter("start_search_radius").as_double());
    edge_fallback_enabled_ = this->get_parameter("edge_fallback_enabled").as_bool();
    fallback_obstacle_padding_1_ = std::max(0.0, this->get_parameter("fallback_obstacle_padding_1").as_double());
    fallback_obstacle_padding_2_ = std::max(0.0, this->get_parameter("fallback_obstacle_padding_2").as_double());
    edge_unknown_radius_ = std::max(0.05, this->get_parameter("edge_unknown_radius").as_double());
    edge_min_goal_distance_ = std::max(0.05, this->get_parameter("edge_min_goal_distance").as_double());
    info_gain_weight_ = this->get_parameter("info_gain_weight").as_double();
    distance_weight_ = this->get_parameter("distance_weight").as_double();
    yaw_weight_ = this->get_parameter("yaw_weight").as_double();
    recent_goal_penalty_ = this->get_parameter("recent_goal_penalty").as_double();
    max_failed_goals_ = std::max(1, static_cast<int>(this->get_parameter("max_failed_goals").as_int()));
    local_escape_target_enabled_ = this->get_parameter("local_escape_target_enabled").as_bool();
    local_escape_distance_ = std::max(0.10, this->get_parameter("local_escape_distance").as_double());
    local_escape_angle_ = clamp(this->get_parameter("local_escape_angle").as_double(), 0.10, 1.40);
    enabled_ = this->get_parameter("enabled").as_bool();

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10).best_effort());
    status_pub_ = this->create_publisher<std_msgs::msg::String>(status_topic_, rclcpp::QoS(10).reliable());
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&FrontierExplorer::map_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FrontierExplorer::scan_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&FrontierExplorer::odom_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&FrontierExplorer::control_loop, this));

    last_map_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_odom_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_plan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    target_selected_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    complete_candidate_since_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    last_progress_time_ = this->now();
    last_angular_sign_change_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Frontier explorer started: map=%s scan=%s odom=%s cmd=%s status=%s",
      map_topic_.c_str(), scan_topic_.c_str(), odom_topic_.c_str(),
      cmd_vel_topic_.c_str(), status_topic_.c_str());
  }

  ~FrontierExplorer()
  {
    publish_stop();
  }

private:
  struct Pose2D
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  struct Point2D
  {
    double x = 0.0;
    double y = 0.0;
  };

  struct FrontierCluster
  {
    std::vector<int> cells;
    int target_index = -1;
    double centroid_x = 0.0;
    double centroid_y = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    double distance_m = 0.0;
  };

  struct GridPlan
  {
    int width = 0;
    int height = 0;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    std::vector<int8_t> data;
    std::vector<uint8_t> blocked;
    double unknown_ratio = 1.0;
  };

  struct QueueNode
  {
    int index = -1;
    double f = 0.0;
    double g = 0.0;
  };

  struct QueueCompare
  {
    bool operator()(const QueueNode& a, const QueueNode& b) const
    {
      return a.f > b.f;
    }
  };

  struct MazeRouteObservation
  {
    bool left_wall = false;
    bool right_wall = false;
    bool any_wall = false;
    bool both_walls = false;
    bool front_blocked = false;
    bool front_wide_clear = false;
    bool deadend_candidate = false;
    bool junction = false;
    bool corner = false;
    double selected_turn_direction = 0.0;
  };

  struct MazeRouteDecision
  {
    std::string state = "drive_open";
    std::string reason = "default";
    double selected_turn_direction = 0.0;
  };

  struct MazeTopologyNode
  {
    int id = -1;
    Pose2D pose;
    std::array<bool, 3> open{{false, false, false}};
    std::array<bool, 3> visited{{false, false, false}};
    bool deadend = false;
  };

  struct GraphOpenings
  {
    std::array<bool, 3> open{{false, false, false}};
    std::array<double, 3> score{{0.0, 0.0, 0.0}};
    int open_count = 0;
  };

  static double clamp(double value, double low, double high)
  {
    return std::min(std::max(value, low), high);
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

  static std::string json_escape(const std::string& input)
  {
    std::ostringstream out;
    for (char c : input) {
      switch (c) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default: out << c; break;
      }
    }
    return out.str();
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    int unknown_count = 0;
    for (const auto value : msg->data) {
      if (value < 0) {
        ++unknown_count;
      }
    }
    const double map_unknown_ratio = msg->data.empty() ?
      unknown_ratio_ :
      static_cast<double>(unknown_count) / static_cast<double>(msg->data.size());
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = msg;
    last_map_time_ = this->now();
    unknown_ratio_ = map_unknown_ratio;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto summary = chassis_controller::summarize_scan_sectors(*msg, 3.5);
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      latest_scan_summary_ = summary;
      front_p20_ = summary.front.p20;
      front_p50_ = summary.front.p50;
      front_min_ = summary.front.min;
      front_valid_count_ = summary.front.valid_count;
      front_left_p20_ = summary.front_left.p20;
      front_right_p20_ = summary.front_right.p20;
      left_p20_ = summary.left.p20;
      right_p20_ = summary.right.p20;
      left_side_p20_ = summary.left_side.p20;
      right_side_p20_ = summary.right_side.p20;
      rear_p20_ = summary.rear.p20;
      last_scan_time_ = this->now();
      has_scan_ = true;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_time_ = this->now();
    has_odom_ = true;
  }

  bool front_blocked(double front_min, double front_p20) const
  {
    return front_min < front_emergency_distance_ || front_p20 < narrow_creep_min_front_p20_;
  }

  bool front_clear(double front_min, double front_p20) const
  {
    return front_min >= front_emergency_distance_ && front_p20 >= front_clear_distance_;
  }

  bool front_emergency(double front_min) const
  {
    return front_min < front_emergency_distance_;
  }

  bool front_hard_stop(double front_min) const
  {
    if (disable_hard_stop_in_mapping_) {
      return false;
    }
    return front_min < hard_stop_distance_;
  }

  bool front_contact_escape(double front_min) const
  {
    return front_min < hard_stop_distance_;
  }

  static std::string normalize_wall_follow_side(std::string side)
  {
    std::transform(side.begin(), side.end(), side.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return side == "left" ? "left" : "right";
  }

  double wall_follow_direction() const
  {
    return maze_wall_follow_side_ == "left" ? 1.0 : -1.0;
  }

  std::string direction_name(double direction) const
  {
    return direction >= 0.0 ? "left" : "right";
  }

  static double finite_or(double value, double fallback)
  {
    return std::isfinite(value) ? value : fallback;
  }

  static double turn_open_score(bool turning_left, const chassis_controller::ScanSectorSummary& summary)
  {
    const auto& diagonal = turning_left ? summary.front_left : summary.front_right;
    const auto& side = turning_left ? summary.left_side : summary.right_side;
    const auto& legacy_side = turning_left ? summary.left : summary.right;
    const double diagonal_p20 = finite_or(diagonal.p20, summary.max_usable_range);
    const double side_p20 = std::max(
      finite_or(side.p20, 0.0),
      finite_or(legacy_side.p20, 0.0));
    return 0.70 * diagonal_p20 + 0.30 * side_p20;
  }

  double side_p20_for_direction(double direction)
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return direction >= 0.0 ? left_p20_ : right_p20_;
  }

  double turn_open_score_for_direction(double direction)
  {
    chassis_controller::ScanSectorSummary summary;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      summary = latest_scan_summary_;
    }
    return turn_open_score(direction >= 0.0, summary);
  }

  void update_turn_score_ema(double left_score, double right_score)
  {
    if (!turn_score_ema_initialized_) {
      left_turn_score_ema_ = left_score;
      right_turn_score_ema_ = right_score;
      turn_score_ema_initialized_ = true;
      return;
    }
    left_turn_score_ema_ =
      turn_score_ema_alpha_ * left_score + (1.0 - turn_score_ema_alpha_) * left_turn_score_ema_;
    right_turn_score_ema_ =
      turn_score_ema_alpha_ * right_score + (1.0 - turn_score_ema_alpha_) * right_turn_score_ema_;
  }

  double choose_stable_turn_direction(
    double current_direction,
    double left_score,
    double right_score,
    const rclcpp::Time& now)
  {
    update_turn_score_ema(left_score, right_score);
    const double candidate = left_turn_score_ema_ >= right_turn_score_ema_ ? 1.0 : -1.0;
    const double current = current_direction >= 0.0 ? 1.0 : -1.0;
    open_side_score_ = std::max(left_turn_score_ema_, right_turn_score_ema_);
    turn_switch_blocked_ = false;

    if (!turn_lock_active_) {
      turn_lock_active_ = true;
      turn_lock_direction_ = candidate;
      turn_lock_start_time_ = now;
      return turn_lock_direction_;
    }

    const double locked_score = turn_lock_direction_ >= 0.0 ? left_turn_score_ema_ : right_turn_score_ema_;
    const double opposite_score = turn_lock_direction_ >= 0.0 ? right_turn_score_ema_ : left_turn_score_ema_;
    const bool lock_expired = (now - turn_lock_start_time_).seconds() >= turn_lock_duration_;
    const bool strong_opposite = opposite_score >= locked_score + turn_switch_margin_;
    if (candidate != turn_lock_direction_ && (!lock_expired || !strong_opposite)) {
      turn_switch_blocked_ = true;
      return turn_lock_direction_;
    }

    if (candidate != turn_lock_direction_ && lock_expired && strong_opposite) {
      turn_lock_direction_ = candidate;
      turn_lock_start_time_ = now;
      return turn_lock_direction_;
    }

    if (current != turn_lock_direction_ && current_direction != 0.0 && lock_expired && strong_opposite) {
      turn_lock_direction_ = current;
      turn_lock_start_time_ = now;
    }
    return turn_lock_direction_;
  }

  void release_turn_lock()
  {
    turn_lock_active_ = false;
    turn_switch_blocked_ = false;
  }

  int count_recent_angular_flips(const rclcpp::Time& now) const
  {
    int count = 0;
    for (const auto& stamp : angular_flip_times_) {
      if ((now - stamp).seconds() <= oscillation_window_) {
        ++count;
      }
    }
    return count;
  }

  void record_angular_direction(double angular_z, const rclcpp::Time& now)
  {
    const int sign = angular_z > angular_deadband_ ? 1 : (angular_z < -angular_deadband_ ? -1 : 0);
    if (sign != 0 && oscillation_last_sign_ != 0 && sign != oscillation_last_sign_) {
      angular_flip_times_.push_back(now);
    }
    if (sign != 0) {
      oscillation_last_sign_ = sign;
    }
    while (!angular_flip_times_.empty() &&
           (now - angular_flip_times_.front()).seconds() > oscillation_window_)
    {
      angular_flip_times_.erase(angular_flip_times_.begin());
    }
    oscillation_flip_count_ = count_recent_angular_flips(now);
    oscillation_detected_ = oscillation_flip_count_ >= oscillation_flip_limit_;
  }

  void start_anti_oscillation_escape(const rclcpp::Time& now)
  {
    if (anti_oscillation_active_) {
      return;
    }
    anti_oscillation_active_ = true;
    anti_oscillation_start_time_ = now;
    chassis_controller::ScanSectorSummary summary;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      summary = latest_scan_summary_;
    }
    const double left_score = turn_open_score(true, summary);
    const double right_score = turn_open_score(false, summary);
    anti_oscillation_direction_ = left_score >= right_score ? 1.0 : -1.0;
    open_side_score_ = std::max(left_score, right_score);
    recovery_direction_ = anti_oscillation_direction_;
    turn_lock_active_ = true;
    turn_lock_direction_ = anti_oscillation_direction_;
    turn_lock_start_time_ = now;
    clear_plan();
  }

  bool publish_anti_oscillation_escape(const rclcpp::Time& now)
  {
    if (!anti_oscillation_active_) {
      return false;
    }
    const double elapsed = (now - anti_oscillation_start_time_).seconds();
    const double total_duration = anti_oscillation_reverse_duration_ + anti_oscillation_turn_duration_;
    if (elapsed >= anti_oscillation_reverse_duration_ &&
        front_clear_frames_ >= std::max(1, front_clear_confirm_frames_ / 2))
    {
      anti_oscillation_active_ = false;
      oscillation_detected_ = false;
      oscillation_flip_count_ = 0;
      angular_flip_times_.clear();
      release_turn_lock();
      clear_plan();
      return false;
    }
    if (elapsed >= total_duration) {
      anti_oscillation_active_ = false;
      oscillation_detected_ = false;
      oscillation_flip_count_ = 0;
      angular_flip_times_.clear();
      clear_plan();
      return false;
    }

    geometry_msgs::msg::Twist cmd;
    if (elapsed < anti_oscillation_reverse_duration_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      escape_phase_ = "anti_oscillation_reverse";
      escape_action_ = "anti_oscillation_reverse";
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_turn_speed_ * (anti_oscillation_direction_ >= 0.0 ? 1.0 : -1.0);
      escape_phase_ = "anti_oscillation_turn";
      escape_action_ = "anti_oscillation_turn";
    }
    double front_min = std::numeric_limits<double>::infinity();
    double front_p20 = std::numeric_limits<double>::infinity();
    double front_p50 = std::numeric_limits<double>::infinity();
    double left_side = std::numeric_limits<double>::infinity();
    double right_side = std::numeric_limits<double>::infinity();
    double left_mid = std::numeric_limits<double>::infinity();
    double right_mid = std::numeric_limits<double>::infinity();
    double front_left = std::numeric_limits<double>::infinity();
    double front_right = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_min = front_min_;
      front_p20 = front_p20_;
      front_p50 = front_p50_;
      left_side = left_side_p20_;
      right_side = right_side_p20_;
      left_mid = left_p20_;
      right_mid = right_p20_;
      front_left = front_left_p20_;
      front_right = front_right_p20_;
    }
    const bool no_reverse_override = apply_maze_no_reverse_mode(
      cmd, now, escape_action_, front_min, front_p20, front_p50,
      left_side, right_side, left_mid, right_mid, front_left, front_right);
    last_cmd_reason_ = "anti_oscillation_escape";
    raw_angular_z_ = cmd.angular.z;
    limited_angular_z_ = cmd.angular.z;
    last_angular_command_ = cmd.angular.z;
    record_angular_direction(cmd.angular.z, now);
    cmd_vel_pub_->publish(cmd);
    set_status(
      no_reverse_override ? "maze_rotate_search" : "anti_oscillation_escape",
      no_reverse_override ? maze_reverse_disabled_reason_ :
        "angular direction oscillation detected, backing out and turning one way");
    return true;
  }

  bool turn_clear_for_direction(double direction, double& score, double& diagonal_p20)
  {
    chassis_controller::ScanSectorSummary summary;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      summary = latest_scan_summary_;
    }
    const bool turning_left = direction >= 0.0;
    const auto& diagonal = turning_left ? summary.front_left : summary.front_right;
    diagonal_p20 = finite_or(diagonal.p20, summary.max_usable_range);
    score = turn_open_score(turning_left, summary);
    return diagonal_p20 >= emergency_turn_min_front_diagonal_p20_ &&
      score >= emergency_turn_min_side_p20_;
  }

  bool rear_clear_for_reverse()
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    return latest_scan_summary_.rear.p20 >= emergency_reverse_min_rear_p20_;
  }

  double choose_emergency_escape_direction()
  {
    chassis_controller::ScanSectorSummary summary;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      summary = latest_scan_summary_;
    }
    const double left_score = turn_open_score(true, summary);
    const double right_score = turn_open_score(false, summary);
    return choose_stable_turn_direction(escape_direction_, left_score, right_score, this->now());
  }

  std::string obstacle_state(double front_min, double front_p20, double front_p50) const
  {
    if (front_hard_stop(front_min)) {
      return "hard_stop";
    }
    if (front_emergency(front_min)) {
      return "emergency";
    }
    if (front_blocked_frames_ >= front_block_confirm_frames_) {
      return "blocked";
    }
    if (front_p20 >= front_clear_distance_) {
      return "clear";
    }
    if (narrow_creep_enabled_ && front_p20 >= narrow_creep_min_front_p20_ && front_p50 >= narrow_creep_p50_distance_) {
      return "narrow_corridor";
    }
    return "watching";
  }

  bool can_creep_in_narrow_corridor(double front_min, double front_p20, double front_p50, double yaw_error) const
  {
    if (!narrow_creep_enabled_ || front_emergency(front_min)) {
      return false;
    }
    if (front_p20 < narrow_creep_min_front_p20_ || front_p20 >= front_clear_distance_) {
      return false;
    }
    if (front_p50 < narrow_creep_p50_distance_) {
      return false;
    }
    return std::abs(yaw_error) <= rotate_in_place_yaw_;
  }

  double current_target_age(const rclcpp::Time& now) const
  {
    if (target_selected_time_.nanoseconds() == 0 || target_type_ == "none") {
      return 0.0;
    }
    return std::max(0.0, (now - target_selected_time_).seconds());
  }

  bool has_active_global_target() const
  {
    return current_goal_cell_ >= 0 && !current_path_.empty() &&
      target_type_ != "local_open_sector" && target_type_ != "none";
  }

  bool should_hold_current_target(
    const Pose2D& pose,
    const rclcpp::Time& now,
    double front_min,
    double front_p20)
  {
    target_switch_blocked_ = false;
    if (!has_active_global_target()) {
      target_switch_reason_ = "no_active_target";
      return false;
    }
    if (front_blocked_frames_ >= front_block_confirm_frames_ ||
        front_emergency(front_min) ||
        front_p20 < narrow_creep_min_front_p20_) {
      target_switch_reason_ = "front_blocked";
      return false;
    }
    const double age = current_target_age(now);
    if (age < target_min_hold_duration_) {
      target_switch_blocked_ = true;
      target_switch_reason_ = "target_hold_time";
      return true;
    }
    const double goal_distance = distance_to_target(pose);
    const double pose_delta = std::hypot(pose.x - last_progress_pose_.x, pose.y - last_progress_pose_.y);
    const bool making_progress =
      last_goal_distance_ - goal_distance > progress_distance_ * 0.5 ||
      pose_delta > progress_distance_ * 0.5 ||
      (now - last_progress_time_).seconds() < std::max(2.0, progress_timeout_ * 0.5);
    if (making_progress && front_clear(front_min, front_p20)) {
      target_switch_blocked_ = true;
      target_switch_reason_ = "current_target_progress";
      return true;
    }
    target_switch_reason_ = "periodic_replan_allowed";
    return false;
  }

  bool should_accept_new_target(
    const Pose2D& pose,
    const FrontierCluster& candidate,
    bool selected_edge_target,
    const GridPlan& grid,
    std::string& reason) const
  {
    if (!has_active_global_target() || current_goal_cell_ < 0) {
      reason = selected_edge_target ? "selected_reachable_edge" : "selected_frontier";
      return true;
    }
    if (target_selected_time_.nanoseconds() == 0) {
      reason = selected_edge_target ? "selected_reachable_edge" : "selected_frontier";
      return true;
    }

    double candidate_x = 0.0;
    double candidate_y = 0.0;
    map_to_world(grid, candidate.target_index, candidate_x, candidate_y);
    const double target_delta = std::hypot(candidate_x - target_.x, candidate_y - target_.y);
    const double old_yaw = std::atan2(target_.y - pose.y, target_.x - pose.x);
    const double new_yaw = std::atan2(candidate_y - pose.y, candidate_x - pose.x);
    const double yaw_delta = std::abs(normalize_angle(new_yaw - old_yaw));
    const double score_gain = candidate.score - current_target_score_;
    const bool same_goal = candidate.target_index == current_goal_cell_ ||
      target_delta < std::max(0.12, goal_tolerance_ * 0.5);

    if (same_goal) {
      reason = "same_target_refresh";
      return true;
    }
    if (target_type_ == "reachable_edge" && !selected_edge_target) {
      reason = "frontier_replaces_edge";
      return true;
    }
    if (score_gain >= target_switch_score_margin_) {
      reason = "better_score";
      return true;
    }
    if (target_delta <= target_switch_distance_threshold_ && yaw_delta <= target_switch_yaw_threshold_) {
      reason = "nearby_target_refresh";
      return true;
    }

    reason = "target_switch_blocked";
    return false;
  }

  double stabilize_angular_command(double desired, double yaw_error, const rclcpp::Time& now)
  {
    if (std::abs(desired) < angular_deadband_) {
      desired = 0.0;
    }

    const int desired_sign = desired > 0.0 ? 1 : (desired < 0.0 ? -1 : 0);
    if (desired_sign != 0 && last_angular_sign_ != 0 && desired_sign != last_angular_sign_) {
      const double since_change = (now - last_angular_sign_change_time_).seconds();
      if (std::abs(yaw_error) < angular_flip_suppression_yaw_ &&
          since_change < angular_sign_hold_duration_) {
        desired = 0.0;
      } else {
        last_angular_sign_ = desired_sign;
        last_angular_sign_change_time_ = now;
      }
    } else if (desired_sign != 0 && desired_sign != last_angular_sign_) {
      last_angular_sign_ = desired_sign;
      last_angular_sign_change_time_ = now;
    }

    if (angular_command_max_delta_ > 0.0) {
      desired = clamp(
        desired,
        last_angular_command_ - angular_command_max_delta_,
        last_angular_command_ + angular_command_max_delta_);
    }
    last_angular_command_ = desired;
    record_angular_direction(desired, now);
    if (oscillation_detected_) {
      start_anti_oscillation_escape(now);
    }
    return desired;
  }

  bool apply_corner_turn_guard(geometry_msgs::msg::Twist& cmd)
  {
    if (!corner_turn_away_enabled_ || std::abs(cmd.angular.z) < angular_deadband_) {
      corner_guard_active_ = false;
      corner_guard_reason_ = "none";
      return false;
    }
    double front_left = std::numeric_limits<double>::infinity();
    double front_right = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_left = front_left_p20_;
      front_right = front_right_p20_;
    }
    const bool left_obstacle =
      front_left < corner_front_diagonal_obstacle_p20_ &&
      front_left + corner_turn_block_margin_ < front_right;
    const bool right_obstacle =
      front_right < corner_front_diagonal_obstacle_p20_ &&
      front_right + corner_turn_block_margin_ < front_left;

    if (left_obstacle && cmd.angular.z > corner_turn_max_toward_obstacle_) {
      raw_angular_z_ = cmd.angular.z;
      cmd.angular.z = -std::min(corner_turn_away_speed_, recovery_turn_speed_);
      if (cmd.linear.x > 0.0) {
        cmd.linear.x = std::min(cmd.linear.x, corner_creep_speed_);
      }
      corner_guard_active_ = true;
      corner_guard_reason_ = "left_front_obstacle_limit_left_turn";
      return true;
    }
    if (right_obstacle && cmd.angular.z < -corner_turn_max_toward_obstacle_) {
      raw_angular_z_ = cmd.angular.z;
      cmd.angular.z = std::min(corner_turn_away_speed_, recovery_turn_speed_);
      if (cmd.linear.x > 0.0) {
        cmd.linear.x = std::min(cmd.linear.x, corner_creep_speed_);
      }
      corner_guard_active_ = true;
      corner_guard_reason_ = "right_front_obstacle_limit_right_turn";
      return true;
    }

    corner_guard_active_ = false;
    corner_guard_reason_ = "none";
    return false;
  }

  bool maze_left_wall_present(double left_side_p20) const
  {
    const double left_near = std::min(
      finite_or(left_side_p20, std::numeric_limits<double>::infinity()),
      finite_or(left_p20_, std::numeric_limits<double>::infinity()));
    return left_near <= maze_side_wall_max_p50_;
  }

  bool maze_right_wall_present(double right_side_p20) const
  {
    const double right_near = std::min(
      finite_or(right_side_p20, std::numeric_limits<double>::infinity()),
      finite_or(right_p20_, std::numeric_limits<double>::infinity()));
    return right_near <= maze_side_wall_max_p50_;
  }

  bool maze_side_wall_present(double left_side_p20, double right_side_p20) const
  {
    return maze_left_wall_present(left_side_p20) && maze_right_wall_present(right_side_p20);
  }

  bool maze_front_continue_clear(double front_min, double front_p20) const
  {
    return front_min >= maze_front_min_continue_ && front_p20 >= maze_front_p20_continue_;
  }

  bool maze_environment_present(double front_min, double front_p20, double front_p50) const
  {
    if (maze_only_mode_) {
      return true;
    }
    const bool left_wall = maze_left_wall_present(left_side_p20_);
    const bool right_wall = maze_right_wall_present(right_side_p20_);
    const bool diagonal_limited =
      front_left_p20_ < maze_exit_clear_p20_ ||
      front_right_p20_ < maze_exit_clear_p20_;
    return left_wall || right_wall ||
      front_min < front_clear_distance_ ||
      front_p20 < front_clear_distance_ ||
      front_p50 < maze_exit_clear_p20_ ||
      diagonal_limited;
  }

  bool should_enter_maze_follow(double front_min, double front_p20, double front_p50) const
  {
    if (!narrow_maze_follow_enabled_) {
      return false;
    }
    if (maze_only_mode_) {
      return true;
    }
    const bool left_wall = maze_left_wall_present(left_side_p20_);
    const bool right_wall = maze_right_wall_present(right_side_p20_);
    const bool side_walls = left_wall && right_wall;
    const bool any_side_wall = left_wall || right_wall;
    const bool front_continue = maze_front_continue_clear(front_min, front_p20);
    const bool oscillating = oscillation_flip_count_ >= std::max(1, oscillation_flip_limit_ - 1);
    const bool diagonal_limited =
      front_left_p20_ < maze_exit_clear_p20_ ||
      front_right_p20_ < maze_exit_clear_p20_;
    const bool front_limited =
      front_min < front_block_distance_ ||
      front_p20 < front_clear_distance_ ||
      front_p50 < maze_exit_clear_p20_;
    const bool corner_likely =
      any_side_wall &&
      (front_limited || diagonal_limited) &&
      std::abs(front_left_p20_ - front_right_p20_) >= maze_corner_front_delta_;
    const bool narrow_watch =
      front_p20 < front_clear_distance_ ||
      front_p50 < maze_exit_clear_p20_ ||
      diagonal_limited;
    const bool one_wall_corner =
      any_side_wall &&
      (front_limited || diagonal_limited || oscillating);
    return (side_walls && (front_continue || diagonal_limited || front_limited) &&
            (oscillating || narrow_watch || corner_likely)) ||
      one_wall_corner;
  }

  bool should_exit_maze_follow(double front_min, double front_p20, double front_p50, const rclcpp::Time& now)
  {
    if (!maze_follow_active_) {
      return true;
    }
    if (front_min < hard_stop_distance_) {
      maze_exit_reason_ = "contact_escape";
      return true;
    }
    if (maze_only_mode_) {
      maze_exit_reason_ = "maze_only_hold";
      return false;
    }
    if (maze_follow_start_time_.nanoseconds() != 0 &&
        (now - maze_follow_start_time_).seconds() < maze_min_hold_duration_)
    {
      maze_exit_reason_ = "min_hold";
      return false;
    }
    const bool side_open =
      !maze_left_wall_present(left_side_p20_) &&
      !maze_right_wall_present(right_side_p20_);
    const bool front_open = front_p20 >= maze_exit_clear_p20_ && front_p50 >= maze_exit_clear_p20_;
    if (side_open && front_open) {
      maze_exit_reason_ = "corridor_open";
      return true;
    }
    return false;
  }

  double route_clearance_score(double value) const
  {
    return clamp(finite_or(value, 0.0), 0.0, 3.5);
  }

  bool route_left_wall_present(double left_side_p20, double left_p20) const
  {
    const double left_near = std::min(
      finite_or(left_side_p20, std::numeric_limits<double>::infinity()),
      finite_or(left_p20, std::numeric_limits<double>::infinity()));
    return left_near <= wide_maze_side_wall_p20_;
  }

  bool route_right_wall_present(double right_side_p20, double right_p20) const
  {
    const double right_near = std::min(
      finite_or(right_side_p20, std::numeric_limits<double>::infinity()),
      finite_or(right_p20, std::numeric_limits<double>::infinity()));
    return right_near <= wide_maze_side_wall_p20_;
  }

  double route_side_open_score(double diagonal_p20, double side_p20, double legacy_side_p20) const
  {
    const double diagonal = route_clearance_score(diagonal_p20);
    const double side = std::max(route_clearance_score(side_p20), route_clearance_score(legacy_side_p20));
    return 0.70 * diagonal + 0.30 * side;
  }

  double route_front_open_score(double front_min, double front_p20, double front_p50) const
  {
    return 0.10 * route_clearance_score(front_min) +
      0.55 * route_clearance_score(front_p20) +
      0.35 * route_clearance_score(front_p50);
  }

  bool maze_reverse_required(double front_min, double front_p20) const
  {
    return front_min < maze_reverse_front_min_ || front_p20 < maze_reverse_front_p20_;
  }

  bool maze_turn_start_blocked(double front_min, double front_p20, double front_p50) const
  {
    return maze_reverse_required(front_min, front_p20) ||
      front_p20 < maze_turn_start_front_p20_ ||
      (front_p20 < front_block_distance_ && front_p50 < maze_turn_start_front_p50_);
  }

  bool deadend_side_open_for_escape(
    double direction,
    double open_score,
    double left_side,
    double right_side,
    double front_left,
    double front_right) const
  {
    return open_score >= maze_deadend_side_open_p20_ &&
      robot_swept_clear_for_turn(direction, left_side, right_side, front_left, front_right, 0.0);
  }

  void bump_maze_confirm_counter(int& counter, bool observed)
  {
    if (observed) {
      counter = std::min(counter + 1, 1000000);
    } else {
      counter = 0;
    }
  }

  void update_maze_route_confirmations(const MazeRouteObservation& observation)
  {
    bump_maze_confirm_counter(maze_left_wall_confirm_frames_, observation.left_wall);
    bump_maze_confirm_counter(maze_right_wall_confirm_frames_, observation.right_wall);
    bump_maze_confirm_counter(maze_any_wall_confirm_frames_, observation.any_wall);
    bump_maze_confirm_counter(maze_both_walls_confirm_frames_, observation.both_walls);
    bump_maze_confirm_counter(maze_front_blocked_confirm_frames_, observation.front_blocked);
    bump_maze_confirm_counter(maze_front_open_confirm_frames_, observation.front_wide_clear);
    bump_maze_confirm_counter(maze_deadend_candidate_confirm_frames_, observation.deadend_candidate);
    bump_maze_confirm_counter(maze_junction_confirm_frames_, observation.junction);
    bump_maze_confirm_counter(maze_corner_confirm_frames_, observation.corner);
  }

  int opening_index_from_direction(double direction) const
  {
    if (direction > 0.5) {
      return 0;
    }
    if (direction < -0.5) {
      return 2;
    }
    return 1;
  }

  double opening_direction_from_index(int index) const
  {
    if (index == 0) {
      return 1.0;
    }
    if (index == 2) {
      return -1.0;
    }
    return 0.0;
  }

  std::string opening_name_from_index(int index) const
  {
    if (index == 0) {
      return "left";
    }
    if (index == 2) {
      return "right";
    }
    return "front";
  }

  int find_near_topology_node(const Pose2D& pose) const
  {
    int best_id = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (const auto& node : maze_topology_nodes_) {
      const double distance = std::hypot(node.pose.x - pose.x, node.pose.y - pose.y);
      if (distance <= maze_node_merge_distance_ && distance < best_distance) {
        best_distance = distance;
        best_id = node.id;
      }
    }
    return best_id;
  }

  MazeTopologyNode& topology_node_by_id(int id)
  {
    for (auto& node : maze_topology_nodes_) {
      if (node.id == id) {
        return node;
      }
    }
    maze_topology_nodes_.push_back(MazeTopologyNode());
    auto& node = maze_topology_nodes_.back();
    node.id = id;
    return node;
  }

  MazeTopologyNode& touch_topology_node(const Pose2D& pose)
  {
    int node_id = find_near_topology_node(pose);
    if (node_id < 0) {
      node_id = next_topology_node_id_++;
      maze_topology_nodes_.push_back(MazeTopologyNode());
      auto& node = maze_topology_nodes_.back();
      node.id = node_id;
      node.pose = pose;
      return node;
    }
    auto& node = topology_node_by_id(node_id);
    node.pose.x = 0.75 * node.pose.x + 0.25 * pose.x;
    node.pose.y = 0.75 * node.pose.y + 0.25 * pose.y;
    node.pose.yaw = pose.yaw;
    return node;
  }

  int count_unvisited_openings(const MazeTopologyNode& node) const
  {
    int count = 0;
    for (size_t i = 0; i < node.open.size(); ++i) {
      if (node.open[i] && !node.visited[i]) {
        ++count;
      }
    }
    return count;
  }

  int maze_topology_edge_count() const
  {
    int count = 0;
    for (const auto& node : maze_topology_nodes_) {
      for (const bool open : node.open) {
        if (open) {
          ++count;
        }
      }
    }
    return count;
  }

  void refresh_topology_openings(MazeTopologyNode& node, const MazeRouteObservation& observation)
  {
    const double threshold = wide_maze_front_clear_p20_;
    node.open[0] = node.open[0] || left_open_score_ >= threshold;
    node.open[1] = node.open[1] || (observation.front_wide_clear && front_open_score_ >= threshold);
    node.open[2] = node.open[2] || right_open_score_ >= threshold;
    topology_unvisited_opening_count_ = count_unvisited_openings(node);
  }

  int choose_topology_opening(const MazeTopologyNode& node) const
  {
    const int preferred_side = maze_wall_follow_side_ == "left" ? 0 : 2;
    const int opposite_side = preferred_side == 0 ? 2 : 0;
    const std::array<int, 3> order{{1, preferred_side, opposite_side}};
    for (const int index : order) {
      if (node.open[index] && !node.visited[index]) {
        return index;
      }
    }
    if (node.open[1]) {
      return 1;
    }
    if (node.open[preferred_side]) {
      return preferred_side;
    }
    if (node.open[opposite_side]) {
      return opposite_side;
    }
    return -1;
  }

  void remember_backtrack_node_if_needed(const MazeTopologyNode& node)
  {
    if (count_unvisited_openings(node) <= 0) {
      return;
    }
    if (maze_backtrack_stack_.empty() || maze_backtrack_stack_.back() != node.id) {
      maze_backtrack_stack_.push_back(node.id);
    }
  }

  void set_active_topology_opening(const MazeTopologyNode& node, int opening_index)
  {
    active_topology_node_id_ = node.id;
    active_topology_opening_index_ = opening_index;
    active_topology_opening_pose_ = node.pose;
  }

  bool hold_active_topology_opening(
    const MazeTopologyNode& node,
    const Pose2D& pose,
    MazeRouteDecision& decision)
  {
    if (active_topology_node_id_ != node.id || active_topology_opening_index_ < 0) {
      return false;
    }
    const double distance_from_selection = std::hypot(
      pose.x - active_topology_opening_pose_.x,
      pose.y - active_topology_opening_pose_.y);
    if (distance_from_selection >= maze_opening_visited_distance_) {
      active_topology_node_id_ = -1;
      active_topology_opening_index_ = -1;
      return false;
    }

    decision.selected_turn_direction = opening_direction_from_index(active_topology_opening_index_);
    topology_selected_opening_ = opening_name_from_index(active_topology_opening_index_);
    decision.reason = "topology_hold_selected_opening";
    decision.state = active_topology_opening_index_ == 1 ? "choose_opening" : "committed_turn";
    return true;
  }

  MazeRouteObservation observe_maze_route(
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right)
  {
    MazeRouteObservation observation;
    observation.left_wall = route_left_wall_present(left_side, left_mid);
    observation.right_wall = route_right_wall_present(right_side, right_mid);
    observation.any_wall = observation.left_wall || observation.right_wall;
    observation.both_walls = observation.left_wall && observation.right_wall;
    observation.front_blocked = maze_turn_start_blocked(front_min, front_p20, front_p50);
    observation.front_wide_clear =
      front_min >= front_emergency_distance_ &&
      front_p20 >= wide_maze_front_clear_p20_ &&
      front_p50 >= wide_maze_front_clear_p20_;

    left_open_score_ = route_side_open_score(front_left, left_side, left_mid);
    front_open_score_ = route_front_open_score(front_min, front_p20, front_p50);
    right_open_score_ = route_side_open_score(front_right, right_side, right_mid);

    const bool left_escape_open =
      deadend_side_open_for_escape(1.0, left_open_score_, left_side, right_side, front_left, front_right);
    const bool right_escape_open =
      deadend_side_open_for_escape(-1.0, right_open_score_, left_side, right_side, front_left, front_right);
    const bool deadend_front_tight =
      maze_reverse_required(front_min, front_p20) ||
      (front_p20 < maze_deadend_front_p20_ && front_p50 < maze_turn_start_front_p50_);
    observation.deadend_candidate = deadend_front_tight && !left_escape_open && !right_escape_open;

    const double open_threshold = wide_maze_front_clear_p20_;
    int open_sectors = 0;
    if (left_open_score_ >= open_threshold) {
      ++open_sectors;
    }
    if (front_open_score_ >= open_threshold) {
      ++open_sectors;
    }
    if (right_open_score_ >= open_threshold) {
      ++open_sectors;
    }
    observation.junction = observation.front_wide_clear && open_sectors >= 2;

    if (left_open_score_ > right_open_score_ + turn_switch_margin_) {
      observation.selected_turn_direction = 1.0;
    } else if (right_open_score_ > left_open_score_ + turn_switch_margin_) {
      observation.selected_turn_direction = -1.0;
    } else {
      observation.selected_turn_direction = wall_follow_direction();
    }

    const bool side_clearly_better =
      std::max(left_open_score_, right_open_score_) >
      front_open_score_ + std::max(0.05, turn_switch_margin_);
    observation.corner =
      observation.front_blocked &&
      (side_clearly_better || std::abs(left_open_score_ - right_open_score_) >= turn_switch_margin_);
    return observation;
  }

  MazeRouteDecision choose_maze_topology_decision(
    const MazeRouteObservation& observation,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double front_min,
    double front_p20)
  {
    MazeRouteDecision decision;
    const bool front_blocked_confirmed =
      maze_front_blocked_confirm_frames_ >= maze_state_confirm_frames_;
    const bool deadend_candidate_confirmed =
      maze_deadend_candidate_confirm_frames_ >= maze_deadend_confirm_frames_;
    const bool any_wall_confirmed =
      maze_any_wall_confirm_frames_ >= maze_state_confirm_frames_;
    const bool junction_confirmed =
      maze_junction_confirm_frames_ >= maze_opening_confirm_frames_;
    const bool left_open = left_open_score_ >= wide_maze_front_clear_p20_;
    const bool front_open = observation.front_wide_clear && front_open_score_ >= wide_maze_front_clear_p20_;
    const bool right_open = right_open_score_ >= wide_maze_front_clear_p20_;
    const bool deadend_confirmed =
      deadend_candidate_confirmed ||
      (front_blocked_confirmed &&
       !left_open && !right_open &&
       front_open_score_ < maze_deadend_front_p20_);

    topology_selected_opening_ = "none";

    if (deadend_confirmed) {
      auto& node = touch_topology_node(pose);
      node.deadend = true;
      current_topology_node_id_ = node.id;
      active_topology_node_id_ = -1;
      active_topology_opening_index_ = -1;
      topology_unvisited_opening_count_ = count_unvisited_openings(node);
      decision.state = maze_backtrack_stack_.empty() ? "deadend_reverse" : "backtrack_to_node";
      decision.reason = deadend_candidate_confirmed ? "topology_deadend_forced" : "topology_deadend";
      decision.selected_turn_direction = wall_follow_direction();
      topology_selected_opening_ = "backtrack";
      set_maze_turn_phase("maze_topology_deadend_reverse", now);
      return decision;
    }

    if (maze_reverse_required(front_min, front_p20)) {
      decision.state = "preclear_reverse";
      decision.reason = "front_contact_risk_confirmed";
      decision.selected_turn_direction = observation.selected_turn_direction;
      return decision;
    }

    const bool stable_junction =
      junction_confirmed && (any_wall_confirmed || front_blocked_confirmed || !front_open);

    if (stable_junction) {
      auto& node = touch_topology_node(pose);
      current_topology_node_id_ = node.id;
      refresh_topology_openings(node, observation);
      if (hold_active_topology_opening(node, pose, decision)) {
        topology_unvisited_opening_count_ = count_unvisited_openings(node);
        return decision;
      }
      const int selected_index = choose_topology_opening(node);
      topology_unvisited_opening_count_ = count_unvisited_openings(node);
      if (selected_index >= 0) {
        node.visited[selected_index] = true;
        set_active_topology_opening(node, selected_index);
        remember_backtrack_node_if_needed(node);
        topology_unvisited_opening_count_ = count_unvisited_openings(node);
        decision.selected_turn_direction = opening_direction_from_index(selected_index);
        topology_selected_opening_ = opening_name_from_index(selected_index);
        decision.reason = "topology_unvisited_opening";
        if (selected_index == 1) {
          decision.state = "choose_opening";
        } else {
          decision.state = "committed_turn";
        }
        set_maze_turn_phase("maze_topology_junction_scan", now);
        return decision;
      }

      if (!maze_backtrack_stack_.empty()) {
        if (maze_backtrack_stack_.back() == node.id) {
          maze_backtrack_stack_.pop_back();
        }
        decision.state = "backtrack_to_node";
        decision.reason = "topology_all_openings_visited";
        decision.selected_turn_direction = wall_follow_direction();
        topology_selected_opening_ = "backtrack";
        return decision;
      }

      decision.state = "complete_check";
      decision.reason = "topology_no_unvisited_opening";
      decision.selected_turn_direction = front_open ? 0.0 : observation.selected_turn_direction;
      topology_selected_opening_ = front_open ? "front" : direction_name(decision.selected_turn_direction);
      return decision;
    }

    if (front_blocked_confirmed) {
      decision.selected_turn_direction = observation.selected_turn_direction;
      if (std::max(left_open_score_, right_open_score_) >= maze_front_p20_continue_) {
        decision.state = "committed_turn";
        decision.reason = "topology_front_blocked_open_side";
      } else {
        decision.state = "preclear_reverse";
        decision.reason = "topology_front_blocked_preclear";
      }
      topology_selected_opening_ = direction_name(decision.selected_turn_direction);
      return decision;
    }

    if (any_wall_confirmed) {
      decision.state = "corridor_drive";
      decision.reason = observation.both_walls ? "topology_corridor_two_walls" : "topology_corridor_single_wall";
      decision.selected_turn_direction = 0.0;
      topology_selected_opening_ = "front";
      return decision;
    }

    decision.state = "drive_open";
    decision.reason = observation.front_wide_clear ? "topology_front_wide_clear" : "topology_wide_drive_forward";
    decision.selected_turn_direction = 0.0;
    topology_selected_opening_ = "front";
    return decision;
  }

  MazeRouteDecision choose_maze_route_decision(
    const MazeRouteObservation& observation,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double front_min,
    double front_p20)
  {
    if (maze_topology_mode_) {
      return choose_maze_topology_decision(observation, pose, now, front_min, front_p20);
    }
    MazeRouteDecision decision;
    const bool front_blocked_confirmed =
      maze_front_blocked_confirm_frames_ >= maze_state_confirm_frames_;
    const bool deadend_candidate_confirmed =
      maze_deadend_candidate_confirm_frames_ >= maze_deadend_confirm_frames_;
    const bool any_wall_confirmed =
      maze_any_wall_confirm_frames_ >= maze_state_confirm_frames_;
    const bool junction_confirmed =
      maze_junction_confirm_frames_ >= maze_state_confirm_frames_;
    const bool corner_confirmed =
      maze_corner_confirm_frames_ >= maze_state_confirm_frames_;

    if (deadend_candidate_confirmed) {
      decision.state = "deadend_reverse";
      decision.reason = "deadend_forced";
      decision.selected_turn_direction = wall_follow_direction();
      return decision;
    }

    if (maze_reverse_required(front_min, front_p20)) {
      decision.state = "preclear_reverse";
      decision.reason = "front_contact_risk_confirmed";
      decision.selected_turn_direction = observation.selected_turn_direction;
      return decision;
    }
    if (front_blocked_confirmed) {
      decision.selected_turn_direction = observation.selected_turn_direction;
      if (corner_confirmed ||
          std::max(left_open_score_, right_open_score_) >= maze_front_p20_continue_)
      {
        decision.state = "corner_turn";
        decision.reason = corner_confirmed ? "confirmed_corner" : "front_blocked_choose_open_side";
      } else {
        decision.state = "preclear_reverse";
        decision.reason = "front_blocked_no_open_side";
      }
      return decision;
    }

    if (junction_confirmed) {
      decision.state = "junction_select";
      decision.reason = "confirmed_multi_sector_opening";
      const double best_side = std::max(left_open_score_, right_open_score_);
      if (front_open_score_ >= best_side - std::max(0.08, turn_switch_margin_)) {
        decision.selected_turn_direction = 0.0;
      } else {
        decision.selected_turn_direction = left_open_score_ >= right_open_score_ ? 1.0 : -1.0;
      }
      return decision;
    }

    if (any_wall_confirmed) {
      decision.state = "corridor_follow";
      decision.reason = observation.both_walls ? "confirmed_two_side_walls" : "confirmed_single_side_wall";
      decision.selected_turn_direction = 0.0;
      return decision;
    }

    decision.state = "drive_open";
    decision.reason = observation.front_wide_clear ? "front_wide_clear" : "wide_ambiguous_drive_forward";
    decision.selected_turn_direction = 0.0;
    return decision;
  }

  double choose_maze_route_turn_direction(double candidate, const rclcpp::Time& now)
  {
    if (candidate == 0.0) {
      candidate = wall_follow_direction();
    }
    candidate = candidate >= 0.0 ? 1.0 : -1.0;
    turn_switch_blocked_ = false;
    if (!maze_turn_lock_active_) {
      maze_turn_lock_active_ = true;
      maze_turn_direction_ = candidate;
      maze_turn_lock_start_time_ = now;
      maze_turn_commit_active_ = true;
      maze_turn_commit_start_time_ = now;
      maze_selected_turn_side_ = direction_name(maze_turn_direction_);
      return maze_turn_direction_;
    }

    const bool lock_expired = (now - maze_turn_lock_start_time_).seconds() >= maze_turn_lock_duration_;
    const double locked_score = maze_turn_direction_ >= 0.0 ? left_open_score_ : right_open_score_;
    const double candidate_score = candidate >= 0.0 ? left_open_score_ : right_open_score_;
    if (candidate != maze_turn_direction_ && (!lock_expired || candidate_score < locked_score + turn_switch_margin_)) {
      turn_switch_blocked_ = true;
      maze_selected_turn_side_ = direction_name(maze_turn_direction_);
      return maze_turn_direction_;
    }

    if (candidate != maze_turn_direction_) {
      maze_turn_direction_ = candidate;
      maze_turn_lock_start_time_ = now;
      maze_turn_commit_active_ = true;
      maze_turn_commit_start_time_ = now;
    }
    maze_selected_turn_side_ = direction_name(maze_turn_direction_);
    return maze_turn_direction_;
  }

  double choose_maze_turn_direction(const rclcpp::Time& now)
  {
    const double left_score = turn_open_score_for_direction(1.0);
    const double right_score = turn_open_score_for_direction(-1.0);
    double front_min = std::numeric_limits<double>::infinity();
    double front_p20 = std::numeric_limits<double>::infinity();
    double front_left = std::numeric_limits<double>::infinity();
    double front_right = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_min = front_min_;
      front_p20 = front_p20_;
      front_left = front_left_p20_;
      front_right = front_right_p20_;
    }
    const bool release_clear = maze_turn_release_clear(front_min, front_p20, front_left, front_right);
    const double commit_age =
      maze_turn_commit_start_time_.nanoseconds() == 0 ? 0.0 :
        std::max(0.0, (now - maze_turn_commit_start_time_).seconds());
    const bool commit_within_duration =
      maze_turn_commit_active_ && commit_age < maze_turn_commit_duration_;
    const bool left_tight = front_left + maze_corner_front_delta_ < front_right &&
      front_left < maze_exit_clear_p20_;
    const bool right_tight = front_right + maze_corner_front_delta_ < front_left &&
      front_right < maze_exit_clear_p20_;
    double candidate = wall_follow_direction();
    const bool wall_follow_side_viable =
      candidate >= 0.0 ? !left_tight : !right_tight;
    if (!wall_follow_side_viable) {
      candidate = -candidate;
    } else if (std::abs(left_score - right_score) > turn_switch_margin_ * 2.0) {
      const double more_open = left_score >= right_score ? 1.0 : -1.0;
      if (more_open != candidate &&
          std::max(left_score, right_score) > std::min(left_score, right_score) + maze_swept_clearance_release_)
      {
        candidate = more_open;
      }
    }
    if (left_tight && !right_tight) {
      candidate = -1.0;
    } else if (right_tight && !left_tight) {
      candidate = 1.0;
    }
    if (commit_within_duration && !release_clear) {
      maze_selected_turn_side_ = direction_name(maze_turn_direction_);
      turn_switch_blocked_ = true;
      return maze_turn_direction_;
    }
    if (maze_turn_commit_active_ && !commit_within_duration) {
      maze_turn_commit_active_ = false;
    }
    if (!maze_turn_lock_active_) {
      maze_turn_lock_active_ = true;
      maze_turn_direction_ = candidate;
      maze_turn_lock_start_time_ = now;
      maze_turn_commit_active_ = true;
      maze_turn_commit_start_time_ = now;
      maze_selected_turn_side_ = direction_name(maze_turn_direction_);
      return maze_turn_direction_;
    }
    const bool lock_expired = (now - maze_turn_lock_start_time_).seconds() >= maze_turn_lock_duration_;
    const double locked_score = maze_turn_direction_ >= 0.0 ? left_score : right_score;
    const double opposite_score = maze_turn_direction_ >= 0.0 ? right_score : left_score;
    if (candidate != maze_turn_direction_ && lock_expired && opposite_score > locked_score + turn_switch_margin_) {
      maze_turn_direction_ = candidate;
      maze_turn_lock_start_time_ = now;
      maze_turn_commit_active_ = true;
      maze_turn_commit_start_time_ = now;
    }
    maze_selected_turn_side_ = direction_name(maze_turn_direction_);
    return maze_turn_direction_;
  }

  bool maze_turn_release_clear(
    double front_min,
    double front_p20,
    double front_left,
    double front_right) const
  {
    return finite_or(front_min, 0.0) >= front_clear_distance_ &&
      finite_or(front_p20, 0.0) >= maze_exit_clear_p20_ &&
      finite_or(front_left, 0.0) >= maze_exit_clear_p20_ &&
      finite_or(front_right, 0.0) >= maze_exit_clear_p20_;
  }

  double robot_half_width_with_margin() const
  {
    return robot_width_ * 0.5 + robot_safety_margin_;
  }

  double robot_front_overhang_from_laser() const
  {
    return std::max(0.0, robot_length_ * 0.5 - laser_x_offset_);
  }

  double direction_side_clearance(double direction, double left_side, double right_side) const
  {
    return finite_or(direction >= 0.0 ? left_side : right_side, 0.0);
  }

  double direction_diagonal_clearance(double direction, double front_left, double front_right) const
  {
    return finite_or(direction >= 0.0 ? front_left : front_right, 0.0);
  }

  double estimate_inner_front_corner_clearance(
    double direction,
    double left_side,
    double right_side,
    double front_left,
    double front_right) const
  {
    const double side_clearance = direction_side_clearance(direction, left_side, right_side);
    const double diagonal_clearance = direction_diagonal_clearance(direction, front_left, front_right);
    const double side_gap = side_clearance - robot_half_width_with_margin();
    const double diagonal_gap = diagonal_clearance - 0.45 * robot_front_overhang_from_laser();
    return std::min(side_gap, diagonal_gap);
  }

  double estimate_swept_clearance(
    double direction,
    double left_side,
    double right_side,
    double front_left,
    double front_right) const
  {
    const double inner_front = estimate_inner_front_corner_clearance(
      direction, left_side, right_side, front_left, front_right);
    const double inner_side = direction_side_clearance(direction, left_side, right_side) -
      robot_half_width_with_margin();
    return std::min(inner_front, inner_side);
  }

  bool robot_swept_clear_for_turn(
    double direction,
    double left_side,
    double right_side,
    double front_left,
    double front_right,
    double threshold) const
  {
    if (!maze_use_robot_swept_volume_) {
      return true;
    }
    return estimate_swept_clearance(direction, left_side, right_side, front_left, front_right) >= threshold;
  }

  double maze_inner_clearance_for_direction(
    double direction,
    double left_side,
    double right_side,
    double front_left,
    double front_right) const
  {
    const double side = direction >= 0.0 ? left_side : right_side;
    const double diagonal = direction >= 0.0 ? front_left : front_right;
    return std::min(finite_or(side, 0.0), finite_or(diagonal, 0.0));
  }

  bool maze_inner_clear_for_turn(
    double direction,
    double left_side,
    double right_side,
    double front_left,
    double front_right,
    double threshold) const
  {
    const double side = finite_or(direction >= 0.0 ? left_side : right_side, 0.0);
    const double diagonal = finite_or(direction >= 0.0 ? front_left : front_right, 0.0);
    return side >= threshold &&
      diagonal >= maze_front_diagonal_clearance_min_p20_ &&
      robot_swept_clear_for_turn(direction, left_side, right_side, front_left, front_right, 0.0);
  }

  void set_maze_turn_phase(const std::string& phase, const rclcpp::Time& now)
  {
    if (maze_turn_phase_ != phase) {
      maze_turn_phase_ = phase;
      maze_turn_phase_start_time_ = now;
    }
  }

  void reset_maze_corner_turn_latch(const std::string& reason)
  {
    maze_corner_turn_latched_ = false;
    maze_corner_latched_direction_ = 0.0;
    maze_corner_release_confirm_frames_ = 0;
    maze_corner_abort_confirm_frames_ = 0;
    maze_corner_preclear_attempts_ = 0;
    maze_corner_preclear_total_distance_ = 0.0;
    maze_corner_turn_state_ = "none";
    maze_corner_cycle_count_ = 0;
    maze_corner_pivot_yaw_delta_ = 0.0;
    maze_corner_reverse_distance_ = 0.0;
    maze_corner_selected_side_ = "none";
    maze_corner_latch_reason_ = reason;
  }

  void set_maze_post_turn_straight(const rclcpp::Time& now, const std::string& reason)
  {
    set_maze_turn_phase("maze_post_turn_straight", now);
    maze_turn_commit_active_ = false;
    maze_turn_lock_active_ = false;
    maze_last_angular_command_ = 0.0;
    maze_corner_reason_ = reason;
    reset_maze_corner_turn_latch(reason);
  }

  void set_deadend_turn_phase(const std::string& phase, const Pose2D& pose, const rclcpp::Time& now)
  {
    if (deadend_turn_phase_ == phase) {
      return;
    }
    deadend_turn_phase_ = phase;
    deadend_turn_phase_start_time_ = now;
    if (phase == "deadend_turn_in_place") {
      deadend_turn_start_yaw_ = pose.yaw;
      deadend_turn_yaw_delta_ = 0.0;
      deadend_front_drive_clear_confirm_frames_ = 0;
      maze_last_angular_command_ = 0.0;
    } else if (phase == "deadend_reverse_to_turn_spot") {
      deadend_reverse_start_pose_ = pose;
      deadend_reverse_chunk_progress_ = 0.0;
      deadend_front_drive_clear_confirm_frames_ = 0;
      maze_last_angular_command_ = 0.0;
    } else if (phase == "deadend_post_turn_straight") {
      maze_last_angular_command_ = 0.0;
    } else if (phase == "deadend_no_safe_turn_spot") {
      maze_last_angular_command_ = 0.0;
    }
  }

  double deadend_side_clearance(double side, double legacy_side) const
  {
    return std::min(
      finite_or(side, std::numeric_limits<double>::infinity()),
      finite_or(legacy_side, std::numeric_limits<double>::infinity()));
  }

  bool deadend_turn_space_available(
    double front_p20,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right,
    double rear)
  {
    const double left_clearance = deadend_side_clearance(left_side, left_mid);
    const double right_clearance = deadend_side_clearance(right_side, right_mid);
    const double front_clearance = std::min(
      finite_or(front_p20, std::numeric_limits<double>::infinity()),
      std::min(
        finite_or(front_left, std::numeric_limits<double>::infinity()),
        finite_or(front_right, std::numeric_limits<double>::infinity())));
    const double rear_clearance = finite_or(rear, std::numeric_limits<double>::infinity());

    deadend_turn_space_ok_ =
      left_clearance >= maze_deadend_in_place_min_side_p20_ &&
      right_clearance >= maze_deadend_in_place_min_side_p20_ &&
      front_clearance >= maze_deadend_in_place_min_front_p20_ &&
      rear_clearance >= maze_deadend_in_place_min_rear_p20_;
    return deadend_turn_space_ok_;
  }

  double choose_deadend_turn_direction() const
  {
    const double recent_penalty = std::max(0.02, turn_switch_margin_ * 0.75);
    const double left_score =
      left_open_score_ - (last_deadend_turn_direction_ > 0.5 ? recent_penalty : 0.0);
    const double right_score =
      right_open_score_ - (last_deadend_turn_direction_ < -0.5 ? recent_penalty : 0.0);
    if (left_score > right_score + turn_switch_margin_) {
      return 1.0;
    }
    if (right_score > left_score + turn_switch_margin_) {
      return -1.0;
    }
    if (std::abs(last_deadend_turn_direction_) > 0.5) {
      return -last_deadend_turn_direction_;
    }
    return wall_follow_direction();
  }

  bool reverse_rear_guard_blocks(double rear)
  {
    deadend_reverse_rear_clearance_ = finite_or(rear, std::numeric_limits<double>::infinity());
    if (!std::isfinite(deadend_reverse_rear_clearance_)) {
      maze_reverse_rear_guard_active_ = false;
      return false;
    }
    if (maze_reverse_rear_guard_active_) {
      if (deadend_reverse_rear_clearance_ >= maze_reverse_rear_release_p20_) {
        maze_reverse_rear_guard_active_ = false;
      }
    } else if (deadend_reverse_rear_clearance_ < maze_reverse_rear_stop_p20_) {
      maze_reverse_rear_guard_active_ = true;
    }
    return maze_reverse_rear_guard_active_;
  }

  bool deadend_front_clear_for_drive(
    double front_min,
    double front_p20,
    double front_p50,
    double front_left,
    double front_right) const
  {
    const double front_nearest = finite_or(front_min, std::numeric_limits<double>::infinity());
    const double front_band = finite_or(front_p20, std::numeric_limits<double>::infinity());
    const double front_median = finite_or(front_p50, std::numeric_limits<double>::infinity());
    const double diagonal_clearance = std::min(
      finite_or(front_left, std::numeric_limits<double>::infinity()),
      finite_or(front_right, std::numeric_limits<double>::infinity()));
    return front_nearest >= front_emergency_distance_ &&
      front_band >= maze_deadend_front_drive_clear_p20_ &&
      front_median >= maze_deadend_front_drive_clear_p50_ &&
      diagonal_clearance >= maze_front_diagonal_clearance_min_p20_;
  }

  bool choose_open_side_turn_direction(
    double current_direction,
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double front_left,
    double front_right,
    double& selected_direction,
    double& selected_score)
  {
    maze_open_side_turn_active_ = false;
    maze_open_side_turn_side_ = "none";
    maze_open_side_turn_score_ = 0.0;
    if (!maze_open_side_turn_enabled_ || front_min < hard_stop_distance_) {
      return false;
    }

    const bool front_clear_enough =
      finite_or(front_p20, 0.0) >= maze_open_side_front_clear_p20_ ||
      finite_or(front_p50, 0.0) >= maze_open_side_front_clear_p50_ ||
      std::max(left_open_score_, right_open_score_) >= maze_open_side_turn_min_score_;
    const bool left_clear =
      left_open_score_ >= maze_open_side_turn_min_score_ &&
      robot_swept_clear_for_turn(1.0, left_side, right_side, front_left, front_right, 0.0);
    const bool right_clear =
      right_open_score_ >= maze_open_side_turn_min_score_ &&
      robot_swept_clear_for_turn(-1.0, left_side, right_side, front_left, front_right, 0.0);
    if (!front_clear_enough || (!left_clear && !right_clear)) {
      return false;
    }

    double direction = 0.0;
    double score = 0.0;
    if (left_clear && right_clear) {
      if (left_open_score_ > right_open_score_ + maze_open_side_turn_score_margin_) {
        direction = 1.0;
        score = left_open_score_;
      } else if (right_open_score_ > left_open_score_ + maze_open_side_turn_score_margin_) {
        direction = -1.0;
        score = right_open_score_;
      } else if (current_direction > 0.5) {
        direction = 1.0;
        score = left_open_score_;
      } else if (current_direction < -0.5) {
        direction = -1.0;
        score = right_open_score_;
      } else {
        direction = wall_follow_direction();
        score = direction > 0.0 ? left_open_score_ : right_open_score_;
      }
    } else if (left_clear) {
      direction = 1.0;
      score = left_open_score_;
    } else {
      direction = -1.0;
      score = right_open_score_;
    }

    selected_direction = direction >= 0.0 ? 1.0 : -1.0;
    selected_score = score;
    maze_open_side_turn_active_ = true;
    maze_open_side_turn_side_ = direction_name(selected_direction);
    maze_open_side_turn_score_ = selected_score;
    return true;
  }

  bool maze_rotate_front_clear_for_drive(double front_min, double front_p20, double front_p50) const
  {
    return finite_or(front_min, std::numeric_limits<double>::infinity()) >= hard_stop_distance_ &&
      (finite_or(front_p20, 0.0) >= maze_rotate_front_clear_p20_ ||
       finite_or(front_p50, 0.0) >= maze_rotate_front_clear_p50_);
  }

  double choose_maze_rotate_search_direction(
    double current_direction,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right,
    double& selected_score) const
  {
    const double left_score = route_side_open_score(front_left, left_side, left_mid);
    const double right_score = route_side_open_score(front_right, right_side, right_mid);
    const bool left_safe = robot_swept_clear_for_turn(1.0, left_side, right_side, front_left, front_right, 0.0);
    const bool right_safe = robot_swept_clear_for_turn(-1.0, left_side, right_side, front_left, front_right, 0.0);
    if (!left_safe && !right_safe) {
      selected_score = 0.0;
      return 0.0;
    }
    if (left_safe && !right_safe) {
      selected_score = left_score;
      return 1.0;
    }
    if (right_safe && !left_safe) {
      selected_score = right_score;
      return -1.0;
    }
    if (left_score > right_score + maze_open_side_turn_score_margin_) {
      selected_score = left_score;
      return 1.0;
    }
    if (right_score > left_score + maze_open_side_turn_score_margin_) {
      selected_score = right_score;
      return -1.0;
    }
    if (current_direction > 0.5) {
      selected_score = left_score;
      return 1.0;
    }
    if (current_direction < -0.5) {
      selected_score = right_score;
      return -1.0;
    }
    const double fallback = wall_follow_direction();
    selected_score = fallback >= 0.0 ? left_score : right_score;
    return fallback >= 0.0 ? 1.0 : -1.0;
  }

  bool reason_requests_no_reverse_search(const std::string& reason) const
  {
    return reason.find("reverse") != std::string::npos ||
      reason.find("backout") != std::string::npos ||
      reason.find("backup") != std::string::npos ||
      reason.find("preclear") != std::string::npos ||
      reason.find("contact") != std::string::npos;
  }

  bool apply_maze_no_reverse_mode(
    geometry_msgs::msg::Twist& cmd,
    const rclcpp::Time& now,
    const std::string& reason,
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right)
  {
    if (!maze_no_reverse_mode_) {
      return false;
    }

    const bool reverse_requested = cmd.linear.x < -0.002;
    maze_reverse_blocked_by_guard_ = reverse_requested;
    if (reverse_requested) {
      cmd.linear.x = 0.0;
    }
    const bool reverse_state_requested =
      cmd.linear.x <= 0.002 && reason_requests_no_reverse_search(reason);
    const bool front_clear = maze_rotate_front_clear_for_drive(front_min, front_p20, front_p50);
    bump_maze_confirm_counter(maze_rotate_front_clear_confirm_frames_, front_clear);
    const bool front_clear_confirmed =
      maze_rotate_front_clear_confirm_frames_ >= maze_rotate_front_clear_confirm_frames_required_;

    if (!maze_rotate_search_active_ && !reverse_requested && !reverse_state_requested) {
      maze_rotate_search_elapsed_s_ = 0.0;
      return false;
    }

    if (maze_rotate_search_active_ && front_clear_confirmed) {
      maze_reverse_disabled_reason_ = "rotate_search_front_clear";
      maze_rotate_search_active_ = false;
      maze_rotate_search_reason_ = "front_clear";
      maze_rotate_search_exit_reason_ = "front_clear";
      maze_rotate_search_direction_ = 0.0;
      maze_rotate_search_switch_count_ = 0;
      maze_rotate_search_elapsed_s_ = 0.0;
      set_maze_post_turn_straight(now, "rotate_search_front_clear");
      cmd.linear.x = std::max(maze_turn_forward_speed_, maze_forward_speed_ * 0.75);
      cmd.angular.z = 0.0;
      maze_action_ = "maze_post_turn_straight";
      maze_corner_phase_ = "maze_post_turn_straight";
      maze_preclear_action_ = "post_turn_straight";
      maze_speed_profile_ = "maze_post_turn_straight";
      maze_last_angular_command_ = 0.0;
      return true;
    }

    double selected_score = 0.0;
    double selected_direction = choose_maze_rotate_search_direction(
      maze_rotate_search_active_ ? maze_rotate_search_direction_ : maze_turn_direction_,
      left_side, right_side, left_mid, right_mid, front_left, front_right, selected_score);
    if (std::abs(selected_direction) < 0.1) {
      maze_rotate_search_direction_ = 0.0;
    }

    if (!maze_rotate_search_active_) {
      maze_rotate_search_active_ = true;
      maze_rotate_search_reason_ = reason;
      maze_rotate_search_start_time_ = now;
      maze_rotate_search_direction_start_time_ = now;
      maze_rotate_search_switch_count_ = 0;
      maze_rotate_front_clear_confirm_frames_ = 0;
      maze_rotate_search_direction_ = selected_direction;
      maze_rotate_search_exit_reason_ = "searching";
      set_maze_turn_phase("maze_rotate_search", now);
    }

    maze_rotate_search_elapsed_s_ =
      maze_rotate_search_start_time_.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now - maze_rotate_search_start_time_).seconds());
    const double direction_elapsed =
      maze_rotate_search_direction_start_time_.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now - maze_rotate_search_direction_start_time_).seconds());

    if (maze_rotate_search_elapsed_s_ >= maze_rotate_search_total_timeout_) {
      maze_rotate_search_exit_reason_ = "continue_search_after_timeout";
      maze_reverse_disabled_reason_ = reason + "_continue_search_after_timeout";
    }

    if (direction_elapsed >= maze_rotate_search_direction_timeout_ &&
        maze_rotate_search_switch_count_ < maze_rotate_search_max_switches_)
    {
      const double opposite = maze_rotate_search_direction_ >= 0.0 ? -1.0 : 1.0;
      const bool opposite_safe = robot_swept_clear_for_turn(
        opposite, left_side, right_side, front_left, front_right, 0.0);
      if (opposite_safe) {
        maze_rotate_search_direction_ = opposite;
        ++maze_rotate_search_switch_count_;
        maze_rotate_search_direction_start_time_ = now;
        maze_rotate_search_exit_reason_ = "switch_direction_timeout";
        selected_score = opposite >= 0.0 ?
          route_side_open_score(front_left, left_side, left_mid) :
          route_side_open_score(front_right, right_side, right_mid);
      }
    } else if (std::abs(maze_rotate_search_direction_) < 0.1) {
      maze_rotate_search_direction_ = selected_direction;
    }

    if (std::abs(maze_rotate_search_direction_) < 0.1 || maze_rotate_search_speed_ <= 0.0) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_action_ = "maze_rotate_search_blocked";
      maze_corner_phase_ = "maze_rotate_search_blocked";
      maze_turn_phase_ = "maze_rotate_search_blocked";
      maze_preclear_action_ = "rotate_search_blocked";
      maze_corner_reason_ = "maze_rotate_no_safe_side";
      maze_speed_profile_ = "maze_rotate_search_blocked";
      maze_reverse_disabled_reason_ = reason + "_no_safe_rotate_side";
      maze_rotate_search_reason_ = "no_safe_rotate_side";
      maze_rotate_search_exit_reason_ = "no_safe_rotate_side";
      maze_last_angular_command_ = 0.0;
      return true;
    }

    cmd.linear.x = 0.0;
    cmd.angular.z = maze_rotate_search_direction_ * maze_rotate_search_speed_;
    maze_turn_direction_ = maze_rotate_search_direction_;
    maze_selected_turn_side_ = direction_name(maze_rotate_search_direction_);
    maze_action_ = "maze_rotate_search";
    maze_corner_phase_ = "maze_rotate_search";
    maze_turn_phase_ = "maze_rotate_search";
    maze_preclear_action_ = "rotate_search";
    maze_corner_reason_ = reason;
    maze_speed_profile_ = "maze_rotate_search";
    maze_reverse_disabled_reason_ = reason;
    maze_rotate_search_reason_ = reason;
    if (maze_rotate_search_exit_reason_ == "none" ||
        maze_rotate_search_exit_reason_ == "front_clear" ||
        maze_rotate_search_exit_reason_ == "no_safe_rotate_side")
    {
      maze_rotate_search_exit_reason_ = "searching";
    }
    maze_open_side_turn_score_ = selected_score;
    maze_last_angular_command_ = cmd.angular.z;
    return true;
  }

  double graph_state_age(const rclcpp::Time& now) const
  {
    return graph_state_start_time_.nanoseconds() == 0 ?
      0.0 : std::max(0.0, (now - graph_state_start_time_).seconds());
  }

  void set_graph_state(const std::string& state, const rclcpp::Time& now, const std::string& reason)
  {
    if (graph_explore_state_ != state) {
      graph_state_start_time_ = now;
    }
    graph_explore_state_ = state;
    maze_route_state_ = state;
    maze_route_reason_ = reason;
    maze_action_ = state;
    maze_corner_phase_ = state;
    maze_turn_phase_ = state;
    maze_corner_reason_ = reason;
    maze_speed_profile_ = "graph_dfs";
  }

  GraphOpenings observe_graph_openings(
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right)
  {
    GraphOpenings openings;
    openings.score[0] = 0.7 * finite_or(front_left, 0.0) + 0.3 * finite_or(left_side, left_mid);
    openings.score[1] = 0.6 * finite_or(front_p20, 0.0) + 0.4 * finite_or(front_p50, 0.0);
    openings.score[2] = 0.7 * finite_or(front_right, 0.0) + 0.3 * finite_or(right_side, right_mid);
    openings.open[1] =
      finite_or(front_min, std::numeric_limits<double>::infinity()) >= hard_stop_distance_ &&
      (finite_or(front_p20, 0.0) >= maze_graph_front_open_p20_ ||
       finite_or(front_p50, 0.0) >= maze_graph_front_open_p50_);
    const bool left_swept_safe = robot_swept_clear_for_turn(
      1.0, left_side, right_side, front_left, front_right, 0.0);
    const bool right_swept_safe = robot_swept_clear_for_turn(
      -1.0, left_side, right_side, front_left, front_right, 0.0);
    const bool left_distance_safe =
      finite_or(front_left, 0.0) >= maze_graph_opening_score_threshold_ &&
      std::max(finite_or(left_side, 0.0), finite_or(left_mid, 0.0)) >= maze_wall_target_clearance_;
    const bool right_distance_safe =
      finite_or(front_right, 0.0) >= maze_graph_opening_score_threshold_ &&
      std::max(finite_or(right_side, 0.0), finite_or(right_mid, 0.0)) >= maze_wall_target_clearance_;
    openings.open[0] =
      openings.score[0] >= maze_graph_opening_score_threshold_ &&
      (left_swept_safe || left_distance_safe);
    openings.open[2] =
      openings.score[2] >= maze_graph_opening_score_threshold_ &&
      (right_swept_safe || right_distance_safe);
    openings.open_count =
      static_cast<int>(openings.open[0]) +
      static_cast<int>(openings.open[1]) +
      static_cast<int>(openings.open[2]);
    graph_left_score_ = openings.score[0];
    graph_front_score_ = openings.score[1];
    graph_right_score_ = openings.score[2];
    return openings;
  }

  bool graph_has_unvisited_openings() const
  {
    for (const auto& node : maze_topology_nodes_) {
      for (size_t i = 0; i < node.open.size(); ++i) {
        if (node.open[i] && !node.visited[i]) {
          return true;
        }
      }
    }
    return false;
  }

  int choose_graph_opening(MazeTopologyNode& node, const GraphOpenings& openings)
  {
    for (size_t i = 0; i < node.open.size(); ++i) {
      node.open[i] = node.open[i] || openings.open[i];
    }
    int best = -1;
    double best_score = -1.0;
    for (int i = 0; i < 3; ++i) {
      if (node.open[i] && !node.visited[i] && openings.open[i] && openings.score[i] > best_score) {
        best = i;
        best_score = openings.score[i];
      }
    }
    if (best >= 0) {
      if (count_unvisited_openings(node) > 1 &&
          (maze_backtrack_stack_.empty() || maze_backtrack_stack_.back() != node.id))
      {
        maze_backtrack_stack_.push_back(node.id);
      }
    }
    topology_unvisited_opening_count_ = count_unvisited_openings(node);
    return best;
  }

  bool active_graph_opening_departed(const Pose2D& pose) const
  {
    if (active_topology_node_id_ < 0 || active_topology_opening_index_ < 0) {
      return false;
    }
    const double distance = std::hypot(
      pose.x - active_topology_opening_pose_.x,
      pose.y - active_topology_opening_pose_.y);
    return distance >= maze_opening_visited_distance_;
  }

  bool active_graph_opening_departing(const Pose2D& pose) const
  {
    return active_topology_node_id_ >= 0 &&
      active_topology_opening_index_ >= 0 &&
      !active_graph_opening_departed(pose);
  }

  void mark_active_graph_opening_if_departed(const Pose2D& pose)
  {
    if (!active_graph_opening_departed(pose)) {
      return;
    }
    auto& node = topology_node_by_id(active_topology_node_id_);
    if (active_topology_opening_index_ >= 0 && active_topology_opening_index_ < 3) {
      node.visited[active_topology_opening_index_] = true;
      topology_unvisited_opening_count_ = count_unvisited_openings(node);
    }
    active_topology_node_id_ = -1;
    active_topology_opening_index_ = -1;
  }

  void start_graph_post_turn(
    const Pose2D& pose,
    const rclcpp::Time& now,
    const std::string& reason)
  {
    graph_post_turn_start_pose_ = pose;
    graph_post_turn_advance_distance_ = 0.0;
    set_graph_state("post_turn_straight", now, reason);
  }

  void start_graph_turn(const Pose2D& pose, const rclcpp::Time& now, int opening_index, double turn_direction)
  {
    graph_selected_opening_index_ = opening_index;
    graph_turn_direction_ = turn_direction >= 0.0 ? 1.0 : -1.0;
    graph_turn_start_yaw_ = pose.yaw;
    graph_turn_target_yaw_ = opening_index == 1 ? 0.0 : (opening_index == 3 ? M_PI : M_PI * 0.5);
    topology_selected_opening_ = opening_index == 3 ? "backtrack" : opening_name_from_index(opening_index);
    if (opening_index == 1) {
      start_graph_post_turn(pose, now, "graph_selected_front");
    } else if (opening_index == 0 || opening_index == 2) {
      graph_pre_turn_shift_start_pose_ = pose;
      graph_pre_turn_shift_distance_ = 0.0;
      graph_pre_turn_shift_outer_clearance_ = 0.0;
      graph_pre_turn_shift_skipped_ = false;
      set_graph_state("pre_turn_outward_shift", now, "graph_selected_side");
    } else {
      set_graph_state("turn_to_opening", now, "graph_selected_opening");
    }
  }

  bool publish_graph_dfs_exploration_command(
    const Pose2D& pose,
    const rclcpp::Time& now,
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right)
  {
    geometry_msgs::msg::Twist cmd;
    const GraphOpenings openings = observe_graph_openings(
      front_min, front_p20, front_p50, left_side, right_side, left_mid, right_mid, front_left, front_right);
    mark_active_graph_opening_if_departed(pose);
    const bool departing_opening = active_graph_opening_departing(pose);
    const bool front_obstacle =
      finite_or(front_min, std::numeric_limits<double>::infinity()) < hard_stop_distance_ ||
      finite_or(front_p20, 0.0) < maze_graph_front_obstacle_p20_;
    const bool junction_front_blocked =
      finite_or(front_min, std::numeric_limits<double>::infinity()) <
      maze_graph_junction_front_min_ ||
      (finite_or(front_p20, 0.0) < maze_graph_junction_front_p20_ &&
      finite_or(front_p50, 0.0) < maze_graph_junction_front_p50_);
    graph_junction_front_guard_active_ = junction_front_blocked;
    const bool side_opening = openings.open[0] || openings.open[2];
    const bool junction_detected =
      !departing_opening && (side_opening || openings.open_count >= 2);

    if (graph_explore_state_ == "none") {
      set_graph_state("drive_center", now, "graph_start");
    }

    if (graph_explore_state_ == "drive_center") {
      if (front_obstacle) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        set_graph_state("scan_openings", now, "front_obstacle");
      } else if (junction_detected) {
        graph_junction_entry_pose_ = pose;
        graph_junction_advance_distance_ = 0.0;
        set_graph_state("approach_junction_center", now, "side_opening_detected");
      } else {
        maze_center_error_ = combined_maze_center_error(left_side, right_side, front_left, front_right);
        const double smoothed = smooth_maze_center_error(maze_center_error_);
        cmd.linear.x = maze_forward_speed_;
        cmd.angular.z = clamp(
          -maze_center_kp_ * smoothed - maze_front_center_kp_ * maze_front_center_error_,
          -maze_max_angular_,
          maze_max_angular_);
        maze_selected_turn_side_ = "front";
        topology_selected_opening_ = "front";
      }
    }

    if (graph_explore_state_ == "approach_junction_center") {
      graph_junction_advance_distance_ = std::hypot(
        pose.x - graph_junction_entry_pose_.x,
        pose.y - graph_junction_entry_pose_.y);
      const bool center_reached =
        graph_junction_advance_distance_ >= maze_graph_junction_center_advance_;
      if (!junction_front_blocked && !center_reached) {
        maze_center_error_ = combined_maze_center_error(
          left_side, right_side, front_left, front_right);
        const double smoothed = smooth_maze_center_error(maze_center_error_);
        cmd.linear.x = std::min(
          maze_graph_junction_approach_speed_, maze_forward_speed_);
        cmd.angular.z = clamp(
          -maze_center_kp_ * smoothed - maze_front_center_kp_ * maze_front_center_error_,
          -maze_max_angular_,
          maze_max_angular_);
        maze_selected_turn_side_ = "junction_center";
        topology_selected_opening_ = "junction_center";
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        set_graph_state(
          "scan_openings",
          now,
          junction_front_blocked ? "junction_front_guard" : "junction_center_reached");
      }
    }

    if (graph_explore_state_ == "scan_openings") {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      if (graph_state_age(now) >= maze_graph_scan_duration_) {
        auto& node = touch_topology_node(pose);
        current_topology_node_id_ = node.id;
        const int opening = choose_graph_opening(node, openings);
        if (opening >= 0) {
          const double direction = opening_direction_from_index(opening);
          set_active_topology_opening(node, opening);
          start_graph_turn(pose, now, opening, direction == 0.0 ? graph_turn_direction_ : direction);
        } else if (!maze_backtrack_stack_.empty()) {
          maze_backtrack_stack_.pop_back();
          const double direction = graph_left_score_ >= graph_right_score_ ? 1.0 : -1.0;
          start_graph_turn(pose, now, 3, direction);
        } else if (!graph_has_unvisited_openings() && maze_topology_nodes_.size() > 1) {
          set_graph_state("complete", now, "all_openings_visited");
        } else {
          const double direction = graph_left_score_ >= graph_right_score_ ? 1.0 : -1.0;
          start_graph_turn(pose, now, 3, direction);
          set_graph_state("turn_to_opening", now, "no_opening_turnaround_search");
        }
      }
    }

    if (graph_explore_state_ == "pre_turn_outward_shift") {
      const double dx = pose.x - graph_pre_turn_shift_start_pose_.x;
      const double dy = pose.y - graph_pre_turn_shift_start_pose_.y;
      graph_pre_turn_shift_distance_ = std::abs(
        -std::sin(graph_pre_turn_shift_start_pose_.yaw) * dx +
        std::cos(graph_pre_turn_shift_start_pose_.yaw) * dy);
      graph_pre_turn_shift_outer_clearance_ = graph_turn_direction_ >= 0.0 ?
        std::min(finite_or(right_side, 0.0), finite_or(front_right, 0.0)) :
        std::min(finite_or(left_side, 0.0), finite_or(front_left, 0.0));
      const bool outer_clear =
        graph_pre_turn_shift_outer_clearance_ >= maze_graph_pre_turn_outer_clearance_;
      const bool shift_complete =
        graph_pre_turn_shift_distance_ >= maze_graph_pre_turn_shift_distance_;
      const bool shift_timed_out =
        graph_state_age(now) >= maze_graph_pre_turn_shift_timeout_;
      if (outer_clear && !shift_complete && !shift_timed_out) {
        cmd.linear.x = 0.0;
        cmd.linear.y = -graph_turn_direction_ * maze_graph_pre_turn_shift_speed_;
        cmd.angular.z = graph_turn_direction_ * maze_graph_pre_turn_shift_angular_;
        maze_selected_turn_side_ = direction_name(graph_turn_direction_);
      } else {
        graph_pre_turn_shift_skipped_ = !outer_clear;
        set_graph_state(
          "turn_to_opening",
          now,
          !outer_clear ? "pre_turn_outer_blocked" :
          (shift_complete ? "pre_turn_shift_complete" : "pre_turn_shift_timeout"));
      }
    }

    if (graph_explore_state_ == "turn_to_opening") {
      const double yaw_delta = std::abs(normalize_angle(pose.yaw - graph_turn_start_yaw_));
      if (yaw_delta + maze_graph_turn_yaw_tolerance_ < graph_turn_target_yaw_ &&
          graph_state_age(now) < maze_graph_turn_timeout_)
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = graph_turn_direction_ * maze_turn_speed_;
        maze_selected_turn_side_ = direction_name(graph_turn_direction_);
      } else {
        start_graph_post_turn(pose, now, "graph_turn_complete");
      }
    }

    if (graph_explore_state_ == "post_turn_straight") {
      graph_post_turn_advance_distance_ = std::hypot(
        pose.x - graph_post_turn_start_pose_.x,
        pose.y - graph_post_turn_start_pose_.y);
      const bool post_turn_front_blocked =
        finite_or(front_min, std::numeric_limits<double>::infinity()) < front_emergency_distance_ ||
        (finite_or(front_p20, 0.0) < maze_graph_post_turn_front_p20_ &&
        finite_or(front_p50, 0.0) < maze_graph_post_turn_front_p50_);
      const bool advance_complete =
        graph_post_turn_advance_distance_ >= maze_graph_post_turn_advance_;
      const bool advance_timed_out =
        graph_state_age(now) >= maze_graph_post_turn_timeout_;
      if (post_turn_front_blocked) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        set_graph_state("scan_openings", now, "post_turn_front_obstacle");
      } else if (!advance_complete && !advance_timed_out) {
        cmd.linear.x = std::min(maze_graph_post_turn_speed_, maze_forward_speed_);
        cmd.angular.z = 0.0;
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        set_graph_state(
          "drive_center",
          now,
          advance_complete ? "post_turn_distance_complete" : "post_turn_timeout");
      }
    }

    if (graph_explore_state_ == "complete" || graph_explore_state_ == "blocked") {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    if (cmd.linear.x < 0.0) {
      cmd.linear.x = 0.0;
      maze_reverse_blocked_by_guard_ = true;
    } else {
      maze_reverse_blocked_by_guard_ = false;
    }
    raw_angular_z_ = cmd.angular.z;
    cmd.angular.z = limit_maze_angular_delta(cmd.angular.z);
    limited_angular_z_ = cmd.angular.z;
    last_angular_command_ = cmd.angular.z;
    record_angular_direction(cmd.angular.z, now);
    last_cmd_reason_ = "graph_dfs_explore";
    cmd_vel_pub_->publish(cmd);
    set_status("graph_dfs_explore", graph_explore_state_);
    return true;
  }

  void switch_deadend_turn_search_direction(const Pose2D& pose, const rclcpp::Time& now, const std::string& reason)
  {
    deadend_turn_direction_ = -deadend_turn_direction_;
    last_deadend_turn_direction_ = deadend_turn_direction_;
    ++deadend_turn_switch_count_;
    deadend_turn_start_yaw_ = pose.yaw;
    deadend_turn_yaw_delta_ = 0.0;
    deadend_front_drive_clear_confirm_frames_ = 0;
    deadend_turn_phase_start_time_ = now;
    deadend_turn_exit_reason_ = reason;
    maze_last_angular_command_ = 0.0;
  }

  void start_deadend_turnaround(const Pose2D& pose, const rclcpp::Time& now, const std::string& reason)
  {
    deadend_turn_active_ = true;
    deadend_turn_direction_ = choose_deadend_turn_direction();
    last_deadend_turn_direction_ = deadend_turn_direction_;
    deadend_turn_start_yaw_ = pose.yaw;
    deadend_reverse_start_pose_ = pose;
    deadend_reverse_total_start_pose_ = pose;
    deadend_turn_yaw_delta_ = 0.0;
    deadend_reverse_distance_ = 0.0;
    deadend_reverse_chunk_progress_ = 0.0;
    deadend_front_drive_clear_ = false;
    deadend_front_drive_clear_confirm_frames_ = 0;
    deadend_turn_switch_count_ = 0;
    deadend_turn_exit_reason_ = "none";
    deadend_turn_reason_ = reason;
    set_deadend_turn_phase("deadend_check_turn_space", pose, now);
  }

  void publish_deadend_turnaround_command(
    geometry_msgs::msg::Twist& cmd,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double front_min,
    double front_p20,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right,
    double front_p50,
    double rear)
  {
    deadend_turn_space_available(
      front_p20, left_side, right_side, left_mid, right_mid, front_left, front_right, rear);
    const auto phase_age = [&]() {
      return deadend_turn_phase_start_time_.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now - deadend_turn_phase_start_time_).seconds());
    };
    deadend_turn_yaw_delta_ = std::abs(normalize_angle(pose.yaw - deadend_turn_start_yaw_));
    deadend_reverse_chunk_progress_ = std::hypot(
      pose.x - deadend_reverse_start_pose_.x,
      pose.y - deadend_reverse_start_pose_.y);
    deadend_reverse_distance_ = std::hypot(
      pose.x - deadend_reverse_total_start_pose_.x,
      pose.y - deadend_reverse_total_start_pose_.y);
    const double rear_clearance = finite_or(rear, std::numeric_limits<double>::infinity());
    const bool reverse_total_exhausted =
      deadend_reverse_distance_ >= maze_deadend_reverse_total_max_distance_;
    reverse_rear_guard_blocks(rear);

    if (front_min < hard_stop_distance_ && deadend_turn_phase_ != "deadend_no_safe_turn_spot") {
      set_deadend_turn_phase("deadend_reverse_to_turn_spot", pose, now);
    }

    if (deadend_turn_phase_ == "deadend_check_turn_space") {
      if (deadend_turn_space_ok_) {
        set_deadend_turn_phase("deadend_turn_in_place", pose, now);
      } else {
        set_deadend_turn_phase("deadend_reverse_to_turn_spot", pose, now);
      }
    }

    if (deadend_turn_phase_ == "deadend_reverse_to_turn_spot") {
      const bool rear_guard_blocked = reverse_rear_guard_blocks(rear);
      if (deadend_turn_space_ok_) {
        set_deadend_turn_phase("deadend_turn_in_place", pose, now);
      } else if (rear_guard_blocked || rear_clearance < maze_deadend_in_place_min_rear_p20_) {
        set_deadend_turn_phase("deadend_no_safe_turn_spot", pose, now);
        deadend_turn_reason_ = rear_guard_blocked ?
          "deadend_reverse_rear_guard_stop" : "deadend_no_rear_clearance_for_turn_spot";
      } else if (reverse_total_exhausted) {
        set_deadend_turn_phase("deadend_no_safe_turn_spot", pose, now);
        deadend_turn_reason_ = "deadend_reverse_total_limit_reached";
      } else if (front_min < hard_stop_distance_ ||
                 (deadend_reverse_chunk_progress_ < maze_deadend_reverse_chunk_distance_ &&
                  phase_age() < maze_deadend_reverse_to_turn_spot_timeout_))
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        deadend_turn_reason_ = "deadend_reverse_to_find_turn_space";
      } else if (phase_age() >= maze_deadend_reverse_to_turn_spot_timeout_ &&
                 deadend_reverse_chunk_progress_ < 0.03)
      {
        set_deadend_turn_phase("deadend_no_safe_turn_spot", pose, now);
        deadend_turn_reason_ = "deadend_reverse_timeout_no_motion";
      } else {
        set_deadend_turn_phase("deadend_check_turn_space", pose, now);
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        deadend_turn_reason_ = "deadend_reverse_chunk_complete_recheck";
      }
    }

    if (deadend_turn_phase_ == "deadend_turn_in_place") {
      deadend_front_drive_clear_ = deadend_front_clear_for_drive(
        front_min, front_p20, front_p50, front_left, front_right);
      bump_maze_confirm_counter(deadend_front_drive_clear_confirm_frames_, deadend_front_drive_clear_);
      double open_side_direction = deadend_turn_direction_;
      double open_side_score = 0.0;
      const bool open_side_available = choose_open_side_turn_direction(
        deadend_turn_direction_, front_min, front_p20, front_p50,
        left_side, right_side, front_left, front_right, open_side_direction, open_side_score);
      if (open_side_available &&
          std::abs(open_side_direction - deadend_turn_direction_) > 0.1 &&
          phase_age() >= maze_open_side_reselect_timeout_)
      {
        deadend_turn_direction_ = open_side_direction;
        last_deadend_turn_direction_ = deadend_turn_direction_;
        deadend_turn_start_yaw_ = pose.yaw;
        deadend_turn_yaw_delta_ = 0.0;
        deadend_turn_phase_start_time_ = now;
        deadend_turn_reason_ = "deadend_open_side_reselected";
      }
      const bool front_drive_clear_confirmed =
        deadend_front_drive_clear_confirm_frames_ >= deadend_front_drive_clear_confirm_frames_required_;
      const bool direction_timed_out = phase_age() >= maze_deadend_turn_search_direction_timeout_;
      const bool direction_yaw_exhausted = deadend_turn_yaw_delta_ >= maze_deadend_turn_search_max_yaw_;
      const bool fallback_limit_reached =
        deadend_turn_yaw_delta_ >= std::max(0.1, maze_deadend_turn_yaw_ - maze_deadend_turn_yaw_tolerance_) ||
        phase_age() >= maze_deadend_turn_timeout_;
      const bool should_switch_direction =
        (direction_timed_out || direction_yaw_exhausted || fallback_limit_reached) &&
        deadend_turn_switch_count_ < maze_deadend_turn_search_max_switches_;
      if (front_drive_clear_confirmed) {
        set_deadend_turn_phase("deadend_post_turn_straight", pose, now);
        deadend_turn_exit_reason_ = "deadend_front_clear_for_drive";
        deadend_turn_reason_ = "deadend_front_clear_for_drive";
      } else if (front_min < hard_stop_distance_ ||
                 (!deadend_turn_space_ok_ && front_min < front_emergency_distance_))
      {
        set_deadend_turn_phase("deadend_reverse_to_turn_spot", pose, now);
        deadend_turn_exit_reason_ = "deadend_turn_lost_front_clearance";
        deadend_turn_reason_ = "deadend_turn_lost_front_clearance";
      } else if (should_switch_direction) {
        switch_deadend_turn_search_direction(
          pose, now, direction_timed_out ? "deadend_turn_direction_timeout" : "deadend_turn_direction_yaw_limit");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        deadend_turn_reason_ = "deadend_turn_search_switch_direction";
      } else if (fallback_limit_reached) {
        set_deadend_turn_phase("deadend_no_safe_turn_spot", pose, now);
        deadend_turn_exit_reason_ = "deadend_turn_search_failed";
        deadend_turn_reason_ = "deadend_turn_search_failed";
      } else {
        cmd.linear.x = 0.0;
        cmd.angular.z = deadend_turn_direction_ * maze_deadend_turn_speed_;
        deadend_turn_reason_ = "deadend_turn_search_front_clear";
      }
    }

    if (deadend_turn_phase_ == "deadend_post_turn_straight") {
      if (phase_age() < maze_post_turn_straight_duration_ && front_min >= front_emergency_distance_) {
        cmd.linear.x = std::max(maze_turn_forward_speed_, maze_forward_speed_ * 0.75);
        cmd.angular.z = 0.0;
        deadend_turn_reason_ = "deadend_post_turn_straight";
      } else {
        deadend_turn_active_ = false;
        deadend_turn_phase_ = "none";
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        deadend_turn_reason_ = "deadend_turn_complete";
      }
    }

    if (deadend_turn_phase_ == "deadend_no_safe_turn_spot") {
      if (deadend_turn_space_ok_ && deadend_turn_exit_reason_ != "deadend_turn_search_failed") {
        set_deadend_turn_phase("deadend_turn_in_place", pose, now);
      } else {
        deadend_turn_reason_ = "deadend_three_point_fallback";
        publish_maze_three_point_turn_command(
          cmd, pose, now, deadend_turn_direction_, front_min, rear, true);
        return;
      }
    }

    maze_action_ = deadend_turn_phase_;
    maze_corner_phase_ = deadend_turn_phase_;
    maze_turn_phase_ = deadend_turn_phase_;
    maze_preclear_action_ = deadend_turn_phase_;
    maze_selected_turn_side_ = direction_name(deadend_turn_direction_);
    maze_speed_profile_ = deadend_turn_phase_;
    maze_corner_reason_ = deadend_turn_reason_;
    maze_last_angular_command_ = cmd.angular.z;
  }

  void set_maze_corner_turn_state(
    const std::string& state,
    const Pose2D& pose,
    const rclcpp::Time& now,
    const std::string& reason)
  {
    if (maze_corner_turn_state_ == state) {
      return;
    }
    maze_corner_turn_state_ = state;
    maze_corner_state_start_time_ = now;
    maze_corner_latch_reason_ = reason;
    set_maze_turn_phase(state, now);
    if (state == "corner_reverse_clearance" ||
        state == "corner_retry_backout" ||
        state == "corner_micro_reverse")
    {
      maze_corner_reverse_start_pose_ = pose;
      maze_corner_reverse_distance_ = 0.0;
    } else if (state == "corner_pivot_align" || state == "corner_pivot_assist") {
      maze_corner_pivot_start_yaw_ = pose.yaw;
      maze_corner_pivot_yaw_delta_ = 0.0;
    }
  }

  void publish_maze_three_point_turn_command(
    geometry_msgs::msg::Twist& cmd,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double turn_direction,
    double front_min,
    double rear,
    bool from_deadend = false)
  {
    if (!maze_corner_turn_latched_) {
      maze_corner_turn_latched_ = true;
      maze_corner_latched_direction_ = turn_direction >= 0.0 ? 1.0 : -1.0;
      maze_corner_cycle_count_ = 0;
      maze_corner_selected_side_ = direction_name(maze_corner_latched_direction_);
      maze_corner_latch_reason_ = from_deadend ? "deadend_three_point_latched" : "corner_three_point_latched";
      set_maze_corner_turn_state("corner_reverse_clearance", pose, now, maze_corner_latch_reason_);
    }

    maze_turn_direction_ = maze_corner_latched_direction_ >= 0.0 ? 1.0 : -1.0;
    maze_corner_selected_side_ = direction_name(maze_turn_direction_);
    maze_selected_turn_side_ = maze_corner_selected_side_;
    maze_inner_side_ = maze_corner_selected_side_;
    maze_turn_blocked_by_inner_clearance_ = false;
    maze_blocked_by_robot_model_ = false;

    const auto state_age = [&]() {
      return maze_corner_state_start_time_.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now - maze_corner_state_start_time_).seconds());
    };
    maze_corner_reverse_distance_ = std::hypot(
      pose.x - maze_corner_reverse_start_pose_.x,
      pose.y - maze_corner_reverse_start_pose_.y);
    maze_corner_pivot_yaw_delta_ = std::abs(normalize_angle(pose.yaw - maze_corner_pivot_start_yaw_));

    const double rear_clearance = finite_or(rear, std::numeric_limits<double>::infinity());
    const bool front_hard = front_min < hard_stop_distance_;
    const bool rear_guard_blocked = reverse_rear_guard_blocks(rear);
    const bool rear_hard = rear_guard_blocked || rear_clearance < maze_deadend_in_place_min_rear_p20_;
    if (maze_corner_turn_state_ == "blocked_no_motion_space" && !(front_hard && rear_hard)) {
      set_maze_corner_turn_state(
        front_hard ? "corner_reverse_clearance" : "corner_pivot_align",
        pose,
        now,
        front_hard ? "blocked_released_reverse" : "blocked_released_pivot");
    }

    if (front_hard && rear_hard) {
      set_maze_corner_turn_state("blocked_no_motion_space", pose, now, "front_and_rear_hard");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_corner_reason_ = "blocked_no_motion_space";
      maze_preclear_action_ = "blocked_no_motion_space";
      maze_speed_profile_ = "maze_corner_blocked";
    } else if (maze_corner_turn_state_ == "corner_reverse_clearance") {
      if (!rear_hard &&
          (front_hard || maze_corner_reverse_distance_ < maze_corner_reverse_target_distance_) &&
          state_age() < 1.6)
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_corner_reason_ = "corner_reverse_clearance";
        maze_preclear_action_ = "corner_reverse_clearance";
        maze_speed_profile_ = "maze_corner_reverse";
      } else {
        set_maze_corner_turn_state("corner_pivot_align", pose, now, "corner_reverse_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_pivot_align") {
      const double pivot_timeout = maze_corner_pivot_speed_ > 0.0 ?
        maze_corner_pivot_yaw_ / maze_corner_pivot_speed_ + 0.6 : 1.2;
      if (maze_corner_pivot_yaw_delta_ < maze_corner_pivot_yaw_ && state_age() < pivot_timeout) {
        cmd.linear.x = 0.0;
        cmd.angular.z = maze_turn_direction_ * maze_corner_pivot_speed_;
        maze_corner_reason_ = "corner_pivot_align";
        maze_preclear_action_ = "corner_pivot_align";
        maze_speed_profile_ = "maze_corner_pivot";
      } else {
        set_maze_corner_turn_state("corner_swing_forward", pose, now, "corner_pivot_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_swing_forward") {
      if (front_hard) {
        ++maze_corner_cycle_count_;
        if (maze_corner_cycle_count_ >= maze_corner_cycle_limit_) {
          set_maze_corner_turn_state("corner_retry_backout", pose, now, "corner_cycle_limit_backout");
        } else {
          set_maze_corner_turn_state("corner_reverse_clearance", pose, now, "corner_front_hard_retry");
        }
      } else if (state_age() < maze_corner_swing_duration_) {
        cmd.linear.x = maze_corner_swing_forward_speed_;
        cmd.angular.z = maze_turn_direction_ * maze_corner_swing_turn_speed_;
        maze_corner_reason_ = "corner_swing_forward";
        maze_preclear_action_ = "corner_swing_forward";
        maze_speed_profile_ = "maze_corner_swing";
      } else {
        set_maze_corner_turn_state("corner_exit_straight", pose, now, "corner_swing_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_retry_backout") {
      if (!rear_hard && maze_corner_reverse_distance_ < maze_corner_retry_backout_distance_ && state_age() < 2.2) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_corner_reason_ = "corner_retry_backout";
        maze_preclear_action_ = "corner_retry_backout";
        maze_speed_profile_ = "maze_corner_retry_backout";
      } else if (rear_hard && front_hard) {
        set_maze_corner_turn_state("blocked_no_motion_space", pose, now, "retry_backout_front_and_rear_hard");
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_corner_reason_ = "blocked_no_motion_space";
        maze_preclear_action_ = "blocked_no_motion_space";
        maze_speed_profile_ = "maze_corner_blocked";
      } else {
        maze_corner_cycle_count_ = 0;
        set_maze_corner_turn_state("corner_pivot_align", pose, now, "corner_retry_backout_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_exit_straight") {
      if (front_hard) {
        ++maze_corner_cycle_count_;
        set_maze_corner_turn_state("corner_reverse_clearance", pose, now, "corner_exit_front_hard_retry");
      } else if (state_age() < maze_corner_exit_straight_duration_) {
        cmd.linear.x = std::max(maze_corner_swing_forward_speed_, maze_forward_speed_ * 0.75);
        cmd.angular.z = 0.0;
        maze_corner_reason_ = "corner_exit_straight";
        maze_preclear_action_ = "corner_exit_straight";
        maze_speed_profile_ = "maze_corner_exit";
      } else {
        set_maze_post_turn_straight(now, from_deadend ? "deadend_three_point_complete" : "corner_three_point_complete");
        cmd.linear.x = std::max(maze_corner_swing_forward_speed_, maze_forward_speed_ * 0.75);
        cmd.angular.z = 0.0;
        maze_action_ = "maze_post_turn_straight";
        maze_corner_phase_ = "maze_post_turn_straight";
        maze_preclear_action_ = "post_turn_straight";
        maze_speed_profile_ = "maze_post_turn_straight";
        maze_last_angular_command_ = 0.0;
        return;
      }
    }

    maze_action_ = maze_corner_turn_state_;
    maze_corner_phase_ = maze_corner_turn_state_;
    maze_turn_phase_ = maze_corner_turn_state_;
    maze_last_angular_command_ = cmd.angular.z;
  }

  void publish_maze_flow_arc_turn_command(
    geometry_msgs::msg::Twist& cmd,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double turn_direction,
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double front_left,
    double front_right,
    double rear,
    bool from_deadend = false)
  {
    if (!maze_corner_turn_latched_) {
      maze_corner_turn_latched_ = true;
      maze_corner_latched_direction_ = turn_direction >= 0.0 ? 1.0 : -1.0;
      maze_corner_cycle_count_ = 0;
      maze_corner_selected_side_ = direction_name(maze_corner_latched_direction_);
      maze_corner_latch_reason_ = from_deadend ? "deadend_flow_arc_latched" : "corner_flow_arc_latched";
      set_maze_corner_turn_state("corner_late_approach", pose, now, maze_corner_latch_reason_);
    }

    maze_turn_direction_ = maze_corner_latched_direction_ >= 0.0 ? 1.0 : -1.0;
    maze_corner_selected_side_ = direction_name(maze_turn_direction_);
    maze_selected_turn_side_ = maze_corner_selected_side_;
    maze_inner_side_ = maze_corner_selected_side_;
    maze_inner_clearance_ = maze_inner_clearance_for_direction(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    maze_inner_front_corner_clearance_ = estimate_inner_front_corner_clearance(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    maze_swept_clearance_ = estimate_swept_clearance(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    maze_turn_blocked_by_inner_clearance_ = false;
    maze_blocked_by_robot_model_ = false;
    maze_reverse_suppressed_by_open_side_ = false;

    const auto state_age = [&]() {
      return maze_corner_state_start_time_.nanoseconds() == 0 ?
        0.0 : std::max(0.0, (now - maze_corner_state_start_time_).seconds());
    };
    maze_corner_reverse_distance_ = std::hypot(
      pose.x - maze_corner_reverse_start_pose_.x,
      pose.y - maze_corner_reverse_start_pose_.y);
    maze_corner_pivot_yaw_delta_ = std::abs(normalize_angle(pose.yaw - maze_corner_pivot_start_yaw_));

    const double rear_clearance = finite_or(rear, std::numeric_limits<double>::infinity());
    const bool front_hard = front_min < hard_stop_distance_;
    const bool front_contact = maze_reverse_required(front_min, front_p20);
    const bool rear_guard_blocked = reverse_rear_guard_blocks(rear);
    const bool rear_hard = rear_guard_blocked || rear_clearance < maze_deadend_in_place_min_rear_p20_;
    double open_side_direction = maze_turn_direction_;
    double open_side_score = 0.0;
    const bool open_side_available = choose_open_side_turn_direction(
      maze_turn_direction_, front_min, front_p20, front_p50,
      left_side, right_side, front_left, front_right, open_side_direction, open_side_score);
    bump_maze_confirm_counter(maze_open_side_confirm_frames_, open_side_available);
    const bool open_side_confirmed =
      open_side_available && maze_open_side_confirm_frames_ >= maze_open_side_confirm_frames_required_;

    auto prefer_open_side_turn = [&](const std::string& reason) {
      maze_corner_latched_direction_ = open_side_direction >= 0.0 ? 1.0 : -1.0;
      maze_turn_direction_ = maze_corner_latched_direction_;
      maze_corner_selected_side_ = direction_name(maze_turn_direction_);
      maze_selected_turn_side_ = maze_corner_selected_side_;
      maze_inner_side_ = maze_corner_selected_side_;
      maze_reverse_suppressed_by_open_side_ = true;
      maze_turn_exit_reason_ = reason;
      set_maze_corner_turn_state("corner_pivot_assist", pose, now, reason);
    };

    if (open_side_confirmed &&
        std::abs(open_side_direction - maze_turn_direction_) > 0.1 &&
        state_age() >= maze_open_side_reselect_timeout_ &&
        maze_corner_turn_state_ != "corner_arc_commit" &&
        maze_corner_turn_state_ != "corner_exit_straight")
    {
      prefer_open_side_turn("open_side_reselect");
    }

    if (front_hard && rear_hard) {
      set_maze_corner_turn_state("blocked_no_motion_space", pose, now, "flow_arc_front_and_rear_hard");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_corner_reason_ = "blocked_no_motion_space";
      maze_preclear_action_ = "blocked_no_motion_space";
      maze_speed_profile_ = "maze_corner_blocked";
    } else if (maze_corner_turn_state_ == "blocked_no_motion_space") {
      set_maze_corner_turn_state(
        front_contact ? "corner_micro_reverse" : "corner_pivot_assist",
        pose,
        now,
        front_contact ? "blocked_released_micro_reverse" : "blocked_released_pivot_assist");
    }

    if (maze_corner_turn_state_ == "corner_late_approach") {
      const bool turn_start_close =
        front_p20 <= maze_turn_start_front_p20_ ||
        front_p50 <= maze_turn_start_front_p50_;
      const double approach_hold = std::max(0.35, maze_junction_dwell_duration_);
      if (front_contact) {
        if (open_side_confirmed && !front_hard) {
          prefer_open_side_turn("late_approach_open_side_turn");
        } else {
          set_maze_corner_turn_state("corner_micro_reverse", pose, now, "late_approach_contact_risk");
        }
      } else if (!turn_start_close && state_age() < approach_hold) {
        cmd.linear.x = maze_corner_approach_speed_;
        cmd.angular.z = maze_turn_direction_ * std::min(maze_corner_arc_angular_ * 0.25, maze_max_angular_);
        maze_corner_reason_ = "corner_late_approach";
        maze_preclear_action_ = "none";
        maze_speed_profile_ = "maze_corner_late_approach";
      } else {
        set_maze_corner_turn_state("corner_arc_commit", pose, now, "late_approach_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_arc_commit") {
      if (front_contact) {
        if (open_side_confirmed && !front_hard) {
          prefer_open_side_turn("arc_contact_open_side_turn");
        } else {
          ++maze_corner_cycle_count_;
          if (maze_corner_cycle_count_ >= maze_corner_cycle_limit_) {
            reset_maze_corner_turn_latch("flow_arc_cycle_limit_three_point");
            publish_maze_three_point_turn_command(
              cmd, pose, now, maze_turn_direction_, front_min, rear, from_deadend);
            maze_corner_reason_ = "flow_arc_three_point_fallback";
            maze_speed_profile_ = "maze_corner_three_point_fallback";
            return;
          }
          set_maze_corner_turn_state("corner_micro_reverse", pose, now, "arc_contact_micro_reverse");
        }
      } else if (state_age() < maze_corner_swing_duration_) {
        cmd.linear.x = maze_corner_arc_speed_;
        cmd.angular.z = maze_turn_direction_ * std::min(maze_corner_arc_angular_, maze_max_angular_);
        maze_corner_reason_ = "corner_arc_commit";
        maze_preclear_action_ = "none";
        maze_speed_profile_ = "maze_corner_flow_arc";
      } else {
        set_maze_corner_turn_state("corner_exit_straight", pose, now, "arc_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_micro_reverse") {
      if (open_side_confirmed && !front_hard) {
        prefer_open_side_turn("micro_reverse_suppressed_by_open_side");
      } else if (!rear_hard &&
          maze_corner_reverse_distance_ < maze_corner_micro_reverse_distance_ &&
          state_age() < 1.0)
      {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_last_angular_command_ = 0.0;
        maze_corner_reason_ = "corner_micro_reverse";
        maze_preclear_action_ = "corner_micro_reverse";
        maze_speed_profile_ = "maze_corner_micro_reverse";
      } else {
        set_maze_corner_turn_state("corner_pivot_assist", pose, now, "micro_reverse_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_pivot_assist") {
      const double pivot_timeout = maze_corner_pivot_speed_ > 0.0 ?
        maze_corner_pivot_assist_yaw_ / maze_corner_pivot_speed_ + 0.4 : 1.0;
      if (maze_corner_pivot_yaw_delta_ < maze_corner_pivot_assist_yaw_ && state_age() < pivot_timeout) {
        cmd.linear.x = 0.0;
        cmd.angular.z = maze_turn_direction_ * maze_corner_pivot_speed_;
        maze_corner_reason_ = "corner_pivot_assist";
        maze_preclear_action_ = "corner_pivot_assist";
        maze_speed_profile_ = "maze_corner_pivot_assist";
      } else {
        set_maze_corner_turn_state("corner_arc_commit", pose, now, "pivot_assist_complete");
      }
    }

    if (maze_corner_turn_state_ == "corner_exit_straight") {
      if (front_contact && maze_corner_cycle_count_ < maze_corner_cycle_limit_) {
        if (open_side_confirmed && !front_hard) {
          prefer_open_side_turn("exit_contact_open_side_turn");
        } else {
          ++maze_corner_cycle_count_;
          set_maze_corner_turn_state("corner_micro_reverse", pose, now, "exit_contact_micro_reverse");
        }
      } else if (state_age() < maze_corner_exit_straight_duration_) {
        cmd.linear.x = std::max(maze_corner_arc_speed_, maze_forward_speed_ * 0.75);
        cmd.angular.z = 0.0;
        maze_last_angular_command_ = 0.0;
        maze_corner_reason_ = "corner_exit_straight";
        maze_preclear_action_ = "post_turn_straight";
        maze_speed_profile_ = "maze_corner_exit";
      } else {
        set_maze_post_turn_straight(now, from_deadend ? "deadend_flow_arc_complete" : "corner_flow_arc_complete");
        cmd.linear.x = std::max(maze_corner_arc_speed_, maze_forward_speed_ * 0.75);
        cmd.angular.z = 0.0;
        maze_action_ = "maze_post_turn_straight";
        maze_corner_phase_ = "maze_post_turn_straight";
        maze_preclear_action_ = "post_turn_straight";
        maze_speed_profile_ = "maze_post_turn_straight";
        maze_last_angular_command_ = 0.0;
        return;
      }
    }

    maze_action_ = maze_corner_turn_state_;
    maze_corner_phase_ = maze_corner_turn_state_;
    maze_turn_phase_ = maze_corner_turn_state_;
    maze_last_angular_command_ = cmd.angular.z;
  }

  void publish_maze_turn_command(
    geometry_msgs::msg::Twist& cmd,
    const Pose2D& pose,
    const rclcpp::Time& now,
    double turn_direction,
    double front_min,
    double front_p20,
    double front_p50,
    double left_side,
    double right_side,
    double left_mid,
    double right_mid,
    double front_left,
    double front_right,
    double rear)
  {
    if (maze_corner_turn_mode_ == "flow_arc") {
      publish_maze_flow_arc_turn_command(
        cmd, pose, now, turn_direction, front_min, front_p20, front_p50,
        left_side, right_side, front_left, front_right, rear, false);
      return;
    }
    if (maze_corner_turn_mode_ == "three_point") {
      publish_maze_three_point_turn_command(cmd, pose, now, turn_direction, front_min, rear, false);
      return;
    }

    if (!maze_corner_turn_latched_) {
      maze_corner_turn_latched_ = true;
      maze_corner_latched_direction_ = turn_direction >= 0.0 ? 1.0 : -1.0;
      maze_corner_latch_reason_ = "corner_turn_latched";
      maze_corner_release_confirm_frames_ = 0;
      maze_corner_abort_confirm_frames_ = 0;
      maze_corner_preclear_attempts_ = 0;
      maze_corner_preclear_total_start_pose_ = pose;
      maze_corner_preclear_start_pose_ = pose;
      maze_corner_preclear_total_distance_ = 0.0;
    }
    maze_turn_direction_ = maze_corner_latched_direction_ >= 0.0 ? 1.0 : -1.0;
    maze_inner_side_ = direction_name(maze_turn_direction_);
    maze_inner_clearance_ = maze_inner_clearance_for_direction(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    maze_inner_front_corner_clearance_ = estimate_inner_front_corner_clearance(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    maze_swept_clearance_ = estimate_swept_clearance(
      maze_turn_direction_, left_side, right_side, front_left, front_right);
    const bool release_clear = maze_inner_clear_for_turn(
      maze_turn_direction_, left_side, right_side, front_left, front_right, maze_inner_clearance_release_p20_);
    const bool min_clear = maze_inner_clear_for_turn(
      maze_turn_direction_, left_side, right_side, front_left, front_right, maze_inner_clearance_min_p20_);
    const bool swept_release_clear =
      !maze_use_robot_swept_volume_ || maze_swept_clearance_ >= maze_swept_clearance_release_;
    const bool swept_min_clear =
      !maze_use_robot_swept_volume_ || maze_swept_clearance_ >= maze_inner_corner_clearance_;
    const bool front_requires_reverse = front_min < front_emergency_distance_;
    const bool release_observed = !front_requires_reverse && release_clear && swept_release_clear;
    const bool abort_observed =
      maze_use_robot_swept_volume_ && maze_swept_clearance_ < maze_swept_clearance_abort_;
    bump_maze_confirm_counter(maze_corner_release_confirm_frames_, release_observed);
    bump_maze_confirm_counter(maze_corner_abort_confirm_frames_, abort_observed);
    const bool release_confirmed =
      maze_corner_release_confirm_frames_ >= maze_corner_release_confirm_frames_required_;
    const bool abort_confirmed =
      maze_corner_abort_confirm_frames_ >= maze_corner_release_confirm_frames_required_;
    maze_turn_blocked_by_inner_clearance_ = !min_clear || !swept_min_clear;
    maze_blocked_by_robot_model_ = !swept_release_clear;
    maze_selected_turn_side_ = direction_name(maze_turn_direction_);

    auto enter_corner_preclear = [&](const std::string& reason) {
      if (maze_turn_phase_ != "maze_corner_preclear") {
        ++maze_corner_preclear_attempts_;
        maze_corner_preclear_start_pose_ = pose;
      }
      set_maze_turn_phase("maze_corner_preclear", now);
      maze_corner_reason_ = reason;
    };

    if (front_min < hard_stop_distance_) {
      set_maze_turn_phase("maze_turn_escape_reverse", now);
      maze_corner_reason_ = "contact_reverse";
    } else if (maze_turn_phase_ == "maze_swept_turn") {
      const double swept_elapsed = maze_turn_phase_start_time_.nanoseconds() == 0 ?
        0.0 : (now - maze_turn_phase_start_time_).seconds();
      if (swept_elapsed >= maze_turn_max_continuous_duration_) {
        set_maze_post_turn_straight(now, "turn_duration_limit_post_straight");
      } else if (swept_elapsed >= maze_swept_turn_min_commit_duration_ && abort_confirmed) {
        enter_corner_preclear("swept_abort_clearance");
      } else {
        maze_corner_reason_ = swept_elapsed < maze_swept_turn_min_commit_duration_ ?
          "swept_turn_min_commit_hold" : "swept_turn_latched";
      }
    } else if (maze_turn_phase_ == "maze_turn_escape_reverse") {
      const double escape_elapsed = maze_turn_phase_start_time_.nanoseconds() == 0 ?
        0.0 : (now - maze_turn_phase_start_time_).seconds();
      if (escape_elapsed >= maze_turn_escape_reverse_duration_) {
        enter_corner_preclear("escape_reverse_complete");
      }
    } else if (release_confirmed) {
      set_maze_turn_phase("maze_swept_turn", now);
      maze_corner_reason_ = "swept_release_confirmed";
    } else {
      enter_corner_preclear(front_requires_reverse ? "front_emergency_reverse" :
        (!swept_release_clear ? "prepare_robot_swept_clearance" : "prepare_inner_release_clearance"));
    }

    const double phase_elapsed = maze_turn_phase_start_time_.nanoseconds() == 0 ?
      0.0 : (now - maze_turn_phase_start_time_).seconds();
    maze_corner_preclear_total_distance_ = std::hypot(
      pose.x - maze_corner_preclear_total_start_pose_.x,
      pose.y - maze_corner_preclear_total_start_pose_.y);
    if (maze_turn_phase_ == "maze_corner_preclear" && !release_confirmed)
    {
      const bool preclear_distance_exhausted =
        maze_corner_preclear_total_distance_ >= maze_corner_preclear_total_max_distance_;
      const bool preclear_attempts_exhausted =
        phase_elapsed >= maze_corner_preclear_duration_ &&
        maze_corner_preclear_attempts_ >= maze_corner_preclear_max_attempts_;
      if (preclear_distance_exhausted || preclear_attempts_exhausted) {
        if (!deadend_turn_active_) {
          start_deadend_turnaround(pose, now, "corner_preclear_exhausted");
        }
        publish_deadend_turnaround_command(
          cmd, pose, now, front_min, front_p20, left_side, right_side,
          left_mid, right_mid, front_left, front_right, front_p50, rear);
        maze_route_state_ = "deadend_turnaround";
        maze_route_reason_ = deadend_turn_reason_;
        return;
      }
      if (phase_elapsed >= maze_corner_preclear_duration_) {
        ++maze_corner_preclear_attempts_;
        maze_corner_preclear_start_pose_ = pose;
        maze_turn_phase_start_time_ = now;
        maze_corner_reason_ = "corner_preclear_retry";
      }
    }

    if (maze_turn_phase_ == "maze_turn_escape_reverse") {
      const bool rear_guard_blocked = reverse_rear_guard_blocks(rear);
      if (rear_guard_blocked) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_last_angular_command_ = 0.0;
        maze_preclear_action_ = "reverse_blocked_by_rear_guard";
        maze_corner_reason_ = "maze_reverse_rear_guard_stop";
        maze_speed_profile_ = "maze_reverse_blocked";
        maze_action_ = maze_turn_phase_;
        maze_corner_phase_ = maze_turn_phase_;
        return;
      }
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_last_angular_command_ = 0.0;
      maze_preclear_action_ = "reverse_to_open_inner_clearance";
      maze_speed_profile_ = "maze_reverse";
    } else if (maze_turn_phase_ == "maze_corner_preclear") {
      const bool rear_guard_blocked = reverse_rear_guard_blocks(rear);
      if (rear_guard_blocked) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_last_angular_command_ = 0.0;
        maze_preclear_action_ = "preclear_blocked_by_rear_guard";
        maze_corner_reason_ = "maze_reverse_rear_guard_stop";
        maze_speed_profile_ = "maze_reverse_blocked";
        maze_action_ = maze_turn_phase_;
        maze_corner_phase_ = maze_turn_phase_;
        return;
      }
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_last_angular_command_ = 0.0;
      maze_preclear_action_ = front_requires_reverse ?
        "reverse_front_emergency_before_swept_turn" : "reverse_before_swept_turn";
      maze_speed_profile_ = "maze_corner_preclear";
    } else if (maze_turn_phase_ == "maze_post_turn_straight") {
      cmd.linear.x = std::max(maze_turn_forward_speed_, maze_forward_speed_ * 0.75);
      cmd.angular.z = 0.0;
      maze_last_angular_command_ = 0.0;
      maze_preclear_action_ = "post_turn_straight";
      maze_speed_profile_ = "maze_post_turn_straight";
    } else {
      const double clearance_factor = clamp(
        (maze_swept_clearance_ - maze_inner_corner_clearance_) /
        std::max(0.01, maze_swept_clearance_release_ - maze_inner_corner_clearance_),
        0.55, 1.0);
      cmd.linear.x = maze_turn_forward_speed_;
      const double max_angular_for_radius =
        maze_min_turn_radius_ > 0.0 ? cmd.linear.x / maze_min_turn_radius_ : maze_max_angular_;
      cmd.angular.z = maze_turn_direction_ * std::min(
        {maze_turn_speed_ * clearance_factor, maze_max_angular_, max_angular_for_radius});
      maze_preclear_action_ = "swept_turn_with_robot_clearance";
      maze_turn_blocked_by_inner_clearance_ = !min_clear || !swept_min_clear;
      maze_blocked_by_robot_model_ = !swept_release_clear;
      maze_speed_profile_ = "maze_swept_turn";
    }
    maze_action_ = maze_turn_phase_;
    maze_corner_phase_ = maze_turn_phase_;
  }

  double smooth_maze_center_error(double raw_error)
  {
    if (!maze_center_ema_initialized_) {
      maze_smoothed_center_error_ = raw_error;
      maze_center_ema_initialized_ = true;
    } else {
      maze_smoothed_center_error_ =
        maze_center_ema_alpha_ * raw_error + (1.0 - maze_center_ema_alpha_) * maze_smoothed_center_error_;
    }
    const double center_deadband = maze_topology_mode_ ?
      std::max(maze_center_deadband_, maze_corridor_center_deadband_) : maze_center_deadband_;
    if (std::abs(maze_smoothed_center_error_) < center_deadband) {
      return 0.0;
    }
    return maze_smoothed_center_error_;
  }

  double front_diagonal_center_error(double front_left, double front_right) const
  {
    const double left = finite_or(front_left, maze_exit_clear_p20_);
    const double right = finite_or(front_right, maze_exit_clear_p20_);
    const double error = right - left;
    return std::abs(error) >= maze_center_lookahead_min_delta_ ? error : 0.0;
  }

  double combined_maze_center_error(
    double left_side,
    double right_side,
    double front_left,
    double front_right)
  {
    maze_side_center_error_ =
      finite_or(right_side, maze_side_wall_max_p50_) -
      finite_or(left_side, maze_side_wall_max_p50_);
    maze_front_center_error_ = front_diagonal_center_error(front_left, front_right);
    return maze_side_error_weight_ * maze_side_center_error_ +
      maze_front_diagonal_error_weight_ * maze_front_center_error_;
  }

  double limit_maze_angular_delta(double desired)
  {
    if (maze_angular_max_delta_ <= 0.0) {
      maze_last_angular_command_ = desired;
      return desired;
    }
    const double limited = clamp(
      desired,
      maze_last_angular_command_ - maze_angular_max_delta_,
      maze_last_angular_command_ + maze_angular_max_delta_);
    maze_last_angular_command_ = limited;
    return limited;
  }

  bool publish_maze_follow_command(const Pose2D& pose, const rclcpp::Time& now)
  {
    if (!narrow_maze_follow_enabled_) {
      return false;
    }
    double front_min = std::numeric_limits<double>::infinity();
    double front_p20 = std::numeric_limits<double>::infinity();
    double front_p50 = std::numeric_limits<double>::infinity();
    double left_side = std::numeric_limits<double>::infinity();
    double right_side = std::numeric_limits<double>::infinity();
    double left_mid = std::numeric_limits<double>::infinity();
    double right_mid = std::numeric_limits<double>::infinity();
    double front_left = std::numeric_limits<double>::infinity();
    double front_right = std::numeric_limits<double>::infinity();
    double rear = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_min = front_min_;
      front_p20 = front_p20_;
      front_p50 = front_p50_;
      left_side = left_side_p20_;
      right_side = right_side_p20_;
      left_mid = left_p20_;
      right_mid = right_p20_;
      front_left = front_left_p20_;
      front_right = front_right_p20_;
      rear = rear_p20_;
    }

    if (!maze_follow_active_ && !should_enter_maze_follow(front_min, front_p20, front_p50)) {
      return false;
    }
    if (maze_follow_active_ && should_exit_maze_follow(front_min, front_p20, front_p50, now)) {
      maze_follow_active_ = false;
      maze_turn_lock_active_ = false;
      maze_turn_commit_active_ = false;
      maze_priority_active_ = false;
      maze_action_ = "none";
      return false;
    }
    if (!maze_follow_active_) {
      maze_follow_active_ = true;
      maze_priority_active_ = true;
      maze_follow_start_time_ = now;
      maze_turn_commit_active_ = false;
      maze_center_ema_initialized_ = false;
      maze_entry_reason_ = "side_walls_and_unstable_frontier";
      maze_exit_reason_ = "none";
      clear_plan();
      target_switch_reason_ = "maze_follow_active";
    }

    if (maze_route_mode_ == "graph_dfs") {
      return publish_graph_dfs_exploration_command(
        pose, now, front_min, front_p20, front_p50,
        left_side, right_side, left_mid, right_mid, front_left, front_right);
    }

    geometry_msgs::msg::Twist cmd;
    corridor_width_estimate_ = finite_or(left_side, 0.0) + finite_or(right_side, 0.0);
    maze_center_error_ = combined_maze_center_error(left_side, right_side, front_left, front_right);
    const double smoothed_center_error = smooth_maze_center_error(maze_center_error_);
    const auto route_observation = observe_maze_route(
      front_min, front_p20, front_p50, left_side, right_side, left_mid, right_mid, front_left, front_right);
    update_maze_route_confirmations(route_observation);
    auto route_decision = choose_maze_route_decision(route_observation, pose, now, front_min, front_p20);
    maze_route_state_ = route_decision.state;
    maze_route_reason_ = route_decision.reason;
    const bool deadend_candidate_confirmed =
      maze_deadend_candidate_confirm_frames_ >= maze_deadend_confirm_frames_;
    const bool maze_stalled =
      maze_only_mode_ &&
      maze_last_progress_time_.nanoseconds() != 0 &&
      (now - maze_last_progress_time_).seconds() > maze_progress_timeout_;
    const double maze_phase_age =
      maze_turn_phase_start_time_.nanoseconds() == 0 ? 0.0 :
        std::max(0.0, (now - maze_turn_phase_start_time_).seconds());
    const bool post_turn_straight_active =
      maze_turn_phase_ == "maze_post_turn_straight" &&
      maze_phase_age < maze_post_turn_straight_duration_ &&
      front_min >= hard_stop_distance_;
    const bool turn_commit_hold =
      maze_turn_commit_active_ &&
      maze_phase_age < maze_turn_max_continuous_duration_ &&
      !maze_turn_release_clear(front_min, front_p20, front_left, front_right);

    if (front_min < hard_stop_distance_ && !deadend_turn_active_ && !deadend_candidate_confirmed) {
      reset_maze_corner_turn_latch("contact_reverse");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      set_maze_turn_phase("maze_turn_escape_reverse", now);
      maze_inner_side_ = "front";
      maze_inner_clearance_ = front_min;
      maze_turn_blocked_by_inner_clearance_ = true;
      maze_preclear_action_ = "contact_reverse";
      maze_action_ = "maze_contact_reverse";
      maze_corner_reason_ = "contact_reverse";
      maze_speed_profile_ = "contact_reverse";
    } else if (post_turn_straight_active) {
      reset_maze_corner_turn_latch("post_turn_straight_hold");
      deadend_turn_active_ = false;
      maze_turn_lock_active_ = false;
      maze_turn_commit_active_ = false;
      maze_turn_blocked_by_inner_clearance_ = false;
      maze_blocked_by_robot_model_ = false;
      maze_inner_side_ = "none";
      maze_inner_clearance_ = std::numeric_limits<double>::infinity();
      maze_swept_clearance_ = std::numeric_limits<double>::infinity();
      maze_inner_front_corner_clearance_ = std::numeric_limits<double>::infinity();
      maze_preclear_action_ = "post_turn_straight";
      maze_corner_reason_ = "post_turn_straight_hold";
      maze_selected_turn_side_ = "none";
      maze_corner_phase_ = "maze_post_turn_straight";
      maze_speed_profile_ = "maze_post_turn_straight";
      cmd.linear.x = std::max(maze_turn_forward_speed_, maze_forward_speed_ * 0.75);
      cmd.angular.z = 0.0;
      maze_last_angular_command_ = 0.0;
      maze_action_ = "maze_post_turn_straight";
    } else if (deadend_turn_active_ ||
               deadend_candidate_confirmed ||
               route_decision.state == "deadend_reverse" ||
               route_decision.state == "backtrack_to_node")
    {
      maze_turn_commit_active_ = false;
      maze_turn_lock_active_ = false;
      maze_turn_blocked_by_inner_clearance_ = false;
      maze_blocked_by_robot_model_ = false;
      maze_inner_side_ = "front";
      maze_inner_clearance_ = front_min;
      maze_swept_clearance_ = std::min(front_left, front_right);
      maze_inner_front_corner_clearance_ = std::min(front_left, front_right);
      if (!maze_deadend_turnaround_enabled_) {
        set_maze_turn_phase("maze_topology_backtrack_reverse", now);
        maze_corner_phase_ = "maze_topology_backtrack_reverse";
        maze_preclear_action_ = route_decision.state;
        maze_corner_reason_ = route_decision.reason;
        maze_selected_turn_side_ = "backtrack";
        maze_speed_profile_ = "maze_topology_reverse";
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        maze_last_angular_command_ = 0.0;
        maze_action_ = route_decision.state;
      } else {
        if (!deadend_turn_active_) {
          reset_maze_corner_turn_latch("deadend_turnaround_start");
          start_deadend_turnaround(
            pose,
            now,
            deadend_candidate_confirmed ? "deadend_candidate_forced" : route_decision.reason);
        }
        publish_deadend_turnaround_command(
          cmd, pose, now, front_min, front_p20, left_side, right_side,
          left_mid, right_mid, front_left, front_right, front_p50, rear);
        maze_route_state_ = "deadend_turnaround";
        maze_route_reason_ = deadend_turn_reason_;
      }
    } else if (!deadend_turn_active_ && maze_stalled) {
      deadend_turn_active_ = false;
      const double turn_direction = maze_turn_lock_active_ ? maze_turn_direction_ : -wall_follow_direction();
      maze_turn_direction_ = turn_direction;
      maze_turn_lock_active_ = true;
      maze_turn_commit_active_ = true;
      maze_turn_commit_start_time_ = now;
      publish_maze_turn_command(
        cmd, pose, now, turn_direction, front_min, front_p20, front_p50, left_side, right_side,
        left_mid, right_mid, front_left, front_right, rear);
      maze_corner_reason_ = "maze_progress_timeout";
      maze_route_state_ = "corner_turn";
      maze_route_reason_ = "maze_progress_timeout";
    } else if (!deadend_turn_active_ &&
               (turn_commit_hold ||
                route_decision.state == "corner_turn" ||
                route_decision.state == "committed_turn"))
    {
      deadend_turn_active_ = false;
      const double turn_direction = turn_commit_hold ?
        maze_turn_direction_ : choose_maze_route_turn_direction(route_decision.selected_turn_direction, now);
      publish_maze_turn_command(
        cmd, pose, now, turn_direction, front_min, front_p20, front_p50, left_side, right_side,
        left_mid, right_mid, front_left, front_right, rear);
      maze_route_state_ = route_decision.state == "committed_turn" ? "committed_turn" : "corner_turn";
      maze_route_reason_ = turn_commit_hold ? "turn_commit_hold" : route_decision.reason;
    } else if (route_decision.state == "preclear_reverse") {
      reset_maze_corner_turn_latch("route_preclear_reverse");
      deadend_turn_active_ = false;
      maze_turn_commit_active_ = false;
      maze_turn_blocked_by_inner_clearance_ = true;
      maze_blocked_by_robot_model_ = true;
      maze_inner_side_ = "front";
      maze_inner_clearance_ = front_min;
      maze_swept_clearance_ = std::min(front_left, front_right);
      maze_inner_front_corner_clearance_ = std::min(front_left, front_right);
      set_maze_turn_phase("maze_corner_preclear", now);
      maze_corner_phase_ = "maze_corner_preclear";
      maze_preclear_action_ = "route_preclear_reverse";
      maze_corner_reason_ = route_decision.reason;
      maze_selected_turn_side_ = route_decision.selected_turn_direction == 0.0 ?
        "none" : direction_name(route_decision.selected_turn_direction);
      maze_speed_profile_ = "maze_route_reverse";
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      maze_last_angular_command_ = 0.0;
      maze_action_ = "maze_route_preclear_reverse";
    } else if (route_decision.state == "junction_select" ||
               route_decision.state == "choose_opening" ||
               route_decision.state == "complete_check")
    {
      reset_maze_corner_turn_latch("junction_or_complete");
      deadend_turn_active_ = false;
      maze_turn_lock_active_ = false;
      maze_turn_commit_active_ = false;
      maze_turn_phase_ = "maze_junction_select";
      maze_turn_blocked_by_inner_clearance_ = false;
      maze_blocked_by_robot_model_ = false;
      maze_inner_side_ = "none";
      maze_inner_clearance_ = std::numeric_limits<double>::infinity();
      maze_swept_clearance_ = std::numeric_limits<double>::infinity();
      maze_inner_front_corner_clearance_ = std::numeric_limits<double>::infinity();
      maze_preclear_action_ = "none";
      maze_corner_reason_ = route_decision.reason;
      maze_corner_phase_ = "maze_junction_select";
      maze_speed_profile_ = "maze_junction";
      cmd.linear.x = maze_topology_mode_ ?
        maze_topology_forward_speed_ : maze_forward_speed_;
      double desired_angular =
        -maze_center_kp_ * 0.5 * smoothed_center_error -
        maze_front_center_kp_ * 0.5 * maze_front_center_error_;
      if (route_decision.selected_turn_direction != 0.0) {
        desired_angular += route_decision.selected_turn_direction *
          std::min(maze_max_angular_ * 0.55, maze_turn_speed_ * 0.45);
        maze_selected_turn_side_ = direction_name(route_decision.selected_turn_direction);
      } else {
        maze_selected_turn_side_ = "front";
      }
      cmd.angular.z = clamp(desired_angular, -maze_max_angular_ * 0.6, maze_max_angular_ * 0.6);
      if (std::abs(cmd.angular.z) < angular_deadband_ * 0.5) {
        cmd.angular.z = 0.0;
      }
      maze_action_ = maze_topology_mode_ ? route_decision.state : "maze_junction_select";
    } else {
      reset_maze_corner_turn_latch("corridor_or_open_drive");
      deadend_turn_active_ = false;
      maze_turn_lock_active_ = false;
      maze_turn_commit_active_ = false;
      const bool corridor_state =
        route_decision.state == "corridor_follow" || route_decision.state == "corridor_drive";
      maze_turn_phase_ = corridor_state ? "maze_corridor_follow" : "maze_drive_open";
      maze_turn_blocked_by_inner_clearance_ = false;
      maze_inner_side_ = "none";
      maze_inner_clearance_ = std::numeric_limits<double>::infinity();
      maze_preclear_action_ = "none";
      maze_corner_reason_ = route_decision.reason;
      maze_selected_turn_side_ = "none";
      maze_corner_phase_ = maze_turn_phase_;
      maze_blocked_by_robot_model_ = false;
      maze_swept_clearance_ = std::numeric_limits<double>::infinity();
      maze_inner_front_corner_clearance_ = std::numeric_limits<double>::infinity();
      maze_speed_profile_ = corridor_state ? "maze_corridor" : "maze_open";
      cmd.linear.x = maze_topology_mode_ && route_observation.front_wide_clear ?
        maze_topology_fast_speed_ : (maze_topology_mode_ ? maze_topology_forward_speed_ : maze_forward_speed_);
      double desired_angular = 0.0;
      if (corridor_state) {
        if (route_observation.both_walls) {
          desired_angular =
            -maze_center_kp_ * smoothed_center_error -
            maze_front_center_kp_ * maze_front_center_error_;
        } else {
          desired_angular = -maze_front_center_kp_ * maze_front_center_error_;
        }
      }
      const bool follow_side_valid =
        (maze_wall_follow_side_ == "right" && route_observation.right_wall) ||
        (maze_wall_follow_side_ == "left" && route_observation.left_wall);
      if (corridor_state && follow_side_valid && !route_observation.both_walls) {
        const double followed_side_distance =
          maze_wall_follow_side_ == "right" ?
          finite_or(right_side, maze_wall_target_clearance_) :
          finite_or(left_side, maze_wall_target_clearance_);
        const double wall_error = followed_side_distance - maze_wall_target_clearance_;
        const double follow_sign = maze_wall_follow_side_ == "right" ? -1.0 : 1.0;
        desired_angular += follow_sign * maze_center_kp_ * wall_error;
      }
      cmd.angular.z = clamp(desired_angular, -maze_max_angular_, maze_max_angular_);
      if (std::abs(cmd.angular.z) < angular_deadband_ * 0.5) {
        cmd.angular.z = 0.0;
      }
      maze_action_ = corridor_state ? "maze_corridor_follow" : "maze_drive_open";
    }

    const std::string no_reverse_reason =
      maze_action_ != "none" ? maze_action_ :
      (maze_preclear_action_ != "none" ? maze_preclear_action_ : maze_route_reason_);
    apply_maze_no_reverse_mode(
      cmd, now, no_reverse_reason, front_min, front_p20, front_p50,
      left_side, right_side, left_mid, right_mid, front_left, front_right);

    yaw_error_ = 0.0;
    raw_angular_z_ = cmd.angular.z;
    cmd.angular.z = limit_maze_angular_delta(cmd.angular.z);
    limited_angular_z_ = cmd.angular.z;
    last_angular_command_ = cmd.angular.z;
    record_angular_direction(cmd.angular.z, now);
    last_cmd_reason_ = "narrow_maze_follow";
    last_progress_time_ = now;
    last_progress_pose_ = pose;
    if (std::abs(cmd.linear.x) > 0.002 || std::abs(cmd.angular.z) > 0.002) {
      const bool new_map_progress = unknown_ratio_ < maze_last_unknown_ratio_ - 0.002;
      const double pose_progress = std::hypot(
        pose.x - maze_last_progress_pose_.x,
        pose.y - maze_last_progress_pose_.y);
      const bool moved_progress = pose_progress >= std::max(0.03, progress_distance_ * 0.4);
      if (maze_last_progress_time_.nanoseconds() == 0 || new_map_progress || moved_progress) {
        maze_last_progress_time_ = now;
        maze_last_unknown_ratio_ = unknown_ratio_;
        maze_last_progress_pose_ = pose;
      }
    }
    cmd_vel_pub_->publish(cmd);
    set_status("narrow_maze_follow", maze_action_);
    return true;
  }

  void update_front_confirm_frames(double front_min, double front_p20)
  {
    if (front_blocked(front_min, front_p20)) {
      front_blocked_frames_ = std::min(front_blocked_frames_ + 1, front_block_confirm_frames_);
      front_clear_frames_ = 0;
    } else if (front_clear(front_min, front_p20)) {
      front_clear_frames_ = std::min(front_clear_frames_ + 1, front_clear_confirm_frames_);
      front_blocked_frames_ = 0;
    } else {
      front_blocked_frames_ = 0;
      front_clear_frames_ = 0;
    }
  }

  void choose_recovery_direction()
  {
    chassis_controller::ScanSectorSummary summary;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      summary = latest_scan_summary_;
    }
    const double left_score = turn_open_score(true, summary);
    const double right_score = turn_open_score(false, summary);
    const auto now = this->now();
    if (std::abs(left_score - right_score) > 0.05 || turn_lock_active_) {
      recovery_direction_ = choose_stable_turn_direction(recovery_direction_, left_score, right_score, now);
      return;
    }
    if (current_goal_cell_ >= 0) {
      Pose2D pose;
      std::string reason;
      if (lookup_robot_pose(pose, reason)) {
        const double target_yaw = std::atan2(target_.y - pose.y, target_.x - pose.x);
        const double yaw_error = normalize_angle(target_yaw - pose.yaw);
        if (std::abs(yaw_error) > 0.05) {
          const double desired = yaw_error >= 0.0 ? 1.0 : -1.0;
          recovery_direction_ = choose_stable_turn_direction(
            desired,
            desired >= 0.0 ? left_score + turn_switch_margin_ : left_score,
            desired < 0.0 ? right_score + turn_switch_margin_ : right_score,
            now);
          return;
        }
      }
    }
    if (recovery_direction_ == 0.0) {
      recovery_direction_ = 1.0;
    }
  }

  void start_emergency_escape(const rclcpp::Time& now)
  {
    escape_direction_ = choose_emergency_escape_direction();
    recovery_direction_ = escape_direction_;
    chosen_open_side_ = direction_name(escape_direction_);
    open_side_score_ = turn_open_score_for_direction(escape_direction_);
    emergency_escape_start_time_ = now;
    emergency_escape_active_ = true;
    escape_action_ = "escape_reverse";
    escape_phase_ = "reverse";
    escape_attempts_ = std::min(escape_attempts_ + 1, max_emergency_escape_attempts_);
  }

  void reset_emergency_escape(bool clear_attempts = false)
  {
    emergency_escape_active_ = false;
    emergency_escape_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    escape_action_ = "none";
    escape_phase_ = "none";
    chosen_open_side_ = "none";
    open_side_score_ = 0.0;
    if (clear_attempts) {
      escape_attempts_ = 0;
    }
  }

  void publish_emergency_escape(const rclcpp::Time& now, double front_min, double front_p20)
  {
    const double total_duration = escape_reverse_duration_ + escape_turn_duration_;
    if (
      !emergency_escape_active_ ||
      emergency_escape_start_time_.nanoseconds() == 0 ||
      (now - emergency_escape_start_time_).seconds() > total_duration)
    {
      start_emergency_escape(now);
    }

    const double elapsed = std::max(0.0, (now - emergency_escape_start_time_).seconds());
    double open_score = 0.0;
    double diagonal_p20 = 0.0;
    const bool turn_side_clear = turn_clear_for_direction(escape_direction_, open_score, diagonal_p20);
    const bool reverse_clear = rear_clear_for_reverse();
    open_side_score_ = open_score;

    geometry_msgs::msg::Twist cmd;
    const bool contact_range = front_min < hard_stop_distance_;
    const bool force_motion = escape_always_try_motion_ && (contact_range || front_emergency(front_min));

    if (elapsed < escape_reverse_duration_ && (reverse_clear || force_motion)) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      escape_action_ = reverse_clear ? "escape_reverse" : "escape_reverse_forced";
      escape_phase_ = "reverse";
    } else if (turn_side_clear || force_motion) {
      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_turn_speed_ * (escape_direction_ >= 0.0 ? 1.0 : -1.0);
      escape_action_ = turn_side_clear ? "escape_turn_open_side" : "escape_turn_probe_open_side";
      escape_phase_ = "turn";
    } else if (reverse_clear) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      escape_action_ = "escape_reverse_wait_turn_clear";
      escape_phase_ = "reverse_wait";
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = recovery_turn_speed_ * 0.5 * (escape_direction_ >= 0.0 ? 1.0 : -1.0);
      escape_action_ = "escape_probe_turn_no_clearance";
      escape_phase_ = "probe";
    }
    double front_p50 = std::numeric_limits<double>::infinity();
    double left_side = std::numeric_limits<double>::infinity();
    double right_side = std::numeric_limits<double>::infinity();
    double left_mid = std::numeric_limits<double>::infinity();
    double right_mid = std::numeric_limits<double>::infinity();
    double front_left = std::numeric_limits<double>::infinity();
    double front_right = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_p50 = front_p50_;
      left_side = left_side_p20_;
      right_side = right_side_p20_;
      left_mid = left_p20_;
      right_mid = right_p20_;
      front_left = front_left_p20_;
      front_right = front_right_p20_;
    }
    const bool no_reverse_override = apply_maze_no_reverse_mode(
      cmd, now, escape_action_, front_min, front_p20, front_p50,
      left_side, right_side, left_mid, right_mid, front_left, front_right);
    last_cmd_reason_ = escape_action_;
    raw_angular_z_ = cmd.angular.z;
    limited_angular_z_ = cmd.angular.z;
    last_angular_command_ = cmd.angular.z;
    record_angular_direction(cmd.angular.z, now);
    cmd_vel_pub_->publish(cmd);

    std::ostringstream message;
    message.setf(std::ios::fixed);
    message.precision(3);
    message << "emergency escape " << escape_action_
            << " side=" << chosen_open_side_
            << " front_min=" << front_min
            << " front_p20=" << front_p20
            << " front_diag_p20=" << diagonal_p20
            << " open_score=" << open_score
            << " rear_p20=" << rear_p20_;
    set_status(
      no_reverse_override ? "maze_rotate_search" : (contact_range ? "contact_escape" : "emergency_escape"),
      no_reverse_override ? maze_reverse_disabled_reason_ : message.str());
  }

  void control_loop()
  {
    const auto now = this->now();
    if (!enabled_) {
      set_status("disabled", "frontier explorer disabled");
      publish_stop();
      publish_status();
      return;
    }

    Pose2D pose;
    std::string wait_reason;
    if (!inputs_ready(now, wait_reason) || !lookup_robot_pose(pose, wait_reason)) {
      clear_plan();
      set_status("waiting", wait_reason);
      publish_stop();
      publish_status();
      return;
    }

    double front_p20 = 0.0;
    double front_p50 = 0.0;
    double front_min = 0.0;
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_p20 = front_p20_;
      front_p50 = front_p50_;
      front_min = front_min_;
    }
    update_front_confirm_frames(front_min, front_p20);
    current_obstacle_state_ = obstacle_state(front_min, front_p20, front_p50);

    if (maze_only_mode_ && publish_maze_follow_command(pose, now)) {
      publish_status();
      return;
    }

    if (front_contact_escape(front_min)) {
      add_failed_goal(current_goal_cell_);
      clear_plan();
      publish_emergency_escape(now, front_min, front_p20);
      publish_status();
      return;
    }

    if (publish_maze_follow_command(pose, now)) {
      publish_status();
      return;
    }

    if (front_emergency(front_min)) {
      add_failed_goal(current_goal_cell_);
      clear_plan();
      publish_emergency_escape(now, front_min, front_p20);
      publish_status();
      return;
    }

    if (publish_anti_oscillation_escape(now)) {
      publish_status();
      return;
    }

    if (emergency_escape_active_) {
      reset_emergency_escape(true);
      if (front_clear_frames_ >= front_clear_confirm_frames_) {
        release_turn_lock();
      }
    }

    if (state_ == "recovery") {
      if (front_obstacle_recovery_) {
        if (front_clear_frames_ >= front_clear_confirm_frames_) {
          front_obstacle_recovery_ = false;
          clear_plan();
        } else {
          set_status("recovery", "front obstacle too close, rotating until front clears");
          publish_recovery_turn();
          publish_status();
          return;
        }
      } else if ((now - recovery_start_time_).seconds() < recovery_duration_) {
        publish_recovery_turn();
        publish_status();
        return;
      } else {
        clear_plan();
      }
    }

    if (front_blocked_frames_ >= front_block_confirm_frames_) {
      add_failed_goal(current_goal_cell_);
      clear_plan();
      choose_recovery_direction();
      if (local_escape_target_enabled_ &&
          select_local_open_target(pose, now, "front blocked, targeting local open sector"))
      {
        follow_plan(pose, now);
        publish_status();
        return;
      }
      front_obstacle_recovery_ = true;
      recovery_start_time_ = now;
      set_status("recovery", "front obstacle too close, rotating to find free direction");
      publish_recovery_turn();
      publish_status();
      return;
    }

    const bool plan_due = last_plan_time_.nanoseconds() == 0 ||
      (now - last_plan_time_).seconds() > replan_interval_;
    if (!current_path_.empty() && plan_due &&
        should_hold_current_target(pose, now, front_min, front_p20))
    {
      last_plan_time_ = now;
      replan_reason_ = target_switch_reason_;
      follow_plan(pose, now);
      publish_status();
      return;
    }
    if (current_path_.empty() && !plan_due) {
      publish_stop();
      publish_status();
      return;
    }

    if (current_path_.empty() || plan_due) {
      if (!make_plan(pose)) {
        publish_stop();
        publish_status();
        return;
      }
    }

    follow_plan(pose, now);
    publish_status();
  }

  bool inputs_ready(const rclcpp::Time& now, std::string& reason)
  {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!latest_map_ || (now - last_map_time_).seconds() > map_timeout_) {
        reason = "waiting for fresh /map";
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      if (!has_scan_ || (now - last_scan_time_).seconds() > scan_timeout_) {
        reason = "waiting for fresh /scan";
        return false;
      }
    }
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_ || (now - last_odom_time_).seconds() > odom_timeout_) {
        reason = "waiting for fresh /odom";
        return false;
      }
    }
    return true;
  }

  bool lookup_robot_pose(Pose2D& pose, std::string& reason)
  {
    try {
      const auto tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      pose.x = tf.transform.translation.x;
      pose.y = tf.transform.translation.y;
      pose.yaw = yaw_from_quaternion(tf.transform.rotation);
      return true;
    } catch (const std::exception& exc) {
      reason = std::string("waiting for TF ") + map_frame_ + " -> " + base_frame_ + ": " + exc.what();
      return false;
    }
  }

  bool make_plan(const Pose2D& pose)
  {
    const auto now = this->now();
    nav_msgs::msg::OccupancyGrid::SharedPtr map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = latest_map_;
    }
    if (!map) {
      set_status("waiting", "waiting for /map");
      return false;
    }

    GridPlan initial_grid = build_grid(*map, obstacle_padding_);
    unknown_ratio_ = initial_grid.unknown_ratio;
    const bool mostly_known = initial_grid.unknown_ratio <= unknown_complete_ratio_;
    if (mostly_known) {
      if (complete_candidate_since_.nanoseconds() == 0) {
        complete_candidate_since_ = now;
      }
      if ((now - complete_candidate_since_).seconds() >= complete_confirm_duration_) {
        clear_plan();
        last_plan_time_ = now;
        set_status("completed", "unknown space ratio is below threshold");
        return false;
      }
    } else {
      complete_candidate_since_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    std::vector<double> paddings = planning_paddings();
    std::string last_start_error = "no reachable exploration target";
    int best_reachable_free_count = 0;
    int best_frontier_count = 0;
    int best_reachable_frontier_count = 0;
    int best_edge_count = 0;
    rejected_too_close_targets_ = 0;
    replan_reason_ = "planning";

    for (double padding : paddings) {
      GridPlan grid = build_grid(*map, padding);
      int start = -1;
      if (!resolve_start_cell(grid, pose, start, last_start_error)) {
        continue;
      }

      std::vector<FrontierCluster> clusters = find_frontiers(grid);
      std::vector<float> distances;
      compute_reachable_distances(grid, start, distances);
      const int reachable_free_count = count_reachable_free(distances);
      best_reachable_free_count = std::max(best_reachable_free_count, reachable_free_count);

      frontier_count_ = static_cast<int>(clusters.size());
      current_planning_padding_ = padding;
      using_edge_fallback_ = false;
      reachable_edge_count_ = 0;

      FrontierCluster selected;
      bool selected_edge_target = false;
      if (!select_frontier(grid, clusters, distances, pose, selected) &&
          edge_fallback_enabled_ && !mostly_known) {
        selected_edge_target = select_reachable_edge_target(grid, distances, pose, selected);
      }

      best_frontier_count = std::max(best_frontier_count, frontier_count_);
      best_reachable_frontier_count = std::max(best_reachable_frontier_count, reachable_frontier_count_);
      best_edge_count = std::max(best_edge_count, reachable_edge_count_);

      if (selected.target_index < 0) {
        continue;
      }

      std::vector<int> path_cells;
      if (!plan_path_a_star(grid, start, selected.target_index, path_cells)) {
        add_failed_goal(selected.target_index);
        continue;
      }

      std::string switch_reason;
      if (!should_accept_new_target(pose, selected, selected_edge_target, grid, switch_reason)) {
        target_switch_blocked_ = true;
        target_switch_reason_ = switch_reason;
        replan_reason_ = switch_reason;
        last_plan_time_ = now;
        RCLCPP_INFO(this->get_logger(),
          "Keeping current target: rejected %s target cell=%d distance=%.2f score=%.2f current_score=%.2f reason=%s",
          selected_edge_target ? "reachable edge" : "frontier",
          selected.target_index, selected.distance_m, selected.score, current_target_score_, switch_reason.c_str());
        return !current_path_.empty();
      }

      current_path_ = cells_to_world_path(grid, path_cells);
      current_goal_cell_ = selected.target_index;
      map_to_world(grid, selected.target_index, target_.x, target_.y);
      last_plan_time_ = now;
      last_progress_time_ = now;
      last_progress_pose_ = pose;
      last_goal_distance_ = distance_to_target(pose);
      state_ = selected_edge_target ? "edge_fallback" : "exploring";
      message_ = selected_edge_target ? "fallback to reachable exploration edge" : "moving to frontier";
      path_points_ = static_cast<int>(current_path_.size());
      no_frontier_recovery_count_ = 0;
      using_edge_fallback_ = selected_edge_target;
      target_type_ = selected_edge_target ? "reachable_edge" : "frontier";
      local_target_type_ = "none";
      current_target_score_ = selected.score;
      target_selected_time_ = now;
      target_switch_blocked_ = false;
      target_switch_reason_ = switch_reason;
      replan_reason_ = switch_reason;
      RCLCPP_INFO(this->get_logger(),
        "Selected %s: padding=%.2f clusters=%d reachable=%d edge=%d rejected_close=%d path=%d target=(%.2f, %.2f) distance=%.2f score=%.2f unknown=%.3f",
        selected_edge_target ? "reachable edge" : "frontier",
        current_planning_padding_, frontier_count_, reachable_frontier_count_, reachable_edge_count_,
        rejected_too_close_targets_, path_points_, target_.x, target_.y, selected.distance_m, selected.score, unknown_ratio_);
      return true;
    }

    clear_plan();
    frontier_count_ = best_frontier_count;
    reachable_frontier_count_ = best_reachable_frontier_count;
    reachable_edge_count_ = best_edge_count;
    last_plan_time_ = now;
    if (best_reachable_free_count <= 0) {
      replan_reason_ = "no_reachable_free_space";
      set_status("blocked", "no reachable free space");
      return false;
    }
    if (mostly_known) {
      replan_reason_ = "low_unknown_ratio";
      set_status("replanning", "low unknown ratio, confirming completion");
      return false;
    }

    ++no_frontier_recovery_count_;
    state_ = "recovery";
    front_obstacle_recovery_ = false;
    recovery_start_time_ = now;
    choose_recovery_direction();
    if (no_frontier_recovery_count_ > max_no_frontier_recoveries_) {
      replan_reason_ = "no_reachable_target_keep_recovery";
      set_status("recovery", "no reachable exploration target after fallback, keep rotating because map is still unknown");
    } else {
      replan_reason_ = last_start_error;
      set_status("recovery", last_start_error);
    }
    return false;
  }

  bool resolve_start_cell(
    const GridPlan& grid,
    const Pose2D& pose,
    int& start,
    std::string& reason) const
  {
    if (!world_to_index(grid, pose.x, pose.y, start)) {
      reason = "robot pose is outside current map";
      return false;
    }
    if (is_free_cell(grid, start) && !grid.blocked[start]) {
      return true;
    }
    int nearest_free = -1;
    if (find_nearest_free_index(grid, start, start_search_radius_, nearest_free)) {
      start = nearest_free;
      return true;
    }
    reason = "robot pose is not in known free map space";
    return false;
  }

  std::vector<double> planning_paddings() const
  {
    std::vector<double> paddings;
    const double values[] = {
      obstacle_padding_,
      fallback_obstacle_padding_1_,
      fallback_obstacle_padding_2_,
    };
    for (double value : values) {
      if (value < 0.0) {
        continue;
      }
      bool exists = false;
      for (double added : paddings) {
        if (std::abs(added - value) < 1e-6) {
          exists = true;
          break;
        }
      }
      if (!exists) {
        paddings.push_back(value);
      }
    }
    std::sort(paddings.begin(), paddings.end(), [](double a, double b) {
      return a > b;
    });
    return paddings;
  }

  GridPlan build_grid(const nav_msgs::msg::OccupancyGrid& map) const
  {
    return build_grid(map, obstacle_padding_);
  }

  GridPlan build_grid(const nav_msgs::msg::OccupancyGrid& map, double obstacle_padding) const
  {
    GridPlan grid;
    grid.width = static_cast<int>(map.info.width);
    grid.height = static_cast<int>(map.info.height);
    grid.resolution = static_cast<double>(map.info.resolution);
    grid.origin_x = static_cast<double>(map.info.origin.position.x);
    grid.origin_y = static_cast<double>(map.info.origin.position.y);
    grid.data = map.data;
    grid.blocked.assign(grid.data.size(), 0);

    int unknown_count = 0;
    for (size_t i = 0; i < grid.data.size(); ++i) {
      if (grid.data[i] < 0) {
        ++unknown_count;
        grid.blocked[i] = 1;
      } else if (grid.data[i] >= occupied_threshold_) {
        grid.blocked[i] = 1;
      } else if (grid.data[i] > free_threshold_) {
        grid.blocked[i] = 1;
      }
    }
    grid.unknown_ratio = grid.data.empty() ? 1.0 : static_cast<double>(unknown_count) / static_cast<double>(grid.data.size());

    const int pad_cells = std::max(0, static_cast<int>(std::ceil(obstacle_padding / std::max(grid.resolution, 0.001))));
    std::vector<uint8_t> inflated = grid.blocked;
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int idx = index(grid, x, y);
        if (grid.data[idx] < occupied_threshold_) {
          continue;
        }
        for (int dy = -pad_cells; dy <= pad_cells; ++dy) {
          for (int dx = -pad_cells; dx <= pad_cells; ++dx) {
            if (dx * dx + dy * dy > pad_cells * pad_cells) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (in_bounds(grid, nx, ny)) {
              inflated[index(grid, nx, ny)] = 1;
            }
          }
        }
      }
    }
    grid.blocked.swap(inflated);
    return grid;
  }

  int count_reachable_free(const std::vector<float>& distances) const
  {
    int count = 0;
    for (float distance : distances) {
      if (distance >= 0.0f) {
        ++count;
      }
    }
    return count;
  }

  bool select_reachable_edge_target(
    const GridPlan& grid,
    const std::vector<float>& distances,
    const Pose2D& pose,
    FrontierCluster& selected)
  {
    reachable_edge_count_ = 0;
    bool have = false;
    double best_score = -std::numeric_limits<double>::infinity();
    const double min_edge_distance = std::max(edge_min_goal_distance_, goal_tolerance_ + 0.15);
    const int unknown_radius_cells = std::max(
      1, static_cast<int>(std::ceil(edge_unknown_radius_ / std::max(grid.resolution, 0.001))));

    for (int cell = 0; cell < static_cast<int>(grid.data.size()); ++cell) {
      if (cell >= static_cast<int>(distances.size()) || distances[cell] < 0.0f ||
          grid.blocked[cell] || !is_free_cell(grid, cell)) {
        continue;
      }
      const double robot_distance = static_cast<double>(distances[cell]) * grid.resolution;
      double wx = 0.0;
      double wy = 0.0;
      map_to_world(grid, cell, wx, wy);
      const double euclidean_distance = std::hypot(wx - pose.x, wy - pose.y);
      if (robot_distance < min_edge_distance || euclidean_distance < min_edge_distance) {
        ++rejected_too_close_targets_;
        continue;
      }
      if (robot_distance > max_goal_distance_ || euclidean_distance > max_goal_distance_) {
        continue;
      }

      int unknown_neighbors = 0;
      if (!unknown_nearby(grid, cell, unknown_radius_cells, unknown_neighbors)) {
        continue;
      }
      ++reachable_edge_count_;

      const double yaw_error = std::abs(normalize_angle(std::atan2(wy - pose.y, wx - pose.x) - pose.yaw));
      const double info_gain = static_cast<double>(unknown_neighbors) * grid.resolution * grid.resolution;
      double recent_penalty = 0.0;
      for (int failed : failed_goals_) {
        int fx = 0;
        int fy = 0;
        int tx_cell = 0;
        int ty_cell = 0;
        index_to_xy(grid, failed, fx, fy);
        index_to_xy(grid, cell, tx_cell, ty_cell);
        if (std::hypot(static_cast<double>(fx - tx_cell), static_cast<double>(fy - ty_cell)) * grid.resolution < 0.7) {
          recent_penalty += recent_goal_penalty_;
        }
      }
      const double score =
        info_gain_weight_ * info_gain -
        distance_weight_ * robot_distance -
        yaw_weight_ * yaw_error -
        recent_penalty;

      if (!have || score > best_score) {
        best_score = score;
        selected.cells.clear();
        selected.cells.push_back(cell);
        selected.target_index = cell;
        selected.centroid_x = wx;
        selected.centroid_y = wy;
        selected.distance_m = euclidean_distance;
        selected.score = score;
        have = true;
      }
    }
    return have;
  }

  bool unknown_nearby(
    const GridPlan& grid,
    int cell,
    int radius_cells,
    int& unknown_count) const
  {
    int sx = 0;
    int sy = 0;
    index_to_xy(grid, cell, sx, sy);
    unknown_count = 0;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        if (dx * dx + dy * dy > radius_cells * radius_cells) {
          continue;
        }
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (!in_bounds(grid, nx, ny)) {
          continue;
        }
        if (grid.data[index(grid, nx, ny)] < 0) {
          ++unknown_count;
        }
      }
    }
    return unknown_count > 0;
  }

  std::vector<FrontierCluster> find_frontiers(const GridPlan& grid) const
  {
    std::vector<uint8_t> frontier(grid.data.size(), 0);
    std::vector<uint8_t> visited(grid.data.size(), 0);
    for (int y = 1; y < grid.height - 1; ++y) {
      for (int x = 1; x < grid.width - 1; ++x) {
        const int idx = index(grid, x, y);
        if (!is_free_cell(grid, idx) || grid.blocked[idx]) {
          continue;
        }
        if (has_unknown_neighbor(grid, x, y)) {
          frontier[idx] = 1;
        }
      }
    }

    std::vector<FrontierCluster> result;
    std::queue<int> q;
    for (size_t i = 0; i < frontier.size(); ++i) {
      if (!frontier[i] || visited[i]) {
        continue;
      }
      FrontierCluster cluster;
      visited[i] = 1;
      q.push(static_cast<int>(i));
      while (!q.empty()) {
        const int current = q.front();
        q.pop();
        cluster.cells.push_back(current);
        int cx = 0;
        int cy = 0;
        index_to_xy(grid, current, cx, cy);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = cx + dx;
            const int ny = cy + dy;
            if (!in_bounds(grid, nx, ny)) {
              continue;
            }
            const int ni = index(grid, nx, ny);
            if (frontier[ni] && !visited[ni]) {
              visited[ni] = 1;
              q.push(ni);
            }
          }
        }
      }
      if (static_cast<int>(cluster.cells.size()) >= min_frontier_cells_) {
        double sum_x = 0.0;
        double sum_y = 0.0;
        for (int cell : cluster.cells) {
          double wx = 0.0;
          double wy = 0.0;
          map_to_world(grid, cell, wx, wy);
          sum_x += wx;
          sum_y += wy;
        }
        cluster.centroid_x = sum_x / static_cast<double>(cluster.cells.size());
        cluster.centroid_y = sum_y / static_cast<double>(cluster.cells.size());
        result.push_back(cluster);
      }
    }
    return result;
  }

  bool select_frontier(
    const GridPlan& grid,
    std::vector<FrontierCluster>& clusters,
    const std::vector<float>& distances,
    const Pose2D& pose,
    FrontierCluster& selected)
  {
    reachable_frontier_count_ = 0;
    bool have = false;
    double best_score = -std::numeric_limits<double>::infinity();
    const double min_frontier_distance = std::max(min_goal_distance_, goal_tolerance_ + 0.15);

    for (auto& cluster : clusters) {
      int best_cell = -1;
      double best_centroid_distance = std::numeric_limits<double>::infinity();
      double best_robot_distance = 0.0;
      for (int cell : cluster.cells) {
        if (cell < 0 || cell >= static_cast<int>(distances.size()) || distances[cell] < 0.0f) {
          continue;
        }
        const double robot_distance = static_cast<double>(distances[cell]) * grid.resolution;
        double wx = 0.0;
        double wy = 0.0;
        map_to_world(grid, cell, wx, wy);
        const double euclidean_distance = std::hypot(wx - pose.x, wy - pose.y);
        if (robot_distance < min_frontier_distance || euclidean_distance < min_frontier_distance) {
          ++rejected_too_close_targets_;
          continue;
        }
        if (robot_distance > max_goal_distance_ || euclidean_distance > max_goal_distance_) {
          continue;
        }
        const double centroid_distance = std::hypot(wx - cluster.centroid_x, wy - cluster.centroid_y);
        if (centroid_distance < best_centroid_distance) {
          best_centroid_distance = centroid_distance;
          best_robot_distance = euclidean_distance;
          best_cell = cell;
        }
      }
      if (best_cell < 0) {
        continue;
      }
      ++reachable_frontier_count_;
      double tx = 0.0;
      double ty = 0.0;
      map_to_world(grid, best_cell, tx, ty);
      const double yaw_error = std::abs(normalize_angle(std::atan2(ty - pose.y, tx - pose.x) - pose.yaw));
      const double info_gain = static_cast<double>(cluster.cells.size()) * grid.resolution * grid.resolution;
      double recent_penalty = 0.0;
      for (int failed : failed_goals_) {
        int fx = 0;
        int fy = 0;
        int tx_cell = 0;
        int ty_cell = 0;
        index_to_xy(grid, failed, fx, fy);
        index_to_xy(grid, best_cell, tx_cell, ty_cell);
        if (std::hypot(static_cast<double>(fx - tx_cell), static_cast<double>(fy - ty_cell)) * grid.resolution < 0.7) {
          recent_penalty += recent_goal_penalty_;
        }
      }
      cluster.target_index = best_cell;
      cluster.distance_m = best_robot_distance;
      cluster.score =
        info_gain_weight_ * info_gain -
        distance_weight_ * best_robot_distance -
        yaw_weight_ * yaw_error -
        recent_penalty;
      if (!have || cluster.score > best_score) {
        best_score = cluster.score;
        selected = cluster;
        have = true;
      }
    }
    return have;
  }

  void compute_reachable_distances(const GridPlan& grid, int start, std::vector<float>& distances) const
  {
    distances.assign(grid.data.size(), -1.0f);
    std::queue<int> q;
    distances[start] = 0.0f;
    q.push(start);
    while (!q.empty()) {
      const int current = q.front();
      q.pop();
      int x = 0;
      int y = 0;
      index_to_xy(grid, current, x, y);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (!in_bounds(grid, nx, ny)) {
            continue;
          }
          const int ni = index(grid, nx, ny);
          if (distances[ni] >= 0.0f || grid.blocked[ni] || !is_free_cell(grid, ni)) {
            continue;
          }
          distances[ni] = distances[current] + ((dx == 0 || dy == 0) ? 1.0f : 1.4142f);
          q.push(ni);
        }
      }
    }
  }

  bool plan_path_a_star(const GridPlan& grid, int start, int goal, std::vector<int>& path) const
  {
    path.clear();
    if (start < 0 || goal < 0 || start >= static_cast<int>(grid.data.size()) || goal >= static_cast<int>(grid.data.size())) {
      return false;
    }

    std::vector<double> g_score(grid.data.size(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(grid.data.size(), -1);
    std::vector<uint8_t> closed(grid.data.size(), 0);
    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open;

    g_score[start] = 0.0;
    open.push(QueueNode{start, heuristic(grid, start, goal), 0.0});
    while (!open.empty()) {
      const QueueNode node = open.top();
      open.pop();
      const int current = node.index;
      if (closed[current]) {
        continue;
      }
      if (current == goal) {
        int p = goal;
        while (p >= 0) {
          path.push_back(p);
          if (p == start) {
            break;
          }
          p = parent[p];
        }
        std::reverse(path.begin(), path.end());
        return path.size() > 1;
      }
      closed[current] = 1;

      int x = 0;
      int y = 0;
      index_to_xy(grid, current, x, y);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (!in_bounds(grid, nx, ny)) {
            continue;
          }
          const int ni = index(grid, nx, ny);
          if (closed[ni] || grid.blocked[ni] || !is_free_cell(grid, ni)) {
            continue;
          }
          const double step = (dx == 0 || dy == 0) ? 1.0 : 1.4142;
          const double tentative = g_score[current] + step;
          if (tentative < g_score[ni]) {
            parent[ni] = current;
            g_score[ni] = tentative;
            open.push(QueueNode{ni, tentative + heuristic(grid, ni, goal), tentative});
          }
        }
      }
    }
    return false;
  }

  std::vector<Point2D> cells_to_world_path(const GridPlan& grid, const std::vector<int>& cells) const
  {
    std::vector<Point2D> path;
    const int stride = std::max(1, static_cast<int>(std::round(0.12 / std::max(grid.resolution, 0.001))));
    for (size_t i = 0; i < cells.size(); i += static_cast<size_t>(stride)) {
      Point2D point;
      map_to_world(grid, cells[i], point.x, point.y);
      path.push_back(point);
    }
    if (!cells.empty()) {
      Point2D goal;
      map_to_world(grid, cells.back(), goal.x, goal.y);
      if (path.empty() || std::hypot(path.back().x - goal.x, path.back().y - goal.y) > 0.05) {
        path.push_back(goal);
      }
    }
    return path;
  }

  void follow_plan(const Pose2D& pose, const rclcpp::Time& now)
  {
    const double goal_distance = distance_to_target(pose);
    if (goal_distance <= goal_tolerance_) {
      const std::string reached_target_type = target_type_;
      const int reached_goal_cell = current_goal_cell_;
      const bool reached_local_target = target_type_ == "local_open_sector";
      last_reached_target_type_ = reached_target_type;
      last_reached_goal_distance_ = goal_distance;
      if (reached_target_type == "reachable_edge" && unknown_ratio_ > unknown_complete_ratio_) {
        add_failed_goal(reached_goal_cell);
        replan_reason_ = "reachable_edge_reached_blacklisted";
      } else {
        replan_reason_ = reached_target_type + "_reached";
      }
      clear_plan();
      set_status(
        "replanning",
        reached_local_target ? "local open sector reached, selecting exploration target" :
          "frontier reached, selecting next target");
      publish_stop();
      return;
    }

    const double pose_delta = std::hypot(pose.x - last_progress_pose_.x, pose.y - last_progress_pose_.y);
    if (last_goal_distance_ - goal_distance > progress_distance_ || pose_delta > progress_distance_) {
      last_progress_time_ = now;
      last_progress_pose_ = pose;
      last_goal_distance_ = goal_distance;
    } else if ((now - last_progress_time_).seconds() > progress_timeout_) {
      add_failed_goal(current_goal_cell_);
      state_ = "recovery";
      front_obstacle_recovery_ = false;
      message_ = "no progress, rotating before replanning";
      ++failure_count_;
      recovery_start_time_ = now;
      choose_recovery_direction();
      publish_recovery_turn();
      return;
    }

    const Point2D target = lookahead_point(pose);
    const double target_yaw = std::atan2(target.y - pose.y, target.x - pose.x);
    const double yaw_error = normalize_angle(target_yaw - pose.yaw);
    yaw_error_ = yaw_error;

    geometry_msgs::msg::Twist cmd;
    if (std::abs(yaw_error) > rotate_in_place_yaw_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = clamp(yaw_kp_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    } else {
      const double yaw_factor = clamp(1.0 - std::abs(yaw_error) / rotate_in_place_yaw_, 0.25, 1.0);
      const double goal_factor = clamp(goal_distance / 0.8, 0.35, 1.0);
      cmd.linear.x = clamp(max_linear_speed_ * yaw_factor * goal_factor, min_linear_speed_, max_linear_speed_);
      cmd.angular.z = clamp(yaw_kp_ * yaw_error, -max_angular_speed_, max_angular_speed_);
    }
    std::string follow_state = target_type_ == "local_open_sector" ?
      "local_open_sector" : (using_edge_fallback_ ? "edge_fallback" : "exploring");
    std::string follow_message = target_type_ == "local_open_sector" ?
      "moving toward local open sector" :
      (using_edge_fallback_ ? "fallback to reachable exploration edge" : "moving to frontier");
    double front_min = std::numeric_limits<double>::infinity();
    double front_p20 = std::numeric_limits<double>::infinity();
    double front_p50 = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(scan_mutex_);
      front_min = front_min_;
      front_p20 = front_p20_;
      front_p50 = front_p50_;
    }

    if (front_contact_escape(front_min) || front_emergency(front_min)) {
      clear_plan();
      publish_emergency_escape(now, front_min, front_p20);
      return;
    }

    if (can_creep_in_narrow_corridor(front_min, front_p20, front_p50, yaw_error) && cmd.linear.x > 0.0) {
      cmd.linear.x = clamp(cmd.linear.x, std::min(min_linear_speed_, narrow_creep_speed_), narrow_creep_speed_);
      cmd.angular.z = clamp(cmd.angular.z, -recovery_turn_speed_, recovery_turn_speed_);
      follow_state = "narrow_corridor_creep";
      follow_message = "slow creep through narrow corridor";
    } else if (front_p20 < narrow_creep_min_front_p20_ && cmd.linear.x > 0.0) {
      cmd.linear.x = 0.0;
      if (std::abs(cmd.angular.z) < recovery_turn_speed_) {
        choose_recovery_direction();
        cmd.angular.z = recovery_turn_speed_ * (recovery_direction_ >= 0.0 ? 1.0 : -1.0);
      }
      follow_state = "near_obstacle_turning";
      follow_message = "turning in place near obstacle";
    }

    raw_angular_z_ = cmd.angular.z;
    if (apply_corner_turn_guard(cmd)) {
      follow_state = "corner_turn_guard";
      follow_message = corner_guard_reason_;
    }
    cmd.angular.z = stabilize_angular_command(cmd.angular.z, yaw_error, now);
    limited_angular_z_ = cmd.angular.z;
    last_cmd_reason_ = follow_state;
    cmd_vel_pub_->publish(cmd);
    set_status(follow_state, follow_message);
  }

  Point2D lookahead_point(const Pose2D& pose) const
  {
    if (current_path_.empty()) {
      return target_;
    }
    size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < current_path_.size(); ++i) {
      const double d = std::hypot(current_path_[i].x - pose.x, current_path_[i].y - pose.y);
      if (d < nearest_distance) {
        nearest_distance = d;
        nearest = i;
      }
    }
    for (size_t i = nearest; i < current_path_.size(); ++i) {
      if (std::hypot(current_path_[i].x - pose.x, current_path_[i].y - pose.y) >= lookahead_distance_) {
        return current_path_[i];
      }
    }
    return current_path_.back();
  }

  void clear_plan()
  {
    current_path_.clear();
    current_goal_cell_ = -1;
    path_points_ = 0;
    using_edge_fallback_ = false;
    target_type_ = "none";
    local_target_type_ = "none";
    last_plan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    target_selected_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    current_target_score_ = -std::numeric_limits<double>::infinity();
    target_switch_blocked_ = false;
    target_switch_reason_ = "plan_cleared";
  }

  bool select_local_open_target(const Pose2D& pose, const rclcpp::Time& now, const std::string& reason)
  {
    if (!local_escape_target_enabled_) {
      return false;
    }
    const double yaw = pose.yaw + (recovery_direction_ >= 0.0 ? local_escape_angle_ : -local_escape_angle_);
    target_.x = pose.x + local_escape_distance_ * std::cos(yaw);
    target_.y = pose.y + local_escape_distance_ * std::sin(yaw);
    current_path_.clear();
    current_path_.push_back(target_);
    current_goal_cell_ = -1;
    path_points_ = 1;
    using_edge_fallback_ = false;
    target_type_ = "local_open_sector";
    local_target_type_ = direction_name(recovery_direction_);
    last_plan_time_ = now;
    target_selected_time_ = now;
    current_target_score_ = 0.0;
    last_progress_time_ = now;
    last_progress_pose_ = pose;
    last_goal_distance_ = distance_to_target(pose);
    state_ = "local_open_sector";
    message_ = reason;
    return true;
  }

  void add_failed_goal(int cell)
  {
    if (cell < 0) {
      return;
    }
    failed_goals_.push_back(cell);
    while (static_cast<int>(failed_goals_.size()) > max_failed_goals_) {
      failed_goals_.erase(failed_goals_.begin());
    }
  }

  void publish_stop()
  {
    last_cmd_reason_ = "stop";
    raw_angular_z_ = 0.0;
    limited_angular_z_ = 0.0;
    last_angular_command_ = 0.0;
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
  }

  void publish_recovery_turn()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = recovery_turn_speed_ * (recovery_direction_ >= 0.0 ? 1.0 : -1.0);
    last_cmd_reason_ = "recovery_turn";
    raw_angular_z_ = cmd.angular.z;
    limited_angular_z_ = cmd.angular.z;
    last_angular_command_ = cmd.angular.z;
    record_angular_direction(cmd.angular.z, this->now());
    cmd_vel_pub_->publish(cmd);
  }

  void set_status(const std::string& state, const std::string& message)
  {
    state_ = state;
    message_ = message;
  }

  void publish_status()
  {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out.setf(std::ios::fixed);
    out.precision(3);
    out << "{";
    out << "\"mode\":\"frontier\",";
    out << "\"motion_policy\":\"" << (disable_hard_stop_in_mapping_ ? "no_global_stop" : "hard_stop") << "\",";
    out << "\"exploration_mode\":\"" << json_escape(state_) << "\",";
    out << "\"state\":\"" << json_escape(state_) << "\",";
    out << "\"message\":\"" << json_escape(message_) << "\",";
    out << "\"frontier_count\":" << frontier_count_ << ",";
    out << "\"reachable_frontier_count\":" << reachable_frontier_count_ << ",";
    out << "\"reachable_edge_count\":" << reachable_edge_count_ << ",";
    out << "\"path_points\":" << path_points_ << ",";
    out << "\"failed_goals\":" << failed_goals_.size() << ",";
    out << "\"failure_count\":" << failure_count_ << ",";
    out << "\"no_frontier_recoveries\":" << no_frontier_recovery_count_ << ",";
    out << "\"replan_reason\":\"" << json_escape(replan_reason_) << "\",";
    out << "\"current_target_age_s\":" << current_target_age(this->now()) << ",";
    out << "\"target_switch_blocked\":" << (target_switch_blocked_ ? "true" : "false") << ",";
    out << "\"target_switch_reason\":\"" << json_escape(target_switch_reason_) << "\",";
    out << "\"rejected_too_close_targets\":" << rejected_too_close_targets_ << ",";
    out << "\"last_reached_target_type\":\"" << json_escape(last_reached_target_type_) << "\",";
    out << "\"last_reached_goal_distance\":" << last_reached_goal_distance_ << ",";
    out << "\"unknown_ratio\":" << unknown_ratio_ << ",";
    out << "\"planning_padding\":" << current_planning_padding_ << ",";
    out << "\"edge_fallback\":" << (using_edge_fallback_ ? "true" : "false") << ",";
    out << "\"front_min\":" << front_min_ << ",";
    out << "\"front_p20\":" << front_p20_ << ",";
    out << "\"front_p50\":" << front_p50_ << ",";
    out << "\"front_valid_count\":" << front_valid_count_ << ",";
    out << "\"front_left_p20\":" << front_left_p20_ << ",";
    out << "\"front_right_p20\":" << front_right_p20_ << ",";
    out << "\"left_p20\":" << left_p20_ << ",";
    out << "\"right_p20\":" << right_p20_ << ",";
    out << "\"left_side_p20\":" << left_side_p20_ << ",";
    out << "\"right_side_p20\":" << right_side_p20_ << ",";
    out << "\"rear_p20\":" << rear_p20_ << ",";
    out << "\"obstacle_state\":\"" << json_escape(current_obstacle_state_) << "\",";
    out << "\"hard_stop\":" << (front_hard_stop(front_min_) ? "true" : "false") << ",";
    out << "\"contact_escape\":" << (front_contact_escape(front_min_) ? "true" : "false") << ",";
    out << "\"blocked_forward_only\":" << ((front_contact_escape(front_min_) || front_emergency(front_min_)) ? "true" : "false") << ",";
    out << "\"hard_stop_distance\":" << hard_stop_distance_ << ",";
    out << "\"front_emergency_distance\":" << front_emergency_distance_ << ",";
    out << "\"front_block_distance\":" << front_block_distance_ << ",";
    out << "\"front_clear_distance\":" << front_clear_distance_ << ",";
    out << "\"narrow_creep_min_front_p20\":" << narrow_creep_min_front_p20_ << ",";
    out << "\"emergency_turn_min_front_diagonal_p20\":" << emergency_turn_min_front_diagonal_p20_ << ",";
    out << "\"emergency_reverse_min_rear_p20\":" << emergency_reverse_min_rear_p20_ << ",";
    out << "\"front_blocked_frames\":" << front_blocked_frames_ << ",";
    out << "\"front_clear_frames\":" << front_clear_frames_ << ",";
    out << "\"recovery_direction\":\"" << (recovery_direction_ >= 0.0 ? "left" : "right") << "\",";
    out << "\"turn_lock_side\":\"" << (turn_lock_active_ ? direction_name(turn_lock_direction_) : "none") << "\",";
    out << "\"turn_lock_age_s\":" << (turn_lock_active_ ? (this->now() - turn_lock_start_time_).seconds() : 0.0) << ",";
    out << "\"turn_switch_blocked\":" << (turn_switch_blocked_ ? "true" : "false") << ",";
    out << "\"oscillation_detected\":" << (oscillation_detected_ ? "true" : "false") << ",";
    out << "\"oscillation_flip_count\":" << oscillation_flip_count_ << ",";
    out << "\"anti_oscillation_active\":" << (anti_oscillation_active_ ? "true" : "false") << ",";
    out << "\"corner_guard_active\":" << (corner_guard_active_ ? "true" : "false") << ",";
    out << "\"corner_guard_reason\":\"" << json_escape(corner_guard_reason_) << "\",";
    out << "\"maze_follow_active\":" << (maze_follow_active_ ? "true" : "false") << ",";
    out << "\"maze_priority_active\":" << (maze_priority_active_ ? "true" : "false") << ",";
    out << "\"maze_only_mode\":" << (maze_only_mode_ ? "true" : "false") << ",";
    out << "\"maze_wall_follow_side\":\"" << json_escape(maze_wall_follow_side_) << "\",";
    out << "\"maze_center_error\":" << maze_center_error_ << ",";
    out << "\"maze_side_center_error\":" << maze_side_center_error_ << ",";
    out << "\"maze_front_center_error\":" << maze_front_center_error_ << ",";
    out << "\"maze_smoothed_center_error\":" << maze_smoothed_center_error_ << ",";
    out << "\"maze_action\":\"" << json_escape(maze_action_) << "\",";
    out << "\"maze_route_mode\":\"" << json_escape(maze_route_mode_) << "\",";
    out << "\"maze_route_state\":\"" << json_escape(maze_route_state_) << "\",";
    out << "\"maze_route_reason\":\"" << json_escape(maze_route_reason_) << "\",";
    out << "\"graph_explore_state\":\"" << json_escape(graph_explore_state_) << "\",";
    out << "\"graph_selected_opening\":\"" << json_escape(topology_selected_opening_) << "\",";
    out << "\"graph_left_score\":" << graph_left_score_ << ",";
    out << "\"graph_front_score\":" << graph_front_score_ << ",";
    out << "\"graph_right_score\":" << graph_right_score_ << ",";
    out << "\"graph_junction_advance_distance\":" << graph_junction_advance_distance_ << ",";
    out << "\"graph_junction_center_advance\":" << maze_graph_junction_center_advance_ << ",";
    out << "\"graph_junction_front_guard_active\":" <<
      (graph_junction_front_guard_active_ ? "true" : "false") << ",";
    out << "\"graph_pre_turn_shift_distance\":" << graph_pre_turn_shift_distance_ << ",";
    out << "\"graph_pre_turn_shift_target\":" << maze_graph_pre_turn_shift_distance_ << ",";
    out << "\"graph_pre_turn_shift_outer_clearance\":" <<
      graph_pre_turn_shift_outer_clearance_ << ",";
    out << "\"graph_pre_turn_shift_skipped\":" <<
      (graph_pre_turn_shift_skipped_ ? "true" : "false") << ",";
    out << "\"graph_post_turn_advance_distance\":" << graph_post_turn_advance_distance_ << ",";
    out << "\"graph_post_turn_advance\":" << maze_graph_post_turn_advance_ << ",";
    out << "\"maze_topology_mode\":" << (maze_topology_mode_ ? "true" : "false") << ",";
    out << "\"topology_node_count\":" << maze_topology_nodes_.size() << ",";
    out << "\"topology_edge_count\":" << maze_topology_edge_count() << ",";
    out << "\"current_node_id\":" << current_topology_node_id_ << ",";
    out << "\"selected_opening\":\"" << json_escape(topology_selected_opening_) << "\",";
    out << "\"unvisited_opening_count\":" << topology_unvisited_opening_count_ << ",";
    out << "\"backtrack_stack_size\":" << maze_backtrack_stack_.size() << ",";
    out << "\"left_open_score\":" << left_open_score_ << ",";
    out << "\"front_open_score\":" << front_open_score_ << ",";
    out << "\"right_open_score\":" << right_open_score_ << ",";
    out << "\"maze_left_wall_confirm_frames\":" << maze_left_wall_confirm_frames_ << ",";
    out << "\"maze_right_wall_confirm_frames\":" << maze_right_wall_confirm_frames_ << ",";
    out << "\"maze_any_wall_confirm_frames\":" << maze_any_wall_confirm_frames_ << ",";
    out << "\"maze_front_blocked_confirm_frames\":" << maze_front_blocked_confirm_frames_ << ",";
    out << "\"maze_front_open_confirm_frames\":" << maze_front_open_confirm_frames_ << ",";
    out << "\"maze_deadend_candidate_confirm_frames\":" << maze_deadend_candidate_confirm_frames_ << ",";
    out << "\"maze_deadend_candidate_active\":" <<
      (maze_deadend_candidate_confirm_frames_ >= maze_deadend_confirm_frames_ ? "true" : "false") << ",";
    out << "\"maze_junction_confirm_frames\":" << maze_junction_confirm_frames_ << ",";
    out << "\"maze_corner_confirm_frames\":" << maze_corner_confirm_frames_ << ",";
    out << "\"maze_turn_lock_side\":\"" << (maze_turn_lock_active_ ? direction_name(maze_turn_direction_) : "none") << "\",";
    out << "\"maze_selected_turn_side\":\"" << json_escape(maze_selected_turn_side_) << "\",";
    out << "\"maze_open_side_turn_active\":" << (maze_open_side_turn_active_ ? "true" : "false") << ",";
    out << "\"maze_open_side_turn_side\":\"" << json_escape(maze_open_side_turn_side_) << "\",";
    out << "\"maze_open_side_turn_score\":" << maze_open_side_turn_score_ << ",";
    out << "\"maze_reverse_suppressed_by_open_side\":" <<
      (maze_reverse_suppressed_by_open_side_ ? "true" : "false") << ",";
    out << "\"maze_turn_exit_reason\":\"" << json_escape(maze_turn_exit_reason_) << "\",";
    out << "\"maze_no_reverse_mode\":" << (maze_no_reverse_mode_ ? "true" : "false") << ",";
    out << "\"maze_rotate_search_active\":" << (maze_rotate_search_active_ ? "true" : "false") << ",";
    out << "\"maze_rotate_search_direction\":\"" <<
      (std::abs(maze_rotate_search_direction_) > 0.1 ? direction_name(maze_rotate_search_direction_) : "none") << "\",";
    out << "\"maze_rotate_search_reason\":\"" << json_escape(maze_rotate_search_reason_) << "\",";
    out << "\"maze_rotate_search_exit_reason\":\"" << json_escape(maze_rotate_search_exit_reason_) << "\",";
    out << "\"maze_rotate_search_elapsed_s\":" << maze_rotate_search_elapsed_s_ << ",";
    out << "\"maze_rotate_search_switch_count\":" << maze_rotate_search_switch_count_ << ",";
    out << "\"maze_reverse_blocked_by_guard\":" <<
      (maze_reverse_blocked_by_guard_ ? "true" : "false") << ",";
    out << "\"maze_reverse_disabled_reason\":\"" << json_escape(maze_reverse_disabled_reason_) << "\",";
    const double maze_turn_commit_age =
      maze_turn_commit_start_time_.nanoseconds() == 0 ? 0.0 :
        std::max(0.0, (this->now() - maze_turn_commit_start_time_).seconds());
    out << "\"maze_turn_commit_age_s\":" << maze_turn_commit_age << ",";
    out << "\"maze_turn_phase\":\"" << json_escape(maze_turn_phase_) << "\",";
    out << "\"maze_corner_phase\":\"" << json_escape(maze_corner_phase_) << "\",";
    out << "\"maze_corner_turn_mode\":\"" << json_escape(maze_corner_turn_mode_) << "\",";
    out << "\"maze_corner_release_confirm_frames\":" << maze_corner_release_confirm_frames_ << ",";
    out << "\"maze_corner_abort_confirm_frames\":" << maze_corner_abort_confirm_frames_ << ",";
    out << "\"maze_corner_preclear_attempts\":" << maze_corner_preclear_attempts_ << ",";
    out << "\"maze_corner_preclear_total_distance\":" << maze_corner_preclear_total_distance_ << ",";
    out << "\"maze_corner_turn_latched\":" << (maze_corner_turn_latched_ ? "true" : "false") << ",";
    out << "\"maze_corner_latch_reason\":\"" << json_escape(maze_corner_latch_reason_) << "\",";
    out << "\"maze_corner_turn_state\":\"" << json_escape(maze_corner_turn_state_) << "\",";
    out << "\"maze_corner_cycle_count\":" << maze_corner_cycle_count_ << ",";
    out << "\"maze_corner_pivot_yaw_delta\":" << maze_corner_pivot_yaw_delta_ << ",";
    out << "\"maze_corner_reverse_distance\":" << maze_corner_reverse_distance_ << ",";
    out << "\"maze_corner_selected_side\":\"" << json_escape(maze_corner_selected_side_) << "\",";
    out << "\"maze_inner_clearance\":" << maze_inner_clearance_ << ",";
    out << "\"maze_swept_clearance\":" << maze_swept_clearance_ << ",";
    out << "\"maze_inner_front_corner_clearance\":" << maze_inner_front_corner_clearance_ << ",";
    out << "\"maze_blocked_by_robot_model\":" << (maze_blocked_by_robot_model_ ? "true" : "false") << ",";
    out << "\"robot_safety_margin\":" << robot_safety_margin_ << ",";
    out << "\"maze_inner_side\":\"" << json_escape(maze_inner_side_) << "\",";
    out << "\"maze_turn_blocked_by_inner_clearance\":" << (maze_turn_blocked_by_inner_clearance_ ? "true" : "false") << ",";
    out << "\"maze_preclear_action\":\"" << json_escape(maze_preclear_action_) << "\",";
    out << "\"maze_corner_reason\":\"" << json_escape(maze_corner_reason_) << "\",";
    out << "\"deadend_turn_phase\":\"" << json_escape(deadend_turn_phase_) << "\",";
    out << "\"deadend_turn_direction\":\"" << json_escape(direction_name(deadend_turn_direction_)) << "\",";
    out << "\"deadend_turn_yaw_delta\":" << deadend_turn_yaw_delta_ << ",";
    out << "\"deadend_front_drive_clear\":" << (deadend_front_drive_clear_ ? "true" : "false") << ",";
    out << "\"deadend_front_drive_clear_confirm_frames\":" << deadend_front_drive_clear_confirm_frames_ << ",";
    out << "\"deadend_turn_search_direction\":\"" << json_escape(direction_name(deadend_turn_direction_)) << "\",";
    out << "\"deadend_turn_switch_count\":" << deadend_turn_switch_count_ << ",";
    out << "\"deadend_turn_exit_reason\":\"" << json_escape(deadend_turn_exit_reason_) << "\",";
    out << "\"deadend_reverse_rear_clearance\":" << deadend_reverse_rear_clearance_ << ",";
    out << "\"maze_reverse_rear_guard_active\":" << (maze_reverse_rear_guard_active_ ? "true" : "false") << ",";
    out << "\"deadend_turn_space_ok\":" << (deadend_turn_space_ok_ ? "true" : "false") << ",";
    out << "\"deadend_reverse_distance\":" << deadend_reverse_distance_ << ",";
    out << "\"deadend_turn_reason\":\"" << json_escape(deadend_turn_reason_) << "\",";
    out << "\"maze_speed_profile\":\"" << json_escape(maze_speed_profile_) << "\",";
    out << "\"maze_entry_reason\":\"" << json_escape(maze_entry_reason_) << "\",";
    out << "\"maze_exit_reason\":\"" << json_escape(maze_exit_reason_) << "\",";
    out << "\"corridor_width_estimate\":" << corridor_width_estimate_ << ",";
    out << "\"yaw_error\":" << yaw_error_ << ",";
    out << "\"raw_angular_z\":" << raw_angular_z_ << ",";
    out << "\"limited_angular_z\":" << limited_angular_z_ << ",";
    out << "\"open_side\":\"" << json_escape(chosen_open_side_ == "none" ? direction_name(recovery_direction_) : chosen_open_side_) << "\",";
    out << "\"open_side_score\":" << open_side_score_ << ",";
    out << "\"emergency_escape_active\":" << (emergency_escape_active_ ? "true" : "false") << ",";
    out << "\"escape_phase\":\"" << json_escape(escape_phase_) << "\",";
    out << "\"escape_action\":\"" << json_escape(escape_action_) << "\",";
    out << "\"chosen_open_side\":\"" << json_escape(chosen_open_side_) << "\",";
    out << "\"escape_attempts\":" << escape_attempts_ << ",";
    out << "\"cmd_reason\":\"" << json_escape(last_cmd_reason_) << "\",";
    out << "\"rectified_target_type\":\"" << json_escape(target_type_) << "\",";
    out << "\"local_target_type\":\"" << json_escape(local_target_type_) << "\",";
    out << "\"failed_goal_count\":" << failed_goals_.size() << ",";
    out << "\"laser_offset\":{\"x\":-0.130,\"y\":0.000,\"z\":0.120},";
    out << "\"target\":{\"x\":" << target_.x << ",\"y\":" << target_.y << "},";
    out << "\"cmd_topic\":\"" << json_escape(cmd_vel_topic_) << "\"";
    out << "}";
    msg.data = out.str();
    status_pub_->publish(msg);
  }

  bool is_free_cell(const GridPlan& grid, int idx) const
  {
    if (idx < 0 || idx >= static_cast<int>(grid.data.size())) {
      return false;
    }
    return grid.data[idx] >= 0 && grid.data[idx] <= free_threshold_;
  }

  bool has_unknown_neighbor(const GridPlan& grid, int x, int y) const
  {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = x + dx;
        const int ny = y + dy;
        if (in_bounds(grid, nx, ny) && grid.data[index(grid, nx, ny)] < 0) {
          return true;
        }
      }
    }
    return false;
  }

  bool world_to_index(const GridPlan& grid, double wx, double wy, int& idx) const
  {
    const int mx = static_cast<int>(std::floor((wx - grid.origin_x) / grid.resolution));
    const int my = static_cast<int>(std::floor((wy - grid.origin_y) / grid.resolution));
    if (!in_bounds(grid, mx, my)) {
      return false;
    }
    idx = index(grid, mx, my);
    return true;
  }

  bool find_nearest_free_index(const GridPlan& grid, int seed, double radius_m, int& nearest) const
  {
    int sx = 0;
    int sy = 0;
    index_to_xy(grid, seed, sx, sy);
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(radius_m / std::max(grid.resolution, 0.001))));
    const int radius_sq = radius_cells * radius_cells;
    int best_sq = std::numeric_limits<int>::max();
    nearest = -1;
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int dist_sq = dx * dx + dy * dy;
        if (dist_sq > radius_sq || dist_sq >= best_sq) {
          continue;
        }
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (!in_bounds(grid, nx, ny)) {
          continue;
        }
        const int idx = index(grid, nx, ny);
        if (is_free_cell(grid, idx) && !grid.blocked[idx]) {
          nearest = idx;
          best_sq = dist_sq;
        }
      }
    }
    return nearest >= 0;
  }

  void map_to_world(const GridPlan& grid, int idx, double& wx, double& wy) const
  {
    int x = 0;
    int y = 0;
    index_to_xy(grid, idx, x, y);
    wx = grid.origin_x + (static_cast<double>(x) + 0.5) * grid.resolution;
    wy = grid.origin_y + (static_cast<double>(y) + 0.5) * grid.resolution;
  }

  double heuristic(const GridPlan& grid, int a, int b) const
  {
    int ax = 0;
    int ay = 0;
    int bx = 0;
    int by = 0;
    index_to_xy(grid, a, ax, ay);
    index_to_xy(grid, b, bx, by);
    return std::hypot(static_cast<double>(ax - bx), static_cast<double>(ay - by));
  }

  double distance_to_target(const Pose2D& pose) const
  {
    return std::hypot(target_.x - pose.x, target_.y - pose.y);
  }

  bool in_bounds(const GridPlan& grid, int x, int y) const
  {
    return x >= 0 && y >= 0 && x < grid.width && y < grid.height;
  }

  int index(const GridPlan& grid, int x, int y) const
  {
    return y * grid.width + x;
  }

  void index_to_xy(const GridPlan& grid, int idx, int& x, int& y) const
  {
    y = idx / grid.width;
    x = idx - y * grid.width;
  }

  std::string map_topic_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  std::string map_frame_;
  std::string base_frame_;
  double publish_rate_hz_ = 5.0;
  double replan_interval_ = 3.0;
  double map_timeout_ = 4.0;
  double scan_timeout_ = 1.5;
  double odom_timeout_ = 2.0;
  int free_threshold_ = 25;
  int occupied_threshold_ = 65;
  double obstacle_padding_ = 0.15;
  int min_frontier_cells_ = 8;
  double min_goal_distance_ = 0.45;
  double max_goal_distance_ = 9.0;
  double unknown_complete_ratio_ = 0.025;
  double complete_confirm_duration_ = 8.0;
  double goal_tolerance_ = 0.28;
  double lookahead_distance_ = 0.65;
  double max_linear_speed_ = 0.08;
  double min_linear_speed_ = 0.03;
  double max_angular_speed_ = 0.25;
  double yaw_kp_ = 0.9;
  double rotate_in_place_yaw_ = 0.55;
  double front_stop_distance_ = 0.50;
  double hard_stop_distance_ = 0.20;
  bool disable_hard_stop_in_mapping_ = true;
  bool escape_always_try_motion_ = true;
  double front_emergency_distance_ = 0.28;
  double front_block_distance_ = 0.45;
  double front_clear_distance_ = 0.60;
  int front_block_confirm_frames_ = 3;
  int front_clear_confirm_frames_ = 5;
  bool narrow_creep_enabled_ = true;
  double narrow_creep_speed_ = 0.025;
  double narrow_creep_min_front_p20_ = 0.35;
  double narrow_creep_p50_distance_ = 0.55;
  double angular_deadband_ = 0.035;
  double angular_flip_suppression_yaw_ = 0.25;
  double angular_sign_hold_duration_ = 0.8;
  double angular_command_max_delta_ = 0.10;
  double turn_lock_duration_ = 2.5;
  double turn_switch_margin_ = 0.12;
  double turn_score_ema_alpha_ = 0.35;
  double oscillation_window_ = 5.0;
  int oscillation_flip_limit_ = 3;
  double anti_oscillation_reverse_duration_ = 1.0;
  double anti_oscillation_turn_duration_ = 2.0;
  double target_min_hold_duration_ = 6.0;
  double target_switch_score_margin_ = 0.35;
  double target_switch_distance_threshold_ = 0.55;
  double target_switch_yaw_threshold_ = 0.75;
  bool corner_turn_away_enabled_ = true;
  double corner_front_diagonal_obstacle_p20_ = 0.42;
  double corner_turn_block_margin_ = 0.08;
  double corner_turn_max_toward_obstacle_ = 0.04;
  double corner_turn_away_speed_ = 0.06;
  double corner_creep_speed_ = 0.025;
  bool narrow_maze_follow_enabled_ = true;
  double maze_side_wall_max_p50_ = 0.55;
  double maze_front_min_continue_ = 0.28;
  double maze_front_p20_continue_ = 0.34;
  double maze_forward_speed_ = 0.022;
  double maze_center_kp_ = 0.45;
  double maze_front_center_kp_ = 0.35;
  double maze_side_error_weight_ = 0.55;
  double maze_front_diagonal_error_weight_ = 0.45;
  double maze_center_lookahead_min_delta_ = 0.04;
  double maze_max_angular_ = 0.10;
  double maze_turn_lock_duration_ = 1.8;
  double maze_exit_clear_p20_ = 0.58;
  double maze_inner_clearance_min_p20_ = 0.22;
  double maze_inner_clearance_release_p20_ = 0.28;
  double maze_front_diagonal_clearance_min_p20_ = 0.24;
  double maze_turn_speed_ = 0.07;
  double maze_turn_forward_speed_ = 0.014;
  double maze_pre_turn_outer_bias_duration_ = 0.8;
  double maze_pre_turn_outer_bias_angular_ = 0.05;
  double maze_turn_escape_reverse_duration_ = 0.7;
  double maze_turn_escape_reverse_speed_ = 0.022;
  double maze_min_hold_duration_ = 3.0;
  double maze_center_deadband_ = 0.025;
  double maze_center_ema_alpha_ = 0.30;
  double maze_angular_max_delta_ = 0.05;
  double maze_turn_commit_duration_ = 2.4;
  double maze_turn_max_continuous_duration_ = 2.0;
  double maze_post_turn_straight_duration_ = 0.8;
  double maze_min_turn_radius_ = 0.32;
  double maze_corner_front_delta_ = 0.06;
  bool maze_only_mode_ = false;
  std::string maze_route_mode_ = "local_sector";
  bool maze_topology_mode_ = false;
  double maze_topology_forward_speed_ = 0.055;
  double maze_topology_fast_speed_ = 0.065;
  double maze_topology_turn_speed_ = 0.13;
  double maze_topology_turn_forward_speed_ = 0.035;
  int maze_opening_confirm_frames_ = 4;
  double maze_junction_dwell_duration_ = 0.4;
  double maze_node_merge_distance_ = 0.35;
  double maze_opening_visited_distance_ = 0.45;
  double maze_deadend_front_p20_ = 0.32;
  int maze_deadend_confirm_frames_ = 3;
  double maze_deadend_side_open_p20_ = 0.45;
  bool maze_deadend_turnaround_enabled_ = true;
  double maze_deadend_turn_speed_ = 0.16;
  double maze_deadend_turn_yaw_ = 3.05;
  double maze_deadend_turn_yaw_tolerance_ = 0.18;
  double maze_deadend_turn_timeout_ = 7.0;
  double maze_deadend_in_place_min_side_p20_ = 0.30;
  double maze_deadend_in_place_min_front_p20_ = 0.26;
  double maze_deadend_in_place_min_rear_p20_ = 0.24;
  double maze_deadend_reverse_to_turn_spot_speed_ = 0.035;
  double maze_deadend_reverse_to_turn_spot_distance_ = 0.25;
  double maze_deadend_reverse_to_turn_spot_timeout_ = 3.0;
  double maze_deadend_reverse_chunk_distance_ = 0.20;
  double maze_deadend_reverse_total_max_distance_ = 0.75;
  double maze_reverse_rear_stop_p20_ = 0.26;
  double maze_reverse_rear_release_p20_ = 0.34;
  double maze_deadend_front_drive_clear_p20_ = 0.38;
  double maze_deadend_front_drive_clear_p50_ = 0.45;
  int deadend_front_drive_clear_confirm_frames_required_ = 3;
  double maze_deadend_turn_search_direction_timeout_ = 3.0;
  double maze_deadend_turn_search_max_yaw_ = 2.4;
  int maze_deadend_turn_search_max_switches_ = 2;
  bool maze_open_side_turn_enabled_ = true;
  double maze_open_side_turn_min_score_ = 0.42;
  double maze_open_side_turn_score_margin_ = 0.08;
  double maze_open_side_front_clear_p20_ = 0.34;
  double maze_open_side_front_clear_p50_ = 0.42;
  int maze_open_side_confirm_frames_required_ = 2;
  double maze_open_side_reselect_timeout_ = 2.0;
  bool maze_no_reverse_mode_ = true;
  double maze_rotate_search_speed_ = 0.16;
  double maze_rotate_search_direction_timeout_ = 2.5;
  double maze_rotate_search_total_timeout_ = 8.0;
  int maze_rotate_search_max_switches_ = 1;
  double maze_rotate_front_clear_p20_ = 0.34;
  double maze_rotate_front_clear_p50_ = 0.42;
  int maze_rotate_front_clear_confirm_frames_required_ = 2;
  double maze_graph_front_obstacle_p20_ = 0.34;
  double maze_graph_front_open_p20_ = 0.42;
  double maze_graph_front_open_p50_ = 0.50;
  double maze_graph_opening_score_threshold_ = 0.42;
  double maze_graph_junction_center_advance_ = 0.34;
  double maze_graph_junction_approach_speed_ = 0.035;
  double maze_graph_junction_front_min_ = 0.26;
  double maze_graph_junction_front_p20_ = 0.28;
  double maze_graph_junction_front_p50_ = 0.40;
  double maze_graph_pre_turn_shift_distance_ = 0.04;
  double maze_graph_pre_turn_shift_speed_ = 0.040;
  double maze_graph_pre_turn_shift_angular_ = 0.040;
  double maze_graph_pre_turn_outer_clearance_ = 0.32;
  double maze_graph_pre_turn_shift_timeout_ = 2.0;
  double maze_graph_post_turn_advance_ = 0.18;
  double maze_graph_post_turn_speed_ = 0.045;
  double maze_graph_post_turn_front_p20_ = 0.28;
  double maze_graph_post_turn_front_p50_ = 0.40;
  double maze_graph_post_turn_timeout_ = 8.0;
  double maze_graph_scan_duration_ = 0.35;
  double maze_graph_turn_timeout_ = 4.0;
  double maze_graph_turn_yaw_tolerance_ = 0.28;
  double maze_complete_confirm_duration_ = 8.0;
  double maze_corridor_center_deadband_ = 0.03;
  double wide_maze_front_clear_p20_ = 0.65;
  double wide_maze_side_wall_p20_ = 0.75;
  int maze_state_confirm_frames_ = 3;
  std::string maze_wall_follow_side_ = "right";
  bool maze_use_robot_swept_volume_ = true;
  double robot_length_ = 0.280;
  double robot_width_ = 0.258;
  double robot_safety_margin_ = 0.045;
  double laser_x_offset_ = -0.13;
  double maze_inner_corner_clearance_ = 0.18;
  double maze_swept_clearance_release_ = 0.23;
  double maze_wall_target_clearance_ = 0.24;
  double maze_corner_preclear_duration_ = 0.8;
  double maze_corner_preclear_reverse_speed_ = 0.022;
  double maze_corner_preclear_outer_angular_ = 0.06;
  int maze_corner_release_confirm_frames_required_ = 2;
  double maze_swept_turn_min_commit_duration_ = 1.0;
  double maze_swept_clearance_abort_ = 0.16;
  int maze_corner_preclear_max_attempts_ = 2;
  double maze_corner_preclear_total_max_distance_ = 0.35;
  std::string maze_corner_turn_mode_ = "flow_arc";
  int maze_corner_cycle_limit_ = 3;
  double maze_turn_start_front_p20_ = 0.28;
  double maze_turn_start_front_p50_ = 0.38;
  double maze_reverse_front_min_ = 0.20;
  double maze_reverse_front_p20_ = 0.24;
  double maze_corner_approach_speed_ = 0.045;
  double maze_corner_arc_speed_ = 0.045;
  double maze_corner_arc_angular_ = 0.16;
  double maze_corner_pivot_assist_yaw_ = 0.25;
  double maze_corner_micro_reverse_distance_ = 0.07;
  double maze_corner_reverse_target_distance_ = 0.14;
  double maze_corner_reverse_speed_ = 0.040;
  double maze_corner_pivot_yaw_ = 0.45;
  double maze_corner_pivot_speed_ = 0.18;
  double maze_corner_swing_duration_ = 1.4;
  double maze_corner_swing_forward_speed_ = 0.040;
  double maze_corner_swing_turn_speed_ = 0.14;
  double maze_corner_exit_straight_duration_ = 0.8;
  double maze_corner_retry_backout_distance_ = 0.25;
  double maze_progress_timeout_ = 8.0;
  double progress_timeout_ = 14.0;
  double progress_distance_ = 0.12;
  double recovery_duration_ = 2.2;
  double recovery_turn_speed_ = 0.18;
  double escape_reverse_speed_ = 0.025;
  double min_escape_reverse_speed_ = 0.02;
  double escape_reverse_duration_ = 1.0;
  double escape_turn_duration_ = 2.0;
  double emergency_turn_min_side_p20_ = 0.26;
  double emergency_turn_min_front_diagonal_p20_ = 0.22;
  double emergency_reverse_min_rear_p20_ = 0.22;
  int max_emergency_escape_attempts_ = 8;
  double start_search_radius_ = 0.60;
  bool edge_fallback_enabled_ = true;
  double fallback_obstacle_padding_1_ = 0.10;
  double fallback_obstacle_padding_2_ = 0.05;
  double edge_unknown_radius_ = 0.25;
  double edge_min_goal_distance_ = 0.30;
  double info_gain_weight_ = 1.0;
  double distance_weight_ = 0.35;
  double yaw_weight_ = 0.15;
  double recent_goal_penalty_ = 12.0;
  int max_failed_goals_ = 12;
  int max_no_frontier_recoveries_ = 5;
  bool local_escape_target_enabled_ = true;
  double local_escape_distance_ = 0.35;
  double local_escape_angle_ = 0.65;
  bool enabled_ = true;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex map_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  rclcpp::Time last_map_time_;

  std::mutex scan_mutex_;
  rclcpp::Time last_scan_time_;
  bool has_scan_ = false;
  chassis_controller::ScanSectorSummary latest_scan_summary_;
  double front_p20_ = std::numeric_limits<double>::infinity();
  double front_p50_ = std::numeric_limits<double>::infinity();
  double front_min_ = std::numeric_limits<double>::infinity();
  int front_valid_count_ = 0;
  double front_left_p20_ = std::numeric_limits<double>::infinity();
  double front_right_p20_ = std::numeric_limits<double>::infinity();
  double left_p20_ = std::numeric_limits<double>::infinity();
  double right_p20_ = std::numeric_limits<double>::infinity();
  double left_side_p20_ = std::numeric_limits<double>::infinity();
  double right_side_p20_ = std::numeric_limits<double>::infinity();
  double rear_p20_ = std::numeric_limits<double>::infinity();
  int front_blocked_frames_ = 0;
  int front_clear_frames_ = 0;

  std::mutex odom_mutex_;
  rclcpp::Time last_odom_time_;
  bool has_odom_ = false;

  std::vector<Point2D> current_path_;
  int current_goal_cell_ = -1;
  Point2D target_;
  Pose2D last_progress_pose_;
  Pose2D maze_last_progress_pose_;
  Pose2D deadend_reverse_start_pose_;
  Pose2D deadend_reverse_total_start_pose_;
  Pose2D maze_corner_preclear_start_pose_;
  Pose2D maze_corner_preclear_total_start_pose_;
  Pose2D maze_corner_reverse_start_pose_;
  rclcpp::Time last_plan_time_;
  rclcpp::Time target_selected_time_;
  rclcpp::Time last_progress_time_;
  rclcpp::Time recovery_start_time_;
  rclcpp::Time emergency_escape_start_time_;
  rclcpp::Time complete_candidate_since_;
  double last_goal_distance_ = std::numeric_limits<double>::infinity();
  double recovery_direction_ = 1.0;
  double escape_direction_ = 1.0;
  double turn_lock_direction_ = 1.0;
  double anti_oscillation_direction_ = 1.0;
  double maze_turn_direction_ = 1.0;
  double left_turn_score_ema_ = 0.0;
  double right_turn_score_ema_ = 0.0;
  bool front_obstacle_recovery_ = false;
  bool emergency_escape_active_ = false;
  bool turn_lock_active_ = false;
  bool turn_switch_blocked_ = false;
  bool turn_score_ema_initialized_ = false;
  bool oscillation_detected_ = false;
  bool anti_oscillation_active_ = false;
  double last_angular_command_ = 0.0;
  double yaw_error_ = 0.0;
  double raw_angular_z_ = 0.0;
  double limited_angular_z_ = 0.0;
  double current_target_score_ = -std::numeric_limits<double>::infinity();
  double maze_center_error_ = 0.0;
  double maze_side_center_error_ = 0.0;
  double maze_front_center_error_ = 0.0;
  double maze_smoothed_center_error_ = 0.0;
  double corridor_width_estimate_ = 0.0;
  double maze_inner_clearance_ = std::numeric_limits<double>::infinity();
  double maze_swept_clearance_ = std::numeric_limits<double>::infinity();
  double maze_inner_front_corner_clearance_ = std::numeric_limits<double>::infinity();
  double maze_last_angular_command_ = 0.0;
  double maze_last_unknown_ratio_ = 1.0;
  double maze_open_side_turn_score_ = 0.0;
  double deadend_turn_direction_ = 1.0;
  double last_deadend_turn_direction_ = 0.0;
  double deadend_turn_start_yaw_ = 0.0;
  double deadend_turn_yaw_delta_ = 0.0;
  double deadend_reverse_distance_ = 0.0;
  double deadend_reverse_chunk_progress_ = 0.0;
  double deadend_reverse_rear_clearance_ = std::numeric_limits<double>::infinity();
  double maze_corner_latched_direction_ = 0.0;
  double maze_corner_preclear_total_distance_ = 0.0;
  double maze_corner_pivot_start_yaw_ = 0.0;
  double maze_corner_pivot_yaw_delta_ = 0.0;
  double maze_corner_reverse_distance_ = 0.0;
  double left_open_score_ = 0.0;
  double front_open_score_ = 0.0;
  double right_open_score_ = 0.0;
  double maze_rotate_search_direction_ = 0.0;
  double maze_rotate_search_elapsed_s_ = 0.0;
  double graph_turn_direction_ = 1.0;
  double graph_turn_start_yaw_ = 0.0;
  double graph_turn_target_yaw_ = 0.0;
  double graph_left_score_ = 0.0;
  double graph_front_score_ = 0.0;
  double graph_right_score_ = 0.0;
  double graph_junction_advance_distance_ = 0.0;
  double graph_post_turn_advance_distance_ = 0.0;
  double graph_pre_turn_shift_distance_ = 0.0;
  double graph_pre_turn_shift_outer_clearance_ = 0.0;
  bool graph_junction_front_guard_active_ = false;
  bool graph_pre_turn_shift_skipped_ = false;
  int last_angular_sign_ = 0;
  int oscillation_last_sign_ = 0;
  int oscillation_flip_count_ = 0;
  int maze_left_wall_confirm_frames_ = 0;
  int maze_right_wall_confirm_frames_ = 0;
  int maze_any_wall_confirm_frames_ = 0;
  int maze_both_walls_confirm_frames_ = 0;
  int maze_front_blocked_confirm_frames_ = 0;
  int maze_front_open_confirm_frames_ = 0;
  int maze_deadend_candidate_confirm_frames_ = 0;
  int maze_junction_confirm_frames_ = 0;
  int maze_corner_confirm_frames_ = 0;
  int maze_corner_release_confirm_frames_ = 0;
  int maze_corner_abort_confirm_frames_ = 0;
  int maze_corner_preclear_attempts_ = 0;
  int maze_corner_cycle_count_ = 0;
  int deadend_front_drive_clear_confirm_frames_ = 0;
  int deadend_turn_switch_count_ = 0;
  int maze_open_side_confirm_frames_ = 0;
  int maze_rotate_front_clear_confirm_frames_ = 0;
  int maze_rotate_search_switch_count_ = 0;
  int graph_selected_opening_index_ = -1;
  rclcpp::Time last_angular_sign_change_time_;
  rclcpp::Time turn_lock_start_time_;
  rclcpp::Time anti_oscillation_start_time_;
  rclcpp::Time maze_turn_lock_start_time_;
  rclcpp::Time maze_turn_phase_start_time_;
  rclcpp::Time maze_follow_start_time_;
  rclcpp::Time maze_turn_commit_start_time_;
  rclcpp::Time deadend_turn_phase_start_time_;
  rclcpp::Time maze_corner_state_start_time_;
  rclcpp::Time maze_last_progress_time_;
  rclcpp::Time maze_rotate_search_start_time_;
  rclcpp::Time maze_rotate_search_direction_start_time_;
  rclcpp::Time graph_state_start_time_;
  std::vector<rclcpp::Time> angular_flip_times_;
  std::vector<int> failed_goals_;

  std::string state_ = "starting";
  std::string message_ = "waiting for inputs";
  std::string current_obstacle_state_ = "unknown";
  std::string last_cmd_reason_ = "none";
  std::string escape_phase_ = "none";
  std::string escape_action_ = "none";
  std::string chosen_open_side_ = "none";
  std::string target_type_ = "none";
  std::string local_target_type_ = "none";
  std::string replan_reason_ = "starting";
  std::string target_switch_reason_ = "none";
  std::string corner_guard_reason_ = "none";
  std::string maze_action_ = "none";
  std::string maze_turn_phase_ = "none";
  std::string maze_corner_phase_ = "none";
  std::string maze_inner_side_ = "none";
  std::string maze_preclear_action_ = "none";
  std::string maze_corner_reason_ = "none";
  std::string maze_selected_turn_side_ = "none";
  std::string maze_speed_profile_ = "default";
  std::string maze_entry_reason_ = "none";
  std::string maze_exit_reason_ = "none";
  std::string maze_route_state_ = "drive_open";
  std::string maze_route_reason_ = "starting";
  std::string graph_explore_state_ = "none";
  std::string topology_selected_opening_ = "none";
  std::string maze_open_side_turn_side_ = "none";
  std::string maze_turn_exit_reason_ = "none";
  std::string maze_rotate_search_reason_ = "none";
  std::string maze_rotate_search_exit_reason_ = "none";
  std::string maze_reverse_disabled_reason_ = "none";
  std::string deadend_turn_phase_ = "none";
  std::string deadend_turn_reason_ = "none";
  std::string deadend_turn_exit_reason_ = "none";
  std::string maze_corner_latch_reason_ = "none";
  std::string maze_corner_turn_state_ = "none";
  std::string maze_corner_selected_side_ = "none";
  std::string last_reached_target_type_ = "none";
  int frontier_count_ = 0;
  int reachable_frontier_count_ = 0;
  int reachable_edge_count_ = 0;
  int path_points_ = 0;
  int failure_count_ = 0;
  int no_frontier_recovery_count_ = 0;
  int next_topology_node_id_ = 1;
  int current_topology_node_id_ = -1;
  int active_topology_node_id_ = -1;
  int active_topology_opening_index_ = -1;
  int topology_unvisited_opening_count_ = 0;
  Pose2D active_topology_opening_pose_;
  Pose2D graph_junction_entry_pose_;
  Pose2D graph_post_turn_start_pose_;
  Pose2D graph_pre_turn_shift_start_pose_;
  std::vector<MazeTopologyNode> maze_topology_nodes_;
  std::vector<int> maze_backtrack_stack_;
  int escape_attempts_ = 0;
  int rejected_too_close_targets_ = 0;
  double last_reached_goal_distance_ = -1.0;
  double open_side_score_ = 0.0;
  double unknown_ratio_ = 1.0;
  double current_planning_padding_ = 0.15;
  bool using_edge_fallback_ = false;
  bool target_switch_blocked_ = false;
  bool corner_guard_active_ = false;
  bool maze_follow_active_ = false;
  bool maze_priority_active_ = false;
  bool maze_turn_lock_active_ = false;
  bool maze_turn_commit_active_ = false;
  bool maze_center_ema_initialized_ = false;
  bool maze_turn_blocked_by_inner_clearance_ = false;
  bool maze_blocked_by_robot_model_ = false;
  bool maze_corner_turn_latched_ = false;
  bool deadend_turn_active_ = false;
  bool deadend_turn_space_ok_ = false;
  bool deadend_front_drive_clear_ = false;
  bool maze_reverse_rear_guard_active_ = false;
  bool maze_open_side_turn_active_ = false;
  bool maze_reverse_suppressed_by_open_side_ = false;
  bool maze_rotate_search_active_ = false;
  bool maze_reverse_blocked_by_guard_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
