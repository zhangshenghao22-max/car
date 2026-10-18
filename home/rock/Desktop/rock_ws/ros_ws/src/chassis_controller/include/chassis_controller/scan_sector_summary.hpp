#pragma once

#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace chassis_controller
{

struct SectorStats
{
  double min = std::numeric_limits<double>::infinity();
  double p10 = std::numeric_limits<double>::infinity();
  double p20 = std::numeric_limits<double>::infinity();
  double p50 = std::numeric_limits<double>::infinity();
  double max = std::numeric_limits<double>::infinity();
  int valid_count = 0;
};

struct ScanSectorSummary
{
  SectorStats front;
  SectorStats front_left;
  SectorStats front_right;
  SectorStats left;
  SectorStats right;
  SectorStats left_side;
  SectorStats right_side;
  SectorStats rear;
  double max_usable_range = std::numeric_limits<double>::infinity();
};

inline double percentile_from_sorted(const std::vector<double>& values, double ratio)
{
  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double clamped = std::min(1.0, std::max(0.0, ratio));
  const auto index = static_cast<size_t>(
    std::floor(clamped * static_cast<double>(values.size() - 1)));
  return values[std::min(index, values.size() - 1)];
}

inline SectorStats make_sector_stats(std::vector<double>& values, double fallback_range)
{
  SectorStats stats;
  stats.valid_count = static_cast<int>(values.size());
  if (values.empty()) {
    stats.min = fallback_range;
    stats.p10 = fallback_range;
    stats.p20 = fallback_range;
    stats.p50 = fallback_range;
    stats.max = fallback_range;
    return stats;
  }

  std::sort(values.begin(), values.end());
  stats.min = values.front();
  stats.p10 = percentile_from_sorted(values, 0.10);
  stats.p20 = percentile_from_sorted(values, 0.20);
  stats.p50 = percentile_from_sorted(values, 0.50);
  stats.max = values.back();
  return stats;
}

inline ScanSectorSummary summarize_scan_sectors(
  const sensor_msgs::msg::LaserScan& scan,
  double max_usable_range,
  double front_half_angle = 0.5235987755982988,
  double side_angle = 1.8325957145940461)
{
  const double diagonal_angle = std::min(side_angle, std::max(front_half_angle, 1.0471975511965976));
  std::vector<double> front_ranges;
  std::vector<double> front_left_ranges;
  std::vector<double> front_right_ranges;
  std::vector<double> left_ranges;
  std::vector<double> right_ranges;
  std::vector<double> left_side_ranges;
  std::vector<double> right_side_ranges;
  std::vector<double> rear_ranges;
  front_ranges.reserve(256);
  front_left_ranges.reserve(256);
  front_right_ranges.reserve(256);
  left_ranges.reserve(256);
  right_ranges.reserve(256);
  left_side_ranges.reserve(256);
  right_side_ranges.reserve(256);
  rear_ranges.reserve(256);

  const double range_max = std::isfinite(scan.range_max) ?
    static_cast<double>(scan.range_max) : max_usable_range;
  const double usable_range = std::min(range_max, max_usable_range);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > usable_range) {
      continue;
    }

    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    if (std::abs(angle) <= front_half_angle) {
      front_ranges.push_back(range);
    }
    if (angle >= 0.0 && angle <= diagonal_angle) {
      front_left_ranges.push_back(range);
    }
    if (angle <= 0.0 && angle >= -diagonal_angle) {
      front_right_ranges.push_back(range);
    }
    if (angle > front_half_angle && angle <= side_angle) {
      left_ranges.push_back(range);
    } else if (angle < -front_half_angle && angle >= -side_angle) {
      right_ranges.push_back(range);
    }
    if (angle > diagonal_angle && angle <= side_angle) {
      left_side_ranges.push_back(range);
    } else if (angle < -diagonal_angle && angle >= -side_angle) {
      right_side_ranges.push_back(range);
    }
    if (std::abs(angle) > side_angle) {
      rear_ranges.push_back(range);
    }
  }

  ScanSectorSummary summary;
  summary.max_usable_range = usable_range;
  summary.front = make_sector_stats(front_ranges, usable_range);
  summary.front_left = make_sector_stats(front_left_ranges, usable_range);
  summary.front_right = make_sector_stats(front_right_ranges, usable_range);
  summary.left = make_sector_stats(left_ranges, usable_range);
  summary.right = make_sector_stats(right_ranges, usable_range);
  summary.left_side = make_sector_stats(left_side_ranges, usable_range);
  summary.right_side = make_sector_stats(right_side_ranges, usable_range);
  summary.rear = make_sector_stats(rear_ranges, usable_range);
  return summary;
}

}  // namespace chassis_controller
