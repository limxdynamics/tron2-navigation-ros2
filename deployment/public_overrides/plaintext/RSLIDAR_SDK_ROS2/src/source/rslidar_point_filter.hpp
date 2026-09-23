/*********************************************************************************************************************
Near-field point-cloud filter embedded in the native ROS 2 RoboSense destination.

The filter preserves the SDK point type directly, including ring and per-point timestamp for XYZIRT clouds. Its
semantics match the established MROS mapping chain:
  1) minimum range;
  2) near-field low intensity;
  3) optional robot self box;
  4) optional azimuth masks;
  5) near-field radius-neighbor rejection.
*********************************************************************************************************************/
#pragma once

#include "msg/rs_msg/lidar_point_cloud_msg.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace robosense
{
namespace lidar
{

class RslidarPointFilter
{
public:
  struct Stats
  {
    std::size_t input = 0;
    std::size_t after_stage1 = 0;
    std::size_t output = 0;
    std::size_t removed = 0;
    double stage1_ms = 0.0;
    double radius_ms = 0.0;
    double total_ms = 0.0;
  };

  void configure(const YAML::Node& node)
  {
    if (!node || !node.IsMap())
    {
      return;
    }

    enabled_ = getOr<bool>(node, "enable", false);
    if (!enabled_)
    {
      return;
    }

    min_range_ = getOr<double>(node, "min_range", 0.3);

    if (const YAML::Node intensity = node["intensity_filter"])
    {
      intensity_filter_enable_ = getOr<bool>(intensity, "enable", false);
      min_intensity_ = getOr<double>(intensity, "min_intensity", 15.0);
      intensity_apply_range_ = getOr<double>(intensity, "apply_range", 2.0);
    }

    if (const YAML::Node self = node["self_filter"])
    {
      self_filter_enable_ = getOr<bool>(self, "enable", false);
      if (self_filter_enable_ && !loadVec3(self["min"], self["max"]))
      {
        self_filter_enable_ = false;
      }
    }

    angle_mask_ranges_.clear();
    if (const YAML::Node angle = node["angle_mask"])
    {
      angle_mask_enable_ = getOr<bool>(angle, "enable", false);
      loadAngleMask(angle["ranges"]);
      if (angle_mask_ranges_.empty())
      {
        angle_mask_enable_ = false;
      }
    }

    if (const YAML::Node radius = node["radius_filter"])
    {
      radius_filter_enable_ = getOr<bool>(radius, "enable", true);
      radius_apply_range_ = getOr<double>(radius, "apply_range", 3.0);
      search_radius_ = getOr<double>(radius, "search_radius", 0.5);
      min_neighbors_ = getOr<int>(radius, "min_neighbors", 4);
      omp_threads_ = getOr<int>(radius, "omp_threads", 4);
    }

    log_throttle_sec_ = getOr<double>(node, "log_throttle_sec", 5.0);
    if (const YAML::Node debug = node["debug"])
    {
      debug_enable_ = getOr<bool>(debug, "enable", false);
      debug_print_throttle_sec_ = getOr<double>(debug, "print_throttle_sec", 5.0);
    }

    min_range_ = std::max(0.0, min_range_);
    min_intensity_ = std::max(0.0, min_intensity_);
    intensity_apply_range_ = std::max(0.0, intensity_apply_range_);
    radius_apply_range_ = std::max(0.0, radius_apply_range_);
    search_radius_ = std::max(1e-3, search_radius_);
    min_neighbors_ = std::max(0, min_neighbors_);
    log_throttle_sec_ = std::max(0.1, log_throttle_sec_);
    debug_print_throttle_sec_ = std::max(0.1, debug_print_throttle_sec_);
    omp_threads_ = std::max(0, omp_threads_);
#ifdef _OPENMP
    omp_threads_effective_ = omp_threads_ > 0 ? omp_threads_ : omp_get_max_threads();
#else
    omp_threads_effective_ = 1;
#endif
    omp_threads_effective_ = std::max(1, omp_threads_effective_);
  }

  bool enabled() const
  {
    return enabled_;
  }

  bool debugEnabled() const
  {
    return debug_enable_;
  }

  double logThrottleSec() const
  {
    return log_throttle_sec_;
  }

  double debugPrintThrottleSec() const
  {
    return debug_print_throttle_sec_;
  }

  std::string description() const
  {
    std::ostringstream out;
    out << "min_range=" << min_range_ << "m"
        << ", intensity=" << (intensity_filter_enable_ ? "on" : "off")
        << "[min=" << min_intensity_ << ",apply=" << intensity_apply_range_ << "m]"
        << ", self=" << (self_filter_enable_ ? "on" : "off")
        << ", angle_mask=" << (angle_mask_enable_ ? "on" : "off")
        << "(" << angle_mask_ranges_.size() << ")"
        << ", radius=" << (radius_filter_enable_ ? "on" : "off")
        << "[apply=" << radius_apply_range_ << "m,r=" << search_radius_
        << "m,min_neighbors=" << min_neighbors_ << ",threads=" << omp_threads_effective_ << "]";
    return out.str();
  }

  void apply(const LidarPointCloudMsg& input,
             LidarPointCloudMsg& output,
             LidarPointCloudMsg* removed,
             Stats* stats = nullptr) const
  {
    using Clock = std::chrono::steady_clock;
    const auto started = Clock::now();

    if (!enabled_)
    {
      output = input;
      if (removed != nullptr)
      {
        copyMetadata(input, *removed);
      }
      return;
    }

    LidarPointCloudMsg::VectorT stage1;
    LidarPointCloudMsg::VectorT rejected;
    stage1.reserve(input.points.size());
    if (removed != nullptr)
    {
      rejected.reserve(input.points.size() / 16U + 1U);
    }

    const double min_range_squared = min_range_ * min_range_;
    const double intensity_range_squared = intensity_apply_range_ * intensity_apply_range_;
    for (const auto& point : input.points)
    {
      if (!isFinite(point))
      {
        continue;
      }

      const double range_squared = squaredRange(point);
      bool reject = range_squared < min_range_squared;
      reject = reject || (intensity_filter_enable_ &&
                          range_squared < intensity_range_squared &&
                          static_cast<double>(point.intensity) < min_intensity_);
      reject = reject || (self_filter_enable_ && inSelfBox(point));
      reject = reject || (angle_mask_enable_ && isAngleMasked(azimuthDegrees(point)));

      if (reject)
      {
        if (removed != nullptr)
        {
          rejected.push_back(point);
        }
      }
      else
      {
        stage1.push_back(point);
      }
    }
    const auto stage1_finished = Clock::now();

    std::vector<unsigned char> keep(stage1.size(), 1U);
    if (radius_filter_enable_ && min_neighbors_ > 0 && !stage1.empty())
    {
      Grid grid;
      grid.reserve(stage1.size() / 4U + 1U);
      for (std::size_t i = 0; i < stage1.size(); ++i)
      {
        grid[cellFor(stage1[i])].push_back(i);
      }
      const Grid& lookup = grid;

      const double apply_range_squared = radius_apply_range_ * radius_apply_range_;
      const double search_radius_squared = search_radius_ * search_radius_;
      const std::size_t required_points = static_cast<std::size_t>(min_neighbors_) + 1U;
      const std::int64_t point_count = static_cast<std::int64_t>(stage1.size());

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic, 512) num_threads(omp_threads_effective_)
#endif
      for (std::int64_t raw_index = 0; raw_index < point_count; ++raw_index)
      {
        const std::size_t index = static_cast<std::size_t>(raw_index);
        const auto& query = stage1[index];
        if (squaredRange(query) > apply_range_squared)
        {
          continue;
        }

        const GridKey center = cellFor(query);
        std::size_t neighbors = 0;
        bool enough = false;
        for (int dx = -1; dx <= 1 && !enough; ++dx)
        {
          for (int dy = -1; dy <= 1 && !enough; ++dy)
          {
            for (int dz = -1; dz <= 1 && !enough; ++dz)
            {
              const GridKey cell{center.x + dx, center.y + dy, center.z + dz};
              const auto bucket = lookup.find(cell);
              if (bucket == lookup.end())
              {
                continue;
              }
              for (const std::size_t candidate_index : bucket->second)
              {
                if (squaredDistance(query, stage1[candidate_index]) <= search_radius_squared)
                {
                  ++neighbors;
                  if (neighbors >= required_points)
                  {
                    enough = true;
                    break;
                  }
                }
              }
            }
          }
        }
        if (!enough)
        {
          keep[index] = 0U;
        }
      }
    }

    copyMetadata(input, output);
    output.points.reserve(stage1.size());
    for (std::size_t i = 0; i < stage1.size(); ++i)
    {
      if (keep[i] != 0U)
      {
        output.points.push_back(stage1[i]);
      }
      else if (removed != nullptr)
      {
        rejected.push_back(stage1[i]);
      }
    }
    finalizeUnorganized(output);

    if (removed != nullptr)
    {
      copyMetadata(input, *removed);
      removed->points = std::move(rejected);
      finalizeUnorganized(*removed);
    }

    const auto radius_finished = Clock::now();
    if (stats != nullptr)
    {
      stats->input = input.points.size();
      stats->after_stage1 = stage1.size();
      stats->output = output.points.size();
      stats->removed = removed != nullptr ? removed->points.size() : input.points.size() - output.points.size();
      stats->stage1_ms = elapsedMs(started, stage1_finished);
      stats->radius_ms = elapsedMs(stage1_finished, radius_finished);
      stats->total_ms = elapsedMs(started, radius_finished);
    }
  }

private:
  struct GridKey
  {
    int x;
    int y;
    int z;

    bool operator==(const GridKey& other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct GridKeyHash
  {
    std::size_t operator()(const GridKey& key) const
    {
      std::size_t seed = std::hash<int>{}(key.x);
      seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      seed ^= std::hash<int>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
      return seed;
    }
  };

  using Grid = std::unordered_map<GridKey, std::vector<std::size_t>, GridKeyHash>;

  template<typename T>
  static T getOr(const YAML::Node& node, const std::string& key, const T& fallback)
  {
    try
    {
      if (node[key] && node[key].Type() != YAML::NodeType::Null)
      {
        return node[key].as<T>();
      }
    }
    catch (...)
    {
    }
    return fallback;
  }

  bool loadVec3(const YAML::Node& minimum, const YAML::Node& maximum)
  {
    try
    {
      if (!minimum || !minimum.IsSequence() || minimum.size() < 3 ||
          !maximum || !maximum.IsSequence() || maximum.size() < 3)
      {
        return false;
      }
      for (std::size_t i = 0; i < 3; ++i)
      {
        const double first = minimum[i].as<double>();
        const double second = maximum[i].as<double>();
        self_box_min_[i] = std::min(first, second);
        self_box_max_[i] = std::max(first, second);
      }
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  void loadAngleMask(const YAML::Node& ranges)
  {
    if (!ranges || !ranges.IsSequence())
    {
      return;
    }
    try
    {
      for (std::size_t i = 0; i < ranges.size(); ++i)
      {
        const YAML::Node range = ranges[i];
        if (!range || !range.IsSequence() || range.size() < 2)
        {
          continue;
        }
        const double minimum = normalizeDegrees(range[0].as<double>());
        const double maximum = normalizeDegrees(range[1].as<double>());
        angle_mask_ranges_.emplace_back(minimum, maximum);
      }
    }
    catch (...)
    {
      angle_mask_ranges_.clear();
    }
  }

  static double normalizeDegrees(double degrees)
  {
    return std::fmod(std::fmod(degrees, 360.0) + 360.0, 360.0);
  }

  template<typename PointT>
  static bool isFinite(const PointT& point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
  }

  template<typename PointT>
  static double squaredRange(const PointT& point)
  {
    return static_cast<double>(point.x) * point.x +
           static_cast<double>(point.y) * point.y +
           static_cast<double>(point.z) * point.z;
  }

  template<typename FirstPointT, typename SecondPointT>
  static double squaredDistance(const FirstPointT& first, const SecondPointT& second)
  {
    const double dx = static_cast<double>(first.x) - second.x;
    const double dy = static_cast<double>(first.y) - second.y;
    const double dz = static_cast<double>(first.z) - second.z;
    return dx * dx + dy * dy + dz * dz;
  }

  template<typename PointT>
  double azimuthDegrees(const PointT& point) const
  {
    constexpr double radians_to_degrees = 57.2957795130823208768;
    double angle = std::atan2(static_cast<double>(point.y), static_cast<double>(point.x)) *
                   radians_to_degrees;
    if (angle < 0.0)
    {
      angle += 360.0;
    }
    return angle;
  }

  bool isAngleMasked(double azimuth) const
  {
    for (const auto& range : angle_mask_ranges_)
    {
      if (range.first <= range.second)
      {
        if (azimuth >= range.first && azimuth <= range.second)
        {
          return true;
        }
      }
      else if (azimuth >= range.first || azimuth <= range.second)
      {
        return true;
      }
    }
    return false;
  }

  template<typename PointT>
  bool inSelfBox(const PointT& point) const
  {
    return point.x >= self_box_min_[0] && point.x <= self_box_max_[0] &&
           point.y >= self_box_min_[1] && point.y <= self_box_max_[1] &&
           point.z >= self_box_min_[2] && point.z <= self_box_max_[2];
  }

  template<typename PointT>
  GridKey cellFor(const PointT& point) const
  {
    return GridKey{
      static_cast<int>(std::floor(static_cast<double>(point.x) / search_radius_)),
      static_cast<int>(std::floor(static_cast<double>(point.y) / search_radius_)),
      static_cast<int>(std::floor(static_cast<double>(point.z) / search_radius_))};
  }

  static void copyMetadata(const LidarPointCloudMsg& source, LidarPointCloudMsg& destination)
  {
    destination.height = 1;
    destination.width = 0;
    destination.is_dense = true;
    destination.timestamp = source.timestamp;
    destination.seq = source.seq;
    destination.frame_id = source.frame_id;
    destination.points.clear();
  }

  static void finalizeUnorganized(LidarPointCloudMsg& cloud)
  {
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.is_dense = true;
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point& start,
                          const std::chrono::steady_clock::time_point& end)
  {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  bool enabled_ = false;
  double min_range_ = 0.3;

  bool intensity_filter_enable_ = false;
  double min_intensity_ = 15.0;
  double intensity_apply_range_ = 2.0;

  bool self_filter_enable_ = false;
  double self_box_min_[3] = {0.0, 0.0, 0.0};
  double self_box_max_[3] = {0.0, 0.0, 0.0};

  bool angle_mask_enable_ = false;
  std::vector<std::pair<double, double>> angle_mask_ranges_;

  bool radius_filter_enable_ = true;
  double radius_apply_range_ = 3.0;
  double search_radius_ = 0.5;
  int min_neighbors_ = 4;
  int omp_threads_ = 4;
  int omp_threads_effective_ = 1;

  double log_throttle_sec_ = 5.0;
  bool debug_enable_ = false;
  double debug_print_throttle_sec_ = 5.0;
};

}  // namespace lidar
}  // namespace robosense
