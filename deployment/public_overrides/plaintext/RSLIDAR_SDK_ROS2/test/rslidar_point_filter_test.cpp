#include "source/rslidar_point_filter.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <iostream>

namespace
{

PointXYZIRT makePoint(float x, float y, float z, std::uint8_t intensity,
                      std::uint16_t ring, double timestamp)
{
  PointXYZIRT point{};
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  point.ring = ring;
  point.timestamp = timestamp;
  return point;
}

bool expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

}  // namespace

int main()
{
  const YAML::Node config = YAML::Load(R"(
enable: true
min_range: 0.1
intensity_filter:
  enable: true
  min_intensity: 10.0
  apply_range: 1.0
self_filter:
  enable: false
radius_filter:
  enable: true
  apply_range: 1.0
  search_radius: 0.3
  min_neighbors: 80
  omp_threads: 4
angle_mask:
  enable: false
log_throttle_sec: 5.0
)");

  robosense::lidar::RslidarPointFilter filter;
  filter.configure(config);

  LidarPointCloudMsg input;
  input.height = 5;
  input.width = 17;
  input.is_dense = true;
  input.timestamp = 1234.5;
  input.seq = 42;
  input.frame_id = "rslidar";

  // 81 mutually adjacent points satisfy min_neighbors=80 (the search also sees the query point itself).
  for (std::uint16_t i = 0; i < 81; ++i)
  {
    const float x = 0.5F + static_cast<float>(i % 9) * 0.002F;
    const float y = static_cast<float>(i / 9) * 0.002F;
    input.points.push_back(makePoint(x, y, 0.0F, 20, i, 1000.0 + i * 0.001));
  }

  input.points.push_back(makePoint(0.5F, 0.0F, 0.0F, 5, 100, 1001.0));   // low intensity
  input.points.push_back(makePoint(0.0F, 0.8F, 0.0F, 20, 101, 1001.1));  // isolated near field
  input.points.push_back(makePoint(2.0F, 0.0F, 0.0F, 1, 102, 1001.2));   // outside filter range
  input.points.push_back(makePoint(0.05F, 0.0F, 0.0F, 20, 103, 1001.3)); // below min range

  LidarPointCloudMsg output;
  LidarPointCloudMsg removed;
  robosense::lidar::RslidarPointFilter::Stats stats;
  filter.apply(input, output, &removed, &stats);

  bool ok = true;
  ok &= expect(filter.enabled(), "filter must be enabled");
  ok &= expect(stats.input == 85, "input count");
  ok &= expect(stats.after_stage1 == 83, "stage-1 count");
  ok &= expect(stats.output == 82, "output count");
  ok &= expect(stats.removed == 3, "removed count");
  ok &= expect(output.height == 1 && output.width == 82, "filtered cloud must be unorganized");
  ok &= expect(removed.height == 1 && removed.width == 3, "removed cloud dimensions");
  ok &= expect(output.seq == input.seq && output.timestamp == input.timestamp && output.frame_id == input.frame_id,
               "cloud metadata must be preserved");
  ok &= expect(output.points.back().ring == 102, "ring field must be preserved");
  ok &= expect(std::abs(output.points.back().timestamp - 1001.2) < 1e-9,
               "per-point timestamp must be preserved");
  ok &= expect(output.points.back().intensity == 1, "points beyond 1 m must not be intensity-filtered");

  return ok ? 0 : 1;
}
