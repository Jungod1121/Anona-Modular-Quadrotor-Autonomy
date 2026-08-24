#pragma once
// Minimal stub replacing ros pcl_conversions for Path E plant build.
//
// NOTE: this is NOT a full pcl_conversions replacement. It converts only the
// field subset the `mighty` package actually uses:
//   - fromROSMsg: x/y/z of every point (other fields dropped by design)
//   - toROSMsg:   full support for pcl::PointXYZI (the only type converted),
//                 xyz-only fallback for any other point type
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace pcl
{

template <typename PointT>
inline void fromROSMsg(const sensor_msgs::msg::PointCloud2 & msg,
  pcl::PointCloud<PointT> & cloud)
{
  cloud.clear();
  const bool has_xyz =
    std::any_of(msg.fields.begin(), msg.fields.end(),
      [](const sensor_msgs::msg::PointField & f) {return f.name == "x";}) &&
    std::any_of(msg.fields.begin(), msg.fields.end(),
      [](const sensor_msgs::msg::PointField & f) {return f.name == "y";}) &&
    std::any_of(msg.fields.begin(), msg.fields.end(),
      [](const sensor_msgs::msg::PointField & f) {return f.name == "z";});
  if (!has_xyz || msg.data.empty()) {
    return;
  }
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      PointT pt;
      pt.x = *iter_x;
      pt.y = *iter_y;
      pt.z = *iter_z;
      cloud.push_back(pt);
    }
  } catch (...) {
    // Malformed field layout: keep whatever was parsed so far instead of
    // silently discarding everything.
  }
}

// Full-fidelity conversion for the XYZI clouds published as ground maps.
inline void toROSMsg(const pcl::PointCloud<pcl::PointXYZI> & cloud,
  sensor_msgs::msg::PointCloud2 & msg)
{
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_bigendian = false;
  msg.is_dense = cloud.is_dense;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 5, sensor_msgs::msg::PointField::FLOAT32,
    "z", 9, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 13, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(static_cast<size_t>(cloud.size()));
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  for (const auto & pt : cloud.points) {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    *iter_i = pt.intensity;
    ++iter_x; ++iter_y; ++iter_z; ++iter_i;
  }
}

// Fallback for other point types: xyz only.
template <typename PointT>
inline void toROSMsg(const pcl::PointCloud<PointT> & cloud,
  sensor_msgs::msg::PointCloud2 & msg)
{
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_bigendian = false;
  msg.is_dense = cloud.is_dense;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 5, sensor_msgs::msg::PointField::FLOAT32,
    "z", 9, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(static_cast<size_t>(cloud.size()));
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  for (const auto & pt : cloud.points) {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    ++iter_x; ++iter_y; ++iter_z;
  }
}

}  // namespace pcl
