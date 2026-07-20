#pragma once
// Minimal stub replacing ros pcl_conversions for Path E plant build.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace pcl {

template <typename PointT>
inline void fromROSMsg(const sensor_msgs::msg::PointCloud2 &msg,
                       pcl::PointCloud<PointT> &cloud) {
  cloud.clear();
  if (msg.fields.empty() || msg.data.empty()) {
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
    cloud.clear();
  }
}

template <typename PointT>
inline void toROSMsg(const pcl::PointCloud<PointT> & /*cloud*/,
                     sensor_msgs::msg::PointCloud2 & /*msg*/) {}

}  // namespace pcl
