#ifndef DECOMP_ROS_UTILS_H
#define DECOMP_ROS_UTILS_H

// Slimmed header for drone_ws Path E (no pcl_conversions / RViz plugin links).
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <decomp_ros_msgs/msg/polyhedron.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace DecompROS {

template <int Dim> nav_msgs::msg::Path vec_to_path(const vec_Vecf<Dim> &vs) {
  nav_msgs::msg::Path path;
  for (const auto& it : vs) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = it(0);
    pose.pose.position.y = it(1);
    pose.pose.position.z = Dim == 2 ? 0 : it(2);
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

inline sensor_msgs::msg::PointCloud vec_to_cloud(const vec_Vec3f &pts) {
  sensor_msgs::msg::PointCloud cloud;
  cloud.points.resize(pts.size());
  for (unsigned int i = 0; i < pts.size(); i++) {
    cloud.points[i].x = pts[i](0);
    cloud.points[i].y = pts[i](1);
    cloud.points[i].z = pts[i](2);
  }
  return cloud;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::msg::PointCloud &cloud) {
  vec_Vec3f pts;
  pts.resize(cloud.points.size());
  for (unsigned int i = 0; i < cloud.points.size(); i++) {
    pts[i](0) = cloud.points[i].x;
    pts[i](1) = cloud.points[i].y;
    pts[i](2) = cloud.points[i].z;
  }
  return pts;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  vec_Vec3f pts;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    pts.emplace_back(*iter_x, *iter_y, *iter_z);
  }
  return pts;
}

inline vec_Vec3f cloud_to_vec(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  return cloud_to_vec(sensor_msgs::msg::PointCloud2::ConstSharedPtr(msg));
}

inline decomp_ros_msgs::msg::Polyhedron polyhedron_to_ros(const Polyhedron3D &poly) {
  decomp_ros_msgs::msg::Polyhedron msg;
  for (const auto &h : poly.hyperplanes()) {
    geometry_msgs::msg::Point pt, n;
    pt.x = h.p_(0); pt.y = h.p_(1); pt.z = h.p_(2);
    n.x = h.n_(0); n.y = h.n_(1); n.z = h.n_(2);
    msg.points.push_back(pt);
    msg.normals.push_back(n);
  }
  return msg;
}

template <int Dim>
decomp_ros_msgs::msg::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>> &vs) {
  decomp_ros_msgs::msg::PolyhedronArray msg;
  for (const auto &v : vs) {
    if constexpr (Dim == 3) {
      msg.polyhedrons.push_back(polyhedron_to_ros(v));
    } else {
      Polyhedron3D p3;
      for (const auto &h : v.hyperplanes()) {
        Vec3f p(h.p_(0), h.p_(1), 0), n(h.n_(0), h.n_(1), 0);
        p3.add(Hyperplane3D(p, n));
      }
      msg.polyhedrons.push_back(polyhedron_to_ros(p3));
    }
  }
  return msg;
}

template <int Dim>
decomp_ros_msgs::msg::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>> & /*Es*/) {
  return decomp_ros_msgs::msg::EllipsoidArray();
}

}  // namespace DecompROS

#endif
