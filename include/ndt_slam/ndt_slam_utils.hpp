#ifndef _NDT_SLAM_UTILS_
#define _NDT_SLAM_UTILS_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace ndt_slam_utils
{

geometry_msgs::msg::Vector3 convert_quaternion_to_euler(
  const geometry_msgs::msg::Quaternion quaternion)
{
  geometry_msgs::msg::Vector3 euler;

  tf2::Quaternion quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(euler.x, euler.y, euler.z);

  return euler;
}

geometry_msgs::msg::Pose convert_transform_to_pose(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform_stamped.transform.translation.x;
  pose.position.y = transform_stamped.transform.translation.y;
  pose.position.z = transform_stamped.transform.translation.z;
  pose.orientation.w = transform_stamped.transform.rotation.w;
  pose.orientation.x = transform_stamped.transform.rotation.x;
  pose.orientation.y = transform_stamped.transform.rotation.y;
  pose.orientation.z = transform_stamped.transform.rotation.z;
  return pose;
}

Eigen::Matrix4f convert_pose_to_matrix(const geometry_msgs::msg::Pose pose)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(pose, affine);
  Eigen::Matrix4f matrix = affine.matrix().cast<float>();
  return matrix;
}

geometry_msgs::msg::Pose convert_matrix_to_pose(const Eigen::Matrix4f matrix)
{
  geometry_msgs::msg::Pose pose;

  const Eigen::Vector3d position = matrix.block<3, 1>(0, 3).cast<double>();
  const Eigen::Quaterniond quaternion(matrix.block<3, 3>(0, 0).cast<double>());

  pose.position = tf2::toMsg(position);
  pose.orientation = tf2::toMsg(quaternion);

  return pose;
}

Eigen::VectorXf convert_matrix_to_vector(const Eigen::Matrix4f matrix)
{
  Eigen::VectorXf vector = Eigen::VectorXf::Zero(6);

  const Eigen::Vector3f position = matrix.block<3, 1>(0, 3).cast<float>();
  const Eigen::Quaternionf quaternion(matrix.block<3, 3>(0, 0).cast<float>());

  double roll, pitch, yaw;
  tf2::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
  tf2::Matrix3x3 mat(q);
  mat.getRPY(roll, pitch, yaw);

  vector << position[0], position[1], position[2], roll, pitch, yaw;

  return vector;
}

Eigen::Matrix4f convert_vector_to_matrix(const Eigen::VectorXf vector)
{
  Eigen::Translation<float, 3> translation(vector[0], vector[1], vector[2]);
  Eigen::Quaternionf rotation = Eigen::AngleAxisf(vector[3], Eigen::Vector3f::UnitX()) *
                                Eigen::AngleAxisf(vector[4], Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(vector[5], Eigen::Vector3f::UnitZ());

  Eigen::Affine3f affine = translation * rotation;

  Eigen::Matrix4f matrix = affine.matrix().cast<float>();

  return matrix;
}

Eigen::Matrix4f convert_transform_to_matrix(
  const geometry_msgs::msg::TransformStamped transform_stamped)
{
  return convert_pose_to_matrix(convert_transform_to_pose(transform_stamped));
}

}  // namespace ndt_slam_utils

#endif
