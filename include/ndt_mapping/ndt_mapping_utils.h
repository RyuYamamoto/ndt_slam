#ifndef _NDT_MAPPING_UTILS_
#define _NDT_MAPPING_UTILS_

#include <geometry_msgs/Pose.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ndt_mapping/data_struct.h>

namespace ndt_mapping_utils
{
geometry_msgs::Pose convertToGeometryPose(const Pose input_pose)
{
  geometry_msgs::Pose output_pose;
  output_pose.position.x = input_pose.x;
  output_pose.position.y = input_pose.y;
  output_pose.position.z = input_pose.z;
  tf2::Quaternion quat;
  quat.setRPY(input_pose.roll, input_pose.pitch, input_pose.yaw);
  output_pose.orientation.w = quat.w();
  output_pose.orientation.x = quat.x();
  output_pose.orientation.y = quat.y();
  output_pose.orientation.z = quat.z();
}

tf2::Transform convertToTransform(const Pose input_pose)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(input_pose.x, input_pose.y, input_pose.z));
  tf2::Quaternion quaternion;
  quaternion.setRPY(input_pose.roll, input_pose.pitch, input_pose.yaw);
  transform.setRotation(quaternion);

  return transform;
}

void publishTF(tf2_ros::TransformBroadcaster broadcaster, const Pose pose, const ros::Time stamp, const std::string frame_id, const std::string child_frame_id)
{
  geometry_msgs::Transform transform;
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped = tf2::toMsg(tf2::Stamped<tf2::Transform>(convertToTransform(pose), stamp, frame_id));
  transform_stamped.child_frame_id = child_frame_id;

  broadcaster.sendTransform(transform_stamped);
}
}

#endif
