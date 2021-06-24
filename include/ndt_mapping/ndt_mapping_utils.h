#ifndef _NDT_MAPPING_UTILS_
#define _NDT_MAPPING_UTILS_

#include <geometry_msgs/Pose.h>

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

}
}

#endif
