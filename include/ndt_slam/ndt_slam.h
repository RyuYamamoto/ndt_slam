#ifndef _NDT_MAPPING_
#define _NDT_MAPPING_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <ndt_slam/data_struct.h>


class NDTSlam
{
  using PointType = pcl::PointXYZI;

public:
  NDTSlam();
  ~NDTSlam() = default;

private:
  void limitCloudScanData(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
    const double max_scan_range);
  void downsample(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr);

  Pose getCurrentPose();

  void imuCorrect(const ros::Time current_scan_time);

  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr & input_points_ptr_msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr & msg);

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber points_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher ndt_aligned_cloud_publisher_;
  ros::Publisher ndt_map_publisher_;
  ros::Publisher ndt_pose_publisher_;
  ros::Publisher transform_probability_publisher_;

  Pose ndt_pose_;
  Pose previous_pose_;

  Eigen::Matrix4f pose_{Eigen::Matrix4f::Identity()};

  ros::Time previous_scan_time_;

  tf2_ros::TransformBroadcaster br_;

  pcl::PointCloud<PointType> map_;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt_;

  bool initial_scan_loaded_{true};

  // rosparam
  double min_scan_range_;
  double max_scan_range_;
  double min_add_scan_shift_;

  // voxel grid filter
  double leaf_size_;

  // NDT config rosparam
  double trans_eps_;
  double step_size_;
  double ndt_res_;
  int max_iter_;

  sensor_msgs::Imu imu_;
  nav_msgs::Odometry odom_;

  pcl::VoxelGrid<PointType> voxel_grid_filter_;
};

#endif
