#pragma 0

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

struct EulerPose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
  EulerPose() : x(0.0), y(0.0), z(0.0), roll(0.0), pitch(0.0), yaw(0.0) {}
};

// odometry・IMU必須、通常実装のNDT使用
class NDTMapping
{
public:
  NDTMapping();
  ~NDTMapping() = default;

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber points_subscriber_;
  ros::Subscriber odom_subscriber_;
  ros::Subscriber imu_subscriber_;
  ros::Publisher ndt_map_publisher_;
  ros::Publisher current_pose_publisher_;

  EulerPose offset_imu_odom_;
  // 点群地図を統合する際に移動量を計算するための初期位置用変数
  EulerPose added_pose_;
  // NDTに基づく自己位置
  EulerPose ndt_pose_; // TODO 消す
  EulerPose current_pose_;
  // 一個前のスキャン時の自己位置
  EulerPose previous_pose_;
  EulerPose current_pose_imu_odom_;
  EulerPose guess_pose_imu_odom_;

  ros::Time current_scan_time_;
  ros::Time previous_scan_time_;

  tf::TransformBroadcaster br_;
  tf::Transform transform_;

  pcl::PointCloud<pcl::PointXYZI> map_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

  bool initial_scan_loaded_;
  bool is_first_map_;

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

  // rosparam
  double tf_x_;
  double tf_y_;
  double tf_z_;
  double tf_roll_;
  double tf_pitch_;
  double tf_yaw_;
  Eigen::Matrix4f tf_btol_, tf_ltob_;  // base_link to sensor_link

  void init(EulerPose & pose);
  void calcImuAndOdometry(const ros::Time time);

  void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr & points);
  void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr & msg);
};
