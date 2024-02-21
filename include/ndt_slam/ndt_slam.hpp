#ifndef _NDT_SLAM_HPP_
#define _NDT_SLAM_HPP_

#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

class NDTSlam : public rclcpp::Node
{
  using PointType = pcl::PointXYZI;

public:
  NDTSlam();
  ~NDTSlam() = default;

private:
  void crop(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
    const double max_scan_range);
  void downsample(
    const pcl::PointCloud<PointType>::Ptr input_ptr,
    const pcl::PointCloud<PointType>::Ptr & output_ptr);

  void sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_points_ptr_msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_ptr_msg);

  void imu_correct(const rclcpp::Time current_scan_time);
  void imu_correct(Eigen::Matrix4f & pose_matrix, const rclcpp::Time stamp);

  geometry_msgs::msg::TransformStamped get_transform(
    const std::string target_frame, const std::string source_frame);
  void transform_point_cloud(
    pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
    const std::string target_frame, const std::string source_frame);
  void publish_tf(
    const Eigen::Matrix4f pose_matrix, const rclcpp::Time stamp, const std::string frame_id,
    const std::string child_frame_id);

  bool save_map_service(
    const std_srvs::srv::Empty::Request::SharedPtr req,
    std_srvs::srv::Empty::Response::SharedPtr res);

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_publisher_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_service_;

  Eigen::Matrix4f pose_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f previous_pose_{Eigen::Matrix4f::Identity()};

  Eigen::Vector3f imu_rotate_vec_;

  rclcpp::Time previous_scan_time_;

  pcl::PointCloud<PointType>::Ptr map_;
  pclomp::NormalDistributionsTransform<PointType, PointType> ndt_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  bool use_imu_{false};
  bool use_debug_{false};
  std::string base_frame_id_;

  double min_scan_range_;
  double max_scan_range_;
  double min_add_scan_shift_;

  // voxel grid filter
  double leaf_size_;
  double map_leaf_size_;

  // NDT config rosparam
  double trans_eps_;
  double step_size_;
  double ndt_res_;
  int max_iter_;
  int omp_num_thread_;

  std::deque<sensor_msgs::msg::Imu> imu_queue_;
};

#endif
