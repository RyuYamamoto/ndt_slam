#include <ndt_slam/ndt_slam.hpp>
#include <ndt_slam/ndt_slam_utils.hpp>

using namespace ndt_slam_utils;

NDTSlam::NDTSlam() : Node("ndt_slam")
{
  base_frame_id_ = declare_parameter<std::string>("base_frame_id");
  min_scan_range_ = declare_parameter<double>("min_scan_range");
  max_scan_range_ = declare_parameter<double>("max_scan_range");
  min_add_scan_shift_ = declare_parameter<double>("min_add_scan_shift");
  ndt_res_ = declare_parameter<double>("ndt_res");
  trans_eps_ = declare_parameter<double>("trans_eps");
  step_size_ = declare_parameter<double>("step_size");
  max_iter_ = declare_parameter<int>("max_iter");
  omp_num_thread_ = declare_parameter<int>("omp_num_thread");
  leaf_size_ = declare_parameter<double>("leaf_size");
  map_leaf_size_ = declare_parameter<double>("map_leaf_size");
  use_imu_ = declare_parameter<bool>("use_imu");
  use_debug_ = declare_parameter<bool>("use_debug");

  ndt_.setTransformationEpsilon(trans_eps_);
  ndt_.setStepSize(step_size_);
  ndt_.setResolution(ndt_res_);
  ndt_.setMaximumIterations(max_iter_);
  if (0 < omp_num_thread_) ndt_.setNumThreads(omp_num_thread_);
  ndt_.setNeighborhoodSearchMethod(pclomp::KDTREE);

  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // create subscriber
  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS(),
    std::bind(&NDTSlam::sensor_callback, this, std::placeholders::_1));
  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 10, std::bind(&NDTSlam::imu_callback, this, std::placeholders::_1));

  // create publisher
  ndt_map_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ndt_map", 10);
  ndt_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);

  // create service server
  save_map_service_ = this->create_service<std_srvs::srv::Empty>(
    "save_map",
    std::bind(&NDTSlam::save_map_service, this, std::placeholders::_1, std::placeholders::_2));
}

void NDTSlam::imu_correct(Eigen::Matrix4f & pose_matrix, const rclcpp::Time stamp)
{
  if (!imu_queue_.empty()) {
    // get latest imu data
    sensor_msgs::msg::Imu latest_imu_msgs;
    for (auto & imu : imu_queue_) {
      latest_imu_msgs = imu;
      const auto time_stamp = latest_imu_msgs.header.stamp;
      if (stamp < time_stamp) {
        break;
      }
    }
    while (!imu_queue_.empty()) {
      if (rclcpp::Time(imu_queue_.front().header.stamp) >= stamp) {
        break;
      }
      imu_queue_.pop_front();
    }

    static rclcpp::Time previous_stamp = stamp;
    static sensor_msgs::msg::Imu previous_imu = latest_imu_msgs;
    const double dt = (stamp - previous_stamp).seconds();

    imu_rotate_vec_.x() +=
      (latest_imu_msgs.angular_velocity.x - previous_imu.angular_velocity.x) * dt;
    imu_rotate_vec_.y() +=
      (latest_imu_msgs.angular_velocity.y - previous_imu.angular_velocity.y) * dt;
    imu_rotate_vec_.z() +=
      (latest_imu_msgs.angular_velocity.z - previous_imu.angular_velocity.z) * dt;

    Eigen::VectorXf pose_vector = convert_matrix_to_vector(pose_matrix);

    pose_vector[3] += imu_rotate_vec_.x();
    pose_vector[4] += imu_rotate_vec_.y();
    pose_vector[5] += imu_rotate_vec_.z();

    pose_matrix = convert_vector_to_matrix(pose_vector);

    previous_imu = latest_imu_msgs;
    previous_stamp = stamp;
  }
}

void NDTSlam::crop(
  const pcl::PointCloud<PointType>::Ptr input_ptr,
  const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
  const double max_scan_range)
{
  for (auto point : input_ptr->points) {
    const double range = std::hypot(point.x, point.y);
    if (min_scan_range < range && range < max_scan_range) {
      output_ptr->push_back(point);
    }
  }
}

void NDTSlam::downsample(
  const pcl::PointCloud<PointType>::Ptr input_ptr,
  const pcl::PointCloud<PointType>::Ptr & output_ptr)
{
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid_filter.setInputCloud(input_ptr);
  voxel_grid_filter.filter(*output_ptr);
}

void NDTSlam::sensor_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_points_ptr_msg)
{
  pcl::PointCloud<PointType>::Ptr points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr limit_points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>);

  const rclcpp::Time current_scan_time = input_points_ptr_msg->header.stamp;
  const std::string sensor_frame_id = input_points_ptr_msg->header.frame_id;
  pcl::fromROSMsg(*input_points_ptr_msg, *points_ptr);

  crop(points_ptr, limit_points_ptr, min_scan_range_, max_scan_range_);

  if (!map_) {
    pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
    transform_point_cloud(limit_points_ptr, transform_cloud_ptr, base_frame_id_, sensor_frame_id);
    map_.reset(new pcl::PointCloud<PointType>);
    map_->header.frame_id = "map";
    *map_ += *transform_cloud_ptr;
    ndt_.setInputTarget(map_);
  }

  pcl::PointCloud<PointType>::Ptr sensor_transform_cloud(new pcl::PointCloud<PointType>);
  transform_point_cloud(limit_points_ptr, sensor_transform_cloud, base_frame_id_, sensor_frame_id);
  downsample(sensor_transform_cloud, filtered_scan_ptr);
  ndt_.setInputSource(filtered_scan_ptr);

  if (use_imu_) imu_correct(pose_, current_scan_time);

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_.align(*output_cloud, pose_);

  const bool convergenced = ndt_.hasConverged();
  const double fitness_score = ndt_.getFitnessScore();
  const int final_iterations = ndt_.getFinalNumIteration();

  if (!convergenced) RCLCPP_WARN(get_logger(), "NDT has not Convergenced!");

  pose_ = ndt_.getFinalTransformation();

  // publish tf
  publish_tf(pose_, current_scan_time, "map", base_frame_id_);

  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::transformPointCloud(
    *limit_points_ptr, *transform_cloud_ptr,
    pose_ * convert_transform_to_matrix(get_transform(base_frame_id_, sensor_frame_id)));

  previous_scan_time_ = current_scan_time;

  const Eigen::Vector3d current_position = pose_.block<3, 1>(0, 3).cast<double>();
  const Eigen::Vector3d previous_position = previous_pose_.block<3, 1>(0, 3).cast<double>();
  const double delta = (current_position - previous_position).norm();
  if (min_add_scan_shift_ <= delta) {
    previous_pose_ = pose_;

    *map_ += *transform_cloud_ptr;
    ndt_.setInputTarget(map_);

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    ndt_map_publisher_->publish(map_msg);
  }

  Eigen::Quaterniond quaternion(pose_.block<3, 3>(0, 0).cast<double>());

  geometry_msgs::msg::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose.position = tf2::toMsg(current_position);
  ndt_pose_msg.pose.orientation = tf2::toMsg(quaternion);
  ndt_pose_publisher_->publish(ndt_pose_msg);

  if (!use_debug_) return;

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "NDT has converged: " << convergenced << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_iterations << std::endl;
  std::cout << "delta: " << delta << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void NDTSlam::transform_point_cloud(
  pcl::PointCloud<PointType>::Ptr input_ptr, pcl::PointCloud<PointType>::Ptr & output_ptr,
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped sensor_frame_transform =
    get_transform(target_frame, source_frame);
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix =
    base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*input_ptr, *output_ptr, base_to_sensor_frame_matrix);
}

geometry_msgs::msg::TransformStamped NDTSlam::get_transform(
  const std::string target_frame, const std::string source_frame)
{
  geometry_msgs::msg::TransformStamped frame_transform;
  try {
    frame_transform = tf_buffer_.lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    frame_transform.header.stamp = rclcpp::Clock().now();
    frame_transform.header.frame_id = target_frame;
    frame_transform.child_frame_id = source_frame;
    frame_transform.transform.translation.x = 0.0;
    frame_transform.transform.translation.y = 0.0;
    frame_transform.transform.translation.z = 0.0;
    frame_transform.transform.rotation.w = 1.0;
    frame_transform.transform.rotation.x = 0.0;
    frame_transform.transform.rotation.y = 0.0;
    frame_transform.transform.rotation.z = 0.0;
  }
  return frame_transform;
}

void NDTSlam::publish_tf(
  const Eigen::Matrix4f pose_matrix, const rclcpp::Time stamp, const std::string frame_id,
  const std::string child_frame_id)
{
  Eigen::Vector3d translation = pose_matrix.block<3, 1>(0, 3).cast<double>();
  Eigen::Quaterniond quaternion(pose_matrix.block<3, 3>(0, 0).cast<double>());

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = stamp;
  transform_stamped.transform.translation = tf2::toMsg2(translation);
  transform_stamped.transform.rotation = tf2::toMsg(quaternion);

  broadcaster_->sendTransform(transform_stamped);
}

bool NDTSlam::save_map_service(
  [[maybe_unused]] const std_srvs::srv::Empty::Request::SharedPtr req,
  [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
{
  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);

  if (map_leaf_size_ <= 0.0) {
    map_cloud = map_;
  } else {
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(map_leaf_size_, map_leaf_size_, map_leaf_size_);
    voxel_grid_filter.setInputCloud(map_);
    voxel_grid_filter.filter(*map_cloud);
  }

  map_cloud->header.frame_id = "map";
  pcl::io::savePCDFile("/tmp/map.pcd", *map_cloud);

  return true;
}

void NDTSlam::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_ptr_msg)
{
  if (!use_imu_) return;

  imu_queue_.emplace_back(*imu_ptr_msg);
}
