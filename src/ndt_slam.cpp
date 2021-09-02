#include <ndt_slam/ndt_slam.h>
#include <ndt_slam/ndt_slam_utils.h>

NDTSlam::NDTSlam()
{
  pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  pnh_.param<double>("min_scan_range", min_scan_range_, 5.0);
  pnh_.param<double>("max_scan_range", max_scan_range_, 200.0);
  pnh_.param<double>("min_add_scan_shift", min_add_scan_shift_, 1.0);
  pnh_.param<double>("ndt_res", ndt_res_, 1.0);
  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size_, 0.1);
  pnh_.param<double>("leaf_size", leaf_size_, 2.0);
  pnh_.param<int>("max_iter", max_iter_, 30);
  pnh_.param<int>("omp_num_thread", omp_num_thread_, 0);

  ndt_.setTransformationEpsilon(trans_eps_);
  ndt_.setStepSize(step_size_);
  ndt_.setResolution(ndt_res_);
  ndt_.setMaximumIterations(max_iter_);
  if (0 < omp_num_thread_) ndt_.setNumThreads(omp_num_thread_);
  ndt_.setNeighborhoodSearchMethod(pclomp::KDTREE);

  map_.reset(new pcl::PointCloud<PointType>);
  map_->header.frame_id = "map";

  // create subscriber
  points_subscriber_ = nh_.subscribe("points_raw", 1000, &NDTSlam::pointsCallback, this);
  odom_subscriber_ = nh_.subscribe("odom", 10, &NDTSlam::odomCallback, this);
  imu_subscriber_ = nh_.subscribe("imu", 10, &NDTSlam::imuCallback, this);

  // create publisher
  ndt_aligned_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("alinged_cloud", 1000);
  ndt_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("ndt_map", 1000);
  ndt_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1000);
  transform_probability_publisher_ = nh_.advertise<std_msgs::Float32>("transform_probability", 1);

  // create service server
  save_map_service_ = pnh_.advertiseService("save_map", &NDTSlam::saveMapService, this);
}

Pose NDTSlam::getCurrentPose() { return ndt_slam_utils::convertMatrixToPoseVec(pose_); }

void NDTSlam::limitCloudScanData(
  const pcl::PointCloud<PointType>::Ptr input_ptr,
  const pcl::PointCloud<PointType>::Ptr & output_ptr, const double min_scan_range,
  const double max_scan_range)
{
  for (auto point : input_ptr->points) {
    const double range = std::hypot(point.x, point.y);
    if (min_scan_range < range && range < max_scan_range) { output_ptr->push_back(point); }
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

void NDTSlam::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr & input_points_ptr_msg)
{
  pcl::PointCloud<PointType>::Ptr points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr limit_points_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr transformed_scan_ptr(new pcl::PointCloud<PointType>);

  const ros::Time current_scan_time = input_points_ptr_msg->header.stamp;
  pcl::fromROSMsg(*input_points_ptr_msg, *points_ptr);

  limitCloudScanData(points_ptr, limit_points_ptr, min_scan_range_, max_scan_range_);

  if (initial_scan_loaded_) {
    initial_scan_loaded_ = false;
    *map_ += *limit_points_ptr;  //transformed_scan_ptr;
    ndt_.setInputTarget(limit_points_ptr);
  }

  downsample(limit_points_ptr, filtered_scan_ptr);

  // transform base_link to sensor_link
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  const std::string sensor_frame_id = input_points_ptr_msg->header.frame_id;
  geometry_msgs::TransformStamped sensor_frame_transform;
  try {
    sensor_frame_transform = tf_buffer_.lookupTransform(
      base_frame_id_, input_points_ptr_msg->header.frame_id,  input_points_ptr_msg->header.stamp, ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR("%s", ex.what());
    sensor_frame_transform.header.stamp = current_scan_time;
    sensor_frame_transform.header.frame_id = base_frame_id_;
    sensor_frame_transform.child_frame_id = sensor_frame_id;
    sensor_frame_transform.transform.translation.x = 0.0;
    sensor_frame_transform.transform.translation.y = 0.0;
    sensor_frame_transform.transform.translation.z = 0.0;
    sensor_frame_transform.transform.rotation.w = 1.0;
    sensor_frame_transform.transform.rotation.x = 0.0;
    sensor_frame_transform.transform.rotation.y = 0.0;
    sensor_frame_transform.transform.rotation.z = 0.0;
  }
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix =
    base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*filtered_scan_ptr, *transform_cloud_ptr, base_to_sensor_frame_matrix);
  ndt_.setInputSource(transform_cloud_ptr);

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_.align(*output_cloud, pose_);

  const bool convergenced = ndt_.hasConverged();
  const double fitness_score = ndt_.getFitnessScore();
  const int final_iterations = ndt_.getFinalNumIteration();

  if (!convergenced) { ROS_WARN("NDT has not Convergenced!"); }

  pose_ = ndt_.getFinalTransformation();

  // publish tf
  ndt_pose_ = getCurrentPose();  // convert matrix to vec
  ndt_slam_utils::publishTF(br_, ndt_pose_, current_scan_time, "map", base_frame_id_);

  previous_scan_time_ = current_scan_time;

  const double delta = std::hypot(ndt_pose_.x - previous_pose_.x, ndt_pose_.y - previous_pose_.y);
  if (min_add_scan_shift_ <= delta) {
    previous_pose_ = ndt_pose_;

    pcl::transformPointCloud(*limit_points_ptr, *transformed_scan_ptr, pose_);
    *map_ += *transformed_scan_ptr;

    ndt_.setInputTarget(map_);

    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_, map_msg);
    ndt_map_publisher_.publish(map_msg);
  }

  sensor_msgs::PointCloud2 aligned_cloud_msg;
  aligned_cloud_msg.header.stamp = current_scan_time;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  ndt_aligned_cloud_publisher_.publish(aligned_cloud_msg);

  std_msgs::Float32 transform_probability;
  transform_probability.data = ndt_.getTransformationProbability();
  transform_probability_publisher_.publish(transform_probability);

  geometry_msgs::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose = ndt_slam_utils::convertToGeometryPose(ndt_pose_);

  ndt_pose_publisher_.publish(ndt_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "NDT has converged: " << convergenced << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << final_iterations << std::endl;
  std::cout << "delta: " << delta << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

bool NDTSlam::saveMapService(ndt_slam::SaveMapRequest & req, ndt_slam::SaveMapResponse & res)
{
  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);

  if (req.resolution <= 0.0) {
    map_cloud = map_;
  } else {
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(req.resolution, req.resolution, req.resolution);
    voxel_grid_filter.setInputCloud(map_);
    voxel_grid_filter.filter(*map_cloud);
  }

  map_cloud->header.frame_id = "map";
  int ret = pcl::io::savePCDFile(req.path, *map_cloud);
  res.ret = (ret == 0);

  return true;
}

void NDTSlam::odomCallback(const nav_msgs::Odometry::ConstPtr & msg) { odom_ = *msg; }

void NDTSlam::imuCallback(const sensor_msgs::Imu::ConstPtr & msg) { imu_ = *msg; }
