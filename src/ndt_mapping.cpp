#include <ndt_mapping/ndt_mapping.h>
#include <ndt_mapping/ndt_mapping_utils.h>

NDTMapping::NDTMapping()
{
  pnh_.param<double>("tf_x", tf_x_, 0.0);
  pnh_.param<double>("tf_y", tf_y_, 0.0);
  pnh_.param<double>("tf_z", tf_z_, 0.0);
  pnh_.param<double>("tf_roll", tf_roll_, 0.0);
  pnh_.param<double>("tf_pitch", tf_pitch_, 0.0);
  pnh_.param<double>("tf_yaw", tf_yaw_, 0.0);

  pnh_.param<double>("min_scan_range", min_scan_range_, 5.0);
  pnh_.param<double>("max_scan_range", max_scan_range_, 200.0);
  pnh_.param<double>("min_add_scan_shift", min_add_scan_shift_, 1.0);
  pnh_.param<double>("ndt_res", ndt_res_, 1.0);
  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size_, 0.1);
  pnh_.param<double>("leaf_size", leaf_size_, 2.0);
  pnh_.param<int>("max_iter", max_iter_, 30);

  ndt_.setTransformationEpsilon(trans_eps_);
  ndt_.setStepSize(step_size_);
  ndt_.setResolution(ndt_res_);
  ndt_.setMaximumIterations(max_iter_);

  Eigen::Translation3f tl_btol(tf_x_, tf_y_, tf_z_);
  Eigen::AngleAxisf rot_x_btol(tf_roll_, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(tf_pitch_, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(tf_yaw_, Eigen::Vector3f::UnitZ());
  // base_link -> lidar_link
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  // lidar_link -> base_link
  tf_ltob_ = tf_btol_.inverse();

  map_.header.frame_id = "map";

  voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

  // create subscriber
  points_subscriber_ = nh_.subscribe("points_raw", 1, &NDTMapping::pointsCallback, this);
  odom_subscriber_ = nh_.subscribe("odom", 10, &NDTMapping::odomCallback, this);
  imu_subscriber_ = nh_.subscribe("imu", 10, &NDTMapping::imuCallback, this);

  // create publisher
  ndt_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("ndt_map", 1000);
  ndt_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1000);
  transform_probability_publisher_ = nh_.advertise<std_msgs::Float32>("transform_probability", 1);
}

void NDTMapping::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr & points)
{
  pcl::PointCloud<PointType> tmp;
  pcl::PointCloud<PointType>::Ptr scan_ptr(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr transformed_scan_ptr(new pcl::PointCloud<PointType>());

  current_scan_time_ = points->header.stamp;
  pcl::fromROSMsg(*points, tmp);

  // スキャンした点群をminとmaxの距離でカットする
  for (const auto p : tmp.points) {
    const double dist = std::hypot(p.x, p.y);
    if (min_scan_range_ < dist && dist < max_scan_range_) { scan_ptr->push_back(p); }
  }

  // 初回スキャン時
  if (initial_scan_loaded_) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol_);
    map_ += *transformed_scan_ptr;
    initial_scan_loaded_ = false;
  }

  // 入力点群を間引く
  voxel_grid_filter_.setInputCloud(scan_ptr);
  voxel_grid_filter_.filter(*filtered_scan_ptr);

  // 入力点群設定
  ndt_.setInputSource(filtered_scan_ptr);

  pcl::PointCloud<PointType>::Ptr map_ptr(new pcl::PointCloud<PointType>(map_));

  // 初回マップ作成タイミング
  if (is_first_map_) {
    ndt_.setInputTarget(map_ptr);
    is_first_map_ = false;
  }

  Eigen::AngleAxisf init_rotation_x(ndt_pose_.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(ndt_pose_.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(ndt_pose_.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(ndt_pose_.x, ndt_pose_.y, ndt_pose_.z);
  Eigen::Matrix4f init_guess =
    (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_.align(*output_cloud, init_guess);
  const double fitness_score = ndt_.getFitnessScore();

  Eigen::Matrix4f t_localizer = ndt_.getFinalTransformation();
  Eigen::Matrix4f t_base_link = t_localizer * tf_ltob_;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_b;
  mat_b.setValue(
    static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
    static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
    static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
    static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
    static_cast<double>(t_base_link(2, 2)));

  // update ndt pose
  ndt_pose_.x = t_base_link(0, 3);
  ndt_pose_.y = t_base_link(1, 3);
  ndt_pose_.z = t_base_link(2, 3);
  mat_b.getRPY(ndt_pose_.roll, ndt_pose_.pitch, ndt_pose_.yaw);

  // publish tf
  ndt_mapping_utils::publishTF(br_, ndt_pose_, current_scan_time_, "map", "base_link");

  previous_scan_time_ = current_scan_time_;

  // 指定距離移動してたら点群地図足し合わせ
  const double shift = std::hypot(ndt_pose_.x - previous_pose_.x, ndt_pose_.y - previous_pose_.y);
  if (min_add_scan_shift_ <= shift) {
    map_ += *transformed_scan_ptr;
    previous_pose_ = ndt_pose_;
    ndt_.setInputTarget(map_ptr);
    // 点群地図を出力
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*map_ptr, map_msg);
    ndt_map_publisher_.publish(map_msg);
  }

  // マッチングの確率を出力
  std_msgs::Float32 transform_probability;
  transform_probability.data = ndt_.getTransformationProbability();
  transform_probability_publisher_.publish(transform_probability);

  // 自己位置を出力
  geometry_msgs::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = "map";
  ndt_pose_msg.header.stamp = current_scan_time_;
  ndt_pose_msg.pose = ndt_mapping_utils::convertToGeometryPose(ndt_pose_);

  ndt_pose_publisher_.publish(ndt_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << points->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points."
            << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points."
            << std::endl;
  std::cout << "map: " << map_.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt_.hasConverged() << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << ndt_.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << ndt_pose_.x << ", " << ndt_pose_.y << ", " << ndt_pose_.z << ", "
            << ndt_pose_.roll << ", " << ndt_pose_.pitch << ", " << ndt_pose_.yaw << ")"
            << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void NDTMapping::odomCallback(const nav_msgs::Odometry::ConstPtr & msg) { odom_ = *msg; }

void NDTMapping::imuCallback(const sensor_msgs::Imu::ConstPtr & msg) { imu_ = *msg; }
