#include "ndt_mapping/ndt_mapping.h"
#include "ndt_mapping/ndt_mapping_utils.h"

NDTMapping::NDTMapping() : initial_scan_loaded_(true), is_first_map_(true)
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

  init(current_pose_);

  // create subscriber
  points_subscriber_ = nh_.subscribe("points_raw", 10, &NDTMapping::pointsCallback, this);
  odom_subscriber_ = nh_.subscribe("odom", 10, &NDTMapping::odomCallback, this);
  imu_subscriber_ = nh_.subscribe("imu", 10, &NDTMapping::imuCallback, this);

  // create publisher
  ndt_map_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("ndt_map", 1000);
  current_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("current_pose", 1000);
}

void NDTMapping::init(EulerPose &pose)
{
  pose.x = 0.0;
  pose.y = 0.0;
  pose.z = 0.0;
  pose.roll = 0.0;
  pose.pitch = 0.0;
  pose.yaw = 0.0;
}

// imuのroll方向の角速度と前進速度から得られる移動差分に対してpitchとyawの角速度で補正したものを足し合わせて推定位置を計算する
void NDTMapping::calcImuAndOdometry(const ros::Time time)
{
  static ros::Time previous_time = time;  // TODO delete static
  double diff_time = (time - previous_time).toSec();

  // imuの角速度から回転差分を計算
  current_pose_imu_odom_.roll += (imu_.angular_velocity.x * diff_time);
  current_pose_imu_odom_.pitch += (imu_.angular_velocity.y * diff_time);
  current_pose_imu_odom_.yaw += (imu_.angular_velocity.z * diff_time);

  // 並進移動量
  const double diff_distance = odom_.twist.twist.linear.x * diff_time;
  // pitchとyawを考慮しオフセット計算
  offset_imu_odom_.x =
    diff_distance * std::cos(-current_pose_imu_odom_.pitch) * std::cos(current_pose_imu_odom_.yaw);
  offset_imu_odom_.y =
    diff_distance * std::cos(-current_pose_imu_odom_.pitch) * std::sin(current_pose_imu_odom_.yaw);
  offset_imu_odom_.z = diff_distance * std::sin(-current_pose_imu_odom_.pitch);

  // imuとodometryに基づく推測位置
  guess_pose_imu_odom_.x = previous_pose_.x + offset_imu_odom_.x;
  guess_pose_imu_odom_.y = previous_pose_.y + offset_imu_odom_.y;
  guess_pose_imu_odom_.z = previous_pose_.z + offset_imu_odom_.z;
  guess_pose_imu_odom_.roll = previous_pose_.roll + offset_imu_odom_.roll;
  guess_pose_imu_odom_.pitch = previous_pose_.pitch + offset_imu_odom_.pitch;
  guess_pose_imu_odom_.yaw = previous_pose_.yaw + offset_imu_odom_.yaw;

  previous_time = time;
}

void NDTMapping::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr & points)
{
  pcl::PointCloud<pcl::PointXYZI> tmp;
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

  current_scan_time_ = points->header.stamp;
  pcl::fromROSMsg(*points, tmp);

  // スキャンした点群をminとmaxの距離でカットする
  for (const auto p : tmp.points) {
    const double dist = std::sqrt(p.x * p.x + p.y * p.y);
    if (min_scan_range_ < dist && dist < max_scan_range_) scan_ptr->push_back(p);
  }

  // 初回スキャン時
  if (initial_scan_loaded_) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol_);
    map_ += *transformed_scan_ptr;
    std::cout << map_.points.size() << std::endl;
    initial_scan_loaded_ = false;
  }

  // 入力点群を間引く
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);

  // NDT設定
  ndt_.setInputSource(filtered_scan_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map_));

  // 初回マップ作成タイミング
  if (is_first_map_) {
    ndt_.setInputTarget(map_ptr);
    is_first_map_ = false;
  }

  // imuとodometryから移動差分を計算する
  calcImuAndOdometry(current_scan_time_);

  Eigen::AngleAxisf init_rotation_x(guess_pose_imu_odom_.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_imu_odom_.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_imu_odom_.yaw, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(
    guess_pose_imu_odom_.x, guess_pose_imu_odom_.y, guess_pose_imu_odom_.z);
  Eigen::Matrix4f init_guess =
    (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  ndt_.align(*output_cloud, init_guess);
  const double fitness_score = ndt_.getFitnessScore();

  Eigen::Matrix4f t_localizer = ndt_.getFinalTransformation();
  Eigen::Matrix4f t_base_link = t_localizer * tf_ltob_;

  pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

  tf::Matrix3x3 mat_l, mat_b;
  mat_l.setValue(
    static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
    static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
    static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
    static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
    static_cast<double>(t_localizer(2, 2)));
  mat_b.setValue(
    static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
    static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
    static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
    static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
    static_cast<double>(t_base_link(2, 2)));

  // update ndt pose
  current_pose_.x = t_base_link(0, 3);
  current_pose_.y = t_base_link(1, 3);
  current_pose_.z = t_base_link(2, 3);
  mat_b.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);

  // base_link -> map
  // TODO implement tf2
  transform_.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
  tf::Quaternion quaternion;
  quaternion.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  transform_.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform_, current_scan_time_, "map", "base_link"));

  current_pose_imu_odom_ = current_pose_;
  previous_pose_ = current_pose_;

  previous_scan_time_ = current_scan_time_;

  // オフセットクリア
  init(offset_imu_odom_);

  // 指定距離移動してたら点群地図足し合わせ
  const double shift = std::sqrt(std::pow(current_pose_.x - added_pose_.x, 2.0) + std::pow(current_pose_.y - added_pose_.y, 2.0));
  if(min_add_scan_shift_ <= shift) {
    map_ += *transformed_scan_ptr;
    added_pose_ = current_pose_;
    ndt_.setInputTarget(map_ptr);
  }

  // 点群地図を出力
  sensor_msgs::PointCloud2 map_msg;
  pcl::toROSMsg(*map_ptr, map_msg);
  ndt_map_publisher_.publish(map_msg);

  // 自己位置を出力
  quaternion.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw);
  geometry_msgs::PoseStamped current_pose_msg;
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time_;
  current_pose_msg.pose.position.x = current_pose_.x;
  current_pose_msg.pose.position.y = current_pose_.y;
  current_pose_msg.pose.position.z = current_pose_.z;
  current_pose_msg.pose.orientation.x = quaternion.x();
  current_pose_msg.pose.orientation.y = quaternion.y();
  current_pose_msg.pose.orientation.z = quaternion.z();
  current_pose_msg.pose.orientation.w = quaternion.w();

  current_pose_publisher_.publish(current_pose_msg);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << points->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
  std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
  std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map_.points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt_.hasConverged() << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "Number of iteration: " << ndt_.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
            << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;
}

void NDTMapping::odomCallback(const nav_msgs::Odometry::ConstPtr & msg) { odom_ = *msg; }

void NDTMapping::imuCallback(const sensor_msgs::Imu::ConstPtr & msg) { imu_ = *msg; }
