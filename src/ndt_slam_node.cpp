#include <ndt_slam/ndt_slam.h>

int main(int argc, char**argv)
{
  ros::init(argc, argv, "ndt_slam_node");

  NDTSlam ndt_slam;

  ros::spin();

  return 0;
}
