#include <ndt_slam/ndt_slam.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NDTSlam>());
  rclcpp::shutdown();
  return 0;
}
