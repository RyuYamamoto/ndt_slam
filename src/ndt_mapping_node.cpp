#include <ndt_mapping/ndt_mapping.h>

int main(int argc, char**argv)
{
  ros::init(argc, argv, "ndt_mapping_node");

  NDTMapping ndt_mapping;

  ros::spin();

  return 0;
}
