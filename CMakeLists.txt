cmake_minimum_required(VERSION 3.0.2)
project(ndt_mapping)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  pcl_ros
  roscpp
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp tf sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ndt_mapping_node src/ndt_mapping_node.cpp src/ndt_mapping.cpp)
target_link_libraries(ndt_mapping_node ${catkin_LIBRARIES})
