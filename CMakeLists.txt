cmake_minimum_required(VERSION 3.0.2)
project(ndt_slam)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  pcl_ros
  roscpp
  tf2
  tf2_sensor_msgs
  ndt_omp
)

add_service_files(
  FILES
  SaveMap.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp tf sensor_msgs pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ndt_slam_node src/ndt_slam_node.cpp src/ndt_slam.cpp)
target_link_libraries(ndt_slam_node ${catkin_LIBRARIES})
