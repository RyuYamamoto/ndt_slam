# ndt_slam

## How to use
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/RyuYamamoto/ndt_slam
catkin build ndt_slam -DCMAKE_BUILD_TYPE=Release
roslaunch ndt_slam ndt_slam.launch
```
open other terminal,
```
rosbag play --clock <ROSBAG PATH>
```

[![](https://img.youtube.com/vi/ncyMT3vk7H4/0.jpg)](https://www.youtube.com/watch?v=ncyMT3vk7H4)
