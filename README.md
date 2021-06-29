# ndt_mapping

## How to use
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/RyuYamamoto/ndt_mapping
catkin build ndt_mapping -DCMAKE_BUILD_TYPE=Release
roslaunch ndt_mapping ndt_mapping.launch
```
open other terminal,
```
rosbag play --clock <ROSBAG PATH>
```

[![](https://img.youtube.com/vi/ncyMT3vk7H4/0.jpg)](https://www.youtube.com/watch?v=ncyMT3vk7H4)
