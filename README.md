# ndt_slam

## How to use

### 1. Build
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/RyuYamamoto/ndt_slam.git
cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. Run
launch ndt_slam.
```bash
ros2 launch ndt_slam ndt_slam.launch.xml
```

play rosbag file.
```bash
ros2 bag play <ROSBAG PATH> --clock 100
```

save pcd map.
```bash
ros2 service call /save_map std_srvs/srv/Empty
# PCD Map are saved in /tmp/map.pcd
```

<div align="center">
<img src="img/ndt-mapping-at-tsukuba.gif" width="600">
</div>

## Dependency package
[ndt_omp](https://github.com/RyuYamamoto/ndt_omp/tree/ros2-galactic)