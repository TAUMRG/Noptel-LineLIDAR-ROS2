# Noptel-LineLIDAR-ROS2
ROS2 gateway for Noptel LineLIDAR

Publishes pointclouds from the lidar to /cloud topic of type PointCLoud2
## Install
Clone into a Ros2 workspace:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/TAUMRG/Noptel-LineLIDAR-ROS2.git
cd ..
```
Change lidar's IP address on the launch file:
https://github.com/TAUMRG/Noptel-LineLIDAR-ROS2/blob/3fd6df3fe0bae0c95dfdec6eacd832e89e7d221a/launch/ll.launch.py#L16


Build with colcon and source:
```
colcon build --symlink-install
source install/setup.bash
```

## Usage

Launch the LineLidar node:
```
ros2 launch Noptel_LineLIDAR_ROS2 ll.launch.py
```

Transformation can be changed with tf:
```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world LineLidar
```

Pointcloud can be visualised with Rviz:
```
rviz2
```
![sweep1](pictures/sweep1.png)

## ToDo

* Services for changing lidar settings
* Diagnostics topic

## Issues

* Setuptools depricated
