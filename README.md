# Autoware_Calibration_Camera_Lidar
This is the calibration toolkit seperated from Autoware. 
## Environment
Ubuntu 18.04 and Ros-melodic
## Installation
1. Install the **nlopt**: https://github.com/stevengj/nlopt
2. Compile the file<br />
   ```
   git clone https://github.com/JunzWu/Autoware_Calibration_Camera_Lidar.git
   cd Autoware_Calibration_Camera_Lidar
   mkdir build
   mkdir devel
   catkin_make
   ```
## Getting Started
`roscore` in another terminal
```
source devel/setup.bash
rosrun calibration_camera_lidar calibration_toolkit
```
## Tutorial of Calibration
https://github.com/Autoware-AI/autoware.ai/wiki/Calibration
In step 3.1, if your recorded PointCloud2 topic is not `/points_raw`, you need to transfer it into `/points_raw` using the command below:
```
rosbag play bagName.bag /velodyne_points:=/points_raw
```
## Reference
https://github.com/XidianLemon/calibration_camera_lidar
