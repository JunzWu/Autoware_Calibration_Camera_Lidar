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
1. `roscore` in another terminal. 
2. Play the rosbag. if your recorded PointCloud2 topic is not `/points_raw`, you need to transfer it into `/points_raw` using the command below". Once the rosbag is playing, quickly pause it using the space button.
   ```
   rosbag play bagName.bag /YourPointCloud2Topic:=/points_raw
   ```
 
3. Open the toolkit
   ```
   source devel/setup.bash
   rosrun calibration_camera_lidar calibration_toolkit
   ```
4. Keep playing the rosbag until the chessboard is in a good position, then pause it again.
## Tutorial of Calibration
https://www.youtube.com/watch?v=pfBmfgHf6zg

The CalibrationToolkit_Manual.pdf is uploaded which also include the calibration process in this toolkit.

**Remember to stay in Mode 1 after pressing the grab button, if you cannot extract the plane of the board, press "↑ ↓ ← →" to adjust the postion.**

## Reference
https://github.com/XidianLemon/calibration_camera_lidar
