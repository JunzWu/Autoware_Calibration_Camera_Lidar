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
1. `roscore` in another terminal
2. `rosrun calibration_camera_lidar calibration_toolkit`
