# PX4-ROS2 Offboard Control (in ROS2)

 - This repository is ROS2 Package to control PX4 multicopter in offboard Mode.

## 1. Setup

* Install Environment
 - FastRTPS(FastDDS) & FastRTPSGen
 - ROS2

* Build

 - Create a workspace directory using:
```
mkdir -p ~/ros2_colcon_ws/src
```

 - clone px4_msgs git repository
```
git clone https://github.com/PX4/px4_msgs.git ~/ros2_colcon_ws/src/px4_msgs
```

 - clone this git repository
```
git clone https://github.com/bbaegi/px4_ros2_offboardcontrol.git ~/ros2_colcon_ws/src/px4_ros_com
```

 - In ROS2 Workspace, use script to build
```
cd ~/ros2_colcon_ws/src/px4_ros_com/scripts
source build_ros2_workspace.bash
```
**If you use Ubuntu terminal as 'terminator', try this command in Ubuntu terminal.**
