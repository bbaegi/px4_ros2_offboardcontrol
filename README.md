# PX4-ROS2 Offboard Control (in ROS2)

This repository is ROS2 Package to control PX4 multicopter in offboard Mode.

## 1. Setup

Install Environment
* FastRTPS(FastDDS) & FastRTPSGen
* ROS2

Build
* Create a workspace directory using:
```bash
$ mkdir -p ~/ros2_colcon_ws/src
```

* Clone px4_msgs git repository
```bash
$ git clone https://github.com/PX4/px4_msgs.git ~/ros2_colcon_ws/src/px4_msgs
```

* Clone this git repository
```bash
$ git clone https://github.com/bbaegi/px4_ros2_offboardcontrol.git ~/ros2_colcon_ws/src/px4_ros_com
```

* In ROS2 Workspace, use script to build
```bash
$ cd ~/ros2_colcon_ws/src/px4_ros_com/scripts
$ source build_ros2_workspace.bash
```
> **Is Your terminal forced to close?**
> If you use Ubuntu terminal as 'terminator', try this command in Ubuntu default terminal.

## 2. Usage

1. Start Gazebo Simulation
```bash
$ make px4_sitl_rtps gazebo
```

2. Activate micrortps_agent daemon (in a new terminal)
```bash
$ source ~/ros2_colcon_ws/install/setup.bash
$ micrortps_agent -t UDP
```

3. Start a listener node using launch file
```bash
$ source ~/ros2_colcon_ws/install/setup.bash
$ ros2 launch px4_ros_com px4_state_listener.launch.py
```
