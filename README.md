# PX4-ROS2 Offboard Control (in ROS2)

This repository is ROS2 Package to control PX4 multicopter in offboard Mode.
(Based in [PX4-ROS2 User Guide](https://docs.px4.io/master/en/ros/ros2_comm.html, "ROS2 User Guide(PX4-ROS2 Bridge)"))
## 1. Setup

Install Environment
* Ubuntu 18.04 LTS or 20.04
* FastRTPS(FastDDS) & FastRTPSGen
* ROS2

(Test Environment)
* Ubuntu 18.04 LTS
* FastRTPS 1.8.4 & FastRTPSGen 1.0.4
* ROS2 Dashing

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
> If you use Ubuntu extended terminal(means no default terminal), try this command in Ubuntu default terminal.

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
$ px4_state_listener
```
