#!/usr/bin/env python

"""
Example to launch a px4_state_listener node.

.. seealso::
    https://index.ros.org/doc/ros2/Launch-system/
"""

from launch import LaunchDescription
import launch_ros.actions
import os


def generate_launch_description():
    if os.environ['ROS_DISTRO'] != "galactic" and os.environ['ROS_DISTRO'] != "rolling":
        return LaunchDescription([
            launch_ros.actions.Node(
                package='px4_ros_com', node_executable='px4_state_listener', output='screen'),
        ])
    else:
        return LaunchDescription([
            launch_ros.actions.Node(
                package='px4_ros_com', executable='px4_state_listener', output='screen'),
        ])
