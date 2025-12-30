#!/usr/bin/env python3
"""
Gazebo Launch File with ros2_control for Lebai LM3-L1 Robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


#move joint
gazebo = ExecuteProcess(
    cmd=[
        'ros2', 'action', 'send_goal',
        '/joint_trajectory_controller/follow_joint_trajectory',
        'control_msgs/action/FollowJointTrajectory',
        'trajectory: {joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], points: [{positions: [0.5, -0.5, 0.5, 0.0, 0.5, 0.0], time_from_start: {sec: 2}}]}'
    ],
    output='screen'
)

gazebo

