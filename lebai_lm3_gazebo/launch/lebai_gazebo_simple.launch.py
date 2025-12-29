#!/usr/bin/env python3
"""
Gazebo Launch File with ros2_control for Lebai LM3-L1 Robot
FIXED: Removed duplicate Robot State Publisher and enforced Sim Time
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    
    # 1. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Gazebo Simulation
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', 
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '/home/can/Aware_ws/src/lebai_lm3_support/worlds/second_welder_wrold.world'
        ],
        output='screen'
    )

    # 3. Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-entity', 'lebai_lm3',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    # 4. Robot Description (Xacro Processing)
    # We do this ONCE to ensure consistency
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("lebai_lm3_gazebo"),
            "urdf",
            "lebai_lm3_l1_gazebo.urdf",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    robot_description_semantic_config = load_file(
        "lebai_lm3_gazebo", "config/lebai_lm3_l1_gazebo.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # 5. Robot State Publisher (FIXED: Only one instance, with use_sim_time)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {'use_sim_time': True} # <--- CRITICAL FIX
        ],
    )

    # 6. ROS 2 Control Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delays to ensure controllers start after robot is spawned
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    # 7. MoveIt Configuration
    kinematics_yaml = load_yaml("lebai_lm3_gazebo", "config/kinematics.yaml")

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ],
            "start_state_max_bounds_error": 0.1,
            "response_adapters": ["default_planner_request_adapters/AddTimeOptimalParameterization"],
        }
    }
    ompl_planning_yaml = load_yaml("lebai_lm3_gazebo", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_config = (
        MoveItConfigsBuilder("lebai_lm3", package_name="lebai_lm3_gazebo")
        .robot_description(file_path="urdf/lebai_lm3_l1_gazebo.urdf")
        .robot_description_semantic(file_path="config/lebai_lm3_l1_gazebo.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"]) 
        .to_moveit_configs()
    )
    move_group_params = moveit_config.to_dict()

    # 8. Move Group Node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_params,
            ompl_planning_pipeline_config,
            {
                'use_sim_time': True, # Ensures MoveIt listens to Gazebo clock
                "publish_robot_description": True,
                "publish_robot_description_semantic": True
            }
        ]
    )

    # 9. RViz
    rviz_base = get_package_share_directory("lebai_lm3_gazebo")
    rviz_full_config = os.path.join(rviz_base, "launch", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
    )
    
    # 10. Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{'use_sim_time': True}] # Even static TF should respect sim time
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        spawn_robot,
        robot_state_publisher, # Only ONE definition now
        static_tf,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
        run_move_group_node,
        rviz_node
    ])