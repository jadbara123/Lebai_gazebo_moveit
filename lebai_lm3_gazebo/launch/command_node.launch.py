#!/usr/bin/env python3
"""
Final Gazebo Launch File: Simulation + MoveIt + Your Python Controller
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # -------------------------------------------------------------------------
    # 1. SETUP MOVEIT CONFIGURATION
    # -------------------------------------------------------------------------
    moveit_config = (
        MoveItConfigsBuilder("lebai_lm3", package_name="lebai_lm3_gazebo")
        .robot_description(file_path="urdf/lebai_lm3_l1_gazebo.urdf")
        .robot_description_semantic(file_path="config/lebai_lm3_l1_gazebo.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"]) 
        .to_moveit_configs()
    )

    # Convert to dictionary and apply critical fixes
    moveit_params = moveit_config.to_dict()
    
    moveit_params.update({
        # FIX: Ensure MoveIt uses the plugin CLASS NAME, not the file path
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "use_sim_time": True,
        # ensure ompl is the default pipeline if not set by the builder
        "default_planning_pipeline": "ompl" 
    })

    # -------------------------------------------------------------------------
    # 2. GAZEBO SIMULATION
    # -------------------------------------------------------------------------
    world_path = os.path.join(
        get_package_share_directory('lebai_lm3_support'),
        'worlds',
        'second_welder_wrold.world' 
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        name='spawn_entity', output='screen',
        arguments=['-entity', 'lebai_lm3', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.0']
    )

    # -------------------------------------------------------------------------
    # 3. STANDARD NODES
    # -------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        name="robot_state_publisher", output="both",
        parameters=[moveit_config.robot_description, {"use_sim_time": True}],
    )

    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="static_transform_publisher", output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{"use_sim_time": True}]
    )

    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", os.path.join(get_package_share_directory("lebai_lm3_gazebo"), "launch", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
    )

    # -------------------------------------------------------------------------
    # 4. YOUR TARGET ROBOT NODE (MoveItPy Client)
    # -------------------------------------------------------------------------
    # This node needs the FULL MoveIt configuration to initialize MoveItPy
    target_action_node = Node(
        package="lebai_lm3_gazebo",
        executable="target_robot.py",
        name="moveit_py_client",
        output="screen",
        parameters=[moveit_params], # Passing the corrected params here is crucial!
    )

    # -------------------------------------------------------------------------
    # 5. CONTROLLERS & SEQUENCING
    # -------------------------------------------------------------------------
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # DELAYS (Ensure strict startup order)
    
    # 1. Spawn Robot -> Start Joint State Broadcaster
    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_robot, on_exit=[joint_state_broadcaster])
    )

    # 2. Start Joint State Broadcaster -> Start Trajectory Controller
    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster, on_exit=[joint_trajectory_controller])
    )

    # 3. Trajectory Controller Ready -> START YOUR PYTHON SCRIPT
    # We wait for the controller to start so MoveItPy doesn't fail to connect
    delay_target_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller, 
            on_exit=[target_action_node] 
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        static_tf,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delay_jsb,
        delay_jtc,
        rviz_node,
        # Your script runs last, once everything is ready:
        delay_target_robot, 
    ])