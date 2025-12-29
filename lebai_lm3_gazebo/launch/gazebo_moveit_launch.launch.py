import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

# --- Helper Functions (From your existing code) ---

def load_file(package_name, file_path):
    """Loads a file's content as a string."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    """Loads a YAML file's content as a Python dictionary."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

# -------------------------------------------------------------

def generate_launch_description():
    
    # --- 0. Path Definitions ---
    
    # Path to the MoveIt controller list (used by move_group)
    moveit_controllers_path = os.path.join(
        get_package_share_directory("lebai_lm3_gazebo"),
        "config",
        "moveit_controllers.yaml"
    )

    # Path to the ros2_controllers definitions (used by the Controller Manager)
    ros2_controllers_config = os.path.join(
        get_package_share_directory("moveit_resources"),
        "fanuc_moveit_config", 
        "config",
        "ros2_controllers.yaml" 
    )

    # --- 1. Robot Description and Semantic Configuration ---
    
    # Load URDF/XACRO (Used to build the robot_description)
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("lebai_lm3_gazebo"),
            "urdf",
            "lebai_lm3_l1_gazebo.urdf",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    
    # Load SRDF
    robot_description_semantic_config = load_file(
        "lebai_lm3_moveit_config", "config/lebai_lm3_l1.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # --- 2. Planning and Controller Configuration ---
    
    kinematics_yaml = load_yaml(
        "lebai_lm3_moveit_config", "config/kinematics.yaml"
    )

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

    # MoveIt Controller Manager Configuration (FIXED for Ros2ControlManager)
    moveit_controllers = {
        # CRITICAL FIX 1: Pass the absolute FILE PATH
        "moveit_controller_manager": moveit_controllers_path, 
        
        # CRITICAL FIX 2: Specify the ROS 2 Control Manager plugin
        "moveit_controller_manager_plugin": "moveit_ros_controllers::Ros2ControlManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # --- 3. Node Definitions ---

    # Publish TF (robot state publisher)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    
    # --- ROS 2 CONTROL STACK (FIXED: Added Controller Manager and Spawners) ---

    # 3.1 Controller Manager Node (CORE ROS 2 Control)
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            {"use_sim_time": True},
            ros2_controllers_config, # Pass the ros2_controllers.yaml path
        ],
        output="screen",
    )
    
    # 3.2 Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    
    # 3.3 Joint Trajectory Controller Spawner
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    
    # --- END ROS 2 CONTROL STACK ---

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    # Start the TouchCameraAction custom node (YOUR SCRIPT)
    target_action_node = Node(
        package="lebai_lm3_gazebo", 
        executable="target_robot.py",
        name="touch_camera_action_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )
    
    # RViz Node
    rviz_base = get_package_share_directory("lebai_lm3_moveit_config")
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

    # --- 4. Return Launch Description ---
    return LaunchDescription(
        [
            
            # Core Robot/TF
            static_tf,
            robot_state_publisher,
            
            # ROS 2 Control Manager MUST start first
            controller_manager_node,
            
            # Spawn the required controllers
            joint_state_broadcaster_spawner,
            joint_trajectory_controller_spawner, 
            
            # Move Group Node (Requires controllers to be running)
            run_move_group_node,
            
            # RViz and Custom Action Node
            rviz_node
        ]
    )