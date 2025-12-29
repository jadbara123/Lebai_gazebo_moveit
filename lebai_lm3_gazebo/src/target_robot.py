#!/usr/bin/env python3
import rclpy
import sys
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path
from moveit.planning import PlanRequestParameters
from geometry_msgs.msg import PoseStamped

def interpolate_pose_linear(start_pose, end_pose, alpha):
    """Linearly interpolate between two poses (0.0 <= alpha <= 1.0)"""
    p = PoseStamped()
    p.header = start_pose.header
    
    # Linear interpolation of Position
    p.pose.position.x = start_pose.pose.position.x * (1 - alpha) + end_pose.pose.position.x * alpha
    p.pose.position.y = start_pose.pose.position.y * (1 - alpha) + end_pose.pose.position.y * alpha
    p.pose.position.z = start_pose.pose.position.z * (1 - alpha) + end_pose.pose.position.z * alpha
    
    # Slerp (Spherical Linear Interpolation) for Orientation would be better here,
    # but for simple cases, copying the target orientation or simple lerp is often 'okay'.
    # For strict correctness, you need a quaternion slerp function.
    p.pose.orientation = end_pose.pose.orientation 
    return p

def main():
    if '--ros-args' not in sys.argv:
        sys.argv.append('--ros-args')

    if not any('use_sim_time' in arg for arg in sys.argv):
        sys.argv.extend(['-p', 'use_sim_time:=true'])

    rclpy.init(args=sys.argv)
    
    # 2. Build Config
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="lebai_lm3_l1_with_doser_and_camera", 
            package_name="lebai_lm3_gazebo"
        )
        .robot_description(file_path="urdf/lebai_lm3_l1_gazebo.urdf")
        .robot_description_semantic(Path("config") / "lebai_lm3_l1_gazebo.srdf")
        .moveit_cpp(Path("config") / "moveit_cpp.yaml")
        .to_moveit_configs()
    ).to_dict()

    if "moveit_cpp" in moveit_config:
        moveit_config["moveit_cpp"]["use_sim_time"] = True
    else:
        moveit_config["moveit_cpp"] = {"use_sim_time": True}

    moveit = MoveItPy(node_name="moveit_py_client", config_dict=moveit_config)

    test_node = rclpy.create_node("test_clock")
    # Wait a moment for /clock to publish
    while rclpy.ok():
        rclpy.spin_once(test_node, timeout_sec=0.1)
        current_time = test_node.get_clock().now()
        
        if current_time.nanoseconds > 0:
            print(f"Synced! Current ROS Time: {current_time.nanoseconds / 1e9}s")
            break
        
    if current_time.nanoseconds == 0:
         print("WARNING: Time is still 0. Gazebo is definitely paused or crashed.")
    
    current_time = test_node.get_clock().now()
    print(f"Current ROS Time: {current_time.nanoseconds / 1e9}s")
    
    if current_time.nanoseconds == 0:
         print("WARNING: Time is 0. Is Gazebo paused?")
    # ------------------------------

    arm = moveit.get_planning_component("manipulator")
   
    # 4. Define Target (Absolute Coordinate)
    target_pose = PoseStamped()
    target_pose.header.frame_id = "base_link"
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = -0.3
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.w = 1.0 # Ensure this is a valid quaternion!

    # 5. Set the Goal
    arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="link_6")

    # 5. Create the Parameters Object
    # FIX: Pass the 'moveit' object (MoveItPy instance), NOT 'arm'.
    # FIX: Do not pass "LIN" here. The constructor only takes 1 argument.
    params = PlanRequestParameters(moveit)

    # 6. Configure for PILZ
    params.planning_pipeline = "pilz_industrial_motion_planner"
    params.planner_id = "LIN"
    
    # Optional: Safety scaling
    params.max_velocity_scaling_factor = 0.1
    params.max_acceleration_scaling_factor = 0.1

    print("Planning Linear Path (PILZ)...")
    
    # 6. Plan using the parameters
    plan_result = arm.plan(parameters=params)
    current_time = test_node.get_clock().now()
    print(f"Current ROS Time: {current_time.nanoseconds / 1e9}s")
    
    if plan_result:
        print("Plan successful, executing...")
        moveit.execute("manipulator", plan_result.trajectory, blocking=True)
    else:
        print("Planning failed!")

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()