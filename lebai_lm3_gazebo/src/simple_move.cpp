#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/task_constructor/task.h>

// Stages
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>

// Solvers
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

// Scene & Execution
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Service Interface
#include "lebai_lm3_gazebo/srv/execute_waypoints.hpp"

#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace moveit::task_constructor;

class MTCServiceNode : public rclcpp::Node {
public:
    MTCServiceNode() : Node("mtc_service_node") {
        // 1. Declare Params Safely
        // We check 'has_parameter' first to avoid crashing if the launch file already declared them
        if (!this->has_parameter("use_sim_time")) {
            this->declare_parameter("use_sim_time", true);
        }
        
        // 2. Load Robot Model Logic 
        load_robot_model();
    }

    // NEW: Initialization function to safely use shared_from_this()
    // This must be called AFTER the node is created in main()
    void initialize() {
        // 3. Setup Scene Monitor
        // shared_from_this() works here because the node pointer now exists
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(shared_from_this(), "robot_description");
        psm_->startStateMonitor("/joint_states");
        psm_->startSceneMonitor();

        // 4. Create Service
        using ServiceT = lebai_lm3_gazebo::srv::ExecuteWaypoints; 
        
        service_ = this->create_service<ServiceT>(
            "execute_waypoints",
            std::bind(&MTCServiceNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service 'execute_waypoints' is ready!");
    }

private:
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;
    rclcpp::Service<lebai_lm3_gazebo::srv::ExecuteWaypoints>::SharedPtr service_;

    void load_robot_model() {
        std::string pkg_share = ament_index_cpp::get_package_share_directory("lebai_lm3_gazebo");
        
        // 1. Load URDF and SRDF (Existing Logic)
        auto load_file = [](const std::string& path) -> std::string {
            std::ifstream file(path); 
            return file.is_open() ? std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>()) : "";
        };

        if (!this->has_parameter("robot_description")) {
             std::string urdf_path = pkg_share + "/urdf/lebai_lm3_l1_gazebo.urdf";
             std::string content = load_file(urdf_path);
             if(!content.empty()) this->declare_parameter("robot_description", content);
        }
        
        if (!this->has_parameter("robot_description_semantic")) {
             std::string srdf_path = pkg_share + "/config/lebai_lm3_l1_gazebo.srdf";
             std::string content = load_file(srdf_path);
             if(!content.empty()) this->declare_parameter("robot_description_semantic", content);
        }

        // 2. Load Kinematics.yaml (NEW LOGIC)
        std::string kin_yaml_path = pkg_share + "/config/kinematics.yaml";
        try {
            YAML::Node kin_yaml = YAML::LoadFile(kin_yaml_path);
            
            // Loop through groups (e.g., "manipulator")
            for (YAML::const_iterator it_group = kin_yaml.begin(); it_group != kin_yaml.end(); ++it_group) {
                std::string group_name = it_group->first.as<std::string>();
                
                // Loop through parameters (e.g., "kinematics_solver", "timeout")
                for (YAML::const_iterator it_param = it_group->second.begin(); it_param != it_group->second.end(); ++it_param) {
                    std::string param_name = it_param->first.as<std::string>();
                    std::string full_param_name = "robot_description_kinematics." + group_name + "." + param_name;
                    
                    if (!this->has_parameter(full_param_name)) {
                        auto value_node = it_param->second;
                        
                        // Handle Double values (resolution, timeout) vs String values (plugin name)
                        try {
                            // Try to read as double first
                            double d_val = value_node.as<double>();
                            this->declare_parameter(full_param_name, d_val);
                        } catch (...) {
                            // Fallback to string if not a number
                            this->declare_parameter(full_param_name, value_node.as<std::string>());
                        }
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "Loaded kinematics parameters from %s", kin_yaml_path.c_str());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load kinematics.yaml: %s", e.what());
        }
    }

    void handle_service(const std::shared_ptr<lebai_lm3_gazebo::srv::ExecuteWaypoints::Request> request,
                        std::shared_ptr<lebai_lm3_gazebo::srv::ExecuteWaypoints::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "Received request with %zu waypoints", request->poses.size());

        if (request->poses.empty()) {
            response->success = false;
            response->message = "No poses provided in request";
            return;
        }

        Task t;
        t.stages()->setName("Waypoint Task");
        
        // Safe to use shared_from_this() here because handle_service only runs after init
        t.loadRobotModel(shared_from_this());

        const std::string group = "manipulator";
        const std::string eef = "link_6";

        // Solver
        auto cartesian_solver = std::make_shared<solvers::CartesianPath>();
        cartesian_solver->setStepSize(0.01);
        cartesian_solver->setJumpThreshold(0.0);

        // STAGE 1: Current State
        {
             planning_scene_monitor::LockedPlanningSceneRO ro_scene(psm_);
             if (!ro_scene) {
                 RCLCPP_ERROR(this->get_logger(), "Failed to lock planning scene");
                 response->success = false;
                 response->message = "Scene lock failed";
                 return;
             }
             auto scene = planning_scene::PlanningScene::clone(ro_scene);
             auto fixed = std::make_unique<stages::FixedState>("Start");
             fixed->setState(scene);
             t.add(std::move(fixed));
        }

        // STAGE 2: Loop through Waypoints
        int count = 0;
        for (const auto& target_pose : request->poses) {
            std::string stage_name = "Waypoint_" + std::to_string(++count);
            
            auto stage = std::make_unique<stages::MoveTo>(stage_name, cartesian_solver);
            stage->setGroup(group);
            stage->setIKFrame(eef);
            
            geometry_msgs::msg::PoseStamped stamped_pose;
            stamped_pose.header.frame_id = "base_link"; // Ensure this frame matches your URDF
            stamped_pose.pose = target_pose;
            
            stage->setGoal(stamped_pose);
            t.add(std::move(stage));
        }

        // PLAN
        try {
            if (t.plan()) {
                RCLCPP_INFO(this->get_logger(), "Planning Success. Executing...");
                
                // EXECUTE
                moveit_task_constructor_msgs::msg::Solution solution_msg;
                t.solutions().front()->toMsg(solution_msg);
                
                moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), group);
                
                for (const auto& sub_trajectory : solution_msg.sub_trajectory) {
                    if (!sub_trajectory.trajectory.joint_trajectory.points.empty()) {
                        moveit_msgs::msg::RobotTrajectory traj;
                        traj.joint_trajectory = sub_trajectory.trajectory.joint_trajectory;
                        move_group.execute(traj);
                    }
                }
                
                response->success = true;
                response->message = "Execution Complete";
            } else {
                response->success = false;
                response->message = "Planning Failed";
                RCLCPP_ERROR(this->get_logger(), "Planning Failed");
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = e.what();
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    // 1. Create the Node Shared Pointer
    auto node = std::make_shared<MTCServiceNode>();
    
    // 2. Initialize it (This runs logic that needs shared_from_this)
    node->initialize();
    
    // 3. Spin with MultiThreadedExecutor
    // Multi-threading is required so the MoveGroup action client 
    // can run while the service callback is still active.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}