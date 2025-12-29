error running the target_robot.py code error log as follows:

WARNING:root:Cannot infer URDF from `/home/can/Aware_ws/install/lebai_lm3_gazebo/share/lebai_lm3_gazebo`. -- using config/lebai_lm3_l1_with_doser_and_camera.urdf
WARNING:root:Cannot infer SRDF from `/home/can/Aware_ws/install/lebai_lm3_gazebo/share/lebai_lm3_gazebo`. -- using config/lebai_lm3_l1_with_doser_and_camera.srdf
[INFO] [1767010692.149114780] [moveit.py.cpp_initializer]: Initialize rclcpp
[INFO] [1767010692.149144030] [moveit.py.cpp_initializer]: Initialize node parameters
[INFO] [1767010692.149156209] [moveit.py.cpp_initializer]: Initialize node and executor
1767010692.150042 [0]    python3: selected interface "lo" is not multicast-capable: disabling multicast
[INFO] [1767010692.161208953] [moveit.py.cpp_initializer]: Spin separate thread
[INFO] [1767010692.164359487] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00301875 seconds
[INFO] [1767010692.164385017] [moveit_robot_model.robot_model]: Loading robot model 'lebai_lm3_l1_with_doser_and_camera'...
[INFO] [1767010692.164392129] [moveit_robot_model.robot_model]: No root/virtual joint specified in SRDF. Assuming fixed joint
[INFO] [1767010692.371561506] [moveit.ros_planning_interface.moveit_cpp]: Listening to 'joint_states' for joint states
[INFO] [1767010692.372104784] [moveit_ros.current_state_monitor]: Listening to joint states on topic 'joint_states'
[INFO] [1767010692.373809528] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/attached_collision_object' for attached collision objects
[INFO] [1767010692.374339420] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Publishing maintained planning scene on 'monitored_planning_scene'
[INFO] [1767010692.374464664] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting planning scene monitor
[INFO] [1767010692.374928545] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to '/planning_scene'
[INFO] [1767010692.374939388] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Starting world geometry update monitor for collision objects, attached objects, octomap updates.
[INFO] [1767010692.375132051] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'collision_object'
[INFO] [1767010692.375614148] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Listening to 'planning_scene_world' for planning scene world geometry
[WARN] [1767010692.376603136] [moveit.ros.occupancy_map_monitor.middleware_handle]: Resolution not specified for Octomap. Assuming resolution = 0.1 instead
[ERROR] [1767010692.376632737] [moveit.ros.occupancy_map_monitor.middleware_handle]: No 3D sensor plugin(s) defined for octomap updates
[INFO] [1767010692.597310564] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'ompl'
[INFO] [1767010692.606744463] [moveit.ros_planning.planning_pipeline]: Using planning interface 'OMPL'
[INFO] [1767010692.609337713] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.path_tolerance' was not set. Using default value: 0.100000
[INFO] [1767010692.609355215] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.resample_dt' was not set. Using default value: 0.100000
[INFO] [1767010692.609359536] [moveit_ros.add_time_optimal_parameterization]: Param 'ompl.min_angle_change' was not set. Using default value: 0.001000
[INFO] [1767010692.609375056] [moveit_ros.fix_workspace_bounds]: Param 'ompl.default_workspace_bounds' was not set. Using default value: 10.000000
[INFO] [1767010692.609385337] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_bounds_error' was set to 0.100000
[INFO] [1767010692.609388916] [moveit_ros.fix_start_state_bounds]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[INFO] [1767010692.609396512] [moveit_ros.fix_start_state_collision]: Param 'ompl.start_state_max_dt' was not set. Using default value: 0.500000
[INFO] [1767010692.609399905] [moveit_ros.fix_start_state_collision]: Param 'ompl.jiggle_fraction' was not set. Using default value: 0.020000
[INFO] [1767010692.609408604] [moveit_ros.fix_start_state_collision]: Param 'ompl.max_sampling_attempts' was not set. Using default value: 100
[INFO] [1767010692.609417044] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Add Time Optimal Parameterization'
[INFO] [1767010692.609421844] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Resolve constraint frames to robot links'
[INFO] [1767010692.609424663] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Workspace Bounds'
[INFO] [1767010692.609453738] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Bounds'
[INFO] [1767010692.609456811] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State In Collision'
[INFO] [1767010692.609459489] [moveit.ros_planning.planning_pipeline]: Using planning request adapter 'Fix Start State Path Constraints'
[INFO] [1767010692.610611491] [moveit.ros_planning_interface.moveit_cpp]: Loading planning pipeline 'pilz_industrial_motion_planner'
[INFO] [1767010692.615785808] [moveit.pilz_industrial_motion_planner.joint_limits_aggregator]: Reading limits from namespace robot_description_planning
[INFO] [1767010692.620365598] [moveit.pilz_industrial_motion_planner]: Available plugins: pilz_industrial_motion_planner/PlanningContextLoaderCIRC pilz_industrial_motion_planner/PlanningContextLoaderLIN pilz_industrial_motion_planner/PlanningContextLoaderPTP 
[INFO] [1767010692.620393576] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderCIRC
[INFO] [1767010692.622101672] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [CIRC]
[INFO] [1767010692.622123794] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderLIN
[INFO] [1767010692.623249709] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [LIN]
[INFO] [1767010692.623272640] [moveit.pilz_industrial_motion_planner]: About to load: pilz_industrial_motion_planner/PlanningContextLoaderPTP
[INFO] [1767010692.624421884] [moveit.pilz_industrial_motion_planner]: Registered Algorithm [PTP]
[INFO] [1767010692.624460931] [moveit.ros_planning.planning_pipeline]: Using planning interface 'Pilz Industrial Motion Planner'
[INFO] [1767010692.641279351] [moveit.plugins.moveit_simple_controller_manager]: Added FollowJointTrajectory controller for joint_trajectory_controller
[INFO] [1767010692.641400986] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.641416597] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.641567122] [moveit_ros.trajectory_execution_manager]: Trajectory execution is managing controllers
Synced! Current ROS Time: 557.116s
Current ROS Time: 557.116s
Planning Linear Path (PILZ)...
[INFO] [1767010692.811473070] [moveit.pilz_industrial_motion_planner.trajectory_generator]: Generating LIN trajectory...
Current ROS Time: 557.116s
Plan successful, executing...
[INFO] [1767010692.819187562] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.819215334] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.819255148] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.819260054] [moveit.plugins.moveit_simple_controller_manager]: Returned 1 controllers in list
[INFO] [1767010692.819284397] [moveit_ros.trajectory_execution_manager]: Validating trajectory with allowed_start_tolerance 0.01
[INFO] [1767010693.819360023] [moveit_ros.current_state_monitor]: Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds. Requested time 1767010692.819291, but latest received state has time 557.566000.
Check clock synchronization if your are running ROS across multiple machines!
[WARN] [1767010693.819398040] [moveit_ros.trajectory_execution_manager]: Failed to validate trajectory: couldn't receive full current joint state within 1s
[INFO] [1767010694.023677290] [moveit.ros_planning_interface.moveit_cpp]: Deleting MoveItCpp
[INFO] [1767010694.024502660] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopped publishing maintained planning scene.
[INFO] [1767010694.026645226] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping world geometry monitor
[INFO] [1767010694.027807789] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Stopping planning scene monitor
Warning: class_loader.ClassLoader: SEVERE WARNING!!! Attempting to unload library while objects created by this loader exist in the heap! You should delete your objects before attempting to unload the library or destroying the ClassLoader. The library will NOT be unloaded.
         at line 127 in ./src/class_loader.cpp
