/*=============================== Stage Controller for Prestage of Two Robots =======================================*/
// Author: Ruize Cao
// Date: 2025/10/1

#include "ocs2_multi_robot/robot_interface/TwoQuadPreStageController.h"

using namespace ocs2::multi_robot;

TwoQuadPreStageController::TwoQuadPreStageController(){
  // ROS Interface
  nh_ = ros::NodeHandle();
  combined_observation_sub_ = nh_.subscribe("/combined_observation", 1, &TwoQuadPreStageController::CombinedObservationCallback, this);
  prestage_mode_service_ = nh_.advertiseService("/PreStageMode", &TwoQuadPreStageController::prestageModeServiceCallback, this);

  // Get robot namespaces
  nh_.getParam("robot1_ns", robot1_ns_);
  nh_.getParam("robot2_ns", robot2_ns_);
  if(robot1_ns_.empty() || robot2_ns_.empty()){
    ROS_ERROR_STREAM("Robot namespace is not set!");
    ros::shutdown();
  }

  // Subscribe to current end-effector poses
  robot1_ee_pose_sub_ = nh_.subscribe("/" + robot1_ns_ + "/z1/current_ee_pose", 10, &TwoQuadPreStageController::Robot1EEPoseCallback, this);
  robot2_ee_pose_sub_ = nh_.subscribe("/" + robot2_ns_ + "/z1/current_ee_pose", 10, &TwoQuadPreStageController::Robot2EEPoseCallback, this);

  // Publishers for arm trajectories only
  robot1_arm_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot1_ns_ + "/z1/traj_points", 1);
  robot2_arm_target_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/" + robot2_ns_ + "/z1/traj_points", 1);

  // Publishers for arm wrench commands
  robot1_arm_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/" + robot1_ns_ + "/z1/arm_wrench_cmd", 1);
  robot2_arm_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/" + robot2_ns_ + "/z1/arm_wrench_cmd", 1);
  
  robot1_gripper_pub_ = nh_.advertise<std_msgs::Float64>("/" + robot1_ns_ + "/z1/gripper_cmd", 1);
  robot2_gripper_pub_ = nh_.advertise<std_msgs::Float64>("/" + robot2_ns_ + "/z1/gripper_cmd", 1);

  // Visualization publishers
  robot1_traj_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/" + robot1_ns_ + "/arm_trajectory_viz", 1);
  robot2_traj_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/" + robot2_ns_ + "/arm_trajectory_viz", 1);

  // External MPC signal publisher
  external_mpc_signal_pub_ = nh_.advertise<std_msgs::Bool>("/external_mpc_signal", 1);

  // Initialize state
  current_mode_ = PreStageMode::IDLE;
  fsm_running_ = false;
  robot1_state_.setZero(SINGLE_ROBOT_STATE_DIM);
  robot2_state_.setZero(SINGLE_ROBOT_STATE_DIM);
  cargo_state_.setZero(CARGO_STATE_DIM);
  
  // Initialize end-effector pose flags
  robot1_ee_pose_received_ = false;
  robot2_ee_pose_received_ = false;
  
  // Initialize trajectory variables
  trajectory_index_ = 0;
  trajectory_initialized_ = false;
}

TwoQuadPreStageController::~TwoQuadPreStageController(){
  stop();
}

void TwoQuadPreStageController::CombinedObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg){
  SystemObservation combined_observation = ros_msg_conversions::readObservationMsg(*msg);
  
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot1_state_ = combined_observation.state.head(SINGLE_ROBOT_STATE_DIM);
  robot2_state_ = combined_observation.state.segment(SINGLE_ROBOT_STATE_DIM, SINGLE_ROBOT_STATE_DIM);
  cargo_state_ = combined_observation.state.tail(CARGO_STATE_DIM);
}

void TwoQuadPreStageController::Robot1EEPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot1_current_ee_pose_ = *msg;
  robot1_ee_pose_received_ = true;
}

void TwoQuadPreStageController::Robot2EEPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  std::lock_guard<std::mutex> lock(state_mutex_);
  robot2_current_ee_pose_ = *msg;
  robot2_ee_pose_received_ = true;
}

void TwoQuadPreStageController::run(){
  fsm_running_ = true;
  float freq = 100.0; // Hz
  ros::Rate rate(freq);
  
  // Start single FSM thread
  fsm_thread_ = std::thread(&TwoQuadPreStageController::fsmThread, this);
  
  while(ros::ok() && fsm_running_){
    ros::spinOnce();
    rate.sleep();
  }
  
  // Clean exit
  stop();
}

void TwoQuadPreStageController::stop(){
  fsm_running_ = false;
  if(fsm_thread_.joinable()){
    fsm_thread_.join();
  }
}

void TwoQuadPreStageController::fsmThread(){
  float freq = 50.0; // Hz
  ros::Rate rate(freq);
  int print_counter = 0;
  
  while(ros::ok() && fsm_running_){
    PreStageMode current_mode;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      current_mode = current_mode_;
    }
    
    switch(current_mode){
      case PreStageMode::IDLE:
        runIdle();
        break;
      case PreStageMode::ARMAPPROACH:
        runArmApproach();
        break;
      case PreStageMode::LIFT:
        runLift();
        break;
      case PreStageMode::EXTERNAL_MPC_SIGNAL:
        runExternalMpcSignal();
        break;
    }
    
    // Debug output every 10 second
    if(print_counter %  50 * (int)(freq) == 0){
      printf("[PreStageController] Current mode: %s\n", getModeString(current_mode).c_str());
    }
    print_counter++;
    
    rate.sleep();
  }
}

void TwoQuadPreStageController::runIdle(){
  // Do nothing
}

void TwoQuadPreStageController::runArmApproach(){
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Initialize trajectory on first enter
  if (!trajectory_initialized_) {
    // Check if we have received both robots' current end-effector poses
    if (!robot1_ee_pose_received_ || !robot2_ee_pose_received_) {
      printf("[PreStageController] Waiting for current end-effector poses from both robots...\n");
      return;
    }
    
    // Calculate handle positions in world frame based on cargo state
    float cargo_yaw = cargo_state_(6);
    float handle_x_world = HANDLE_X_OFFSET * cos(cargo_yaw) - HANDLE_Y_OFFSET * sin(cargo_yaw);
    float handle_y_world = HANDLE_X_OFFSET * sin(cargo_yaw) + HANDLE_Y_OFFSET * cos(cargo_yaw);
    
    // Create trajectory with more points for smaller steps
    const int num_points = 200;  // Decreased from 500 to 200
    const int lift_points = 60;  // First 30% for lifting up
    const int approach_points = 140; // Last 70% for horizontal approach
    
    // Get current arm positions from actual end-effector poses
    float robot1_start_x = robot1_current_ee_pose_.pose.position.x;
    float robot1_start_y = robot1_current_ee_pose_.pose.position.y;
    float robot1_start_z = robot1_current_ee_pose_.pose.position.z;
    geometry_msgs::Quaternion robot1_start_orientation = robot1_current_ee_pose_.pose.orientation;
    
    float robot2_start_x = robot2_current_ee_pose_.pose.position.x;
    float robot2_start_y = robot2_current_ee_pose_.pose.position.y;
    float robot2_start_z = robot2_current_ee_pose_.pose.position.z;
    geometry_msgs::Quaternion robot2_start_orientation = robot2_current_ee_pose_.pose.orientation;
    
    float target_z = cargo_state_(2) + HANDLE_Z_OFFSET - 0.015;
    
    // Clear and reserve space for trajectories
    robot1_trajectory_.clear();
    robot2_trajectory_.clear();
    robot1_trajectory_.reserve(num_points);
    robot2_trajectory_.reserve(num_points);
    
    for(int i = 0; i < num_points; i++){
      geometry_msgs::PoseStamped robot1_pose, robot2_pose;
      robot1_pose.header.stamp = ros::Time::now();
      robot1_pose.header.frame_id = "world";
      robot2_pose.header.stamp = ros::Time::now();
      robot2_pose.header.frame_id = "world";
      
      if(i < lift_points){
        // Phase 1: Lift up to target height
        float lift_ratio = static_cast<float>(i) / lift_points;
        
        // Robot 1: Lift up vertically
        robot1_pose.pose.position.x = robot1_start_x;
        robot1_pose.pose.position.y = robot1_start_y;
        robot1_pose.pose.position.z = robot1_start_z + (target_z - robot1_start_z) * lift_ratio;
        robot1_pose.pose.orientation = robot1_start_orientation;
        
        // Robot 2: Lift up vertically
        robot2_pose.pose.position.x = robot2_start_x;
        robot2_pose.pose.position.y = robot2_start_y;
        robot2_pose.pose.position.z = robot2_start_z + (target_z - robot2_start_z) * lift_ratio;
        robot2_pose.pose.orientation = robot2_start_orientation;
        
      } else {
        // Phase 2: Approach horizontally at target height
        float approach_ratio = static_cast<float>(i - lift_points) / approach_points;
        
        // Target positions
        float robot1_target_x = cargo_state_(0) - handle_x_world + 0.04;
        float robot1_target_y = cargo_state_(1) - handle_y_world;
        float robot2_target_x = cargo_state_(0) + handle_x_world - 0.03;
        float robot2_target_y = cargo_state_(1) + handle_y_world;
        
        // Robot 1: Approach target horizontally
        robot1_pose.pose.position.x = robot1_start_x + (robot1_target_x - robot1_start_x) * approach_ratio;
        robot1_pose.pose.position.y = robot1_start_y + (robot1_target_y - robot1_start_y) * approach_ratio;
        robot1_pose.pose.position.z = target_z;
        
        // Gradually rotate to face the handle
        float theta = cargo_state_(6); // Use cargo yaw as theta
        robot1_pose.pose.orientation.x = 0.0;
        robot1_pose.pose.orientation.y = 0.0;
        robot1_pose.pose.orientation.z = sin(theta / 2.0);
        robot1_pose.pose.orientation.w = cos(theta / 2.0);
        
        // robot1_pose.pose.orientation = robot1_start_orientation; // Default to start orientation
        
        // Robot 2: Approach target horizontally
        robot2_pose.pose.position.x = robot2_start_x + (robot2_target_x - robot2_start_x) * approach_ratio;
        robot2_pose.pose.position.y = robot2_start_y + (robot2_target_y - robot2_start_y) * approach_ratio;
        robot2_pose.pose.position.z = target_z;

        theta = cargo_state_(6) + M_PI * 0.99; // Use cargo yaw as theta
        robot2_pose.pose.orientation.x = 0.0;
        robot2_pose.pose.orientation.y = 0.0;
        robot2_pose.pose.orientation.z = sin(theta / 2.0);
        robot2_pose.pose.orientation.w = cos(theta / 2.0);
        
        // Face opposite direction for robot 2
        robot2_pose.pose.orientation = robot2_start_orientation; // Default to start orientation
      }
      
      robot1_trajectory_.push_back(robot1_pose);
      robot2_trajectory_.push_back(robot2_pose);
    }
    
    trajectory_index_ = 0;
    trajectory_initialized_ = true;
    
    printf("[PreStageController] Trajectory initialized with %d points\n", num_points);
  }
  
  // Execute trajectory step by step
  if (trajectory_index_ < robot1_trajectory_.size()) {
    // Publish current trajectory point
    robot1_arm_target_pub_.publish(robot1_trajectory_[trajectory_index_]);
    robot2_arm_target_pub_.publish(robot2_trajectory_[trajectory_index_]);
    
    // Visualize trajectory progress for both robots
    visualizeTrajectoryProgress(robot1_trajectory_[trajectory_index_], trajectory_index_, robot1_traj_viz_pub_, "robot1");
    visualizeTrajectoryProgress(robot2_trajectory_[trajectory_index_], trajectory_index_, robot2_traj_viz_pub_, "robot2");
    
    // Open grippers
    std_msgs::Float64 gripper_open;
    gripper_open.data = GRIPPER_OPEN_ANGLE;
    robot1_gripper_pub_.publish(gripper_open);
    robot2_gripper_pub_.publish(gripper_open);
    
    trajectory_index_++;
    
    if (trajectory_index_ % 10 == 0) {
      printf("[PreStageController] Trajectory progress: %d/%zu\n", trajectory_index_, robot1_trajectory_.size());
    }
  } else {
    // Trajectory completed, return to IDLE
    current_mode_ = PreStageMode::IDLE;
    trajectory_initialized_ = false;
    printf("[PreStageController] Arm approach completed, returning to IDLE\n");
  }
}

void TwoQuadPreStageController::runLift(){
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  // Initialize trajectory on first enter
  if (!trajectory_initialized_) {
    // Check if we have received both robots' current end-effector poses
    if (!robot1_ee_pose_received_ || !robot2_ee_pose_received_) {
      printf("[PreStageController] Waiting for current end-effector poses from both robots...\n");
      return;
    }
    
    // Create trajectory with 50 points for lifting
    const int num_points = 200;
    const int gripper_close_points = 20;  // First 20% for closing grippers
    const int lift_points = 180; // Last 80% for lifting up
    
    // Get current arm positions from actual end-effector poses
    float robot1_start_x = robot1_current_ee_pose_.pose.position.x;
    float robot1_start_y = robot1_current_ee_pose_.pose.position.y;
    float robot1_start_z = robot1_current_ee_pose_.pose.position.z;
    geometry_msgs::Quaternion robot1_start_orientation = robot1_current_ee_pose_.pose.orientation;
    
    float robot2_start_x = robot2_current_ee_pose_.pose.position.x;
    float robot2_start_y = robot2_current_ee_pose_.pose.position.y;
    float robot2_start_z = robot2_current_ee_pose_.pose.position.z;
    geometry_msgs::Quaternion robot2_start_orientation = robot2_current_ee_pose_.pose.orientation;
    
    // Target height: current position + 0.1m
    float target_z = robot1_start_z + 0.3f;
    
    // Clear and reserve space for trajectories
    robot1_trajectory_.clear();
    robot2_trajectory_.clear();
    robot1_trajectory_.reserve(num_points);
    robot2_trajectory_.reserve(num_points);
    
    for(int i = 0; i < num_points; i++){
      geometry_msgs::PoseStamped robot1_pose, robot2_pose;
      robot1_pose.header.stamp = ros::Time::now();
      robot1_pose.header.frame_id = "world";
      robot2_pose.header.stamp = ros::Time::now();
      robot2_pose.header.frame_id = "world";
      
      if(i < gripper_close_points){
        // Phase 1: Close grippers while maintaining position
        
        // Robot 1: Keep position constant during gripper closing
        robot1_pose.pose.position.x = robot1_start_x;
        robot1_pose.pose.position.y = robot1_start_y;
        robot1_pose.pose.position.z = robot1_start_z;
        robot1_pose.pose.orientation = robot1_start_orientation;
        
        // Robot 2: Keep position constant during gripper closing
        robot2_pose.pose.position.x = robot2_start_x;
        robot2_pose.pose.position.y = robot2_start_y;
        robot2_pose.pose.position.z = robot2_start_z;
        robot2_pose.pose.orientation = robot2_start_orientation;
        
      } else {
        // Phase 2: Lift up by 0.1m
        float lift_ratio = static_cast<float>(i - gripper_close_points) / lift_points;
        
        // Robot 1: Lift up vertically
        robot1_pose.pose.position.x = robot1_start_x;
        robot1_pose.pose.position.y = robot1_start_y;
        robot1_pose.pose.position.z = robot1_start_z + (target_z - robot1_start_z) * lift_ratio;
        robot1_pose.pose.orientation = robot1_start_orientation;
        
        // Robot 2: Lift up vertically
        robot2_pose.pose.position.x = robot2_start_x;
        robot2_pose.pose.position.y = robot2_start_y;
        robot2_pose.pose.position.z = robot2_start_z + (target_z - robot2_start_z) * lift_ratio;
        robot2_pose.pose.orientation = robot2_start_orientation;
      }
      
      robot1_trajectory_.push_back(robot1_pose);
      robot2_trajectory_.push_back(robot2_pose);
    }
    
    trajectory_index_ = 0;
    trajectory_initialized_ = true;
    
    printf("[PreStageController] Lift trajectory initialized with %d points\n", num_points);
  }
  

  // Execute trajectory step by step
  if (trajectory_index_ < robot1_trajectory_.size()) {
    
    // HardCode Arm Wrench Command to Assist Lifting
    geometry_msgs::WrenchStamped arm_wrench_cmd;
    arm_wrench_cmd.header.stamp = ros::Time::now();
    arm_wrench_cmd.header.frame_id = "world";
    arm_wrench_cmd.wrench.force.x = 0.0;
    arm_wrench_cmd.wrench.force.y = 0.0;
    arm_wrench_cmd.wrench.force.z = -1.0 * 3.0 * 9.81 * 0.5; // Half of robot weight to assist lifting
    arm_wrench_cmd.wrench.torque.x = 0.0;
    arm_wrench_cmd.wrench.torque.y = 0.0;
    arm_wrench_cmd.wrench.torque.z = 0.0;
    robot1_arm_wrench_pub_.publish(arm_wrench_cmd);
    robot2_arm_wrench_pub_.publish(arm_wrench_cmd);

    // Publish current trajectory point
    robot1_arm_target_pub_.publish(robot1_trajectory_[trajectory_index_]);
    robot2_arm_target_pub_.publish(robot2_trajectory_[trajectory_index_]);
    
    // Visualize trajectory progress for both robots
    visualizeTrajectoryProgress(robot1_trajectory_[trajectory_index_], trajectory_index_, robot1_traj_viz_pub_, "robot1");
    visualizeTrajectoryProgress(robot2_trajectory_[trajectory_index_], trajectory_index_, robot2_traj_viz_pub_, "robot2");
    
    // Handle gripper commands based on phase
    std_msgs::Float64 gripper_cmd;
    if (trajectory_index_ < 10) {
      // Phase 1: Close grippers
      gripper_cmd.data = GRIPPER_CLOSE_ANGLE;
    } else {
      // Phase 2: Keep grippers closed
      gripper_cmd.data = GRIPPER_CLOSE_ANGLE;
    }
    robot1_gripper_pub_.publish(gripper_cmd);
    robot2_gripper_pub_.publish(gripper_cmd);
    
    trajectory_index_++;
    
    if (trajectory_index_ % 5 == 0) {
      printf("[PreStageController] Lift progress: %d/%zu\n", trajectory_index_, robot1_trajectory_.size());
    }
  } else {
    // Trajectory completed, return to IDLE
    current_mode_ = PreStageMode::IDLE;
    trajectory_initialized_ = false;
    printf("[PreStageController] Lift completed, returning to IDLE\n");
  }
}

void TwoQuadPreStageController::runExternalMpcSignal(){
  std_msgs::Bool msg;
  msg.data = true;
  external_mpc_signal_pub_.publish(msg);
  current_mode_ = PreStageMode::IDLE;
  printf("[PreStageController] External MPC signal sent, returning to IDLE\n");
}

/************************************* Visualization ******************************************/
void TwoQuadPreStageController::visualizeTrajectoryProgress(const geometry_msgs::PoseStamped& current_pose, int pose_index, 
                                                          ros::Publisher& viz_pub, const std::string& robot_name) {
  visualization_msgs::MarkerArray marker_array;
  
  // Transform pose from world frame to odom frame for visualization
  geometry_msgs::PoseStamped odom_pose = current_pose;
  odom_pose.header.frame_id = "odom";
  
  // Note: Since trajectory is computed in world coordinates but we want to visualize in odom,
  // and there's no direct transform between world and odom, we assume they are aligned
  // (same coordinate system). If they are not aligned in your setup, you would need
  // to apply the appropriate transform here.
  
  // Create current position marker
  visualization_msgs::Marker current_marker;
  current_marker.header.frame_id = "odom";  // Back to odom as requested
  current_marker.header.stamp = ros::Time::now();
  current_marker.ns = robot_name + "_current";
  current_marker.id = 0;
  current_marker.type = visualization_msgs::Marker::SPHERE;
  current_marker.action = visualization_msgs::Marker::ADD;
  
  current_marker.pose = odom_pose.pose;
  current_marker.scale.x = 0.05;
  current_marker.scale.y = 0.05;
  current_marker.scale.z = 0.05;
  
  // Set color based on robot name
  if (robot_name == "robot1") {
    current_marker.color.r = 1.0; // Red for robot1
    current_marker.color.g = 0.0;
    current_marker.color.b = 0.0;
  } else {
    current_marker.color.r = 0.0; // Blue for robot2
    current_marker.color.g = 0.0;
    current_marker.color.b = 1.0;
  }
  current_marker.color.a = 1.0;
  current_marker.lifetime = ros::Duration(1.0);
  
  marker_array.markers.push_back(current_marker);
  
  // Create progress text marker
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "odom";  // Back to odom as requested
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = robot_name + "_progress";
  text_marker.id = 1;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  
  text_marker.pose.position.x = odom_pose.pose.position.x;
  text_marker.pose.position.y = odom_pose.pose.position.y;
  text_marker.pose.position.z = odom_pose.pose.position.z + 0.1; // Offset above the sphere
  text_marker.pose.orientation.w = 1.0;
  
  text_marker.scale.z = 0.03; // Text size
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.lifetime = ros::Duration(1.0);
  
  // Set progress text
  std::stringstream ss;
  ss << robot_name << ": " << pose_index;
  text_marker.text = ss.str();
  
  marker_array.markers.push_back(text_marker);
  
  // Publish the markers
  viz_pub.publish(marker_array);
}

/************************************* Service Handle ******************************************/
bool TwoQuadPreStageController::prestageModeServiceCallback(ocs2_multi_robot::PreStageMode::Request &req, ocs2_multi_robot::PreStageMode::Response &res){
  int mode_int = req.mode;
  
  // Check for invalid mode values
  if(mode_int < 0 || mode_int > static_cast<int>(PreStageMode::EXTERNAL_MPC_SIGNAL)){
    res.result = false;
    res.message = "Invalid mode requested. Valid modes: 0(IDLE), 1(ARMAPPROACH), 2(LIFT)";
    ROS_WARN_STREAM("[PreStageController] Invalid mode requested: " << mode_int);
    return false;
  }
  
  PreStageMode mode = static_cast<PreStageMode>(mode_int);
  
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_mode_ = mode;
  }
  
  res.result = true;
  res.message = "All robots mode set to " + getModeString(mode);
  
  printf("[PreStageController] Mode changed to: %s\n", getModeString(mode).c_str());
  
  return true;
}

std::string TwoQuadPreStageController::getModeString(PreStageMode mode){
  switch(mode){
    case PreStageMode::IDLE: return "IDLE";
    case PreStageMode::ARMAPPROACH: return "ARMAPPROACH";
    case PreStageMode::LIFT: return "LIFT";
    case PreStageMode::EXTERNAL_MPC_SIGNAL: return "EXTERNAL_MPC_SIGNAL";
    default: return "UNKNOWN";
  }
}


