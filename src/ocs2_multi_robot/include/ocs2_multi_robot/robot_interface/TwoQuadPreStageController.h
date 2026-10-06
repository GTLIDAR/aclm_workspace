/*=============================== Stage Controller for Prestage of Two Robots =======================================*/
// Author: Ruize Cao
// Date: 2025/10/1

#pragma once

#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_mpc/SystemObservation.h>

#include <std_msgs/Bool.h>

#include <mutex>
#include <thread>
#include <string>

#include <ros/init.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/MarkerArray.h>
#include <ocs2_multi_robot/PreStageMode.h>

#include <ocs2_multi_robot/common/Types.h>

namespace ocs2{
  namespace multi_robot{

enum class PreStageMode{
  IDLE = 0,
  ARMAPPROACH = 1,
  LIFT = 2,
  EXTERNAL_MPC_SIGNAL = 3
};

constexpr float ROBOT2BOX_DIS = 1.5; // Desired distance between robot and box
constexpr float TARGET_DISPLACEMENT_VELOCITY = 0.3; // m/s
constexpr float TARGET_ROTATION_VELOCITY = 0.3; // rad/s
constexpr float HANDLE_X_OFFSET = 0.6;
constexpr float HANDLE_Y_OFFSET = 0.0;
constexpr float HANDLE_Z_OFFSET = 0.30; // Lower a bit to account for gripper offset
constexpr float GRIPPER_OPEN_ANGLE =  -80.0 * M_PI / 180.0;
constexpr float GRIPPER_CLOSE_ANGLE =  0.0 * M_PI / 180.0;
constexpr float COM_HEIGHT = 0.487695;
constexpr float B1TOZ1_X_OFFSET = 0.3455; // x offset from b1 base to z1 base
constexpr float B1TOZ1_Y_OFFSET = 0.0; // y offset from
constexpr float B1TOZ1_Z_OFFSET = 0.085; // z offset from b1 base to z1 base

class TwoQuadPreStageController{
public:
  TwoQuadPreStageController();
  ~TwoQuadPreStageController();

  void run();
  void stop();
  
private:
  // ROS Interface
  ros::NodeHandle nh_;
  ros::Subscriber combined_observation_sub_;
  ros::ServiceServer prestage_mode_service_;

  void CombinedObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);
  
  // Current end-effector pose subscribers
  ros::Subscriber robot1_ee_pose_sub_;
  ros::Subscriber robot2_ee_pose_sub_;
  
  void Robot1EEPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void Robot2EEPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  
  // Publishers for arm trajectories only
  ros::Publisher robot1_arm_target_pub_;
  ros::Publisher robot2_arm_target_pub_;
  ros::Publisher robot1_arm_wrench_pub_;
  ros::Publisher robot2_arm_wrench_pub_;
  ros::Publisher robot1_gripper_pub_;
  ros::Publisher robot2_gripper_pub_;
  
  // Visualization publishers
  ros::Publisher robot1_traj_viz_pub_;
  ros::Publisher robot2_traj_viz_pub_;

  // External MPC signal publisher
  ros::Publisher external_mpc_signal_pub_;
  
  // FSM
  PreStageMode current_mode_;
  bool fsm_running_;
  std::string robot1_ns_;
  std::string robot2_ns_;
  
  // State variables
  std::mutex state_mutex_;
  vector_t robot1_state_;
  vector_t robot2_state_;
  vector_t cargo_state_;
  
  // Current end-effector poses
  geometry_msgs::PoseStamped robot1_current_ee_pose_;
  geometry_msgs::PoseStamped robot2_current_ee_pose_;
  bool robot1_ee_pose_received_;
  bool robot2_ee_pose_received_;
  
  // Trajectory execution variables
  std::vector<geometry_msgs::PoseStamped> robot1_trajectory_;
  std::vector<geometry_msgs::PoseStamped> robot2_trajectory_;
  int trajectory_index_;
  bool trajectory_initialized_;
  
  // Single FSM thread
  std::thread fsm_thread_;
  void fsmThread();
  
  // FSM state functions
  void runIdle();
  void runArmApproach();
  void runLift();
  void runExternalMpcSignal();

  // Trajectory visualization functions
  void visualizeTrajectoryProgress(const geometry_msgs::PoseStamped& current_pose, int pose_index, 
                                  ros::Publisher& viz_pub, const std::string& robot_name);

  // Service Server
  bool prestageModeServiceCallback(ocs2_multi_robot::PreStageMode::Request &req, ocs2_multi_robot::PreStageMode::Response &res);
  std::string getModeString(PreStageMode mode);
};    

/*********************************** Utility ******************************************* */

scalar_t estimateTimeToTarget(const vector_t& desired_base_displacement)
{
  const scalar_t& dx = desired_base_displacement(0);
  const scalar_t& dy = desired_base_displacement(1);
  const scalar_t& dyaw = desired_base_displacement(3);
  const scalar_t rotation_time = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotation_time, displacement_time);
}

geometry_msgs::Quaternion axisAngleToQuaternion(const Eigen::Vector3d& axis, double angle) {
  // Normalize the axis vector
  Eigen::Vector3d normalized_axis = axis.normalized();
  
  // Calculate quaternion components
  double half_angle = angle * 0.5;
  double sin_half = sin(half_angle);
  double cos_half = cos(half_angle);
  
  // Create quaternion: q = cos(θ/2) + sin(θ/2) * (xi + yj + zk)
  geometry_msgs::Quaternion quat_msg;
  quat_msg.w = cos_half;                        // Scalar part
  quat_msg.x = normalized_axis.x() * sin_half;  // Vector part x
  quat_msg.y = normalized_axis.y() * sin_half;  // Vector part y  
  quat_msg.z = normalized_axis.z() * sin_half;  // Vector part z
  
  return quat_msg;
}

void Z1PathGenerator( const int steps, std::vector<float>& x_path, std::vector<float>& z_path){
  x_path.clear();
  z_path.clear();

  // Solved by Matlab
  std::vector<float> a = {4.1346, -1.954, -2.60852, 1.9149, 0.0};
  
  float x_len = ROBOT2BOX_DIS - HANDLE_X_OFFSET - B1TOZ1_X_OFFSET;
  float x_step = x_len / steps;
  float x,z;

  for (int i = 0; i <= steps; ++i) {
    x = i * x_step;
    z = a[4]*pow(x,4) + a[3]*pow(x,3) + a[2]*pow(x,2) + a[1]*x + a[0];
    x_path.push_back(x);
    z_path.push_back(z);
  }
};

}
}