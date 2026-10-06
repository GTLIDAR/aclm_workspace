#include <ros/package.h>
#include <ros/ros.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_multi_robot/common/Types.h"

using namespace ocs2;
using namespace ocs2::multi_robot;

namespace {

scalar_t TARGET_DISPLACEMENT_VELOCITY = 0.2;
scalar_t TARGET_ROTATION_VELOCITY     = 0.4;
scalar_t COM_HEIGHT                   = 0.7;
std::vector<vector_t> ROBOT_BASE_OFFSETS;
size_t num_robots_local = 0;

scalar_t estimateTimeToTarget(const vector_t& desired_base_displacement) {
  const scalar_t& dx   = desired_base_displacement(0);
  const scalar_t& dy   = desired_base_displacement(1);
  const scalar_t& dyaw = desired_base_displacement(3);
  const scalar_t rotation_time    = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement     = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotation_time, displacement_time);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& target_pose, const SystemObservation& observation,
                                                  const scalar_t& target_reaching_time) {
  // desired time trajectory
  const scalar_array_t time_trajectory{observation.time, target_reaching_time};

  // desired state trajectory
  vector_t current_pose = observation.state;

  vector_array_t state_trajectory(2, vector_t::Zero(observation.state.size()));
  state_trajectory[0] = current_pose;
  state_trajectory[1] = target_pose;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(2, vector_t::Zero(observation.input.size()));

  return {time_trajectory, state_trajectory, input_trajectory};
}

vector_t getRobotStateInWorldFrame(const vector_t& robot_base_offset, const vector_t& cargo_base_target_state) {
  Eigen::Vector3d cargo_base_target_pos = cargo_base_target_state.segment<3>(0);   // [x, y, z]
  Eigen::Vector3d cargo_base_target_ori = cargo_base_target_state.segment<3>(6);   // [yaw, pitch, roll]

  // Create rotation matrix from ZYX Euler angles (yaw, pitch, roll)
  double yaw = cargo_base_target_ori[0], pitch = cargo_base_target_ori[1], roll = cargo_base_target_ori[2];
  Eigen::Matrix3d R_world_base;
  R_world_base = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                 Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                 Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  vector_t robot_target_state_world(SINGLE_ROBOT_STATE_DIM);
  robot_target_state_world.setZero();

  // Extract robot position in object base frame
  Eigen::Vector3d pos_base = robot_base_offset.segment<3>(0);

  // Transform to world frame
  Eigen::Vector3d pos_world = R_world_base * pos_base + cargo_base_target_pos;

  // Set the position and orientation in the state vector
  robot_target_state_world.segment<3>(0) = pos_world;
  robot_target_state_world(2)            = COM_HEIGHT;
  robot_target_state_world.segment<3>(6) = cargo_base_target_ori + robot_base_offset.segment<3>(3);

  return robot_target_state_world;
}

/**
 * Converts the pose of the interactive marker to TargetTrajectories for the
 * full multi-robot + cargo state, consistent with
 * MultiRobotGeneralizedTargetTrajectoriesPublisher.
 */
TargetTrajectories cargoGoalToTargetTrajectories(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                                 const SystemObservation& observation) {
  const vector_t current_pose = observation.state;
  const size_t stateDim       = num_robots_local * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;

  // Build a compact "goal" for cargo base: [x, y, ?, yaw]
  const auto eulerZYX = orientation.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
  const scalar_t yaw  = eulerZYX[0];
  vector_t goal(4);
  goal.setZero();
  goal(0) = position.x();
  goal(1) = position.y();
  goal(2) = position.z();
  goal(3) = yaw;

  const vector_t target_pose = [&]() {
    vector_t target(stateDim);
    target.setZero();

    const size_t cargoStateOffset = num_robots_local * SINGLE_ROBOT_STATE_DIM;
    target(cargoStateOffset)     = goal(0);  // cargo com_x
    target(cargoStateOffset + 1) = goal(1);  // cargo com_y
    target(cargoStateOffset + 2) = goal(2);  // cargo com_z
    target(cargoStateOffset + 6) = goal(3);  // cargo yaw
    

    vector_t cargo_base_target_state = target.segment<12>(cargoStateOffset);

    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
      vector_t robot_target_state   = getRobotStateInWorldFrame(ROBOT_BASE_OFFSETS[robot], cargo_base_target_state);
      target.segment<SINGLE_ROBOT_STATE_DIM>(robotStateOffset) = robot_target_state;
    }

    return target;
  }();

  const scalar_t target_reaching_time = observation.time + estimateTimeToTarget(target_pose - current_pose);
  return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
}

}  // namespace

int main(int argc, char* argv[]) {
  const std::string nodeName  = "cargo_pose_target";
  const std::string robotName = "legged_robot";

  ::ros::init(argc, argv, nodeName);
  ::ros::NodeHandle nh;

  // Get node parameters
  std::string taskFile = ros::package::getPath("ocs2_multi_robot") + "/config/info/three_quadruped_w_cargo.info";
  nh.getParam("taskFile", taskFile);
  std::cerr << "[CargoPoseTarget] Loading task file: " << taskFile << std::endl;

  // Determine number of robots from cargo_model.handle_* entries
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const auto& cargoModel = pt.get_child("cargo_model");
  num_robots_local       = 0;
  for (const auto& pair : cargoModel) {
    if (pair.first.find("handle_") == 0) {
      num_robots_local++;
    }
  }

  // Load COM height and base offsets
  loadData::loadCppDataType(taskFile, "com_height.height", COM_HEIGHT);
  ROBOT_BASE_OFFSETS.resize(num_robots_local);
  for (size_t robot = 0; robot < num_robots_local; ++robot) {
    ROBOT_BASE_OFFSETS[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile, "initialStateOffset.robot_" + std::to_string(robot + 1), ROBOT_BASE_OFFSETS[robot]);
  }

  // Load target motion parameters
  loadData::loadCppDataType(taskFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(taskFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);

  TargetTrajectoriesInteractiveMarker targetPoseCommand(nh, robotName, &cargoGoalToTargetTrajectories);
  targetPoseCommand.publishInteractiveMarker();

  ros::spin();

  return 0;
}



