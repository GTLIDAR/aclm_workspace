#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesInteractiveMarker.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include "ocs2_quadruped/common/ModelSettings.h"
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace quadruped;

namespace
{
scalar_t TARGET_VELOCITY;
scalar_t TARGET_VELOCITY_ROTATION;
std::string taskFile, urdfFile, referenceFile, eeFrame;
pinocchio::Model rmodel_;
pinocchio::Data rdata_;
}  // namespace

vector_t computeEEposePinocchio(const vector_t& observation, const std::string& ee_name){
  // Output should be x y z (w x y z)
  pinocchio::forwardKinematics(rmodel_, rdata_, observation);
  pinocchio::updateFramePlacements(rmodel_, rdata_);
  vector_t ee_pose(3 + 4);
  ee_pose << rdata_.oMf[rmodel_.getBodyId(ee_name)].translation(),
             matrixToQuaternion(rdata_.oMf[rmodel_.getBodyId(ee_name)].rotation()).coeffs();
  return ee_pose;
}

scalar_t estimateTimeToTarget(const vector_t& current_pose, const vector_t& desired_pose)
{
  auto diff = current_pose.head(3) - desired_pose.head(3);
  const scalar_t& dx = diff(0);
  const scalar_t& dy = diff(1);
  const scalar_t& dz = diff(2);
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
  const scalar_t displacement_time = displacement / TARGET_VELOCITY;

  const Eigen::Quaterniond quat_curr(current_pose.tail<4>());
  const Eigen::Quaterniond quat_des(desired_pose.tail<4>());
  const scalar_t rotational_time = std::abs(quat_curr.angularDistance(quat_des)) / TARGET_VELOCITY_ROTATION;
  std::cout << "translational time is: " << displacement_time << std::endl;
  std::cout << "rotational time is:    " << rotational_time << std::endl;
  return std::max(displacement_time, rotational_time);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& target_pose, const SystemObservation& observation,
                                                  const scalar_t& target_reaching_time)
{
  // desired time trajectory
  const scalar_array_t time_trajectory{ observation.time, target_reaching_time };

  // desired state trajectory
  vector_t current_pose = computeEEposePinocchio(observation.state.segment(6, rmodel_.nq), eeFrame);
  vector_array_t state_trajectory(2, vector_t::Zero(7));
  state_trajectory[0] << current_pose;
  state_trajectory[1] << target_pose;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t input_trajectory(2, vector_t::Zero(observation.input.size()));

  return { time_trajectory, state_trajectory, input_trajectory };
}

/**
 * Converts command line to TargetTrajectories.
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, 
                                                   const SystemObservation& observation) {                                                
  // state trajectory: 3 + 4 for desired position vector and orientation quaternion
  const vector_t current_ee = computeEEposePinocchio(observation.state.segment(6, rmodel_.nq), eeFrame);  
  Eigen::Quaterniond desired_orientation;
  desired_orientation.w() = commadLineTarget(6);
  desired_orientation.x() = commadLineTarget(3);
  desired_orientation.y() = commadLineTarget(4);
  desired_orientation.z() = commadLineTarget(5);
  const vector_t target_ee = (vector_t(7) << commadLineTarget.segment(0, 3), desired_orientation.coeffs()).finished();

  const scalar_t target_reaching_time = observation.time + estimateTimeToTarget(current_ee, target_ee);
  std::cout << "current pose (x, y, z, qx, qy, qz, qw) is : " << current_ee.transpose() << std::endl;
  std::cout << "goal pose (x, y, z, qx, qy, qz, qw) is : " << target_ee.transpose() << std::endl;
  std::cout << "desired reaching time is: " << target_reaching_time - observation.time << std::endl;
  return targetPoseToTargetTrajectories(target_ee, observation, target_reaching_time);
}

int main(int argc, char* argv[]) {
  const std::string robotName = "loco_manipulation";
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  
  nodeHandle.getParam("/taskFileManipulation", taskFile);
  nodeHandle.getParam("/urdfFile", urdfFile);
  nodeHandle.getParam("/referenceFileManipulation", referenceFile);

  loadData::loadCppDataType(referenceFile, "targetVelocity", TARGET_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetVelocityRotation", TARGET_VELOCITY_ROTATION);
  loadData::loadCppDataType(taskFile, "model_settings.eeFrame", eeFrame);

  bool verbose;
  ModelSettings modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);;
  PinocchioInterface pino_interface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames));
  rmodel_ = pino_interface.getModel();
  rdata_ = pino_interface.getData();

  const scalar_array_t relativeEELimit{10.0, 10.0, 0.35, 1.0, 1.0, 1.0, 1.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeEELimit, &commandLineToTargetTrajectories);
  
  const std::string commandMsg = "Enter XYZ and qX qY qZ qW for the desired EE, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}