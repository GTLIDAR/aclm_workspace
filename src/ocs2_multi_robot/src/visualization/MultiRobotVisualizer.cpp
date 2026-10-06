#include "ocs2_multi_robot/visualization/MultiRobotVisualizer.h"
#include <ros/package.h>

// OCS2
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include "ocs2_quadruped/gait/MotionPhaseDefinition.h"

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/JointState.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// Boost for config parsing
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
MultiRobotVisualizer::MultiRobotVisualizer(ros::NodeHandle& nodeHandle,
                                           std::string taskFile,
                                           std::string terrainType,
                                           scalar_t maxUpdateFrequency)
    : lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
  taskFile_ = taskFile;
  
  // Determine number of robots by counting handles in config
  numRobots_ = 0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  const auto& cargoModel = pt.get_child("cargo_model");
  for (const auto& pair : cargoModel) {
    if (pair.first.find("handle_") == 0) {
      numRobots_++;
    }
  }
  
  if (numRobots_ == 0) {
    throw std::runtime_error("[MultiRobotVisualizer] No robot handles found in config file!");
  }
  
  std::cerr << "[MultiRobotVisualizer] Detected " << numRobots_ << " robots" << std::endl;
  
  launchVisualizerNode(nodeHandle);
  heightMapPtr_ = HeightMap::MakeTerrain(terrain_mapping[terrainType]);
  getTerrainMarker();
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon);

  // The hip joint positions relative to base (from joint origins in URDF)
  std::vector<vector3_t> base2hip_vec;
  Eigen::VectorXd base2hip_stacked(12);
  loadData::loadEigenMatrix(taskFile_, "robot_model.base2hip", base2hip_stacked);
  for (size_t leg = 0; leg < 4; leg++) {
    base2hip_vec.push_back(base2hip_stacked.segment(3*leg, 3));
  }
  Eigen::Vector3d hfe_to_haa_y;
  loadData::loadEigenMatrix(taskFile_, "robot_model.hfe_to_haa_y", 
    hfe_to_haa_y);
  double thigh_length, shank_length;
  loadData::loadCppDataType(taskFile_, "robot_model.thigh_length", thigh_length);
  loadData::loadCppDataType(taskFile_, "robot_model.shank_length", shank_length);

  // Load all robot handles dynamically
  robot_handles_.resize(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robot_handles_[robot] = vector_t::Zero(6);
    loadData::loadEigenMatrix(taskFile_, "cargo_model.handle_" + std::to_string(robot + 1), robot_handles_[robot]);
  }
  handle_positions_world_.resize(numRobots_);
  handle_quaternion_world_.resize(numRobots_);

  std::string urdfPath;
  nodeHandle.param<std::string>("urdfFile", urdfPath,
      (ros::package::getPath("ocs2_multi_robot") + "/assets/robot/b1z1_description") + "/urdf/b1z1.urdf");

  // Determine cargo root TF frame from the box/table URDF
  std::string cargoUrdfPath;
  nodeHandle.getParam("cargoUrdfFile", cargoUrdfPath);
  cargoRootFrame_ = "/cargo/box";  // default fallback
  if (!cargoUrdfPath.empty()) {
    urdf::Model cargoUrdfModel;
    if (cargoUrdfModel.initFile(cargoUrdfPath)) {
      cargoRootFrame_ = "/cargo/" + cargoUrdfModel.getRoot()->name;
      std::cerr << "[MultiRobotVisualizer] Cargo root frame: " << cargoRootFrame_ << std::endl;
    } else {
      std::cerr << "[MultiRobotVisualizer] Failed to parse cargo URDF, using default: " << cargoRootFrame_ << std::endl;
    }
  }
  
  WholeBodyIK::LegGeometry geometry;
  std::copy(base2hip_vec.begin(), base2hip_vec.end(), geometry.baseToHip.begin());
  geometry.hipToThigh = hfe_to_haa_y;
  geometry.thighLength = thigh_length;
  geometry.shankLength = shank_length;
  WholeBodyIK::Settings settings;
  settings.legMode = WholeBodyIK::LegMode::Analytical;
  settings.analyticalLegs = geometry;
  settings.dt = minPublishTimeDifference_;
  wholeBodyIk_.resize(numRobots_);
  ikStates_.resize(numRobots_);
  ikInitialized_.assign(numRobots_, false);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    wholeBodyIk_[robot] = std::make_unique<WholeBodyIK>(urdfPath, settings);
    ikStates_[robot] = wholeBodyIk_[robot]->neutralState();
  }

  realTrajectoryPoints_.resize(numRobots_ + 1);  // +1 for cargo trajectory
  
  realFootTrajectoryPoints_.resize(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    realFootTrajectoryPoints_[robot].resize(QUADRUPED_FOOT_NUM);
  }
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // Create publishers for all robots dynamically
  robot_costDesiredBasePositionPublishers_.resize(numRobots_);
  robot_costDesiredFeetPositionPublishers_.resize(numRobots_);
  robot_realfootPositionPublishers_.resize(numRobots_);
  robot_armDesiredPosePublishers_.resize(numRobots_);
  robot_namespaces_.resize(numRobots_);
  robot_joint_states_publishers_.resize(numRobots_);
  // Resize recorded real trajectory points
  robot_realTrajectoryPublishers_.resize(numRobots_ + 1);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotNum = robot + 1;
    const std::string robotPrefix = "/robot" + std::to_string(robotNum);
    
    // Base trajectory publisher
    robot_costDesiredBasePositionPublishers_[robot] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/desiredBaseTrajectory", 1);
    
    // Feet trajectory publishers
    robot_costDesiredFeetPositionPublishers_[robot].resize(QUADRUPED_FOOT_NUM);
    robot_realfootPositionPublishers_[robot].resize(QUADRUPED_FOOT_NUM);
    robot_costDesiredFeetPositionPublishers_[robot][0] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/desiredFeetTrajectory/LF", 1);
    robot_costDesiredFeetPositionPublishers_[robot][1] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/desiredFeetTrajectory/RF", 1);
    robot_costDesiredFeetPositionPublishers_[robot][2] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/desiredFeetTrajectory/LH", 1);
    robot_costDesiredFeetPositionPublishers_[robot][3] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/desiredFeetTrajectory/RH", 1);

    robot_realfootPositionPublishers_[robot][0] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/realFeetTrajectory/LF", 1);
    robot_realfootPositionPublishers_[robot][1] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/realFeetTrajectory/RF", 1);
    robot_realfootPositionPublishers_[robot][2] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/realFeetTrajectory/LH", 1);
    robot_realfootPositionPublishers_[robot][3] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/realFeetTrajectory/RH", 1);

    // Arm desired pose publisher
    robot_armDesiredPosePublishers_[robot] = nodeHandle.advertise<visualization_msgs::MarkerArray>(
      robotPrefix + "/cen_opt/desiredArmPose", 1);

    // Real trajectory publisher
    robot_realTrajectoryPublishers_[robot] = nodeHandle.advertise<visualization_msgs::Marker>(
      robotPrefix + "/cen_opt/realTrajectory", 1);
    
    // Get robot namespace from launch file
    std::string paramName = "robot" + std::to_string(robotNum) + "_ns";
    if (!nodeHandle.getParam(paramName, robot_namespaces_[robot])) {
      // Fallback to default naming if param not found
      robot_namespaces_[robot] = "b1z1_" + std::to_string(robotNum);
    }
    
    // Joint state publisher
    robot_joint_states_publishers_[robot] = nodeHandle.advertise<sensor_msgs::JointState>(
      "/" + robot_namespaces_[robot] + "/joint_states", 1);
  }

  robot_realTrajectoryPublishers_[numRobots_] = nodeHandle.advertise<visualization_msgs::Marker>(
      "/cargo/cen_opt/realTrajectory", 1);

  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/optimizedStateTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/currentState", 1);
  centroidPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/centroid", 1);
  terrainPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/terrain", 1);

  // Cargo target trajectory publisher
  cargoTargetTrajectoryPub_ = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/cargo_target_trajectory", 1);
}

void MultiRobotVisualizer::publishInitialState(vector_t& initState) {
  const auto timeStamp = ros::Time::now();
  centroid_msgs.markers.clear();
  
  vector_t centroidPose1(6);
  vector_t centroidPose2(6);
  centroidPose1 << initState.segment(0, 3), initState.segment(6, 3);
  centroidPose2 << initState.segment(24, 3), initState.segment(30, 3);
  vector3_t ellip_size;
  ellip_size << 0.4, 0.2, 0.2;
  publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::red, "robot_1_init");
  publishCentroidTransform(timeStamp, centroidPose2, ellip_size, Color::green, "robot_2_init");
  
  // Publish all batched markers at once
  centroidPublisher_.publish(centroid_msgs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::updateCentroidalState(const SystemObservation& observation) {
  const auto timeStamp = ros::Time::now();
  
  centroid_msgs.markers.clear();
  currentState_msgs.markers.clear();
  
  // Verify state and input sizes
  const size_t expectedStateSize = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  const size_t expectedInputSize = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
  
  if (observation.state.size() != expectedStateSize) {
    std::cerr << "[MultiRobotVisualizer] State size mismatch! Expected: " << expectedStateSize 
              << ", got: " << observation.state.size() << " (numRobots: " << numRobots_ << ")" << std::endl;
  }
  if (observation.input.size() != expectedInputSize) {
    std::cerr << "[MultiRobotVisualizer] Input size mismatch! Expected: " << expectedInputSize 
              << ", got: " << observation.input.size() << " (numRobots: " << numRobots_ << ")" << std::endl;
  }

  // Compute cartesian state and inputs for all robots
  const size_t totalFeet = numRobots_ * QUADRUPED_FOOT_NUM;
  std::vector<vector3_t> feetPositions(totalFeet);
  std::vector<vector3_t> feetForces(totalFeet);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    const size_t robotInputOffset = robot * SINGLE_ROBOT_INPUT_DIM;
    const size_t feetOffset = robot * QUADRUPED_FOOT_NUM;
    
    // Verify we have enough state/input for this robot
    if (observation.state.size() < robotStateOffset + SINGLE_ROBOT_STATE_DIM) {
      std::cerr << "[MultiRobotVisualizer] State too small for robot " << robot << std::endl;
      continue;
    }
    if (observation.input.size() < robotInputOffset + SINGLE_ROBOT_INPUT_DIM) {
      std::cerr << "[MultiRobotVisualizer] Input too small for robot " << robot << std::endl;
      continue;
  }
    
  for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
      const size_t footStateOffset = robotStateOffset + 12 + i * 3;
      const size_t footInputOffset = robotInputOffset + i * 3;
      
      if (observation.state.size() >= footStateOffset + 3) {
        feetPositions[feetOffset + i] = observation.state.segment(footStateOffset, 3);
      } else {
        std::cerr << "[MultiRobotVisualizer] State too small for robot " << robot << " foot " << i << std::endl;
        feetPositions[feetOffset + i].setZero();
      }
      
      if (observation.input.size() >= footInputOffset + 3) {
        feetForces[feetOffset + i] = observation.input.segment(footInputOffset, 3);
      } else {
        std::cerr << "[MultiRobotVisualizer] Input too small for robot " << robot << " foot " << i << std::endl;
        feetForces[feetOffset + i].setZero();
      }
    }
  }

  // Hardcoded contact state for now (all feet in contact)
  std::vector<bool> contactFlags(totalFeet, true);
  publishCartesianMarkers(timeStamp, contactFlags, feetPositions, feetForces);


  // Publish the arm forces
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  std::vector<vector3_t> armForces(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    armForces[robot] = observation.input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
  }
  
  // Compute handle positions from observation state (not from stale member variable)
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  std::vector<vector3_t> handle_positions_world(numRobots_);
  if (observation.state.size() >= cargoStateOffset + CARGO_STATE_DIM) {
    vector_t cargoPose(6);
    cargoPose << observation.state.segment(cargoStateOffset, 3), 
                 observation.state.segment(cargoStateOffset + 6, 3);
    const vector3_t cargo_pos_world = cargoPose.head<3>();
    const vector3_t cargo_eul_zyx = cargoPose.tail<3>();
    const matrix3_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargo_eul_zyx);
    
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const vector3_t handle_pos_cargo = robot_handles_[robot].head<3>();
      handle_positions_world[robot] = cargo_pos_world + w_R_cargo * handle_pos_cargo;
    }
  } else {
    // Fallback to member variable if cargo state is not available
    handle_positions_world = handle_positions_world_;
  }
  
  publishRobotArmForces(timeStamp, handle_positions_world, armForces); 
  
  // Publish all batched cartesian markers at once
  currentStatePublisher_.publish(currentState_msgs);

  // Publish centroids for all robots
  const bool hasCargo = (observation.state.size() > cargoStateOffset);
  
    vector3_t robot_ellip_size;
    robot_ellip_size << 0.4, 0.2, 0.2;
  
  std::vector<Color> robotColors = {Color::yellow, Color::blue, Color::green, Color::orange};
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    vector_t centroidPose(6);
    centroidPose << observation.state.segment(robotStateOffset, 3), 
                     observation.state.segment(robotStateOffset + 6, 3);
    const std::string frameName = "robot" + std::to_string(robot + 1);
    publishCentroidTransform(timeStamp, centroidPose, robot_ellip_size, 
                             robotColors[robot % robotColors.size()], frameName);
  }
  
  if (hasCargo) {
    vector_t cargoPose(6);
    cargoPose << observation.state.segment(cargoStateOffset, 3), 
                 observation.state.segment(cargoStateOffset + 6, 3);
    vector3_t cargo_cube_size;
    cargo_cube_size << 1.0, 1.0, 1.0;
    publishCargoTransform(timeStamp, cargoPose, cargo_cube_size, Color::red, "cargo");
  }
  
  // Publish base transforms for elevation mapping (perceptive terrain mode)
  publishRobotBaseTransformsForElevationMapping(timeStamp, observation);
  
  // Publish all batched markers at once
  centroidPublisher_.publish(centroid_msgs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::updateFullBodyState(const SystemObservation& observation) {
  if (!std::isfinite(observation.time) || observation.time == lastIkTime_) return;
  const scalar_t ikDt = lastIkTime_ >= 0.0 && observation.time > lastIkTime_
      ? std::min(observation.time - lastIkTime_, scalar_t(0.1)) : minPublishTimeDifference_;
  lastIkTime_ = observation.time;
  const auto timeStamp = ros::Time::now();
  
  currentState_msgs.markers.clear();
  
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  const size_t minStateSize = cargoStateOffset + CARGO_STATE_DIM;
  
  // Check if state vector has enough elements
  if (observation.state.size() < minStateSize) {
    std::cerr << "[MultiRobotVisualizer] State vector too small: " << observation.state.size() 
              << " < " << minStateSize << std::endl;
    return;
  }
  
  // Extract robot base poses for all robots
  std::vector<vector_t> robotBasePoses(numRobots_);
  std::vector<bool> robotBaseValid(numRobots_, true);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    if (observation.state.size() < robotStateOffset + SINGLE_ROBOT_STATE_DIM) {
      robotBaseValid[robot] = false;
      robotBasePoses[robot] = vector_t::Zero(6);
      continue;
    }
    robotBasePoses[robot] = vector_t(6);
    robotBasePoses[robot] << observation.state.segment(robotStateOffset, 3), 
                             observation.state.segment(robotStateOffset + 6, 3);
    
    // Check for NaN in base pose
    if (robotBasePoses[robot].array().isNaN().any()) {
      robotBaseValid[robot] = false;
      robotBasePoses[robot].setZero();
    }
  }
  
  // Extract cargo pose
  vector_t cargoPose(6);
  if (observation.state.size() >= cargoStateOffset + CARGO_STATE_DIM) {
    cargoPose << observation.state.segment(cargoStateOffset, 3), 
                 observation.state.segment(cargoStateOffset + 6, 3);
  } else {
    cargoPose.setZero();
  }

  std::vector<std::vector<vector3_t>> feetPositions_world(numRobots_);
  std::vector<bool> robotFeetValid(numRobots_, true);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    feetPositions_world[robot].resize(QUADRUPED_FOOT_NUM);
    
    if (!robotBaseValid[robot]) {
      robotFeetValid[robot] = false;
      for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
        feetPositions_world[robot][i].setZero();
      }
      continue;
    }
    
  // Extract feet positions from state (world frame)
    bool feetValid = true;
    for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
      const size_t footOffset = robotStateOffset + 12 + i * 3;
      if (observation.state.size() >= footOffset + 3) {
        feetPositions_world[robot][i] = observation.state.segment(footOffset, 3);
        // Check for NaN
        if (feetPositions_world[robot][i].array().isNaN().any()) {
          feetValid = false;
          feetPositions_world[robot][i].setZero();
        }
      } else {
        feetValid = false;
        feetPositions_world[robot][i].setZero();
      }
    }
    
    if (!feetValid) {
      robotFeetValid[robot] = false;
    }
  }

  std::vector<vector_t> leg_joints(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    leg_joints[robot] = ikStates_[robot].joints.head<12>();
  }

  // Compute arm EE target in each robot's z1 base frame and solve IK (single step)
  // Cargo pose in world
  const vector3_t cargo_pos_world = cargoPose.head<3>();
  const vector3_t cargo_eul_zyx = cargoPose.tail<3>();
  const matrix3_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargo_eul_zyx);

  // Compute world poses of all handles (used as arm EE targets)
  std::vector<vector3_t> handle_positions_world(numRobots_);
  std::vector<matrix3_t> handle_rotations_world(numRobots_);
  std::vector< Eigen::Quaternion<scalar_t> > handle_quaternion_world(numRobots_);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
  // Handle poses in cargo frame (6D: x y z yaw pitch roll)
    vector3_t handle_pos_cargo = robot_handles_[robot].head<3>();
    matrix3_t cargo_R_handle = getRotationMatrixFromZyxEulerAngles<scalar_t>(robot_handles_[robot].tail<3>());

    // World poses of handles
    handle_positions_world[robot] = cargo_pos_world + w_R_cargo * handle_pos_cargo;
    handle_rotations_world[robot] = w_R_cargo * cargo_R_handle;

    // Publish handle positions to tf
    geometry_msgs::TransformStamped handleToWorldTransform;
    handleToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    handleToWorldTransform.child_frame_id = "handle_" + std::to_string(robot + 1);
    handleToWorldTransform.transform.translation = getVectorMsg(handle_positions_world[robot]);
    handle_quaternion_world[robot] = Eigen::Quaternion<scalar_t>(handle_rotations_world[robot]);
    handleToWorldTransform.transform.rotation = getOrientationMsg(handle_quaternion_world[robot]);
    tfBroadcaster_.sendTransform(handleToWorldTransform);
  }
  // Publish Arm Desired Poses
  // publishDesiredArmPose(timeStamp, handle_positions_world, handle_quaternion_world);

  // Publish the arm forces
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  std::vector<vector3_t> armForces(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    armForces[robot] = observation.input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
  }
  publishRobotArmForces(timeStamp, handle_positions_world, armForces); 

  std::vector<vector_t> arm_q(numRobots_);
  std::vector<vector_t> arm_qd(numRobots_);
  std::vector<bool> armValid(numRobots_, true);
  double ikSolveMilliseconds = 0.0;
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    if (!robotBaseValid[robot]) {
      armValid[robot] = false;
      arm_q[robot] = ikStates_[robot].joints.tail<6>();
      arm_qd[robot] = vector_t::Zero(6);
      continue;
    }
    
    const vector3_t base_pos_world = robotBasePoses[robot].head<3>();
    const vector3_t base_eul_zyx = robotBasePoses[robot].tail<3>();
    const matrix3_t w_R_b = getRotationMatrixFromZyxEulerAngles<scalar_t>(base_eul_zyx);
    
    // Check for NaN in cargo or handle positions
    if (cargoPose.array().isNaN().any() || handle_positions_world[robot].array().isNaN().any()) {
      armValid[robot] = false;
      arm_q[robot] = ikStates_[robot].joints.tail<6>();
      arm_qd[robot] = vector_t::Zero(6);
      continue;
    }
    
    auto& state = ikStates_[robot];
    state.worldFromBase.translation() = base_pos_world;
    state.worldFromBase.linear() = w_R_b;
    WholeBodyIK::Targets targets;
    targets.arm.translation() = handle_positions_world[robot];
    targets.arm.linear() = handle_rotations_world[robot];
    for (size_t foot = 0; foot < 4; ++foot) {
      targets.feet[foot] = feetPositions_world[robot][foot];
    }
    if (!ikInitialized_[robot]) {
      const auto initializationStart = ros::WallTime::now();
      if (!robotFeetValid[robot]) return;
      auto candidate = state;
      wholeBodyIk_[robot]->setTimeStep(0.01);
      bool converged = false;
      for (size_t iteration = 0; iteration < 2000; ++iteration) {
        const auto actual = wholeBodyIk_[robot]->currentTargets(candidate);
        double positionError = (actual.arm.translation() - targets.arm.translation()).norm();
        for (size_t foot = 0; foot < 4; ++foot) {
          positionError = std::max(positionError, (actual.feet[foot] - targets.feet[foot]).norm());
        }
        const double orientationError = std::abs(Eigen::AngleAxisd(
            targets.arm.linear() * actual.arm.linear().transpose()).angle());
        if (positionError < 0.01 && orientationError < 0.02) {
          ROS_INFO("[IK] Robot %zu initial residual: position %.6f m, orientation %.6f rad", robot + 1,
                   positionError, orientationError);
          converged = true;
          break;
        }
        const auto result = wholeBodyIk_[robot]->solve(candidate, targets);
        if (!result.success) {
          throw std::runtime_error("Initial whole-body IK failed for robot " +
              std::to_string(robot + 1) + ": " + result.message);
        }
        candidate = result.state;
      }
      wholeBodyIk_[robot]->setTimeStep(ikDt);
      if (!converged) {
        throw std::runtime_error("Initial whole-body IK did not reach the handle for robot " +
            std::to_string(robot + 1));
      }
      state = candidate;
      ikInitialized_[robot] = true;
        ROS_INFO("[IKTiming] Robot %zu initial IK: %.3f ms", robot + 1,
          (ros::WallTime::now() - initializationStart).toSec() * 1000.0);
    }
    leg_joints[robot] = state.joints.head<12>();
    arm_q[robot] = state.joints.tail<6>();
    arm_qd[robot] = vector_t::Zero(6);
    if (robotFeetValid[robot]) {
      wholeBodyIk_[robot]->setTimeStep(ikDt);
      const auto solveStart = ros::WallTime::now();
      const auto result = wholeBodyIk_[robot]->solve(state, targets);
      ikSolveMilliseconds += (ros::WallTime::now() - solveStart).toSec() * 1000.0;
      if (result.success) {
        state = result.state;
        leg_joints[robot] = state.joints.head<12>();
        arm_q[robot] = state.joints.tail<6>();
        arm_qd[robot] = result.jointVelocity.tail<6>();
      } else {
        ROS_WARN_STREAM_THROTTLE(1.0, "[Visualizer] Holding IK reference: " << result.message);
      }
    }
    
    // Check for NaN in arm IK result
    if (arm_q[robot].array().isNaN().any()) {
      armValid[robot] = false;
      arm_q[robot].setZero();
    }
  }

  // Concatenate the joint angles: 12 leg + 6 arm = 18 per robot, plus gripper = 19
  ROS_INFO_THROTTLE(2.0, "[IKTiming] %zu robots steady IK total: %.3f ms", numRobots_, ikSolveMilliseconds);
  if (std::find(ikInitialized_.begin(), ikInitialized_.end(), false) != ikInitialized_.end()) return;
  std::vector<vector_t> ik_results(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    ik_results[robot] = vector_t(19);
    ik_results[robot].head(12) = leg_joints[robot];
    if (armValid[robot]) {
      ik_results[robot].segment(12, 6) = arm_q[robot].head(6);
    } else {
      ik_results[robot].segment(12, 6) = ikStates_[robot].joints.tail<6>();
    }
    // Gripper opening depends on cargo type: box (thin handle), table (thick edge), stretcher (wide bar)
    if (cargoRootFrame_.find("box") != std::string::npos) {
      ik_results[robot](18) = -0.28;
    } else if (cargoRootFrame_.find("table") != std::string::npos) {
      ik_results[robot](18) = -0.32;
    } else if (cargoRootFrame_.find("stretcher") != std::string::npos){
      ik_results[robot](18) = -0.5;
    }
    
    // Final check for NaN before publishing
    if (ik_results[robot].array().isNaN().any()) {
      ik_results[robot].setZero();
    }
  }

  // Publish the joint angles and base transforms for all robots
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    // Always publish base transform (even if invalid, use zeros) to maintain TF tree
    publishBaseTransform(timeStamp, robotBasePoses[robot], robot_namespaces_[robot]);
    publishJointTransforms(timeStamp, ik_results[robot], robot_namespaces_[robot]);
  }
  
  // Publish all batched cartesian markers at once
  currentStatePublisher_.publish(currentState_msgs);
}



void MultiRobotVisualizer::updateBaseTrasform(const SystemObservation& observation) {
  const auto timeStamp = ros::Time::now();
  
  currentState_msgs.markers.clear();
  
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  const size_t minStateSize = cargoStateOffset + CARGO_STATE_DIM;
  
  // Check if state vector has enough elements
  if (observation.state.size() < minStateSize) {
    std::cerr << "[MultiRobotVisualizer] State vector too small: " << observation.state.size() 
              << " < " << minStateSize << std::endl;
    return;
  }
  
  // Extract cargo pose
  vector_t cargoPose(6);
  cargoPose << observation.state.segment(cargoStateOffset, 3), 
               observation.state.segment(cargoStateOffset + 6, 3);
 
  // Compute arm EE target in each robot's z1 base frame
  // Cargo pose in world
  const vector3_t cargo_pos_world = cargoPose.head<3>();
  const vector3_t cargo_eul_zyx = cargoPose.tail<3>();
  const matrix3_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargo_eul_zyx);

  // Compute world poses of all handles
  std::vector<vector3_t> handle_positions_world(numRobots_);
  std::vector<matrix3_t> handle_rotations_world(numRobots_);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
  // Handle poses in cargo frame (6D: x y z yaw pitch roll)
    vector3_t handle_pos_cargo = robot_handles_[robot].head<3>();
    matrix3_t cargo_R_handle = getRotationMatrixFromZyxEulerAngles<scalar_t>(robot_handles_[robot].tail<3>());

    // World poses of handles
    handle_positions_world[robot] = cargo_pos_world + w_R_cargo * handle_pos_cargo;
    handle_rotations_world[robot] = w_R_cargo * cargo_R_handle;

    // Publish handle positions to tf
    geometry_msgs::TransformStamped handleToWorldTransform;
    handleToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    handleToWorldTransform.child_frame_id = "handle_" + std::to_string(robot + 1);
    handleToWorldTransform.transform.translation = getVectorMsg(handle_positions_world[robot]);
    Eigen::Quaternion<scalar_t> q_world_handle(handle_rotations_world[robot]);
    handleToWorldTransform.transform.rotation = getOrientationMsg(q_world_handle);
    tfBroadcaster_.sendTransform(handleToWorldTransform);
  }

  // Publish the arm forces
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  std::vector<vector3_t> armForces(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    armForces[robot] = observation.input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
  }
  publishRobotArmForces(timeStamp, handle_positions_world, armForces); 
  
  // Publish all batched cartesian markers at once
  currentStatePublisher_.publish(currentState_msgs);
}


/******************************************************************************************************/
/******************************************************************************************************/
/**************************************************************************************************/
/************ For Centralized Observation Node  *************/
void MultiRobotVisualizer::PublishHandleTF(const SystemObservation& observation) {
  
  // Check if handle positions have been set
  if( abs(handle_quaternion_world_[0].norm()) < 1e-5) {
    ROS_WARN("[MultiRobotVisualizer] Waitting for incoming cargo observation....");
    return;
  }

  const auto timeStamp = ros::Time::now();
  currentState_msgs.markers.clear();

  for (size_t robot = 0; robot < numRobots_; ++robot) {
   
    // Publish handle positions to tf
    geometry_msgs::TransformStamped handleToWorldTransform;
    handleToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    handleToWorldTransform.child_frame_id = "handle_" + std::to_string(robot + 1);
    handleToWorldTransform.transform.translation = getVectorMsg(handle_positions_world_[robot]);
    handleToWorldTransform.transform.rotation = getOrientationMsg(handle_quaternion_world_[robot]);
    tfBroadcaster_.sendTransform(handleToWorldTransform);
  }

  // Publish the arm forces
  const size_t cargoInputOffset = numRobots_ * SINGLE_ROBOT_INPUT_DIM;
  std::vector<vector3_t> armForces(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    armForces[robot] = observation.input.segment(cargoInputOffset + robot * ARM_CONTACT_DIM, 3);
  }
  publishRobotArmForces(timeStamp, handle_positions_world_, armForces); 
  
  // Publish all batched cartesian markers at once
  currentStatePublisher_.publish(currentState_msgs);
}


void MultiRobotVisualizer::updateRobotHandlePoses(const std::vector<vector3_t>& handle_positions_world,
                          const std::vector<Eigen::Quaternion<scalar_t>>& handle_quaternion_world) {
  handle_positions_world_ = handle_positions_world;
  handle_quaternion_world_ = handle_quaternion_world;
};


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose,
                            const std::string& robot_namespace) {
  geometry_msgs::TransformStamped baseToWorldTransform;
  baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  baseToWorldTransform.child_frame_id = robot_namespace + "/base";

  // Convert Euler angles (yaw, pitch, roll) to quaternion
  const vector3_t euler_angles = basePose.tail<3>();
  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(euler_angles);
  baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
  tfBroadcaster_.sendTransform(baseToWorldTransform);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles,
                              const std::string& robot_namespace) const {
  sensor_msgs::JointState jointStateMsg;
  jointStateMsg.header.stamp = timeStamp;
  
  // Define joint names (leg joints + arm joints)
  std::vector<std::string> jointNames = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", 
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "jointGripper"
  };

  if (jointAngles.size() != static_cast<Eigen::Index>(jointNames.size())) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "[MultiRobotVisualizer] Joint state size mismatch for "
        << robot_namespace << ": expected " << jointNames.size() << ", got " << jointAngles.size());
    return;
  }

  jointStateMsg.name = jointNames;
  jointStateMsg.position.resize(jointAngles.size());
  
  // Copy joint angles to the message
  for (size_t i = 0; i < jointAngles.size(); ++i) {
    jointStateMsg.position[i] = jointAngles[i];
  }
  
  // Find the matching robot namespace and publish
  bool found = false;
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    if (robot_namespace == robot_namespaces_[robot]) {
      robot_joint_states_publishers_[robot].publish(jointStateMsg);
      found = true;
      break;
    }
  }
  
  if (!found) {
    std::cerr << "[MultiRobotVisualizer] Warning: Invalid namespace: " << robot_namespace << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  currentTime = observation.time;
  const auto timeStamp = ros::Time::now();
  if(currentTime < 0.2) {
    publishTerrain();
  }

  // ===== MPC Frequency and Time Scale Logging =====
  const ros::WallTime nowWall = ros::WallTime::now();
  if (lastWallTime_.toSec() > 0.0) {
    const double wallDt = (nowWall - lastWallTime_).toSec();
    const double simDt = observation.time - lastSimTime_;
    wallTimeAccumulator_ += wallDt;
    simTimeAccumulator_ += simDt;
    timingStepCount_++;

    // Log every ~2 seconds of wall-clock time
    if (wallTimeAccumulator_ >= 2.0 && timingStepCount_ > 0) {
      const double avgWallDt = wallTimeAccumulator_ / timingStepCount_;
      const double mrtFreq = 1.0 / avgWallDt;  // actual MRT observer loop frequency
      const double timeScale = simTimeAccumulator_ / wallTimeAccumulator_;  // sim_time / wall_time
      ROS_INFO("[Timing] MRT loop: %.1f Hz (target 400) | Time scale: %.3fx real-time | Sim time: %.2f s | Wall per step: %.1f ms",
               mrtFreq, timeScale, observation.time, avgWallDt * 1000.0);
      wallTimeAccumulator_ = 0.0;
      simTimeAccumulator_ = 0.0;
      timingStepCount_ = 0;
    }
  }
  lastWallTime_ = nowWall;
  lastSimTime_ = observation.time;

  // Throttle visualization publishing to ~30 Hz (wall-clock) to avoid flooding RViz
  const double vizDt = (nowWall - lastVizWallTime_).toSec();
  if (vizDt < minVizPeriod_) {
    return;  // Skip this update, too soon since last visualization publish
  }
  lastVizWallTime_ = nowWall;

  updateCentroidalState(observation);
  updateFullBodyState(observation);
  publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                  primalSolution.modeSchedule_);

  publishRealTrajectory(timeStamp, observation.state);
}

void MultiRobotVisualizer::updateObservation(const SystemObservation& observation) {
  currentTime = observation.time;
  const auto timeStamp = ros::Time::now();
  if(currentTime < 0.2) {
    publishTerrain();
  }

  updateCentroidalState(observation);
  PublishHandleTF(observation);

}

void MultiRobotVisualizer::updatePolicy(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  currentTime = observation.time;
  const auto timeStamp = ros::Time::now();
  
  updateCentroidalState(observation);
  PublishHandleTF(observation);
  publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
  publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
                                  primalSolution.modeSchedule_);
}

void MultiRobotVisualizer::publishTerrain() {
  terrainPublisher_.publish(terrain_msg);
}

void MultiRobotVisualizer::getTerrainMarker() {
  // x-y area patch that should be drawn in rviz
  double dxy   =  0.06;
  double x_min = -1.0;
  double x_max =  4.0;
  double y_min = -1.0;
  double y_max =  1.0;

  visualization_msgs::Marker m;
  int id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.scale.z = 0.003;
  m.ns = "terrain";
  m.header.frame_id = frameId_;
//    m.color.r = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.r = 255./255; m.color.g  = 204./255; m.color.b  = 204./255; // pink
  m.color.a = 0.7;

  visualization_msgs::MarkerArray msg;
  double x =  x_min;
  while (x < x_max) {
    double y = y_min;
    while (y < y_max) {
    // position
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = heightMapPtr_->GetHeight(x,y);

    // orientation
    Eigen::Vector3d n = heightMapPtr_->GetNormalizedBasis(HeightMap::Normal, x, y);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), n);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    // enlarge surface-path when tilting
    double gain = 1.5;
    m.scale.x = (1+gain*n.cwiseAbs().x())*dxy;
    m.scale.y = (1+gain*n.cwiseAbs().y())*dxy;


    m.id = id++;
    terrain_msg.markers.push_back(m);

    y += dxy;
    }
    x += dxy;
  }
}

void MultiRobotVisualizer::publishRealTrajectory(ros::Time timeStamp, const vector_t& stateTrajectory) {
  vector_t states = vector_t::Zero(3); 
  const size_t StateOffset = SINGLE_ROBOT_STATE_DIM;

  // Accumulate trajectory points for all robots + cargo
  for (size_t robot = 0; robot < numRobots_ + 1; ++robot) {
    states = stateTrajectory.segment(robot * StateOffset, 3);
    geometry_msgs::Pose robot_pose;
    robot_pose.position = getPointMsg(states);
    realTrajectoryPoints_[robot].push_back(robot_pose.position);
    if (realTrajectoryPoints_[robot].size() > storeLen) {
      realTrajectoryPoints_[robot].erase(realTrajectoryPoints_[robot].begin());
    }
  }

  // Accumulate foot trajectory points
  vector_t footPositions = vector_t::Zero(12);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    footPositions = stateTrajectory.segment(robot * StateOffset + 12, 12);
    for (size_t foot = 0; foot < QUADRUPED_FOOT_NUM; ++foot) {
      const vector3_t footPos = footPositions.segment(foot * 3, 3);
      if (footPos.array().isNaN().any()) continue;
      geometry_msgs::Pose foot_pose;
      foot_pose.position = getPointMsg(footPos);
      realFootTrajectoryPoints_[robot][foot].push_back(foot_pose.position);
      if (realFootTrajectoryPoints_[robot][foot].size() > storeLen) {
        realFootTrajectoryPoints_[robot][foot].erase(realFootTrajectoryPoints_[robot][foot].begin());
      }
    }
  }

  // Throttle publishing to ~2 Hz to avoid killing RViz FPS
  recordTimeIndex_++;
  if (recordTimeIndex_ % 15 != 0) return;

  // Publish base trajectories (need >= 2 points for LINE_STRIP)
  std::vector<Color> robotColors = {Color::green, Color::red, Color::blue, Color::orange};
  for (size_t robot = 0; robot < numRobots_ + 1; ++robot) {
    if (realTrajectoryPoints_[robot].size() < 2) continue;
    auto pointsCopy = realTrajectoryPoints_[robot];
    auto line = getLineMsg(std::move(pointsCopy), robotColors[robot % robotColors.size()], trajectoryLineWidth_);
    line.header = getHeaderMsg(frameId_, timeStamp);
    line.ns = "real_trajectory_robot_" + std::to_string(robot + 1);
    line.id = recordTimeIndex_;
    robot_realTrajectoryPublishers_[robot].publish(line);
  }

  // Publish foot trajectories (need >= 2 points for LINE_STRIP)
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    for (size_t foot = 0; foot < QUADRUPED_FOOT_NUM; ++foot) {
      if (realFootTrajectoryPoints_[robot][foot].size() < 2) continue;
      auto footPointsCopy = realFootTrajectoryPoints_[robot][foot];
      auto line = getLineMsg(std::move(footPointsCopy), feetColorMap_[foot % feetColorMap_.size()], trajectoryLineWidth_);
      line.header = getHeaderMsg(frameId_, timeStamp);
      line.ns = "real_foot_trajectory_robot_" + std::to_string(robot + 1) + "_foot_" + std::to_string(foot + 1);
      line.id = recordTimeIndex_;
      robot_realfootPositionPublishers_[robot][foot].publish(line);
    }
  }
}


void MultiRobotVisualizer::publishRobotArmForces(ros::Time timeStamp, const std::vector<vector3_t>& armPositions,
                               const std::vector<vector3_t>& armForces) {
  // Add arm positions and forces to the batch (don't publish immediately)
  // Use different namespaces to avoid ID conflicts with foot markers
  const size_t startIndex = currentState_msgs.markers.size();
  for (size_t i = 0; i < armPositions.size(); ++i) {
    auto footMarker = getFootMarker(armPositions[i], true, Color::green, footMarkerDiameter_, footAlphaWhenLifted_);
    footMarker.ns = "Arm EE Positions";  // Different namespace to avoid conflicts
    currentState_msgs.markers.emplace_back(footMarker);
    
    auto forceMarker = getForceMarker(armForces[i], armPositions[i], true, Color::green, forceScale_);
    forceMarker.ns = "Arm EE Forces";  // Different namespace to avoid conflicts
    currentState_msgs.markers.emplace_back(forceMarker);
  }

  // Give markers an id and a frame
  assignHeader(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishRobotBaseTransformsForElevationMapping(ros::Time timeStamp, 
                                                                          const SystemObservation& observation) {
  // Publish base transforms for each robot (used by elevation mapping in perceptive terrain mode)
  // Each robot's base frame is named "base_N" where N is 1-indexed
  // Also publish a primary "base" frame using the first robot's pose for single-robot elevation mapping configs

  for (size_t robotId = 0; robotId < numRobots_; ++robotId) {
    const size_t stateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    
    // Check if we have enough state for this robot
    if (observation.state.size() < stateOffset + SINGLE_ROBOT_STATE_DIM) {
      continue;
    }
    
    // Extract COM position and orientation from state
    const vector3_t comPosition = observation.state.segment(stateOffset, 3);
    const vector3_t eulerZyx = observation.state.segment(stateOffset + 6, 3);
    
    // Check for NaN
    if (comPosition.array().isNaN().any() || eulerZyx.array().isNaN().any()) {
      continue;
    }
    
    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(eulerZyx);

    // Publish robot-specific base frame (base_1, base_2, etc.)
    geometry_msgs::TransformStamped robotBaseTransform;
    robotBaseTransform.header = getHeaderMsg(frameId_, timeStamp);
    robotBaseTransform.child_frame_id = "base_" + std::to_string(robotId + 1);
    robotBaseTransform.transform.translation = getVectorMsg(comPosition);
    robotBaseTransform.transform.rotation = getOrientationMsg(q_world_base);
    tfBroadcaster_.sendTransform(robotBaseTransform);

    // Publish primary "base" frame using first robot's pose (for elevation_mapping compatibility)
    if (robotId == 0) {
      geometry_msgs::TransformStamped primaryBaseTransform;
      primaryBaseTransform.header = getHeaderMsg(frameId_, timeStamp);
      primaryBaseTransform.child_frame_id = "base";
      primaryBaseTransform.transform.translation = getVectorMsg(comPosition);
      primaryBaseTransform.transform.rotation = getOrientationMsg(q_world_base);
      tfBroadcaster_.sendTransform(primaryBaseTransform);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
  const vector3_t& ellip_size, Color color, const std::string& frameName) {
  
  geometry_msgs::TransformStamped centroidToWorldTransform;
  centroidToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  centroidToWorldTransform.child_frame_id = frameName;

  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(centroidPose.tail<3>()));
  centroidToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  centroidToWorldTransform.transform.translation = getVectorMsg(centroidPose.head<3>());
  tfBroadcaster_.sendTransform(centroidToWorldTransform);

  // Add ellipsoid marker to the batch (don't publish immediately)
  visualization_msgs::Marker ellipsoid;
  ellipsoid.header = getHeaderMsg(frameId_, timeStamp);
  ellipsoid.type = visualization_msgs::Marker::SPHERE;
  ellipsoid.pose.position.x = centroidPose[0];
  ellipsoid.pose.position.y = centroidPose[1];
  ellipsoid.pose.position.z = centroidPose[2];
  ellipsoid.pose.orientation = getOrientationMsg(q_world_base);

  ellipsoid.scale.x = ellip_size[0];
  ellipsoid.scale.y = ellip_size[1];
  ellipsoid.scale.z = ellip_size[2];

  const auto rgb = getRGB(color);
  ellipsoid.color.r = rgb[0];
  ellipsoid.color.g = rgb[1];
  ellipsoid.color.b = rgb[2];
  ellipsoid.color.a = 0.8;
  ellipsoid.ns = frameName;
  ellipsoid.id = 0;

  centroid_msgs.markers.push_back(ellipsoid);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishCargoTransform(ros::Time timeStamp, const vector_t& cargoPose, 
  const vector3_t& cargo_size, Color color, const std::string& frameName) {

  geometry_msgs::TransformStamped cargoToWorldTransform;
  cargoToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  cargoToWorldTransform.child_frame_id = frameName;

  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(cargoPose.tail<3>()));
  cargoToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  cargoToWorldTransform.transform.translation = getVectorMsg(cargoPose.head<3>());
  // Also publish to cargo root link so the URDF's robot_state_publisher (tf_prefix=cargo) can connect its frames
  geometry_msgs::TransformStamped cargoToWorldTransform2;
  cargoToWorldTransform2 = cargoToWorldTransform;
  cargoToWorldTransform2.child_frame_id = cargoRootFrame_;
  tfBroadcaster_.sendTransform(cargoToWorldTransform);
  tfBroadcaster_.sendTransform(cargoToWorldTransform2);

  // Add cube marker to the batch (don't publish immediately)
  visualization_msgs::Marker cube;
  cube.header = getHeaderMsg(frameId_, timeStamp);
  cube.type = visualization_msgs::Marker::CUBE;
  cube.pose.position.x = cargoPose[0];
  cube.pose.position.y = cargoPose[1];
  cube.pose.position.z = cargoPose[2];
  cube.pose.orientation = getOrientationMsg(q_world_base);

  cube.scale.x = cargo_size[0];
  cube.scale.y = cargo_size[1];
  cube.scale.z = cargo_size[2];

  const auto rgb = getRGB(color);
  cube.color.r = rgb[0];
  cube.color.g = rgb[1];
  cube.color.b = rgb[2];
  cube.color.a = 0.8;
  cube.ns = frameName;
  cube.id = 0;

  centroid_msgs.markers.push_back(cube);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishCartesianMarkers(ros::Time timeStamp, const std::vector<bool>& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) {
  // Add feet positions and forces to the batch (don't publish immediately)
  const size_t startIndex = currentState_msgs.markers.size();
  const size_t totalFeet = feetPositions.size();
  for (size_t i = 0; i < totalFeet; ++i) {
    currentState_msgs.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i % 4], footMarkerDiameter_, footAlphaWhenLifted_));
    currentState_msgs.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  // currentState_msgs.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
  //                                                            contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  // currentState_msgs.markers.emplace_back(
  //     getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories) {
  const auto& stateTrajectory = targetTrajectories.stateTrajectory;
  const auto& inputTrajectory = targetTrajectories.inputTrajectory;

  // Check if we have enough points for LINE_STRIP markers (need at least 2 points)
  if (stateTrajectory.size() < 2) {
    return;  // Don't publish if we don't have enough points
  }

  // Reserve messages for all robots
  std::vector<std::vector<geometry_msgs::Point>> robot_desiredBasePositionMsgs(numRobots_);
  std::vector<std::vector<std::vector<geometry_msgs::Point>>> robot_desiredFeetPositionMsgs(numRobots_);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robot_desiredBasePositionMsgs[robot].reserve(stateTrajectory.size());
    robot_desiredFeetPositionMsgs[robot].resize(QUADRUPED_FOOT_NUM);
    for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
      robot_desiredFeetPositionMsgs[robot][i].reserve(stateTrajectory.size());
    }
  }

  for (size_t j = 0; j < stateTrajectory.size(); j++) {
    const auto state = stateTrajectory.at(j);
    
    // Check state size
    const size_t expectedStateSize = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
    if (state.size() < expectedStateSize) {
      std::cerr << "[MultiRobotVisualizer] Warning: State size mismatch in desired trajectory. Expected: " 
                << expectedStateSize << ", got: " << state.size() << std::endl;
      continue; // Skip this state
    }

    // Extract trajectories for all robots
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
      
      // Verify we have enough state for this robot
      if (state.size() < robotStateOffset + SINGLE_ROBOT_STATE_DIM) {
        std::cerr << "[MultiRobotVisualizer] Warning: State too small for robot " << robot << " in desired trajectory" << std::endl;
        continue;
      }
      
      // Robot base trajectory
      geometry_msgs::Pose robot_pose;
      robot_pose.position = getPointMsg(state.segment(robotStateOffset, 3));
      robot_desiredBasePositionMsgs[robot].push_back(robot_pose.position);

      // Robot feet trajectories
      for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
        const size_t footOffset = robotStateOffset + 12 + i * 3;
        if (state.size() >= footOffset + 3) {
      geometry_msgs::Pose footPose;
          footPose.position = getPointMsg(state.segment(footOffset, 3));
          robot_desiredFeetPositionMsgs[robot][i].push_back(footPose.position);
        } else {
          std::cerr << "[MultiRobotVisualizer] Warning: State too small for robot " << robot << " foot " << i << " in desired trajectory" << std::endl;
        }
      }
    }
  }

  // Create and publish trajectories for all robots
  std::vector<Color> robotColors = {Color::green, Color::red, Color::blue, Color::orange};
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    auto robot_comLineMsg = getLineMsg(std::move(robot_desiredBasePositionMsgs[robot]), 
                                       robotColors[robot % robotColors.size()], trajectoryLineWidth_);
    robot_comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    robot_comLineMsg.id = 0;
    robot_costDesiredBasePositionPublishers_[robot].publish(robot_comLineMsg);

    for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
      auto footLineMsg = getLineMsg(std::move(robot_desiredFeetPositionMsgs[robot][i]), 
                                     feetColorMap_[i], trajectoryLineWidth_);
    footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    footLineMsg.id = 0;
      robot_costDesiredFeetPositionPublishers_[robot][i].publish(footLineMsg);
    }
  }

  // =============== CARGO TRAJECTORY VISUALIZATION ===============
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  
  // Cargo trajectory as LINE_STRIP marker (same style as robot base trajectories)
  std::vector<geometry_msgs::Point> cargoPositionMsgs;
  cargoPositionMsgs.reserve(stateTrajectory.size());
  
  for (size_t i = 0; i < stateTrajectory.size(); ++i) {
    const auto& state = stateTrajectory[i];
    if (state.size() > cargoStateOffset + 2) {
      cargoPositionMsgs.push_back(getPointMsg(state.segment(cargoStateOffset, 3)));
    }
  }
  
  if (cargoPositionMsgs.size() >= 2) {
    auto cargoLineMsg = getLineMsg(std::move(cargoPositionMsgs), Color::orange, trajectoryLineWidth_);
    cargoLineMsg.header = getHeaderMsg(frameId_, timeStamp);
    cargoLineMsg.id = 0;
    cargoTargetTrajectoryPub_.publish(cargoLineMsg);
  }
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishDesiredArmPose(ros::Time timeStamp, const std::vector<vector3_t>& position, 
      const std::vector< Eigen::Quaternion<scalar_t> >& orientation) {
  
  // Create marker array for arm pose visualization
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.reserve(numRobots_); // One for position sphere, one for orientation arrow
  for (size_t robotIndex = 0; robotIndex < numRobots_; ++robotIndex) {
    // Create position marker (sphere)
    visualization_msgs::Marker positionMarker;
    positionMarker.header = getHeaderMsg(frameId_, timeStamp);
    positionMarker.ns = robot_namespaces_[robotIndex];
    positionMarker.id = robotIndex;
    positionMarker.type = visualization_msgs::Marker::SPHERE;
    positionMarker.action = visualization_msgs::Marker::ADD;
    
    positionMarker.pose.position = getPointMsg(position[robotIndex]);
    positionMarker.pose.orientation = getOrientationMsg(orientation[robotIndex]);
    
    positionMarker.scale.x = 0.05; // 5cm diameter sphere
    positionMarker.scale.y = 0.05;
    positionMarker.scale.z = 0.05;
    
    std::vector<double> rgb(3, 0.0);
    rgb[robotIndex % 3] = 1.0; // Red, Green, or Blue based on robot index
    positionMarker.color.r = rgb[0]; 
    positionMarker.color.g = rgb[1];
    positionMarker.color.b = rgb[2];
    positionMarker.color.a = 0.8;
    
    markerArray.markers.push_back(positionMarker);
    
    // Create orientation marker (arrow)
    visualization_msgs::Marker orientationMarker;
    orientationMarker.header = getHeaderMsg(frameId_, timeStamp);
    orientationMarker.ns = robot_namespaces_[robotIndex];
    orientationMarker.id = 1;
    orientationMarker.type = visualization_msgs::Marker::ARROW;
    orientationMarker.action = visualization_msgs::Marker::ADD;
    
    orientationMarker.pose.position = getPointMsg(position[robotIndex]);
    orientationMarker.pose.orientation = getOrientationMsg(orientation[robotIndex]);
    
    orientationMarker.scale.x = 0.1; // Arrow length
    orientationMarker.scale.y = 0.01; // Arrow width
    orientationMarker.scale.z = 0.01; // Arrow height
    
    // Same color as position marker but with different alpha
    orientationMarker.color = positionMarker.color;
    orientationMarker.color.a = 0.9;
    
    markerArray.markers.push_back(orientationMarker);
    
    // Publish the markers
    robot_armDesiredPosePublishers_[robotIndex].publish(markerArray); 
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MultiRobotVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                                           const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
  if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
    std::cout << "Nothing is here" << std::endl;
    return;
  }

  // Check if we have enough points for LINE_STRIP markers (need at least 2 points)
  if (mpcStateTrajectory.size() < 2) {
    return;  // Don't publish if we don't have enough points
  }

  // Reserve messages for all robots
  std::vector<std::vector<std::vector<geometry_msgs::Point>>> robot_feetMsgs(numRobots_);
  std::vector<std::vector<geometry_msgs::Point>> robot_mpcComPositionMsgs(numRobots_);

  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robot_feetMsgs[robot].resize(QUADRUPED_FOOT_NUM);
    std::for_each(robot_feetMsgs[robot].begin(), robot_feetMsgs[robot].end(), [&](std::vector<geometry_msgs::Point>& v) { 
    v.reserve(mpcStateTrajectory.size()); 
  });
    robot_mpcComPositionMsgs[robot].reserve(mpcStateTrajectory.size());
  }

  // Reserve message for cargo
  const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
  std::vector<geometry_msgs::Point> cargo_mpcPositionMsgs;
  cargo_mpcPositionMsgs.reserve(mpcStateTrajectory.size());
  

  // Extract trajectories for all robots
  std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
    // Check state size
    const size_t expectedStateSize = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
    if (state.size() < expectedStateSize) {
      std::cerr << "[MultiRobotVisualizer] Warning: State size mismatch in optimized trajectory. Expected: " 
                << expectedStateSize << ", got: " << state.size() << std::endl;
      return; // Skip this state
    }
    
    // Cargo trajectory
    if (state.size() > cargoStateOffset + 2) {
    geometry_msgs::Pose cargo_pose;
      cargo_pose.position = getPointMsg(state.segment(cargoStateOffset, 3));
    cargo_mpcPositionMsgs.push_back(cargo_pose.position);
    }

    // Extract trajectories for all robots
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
      
      // Verify we have enough state for this robot
      if (state.size() < robotStateOffset + SINGLE_ROBOT_STATE_DIM) {
        std::cerr << "[MultiRobotVisualizer] Warning: State too small for robot " << robot << std::endl;
        continue;
      }
      
      // Robot COM trajectory
      geometry_msgs::Pose robot_pose;
      robot_pose.position = getPointMsg(state.segment(robotStateOffset, 3));
      robot_mpcComPositionMsgs[robot].push_back(robot_pose.position);

      // Robot feet trajectories
      for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
        const size_t footOffset = robotStateOffset + 12 + i * 3;
        if (state.size() >= footOffset + 3) {
          const auto position = getPointMsg(state.segment(footOffset, 3));
          // Check for NaN or invalid values
          if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
            std::cerr << "[MultiRobotVisualizer] Warning: NaN detected for robot " << robot + 1 
                      << " foot " << i << " at offset " << footOffset << std::endl;
            continue; // Skip this point
          }
          robot_feetMsgs[robot][i].push_back(position);
        } else {
          std::cerr << "[MultiRobotVisualizer] Warning: State too small for robot " << robot + 1 
                    << " (index " << robot << ") foot " << i << " (offset " << footOffset 
                    << ", state size " << state.size() << ")" << std::endl;
        }
      }
    }
  });

  // Create marker array for all robots
  visualization_msgs::MarkerArray markerArray;

  // First, add a DELETEALL marker to clear any stale markers from previous publishes
  // (marker count can vary between iterations, leaving ghost markers in RViz)
  visualization_msgs::Marker deleteAllMarker;
  deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.push_back(deleteAllMarker);

  markerArray.markers.reserve(numRobots_ * (QUADRUPED_FOOT_NUM + 1) + 2);  // 4 feet + 1 COM per robot + cargo + deleteAll

  // Robot markers
  std::vector<Color> robotColors = {Color::red, Color::blue, Color::green, Color::orange};
  const std::vector<std::string> footNames = {"FL", "FR", "RL", "RR"};
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const std::string robotNs = "Robot" + std::to_string(robot + 1) + "_";

    // Feet trajectories - use unique namespace per foot to avoid conflicts
    for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
      // Only create marker if we have at least 2 points (required for LINE_STRIP)
      if (robot_feetMsgs[robot][i].size() >= 2) {
        markerArray.markers.emplace_back(getLineMsg(std::move(robot_feetMsgs[robot][i]), feetColorMap_[i], trajectoryLineWidth_));
        markerArray.markers.back().ns = robotNs + "EE_Trajectory_" + footNames[i];
      } else if (robot_feetMsgs[robot][i].size() > 0) {
        // If we have only 1 point, create a SPHERE marker instead
        visualization_msgs::Marker sphereMarker;
        sphereMarker.type = visualization_msgs::Marker::SPHERE;
        sphereMarker.pose.position = robot_feetMsgs[robot][i][0];
        sphereMarker.scale.x = trajectoryLineWidth_;
        sphereMarker.scale.y = trajectoryLineWidth_;
        sphereMarker.scale.z = trajectoryLineWidth_;
        sphereMarker.color = getColor(feetColorMap_[i]);
        sphereMarker.ns = robotNs + "EE_Trajectory_" + footNames[i];
        markerArray.markers.push_back(sphereMarker);
      } else {
        // Debug: log when trajectory is empty
        std::cerr << "[MultiRobotVisualizer] Warning: Empty trajectory for robot " << robot + 1 
                  << " foot " << footNames[i] << std::endl;
      }
  }
    
    // COM trajectory - only create if we have at least 2 points
    if (robot_mpcComPositionMsgs[robot].size() >= 2) {
      markerArray.markers.emplace_back(getLineMsg(std::move(robot_mpcComPositionMsgs[robot]), 
                                                  robotColors[robot % robotColors.size()], trajectoryLineWidth_));
      markerArray.markers.back().ns = robotNs + "CoM_Trajectory";
    } else if (robot_mpcComPositionMsgs[robot].size() > 0) {
      // If we have only 1 point, create a SPHERE marker instead
      visualization_msgs::Marker sphereMarker;
      sphereMarker.type = visualization_msgs::Marker::SPHERE;
      sphereMarker.pose.position = robot_mpcComPositionMsgs[robot][0];
      sphereMarker.scale.x = trajectoryLineWidth_;
      sphereMarker.scale.y = trajectoryLineWidth_;
      sphereMarker.scale.z = trajectoryLineWidth_;
      sphereMarker.color = getColor(robotColors[robot % robotColors.size()]);
      sphereMarker.ns = robotNs + "CoM_Trajectory";
      markerArray.markers.push_back(sphereMarker);
    } else {
      std::cerr << "[MultiRobotVisualizer] Warning: Empty COM trajectory for robot " << robot + 1 << std::endl;
    }
  }

  // Cargo marker (namespace: "Cargo_...")
  if (!cargo_mpcPositionMsgs.empty()) {
  markerArray.markers.emplace_back(getLineMsg(std::move(cargo_mpcPositionMsgs), Color::green, trajectoryLineWidth_));
  markerArray.markers.back().ns = "Cargo_Trajectory";
  }

  // Future footholds (touchdown positions) for all robots
  // Create separate sphere lists for each robot for better visualization
  std::vector<visualization_msgs::Marker> robotFootholdMarkers(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotFootholdMarkers[robot].type = visualization_msgs::Marker::SPHERE_LIST;
    robotFootholdMarkers[robot].scale.x = footMarkerDiameter_;
    robotFootholdMarkers[robot].scale.y = footMarkerDiameter_;
    robotFootholdMarkers[robot].scale.z = footMarkerDiameter_;
    robotFootholdMarkers[robot].ns = "Robot" + std::to_string(robot + 1) + "_Future_footholds";
    robotFootholdMarkers[robot].pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  }
  
  // Process future footholds for all robots
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto& subsystemSequence = modeSchedule.modeSequence;
  const auto tStart = mpcTimeTrajectory.front();
  const auto tEnd = mpcTimeTrajectory.back();
  
  for (size_t event = 0; event < eventTimes.size(); ++event) {
    if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {
      // modeNumber2StanceLeg returns a 4-element array for a single quadruped
      // The mode sequence applies to all robots (synchronized gait)
      const auto preEventContactFlags = quadruped::modeNumber2StanceLeg(subsystemSequence[event]);
      const auto postEventContactFlags = quadruped::modeNumber2StanceLeg(subsystemSequence[event + 1]);
      const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

      // Process future footholds for all robots
      for (size_t robot = 0; robot < numRobots_; ++robot) {
        const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
        
        for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
          // Use local foot index (0-3) for contact flag lookup since all robots share the same gait
          if (!preEventContactFlags[i] && postEventContactFlags[i]) {
            // This foot is transitioning from swing to stance (touchdown)
            const size_t footOffset = robotStateOffset + 12 + i * 3;
            if (postEventState.size() >= footOffset + 3) {
              robotFootholdMarkers[robot].points.emplace_back(getPointMsg(postEventState.segment(footOffset, 3)));
              robotFootholdMarkers[robot].colors.push_back(getColor(feetColorMap_[i]));
            }
          }
        }
      }
    }
  }

  // Add foothold markers for each robot
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    if (!robotFootholdMarkers[robot].points.empty()) {
      markerArray.markers.push_back(std::move(robotFootholdMarkers[robot]));
    }
  }

  // Add end sphere for cargo
  visualization_msgs::Marker cargoEndMarker;
  cargoEndMarker.type = visualization_msgs::Marker::SPHERE;
  cargoEndMarker.ns = "Cargo_End";
  cargoEndMarker.scale.x = footMarkerDiameter_;
  cargoEndMarker.scale.y = footMarkerDiameter_;
  cargoEndMarker.scale.z = footMarkerDiameter_;
  cargoEndMarker.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
  cargoEndMarker.color = getColor(Color::green, 1.0);
  if (!mpcStateTrajectory.empty() && mpcStateTrajectory.back().size() > cargoStateOffset) {
    cargoEndMarker.pose.position = getPointMsg(mpcStateTrajectory.back().segment(cargoStateOffset, 3));
    markerArray.markers.push_back(std::move(cargoEndMarker));
  }

  // Add headers and IDs
  assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

  stateOptimizedPublisher_.publish(markerArray);
}

}  // namespace multi_robot
}  // namespace ocs2