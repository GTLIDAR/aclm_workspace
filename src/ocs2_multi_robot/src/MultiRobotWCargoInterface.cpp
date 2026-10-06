#include <pinocchio/fwd.hpp>

#include <ros/ros.h>

#include "ocs2_multi_robot/MultiRobotWCargoInterface.h"
#include "ocs2_multi_robot/terrain/HeightMapExamples.h"
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include "ocs2_multi_robot/reference_manager/PerceptiveMultiRobotReferenceManager.h"
#include "ocs2_multi_robot/precomputation/MultiRobotPerceptivePreComputation.h"
#include "ocs2_multi_robot/utils/OrientationTools.h"
#include "ocs2_multi_robot/constraint/OrientationConstraint.h"
#include "ocs2_multi_robot/constraint/PerceptiveObjectBoundConstraint.h"
#include "ocs2_multi_robot/constraint/MultiRobotFootCollisionConstraint.h"
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {
namespace multi_robot {

MultiRobotWCargoInterface::MultiRobotWCargoInterface(const std::string& taskFile, 
                                                     const std::string& libraryFolder,
                                                     bool verbose, bool loadCostMatrices) 
  : MultiRobotInterfaceAbstract(taskFile, libraryFolder, verbose), loadCostMatrices_(loadCostMatrices) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile_);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[MultiRobotWCargoInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[MultiRobotWCargoInterface] Task file not found: " + taskFilePath.string());
  }
  // create library folder if it does not exist
  boost::filesystem::path libraryFolderPath(libraryFolder);
  boost::filesystem::create_directories(libraryFolderPath);
  std::cerr << "[MultiRobotWCargoInterface] Generated library path: " << libraryFolderPath << std::endl;

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
    throw std::runtime_error("[MultiRobotWCargoInterface] No robot handles found in config file!");
  }
  
  std::cerr << "[MultiRobotWCargoInterface] Detected " << numRobots_ << " robots" << std::endl;
  
  // Generate robot foot names
  robotFootNames_.clear();
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotNum = robot + 1;
    robotFootNames_.push_back(std::to_string(robotNum) + "FL");
    robotFootNames_.push_back(std::to_string(robotNum) + "FR");
    robotFootNames_.push_back(std::to_string(robotNum) + "RL");
    robotFootNames_.push_back(std::to_string(robotNum) + "RR");
  }

  // New initialization scheme
  const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  initialState_.resize(stateDim);

  // Load the relative robot state (defines foot positions relative to robot COM)
  vector_t initialRobotState(24);  // 6 (COM) + 6 (orientation/AM) + 12 (foot positions)
  loadData::loadEigenMatrix(taskFile_, "initialRobotState", initialRobotState);
  
  // Load the cargo state (becomes last 12 elements)
  vector_t initialCargoState(12);  // 6 (COM) + 6 (orientation/AM)
  loadData::loadEigenMatrix(taskFile_, "initialCargoState", initialCargoState);
  
  // Load robot offsets relative to cargo
  std::vector<vector_t> robotOffsets(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotOffsets[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffsets[robot]);
  }
  
  // Set cargo state (last 12 elements)
  initialState_.segment(stateDim - CARGO_STATE_DIM, CARGO_STATE_DIM) = initialCargoState;
  
  // Compute each robot's state
  vector3_t cargo_euler = initialCargoState.segment(6, 3);
  Eigen::AngleAxis<scalar_t> yaw_c(cargo_euler(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
  Eigen::AngleAxis<scalar_t> pit_c(cargo_euler(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
  Eigen::AngleAxis<scalar_t> rol_c(cargo_euler(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
  Eigen::Quaternion<scalar_t> q_cargo_zyx = yaw_c * pit_c * rol_c;
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
    
    // COM position = cargo COM + robot offset
    initialState_.segment(robotStateOffset, 3) = initialCargoState.segment(0, 3) + robotOffsets[robot].segment(0, 3);
    initialState_(robotStateOffset + 2) = initialRobotState(2);
    initialState_.segment(robotStateOffset + 3, 3) = initialRobotState.segment(3, 3);  // COM velocity
    
    // Orientation composition (ZYX): R_robot = R_cargo * R_offset
    vector3_t robot_euler_offset = robotOffsets[robot].segment(3, 3);
    Eigen::AngleAxis<scalar_t> yaw_o(robot_euler_offset(0), Eigen::Matrix<scalar_t,3,1>(0,0,1));
    Eigen::AngleAxis<scalar_t> pit_o(robot_euler_offset(1), Eigen::Matrix<scalar_t,3,1>(0,1,0));
    Eigen::AngleAxis<scalar_t> rol_o(robot_euler_offset(2), Eigen::Matrix<scalar_t,3,1>(1,0,0));
    Eigen::Quaternion<scalar_t> q_offset_zyx = yaw_o * pit_o * rol_o;
    Eigen::Quaternion<scalar_t> q_robot = q_cargo_zyx * q_offset_zyx;
    matrix3_t R_robot = q_robot.toRotationMatrix();
    // Use RotM2ZYXEuler to avoid gimbal lock issues with eulerAngles()
    // This function handles the singular case (pitch = ±π/2) properly
    vector3_t robot_euler = RotM2ZYXEuler(R_robot);  // Returns [yaw, pitch, roll] in ZYX order
    
    // Debug output for robot 3
    if (robot == 2 && numRobots_ >= 3) {
      std::cerr << "[MultiRobotWCargoInterface] Robot 3 initialization:" << std::endl;
      std::cerr << "  Cargo euler: [" << cargo_euler.transpose() << "]" << std::endl;
      std::cerr << "  Robot offset euler: [" << robot_euler_offset.transpose() << "]" << std::endl;
      std::cerr << "  Computed robot euler: [" << robot_euler.transpose() << "]" << std::endl;
      std::cerr << "  Rotation matrix R_robot:" << std::endl << R_robot << std::endl;
    }
    
    initialState_.segment(robotStateOffset + 6, 3) = robot_euler;
    initialState_.segment(robotStateOffset + 9, 3) = initialRobotState.segment(9, 3);  // Angular momentum
    
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ee++) {
      const vector3_t p_in_default_frame = getNominalEePosWrtCom(ee);
      const vector3_t p_world_new = initialState_.segment(robotStateOffset, 3) + R_robot * p_in_default_frame;
      initialState_.segment(robotStateOffset + 12 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world_new;
    }
  }
  
  std::cerr << "x_init:   " << initialState_.transpose() << std::endl;

  // DDP SQP MPC settings
  ddpSettings_ = ddp::loadSettings(taskFile_, "ddp");
  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc");
  sqpSettings_ = sqp::loadSettings(taskFile_, "sqp");
  
  // Load swing trajectory config once to get base swing height
  auto swingTrajectoryConfig = quadruped::loadSwingTrajectorySettings(taskFile_, "swing_trajectory_config", verbose);
  
  // Create swing trajectory planners - one per robot for multi-robot terrain adaptation
  std::vector<std::shared_ptr<quadruped::SwingTrajectoryPlanner>> swingTrajectoryPlanners;
  swingTrajectoryPlanners.reserve(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    swingTrajectoryPlanners.push_back(
        std::make_shared<quadruped::SwingTrajectoryPlanner>(swingTrajectoryConfig, 4));
  }
  ROS_INFO("[MultiRobotWCargoInterface] Created %zu swing trajectory planners (one per robot)", numRobots_);

  // Terrain module
  std::string terrain_name;
  loadData::loadCppDataType(taskFile_, "terrain_settings.terrain_type", terrain_name);
  auto heightMap = HeightMap::MakeTerrain(terrain_mapping[terrain_name]);

  ros::NodeHandle nh;

  // Check if perceptive mode is enabled (global parameter)
  usePerceptive_ = false;
  ros::param::get("/use_perceptive", usePerceptive_);
  
  // Check if obstacle avoidance is enabled (global parameter)
  enableObstacleAvoidance_ = false;
  ros::param::get("/enable_obstacle_avoidance", enableObstacleAvoidance_);
  ROS_INFO("[MultiRobotWCargoInterface] Obstacle avoidance parameter: %s", enableObstacleAvoidance_ ? "ENABLED" : "DISABLED");

  // Create ConvexRegionSelector for perceptive terrain mode
  std::shared_ptr<HeightMap> heightMapForRefMgr;
  if (usePerceptive_) {
    ROS_INFO("[MultiRobotWCargoInterface] Perceptive terrain mode ENABLED");
    // Create GridMapHeightMap and PlanarTerrain for perceptive mode
    // Use initial foot z position as default height
    double initial_foot_z = initialState_(14);  // FL_z from first robot's initialState
    gridMapHeightMapPtr_ = std::make_shared<MultiRobotGridMapHeightMap>(initial_foot_z);
    planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
    
    // Load height filter parameters for planar region selection
    double minPlanarHeight = -std::numeric_limits<double>::infinity();
    double maxPlanarHeight = std::numeric_limits<double>::infinity();
    try {
      loadData::loadCppDataType(taskFile_, "perceptive.min_planar_height", minPlanarHeight);
      loadData::loadCppDataType(taskFile_, "perceptive.max_planar_height", maxPlanarHeight);
    } catch (const boost::property_tree::ptree_bad_path&) {
      ROS_WARN("[MultiRobotWCargoInterface] perceptive height filter is not configured; accepting all terrain heights");
    }
    ROS_INFO("[MultiRobotWCargoInterface] Planar height filter: [%f, %f]", minPlanarHeight, maxPlanarHeight);
    
    // Load CoM height for terrain-aware reference generation
    scalar_t comHeight = 0.5;
    loadData::loadCppDataType(taskFile_, "com_height.height", comHeight);
    ROS_INFO("[MultiRobotWCargoInterface] COM height: %f", comHeight);
    
    // Create ConvexRegionSelector with planarTerrain, heightMap, and comHeight
    convexRegionSelectorPtr_ = std::make_shared<MultiRobotConvexRegionSelector>(
        numRobots_, planarTerrainPtr_, gridMapHeightMapPtr_, 16, comHeight, minPlanarHeight, maxPlanarHeight);
    
    // Initialize visualization in ConvexRegionSelector
    ros::NodeHandle nh;
    convexRegionSelectorPtr_->initializeVisualization(nh, "odom");
    
    heightMapForRefMgr = gridMapHeightMapPtr_;
    ROS_INFO("[MultiRobotWCargoInterface] Using GridMapHeightMap with default height: %f", initial_foot_z);
  } else {
    ROS_INFO("[MultiRobotWCargoInterface] Perceptive terrain mode DISABLED");
    convexRegionSelectorPtr_ = nullptr;
    gridMapHeightMapPtr_ = nullptr;
    planarTerrainPtr_ = nullptr;
    heightMapForRefMgr = heightMap;
  }

  // Mode schedule manager
  if (usePerceptive_) {
    // Load CoM height for terrain-aware reference generation
    scalar_t comHeight = 0.5;
    loadData::loadCppDataType(taskFile_, "com_height.height", comHeight);
    
    // Load cargo height offset from config
    scalar_t cargoHeightOffset = 0.8;
    try {
      loadData::loadCppDataType(taskFile_, "cargo_height_offset.height", cargoHeightOffset);
      ROS_INFO("[MultiRobotWCargoInterface] Loaded cargo_height_offset: %f", cargoHeightOffset);
    } catch (...) {
      ROS_WARN("[MultiRobotWCargoInterface] cargo_height_offset not found in config, using default: %f", cargoHeightOffset);
    }
    
    // Use PerceptiveMultiRobotReferenceManager with convex region selector
    auto perceptiveRefMgr = std::make_shared<PerceptiveMultiRobotReferenceManager>(loadGaitSchedule(taskFile_, verbose),
            swingTrajectoryPlanners,
            heightMapForRefMgr,
            convexRegionSelectorPtr_,
            numRobots_,
            comHeight,
            cargoHeightOffset,
            swingTrajectoryConfig.swingHeight);
    
    // Pass robot base offsets for terrain-averaged cargo z computation
    std::vector<vector_t> robotOffsetsForRefMgr(numRobots_);
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      robotOffsetsForRefMgr[robot] = vector_t(6);
      loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffsetsForRefMgr[robot]);
    }
    perceptiveRefMgr->setRobotBaseOffsets(robotOffsetsForRefMgr);

    referenceManagerPtr_ = perceptiveRefMgr;
    ROS_INFO("[MultiRobotWCargoInterface] Using PerceptiveMultiRobotReferenceManager with %zu swing planners", numRobots_);
  } else {
    // Standard non-perceptive reference manager - use first swing planner
    referenceManagerPtr_ =
            std::make_shared<SwitchedModelReferenceManagerWithTerrain>(loadGaitSchedule(taskFile_, verbose),
            swingTrajectoryPlanners.empty() ? nullptr : swingTrajectoryPlanners[0],
          heightMapForRefMgr,
          numRobots_);
  }
  /*
   * Optimal control problem
   */
  ROS_INFO("good before setup OCP");
  setupOptimalControlProblem();
}

void MultiRobotWCargoInterface::setupOptimalControlProblem() {
  OptimalControlProblem problem;
  const size_t stateDim = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  const size_t inputDim = numRobots_ * SINGLE_ROBOT_INPUT_DIM + numRobots_ * ARM_CONTACT_DIM;
  
  // Cost
  matrix_t Q(stateDim, stateDim);
  matrix_t R(inputDim, inputDim);
  matrix_t Qf(stateDim, stateDim);

  if (loadCostMatrices_) {
    loadData::loadEigenMatrix(taskFile_, "centroidal_weights.Q", Q);
    loadData::loadEigenMatrix(taskFile_, "centroidal_weights.R", R);
    loadData::loadEigenMatrix(taskFile_, "centroidal_weights.Q_final", Qf);
  }

  // Validate matrix dimensions
  if (Q.rows() != stateDim || Q.cols() != stateDim) {
    throw std::runtime_error("[MultiRobotWCargoInterface] Q matrix dimension mismatch: expected " + 
                             std::to_string(stateDim) + "x" + std::to_string(stateDim) + 
                             ", got " + std::to_string(Q.rows()) + "x" + std::to_string(Q.cols()));
  }
  if (R.rows() != inputDim || R.cols() != inputDim) {
    throw std::runtime_error("[MultiRobotWCargoInterface] R matrix dimension mismatch: expected " + 
                             std::to_string(inputDim) + "x" + std::to_string(inputDim) + 
                             ", got " + std::to_string(R.rows()) + "x" + std::to_string(R.cols()));
  }
  if (Qf.rows() != stateDim || Qf.cols() != stateDim) {
    throw std::runtime_error("[MultiRobotWCargoInterface] Qf matrix dimension mismatch: expected " + 
                             std::to_string(stateDim) + "x" + std::to_string(stateDim) + 
                             ", got " + std::to_string(Qf.rows()) + "x" + std::to_string(Qf.cols()));
  }

  scalar_t robot_mass, cargo_mass;
  loadData::loadCppDataType(taskFile_, "robot_model.mass", robot_mass);
  loadData::loadCppDataType(taskFile_, "cargo_model.mass", cargo_mass);
  problem.costPtr->add("cost", std::make_unique<MultiRobotWCargoTrackingCost>(Q, R, robot_mass, cargo_mass, numRobots_, *referenceManagerPtr_));
  problem.finalCostPtr->add("finalCost", std::make_unique<MultiRobotWCargoStateTrackingCost>(Qf, numRobots_, *referenceManagerPtr_));
  ROS_INFO("good after setup cost");
  
  // Constraints
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile_, verbose_);

  // kinematics box constraint settings
  RelaxedBarrierPenalty::Config barrierPenaltyConfigKinematicsBox;
  barrierPenaltyConfigKinematicsBox = loadKinematicsBoxSettings(taskFile_, verbose_);

  // foot placement constraint settings
  RelaxedBarrierPenalty::Config barrierPenaltyConfigFootPlacement;
  barrierPenaltyConfigFootPlacement = loadFootPlacementSettings(taskFile_, verbose_);

  // perceptive foot placement constraint settings (only load if perceptive mode is enabled)
  RelaxedBarrierPenalty::Config barrierPenaltyConfigPerceptiveFootPlacement;
  if (usePerceptive_) {
    barrierPenaltyConfigPerceptiveFootPlacement = loadPerceptiveFootPlacementSettings(taskFile_, verbose_);
  }

  // arm kinematics box constraint settings
  RelaxedBarrierPenalty::Config barrierPenaltyConfigArmKinematicsBox;
  barrierPenaltyConfigArmKinematicsBox = loadArmKinematicsBoxSettings(taskFile_, verbose_);
  bool hard_constraint = false;

  ROS_INFO("good before setup constraints");
  const size_t totalFeet = numRobots_ * QUADRUPED_FOOT_NUM;
  
  // Get pointer to PerceptiveMultiRobotReferenceManager if in perceptive mode
  PerceptiveMultiRobotReferenceManager* perceptiveRefMgr = nullptr;
  if (usePerceptive_) {
    perceptiveRefMgr = dynamic_cast<PerceptiveMultiRobotReferenceManager*>(referenceManagerPtr_.get());
    if (perceptiveRefMgr == nullptr) {
      ROS_WARN("[MultiRobotWCargoInterface] Could not cast to PerceptiveMultiRobotReferenceManager for swing constraints");
    }
  }
  
  for(size_t indx = 0; indx < totalFeet; indx++) {
    const size_t robotId = indx / QUADRUPED_FOOT_NUM;
    const size_t footInRobot = indx % QUADRUPED_FOOT_NUM;
    const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    // valueIndex: for ZeroForceConstraint and ZeroVelocityConstraint
    // Maps foot index to input space: valueIndex * 3 = input offset for forces
    // Each robot has 8 "slots" in valueIndex space (4 feet forces + 4 feet velocities)
    // Robot 0: valueIndex = 0-3, Robot 1: valueIndex = 8-11, Robot 2: valueIndex = 16-19
    size_t valueIndex = robotId * QUADRUPED_FOOT_NUM * 2 + footInRobot;
    
    problem.softConstraintPtr->add(robotFootNames_[indx] + "_frictionCone",
                                        getFrictionConeConstraint(indx, frictionCoefficient, barrierPenaltyConfig));
    problem.equalityConstraintPtr->add(robotFootNames_[indx] + "_zeroForce", 
                std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, indx, valueIndex)));
    problem.equalityConstraintPtr->add(robotFootNames_[indx] + "_zeroVelocity",
                std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraint(*referenceManagerPtr_, indx, valueIndex)));
    
    if(hard_constraint) {
      problem.equalityConstraintPtr->add(robotFootNames_[indx] + "_footPlacement",
                  std::unique_ptr<StateInputConstraint>(new FootPlacementConstraint(*referenceManagerPtr_, indx, valueIndex, robotStateOffset)));
    } else {
      // Add perceptive foot placement constraint if enabled
      if (usePerceptive_ && convexRegionSelectorPtr_) {
        problem.softConstraintPtr->add(robotFootNames_[indx] + "_perceptiveFootPlacement",
                    getPerceptiveFootPlacementConstraint(indx, barrierPenaltyConfigPerceptiveFootPlacement));
        ROS_INFO_STREAM("[MultiRobotWCargoInterface] Added PerceptiveFootPlacementConstraint for " << robotFootNames_[indx]);
      }
      // Standard height-based foot placement constraint
      problem.softConstraintPtr->add(robotFootNames_[indx] + "_footPlacement",
                  getFootPlacementConstraint(indx, barrierPenaltyConfigFootPlacement));  
    }
    problem.softConstraintPtr->add(robotFootNames_[indx] + "_kinematicBox",
                                    getKinematicsBoxConstraint(indx, barrierPenaltyConfigKinematicsBox));
    
    // Normal velocity constraint for terrain-aware swing trajectories
    // This ensures foot z-velocity follows the swing trajectory planner during swing phase
    // The position error gain helps guide the foot to the correct terrain height
    if (perceptiveRefMgr != nullptr) {
      auto robotSwingPlanner = perceptiveRefMgr->getSwingTrajectoryPlannerForRobot(robotId);
      if (robotSwingPlanner) {
        const scalar_t positionErrorGain = 5.0;  // Position feedback gain for swing trajectory tracking
        problem.equalityConstraintPtr->add(robotFootNames_[indx] + "_normalVelocity",
                    std::unique_ptr<StateInputConstraint>(
                        new NormalVelocityConstraint(*referenceManagerPtr_, robotSwingPlanner, indx, valueIndex, 
                                                     robotStateOffset, positionErrorGain)));
        ROS_INFO_STREAM("[MultiRobotWCargoInterface] Added NormalVelocityConstraint for " << robotFootNames_[indx] 
                        << " with Robot " << robotId + 1 << " swing planner (posGain=" << positionErrorGain << ")");
      }
      
      // Add foot collision constraint for swing phase (SDF-based when available) - only in perceptive mode
      // if (perceptiveRefMgr) {
      //   scalar_t footCollisionClearance = 0.02;  // default: 2cm minimum clearance
      //   loadData::loadCppDataType(taskFile_, "footCollisionSoftConstraint.clearance", footCollisionClearance);
        
      //   auto footCollisionConstraint = std::make_unique<MultiRobotFootCollisionConstraint>(
      //       *perceptiveRefMgr, indx, robotStateOffset, footCollisionClearance);
        
      //   RelaxedBarrierPenalty::Config footCollisionPenaltyConfig;
      //   loadData::loadCppDataType(taskFile_, "footCollisionSoftConstraint.mu", footCollisionPenaltyConfig.mu);
      //   loadData::loadCppDataType(taskFile_, "footCollisionSoftConstraint.delta", footCollisionPenaltyConfig.delta);
      //   auto footCollisionPenalty = std::make_unique<RelaxedBarrierPenalty>(footCollisionPenaltyConfig);
        
      //   problem.softConstraintPtr->add(robotFootNames_[indx] + "_collision",
      //               std::make_unique<StateInputSoftConstraint>(std::move(footCollisionConstraint), std::move(footCollisionPenalty)));
      //   ROS_INFO_STREAM("[MultiRobotWCargoInterface] Added FootCollisionConstraint for " << robotFootNames_[indx] 
      //                   << " (clearance=" << footCollisionClearance << "m, mu=" << footCollisionPenaltyConfig.mu 
      //                   << ", delta=" << footCollisionPenaltyConfig.delta << ")");
      // }
    } else {
      // Non-perceptive mode: use base swing trajectory planner for all feet
      auto baseSwingPlanner = referenceManagerPtr_->getSwingTrajectoryPlanner();
      if (baseSwingPlanner) {
        const scalar_t positionErrorGain = 5.0;
        // Use footInRobot (0-3) for swing planner queries since planner is per-robot
        problem.equalityConstraintPtr->add(robotFootNames_[indx] + "_normalVelocity",
                    std::unique_ptr<StateInputConstraint>(
                        new NormalVelocityConstraint(*referenceManagerPtr_, baseSwingPlanner, indx, valueIndex, 
                                                     robotStateOffset, positionErrorGain)));
        ROS_INFO_STREAM("[MultiRobotWCargoInterface] Added NormalVelocityConstraint for " << robotFootNames_[indx] 
                        << " (non-perceptive mode, posGain=" << positionErrorGain << ")");
      }
    }
  }
  
  // Arm kinematics box constraints
  std::vector<vector_t> robotHandles(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotHandles[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile_, "cargo_model.handle_" + std::to_string(robot + 1), robotHandles[robot]);
  }
  problem.softConstraintPtr->add("armKinematicsBox",
                                  getArmKinematicsBoxConstraint(robotHandles, barrierPenaltyConfigArmKinematicsBox));
  
  // Robot-cargo formation constraint (keeps robots in position relative to cargo)
  RelaxedBarrierPenalty::Config barrierPenaltyConfigFormation;
  barrierPenaltyConfigFormation = loadFormationConstraintSettings(taskFile_, verbose_);
  
  vector3_t formationTolerance = vector3_t::Constant(0.3);  // default tolerance
  loadData::loadEigenMatrix(taskFile_, "formationConstraint.position_tolerance", formationTolerance);
  
  std::vector<vector_t> robotOffsets(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotOffsets[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffsets[robot]);
  }
  auto formationConstraint = std::make_unique<MultiRobotCargoFormationConstraint>(
      *referenceManagerPtr_, robotOffsets, formationTolerance, numRobots_);
  auto formationPenalty = std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfigFormation);
  problem.softConstraintPtr->add("robotCargoFormation",
                                  std::make_unique<StateInputSoftConstraint>(std::move(formationConstraint), std::move(formationPenalty)));
  
  // Orientation constraint settings (pitch and roll limits for robots and cargo)
  RelaxedBarrierPenalty::Config barrierPenaltyConfigOrientation;
  barrierPenaltyConfigOrientation = loadOrientationConstraintSettings(taskFile_, verbose_);
  
  // Orientation constraints for robots and cargo
  problem.stateSoftConstraintPtr->add("orientationConstraint",
                                      std::unique_ptr<StateCost>(new StateSoftConstraint(
                                          std::make_unique<OrientationConstraint>(taskFile_, numRobots_),
                                          std::make_unique<RelaxedBarrierPenalty>(barrierPenaltyConfigOrientation))));

  // Manipulation friction cone constraints
  scalar_t manipulationFrictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config manipulationFrictionBarrier;
  std::tie(manipulationFrictionCoefficient, manipulationFrictionBarrier) = loadManipulationFrictionConeSettings(taskFile_, verbose_);
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    auto manipFrictionConstraint = std::make_unique<ManipulationFrictionConeConstraint>(
        FrictionConeConstraint::Config(manipulationFrictionCoefficient), robot, robotHandles[robot], numRobots_);
    auto manipFrictionPenalty = std::make_unique<RelaxedBarrierPenalty>(manipulationFrictionBarrier);
    problem.softConstraintPtr->add("manipFriction_robot" + std::to_string(robot),
                                   std::make_unique<StateInputSoftConstraint>(std::move(manipFrictionConstraint), std::move(manipFrictionPenalty)));
  }
  
  // Manipulation torque box constraints (only if ARM_CONTACT_DIM == 6)
  if (ARM_CONTACT_DIM == 6) {
    vector3_t torqueLowerBound, torqueUpperBound;
    std::tie(torqueLowerBound, torqueUpperBound) = loadManipulationTorqueBounds(taskFile_, verbose_);
    RelaxedBarrierPenalty::Config torqueBoxBarrier = loadManipulationTorqueBoxSettings(taskFile_, verbose_);
    
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      auto manipTorqueConstraint = std::make_unique<ManipulationTorqueBoxConstraint>(robot, torqueLowerBound, torqueUpperBound, numRobots_);
      auto manipTorquePenalty = std::make_unique<RelaxedBarrierPenalty>(torqueBoxBarrier);
      problem.softConstraintPtr->add("manipTorque_robot" + std::to_string(robot),
                                      std::make_unique<StateInputSoftConstraint>(std::move(manipTorqueConstraint), std::move(manipTorquePenalty)));
    }
  }
  
  ROS_INFO("good before setup dynamics");
  // Dynamics
  bool recompileLibraries{false};
  ocs2::loadData::loadCppDataType(taskFile_, "centroidal_interface.recompileLibraries", recompileLibraries);
  matrix_t robot_inertia(3, 3);
  cargo_inertia_ = matrix_t(3, 3);
  loadData::loadEigenMatrix(taskFile_, "robot_model.inertia", robot_inertia);
  loadData::loadEigenMatrix(taskFile_, "cargo_model.inertia", cargo_inertia_);

  problem.dynamicsPtr.reset(new MultiRobotWCargoSRBDAD(robot_mass, cargo_mass, 
                                                    robot_inertia, cargo_inertia_,
                                                    robotHandles, numRobots_,
                                                    libraryFolder_, recompileLibraries));

  // Bound constraints - terrain-relative in perceptive mode, absolute otherwise
  RelaxedBarrierPenalty::Config boundsConfig;
  loadData::loadCppDataType(taskFile_, "penalty_config.bounds.mu", boundsConfig.mu);
  loadData::loadCppDataType(taskFile_, "penalty_config.bounds.delta", boundsConfig.delta);
  
  if (usePerceptive_ && gridMapHeightMapPtr_) {
    // Perceptive mode: use terrain-relative height bounds (PerceptiveObjectBoundConstraint)
    // Terrain height = average at nominal robot base positions (not at cargo position)
    std::vector<vector_t> boundsRobotOffsets(numRobots_);
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      boundsRobotOffsets[robot] = vector_t(6);
      loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), boundsRobotOffsets[robot]);
    }
    problem.stateSoftConstraintPtr->add("state_bound",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<PerceptiveObjectBoundConstraint>(taskFile_, numRobots_, gridMapHeightMapPtr_, boundsRobotOffsets),
                                            std::make_unique<RelaxedBarrierPenalty>(boundsConfig))));
    ROS_INFO("[MultiRobotWCargoInterface] PerceptiveObjectBoundConstraint ENABLED (terrain-relative height bounds, avg robot terrain)");
  } else {
    // Non-perceptive mode: use absolute z bounds (ObjectBoundConstraint)
    problem.stateSoftConstraintPtr->add("state_bound",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<ObjectBoundConstraint>(taskFile_, numRobots_),
                                            std::make_unique<RelaxedBarrierPenalty>(boundsConfig))));
    ROS_INFO("[MultiRobotWCargoInterface] ObjectBoundConstraint ENABLED (absolute height bounds)");
  }

  // CBFs
  RelaxedBarrierPenalty::Config cbfConfig;
  loadData::loadCppDataType(taskFile_, "penalty_config.cbf.mu", cbfConfig.mu);
  loadData::loadCppDataType(taskFile_, "penalty_config.cbf.delta", cbfConfig.delta);
  RelaxedBarrierPenalty::Config cbf2dConfig;
  loadData::loadCppDataType(taskFile_, "2d_penalty_config.mu", cbf2dConfig.mu);
  loadData::loadCppDataType(taskFile_, "2d_penalty_config.delta", cbf2dConfig.delta);

  // Obstacle avoidance constraints (only if enabled)
  if (enableObstacleAvoidance_) {
    ROS_INFO("[MultiRobotWCargoInterface] Obstacle avoidance ENABLED");
    
    // 2D Obstacles (box 1)
    vector_array_t obstacles_box_1_pose_array;
    matrix_t obstacles_box_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_1.pose", obstacles_box_1_pose_matrix);
    for (int i = 0; i < obstacles_box_1_pose_matrix.rows(); ++i)
    {
      obstacles_box_1_pose_array.push_back(obstacles_box_1_pose_matrix.row(i));
    }

    scalar_array_t obstacles_box_1_radius;
    loadData::loadStdVector(taskFile_, "obstacles_box_1.radius", obstacles_box_1_radius, verbose_);

    obstaclesBox1Ptr_.reset(new Obstacles(obstacles_box_1_pose_array));

    // 2D Obstacles (box 2)
    vector_array_t obstacles_box_2_pose_array;
    matrix_t obstacles_box_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_2.pose", obstacles_box_2_pose_matrix);
    for (int i = 0; i < obstacles_box_2_pose_matrix.rows(); ++i)
    {
      obstacles_box_2_pose_array.push_back(obstacles_box_2_pose_matrix.row(i));
    }

    scalar_array_t obstacles_box_2_radius;
    loadData::loadStdVector(taskFile_, "obstacles_box_2.radius", obstacles_box_2_radius, verbose_);

    obstaclesBox2Ptr_.reset(new Obstacles(obstacles_box_2_pose_array));

    // 2D Obstacles narrow path (wall 1)
    vector_array_t obstacles_wall_1_pose_array;
    matrix_t obstacles_wall_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_1.pose", obstacles_wall_1_pose_matrix);
    for (int i = 0; i < obstacles_wall_1_pose_matrix.rows(); ++i)
    {
      obstacles_wall_1_pose_array.push_back(obstacles_wall_1_pose_matrix.row(i));
    }

    scalar_array_t obstacles_wall_1_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_1.radius", obstacles_wall_1_radius, verbose_);

    obstaclesWall1Ptr_.reset(new Obstacles(obstacles_wall_1_pose_array));

    // 2D Obstacles narrow path (wall 2)
    vector_array_t obstacles_wall_2_pose_array;
    matrix_t obstacles_wall_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_2.pose", obstacles_wall_2_pose_matrix);
    for (int i = 0; i < obstacles_wall_2_pose_matrix.rows(); ++i)
    {
      obstacles_wall_2_pose_array.push_back(obstacles_wall_2_pose_matrix.row(i));
    }

    scalar_array_t obstacles_wall_2_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_2.radius", obstacles_wall_2_radius, verbose_);

    obstaclesWall2Ptr_.reset(new Obstacles(obstacles_wall_2_pose_array));

    // 3D Obstacles (wall 3)
    vector_array_t obstacles_wall_3_pose_array;
    matrix_t obstacles_wall_3_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_3.pose", obstacles_wall_3_pose_matrix);
    for (int i = 0; i < obstacles_wall_3_pose_matrix.rows(); ++i)
    {
      obstacles_wall_3_pose_array.push_back(obstacles_wall_3_pose_matrix.row(i));
    }

    scalar_array_t obstacles_wall_3_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_3.radius", obstacles_wall_3_radius, verbose_);

    obstaclesWall3Ptr_.reset(new Obstacles(obstacles_wall_3_pose_array));

    // CBF constraint for cargo (box 1, box 2, wall 1, wall 2, wall 3)
    problem.stateSoftConstraintPtr->add("Cargo_Box_1_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesBox1Ptr_, obstacles_box_1_radius, numRobots_),
                                                                                          std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Box_2_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesBox2Ptr_, obstacles_box_2_radius, numRobots_),
                                                                                          std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_1_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesWall1Ptr_, obstacles_wall_1_radius, numRobots_),
                                                                                          std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_2_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesWall2Ptr_, obstacles_wall_2_radius, numRobots_),
                                                                                          std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_3_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(std::make_unique<ObjectCBFConstraint>(obstaclesWall3Ptr_, obstacles_wall_3_radius, numRobots_),
                                                                                          std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));

    // CBF constraints for all robots (box 1, box 2, wall 1, wall 2)
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      problem.stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Box_1_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<RobotCBFConstraint>(obstaclesBox1Ptr_, obstacles_box_1_radius, robot),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem.stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Box_2_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<RobotCBFConstraint>(obstaclesBox2Ptr_, obstacles_box_2_radius, robot),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem.stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Wall_1_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<RobotCBFConstraint>(obstaclesWall1Ptr_, obstacles_wall_1_radius, robot),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem.stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Wall_2_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<RobotCBFConstraint>(obstaclesWall2Ptr_, obstacles_wall_2_radius, robot),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
    }
  } else {
    ROS_INFO("[MultiRobotWCargoInterface] Obstacle avoidance DISABLED");
  }

  // PreComputation for perceptive foot placement (caches (A,b) parameters)
  if (usePerceptive_ && convexRegionSelectorPtr_) {
    problem.preComputationPtr.reset(new MultiRobotPerceptivePreComputation(
        *convexRegionSelectorPtr_, numRobots_, 16));
    ROS_INFO("[MultiRobotWCargoInterface] Added MultiRobotPerceptivePreComputation for optimized constraint evaluation");
  }

  ROS_INFO("good before rollout");
  // Rollout
  auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  rolloutPtrs_.push_back(std::make_unique<TimeTriggeredRollout>(*problem.dynamicsPtr, rolloutSettings));
  ROS_INFO("good before init");
  // Initialization
  centroidalInitializerPtrs_.push_back(std::make_unique<MultiRobotWCargoInitializer>(robot_mass, cargo_mass, numRobots_, *referenceManagerPtr_));
  problems_.push_back(problem);
}

std::unique_ptr<StateInputCost> MultiRobotWCargoInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  const size_t robotId = contactPointIndex / QUADRUPED_FOOT_NUM;
  const size_t footInRobot = contactPointIndex % QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  // valueIndex maps to input space: valueIndex * 3 = input offset for forces
  size_t valueIndex = robotId * QUADRUPED_FOOT_NUM * 2 + footInRobot;
  std::unique_ptr<FrictionConeConstraint> frictionConeConstraintPtr(
          new FrictionConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, valueIndex, robotStateOffset));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionConeConstraintPtr), std::move(penalty)));
}

std::unique_ptr<StateInputCost> MultiRobotWCargoInterface::getKinematicsBoxConstraint(size_t contactPointIndex,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  const size_t robotId = contactPointIndex / QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  const size_t footInRobot = contactPointIndex % QUADRUPED_FOOT_NUM;
  // valueIndex maps to input space: valueIndex * 3 = input offset for velocities
  size_t valueIndex = robotId * QUADRUPED_FOOT_NUM * 2 + footInRobot;
  std::unique_ptr<KinematicsBoxConstraint> kinematicsBoxConstraintPtr(
          new KinematicsBoxConstraint(*referenceManagerPtr_, contactPointIndex, valueIndex, robotStateOffset));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(kinematicsBoxConstraintPtr), std::move(penalty)));
}

std::unique_ptr<StateInputCost> MultiRobotWCargoInterface::getFootPlacementConstraint(size_t contactPointIndex,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  const size_t robotId = contactPointIndex / QUADRUPED_FOOT_NUM;
  const size_t footInRobot = contactPointIndex % QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  // valueIndex maps to input space
  size_t valueIndex = robotId * QUADRUPED_FOOT_NUM * 2 + footInRobot;
  std::unique_ptr<FootPlacementConstraint> footPlacementConstraintPtr(
          new FootPlacementConstraint(*referenceManagerPtr_, contactPointIndex, valueIndex, robotStateOffset));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(footPlacementConstraintPtr), std::move(penalty)));
}

std::unique_ptr<StateInputCost> MultiRobotWCargoInterface::getPerceptiveFootPlacementConstraint(
    size_t contactPointIndex, const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  const size_t robotId = contactPointIndex / QUADRUPED_FOOT_NUM;
  const size_t footInRobot = contactPointIndex % QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  // valueIndex maps to input space: valueIndex * 3 = input offset for velocities
  size_t valueIndex = robotId * QUADRUPED_FOOT_NUM * 2 + footInRobot;
  
  std::unique_ptr<MultiRobotPerceptiveFootPlacementConstraint> perceptiveFootPlacementConstraintPtr(
      new MultiRobotPerceptiveFootPlacementConstraint(*referenceManagerPtr_, *convexRegionSelectorPtr_, 
                                                      contactPointIndex, valueIndex, robotStateOffset, 16));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(perceptiveFootPlacementConstraintPtr), std::move(penalty)));
}

std::shared_ptr<GaitSchedule> MultiRobotWCargoInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = quadruped::loadModeSchedule(file, "initialModeSchedule", false);
  std::string gait_name;
  loadData::loadCppDataType(file, "defaultModeSequenceTemplate.gait_name", gait_name); 
  auto defaultModeSequenceTemplate = quadruped::loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);
  if (gait_name == "trotting") {
    defaultModeSequenceTemplate = quadruped::loadModeSequenceTemplate(file, "trottingGait", false);
  } else if (gait_name == "jumping") {
    defaultModeSequenceTemplate = quadruped::loadModeSequenceTemplate(file, "jumpingGait", false);
  }

  double phaseTransitionStanceTime;
  loadData::loadCppDataType(file, "gait_settings.phaseTransitionStanceTime", phaseTransitionStanceTime);

  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<quadruped::GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, phaseTransitionStanceTime);
}

std::pair<scalar_t, RelaxedBarrierPenalty::Config> MultiRobotWCargoInterface::loadFrictionConeSettings(const std::string& taskFile,
                                                                                                  bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadKinematicsBoxSettings(const std::string& taskFile,
                                                                                   bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "kinematicsBoxSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Kinematics box Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadFootPlacementSettings(const std::string& taskFile,
                                                                                   bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacementSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Foot placement constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadPerceptiveFootPlacementSettings(const std::string& taskFile,
                                                                                            bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "perceptiveFootPlacementSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  
  if (verbose) {
    std::cerr << "\n #### Perceptive Foot Placement constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  try {
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  } catch (const std::exception& e) {
    if (verbose) {
      std::cerr << " #### Using default values for perceptive foot placement (config not found)\n";
    }
  }
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadArmKinematicsBoxSettings(const std::string& taskFile,
                                                                                   bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "armKinematicsBoxSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Arm Kinematics Box constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

std::pair<scalar_t, RelaxedBarrierPenalty::Config> MultiRobotWCargoInterface::loadManipulationFrictionConeSettings(
    const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "manipulationFrictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Manipulation Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

std::pair<vector3_t, vector3_t> MultiRobotWCargoInterface::loadManipulationTorqueBounds(const std::string& taskFile,
                                                                                      bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "manipulationTorqueBoxConstraint.";

  vector3_t lowerBound = vector3_t::Constant(-1e30);
  vector3_t upperBound = vector3_t::Constant(1e30);

  if (verbose) {
    std::cerr << "\n #### Manipulation Torque Box Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }

  loadData::loadEigenMatrix(taskFile, prefix + "lowerBound", lowerBound);
  loadData::loadEigenMatrix(taskFile, prefix + "upperBound", upperBound);

  if (verbose) {
    std::cerr << " #### Lower bound: " << lowerBound.transpose() << "\n";
    std::cerr << " #### Upper bound: " << upperBound.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::make_pair(lowerBound, upperBound);
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadManipulationTorqueBoxSettings(const std::string& taskFile,
                                                                                          bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "manipulationTorqueBoxConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Manipulation Torque Box Barrier Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

std::unique_ptr<StateInputCost> MultiRobotWCargoInterface::getArmKinematicsBoxConstraint(const std::vector<vector_t>& robotHandles,
                                                                                      const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  std::unique_ptr<MultiRobotArmKinematicsBoxConstraint> armKinematicsBoxConstraintPtr(
          new MultiRobotArmKinematicsBoxConstraint(*referenceManagerPtr_, robotHandles, numRobots_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(armKinematicsBoxConstraintPtr), std::move(penalty)));
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadOrientationConstraintSettings(const std::string& taskFile,
                                                                                   bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "orientationConstraintSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Orientation Constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

RelaxedBarrierPenalty::Config MultiRobotWCargoInterface::loadFormationConstraintSettings(
    const std::string& taskFile, bool verbose) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "formationConstraintSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  
  if (verbose) {
    std::cerr << "\n #### Formation Constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

}  // namespace multi_robot
}  // namespace ocs2