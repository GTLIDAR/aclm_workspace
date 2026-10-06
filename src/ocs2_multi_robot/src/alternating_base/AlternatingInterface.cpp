#include "ocs2_multi_robot/alternating_base/AlternatingInterface.h"

#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include "ocs2_multi_robot/utils/OrientationTools.h"

#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

#include "ocs2_multi_robot/cost/distributed/CargoQuadraticTrackingCost.h"
#include "ocs2_multi_robot/cost/distributed/SingleRobotQuadraticTrackingCost.h"
#include "ocs2_multi_robot/dynamics/distributed/CargowithArmForceAD.h"
#include "ocs2_multi_robot/dynamics/distributed/SRBDwithArmForceAD.h"
#include "ocs2_multi_robot/constraint/FrictionConeConstraint.h"
#include "ocs2_multi_robot/constraint/FootPlacementConstraint.h"
#include "ocs2_multi_robot/constraint/KinematicsBoxConstraint.h"
#include "ocs2_multi_robot/constraint/NormalVelocityConstraint.h"
#include "ocs2_multi_robot/constraint/ZeroForceConstraint.h"
#include "ocs2_multi_robot/constraint/ZeroVelocityConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotArmKinematicsBoxConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoManipulationFrictionConeConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotManipulationFrictionConeConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoManipulationTorqueBoxConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotManipulationTorqueBoxConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotCBFConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoCBFConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotCargoFormationConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoRobotFormationConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotOrientationConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoOrientationConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoArmKinematicsBoxConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/CargoBoundConstraint.h"
#include "ocs2_multi_robot/constraint/distributed/PerceptiveCargoBoundConstraint.h"
#include "ocs2_multi_robot/constraint/Obstacles.h"
#include "ocs2_multi_robot/initialization/CargoInitializer.h"
#include "ocs2_multi_robot/initialization/SingleRobotInitializer.h"
#include "ocs2_multi_robot/terrain/HeightMapExamples.h"
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include "ocs2_multi_robot/reference_manager/PerceptiveMultiRobotReferenceManager.h"
#include "ocs2_multi_robot/precomputation/AlternatingPerceptivePreComputation.h"
#include "ocs2_multi_robot/constraint/distributed/SingleRobotPerceptiveFootPlacementConstraint.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include <ros/ros.h>

#include <ocs2_quadruped/common/utils.h>
#include <ocs2_quadruped/foot_planner/SwingTrajectoryPlanner.h>

namespace ocs2 {
namespace multi_robot {

namespace {

template <typename MatrixType>
void loadMatrixOrIdentity(const std::string& taskFile, const std::string& fieldName, MatrixType& matrix, bool verbose) {
  try {
    loadData::loadEigenMatrix(taskFile, fieldName, matrix);
  } catch (const std::exception& e) {
    if (verbose) {
      std::cerr << "[AlternatingInterface] Could not load \"" << fieldName << "\". Using identity. Reason: " << e.what() << '\n';
    }
    matrix.setIdentity();
  }
}

std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadManipulationFrictionConeSettings(const std::string& taskFile, bool verbose) {
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

std::pair<vector3_t, vector3_t> loadManipulationTorqueBounds(const std::string& taskFile, bool verbose) {
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

RelaxedBarrierPenalty::Config loadManipulationTorqueBoxSettings(const std::string& taskFile, bool verbose) {
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

std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile, bool verbose) {
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

RelaxedBarrierPenalty::Config loadKinematicsBoxSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "kinematicsBoxSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Kinematics Box Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadFootPlacementSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacementSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Foot Placement Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadArmKinematicsBoxSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "armKinematicsBoxSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Arm Kinematics Box Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadFormationConstraintSettings(const std::string& taskFile, bool verbose) {
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

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadOrientationConstraintSettings(const std::string& taskFile, bool verbose) {
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

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadCBFConstraintSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "penalty_config.cbf.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### CBF Constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadCBF2DConstraintSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "2d_penalty_config.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### 2D CBF Constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadBoundConstraintSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "penalty_config.bounds.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Bound Constraint Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  return barrierPenaltyConfig;
}

RelaxedBarrierPenalty::Config loadPerceptiveFootPlacementSettings(const std::string& taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "perceptiveFootPlacementSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose) {
    std::cerr << "\n #### Perceptive Foot Placement Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  try {
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose);
  } catch (...) {
    if (verbose) {
      std::cerr << " #### Using default values for perceptive foot placement (config not found)\n";
    }
  }
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
  return barrierPenaltyConfig;
}

}  // namespace

AlternatingInterface::AlternatingInterface(const std::string& taskFile, const std::string& libraryFolder, bool verbose)
    : taskFile_(taskFile), libraryFolder_(libraryFolder), verbose_(verbose) {
  boost::filesystem::path taskFilePath(taskFile_);
  if (!boost::filesystem::exists(taskFilePath)) {
    throw std::invalid_argument("[AlternatingInterface] Task file not found: " + taskFilePath.string());
  }
  boost::filesystem::create_directories(libraryFolder_);

  mpcSettings_ = mpc::loadSettings(taskFile_, "mpc", verbose_);
  sqpSettings_ = sqp::loadSettings(taskFile_, "sqp", verbose_);
  alternatingSettings_ = loadAlternatingSettings(taskFile_, "alternating", verbose_);
  if (alternatingSettings_.numConsensusConstraints <= 0) {
    throw std::invalid_argument("[AlternatingInterface] alternating.numConsensusConstraints must be positive");
  }
  numRobots_ = static_cast<size_t>(alternatingSettings_.numConsensusConstraints);
  numArms_ = numRobots_;
  robotPreComputationPtrs_.resize(numRobots_);

  loadHandlePositions();
  loadModelParameters();
  loadInitialConditions();
  
  // Check if perceptive mode is enabled
  usePerceptive_ = false;
  try {
    loadData::loadCppDataType(taskFile_, "terrain_settings.use_perceptive", usePerceptive_);
  } catch (...) {
    if (verbose_) {
      std::cerr << "[AlternatingInterface] use_perceptive not found in config, defaulting to false\n";
    }
  }
  ros::NodeHandle nh;
  nh.getParam("use_perceptive", usePerceptive_);

  // Initialize perceptive components if enabled
  if (usePerceptive_) {
    ROS_INFO("[AlternatingInterface] Perceptive terrain mode ENABLED");
    double initial_foot_z = robotInitialStates_[0](14);  // FL_z from first robot
    gridMapHeightMapPtr_ = std::make_shared<ocs2::multi_robot::MultiRobotGridMapHeightMap>(initial_foot_z);
    planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
    
    double minPlanarHeight = -std::numeric_limits<double>::infinity();
    double maxPlanarHeight = std::numeric_limits<double>::infinity();
    try {
      loadData::loadCppDataType(taskFile_, "perceptive.min_planar_height", minPlanarHeight);
      loadData::loadCppDataType(taskFile_, "perceptive.max_planar_height", maxPlanarHeight);
    } catch (...) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default planar height bounds\n";
      }
    }
    
    // Load COM height for nominal foothold computation
    double comHeight = 0.5;
    try {
      loadData::loadCppDataType(taskFile_, "com_height.height", comHeight);
    } catch (...) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default COM height: " << comHeight << '\n';
      }
    }
    
    convexRegionSelectorPtr_ = std::make_shared<ocs2::multi_robot::MultiRobotConvexRegionSelector>(
        numRobots_, planarTerrainPtr_, gridMapHeightMapPtr_, 16, comHeight, minPlanarHeight, maxPlanarHeight);
    
    // Initialize visualization in ConvexRegionSelector
    convexRegionSelectorPtr_->initializeVisualization(nh, "odom");
    
    ROS_INFO("[AlternatingInterface] Created MultiRobotConvexRegionSelector");
  } else {
    ROS_INFO("[AlternatingInterface] Perceptive terrain mode DISABLED");
  }

  initializeReferenceManager();
  setupCargoOptimalControlProblem();
  setupSingleRobotOptimalControlProblems();
  twoQuadWCargoInitializerPtr_ = std::make_unique<TwoQuadWCargoInitializer>(robotMass_, cargoMass_, *referenceManagerPtr_);
}

void AlternatingInterface::initializeReferenceManager() {
  // Swing trajectory planners (one per robot for perceptive mode)
  std::vector<std::shared_ptr<quadruped::SwingTrajectoryPlanner>> swingTrajectoryPlanners;
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    swingTrajectoryPlanners.push_back(std::make_shared<quadruped::SwingTrajectoryPlanner>(
        quadruped::loadSwingTrajectorySettings(taskFile_, "swing_trajectory_config", verbose_), 4));
  }

  // Terrain module
  std::string terrain_name;
  loadData::loadCppDataType(taskFile_, "terrain_settings.terrain_type", terrain_name);
  auto heightMap = HeightMap::MakeTerrain(terrain_mapping.at(terrain_name));

  std::shared_ptr<HeightMap> heightMapForRefMgr;
  if (usePerceptive_ && gridMapHeightMapPtr_) {
    heightMapForRefMgr = std::static_pointer_cast<HeightMap>(gridMapHeightMapPtr_);
  } else {
    heightMapForRefMgr = heightMap;
  }

  if (usePerceptive_ && convexRegionSelectorPtr_) {
    scalar_t comHeight = 0.5;
    try {
      loadData::loadCppDataType(taskFile_, "com_height", comHeight);
    } catch (...) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default CoM height: " << comHeight << '\n';
      }
    }

    scalar_t cargoHeightOffset = 0.8;
    try {
      loadData::loadCppDataType(taskFile_, "cargo_height_offset.height", cargoHeightOffset);
    } catch (...) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default cargo height offset: " << cargoHeightOffset << '\n';
      }
    }
    ROS_INFO("[AlternatingInterface] cargo_height_offset = %.4f", cargoHeightOffset);
    
    referenceManagerPtr_ = std::make_shared<PerceptiveMultiRobotReferenceManager>(
        loadGaitSchedule(taskFile_, verbose_),
        swingTrajectoryPlanners,
        heightMapForRefMgr,
        convexRegionSelectorPtr_,
        numRobots_,
        comHeight,
        cargoHeightOffset);
    
    // Pass robot base offsets for terrain-averaged cargo z computation
    auto perceptiveRefMgr = std::dynamic_pointer_cast<PerceptiveMultiRobotReferenceManager>(referenceManagerPtr_);
    if (perceptiveRefMgr) {
      std::vector<vector_t> robotOffsetsForRefMgr(numRobots_);
      for (size_t robot = 0; robot < numRobots_; ++robot) {
        robotOffsetsForRefMgr[robot] = vector_t(6);
        loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffsetsForRefMgr[robot]);
      }
      perceptiveRefMgr->setRobotBaseOffsets(robotOffsetsForRefMgr);
    }
    
    ROS_INFO("[AlternatingInterface] Using PerceptiveMultiRobotReferenceManager");
  } else {
    referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManagerWithTerrain>(
        loadGaitSchedule(taskFile_, verbose_),
        swingTrajectoryPlanners.empty() ? nullptr : swingTrajectoryPlanners[0],
      heightMapForRefMgr,
      numRobots_);
    ROS_INFO("[AlternatingInterface] Using SwitchedModelReferenceManagerWithTerrain");
  }
}

void AlternatingInterface::loadHandlePositions() {
  handlePositions_.resize(numArms_);
  for (size_t i = 0; i < numArms_; ++i) {
    vector_t handle(6);
    const std::string fieldName = "cargo_model.handle_" + std::to_string(i + 1);
    loadData::loadEigenMatrix(taskFile_, fieldName, handle);
    handlePositions_[i] = handle;
  }
}

void AlternatingInterface::loadModelParameters() {
  loadData::loadCppDataType(taskFile_, "cargo_model.mass", cargoMass_);
  cargoInertia_.resize(3, 3);
  loadData::loadEigenMatrix(taskFile_, "cargo_model.inertia", cargoInertia_);

  loadData::loadCppDataType(taskFile_, "robot_model.mass", robotMass_);
  robotInertia_.resize(3, 3);
  loadData::loadEigenMatrix(taskFile_, "robot_model.inertia", robotInertia_);

  loadData::loadCppDataType(taskFile_, "centroidal_interface.recompileLibraries", recompileLibraries_);
}

void AlternatingInterface::loadInitialConditions() {
  cargoInitialState_.setZero(CARGO_STATE_DIM);
  loadData::loadEigenMatrix(taskFile_, "initialCargoState", cargoInitialState_);

  robotInitialStates_.clear();
  robotInitialStates_.resize(numRobots_);
  vector_t nominalRobotState(SINGLE_ROBOT_STATE_DIM);
  loadData::loadEigenMatrix(taskFile_, "initialRobotState", nominalRobotState);

  const vector3_t cargoPosition = cargoInitialState_.segment(0, 3);
  const vector3_t cargoEulerZyx = cargoInitialState_.segment(6, 3);
  const Eigen::AngleAxis<scalar_t> yawCargo(cargoEulerZyx(0), vector3_t::UnitZ());
  const Eigen::AngleAxis<scalar_t> pitchCargo(cargoEulerZyx(1), vector3_t::UnitY());
  const Eigen::AngleAxis<scalar_t> rollCargo(cargoEulerZyx(2), vector3_t::UnitX());
  const Eigen::Quaternion<scalar_t> qCargoZyx = yawCargo * pitchCargo * rollCargo;
  const matrix3_t R_cargo = qCargoZyx.toRotationMatrix();

  // offsets relative to the cargo (position xyz, orientation rpy)
  vector_t robotOffset(6);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffset);

    vector_t robotState = nominalRobotState;
    const vector3_t positionOffset = robotOffset.segment(0, 3);
    const vector3_t eulerOffset = robotOffset.segment(3, 3);

    // Compose rotations in world frame: R_robot = R_cargo * R_offset
    const Eigen::AngleAxis<scalar_t> yawOffset(eulerOffset(0), vector3_t::UnitZ());
    const Eigen::AngleAxis<scalar_t> pitchOffset(eulerOffset(1), vector3_t::UnitY());
    const Eigen::AngleAxis<scalar_t> rollOffset(eulerOffset(2), vector3_t::UnitX());
    const Eigen::Quaternion<scalar_t> qOffsetZyx = yawOffset * pitchOffset * rollOffset;
    const matrix3_t R_robot = (qCargoZyx * qOffsetZyx).toRotationMatrix();
    // Use RotM2ZYXEuler to avoid numerical issues with eulerAngles()
    // This function uses explicit atan2 calls which are more numerically stable
    const vector3_t eulerRobot = RotM2ZYXEuler(R_robot);  // Returns [yaw, pitch, roll] in ZYX order

    // Position and orientation in world frame
    robotState.segment(0, 3) = cargoPosition + positionOffset;
    robotState.segment(6, 3) = eulerRobot;

    // Update foot positions using rigid body transform
    for (size_t ee = 0; ee < QUADRUPED_FOOT_NUM; ++ee) {
      const vector3_t p_nominal = getNominalEePosWrtCom(ee);
      const vector3_t p_world = robotState.segment(0, 3) + R_robot * p_nominal;
      robotState.segment(12 + ee * QUADRUPED_CONTACT_DIM, 3) = p_world;
    }

    robotInitialStates_[robot] = robotState;
  }
}

void AlternatingInterface::setupCargoOptimalControlProblem() {
  cargoOptimalControlProblem_ = std::make_shared<OptimalControlProblem>();
  auto& problem = *cargoOptimalControlProblem_;

  matrix_t Q = matrix_t::Identity(CARGO_STATE_DIM, CARGO_STATE_DIM);
  const size_t cargoInputDim = numArms_ * ARM_CONTACT_DIM;
  matrix_t R = matrix_t::Identity(cargoInputDim, cargoInputDim);
  matrix_t Qf = matrix_t::Identity(CARGO_STATE_DIM, CARGO_STATE_DIM);

  loadMatrixOrIdentity(taskFile_, "alternating.cargo_cost.Q", Q, verbose_);
  loadMatrixOrIdentity(taskFile_, "alternating.cargo_cost.R", R, verbose_);
  loadMatrixOrIdentity(taskFile_, "alternating.cargo_cost.Q_final", Qf, verbose_);

  if (verbose_) {
    std::cerr << "cargo Q:  \n" << Q << "\n";
    std::cerr << "cargo R:  \n" << R << "\n";
    std::cerr << "cargo Q_final:\n" << Qf << "\n";
  }

  vector_t rho = alternatingSettings_.rho;
  if (rho.size() == 0) {
    rho = vector_t::Zero(numArms_);
    try {
      loadData::loadEigenMatrix(taskFile_, "alternating.rho", rho);
    } catch (const std::exception& e) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default cargo AL weights. Reason: " << e.what() << '\n';
      }
    }
  }
  if (verbose_) {
    std::cerr << "[AlternatingInterface] Cargo AL weights size: " << rho.size()
              << ", expected arms: " << numArms_ << '\n';
  }

  // Load manipulation friction cone settings separately
  scalar_t cargoFrictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config cargoFrictionBarrier;
  std::tie(cargoFrictionCoefficient, cargoFrictionBarrier) = loadManipulationFrictionConeSettings(taskFile_, verbose_);

  // Load torque bounds and barrier config for manipulation torques
  vector3_t torqueLowerBound, torqueUpperBound;
  std::tie(torqueLowerBound, torqueUpperBound) = loadManipulationTorqueBounds(taskFile_, verbose_);
  RelaxedBarrierPenalty::Config torqueBoxBarrier = loadManipulationTorqueBoxSettings(taskFile_, verbose_);

  problem.costPtr->add("cargo_tracking",
                       std::make_unique<CargoQuadraticTrackingCost>(Q, R, rho, numArms_, cargoMass_, *referenceManagerPtr_));
  problem.finalCostPtr->add("cargo_final", std::make_unique<CargoQuadraticStateTrackingCost>(Qf, *referenceManagerPtr_));

  auto cargoPreComputation = std::make_unique<AlternatingPreComputation>(*referenceManagerPtr_, handlePositions_, AlternatingTargetTrajectories(), numRobots_);
  cargoPreComputation_ = cargoPreComputation.get();
  problem.preComputationPtr = std::move(cargoPreComputation);

  problem.dynamicsPtr.reset(
      new CargowithArmForceAD(cargoMass_, cargoInertia_, static_cast<int>(numArms_), handlePositions_, libraryFolder_, recompileLibraries_));

  for (size_t arm = 0; arm < numArms_; ++arm) {
    auto manipConstraint = std::make_unique<CargoManipulationFrictionConeConstraint>(
        FrictionConeConstraint::Config(cargoFrictionCoefficient), arm, handlePositions_[arm]);
    auto manipPenalty = std::make_unique<RelaxedBarrierPenalty>(cargoFrictionBarrier);
    problem.softConstraintPtr->add("cargo_manipFriction_" + std::to_string(arm),
                                   std::make_unique<StateInputSoftConstraint>(std::move(manipConstraint), std::move(manipPenalty)));
    
    // Add torque box constraint if ARM_CONTACT_DIM == 6
    if (ARM_CONTACT_DIM == 6) {
      auto torqueConstraint = std::make_unique<CargoManipulationTorqueBoxConstraint>(arm, torqueLowerBound, torqueUpperBound);
      auto torquePenalty = std::make_unique<RelaxedBarrierPenalty>(torqueBoxBarrier);
      problem.softConstraintPtr->add("cargo_manipTorque_" + std::to_string(arm),
                                     std::make_unique<StateInputSoftConstraint>(std::move(torqueConstraint), std::move(torquePenalty)));
    }
  }

  // Load formation constraint settings
  RelaxedBarrierPenalty::Config formationBarrier = loadFormationConstraintSettings(taskFile_, verbose_);
  vector3_t formationTolerance = vector3_t::Constant(0.3);  // default tolerance
  try {
    loadData::loadEigenMatrix(taskFile_, "formationConstraint.position_tolerance", formationTolerance);
  } catch (const std::exception& e) {
    if (verbose_) {
      std::cerr << "[AlternatingInterface] Using default formation tolerance. Reason: " << e.what() << '\n';
    }
  }

  // Load robot offsets for formation constraint
  std::vector<vector_t> robotOffsets(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotOffsets[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffsets[robot]);
  }

  // Cargo-robot formation constraint (cargo side)
  auto cargoFormationConstraint = std::make_unique<CargoRobotFormationConstraint>(
      *referenceManagerPtr_, robotOffsets, formationTolerance, numRobots_);
  auto cargoFormationPenalty = std::make_unique<RelaxedBarrierPenalty>(formationBarrier);
  problem.softConstraintPtr->add("cargo_robotFormation",
                                std::make_unique<StateInputSoftConstraint>(std::move(cargoFormationConstraint), std::move(cargoFormationPenalty)));

  // Cargo arm kinematics box constraint
  RelaxedBarrierPenalty::Config armBarrier = loadArmKinematicsBoxSettings(taskFile_, verbose_);
  auto cargoArmConstraint = std::make_unique<CargoArmKinematicsBoxConstraint>(
      *referenceManagerPtr_, handlePositions_, numRobots_);
  auto cargoArmPenalty = std::make_unique<RelaxedBarrierPenalty>(armBarrier);
  problem.softConstraintPtr->add("cargo_armKinematicsBox",
                                std::make_unique<StateInputSoftConstraint>(std::move(cargoArmConstraint), std::move(cargoArmPenalty)));

  // Cargo orientation constraint
  RelaxedBarrierPenalty::Config orientationBarrier = loadOrientationConstraintSettings(taskFile_, verbose_);
  auto cargoOrientationConstraint = std::make_unique<CargoOrientationConstraint>(taskFile_);
  auto cargoOrientationPenalty = std::make_unique<RelaxedBarrierPenalty>(orientationBarrier);
  problem.stateSoftConstraintPtr->add("cargo_orientation",
                                      std::make_unique<StateSoftConstraint>(std::move(cargoOrientationConstraint), std::move(cargoOrientationPenalty)));

  // Cargo height bound constraint - terrain-relative in perceptive mode, absolute otherwise
  RelaxedBarrierPenalty::Config boundBarrier = loadBoundConstraintSettings(taskFile_, verbose_);
  if (usePerceptive_ && gridMapHeightMapPtr_) {
    // Perceptive mode: use terrain-relative height bounds (PerceptiveCargoBoundConstraint)
    // Terrain height = average at nominal robot base positions (not at cargo position)
    std::vector<vector_t> boundsRobotOffsets(numRobots_);
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      boundsRobotOffsets[robot] = vector_t(6);
      loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), boundsRobotOffsets[robot]);
    }
    auto cargoBoundConstraint = std::make_unique<PerceptiveCargoBoundConstraint>(
        taskFile_, numRobots_, gridMapHeightMapPtr_, boundsRobotOffsets);
    auto cargoBoundPenalty = std::make_unique<RelaxedBarrierPenalty>(boundBarrier);
    problem.stateSoftConstraintPtr->add("cargo_bound",
                                        std::make_unique<StateSoftConstraint>(std::move(cargoBoundConstraint), std::move(cargoBoundPenalty)));
    if (verbose_) {
      ROS_INFO("[AlternatingInterface] PerceptiveCargoBoundConstraint ENABLED (terrain-relative height bounds, avg robot terrain)");
    }
  } else {
    // Non-perceptive mode: use absolute z bounds (CargoBoundConstraint)
    auto cargoBoundConstraint = std::make_unique<CargoBoundConstraint>(taskFile_);
    auto cargoBoundPenalty = std::make_unique<RelaxedBarrierPenalty>(boundBarrier);
    problem.stateSoftConstraintPtr->add("cargo_bound",
                                        std::make_unique<StateSoftConstraint>(std::move(cargoBoundConstraint), std::move(cargoBoundPenalty)));
    if (verbose_) {
      ROS_INFO("[AlternatingInterface] CargoBoundConstraint ENABLED (absolute height bounds)");
    }
  }

  // Obstacle avoidance constraints (only if enabled)
  bool enableObstacleAvoidance_ = false;
  ros::param::get("/enable_obstacle_avoidance", enableObstacleAvoidance_);
  ROS_INFO("[AlternatingInterface] Obstacle avoidance parameter: %s", enableObstacleAvoidance_ ? "ENABLED" : "DISABLED");

  if (enableObstacleAvoidance_) {
    ROS_INFO("[AlternatingInterface] Obstacle avoidance ENABLED");
    
    // CBF constraint settings (3D penalty for cargo obstacles)
    RelaxedBarrierPenalty::Config cbfConfig = loadCBFConstraintSettings(taskFile_, verbose_);

    // 2D Obstacles (box 1)
    vector_array_t obstacles_box_1_pose_array;
    matrix_t obstacles_box_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_1.pose", obstacles_box_1_pose_matrix);
    for (int i = 0; i < obstacles_box_1_pose_matrix.rows(); ++i) {
      obstacles_box_1_pose_array.push_back(obstacles_box_1_pose_matrix.row(i));
    }
    scalar_array_t obstacles_box_1_radius;
    loadData::loadStdVector(taskFile_, "obstacles_box_1.radius", obstacles_box_1_radius, verbose_);
    auto obstaclesBox1Ptr = std::make_shared<Obstacles>(obstacles_box_1_pose_array);

    // 2D Obstacles (box 2)
    vector_array_t obstacles_box_2_pose_array;
    matrix_t obstacles_box_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_2.pose", obstacles_box_2_pose_matrix);
    for (int i = 0; i < obstacles_box_2_pose_matrix.rows(); ++i) {
      obstacles_box_2_pose_array.push_back(obstacles_box_2_pose_matrix.row(i));
    }
    scalar_array_t obstacles_box_2_radius;
    loadData::loadStdVector(taskFile_, "obstacles_box_2.radius", obstacles_box_2_radius, verbose_);
    auto obstaclesBox2Ptr = std::make_shared<Obstacles>(obstacles_box_2_pose_array);

    // 2D Obstacles narrow path (wall 1)
    vector_array_t obstacles_wall_1_pose_array;
    matrix_t obstacles_wall_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_1.pose", obstacles_wall_1_pose_matrix);
    for (int i = 0; i < obstacles_wall_1_pose_matrix.rows(); ++i) {
      obstacles_wall_1_pose_array.push_back(obstacles_wall_1_pose_matrix.row(i));
    }
    scalar_array_t obstacles_wall_1_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_1.radius", obstacles_wall_1_radius, verbose_);
    auto obstaclesWall1Ptr = std::make_shared<Obstacles>(obstacles_wall_1_pose_array);

    // 2D Obstacles narrow path (wall 2)
    vector_array_t obstacles_wall_2_pose_array;
    matrix_t obstacles_wall_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_2.pose", obstacles_wall_2_pose_matrix);
    for (int i = 0; i < obstacles_wall_2_pose_matrix.rows(); ++i) {
      obstacles_wall_2_pose_array.push_back(obstacles_wall_2_pose_matrix.row(i));
    }
    scalar_array_t obstacles_wall_2_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_2.radius", obstacles_wall_2_radius, verbose_);
    auto obstaclesWall2Ptr = std::make_shared<Obstacles>(obstacles_wall_2_pose_array);

    // 3D Obstacles (wall 3)
    vector_array_t obstacles_wall_3_pose_array;
    matrix_t obstacles_wall_3_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_3.pose", obstacles_wall_3_pose_matrix);
    for (int i = 0; i < obstacles_wall_3_pose_matrix.rows(); ++i) {
      obstacles_wall_3_pose_array.push_back(obstacles_wall_3_pose_matrix.row(i));
    }
    scalar_array_t obstacles_wall_3_radius;
    loadData::loadStdVector(taskFile_, "obstacles_wall_3.radius", obstacles_wall_3_radius, verbose_);
    auto obstaclesWall3Ptr = std::make_shared<Obstacles>(obstacles_wall_3_pose_array);

    // CBF constraint for cargo (box 1, box 2, wall 1, wall 2, wall 3)
    problem.stateSoftConstraintPtr->add("Cargo_Box_1_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<CargoCBFConstraint>(obstaclesBox1Ptr, obstacles_box_1_radius),
                                            std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Box_2_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<CargoCBFConstraint>(obstaclesBox2Ptr, obstacles_box_2_radius),
                                            std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_1_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<CargoCBFConstraint>(obstaclesWall1Ptr, obstacles_wall_1_radius),
                                            std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_2_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<CargoCBFConstraint>(obstaclesWall2Ptr, obstacles_wall_2_radius),
                                            std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
    problem.stateSoftConstraintPtr->add("Cargo_Wall_3_cbf",
                                        std::unique_ptr<StateCost>(new StateSoftConstraint(
                                            std::make_unique<CargoCBFConstraint>(obstaclesWall3Ptr, obstacles_wall_3_radius),
                                            std::make_unique<RelaxedBarrierPenalty>(cbfConfig))));
  } else {
    ROS_INFO("[AlternatingInterface] Obstacle avoidance DISABLED");
  }

  const auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");
  cargoRolloutPtr_ = std::make_unique<TimeTriggeredRollout>(*problem.dynamicsPtr, rolloutSettings);
  cargoInitializerPtr_ = std::make_unique<CargoInitializer>(cargoMass_, numArms_);

  optimalControlProblems_.push_back(cargoOptimalControlProblem_);
}

void AlternatingInterface::setupSingleRobotOptimalControlProblems() {
  singleRobotOptimalControlProblems_.clear();
  singleRobotOptimalControlProblems_.reserve(numRobots_);
  singleRobotRolloutPtrs_.clear();
  singleRobotInitializerPtrs_.clear();
  robotPreComputationPtrs_.assign(numRobots_, nullptr);

  matrix_t Q = matrix_t::Identity(SINGLE_ROBOT_STATE_DIM, SINGLE_ROBOT_STATE_DIM);
  matrix_t R = matrix_t::Identity(ALTERNATING_SINGLE_ROBOT_INPUT_DIM, ALTERNATING_SINGLE_ROBOT_INPUT_DIM);
  matrix_t Qf = matrix_t::Identity(SINGLE_ROBOT_STATE_DIM, SINGLE_ROBOT_STATE_DIM);
  loadMatrixOrIdentity(taskFile_, "alternating.single_robot_cost.Q", Q, verbose_);
  loadMatrixOrIdentity(taskFile_, "alternating.single_robot_cost.R", R, verbose_);
  loadMatrixOrIdentity(taskFile_, "alternating.single_robot_cost.Q_final", Qf, verbose_);
  if (verbose_) {
    std::cerr << "single-robot Q:  \n" << Q << "\n";
    std::cerr << "single-robot R:  \n" << R << "\n";
    std::cerr << "single-robot Q_final:\n" << Qf << "\n";
  }

  vector_t rho_full = alternatingSettings_.rho;
  if (rho_full.size() == 0) {
    rho_full = vector_t::Zero(numArms_);
    try {
      loadData::loadEigenMatrix(taskFile_, "alternating.rho", rho_full);
    } catch (const std::exception& e) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default single-robot AL weights. Reason: " << e.what() << '\n';
      }
    }
  }

  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config frictionBarrier;
  std::tie(frictionCoefficient, frictionBarrier) = loadFrictionConeSettings(taskFile_, verbose_);

  // Load torque bounds and barrier config for manipulation torques
  vector3_t torqueLowerBound, torqueUpperBound;
  std::tie(torqueLowerBound, torqueUpperBound) = loadManipulationTorqueBounds(taskFile_, verbose_);
  RelaxedBarrierPenalty::Config torqueBoxBarrier = loadManipulationTorqueBoxSettings(taskFile_, verbose_);

  RelaxedBarrierPenalty::Config kinematicsBarrier = loadKinematicsBoxSettings(taskFile_, verbose_);
  RelaxedBarrierPenalty::Config footPlacementBarrier = loadFootPlacementSettings(taskFile_, verbose_);
  RelaxedBarrierPenalty::Config armBarrier = loadArmKinematicsBoxSettings(taskFile_, verbose_);
  const auto rolloutSettings = rollout::loadSettings(taskFile_, "rollout");

  // Load obstacle data once for all robots (only if obstacle avoidance is enabled)
  bool enableObstacleAvoidance_robot = false;
  ros::param::get("/enable_obstacle_avoidance", enableObstacleAvoidance_robot);
  
  std::shared_ptr<Obstacles> obstaclesBox1Ptr, obstaclesBox2Ptr, obstaclesWall1Ptr, obstaclesWall2Ptr;
  scalar_array_t obstacles_box_1_radius, obstacles_box_2_radius, obstacles_wall_1_radius, obstacles_wall_2_radius;
  RelaxedBarrierPenalty::Config cbf2dConfig;
  
  if (enableObstacleAvoidance_robot) {
    cbf2dConfig = loadCBF2DConstraintSettings(taskFile_, verbose_);
    
    // 2D Obstacles (box 1)
    vector_array_t obstacles_box_1_pose_array;
    matrix_t obstacles_box_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_1.pose", obstacles_box_1_pose_matrix);
    for (int i = 0; i < obstacles_box_1_pose_matrix.rows(); ++i) {
      obstacles_box_1_pose_array.push_back(obstacles_box_1_pose_matrix.row(i));
    }
    loadData::loadStdVector(taskFile_, "obstacles_box_1.radius", obstacles_box_1_radius, verbose_);
    obstaclesBox1Ptr = std::make_shared<Obstacles>(obstacles_box_1_pose_array);

    // 2D Obstacles (box 2)
    vector_array_t obstacles_box_2_pose_array;
    matrix_t obstacles_box_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_box_2.pose", obstacles_box_2_pose_matrix);
    for (int i = 0; i < obstacles_box_2_pose_matrix.rows(); ++i) {
      obstacles_box_2_pose_array.push_back(obstacles_box_2_pose_matrix.row(i));
    }
    loadData::loadStdVector(taskFile_, "obstacles_box_2.radius", obstacles_box_2_radius, verbose_);
    obstaclesBox2Ptr = std::make_shared<Obstacles>(obstacles_box_2_pose_array);

    // 2D Obstacles narrow path (wall 1)
    vector_array_t obstacles_wall_1_pose_array;
    matrix_t obstacles_wall_1_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_1.pose", obstacles_wall_1_pose_matrix);
    for (int i = 0; i < obstacles_wall_1_pose_matrix.rows(); ++i) {
      obstacles_wall_1_pose_array.push_back(obstacles_wall_1_pose_matrix.row(i));
    }
    loadData::loadStdVector(taskFile_, "obstacles_wall_1.radius", obstacles_wall_1_radius, verbose_);
    obstaclesWall1Ptr = std::make_shared<Obstacles>(obstacles_wall_1_pose_array);

    // 2D Obstacles narrow path (wall 2)
    vector_array_t obstacles_wall_2_pose_array;
    matrix_t obstacles_wall_2_pose_matrix(1, 3);
    loadData::loadEigenMatrix(taskFile_, "obstacles_wall_2.pose", obstacles_wall_2_pose_matrix);
    for (int i = 0; i < obstacles_wall_2_pose_matrix.rows(); ++i) {
      obstacles_wall_2_pose_array.push_back(obstacles_wall_2_pose_matrix.row(i));
    }
    loadData::loadStdVector(taskFile_, "obstacles_wall_2.radius", obstacles_wall_2_radius, verbose_);
    obstaclesWall2Ptr = std::make_shared<Obstacles>(obstacles_wall_2_pose_array);
  }

  for (size_t robot = 0; robot < numRobots_; ++robot) {
    auto problem = std::make_shared<OptimalControlProblem>();

    matrix_t Q_robot = Q;
    matrix_t R_robot = R;
    matrix_t Qf_robot = Qf;
    vector_t rho_robot(1);
    rho_robot(0) = rho_full(robot);

    problem->costPtr->add("robot_tracking_" + std::to_string(robot),
                          std::make_unique<SingleRobotQuadraticTrackingCost>(Q_robot, R_robot, rho_robot, robot, robotMass_,
                                                                               cargoMass_, numRobots_, *referenceManagerPtr_));
    problem->finalCostPtr->add("robot_final_" + std::to_string(robot),
                               std::make_unique<SingleRobotQuadraticStateTrackingCost>(Qf_robot, robot, *referenceManagerPtr_));

    std::unique_ptr<AlternatingPreComputation> robotPreComp;
    if (usePerceptive_ && convexRegionSelectorPtr_) {
      robotPreComp = std::make_unique<AlternatingPerceptivePreComputation>(
          *referenceManagerPtr_, handlePositions_, AlternatingTargetTrajectories(), numRobots_,
          convexRegionSelectorPtr_, 16);
    } else {
      robotPreComp = std::make_unique<AlternatingPreComputation>(
          *referenceManagerPtr_, handlePositions_, AlternatingTargetTrajectories(), numRobots_);
    }
    robotPreComputationPtrs_[robot] = robotPreComp.get();
    problem->preComputationPtr = std::move(robotPreComp);

    problem->dynamicsPtr.reset(new SRBDwithArmForceAD(static_cast<int>(robot), robotMass_, robotInertia_, QUADRUPED_FOOT_NUM,
                                                      1 /* arm per robot */, libraryFolder_, recompileLibraries_, *robotPreComputationPtrs_[robot]));

    // Foot names for a single robot
    static const std::vector<std::string> SINGLE_ROBOT_FOOT_NAMES = {"FL", "FR", "RL", "RR"};
    
    for (size_t foot = 0; foot < QUADRUPED_FOOT_NUM; ++foot) {
      const size_t globalFootIndex = robot * QUADRUPED_FOOT_NUM + foot;
      const size_t localFootIndex = foot;
      const std::string constraintPrefix = "robot" + std::to_string(robot) + "_" + SINGLE_ROBOT_FOOT_NAMES[foot];

      // Friction cone soft constraint
      FrictionConeConstraint::Config frictionConfig(frictionCoefficient);
      auto frictionConstraint =
          std::make_unique<FrictionConeConstraint>(*referenceManagerPtr_, std::move(frictionConfig), globalFootIndex, localFootIndex, 0);
      auto frictionPenalty = std::make_unique<RelaxedBarrierPenalty>(frictionBarrier);
      problem->softConstraintPtr->add(constraintPrefix + "_frictionCone",
                                      std::make_unique<StateInputSoftConstraint>(std::move(frictionConstraint), std::move(frictionPenalty)));

      // Zero force / velocity constraints
      problem->equalityConstraintPtr->add(
          constraintPrefix + "_zeroForce",
          std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, globalFootIndex, localFootIndex)));
      problem->equalityConstraintPtr->add(
          constraintPrefix + "_zeroVelocity",
          std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraint(*referenceManagerPtr_, globalFootIndex, localFootIndex)));

      // Normal velocity constraint (equality) to shape swing trajectories in z
      // This ensures foot z-velocity follows the swing trajectory planner during swing phase
      std::shared_ptr<quadruped::SwingTrajectoryPlanner> robotSwingPlanner = nullptr;
      
      if (usePerceptive_) {
        // In perceptive mode, use robot-specific swing planner for terrain-aware trajectories
        auto* perceptiveRefMgr = dynamic_cast<PerceptiveMultiRobotReferenceManager*>(referenceManagerPtr_.get());
        if (perceptiveRefMgr != nullptr) {
          robotSwingPlanner = perceptiveRefMgr->getSwingTrajectoryPlannerForRobot(robot);
        }
      } else {
        // In non-perceptive mode, use the shared swing trajectory planner
        auto* switchedRefMgr = dynamic_cast<SwitchedModelReferenceManagerWithTerrain*>(referenceManagerPtr_.get());
        if (switchedRefMgr != nullptr) {
          robotSwingPlanner = switchedRefMgr->getSwingTrajectoryPlanner();
        }
      }
      
      if (robotSwingPlanner) {
        scalar_t positionErrorGain = 5.0;  // Default position feedback gain
        try {
          loadData::loadCppDataType(taskFile_, "swing_trajectory_config.position_error_gain", positionErrorGain);
        } catch (...) {
          if (verbose_) {
            std::cerr << "[AlternatingInterface] Using default position error gain: " << positionErrorGain << '\n';
          }
        }
        
        // In distributed mode, robotStateOffset = 0 (each robot has its own state)
        const size_t robotStateOffset = 0;
        problem->equalityConstraintPtr->add(
            constraintPrefix + "_normalVelocity",
            std::unique_ptr<StateInputConstraint>(
                new NormalVelocityConstraint(*referenceManagerPtr_, robotSwingPlanner, globalFootIndex, localFootIndex,
                                             robotStateOffset, positionErrorGain)));
        if (verbose_) {
          ROS_INFO("[AlternatingInterface] Added NormalVelocityConstraint for robot %zu foot %zu (posGain=%.2f, perceptive=%s)",
                  robot, localFootIndex, positionErrorGain, usePerceptive_ ? "yes" : "no");
        }
      } else {
        ROS_WARN_THROTTLE(2.0, "[AlternatingInterface] NormalVelocityConstraint requested but swing planner not available for robot %zu", robot);
      }

      // Foot placement soft constraint
      auto footPlacementConstraint =
          std::make_unique<FootPlacementConstraint>(*referenceManagerPtr_, globalFootIndex, localFootIndex, 0);
      auto footPlacementPenalty = std::make_unique<RelaxedBarrierPenalty>(footPlacementBarrier);
      problem->softConstraintPtr->add(constraintPrefix + "_footPlacement",
                                      std::make_unique<StateInputSoftConstraint>(std::move(footPlacementConstraint),
                                                                                 std::move(footPlacementPenalty)));

      // Kinematics box soft constraint
      auto kinematicsConstraint =
          std::make_unique<KinematicsBoxConstraint>(*referenceManagerPtr_, globalFootIndex, localFootIndex, 0);
      auto kinematicsPenalty = std::make_unique<RelaxedBarrierPenalty>(kinematicsBarrier);
      problem->softConstraintPtr->add(constraintPrefix + "_kinematicsBox",
                                      std::make_unique<StateInputSoftConstraint>(std::move(kinematicsConstraint),
                                                                                 std::move(kinematicsPenalty)));
    }

    // Perceptive foot placement constraints (if enabled)
    if (usePerceptive_ && convexRegionSelectorPtr_) {
      RelaxedBarrierPenalty::Config perceptiveFootBarrier = loadPerceptiveFootPlacementSettings(taskFile_, verbose_);
      
      for (size_t foot = 0; foot < QUADRUPED_FOOT_NUM; ++foot) {
        auto perceptiveFootConstraint = std::make_unique<SingleRobotPerceptiveFootPlacementConstraint>(
            *referenceManagerPtr_, *convexRegionSelectorPtr_, robot, foot, 16);
        auto perceptiveFootPenalty = std::make_unique<RelaxedBarrierPenalty>(perceptiveFootBarrier);
        problem->softConstraintPtr->add("robot" + std::to_string(robot) + "_foot" + std::to_string(foot) + "_perceptive",
                                        std::make_unique<StateInputSoftConstraint>(std::move(perceptiveFootConstraint), 
                                                                                    std::move(perceptiveFootPenalty)));
      }
      if (verbose_) {
        ROS_INFO("[AlternatingInterface] Added perceptive foot placement constraints for robot %zu", robot);
      }
    }

    // Arm kinematics box constraint (soft)
    auto armConstraint = std::make_unique<SingleRobotArmKinematicsBoxConstraint>(*referenceManagerPtr_, robot);
    auto armPenalty = std::make_unique<RelaxedBarrierPenalty>(armBarrier);
    problem->softConstraintPtr->add("robot" + std::to_string(robot) + "_armKinematicsBox",
                                    std::make_unique<StateInputSoftConstraint>(std::move(armConstraint), std::move(armPenalty)));

    // Manipulation friction cone constraint (soft)
    scalar_t robotManipFrictionCoefficient = 0.7;
    RelaxedBarrierPenalty::Config robotManipFrictionBarrier;
    std::tie(robotManipFrictionCoefficient, robotManipFrictionBarrier) = loadManipulationFrictionConeSettings(taskFile_, verbose_);
    auto manipConstraint = std::make_unique<SingleRobotManipulationFrictionConeConstraint>(
        FrictionConeConstraint::Config(robotManipFrictionCoefficient), robot, handlePositions_[robot]);
    auto manipPenalty = std::make_unique<RelaxedBarrierPenalty>(robotManipFrictionBarrier);
    problem->softConstraintPtr->add("robot" + std::to_string(robot) + "_manipFriction",
                                    std::make_unique<StateInputSoftConstraint>(std::move(manipConstraint), std::move(manipPenalty)));
    
    // Manipulation torque box constraint (soft) - only active when ARM_CONTACT_DIM == 6
    if (ARM_CONTACT_DIM == 6) {
      auto torqueConstraint = std::make_unique<SingleRobotManipulationTorqueBoxConstraint>(robot, torqueLowerBound, torqueUpperBound);
      auto torquePenalty = std::make_unique<RelaxedBarrierPenalty>(torqueBoxBarrier);
      problem->softConstraintPtr->add("robot" + std::to_string(robot) + "_manipTorque",
                                       std::make_unique<StateInputSoftConstraint>(std::move(torqueConstraint), std::move(torquePenalty)));
    }

    // Robot-cargo formation constraint (robot side)
    RelaxedBarrierPenalty::Config formationBarrier = loadFormationConstraintSettings(taskFile_, verbose_);
    vector3_t formationTolerance = vector3_t::Constant(0.3);  // default tolerance
    try {
      loadData::loadEigenMatrix(taskFile_, "formationConstraint.position_tolerance", formationTolerance);
    } catch (const std::exception& e) {
      if (verbose_) {
        std::cerr << "[AlternatingInterface] Using default formation tolerance. Reason: " << e.what() << '\n';
      }
    }
    vector_t robotOffset(6);
    loadData::loadEigenMatrix(taskFile_, "initialStateOffset.robot_" + std::to_string(robot + 1), robotOffset);
    auto robotFormationConstraint = std::make_unique<SingleRobotCargoFormationConstraint>(
        *referenceManagerPtr_, robotOffset, formationTolerance, robot);
    auto robotFormationPenalty = std::make_unique<RelaxedBarrierPenalty>(formationBarrier);
    problem->softConstraintPtr->add("robot" + std::to_string(robot) + "_cargoFormation",
                                    std::make_unique<StateInputSoftConstraint>(std::move(robotFormationConstraint), std::move(robotFormationPenalty)));

    // Robot orientation constraint
    RelaxedBarrierPenalty::Config orientationBarrier = loadOrientationConstraintSettings(taskFile_, verbose_);
    auto robotOrientationConstraint = std::make_unique<SingleRobotOrientationConstraint>(taskFile_);
    auto robotOrientationPenalty = std::make_unique<RelaxedBarrierPenalty>(orientationBarrier);
    problem->stateSoftConstraintPtr->add("robot" + std::to_string(robot) + "_orientation",
                                        std::make_unique<StateSoftConstraint>(std::move(robotOrientationConstraint), std::move(robotOrientationPenalty)));

    // Robot CBF constraints for obstacles (only if enabled)
    if (enableObstacleAvoidance_robot) {
      // CBF constraints for robot (box 1, box 2, wall 1, wall 2)
      problem->stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Box_1_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<SingleRobotCBFConstraint>(obstaclesBox1Ptr, obstacles_box_1_radius),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem->stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Box_2_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<SingleRobotCBFConstraint>(obstaclesBox2Ptr, obstacles_box_2_radius),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem->stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Wall_1_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<SingleRobotCBFConstraint>(obstaclesWall1Ptr, obstacles_wall_1_radius),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
      problem->stateSoftConstraintPtr->add("Robot" + std::to_string(robot + 1) + "_Wall_2_cbf",
                                          std::unique_ptr<StateCost>(new StateSoftConstraint(
                                              std::make_unique<SingleRobotCBFConstraint>(obstaclesWall2Ptr, obstacles_wall_2_radius),
                                              std::make_unique<RelaxedBarrierPenalty>(cbf2dConfig))));
    }

    singleRobotRolloutPtrs_.push_back(std::make_unique<TimeTriggeredRollout>(*problem->dynamicsPtr, rolloutSettings));
    singleRobotInitializerPtrs_.push_back(
      std::make_unique<SingleRobotInitializer>(robotMass_, QUADRUPED_FOOT_NUM, cargoMass_, robot, numRobots_, *referenceManagerPtr_));

    singleRobotOptimalControlProblems_.push_back(problem);
    optimalControlProblems_.push_back(problem);
  }
}

void AlternatingInterface::updatePreComputations(const AlternatingTargetTrajectories& trajectories) {
  if (cargoPreComputation_ != nullptr) {
    cargoPreComputation_->updatePreviousSolution(trajectories);
  }
  for (auto* preComp : robotPreComputationPtrs_) {
    if (preComp != nullptr) {
      preComp->updatePreviousSolution(trajectories);
    }
  }
}

std::shared_ptr<quadruped::GaitSchedule> AlternatingInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
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

  if (verbose) {
    std::cerr << "\n#### Modes Schedule: ";
    std::cerr << "\n#### =============================================================================\n";
    std::cerr << "Initial Modes Schedule: \n" << initModeSchedule;
    std::cerr << "Default Modes Sequence Template: \n" << defaultModeSequenceTemplate;
    std::cerr << "#### =============================================================================\n";
  }

  return std::make_shared<quadruped::GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, phaseTransitionStanceTime);
}

}  // namespace multi_robot
}  // namespace ocs2


