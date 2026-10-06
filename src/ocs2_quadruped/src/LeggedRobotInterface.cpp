//
// Created by zzhou387 on 9/9/22.
//
#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include "ocs2_quadruped/LeggedRobotInterface.h"
#include "ocs2_quadruped/LeggedRobotPreComputation.h"
#include "ocs2_quadruped/constraint/FrictionConeConstraint.h"
#include "ocs2_quadruped/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_quadruped/constraint/ZeroForceConstraint.h"
#include "ocs2_quadruped/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_quadruped/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped/constraint/CollisionAvoidanceConstraint.h"
#include "ocs2_quadruped/cost/LeggedRobotQuadraticTrackingCost.h"
#include "ocs2_quadruped/dynamics/LeggedRobotDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace quadruped {
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, 
                                           const std::string& referenceFile, const std::string& rebarFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[QuadrupedInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[QuadrupedInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[QuadrupedInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = sqp::loadSettings(taskFile, "multiple_shooting", verbose);

  // OptimalConrolProblem
  setupOptimalControlProblem(taskFile, urdfFile, referenceFile, rebarFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, const std::string& rebarFile,
                                                     bool verbose) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));
//  std::cerr << "[Debug] default joint position is: " <<

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
          *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
          centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
          modelSettings_.contactNames6DoF);

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
          new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose), 4));

  // Mode schedule manager
  referenceManagerPtr_ =
          std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(referenceFile, verbose), std::move(swingTrajectoryPlanner));

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimalConrolProblem] The analytical dynamics class is not yet implemented!");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_, false));

  // Constraint terms
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile, verbose);
  
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlacementSoftConstraint.";
  RelaxedBarrierPenalty::Config barrierPenaltyConfigFoot;
  loadData::loadPtreeValue(pt, barrierPenaltyConfigFoot.mu, prefix + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigFoot.delta, prefix + "delta", verbose);

  const std::string prefix_collision = "collisionAvoidanceSoftConstraint.";
  RelaxedBarrierPenalty::Config barrierPenaltyConfigCollision;
  scalar_t minimumDistance;
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCollision.mu, prefix_collision + "mu", verbose);
  loadData::loadPtreeValue(pt, barrierPenaltyConfigCollision.delta, prefix_collision + "delta", verbose);
  loadData::loadPtreeValue(pt, minimumDistance, prefix_collision + "minimumDistance", verbose);
  
  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  
  Json::Value rebar_file;
  std::ifstream rebar_fstream(rebarFile);
  rebar_fstream >> rebar_file;
  auto rebar_setup = rebar_file["rebar_setup"];
  if(rebar_setup.getMemberNames().empty()){
    throw std::runtime_error("[LeggedRobotInterface::setupOptimalConrolProblem] The rebar layout is not specified correctly");
  }
  std::vector<std::pair<vector3_t, vector3_t>> rebar_ends;
  std::cerr << "\n #### Rebar layout: ";
  std::cerr << "\n #### =============================================================================\n";
  for (auto r : rebar_setup["spacing"]) {
    vector3_t end1, end2;
    end1 << r[0][0].asDouble(), r[0][1].asDouble(), 0.05; // overwrite the z
    end2 << r[1][0].asDouble(), r[1][1].asDouble(), 0.05;
    std::cerr << end1.transpose() << "  " << end2.transpose() << "\n"; 
    rebar_ends.push_back(std::pair<vector3_t, vector3_t>(end1, end2));
  }
  std::cerr << " #### =============================================================================\n";

  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
    if (useAnalyticalGradientsConstraints) {
      throw std::runtime_error(
              "[LeggedRobotInterface::setupOptimalConrolProblem] The analytical end-effector linear constraint is not implemented!");
    } else {
      const auto infoCppAd = centroidalModelInfo_.toCppAd();
      const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
      auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
        const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
        updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
      };
      eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                    centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                    velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                    modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    }

    problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                        getFrictionConeConstraint(i, frictionCoefficient, barrierPenaltyConfig));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                            getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity",
                                            getNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
    
    // problemPtr_->softConstraintPtr->add(footName + "_collisionAvoidance",
    //                                     getCollisionAvoidanceConstraint(*eeKinematicsPtr, i, minimumDistance, barrierPenaltyConfigCollision,
    //                                     rebar_ends));
   
    const bool useHardConstraint = false;
    // Hard constraint
    // if (useHardConstraint) {
    //   problemPtr_->equalityConstraintPtr->add(
    //       footName + "_footPlacement",
    //       getFootPlacementConstraint(*eeKinematicsPtr, i, rebar_ends));
    // }
    // // Soft constraint
    // else {
    // problemPtr_->softConstraintPtr->add(
    //     footName + "_footPlacement",
    //     getFootPlacementConstraint(*eeKinematicsPtr, i,
    //                                 barrierPenaltyConfigFoot, rebar_ends));
    // }
  }

  // Pre-computation
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtrAll;
  const auto infoCppAd = centroidalModelInfo_.toCppAd();
  const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
  auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
    const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
    updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
  };
  eeKinematicsPtrAll.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, modelSettings_.contactNames3DoF,
                                                                centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                velocityUpdateCallback, "all_feet", modelSettings_.modelFolderCppAd,
                                                                modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
  

  
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*referenceManagerPtr_, *eeKinematicsPtrAll,
                                                                     centroidalModelInfo_, modelSettings_, rebar_ends));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& file, bool verbose) const {
  const auto initModeSchedule = loadModeSchedule(file, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(file, "defaultModeSequenceTemplate", false);

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

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LeggedRobotInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
  const size_t totalContactDim = 3 * info.numThreeDofContacts;

  vector_t initialState(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState);

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t baseToFeetJacobians(totalContactDim, info.actuatedDofNum);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, info.actuatedDofNum) =
            jacobianWorldToContactPointInWorldFrame.block(0, 6, 3, info.actuatedDofNum);
  }

  matrix_t R_taskspace(totalContactDim + totalContactDim, totalContactDim + totalContactDim);
  loadData::loadEigenMatrix(taskFile, "R", R_taskspace);

  matrix_t R = matrix_t::Zero(info.inputDim, info.inputDim);
  // Contact Forces
  R.topLeftCorner(totalContactDim, totalContactDim) = R_taskspace.topLeftCorner(totalContactDim, totalContactDim);
  // Joint velocities
  R.bottomRightCorner(info.actuatedDofNum, info.actuatedDofNum) =
          baseToFeetJacobians.transpose() * R_taskspace.bottomRightCorner(totalContactDim, totalContactDim) * baseToFeetJacobians;
  return R;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
                                                                          bool verbose) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R = initializeInputCostWeight(taskFile, info);

  if (verbose) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::loadFrictionConeSettings(const std::string& taskFile,
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraint> frictionConeConstraintPtr(
          new FrictionConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      bool useAnalyticalGradients) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };

  if (useAnalyticalGradients) {
    throw std::runtime_error(
            "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                                 eeZeroVelConConfig(modelSettings_.positionErrorGain)));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                        size_t contactPointIndex,
                                                                                        bool useAnalyticalGradients) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
            "[LeggedRobotInterface::getNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
  }
}

std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                       size_t contactPointIndex, 
                                                                                       std::vector<std::pair<vector3_t, vector3_t>> rebarFile) {
  return std::unique_ptr<StateInputConstraint>(new FootPlacementConstraint(
      *referenceManagerPtr_, eeKinematics, contactPointIndex, rebarFile));
}

std::unique_ptr<StateInputCost> LeggedRobotInterface::getFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                 size_t contactPointIndex,
                                                                                 const RelaxedBarrierPenalty::Config& barrierPenaltyConfig,
                                                                                 std::vector<std::pair<vector3_t, vector3_t>> rebarFile) {
  std::unique_ptr<FootPlacementConstraint> footPlacementConstraintPtr(
new FootPlacementConstraint(*referenceManagerPtr_, eeKinematics,
                                  contactPointIndex, rebarFile));
  std::unique_ptr<PenaltyBase> penalty(
      new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
      std::move(footPlacementConstraintPtr), std::move(penalty)));
};

std::unique_ptr<StateInputCost> LeggedRobotInterface::getCollisionAvoidanceConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      scalar_t minimumDistance,
                                                                                      const RelaxedBarrierPenalty::Config& barrierPenaltyConfig,
                                                                                      std::vector<std::pair<vector3_t, vector3_t>> rebarFile) {
  std::unique_ptr<CollisionAvoidanceConstraint> collisionAvoidanceConstraintPtr(
new CollisionAvoidanceConstraint(*referenceManagerPtr_, eeKinematics,
                                  contactPointIndex, minimumDistance, rebarFile));
  std::unique_ptr<PenaltyBase> penalty(
      new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(
      std::move(collisionAvoidanceConstraintPtr), std::move(penalty)));
};

} // namespace quadruped
} // namespace ocs2
