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
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_quadruped/LocoManipulationInterface.h"
#include "ocs2_quadruped/constraint/FrictionConeConstraintInvariant.h"
#include "ocs2_quadruped/constraint/EndEffectorConstraint.h"
#include "ocs2_quadruped/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_quadruped/constraint/ZeroForceConstraint.h"
#include "ocs2_quadruped/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_quadruped/cost/LocoManipulationQuadraticTrackingCost.h"
#include "ocs2_quadruped/dynamics/LeggedRobotDynamicsAD.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace quadruped {
LocoManipulationInterface::LocoManipulationInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LocoManipulationInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManipulationInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LocoManipulationInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManipulationInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LocoManipulationInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LocoManipulationInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = sqp::loadSettings(taskFile, "multiple_shooting", verbose);
  loadData::loadCppDataType(taskFile, "model_settings.eeFrame", eeFrame_);

  // OptimalConrolProblem
  setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LocoManipulationInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
          *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
          centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
          modelSettings_.contactNames6DoF);

  // Reference manager
  referenceManagerPtr_.reset(new ReferenceManager);
  const auto initModeSchedule = loadModeSchedule(referenceFile, "initialModeSchedule", false);
  referenceManagerPtr_->setModeSchedule(initModeSchedule);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LocoManipulationInterface::setupOptimalConrolProblem] The analytical dynamics class is not yet implemented!");
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

  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
    if (useAnalyticalGradientsConstraints) {
      throw std::runtime_error(
              "[LocoManipulationInterface::setupOptimalConrolProblem] The analytical end-effector linear constraint is not implemented!");
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
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                            getZeroVelocityConstraint(*eeKinematicsPtr, useAnalyticalGradientsConstraints));
   
  }

  // manipulator end effector constraint
  problemPtr_->stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(eeFrame_, taskFile, "endEffector",
                                                                                   useAnalyticalGradientsConstraints));
  problemPtr_->finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(eeFrame_, taskFile, "finalEndEffector",
                                                                                        useAnalyticalGradientsConstraints));
  
  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LocoManipulationPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LocoManipulationInitializer(centroidalModelInfo_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
matrix_t LocoManipulationInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info) {
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
std::unique_ptr<StateInputCost> LocoManipulationInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info,
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

  return std::unique_ptr<StateInputCost>(new LocoManipulationStateInputQuadraticCost(std::move(Q), std::move(R), info));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LocoManipulationInterface::loadFrictionConeSettings(const std::string& taskFile,
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
std::unique_ptr<StateInputCost> LocoManipulationInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  FrictionConeConstraintInvariant::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraintInvariant> frictionConeConstraintPtr(
          new FrictionConeConstraintInvariant(std::move(frictionConeConConfig), contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionConeConstraintPtr), std::move(penalty)));
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LocoManipulationInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
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
            "[LocoManipulationInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new EndEffectorLinearConstraint(eeKinematics, 3, eeZeroVelConConfig(modelSettings_.positionErrorGain)));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LocoManipulationInterface::getEndEffectorConstraint(const std::string& ee_name,
                                                                               const std::string& taskFile,
                                                                               const std::string& prefix,
                                                                               bool useAnalyticalGradients) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cout << "EE name is: " << ee_name << std::endl;
  std::cerr << " #### =============================================================================\n";

  std::unique_ptr<StateConstraint> constraint;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }
  if (useAnalyticalGradients) {
    throw std::runtime_error(
            "[LocoManipulationInterface::setupOptimalConrolProblem] The analytical end-effector constraint is not implemented!");
  } else {
    const auto infoCppAd = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
    auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
      const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
      updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
    };
    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {ee_name},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, ee_name, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
  }
  constraint.reset(new EndEffectorConstraint(*eeKinematicsPtr, *referenceManagerPtr_));
  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}

} // namespace quadruped
} // namespace ocs2
