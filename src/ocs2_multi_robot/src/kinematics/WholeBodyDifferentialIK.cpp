#include "ocs2_multi_robot/kinematics/WholeBodyDifferentialIK.h"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <qpOASES.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace legged_kinematics {
namespace {
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Twist = Eigen::Matrix<double, 6, 1>;

bool validPose(const Eigen::Isometry3d& pose) {
  return pose.matrix().allFinite() &&
         (pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(pose.linear().determinant() - 1.0) < 1e-6 &&
         pose.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-6);
}

Eigen::Vector3d bounded(Eigen::Vector3d value, double limit) {
  const double norm = value.norm();
  if (norm > limit) value *= limit / norm;
  return value;
}

Twist poseError(const Eigen::Isometry3d& desired, const Eigen::Isometry3d& current,
                double gain, double linearLimit, double angularLimit) {
  Twist velocity;
  velocity.head<3>() = bounded(gain * (desired.translation() - current.translation()), linearLimit);
  const Eigen::AngleAxisd rotation(desired.linear() * current.linear().transpose());
  velocity.tail<3>() = bounded(gain * rotation.angle() * rotation.axis(), angularLimit);
  return velocity;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d matrix;
  matrix << 0, -value.z(), value.y(), value.z(), 0, -value.x(), -value.y(), value.x(), 0;
  return matrix;
}
}

struct WholeBodyDifferentialIK::Impl {
  struct Endpoint {
    KDL::Chain chain;
    std::vector<int> indices;
  };
  Settings settings;
  std::vector<std::string> names;
  Eigen::VectorXd lower, upper, speed;
  std::vector<Endpoint> endpoints;

  Eigen::Matrix<double, 12, 1> analyticalLegAngles(const std::array<Eigen::Vector3d, 4>& feet) const {
    const auto& geometry = *settings.analyticalLegs;
    Eigen::Matrix<double, 12, 1> angles;
    for (size_t foot = 0; foot < feet.size(); ++foot) {
      const double side = foot % 2 == 0 ? 1.0 : -1.0;
      Eigen::Vector3d position = feet[foot] - geometry.baseToHip[foot];
      position.y() *= side;
      const double lateral = geometry.hipToThigh.y();
      const double radialSquared = position.y() * position.y() + position.z() * position.z() - lateral * lateral;
      if (radialSquared < 0.0) throw std::runtime_error("Analytical hip target unreachable");
      const double hip = std::atan2(position.y(), -position.z()) - std::atan2(lateral, std::sqrt(radialSquared));
      position = Eigen::AngleAxisd(-hip, Eigen::Vector3d::UnitX()) * position;
      position -= geometry.hipToThigh;
      const double distanceSquared = position.x() * position.x() + position.z() * position.z();
      const double distance = std::sqrt(distanceSquared);
      const double thigh = geometry.thighLength;
      const double shank = geometry.shankLength;
      if (distance < 1e-12 || distance > thigh + shank + 1e-9 || distance < std::abs(thigh - shank) - 1e-9) {
        throw std::runtime_error("Analytical leg target unreachable");
      }
      const double beta = std::acos(std::clamp((thigh * thigh + distanceSquared - shank * shank) / (2.0 * thigh * distance), -1.0, 1.0));
      const double gamma = std::acos(std::clamp((thigh * thigh + shank * shank - distanceSquared) / (2.0 * thigh * shank), -1.0, 1.0));
      const double pi = std::acos(-1.0);
      angles.segment<3>(3 * foot) << side * hip, std::atan2(-position.z(), position.x()) - 0.5 * pi + beta, gamma - pi;
    }
    return angles;
  }

  Impl(const std::string& file, Settings options) : settings(std::move(options)) {
    for (double value : {settings.dt, settings.gain, settings.regularization, settings.maxLinearSpeed,
                         settings.maxAngularSpeed, settings.maxBaseLinearSpeed, settings.maxBaseAngularSpeed}) {
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("IK periods, damping, gains and speed limits must be positive");
    }
    for (double value : {settings.footWeight, settings.armPositionWeight, settings.armOrientationWeight, settings.baseWeight}) {
      if (!std::isfinite(value) || value < 0) throw std::invalid_argument("IK task weights must be finite and nonnegative");
    }
    if (settings.legMode == LegMode::Analytical &&
        (settings.baseMode != BaseMode::Fixed || settings.model == Model::Arm || !settings.analyticalLegs)) {
      throw std::invalid_argument("Analytical legs require a fixed B1/B1Z1 base and analytical geometry");
    }
    if (settings.analyticalLegs) {
      const auto& geometry = *settings.analyticalLegs;
      if (!geometry.hipToThigh.allFinite() || !std::isfinite(geometry.thighLength) ||
          !std::isfinite(geometry.shankLength) || geometry.thighLength <= 0.0 || geometry.shankLength <= 0.0) {
        throw std::invalid_argument("Analytical legs require finite geometry and positive segment lengths");
      }
      for (const auto& hip : geometry.baseToHip) {
        if (!hip.allFinite()) throw std::invalid_argument("Invalid analytical hip offset");
      }
    }
    urdf::Model model;
    KDL::Tree tree;
    if (!model.initFile(file) || !kdl_parser::treeFromUrdfModel(model, tree)) throw std::invalid_argument("Cannot parse IK URDF");
    std::vector<std::string> frames;
    if (settings.model != Model::Arm) {
      for (const std::string leg : {"FL", "FR", "RL", "RR"}) {
        for (const std::string joint : {"hip", "thigh", "calf"}) names.push_back(leg + "_" + joint + "_joint");
        frames.push_back(leg + "_foot");
      }
    }
    if (settings.model != Model::B1) {
      for (int joint = 1; joint <= 6; ++joint) names.push_back("joint" + std::to_string(joint));
      frames.push_back("virtual_ee_link");
    }
    lower.resize(names.size()); upper.resize(names.size()); speed.resize(names.size());
    for (size_t index = 0; index < names.size(); ++index) {
      const auto joint = model.getJoint(names[index]);
      if (!joint || joint->mimic || !joint->limits ||
          (joint->type != urdf::Joint::REVOLUTE && joint->type != urdf::Joint::CONTINUOUS) ||
          !std::isfinite(joint->limits->velocity) || joint->limits->velocity <= 0) {
        throw std::invalid_argument("Missing/unsupported IK joint or velocity limits: " + names[index]);
      }
      const bool continuous = joint->type == urdf::Joint::CONTINUOUS;
      lower[index] = continuous ? -std::numeric_limits<double>::infinity() : joint->limits->lower;
      upper[index] = continuous ? std::numeric_limits<double>::infinity() : joint->limits->upper;
      speed[index] = joint->limits->velocity;
      if (!continuous && (!std::isfinite(lower[index]) || !std::isfinite(upper[index]) || lower[index] > upper[index])) {
        throw std::invalid_argument("Invalid joint position limits: " + names[index]);
      }
    }
    const std::string root = settings.model == Model::Arm ? "link00" : "base";
    for (const auto& frame : frames) {
      Endpoint endpoint;
      if (!tree.getChain(root, frame, endpoint.chain)) throw std::invalid_argument("Missing IK chain: " + frame);
      for (const auto& segment : endpoint.chain.segments) {
        if (segment.getJoint().getType() == KDL::Joint::None) continue;
        const auto found = std::find(names.begin(), names.end(), segment.getJoint().getName());
        if (found == names.end()) throw std::invalid_argument("Unexpected joint in IK chain: " + segment.getJoint().getName());
        endpoint.indices.push_back(std::distance(names.begin(), found));
      }
      endpoints.push_back(std::move(endpoint));
    }
  }

  void validate(const State& state, bool checkLimits = true) const {
    if (state.joints.size() != lower.size() || !state.joints.allFinite() || !validPose(state.worldFromBase)) {
      throw std::invalid_argument("Kinematics requires finite joint positions of the correct size and a rigid base pose");
    }
    if (checkLimits && ((state.joints.array() < lower.array() - 1e-6).any() ||
                        (state.joints.array() > upper.array() + 1e-6).any())) {
      throw std::invalid_argument("IK requires an in-limit state");
    }
  }

  Eigen::Isometry3d kinematics(size_t index, const State& state, Matrix* jacobian = nullptr) const {
    const auto& endpoint = endpoints.at(index);
    KDL::JntArray joints(endpoint.indices.size());
    for (size_t joint = 0; joint < endpoint.indices.size(); ++joint) joints(joint) = state.joints[endpoint.indices[joint]];
    KDL::ChainFkSolverPos_recursive fk(endpoint.chain);
    KDL::Frame frame;
    if (fk.JntToCart(joints, frame) < 0) throw std::runtime_error("IK FK failed");
    Eigen::Isometry3d local = Eigen::Isometry3d::Identity();
    for (int row = 0; row < 3; ++row) {
      local.translation()[row] = frame.p[row];
      for (int col = 0; col < 3; ++col) local.linear()(row, col) = frame.M(row, col);
    }
    const Eigen::Isometry3d world = state.worldFromBase * local;
    if (jacobian) {
      const int offset = settings.baseMode == BaseMode::Floating ? 6 : 0;
      jacobian->setZero(6, offset + names.size());
      KDL::ChainJntToJacSolver solver(endpoint.chain);
      KDL::Jacobian localJacobian(endpoint.indices.size());
      if (solver.JntToJac(joints, localJacobian) < 0) throw std::runtime_error("IK Jacobian failed");
      for (size_t joint = 0; joint < endpoint.indices.size(); ++joint) {
        jacobian->block<3, 1>(0, offset + endpoint.indices[joint]) = state.worldFromBase.linear() * localJacobian.data.block<3, 1>(0, joint);
        jacobian->block<3, 1>(3, offset + endpoint.indices[joint]) = state.worldFromBase.linear() * localJacobian.data.block<3, 1>(3, joint);
      }
      if (offset) {
        jacobian->block<3, 3>(0, 0).setIdentity();
        jacobian->block<3, 3>(0, 3) = -skew(world.translation() - state.worldFromBase.translation());
        jacobian->block<3, 3>(3, 3).setIdentity();
      }
    }
    return world;
  }
};

WholeBodyDifferentialIK::WholeBodyDifferentialIK(const std::string& file, Settings settings)
    : impl_(new Impl(file, std::move(settings))) {}
WholeBodyDifferentialIK::~WholeBodyDifferentialIK() = default;
const std::vector<std::string>& WholeBodyDifferentialIK::jointNames() const { return impl_->names; }
const Eigen::VectorXd& WholeBodyDifferentialIK::lowerLimits() const { return impl_->lower; }
const Eigen::VectorXd& WholeBodyDifferentialIK::upperLimits() const { return impl_->upper; }
const Eigen::VectorXd& WholeBodyDifferentialIK::velocityLimits() const { return impl_->speed; }

WholeBodyDifferentialIK::State WholeBodyDifferentialIK::neutralState() const {
  State state;
  state.joints = Eigen::VectorXd::Zero(impl_->names.size()).cwiseMax(impl_->lower).cwiseMin(impl_->upper);
  return state;
}

WholeBodyDifferentialIK::Targets WholeBodyDifferentialIK::currentTargets(const State& state) const {
  impl_->validate(state, false);
  Targets targets;
  for (auto& foot : targets.feet) foot.setZero();
  if (impl_->settings.model != Model::Arm) {
    for (size_t foot = 0; foot < 4; ++foot) targets.feet[foot] = impl_->kinematics(foot, state).translation();
  }
  if (impl_->settings.model != Model::B1) targets.arm = impl_->kinematics(impl_->endpoints.size() - 1, state);
  targets.base = state.worldFromBase;
  return targets;
}

void WholeBodyDifferentialIK::setTimeStep(double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) throw std::invalid_argument("IK time step must be finite and positive");
  impl_->settings.dt = dt;
}

void WholeBodyDifferentialIK::setTaskWeights(double position, double orientation, double regularization) {
  if (!std::isfinite(position) || !std::isfinite(orientation) || !std::isfinite(regularization) ||
      position < 0 || orientation < 0 || regularization <= 0) throw std::invalid_argument("Invalid IK weights");
  impl_->settings.armPositionWeight = position;
  impl_->settings.armOrientationWeight = orientation;
  impl_->settings.regularization = regularization;
}

WholeBodyDifferentialIK::Result WholeBodyDifferentialIK::solve(const State& state, const Targets& targets) {
  Result result;
  result.state = state;
  result.jointVelocity = Eigen::VectorXd::Zero(impl_->names.size());
  try {
    impl_->validate(state);
    const auto& settings = impl_->settings;
    const bool hasLegs = settings.model != Model::Arm;
    const bool hasArm = settings.model != Model::B1;
    if ((hasArm && !validPose(targets.arm)) || (targets.trackBase && !validPose(targets.base))) throw std::invalid_argument("Invalid IK target pose");
    const int offset = settings.baseMode == BaseMode::Floating ? 6 : 0;
    if (targets.trackBase && !offset) throw std::invalid_argument("Base tracking requires floating-base IK");
    const int variables = offset + impl_->names.size();
    Matrix hessian = settings.regularization * Matrix::Identity(variables, variables);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(variables);
    Eigen::VectorXd lower(variables), upper(variables);
    lower.tail(impl_->names.size()) = (-impl_->speed).cwiseMax((impl_->lower - state.joints) / settings.dt);
    upper.tail(impl_->names.size()) = impl_->speed.cwiseMin((impl_->upper - state.joints) / settings.dt);
    if (offset) {
      lower.head<3>().setConstant(-settings.maxBaseLinearSpeed);
      lower.segment<3>(3).setConstant(-settings.maxBaseAngularSpeed);
      upper.head(6) = -lower.head(6);
    }
    Matrix constraints = Matrix::Zero(12, variables);
    Eigen::VectorXd constraintTarget = Eigen::VectorXd::Zero(12);
    int constraintCount = 0;
    auto addTask = [&](const Matrix& jacobian, const Eigen::VectorXd& velocity, double weight) {
      hessian.noalias() += weight * jacobian.transpose() * jacobian;
      gradient.noalias() -= weight * jacobian.transpose() * velocity;
    };
    if (hasLegs) {
      for (size_t foot = 0; foot < 4; ++foot) {
        if (!targets.feet[foot].allFinite()) throw std::invalid_argument("Nonfinite foot target");
        Matrix jacobian;
        const auto pose = impl_->kinematics(foot, state, &jacobian);
        const Eigen::Vector3d velocity = bounded(settings.gain * (targets.feet[foot] - pose.translation()), settings.maxLinearSpeed);
        if (targets.stance[foot]) {
          constraints.middleRows(constraintCount, 3) = jacobian.topRows(3);
          constraintTarget.segment<3>(constraintCount) = velocity;
          constraintCount += 3;
        } else {
          addTask(jacobian.topRows(3), velocity, settings.footWeight);
        }
      }
      if (settings.legMode == LegMode::Analytical) {
        std::array<Eigen::Vector3d, 4> localFeet;
        for (size_t foot = 0; foot < 4; ++foot) localFeet[foot] = state.worldFromBase.inverse() * targets.feet[foot];
        State analytical = state;
        analytical.joints.head(12) = impl_->analyticalLegAngles(localFeet);
        impl_->validate(analytical);
        for (size_t foot = 0; foot < 4; ++foot) {
          if ((impl_->kinematics(foot, analytical).translation() - targets.feet[foot]).norm() > 1e-4) {
            throw std::runtime_error("Analytical leg target unreachable or geometry differs from URDF");
          }
        }
        const Eigen::VectorXd referenceVelocity = ((analytical.joints - state.joints) / settings.dt).cwiseMax(lower).cwiseMin(upper);
        lower.head(12) = referenceVelocity.head(12);
        upper.head(12) = referenceVelocity.head(12);
      }
    }
    if (hasArm) {
      Matrix jacobian;
      const auto pose = impl_->kinematics(impl_->endpoints.size() - 1, state, &jacobian);
      const Twist velocity = poseError(targets.arm, pose, settings.gain, settings.maxLinearSpeed, settings.maxAngularSpeed);
      addTask(jacobian.topRows(3), velocity.head<3>(), settings.armPositionWeight);
      addTask(jacobian.bottomRows(3), velocity.tail<3>(), settings.armOrientationWeight);
    }
    if (offset && targets.trackBase) {
      Matrix jacobian = Matrix::Zero(6, variables);
      jacobian.leftCols(6).setIdentity();
      addTask(jacobian, poseError(targets.base, state.worldFromBase, settings.gain, settings.maxBaseLinearSpeed,
                                  settings.maxBaseAngularSpeed), settings.baseWeight);
    }
    if (!hessian.allFinite() || !gradient.allFinite() || !constraints.allFinite() ||
        (lower.array() > upper.array()).any()) throw std::runtime_error("Invalid whole-body QP");
    constraints.conservativeResize(constraintCount, variables);
    constraintTarget.conservativeResize(constraintCount);
    qpOASES::QProblem problem(variables, constraintCount);
    qpOASES::Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);
    qpOASES::int_t iterations = 200;
    Eigen::VectorXd velocity(variables);
    const auto status = problem.init(hessian.data(), gradient.data(), constraintCount ? constraints.data() : nullptr,
                                    lower.data(), upper.data(), constraintCount ? constraintTarget.data() : nullptr,
                                    constraintCount ? constraintTarget.data() : nullptr, iterations);
    if (status != qpOASES::SUCCESSFUL_RETURN || problem.getPrimalSolution(velocity.data()) != qpOASES::SUCCESSFUL_RETURN ||
        !velocity.allFinite() || (velocity.array() < lower.array() - 1e-7).any() || (velocity.array() > upper.array() + 1e-7).any() ||
        (constraintCount && (constraints * velocity - constraintTarget).lpNorm<Eigen::Infinity>() > 1e-6)) {
      throw std::runtime_error("Whole-body IK QP infeasible or did not converge");
    }
    result.jointVelocity = velocity.tail(impl_->names.size());
    result.state.joints = state.joints + settings.dt * result.jointVelocity;
    if (offset) {
      result.baseVelocity = velocity.head<6>();
      result.state.worldFromBase.translation() += settings.dt * result.baseVelocity.head<3>();
      const Eigen::Vector3d rotation = settings.dt * result.baseVelocity.tail<3>();
      if (rotation.norm() > 1e-12) result.state.worldFromBase.linear() = Eigen::AngleAxisd(rotation.norm(), rotation.normalized()).toRotationMatrix() * state.worldFromBase.linear();
    }
    result.success = true;
  } catch (const std::exception& error) {
    result.message = error.what();
  }
  return result;
}

}