#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace legged_kinematics {

class WholeBodyDifferentialIK {
 public:
  enum class Model { B1, B1Z1, Arm };
  enum class BaseMode { Fixed, Floating };
  enum class LegMode { Differential, Analytical };
  struct LegGeometry {
    std::array<Eigen::Vector3d, 4> baseToHip{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    Eigen::Vector3d hipToThigh = Eigen::Vector3d::Zero();
    double thighLength = 0.0;
    double shankLength = 0.0;
  };
  struct Settings {
    Model model = Model::B1Z1;
    BaseMode baseMode = BaseMode::Fixed;
    LegMode legMode = LegMode::Differential;
    double dt = 0.002;
    double gain = 5.0;
    double footWeight = 1.0;
    double armPositionWeight = 1.0;
    double armOrientationWeight = 1.0;
    double baseWeight = 1.0;
    double regularization = 1e-4;
    double maxLinearSpeed = 0.25;
    double maxAngularSpeed = 1.0;
    double maxBaseLinearSpeed = 0.2;
    double maxBaseAngularSpeed = 0.5;
    std::optional<LegGeometry> analyticalLegs;
  };
  struct State {
    Eigen::Isometry3d worldFromBase = Eigen::Isometry3d::Identity();
    Eigen::VectorXd joints;
  };
  struct Targets {
    std::array<Eigen::Vector3d, 4> feet{{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
    std::array<bool, 4> stance{{false, false, false, false}};
    Eigen::Isometry3d arm = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base = Eigen::Isometry3d::Identity();
    bool trackBase = false;
  };
  struct Result {
    State state;
    Eigen::VectorXd jointVelocity;
    Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    bool success = false;
    std::string message;
  };

  WholeBodyDifferentialIK(const std::string& urdfFile, Settings settings);
  ~WholeBodyDifferentialIK();
  Result solve(const State& state, const Targets& targets);
  Targets currentTargets(const State& state) const;
  State neutralState() const;
  void setTaskWeights(double position, double orientation, double regularization);
  void setTimeStep(double dt);
  const std::vector<std::string>& jointNames() const;
  const Eigen::VectorXd& lowerLimits() const;
  const Eigen::VectorXd& upperLimits() const;
  const Eigen::VectorXd& velocityLimits() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}