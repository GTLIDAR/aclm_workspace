#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Distributed orientation constraint for pitch and roll angles of a single robot.
 * 
 * Robot state: [0-23] = [x, y, z, vx, vy, vz, yaw, pitch, roll, ...]
 * 
 * This constraint enforces bounds on pitch and roll:
 *   -pitch_max <= pitch <= pitch_max
 *   -roll_max <= roll <= roll_max
 * 
 * Constraint formulation (h >= 0):
 *   h1 = pitch_max - pitch (upper bound)
 *   h2 = pitch + pitch_max (lower bound)
 *   h3 = roll_max - roll (upper bound)
 *   h4 = roll + roll_max (lower bound)
 * 
 * Total: 4 constraints
 */
class SingleRobotOrientationConstraint final : public StateConstraint {
 public:
  /**
   * @brief Configuration for orientation bounds
   */
  struct Config {
    scalar_t pitch_max = 0.5;   // Maximum pitch angle (rad)
    scalar_t roll_max = 0.5;    // Maximum roll angle (rad)
  };

  /**
   * @brief Constructor
   * @param config Configuration containing orientation bounds
   */
  SingleRobotOrientationConstraint(const Config& config)
      : StateConstraint(ConstraintOrder::Linear), config_(config) {}

  /**
   * @brief Constructor that loads configuration from task file
   * @param taskFile Path to the task file
   */
  SingleRobotOrientationConstraint(const std::string& taskFile)
      : StateConstraint(ConstraintOrder::Linear) {
    loadData::loadCppDataType(taskFile, "orientationConstraint.robot_pitch_max", config_.pitch_max);
    loadData::loadCppDataType(taskFile, "orientationConstraint.robot_roll_max", config_.roll_max);
  }

  ~SingleRobotOrientationConstraint() override = default;
  SingleRobotOrientationConstraint* clone() const override { return new SingleRobotOrientationConstraint(*this); }

  /**
   * @brief Returns the number of constraints
   * 4 constraints (pitch upper/lower, roll upper/lower)
   */
  size_t getNumConstraints(scalar_t /*time*/) const override { return 4; }

  /**
   * @brief Evaluates the constraint values
   * Constraints are formulated as h >= 0
   */
  vector_t getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const override {
    vector_t constraint(4);

    const scalar_t pitch = state(7);  // pitch at offset 7
    const scalar_t roll = state(8);   // roll at offset 8

    constraint(0) = config_.pitch_max - pitch;   // pitch upper bound
    constraint(1) = pitch + config_.pitch_max;   // pitch lower bound
    constraint(2) = config_.roll_max - roll;     // roll upper bound
    constraint(3) = roll + config_.roll_max;     // roll lower bound

    return constraint;
  }

  /**
   * @brief Computes the linear approximation of the constraint
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation linearApproximation;
    linearApproximation.f = getValue(time, state, preComp);

    matrix_t C = matrix_t::Zero(4, state.size());

    // pitch at index 7, roll at index 8
    C(0, 7) = -1.0;   // d(pitch_max - pitch)/d(pitch)
    C(1, 7) = 1.0;    // d(pitch + pitch_max)/d(pitch)
    C(2, 8) = -1.0;   // d(roll_max - roll)/d(roll)
    C(3, 8) = 1.0;    // d(roll + roll_max)/d(roll)

    linearApproximation.dfdx = C;
    return linearApproximation;
  }

 private:
  SingleRobotOrientationConstraint(const SingleRobotOrientationConstraint& other) = default;
  Config config_;
};

}  // namespace multi_robot
}  // namespace ocs2
