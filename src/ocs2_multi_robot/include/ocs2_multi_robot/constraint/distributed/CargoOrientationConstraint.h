#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Distributed orientation constraint for pitch and roll angles of cargo.
 * 
 * Cargo state: [0-11] = [x, y, z, vx, vy, vz, yaw, pitch, roll, ...]
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
class CargoOrientationConstraint final : public StateConstraint {
 public:
  /**
   * @brief Configuration for orientation bounds
   */
  struct Config {
    scalar_t pitch_max = 0.3;   // Maximum pitch angle (rad)
    scalar_t roll_max = 0.3;    // Maximum roll angle (rad)
  };

  /**
   * @brief Constructor
   * @param config Configuration containing orientation bounds
   */
  CargoOrientationConstraint(const Config& config)
      : StateConstraint(ConstraintOrder::Linear), config_(config) {}

  /**
   * @brief Constructor that loads configuration from task file
   * @param taskFile Path to the task file
   */
  CargoOrientationConstraint(const std::string& taskFile)
      : StateConstraint(ConstraintOrder::Linear) {
    loadData::loadCppDataType(taskFile, "orientationConstraint.cargo_pitch_max", config_.pitch_max);
    loadData::loadCppDataType(taskFile, "orientationConstraint.cargo_roll_max", config_.roll_max);
  }

  ~CargoOrientationConstraint() override = default;
  CargoOrientationConstraint* clone() const override { return new CargoOrientationConstraint(*this); }

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

    const scalar_t cargo_pitch = state(7);  // pitch at offset 7 within cargo state
    const scalar_t cargo_roll = state(8);   // roll at offset 8 within cargo state

    constraint(0) = config_.pitch_max - cargo_pitch;   // pitch upper bound
    constraint(1) = cargo_pitch + config_.pitch_max;   // pitch lower bound
    constraint(2) = config_.roll_max - cargo_roll;     // roll upper bound
    constraint(3) = cargo_roll + config_.roll_max;     // roll lower bound

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

    C(0, 7) = -1.0;
    C(1, 7) = 1.0;
    C(2, 8) = -1.0;
    C(3, 8) = 1.0;

    linearApproximation.dfdx = C;
    return linearApproximation;
  }

 private:
  CargoOrientationConstraint(const CargoOrientationConstraint& other) = default;
  Config config_;
};

}  // namespace multi_robot
}  // namespace ocs2
