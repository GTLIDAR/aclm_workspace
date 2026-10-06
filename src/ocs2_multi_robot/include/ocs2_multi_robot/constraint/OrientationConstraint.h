#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Orientation constraint for pitch and roll angles of robots and cargo.
 * 
 * State layout (for N quadrupeds with cargo):
 *   Robot i: [i*24 + 0-2] COM position, [i*24 + 3-5] COM velocity, [i*24 + 6-8] ZYX orientation (yaw, pitch, roll), 
 *            [i*24 + 9-11] angular momentum, [i*24 + 12-23] foot positions
 *   Cargo:   [N*24 + 0-2] COM position, [N*24 + 3-5] COM velocity, [N*24 + 6-8] ZYX orientation (yaw, pitch, roll), 
 *            [N*24 + 9-11] angular momentum
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
 * Total: 4 constraints per entity (robots + cargo) = 4 * (numRobots + 1) constraints
 */
class OrientationConstraint final : public StateConstraint {
 public:
  /**
   * @brief Configuration for orientation bounds
   */
  struct Config {
    scalar_t robot_pitch_max = 0.5;   // Maximum pitch angle for robots (rad)
    scalar_t robot_roll_max = 0.5;    // Maximum roll angle for robots (rad)
    scalar_t cargo_pitch_max = 0.3;   // Maximum pitch angle for cargo (rad)
    scalar_t cargo_roll_max = 0.3;    // Maximum roll angle for cargo (rad)
  };

  /**
   * @brief Constructor
   * @param config Configuration containing orientation bounds
   * @param numRobots Number of robots
   */
  OrientationConstraint(const Config& config, size_t numRobots)
      : StateConstraint(ConstraintOrder::Linear), config_(config), numRobots_(numRobots) {}

  /**
   * @brief Constructor that loads configuration from task file
   * @param taskFile Path to the task file
   * @param numRobots Number of robots
   */
  OrientationConstraint(const std::string& taskFile, size_t numRobots)
      : StateConstraint(ConstraintOrder::Linear), numRobots_(numRobots) {
    loadData::loadCppDataType(taskFile, "orientationConstraint.robot_pitch_max", config_.robot_pitch_max);
    loadData::loadCppDataType(taskFile, "orientationConstraint.robot_roll_max", config_.robot_roll_max);
    loadData::loadCppDataType(taskFile, "orientationConstraint.cargo_pitch_max", config_.cargo_pitch_max);
    loadData::loadCppDataType(taskFile, "orientationConstraint.cargo_roll_max", config_.cargo_roll_max);
  }

  ~OrientationConstraint() override = default;
  OrientationConstraint* clone() const override { return new OrientationConstraint(*this); }

  /**
   * @brief Returns the number of constraints
   * 4 constraints (pitch upper/lower, roll upper/lower) x (numRobots + 1 cargo)
   */
  size_t getNumConstraints(scalar_t /*time*/) const override { return 4 * (numRobots_ + 1); }

  /**
   * @brief Evaluates the constraint values
   * Constraints are formulated as h >= 0
   */
  vector_t getValue(scalar_t /*time*/, const vector_t& state, const PreComputation& /*preComp*/) const override {
    const size_t numConstraints = 4 * (numRobots_ + 1);
    vector_t constraint(numConstraints);

    // Robot constraints
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const size_t baseIdx = robot * SINGLE_ROBOT_STATE_DIM;
      const scalar_t pitch = state(baseIdx + 7);  // pitch at offset 7
      const scalar_t roll = state(baseIdx + 8);   // roll at offset 8

      const size_t constraintIdx = robot * 4;
      constraint(constraintIdx + 0) = config_.robot_pitch_max - pitch;   // pitch upper bound
      constraint(constraintIdx + 1) = pitch + config_.robot_pitch_max;   // pitch lower bound
      constraint(constraintIdx + 2) = config_.robot_roll_max - roll;     // roll upper bound
      constraint(constraintIdx + 3) = roll + config_.robot_roll_max;     // roll lower bound
    }

    // Cargo constraints
    const size_t cargoBaseIdx = numRobots_ * SINGLE_ROBOT_STATE_DIM;
    const scalar_t cargo_pitch = state(cargoBaseIdx + 7);  // pitch at offset 7 within cargo state
    const scalar_t cargo_roll = state(cargoBaseIdx + 8);   // roll at offset 8 within cargo state

    const size_t cargoConstraintIdx = numRobots_ * 4;
    constraint(cargoConstraintIdx + 0) = config_.cargo_pitch_max - cargo_pitch;   // pitch upper bound
    constraint(cargoConstraintIdx + 1) = cargo_pitch + config_.cargo_pitch_max;   // pitch lower bound
    constraint(cargoConstraintIdx + 2) = config_.cargo_roll_max - cargo_roll;     // roll upper bound
    constraint(cargoConstraintIdx + 3) = cargo_roll + config_.cargo_roll_max;     // roll lower bound

    return constraint;
  }

  /**
   * @brief Computes the linear approximation of the constraint
   */
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                           const PreComputation& preComp) const override {
    VectorFunctionLinearApproximation linearApproximation;
    linearApproximation.f = getValue(time, state, preComp);

    const size_t numConstraints = 4 * (numRobots_ + 1);
    matrix_t C = matrix_t::Zero(numConstraints, state.size());

    // Robot Jacobians
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const size_t baseIdx = robot * SINGLE_ROBOT_STATE_DIM;
      const size_t constraintIdx = robot * 4;

      // pitch at baseIdx + 7, roll at baseIdx + 8
      C(constraintIdx + 0, baseIdx + 7) = -1.0;   // d(pitch_max - pitch)/d(pitch)
      C(constraintIdx + 1, baseIdx + 7) = 1.0;    // d(pitch + pitch_max)/d(pitch)
      C(constraintIdx + 2, baseIdx + 8) = -1.0;   // d(roll_max - roll)/d(roll)
      C(constraintIdx + 3, baseIdx + 8) = 1.0;    // d(roll + roll_max)/d(roll)
    }

    // Cargo Jacobian
    const size_t cargoBaseIdx = numRobots_ * SINGLE_ROBOT_STATE_DIM;
    const size_t cargoConstraintIdx = numRobots_ * 4;

    C(cargoConstraintIdx + 0, cargoBaseIdx + 7) = -1.0;
    C(cargoConstraintIdx + 1, cargoBaseIdx + 7) = 1.0;
    C(cargoConstraintIdx + 2, cargoBaseIdx + 8) = -1.0;
    C(cargoConstraintIdx + 3, cargoBaseIdx + 8) = 1.0;

    linearApproximation.dfdx = C;
    return linearApproximation;
  }

 private:
  OrientationConstraint(const OrientationConstraint& other) = default;
  Config config_;
  size_t numRobots_;
};

}  // namespace multi_robot
}  // namespace ocs2
