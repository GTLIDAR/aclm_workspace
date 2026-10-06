#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>

namespace ocs2 {
namespace multi_robot {

static constexpr size_t NUM_ROBOTS = 2;
static constexpr size_t QUADRUPED_FOOT_NUM = 4;

template <typename T>
using feet_array_t = std::array<T, NUM_ROBOTS * QUADRUPED_FOOT_NUM>;
using contact_flag_t = std::vector<bool>;

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

static constexpr size_t QUADRUPED_CONTACT_DIM = 3;
static constexpr size_t ARM_CONTACT_DIM = 6;

// If false, the torque part of the 6D arm contact (indices 3-5) is ignored in the system dynamics.
// The input dimension remains ARM_CONTACT_DIM, but only the force part (0-2) affects the dynamics.
static constexpr bool USE_ARM_TORQUE_IN_DYNAMICS = true;

static constexpr size_t CARGO_STATE_DIM = 12;
static constexpr size_t SINGLE_ROBOT_STATE_DIM = 24;
static constexpr size_t SINGLE_ROBOT_INPUT_DIM = 24;
static constexpr size_t STATE_DIM_AE = 48;
static constexpr size_t INPUT_DIM_AE = 48;
static constexpr size_t STATE_DIM_TWO_QUAD_W_CARGO = 60;
static constexpr size_t INPUT_DIM_TWO_QUAD_W_CARGO = 60;
static constexpr size_t STATE_DIM_ELLIPSOID = 27;
static constexpr size_t INPUT_DIM_ELLIPSOID = 30;
static constexpr size_t STATE_DIM_AE_WITH_ELLIPSOID = 30;
static constexpr size_t INPUT_DIM_AE_WITH_ELLIPSOID = 30;
static constexpr size_t ROBOTS_FOOT_NUM = NUM_ROBOTS * QUADRUPED_FOOT_NUM; // was QUADRUPED_FOOT_NUM

static constexpr size_t DUAL_VARIABLE_DIM = ARM_CONTACT_DIM;
static constexpr size_t ALTERNATING_SINGLE_ROBOT_INPUT_DIM = SINGLE_ROBOT_INPUT_DIM + ARM_CONTACT_DIM;

/** @brief Indicate dimension in 2D */
enum Dim2D {
    dim2X = 0, dim2Y
};

/** @brief Indicate dimension in 3D */
enum Dim3D {
    dim3X = 0, dim3Y, dim3Z
};

static const vector3_t FL_ee_nominal = (vector_t(3) << 0.3455, 0.19875, -0.487695).finished();
static const vector3_t FR_ee_nominal = (vector_t(3) << 0.3455, -0.19875, -0.487695).finished();
static const vector3_t RL_ee_nominal = (vector_t(3) << -0.3455, 0.19875, -0.487695).finished();
static const vector3_t RR_ee_nominal = (vector_t(3) << -0.3455, -0.19875, -0.487695).finished();

// Nominal end-effector positions for a single robot (FL, FR, RL, RR)
static const std::vector<vector3_t> SINGLE_ROBOT_NOMINAL_EE_POS = {
  FL_ee_nominal,
  FR_ee_nominal,
  RL_ee_nominal,
  RR_ee_nominal
};

// Helper function to get nominal end-effector position for a local foot index (0-3)
inline vector3_t getNominalEePosWrtCom(size_t localFootIndex) {
  if (localFootIndex >= QUADRUPED_FOOT_NUM) {
    throw std::runtime_error("[getNominalEePosWrtCom] localFootIndex (" + 
                             std::to_string(localFootIndex) + ") must be < " + 
                             std::to_string(QUADRUPED_FOOT_NUM));
  }
  return SINGLE_ROBOT_NOMINAL_EE_POS[localFootIndex];
}

static const vector3_t max_dev_from_nominal = (vector_t(3) << 0.25, 0.25, 0.25).finished();
static const vector_t arm_max_dev_from_base = (vector_t(6) << 0.6, -0.4, 0.3, 
                                                              1.0, 0.4, 0.6).finished();
}  // namespace multi_robot
}  // namespace ocs2
