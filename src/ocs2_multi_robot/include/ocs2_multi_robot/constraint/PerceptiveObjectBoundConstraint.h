
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_multi_robot/common/Types.h>
#include "ocs2_multi_robot/terrain/HeightMap.h"

namespace ocs2
{
  namespace multi_robot
  {

    /**
     * @brief Terrain-aware height bound constraint for cargo.
     * 
     * Constrains cargo height RELATIVE to average terrain at nominal robot base positions:
     *   h_min <= cargo_z - avg_terrain_z(robot_positions) <= h_max
     * 
     * Robot positions are computed from cargo pose + rotated offsets.
     * Cargo state is at index: numRobots * SINGLE_ROBOT_STATE_DIM
     */
    class PerceptiveObjectBoundConstraint final : public StateConstraint
    {
    public:
      /**
       * Constructor with height map (terrain-relative bounds)
       * @param taskFile Path to config file with height_bounds settings
       * @param numRobots Number of robots in the system
       * @param heightMapPtr Shared pointer to height map for terrain queries
       * @param robotBaseOffsets Robot base offsets relative to cargo [x,y,z,yaw,pitch,roll]
       */
      PerceptiveObjectBoundConstraint(const std::string &taskFile, size_t numRobots, 
                                      std::shared_ptr<HeightMap> heightMapPtr,
                                      const std::vector<vector_t>& robotBaseOffsets = {})
          : StateConstraint(ConstraintOrder::Linear), numRobots_(numRobots), 
            heightMapPtr_(heightMapPtr), robotBaseOffsets_(robotBaseOffsets)
      {
        loadData::loadCppDataType(taskFile, "height_bounds.h_min", h_min);
        loadData::loadCppDataType(taskFile, "height_bounds.h_max", h_max);
      }

      ~PerceptiveObjectBoundConstraint() override = default;
      PerceptiveObjectBoundConstraint *clone() const override { return new PerceptiveObjectBoundConstraint(*this); }

      size_t getNumConstraints(scalar_t /*time*/) const override { return 2; }

      vector_t getValue(scalar_t /*time*/, const vector_t &state, const PreComputation &/*preComp*/) const override
      {
        const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
        const scalar_t cargo_x = state(cargoStateOffset);
        const scalar_t cargo_y = state(cargoStateOffset + 1);
        const scalar_t cargo_z = state(cargoStateOffset + 2);
        const scalar_t cargo_yaw = state(cargoStateOffset + 6);
        
        // Compute average terrain height at nominal robot base positions
        scalar_t terrain_z = computeAvgRobotTerrainHeight(cargo_x, cargo_y, cargo_yaw);
        scalar_t relative_height = cargo_z - terrain_z;
        
        // Constraints: h_min <= relative_height <= h_max
        vector_t constraint(2);
        constraint << relative_height - h_min, h_max - relative_height;
        return constraint;
      }

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;
        linearApproximation.f = getValue(time, state, preComp);
        
        const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
        const scalar_t cargo_x = state(cargoStateOffset);
        const scalar_t cargo_y = state(cargoStateOffset + 1);
        const scalar_t cargo_yaw = state(cargoStateOffset + 6);
        
        matrix_t C = matrix_t::Zero(2, state.size());
        
        // Compute terrain gradient via finite differences on avgRobotTerrainHeight
        constexpr scalar_t delta = 0.02;
        const scalar_t h0 = computeAvgRobotTerrainHeight(cargo_x, cargo_y, cargo_yaw);
        const scalar_t dh_dx = (computeAvgRobotTerrainHeight(cargo_x + delta, cargo_y, cargo_yaw) - 
                                computeAvgRobotTerrainHeight(cargo_x - delta, cargo_y, cargo_yaw)) / (2.0 * delta);
        const scalar_t dh_dy = (computeAvgRobotTerrainHeight(cargo_x, cargo_y + delta, cargo_yaw) - 
                                computeAvgRobotTerrainHeight(cargo_x, cargo_y - delta, cargo_yaw)) / (2.0 * delta);
        const scalar_t dh_dyaw = (computeAvgRobotTerrainHeight(cargo_x, cargo_y, cargo_yaw + delta) - 
                                  computeAvgRobotTerrainHeight(cargo_x, cargo_y, cargo_yaw - delta)) / (2.0 * delta);
        
        // Jacobian of relative_height = cargo_z - avg_terrain_z(cargo_x, cargo_y, cargo_yaw)
        // d(relative_height)/d(cargo_x) = -dh_dx
        // d(relative_height)/d(cargo_y) = -dh_dy
        // d(relative_height)/d(cargo_z) = 1
        // d(relative_height)/d(cargo_yaw) = -dh_dyaw
        
        // Constraint 0: relative_height - h_min
        C(0, cargoStateOffset)     = -dh_dx;   // d/d(cargo_x)
        C(0, cargoStateOffset + 1) = -dh_dy;   // d/d(cargo_y)
        C(0, cargoStateOffset + 2) = 1.0;      // d/d(cargo_z)
        C(0, cargoStateOffset + 6) = -dh_dyaw; // d/d(cargo_yaw)
        
        // Constraint 1: h_max - relative_height
        C(1, cargoStateOffset)     = dh_dx;    // d/d(cargo_x)
        C(1, cargoStateOffset + 1) = dh_dy;    // d/d(cargo_y)
        C(1, cargoStateOffset + 2) = -1.0;     // d/d(cargo_z)
        C(1, cargoStateOffset + 6) = dh_dyaw;  // d/d(cargo_yaw)
        
        linearApproximation.dfdx = C;
        return linearApproximation;
      }

    private:
      PerceptiveObjectBoundConstraint(const PerceptiveObjectBoundConstraint &other) = default;
      
      /**
       * @brief Compute average terrain height at nominal robot base positions.
       * Robot positions = cargo(x,y) + R_yaw * offset(x,y) for each robot.
       * Falls back to terrain at cargo position if no offsets are set.
       */
      scalar_t computeAvgRobotTerrainHeight(scalar_t cargo_x, scalar_t cargo_y, scalar_t cargo_yaw) const
      {
        if (!heightMapPtr_) return 0.0;
        
        if (robotBaseOffsets_.empty()) {
          // Fallback: use terrain at cargo position
          return heightMapPtr_->GetHeight(cargo_x, cargo_y);
        }
        
        const scalar_t cos_yaw = std::cos(cargo_yaw);
        const scalar_t sin_yaw = std::sin(cargo_yaw);
        
        scalar_t sum = 0.0;
        size_t count = 0;
        for (size_t r = 0; r < robotBaseOffsets_.size() && r < numRobots_; ++r) {
          const scalar_t ox = robotBaseOffsets_[r](0);
          const scalar_t oy = robotBaseOffsets_[r](1);
          const scalar_t robot_x = cargo_x + cos_yaw * ox - sin_yaw * oy;
          const scalar_t robot_y = cargo_y + sin_yaw * ox + cos_yaw * oy;
          sum += heightMapPtr_->GetHeight(robot_x, robot_y);
          count++;
        }
        return (count > 0) ? sum / count : 0.0;
      }
      
      scalar_t h_min, h_max;
      size_t numRobots_;
      std::shared_ptr<HeightMap> heightMapPtr_;
      std::vector<vector_t> robotBaseOffsets_;
    };

  } // namespace multi_robot
} // namespace ocs2
