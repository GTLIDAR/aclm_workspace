
#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_multi_robot/constraint/Obstacles.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2
{
  namespace multi_robot
  {

    /**
     * @brief CBF constraint for cargo avoiding 3D obstacles.
     * 
     * Cargo state starts at index numRobots * SINGLE_ROBOT_STATE_DIM.
     * Cargo position: [cargoBase + 0, cargoBase + 1, cargoBase + 2] = [x, y, z]
     * Cargo velocity: [cargoBase + 3, cargoBase + 4, cargoBase + 5] = [vx, vy, vz]
     */
    class ObjectCBFConstraint final : public StateConstraint
    {
    public:
      ObjectCBFConstraint(std::shared_ptr<Obstacles> obstacles, scalar_array_t radius_array, size_t numRobots)
          : StateConstraint(ConstraintOrder::Linear), radius_array_(radius_array), obstacles_(obstacles), numRobots_(numRobots) {};

      ~ObjectCBFConstraint() override = default;
      ObjectCBFConstraint *clone() const override { return new ObjectCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return obstacles_->getObstacles().size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        auto pos_array_ = obstacles_->getObstacles();

        vector_t constraint(pos_array_.size());

        // Cargo state base index
        const size_t cargoBase = numRobots_ * SINGLE_ROBOT_STATE_DIM;

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.05) * (radius_array_[i] + 0.05) +
                       (state(cargoBase + 0) - pos_array_[i](0)) * (state(cargoBase + 0) - pos_array_[i](0)) +
                       (state(cargoBase + 1) - pos_array_[i](1)) * (state(cargoBase + 1) - pos_array_[i](1)) +
                       (state(cargoBase + 2) - pos_array_[i](2)) * (state(cargoBase + 2) - pos_array_[i](2));
          constraint(i) = B +
                          2 * (state(cargoBase + 0) - pos_array_[i](0)) * state(cargoBase + 3) +
                          2 * (state(cargoBase + 1) - pos_array_[i](1)) * state(cargoBase + 4) +
                          2 * (state(cargoBase + 2) - pos_array_[i](2)) * state(cargoBase + 5);
        }

        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;

        auto pos_array_ = obstacles_->getObstacles();
        linearApproximation.f = getValue(time, state, preComp);

        matrix_t C(pos_array_.size(), state.size());

        // Cargo state base index
        const size_t cargoBase = numRobots_ * SINGLE_ROBOT_STATE_DIM;

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          C.row(i).setZero();
          C(i, cargoBase + 0) = 2 * (state(cargoBase + 0) - pos_array_[i](0)) + 2 * state(cargoBase + 3);
          C(i, cargoBase + 1) = 2 * (state(cargoBase + 1) - pos_array_[i](1)) + 2 * state(cargoBase + 4);
          C(i, cargoBase + 2) = 2 * (state(cargoBase + 2) - pos_array_[i](2)) + 2 * state(cargoBase + 5);
          C(i, cargoBase + 3) = 2 * (state(cargoBase + 0) - pos_array_[i](0));
          C(i, cargoBase + 4) = 2 * (state(cargoBase + 1) - pos_array_[i](1));
          C(i, cargoBase + 5) = 2 * (state(cargoBase + 2) - pos_array_[i](2));
        }

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

    private:
      ObjectCBFConstraint(const ObjectCBFConstraint &other) = default;
      const scalar_array_t radius_array_;
      std::shared_ptr<Obstacles> obstacles_;
      size_t numRobots_;
    };

  } // namespace multi_robot
} // namespace ocs2
