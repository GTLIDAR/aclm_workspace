#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_multi_robot/constraint/Obstacles.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2
{
  namespace multi_robot
  {

    /**
     * @brief Distributed CBF constraint for cargo avoiding 3D obstacles.
     * 
     * This is the distributed version of ObjectCBFConstraint for use in AlternatingInterface.
     * Cargo state: [0-11] = [x, y, z, vx, vy, vz, yaw, pitch, roll, ...]
     * Cargo position: [0, 1, 2] = [x, y, z]
     * Cargo velocity: [3, 4, 5] = [vx, vy, vz]
     */
    class CargoCBFConstraint final : public StateConstraint
    {
    public:
      CargoCBFConstraint(std::shared_ptr<Obstacles> obstacles, scalar_array_t radius_array)
          : StateConstraint(ConstraintOrder::Linear), radius_array_(radius_array), obstacles_(obstacles) {};

      ~CargoCBFConstraint() override = default;
      CargoCBFConstraint *clone() const override { return new CargoCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return obstacles_->getObstacles().size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        auto pos_array_ = obstacles_->getObstacles();

        vector_t constraint(pos_array_.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.05) * (radius_array_[i] + 0.05) +
                       (state(0) - pos_array_[i](0)) * (state(0) - pos_array_[i](0)) +
                       (state(1) - pos_array_[i](1)) * (state(1) - pos_array_[i](1)) +
                       (state(2) - pos_array_[i](2)) * (state(2) - pos_array_[i](2));
          constraint(i) = B +
                          2 * (state(0) - pos_array_[i](0)) * state(3) +
                          2 * (state(1) - pos_array_[i](1)) * state(4) +
                          2 * (state(2) - pos_array_[i](2)) * state(5);
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

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          C.row(i).setZero();
          C(i, 0) = 2 * (state(0) - pos_array_[i](0)) + 2 * state(3);
          C(i, 1) = 2 * (state(1) - pos_array_[i](1)) + 2 * state(4);
          C(i, 2) = 2 * (state(2) - pos_array_[i](2)) + 2 * state(5);
          C(i, 3) = 2 * (state(0) - pos_array_[i](0));
          C(i, 4) = 2 * (state(1) - pos_array_[i](1));
          C(i, 5) = 2 * (state(2) - pos_array_[i](2));
        }

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

    private:
      CargoCBFConstraint(const CargoCBFConstraint &other) = default;
      const scalar_array_t radius_array_;
      std::shared_ptr<Obstacles> obstacles_;
    };

  } // namespace multi_robot
} // namespace ocs2
