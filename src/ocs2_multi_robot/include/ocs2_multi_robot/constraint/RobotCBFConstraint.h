#pragma once

#include <ocs2_multi_robot/constraint/ObjectCBFConstraint.h>

namespace ocs2
{
  namespace multi_robot
  {

    /**
     * @brief Generalized CBF constraint for any robot in a multi-robot system.
     * 
     * This class replaces the individual Robot1CBFConstraint, Robot2CBFConstraint, etc.
     * by parameterizing the state indices based on robot index.
     * 
     * State layout per robot (24 states each):
     *   - indices 0, 1: position x, y
     *   - indices 3, 4: velocity x, y
     * 
     * For robot i (0-indexed), the state offset is i * 24.
     */
    class RobotCBFConstraint final : public StateConstraint
    {
    public:
      /**
       * @brief Construct a RobotCBFConstraint for a specific robot.
       * 
       * @param obstacles Shared pointer to obstacle positions
       * @param radius_array Array of obstacle radii
       * @param robotIndex 0-indexed robot number (0 for Robot1, 1 for Robot2, etc.)
       * @param stateOffsetPerRobot Number of states per robot (default 24)
       */
      RobotCBFConstraint(std::shared_ptr<Obstacles> obstacles, 
                         scalar_array_t radius_array,
                         size_t robotIndex,
                         size_t stateOffsetPerRobot = 24)
          : StateConstraint(ConstraintOrder::Linear), 
            radius_array_(radius_array), 
            obstacles_(obstacles),
            robotIndex_(robotIndex),
            stateOffset_(robotIndex * stateOffsetPerRobot),
            posXIdx_(stateOffset_ + 0),
            posYIdx_(stateOffset_ + 1),
            velXIdx_(stateOffset_ + 3),
            velYIdx_(stateOffset_ + 4) {};

      ~RobotCBFConstraint() override = default;
      RobotCBFConstraint *clone() const override { return new RobotCBFConstraint(*this); }

      size_t getNumConstraints(scalar_t time) const override { return obstacles_->getObstacles().size(); };

      vector_t getValue(scalar_t time, const vector_t &state, const PreComputation &preComp) const override
      {
        auto pos_array_ = obstacles_->getObstacles();

        vector_t constraint(pos_array_.size());

        for (size_t i = 0; i < pos_array_.size(); ++i)
        {
          scalar_t B = -(radius_array_[i] + 0.2) * (radius_array_[i] + 0.2) +
                       (state(posXIdx_) - pos_array_[i](0)) * (state(posXIdx_) - pos_array_[i](0)) +
                       (state(posYIdx_) - pos_array_[i](1)) * (state(posYIdx_) - pos_array_[i](1));
          constraint(i) = B +
                          2 * (state(posXIdx_) - pos_array_[i](0)) * state(velXIdx_) +
                          2 * (state(posYIdx_) - pos_array_[i](1)) * state(velYIdx_);
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
          C(i, posXIdx_) = 2 * (state(posXIdx_) - pos_array_[i](0)) + 2 * state(velXIdx_);
          C(i, posYIdx_) = 2 * (state(posYIdx_) - pos_array_[i](1)) + 2 * state(velYIdx_);
          C(i, velXIdx_) = 2 * (state(posXIdx_) - pos_array_[i](0));
          C(i, velYIdx_) = 2 * (state(posYIdx_) - pos_array_[i](1));
        }

        linearApproximation.dfdx = C;
        return linearApproximation;
      };

      size_t getRobotIndex() const { return robotIndex_; }

    private:
      RobotCBFConstraint(const RobotCBFConstraint &other) = default;
      
      const scalar_array_t radius_array_;
      std::shared_ptr<Obstacles> obstacles_;
      
      const size_t robotIndex_;
      const size_t stateOffset_;
      const size_t posXIdx_;
      const size_t posYIdx_;
      const size_t velXIdx_;
      const size_t velYIdx_;
    };

  } // namespace multi_robot
} // namespace ocs2
