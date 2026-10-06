#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_multi_robot/common/Types.h>

namespace ocs2
{
  namespace multi_robot
  {

    /**
     * @brief Distributed height bound constraint for cargo.
     * 
     * Cargo z position is at index 2.
     * 
     * Constraint formulation (h >= 0):
     *   h1 = z - h_min (lower bound)
     *   h2 = h_max - z (upper bound)
     */
    class CargoBoundConstraint final : public StateConstraint
    {
    public:
      CargoBoundConstraint(const std::string &taskFile)
          : StateConstraint(ConstraintOrder::Linear)
      {
        loadData::loadCppDataType(taskFile, "height_bounds.h_min", h_min);
        loadData::loadCppDataType(taskFile, "height_bounds.h_max", h_max);
      };

      ~CargoBoundConstraint() override = default;
      CargoBoundConstraint *clone() const override { return new CargoBoundConstraint(*this); }

      size_t getNumConstraints(scalar_t /*time*/) const override { return 2; };

      vector_t getValue(scalar_t /*time*/, const vector_t &state, const PreComputation &/*preComp*/) const override
      {
        // Cargo z position index
        const size_t cargoZIdx = 2;
        
        // Only constrain the cargo's height
        vector_t constraint(2);
        constraint << state(cargoZIdx) - h_min, h_max - state(cargoZIdx);
        return constraint;
      };

      VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t &state,
                                                               const PreComputation &preComp) const override
      {
        VectorFunctionLinearApproximation linearApproximation;
        linearApproximation.f = getValue(time, state, preComp);
        
        // Cargo z position index
        const size_t cargoZIdx = 2;
        
        matrix_t C = matrix_t::Zero(2, state.size());
        // Derivative with respect to cargo z position
        C(0, cargoZIdx) = 1;
        C(1, cargoZIdx) = -1;
        linearApproximation.dfdx = C;
        return linearApproximation;
      };

    private:
      CargoBoundConstraint(const CargoBoundConstraint &other) = default;
      scalar_t h_min, h_max;
    };

  } // namespace multi_robot
} // namespace ocs2
