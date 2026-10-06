#pragma once

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/search_strategy/FilterLinesearch.h>

namespace ocs2 {
namespace multi_robot {

/** Different types of convergence */
enum class Convergence { FALSE, ITERATIONS, PRIMAL };

/** Struct to contain the result and logging data of the stepsize computation */
struct AlternatingSolverStatus {
  AlternatingSolverStatus() {
    clear();
  }
  
  void clear() {
    subproblemPerformanceIndices.clear();
    totalConstraintViolationAfterStep = 0.0;
    totalPrimalResidualSSE = 0.0;
    perConstraintResidualSSE.clear();
    totalDualResidualSSE = 0.0;
    perConstraintDualResidualSSE.clear();
  }
  // Performance result after the step
  std::vector<PerformanceIndex> subproblemPerformanceIndices;
  scalar_t totalConstraintViolationAfterStep;  // constraint metric used in the line search
  scalar_t totalPrimalResidualSSE;        // primal residual metric
  std::vector<scalar_t> perConstraintResidualSSE;  // primal residual SSE for each consensus constraint (each robot)

  // Dual residual metrics (standard ADMM style)
  // totalDualResidualSSE aggregates ||s^k|| over all robots, handles, and time steps
  scalar_t totalDualResidualSSE;
  // perConstraintDualResidualSSE stores dual residual SSE per robot
  std::vector<scalar_t> perConstraintDualResidualSSE;
};

/** Transforms sqp::Convergence to string */
inline std::string toString(const Convergence& convergence) {
  switch (convergence) {
    case Convergence::ITERATIONS:
      return "[ADMM] Maximum number of iterations reached";
    case Convergence::PRIMAL:
      return "[ADMM] Primal update below tolerance";
    case Convergence::FALSE:
    default:
      return "[ADMM] Not Converged";
  }
}

}  // namespace multi_robot
}  // namespace ocs2
