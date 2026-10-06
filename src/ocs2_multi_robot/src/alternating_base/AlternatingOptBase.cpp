#include "ocs2_multi_robot/alternating_base/AlternatingOptBase.h"

namespace ocs2 {
namespace multi_robot {
AlternatingOptBase::AlternatingOptBase(AlternatingOptSettings settings)
  : settings_(settings),
    totalNumIterations_(0) {
  maxNumIterations_ = settings_.max_iter;
  if (maxNumIterations_ <= 0) {
    maxNumIterations_ = std::numeric_limits<int>::infinity();
  }
}

void AlternatingOptBase::reset() {
  primalSolution_ = PrimalSolution();
  totalNumIterations_ = 0;
  totalPerformanceIndeces_ = PerformanceIndex();
}

void AlternatingOptBase::runImpl(scalar_t initTime, 
                                 const vector_t& initState, 
                                 scalar_t finalTime) {
  if (settings_.printSolverStatus) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ ADMM solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  int iter = 0;
  AlternatingPreRun(iter, initTime, initState, finalTime);

  std::vector<PrimalSolution> optimizedSolutions;
  Convergence convergence = Convergence::FALSE;
  while (convergence == Convergence::FALSE) {
    if (settings_.printSolverStatus) {
      std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++";
      std::cerr << "\n++++ ADMM solver is running at iteration " << iter << " +++++";
      std::cerr << "\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    }

    optimizedSolutions = iterImpl(iter, initTime, initState, finalTime);

    // Check convergence
    convergence = checkConvergence(iter);

    iter++;
    totalNumIterations_++;
  }

  // TODO: add the primal solution extraction here
  primalSolution_ = toPrimalSolution(optimizedSolutions);

  AlternatingPostRun(iter, initTime, initState, finalTime);
  if (settings_.printSolverStatus) {
    std::cerr << "\nConvergence : " << toString(convergence) << "\n";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ ADMM solver has terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }
}

} // namespace multi_robot
} // namespace ocs2