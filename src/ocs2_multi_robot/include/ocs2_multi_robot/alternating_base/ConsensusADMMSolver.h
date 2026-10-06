#pragma once

#include <memory>
#include <vector>

#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_sqp/SqpSolver.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ocs2_core/misc/Benchmark.h>

#include "ocs2_multi_robot/AlternatingPreComputation.h"
#include "ocs2_multi_robot/alternating_base/AlternatingInterface.h"
#include "ocs2_multi_robot/alternating_base/AlternatingOptBase.h"
#include "ocs2_multi_robot/alternating_base/ThreadSafeReferenceManager.h"
#include "ocs2_multi_robot/common/AlternatingTargetTrajectories.h"
#include "ocs2_multi_robot/common/Types.h"
#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreadingHelperFunctions.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>
#include <ocs2_oc/multiple_shooting/Initialization.h>

namespace ocs2 {
class SqpSolver;
namespace multi_robot {

class ConsensusADMMSolver : public AlternatingOptBase {
public:
  ConsensusADMMSolver(const AlternatingInterface& alternatingInterface);
  ~ConsensusADMMSolver() override;

  void reset() override;

private:
  void AlternatingPreRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;
  std::vector<PrimalSolution> iterImpl(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;
  void AlternatingPostRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;
  Convergence checkConvergence(int iteration) override;
  PrimalSolution toPrimalSolution(std::vector<PrimalSolution> optimizedStates) override;

  void runCargoSubproblem(scalar_t initTime, scalar_t finalTime);
  void runRobotSubproblems(scalar_t initTime, scalar_t finalTime);
  void runRobotSubproblemsParallel(scalar_t initTime, scalar_t finalTime);
  void runCargoAndRobotSubproblemsParallel(scalar_t initTime, scalar_t finalTime);
  void updatePreComputations();
  void updateDualVariables();

  /**
   * Update the previous solution with the current solution;
   * This could happen in two cases:
   * 1. First ADMM iteration update with mixed time stamps of cargo and robot solutions.
   * 2. Subsequent ADMM iterations update with the same time stamps of cargo and robot solutions.
   */
  void updatePreviousSolution();

  /**
   * Update the previous solution with the reference and cargo solution;
   * Only needed for sequential update type with mixed time stamps of reference and cargo.
   */
  void updatePreviousSolutionWithRefAndCargo();

  void initializeStateInputDualTrajectories(const vector_t& initState, const std::vector<AnnotatedTime>& timeDiscretization,
    const PrimalSolution& primalSolution, Initializer& initializer, vector_array_t& stateTrajectory,
    vector_array_t& inputTrajectory, vector_array_t& dualVariableTrajectory) const;

  const AlternatingInterface& alternatingInterface_;

  size_t numRobots_;
  size_t cargoStateDim_{0};
  size_t cargoInputDim_{0};
  size_t robotStateDim_{0};
  size_t robotInputDim_{0};

  std::shared_ptr<OptimalControlProblem> cargoOcpPtr_;
  std::vector<std::shared_ptr<OptimalControlProblem>> robotOcpPtrs_;

  std::unique_ptr<SqpSolver> cargoSolver_;
  std::vector<std::unique_ptr<SqpSolver>> robotSolvers_;

  // previousSolution_ is used to feed AlternatingPreComputation during the *current*
  // iteration (it may be updated mid-iteration in SEQUENTIAL mode).
  AlternatingTargetTrajectories previousSolution_;

  // previousSolutionForDual_ stores the full stacked solution from the *previous*
  // ADMM iteration and is only updated once per iteration, after updateDualVariables().
  // This is used exclusively to compute dual residuals across iterations.
  AlternatingTargetTrajectories previousSolutionForDual_;
  vector_array_t dualTrajectory_;
  vector_array_t residualTrajectory_;
  AlternatingSolverStatus solverStatus_;
  std::vector<AlternatingSolverStatus> iterationStatusHistory_;  // Store status for each ADMM iteration

  PrimalSolution cargoSolution_;
  std::vector<PrimalSolution> robotSolutions_;

  vector_t cargoInitialState_;
  std::vector<vector_t> robotInitialStates_;
  vector_t rho_;
  
  ThreadPool threadPool_;  // Thread pool for parallel execution
  std::shared_ptr<ThreadSafeReferenceManager> threadSafeReferenceManager_;  // Thread-safe wrapper for reference manager

  // Benchmarking timers
  benchmark::RepeatedTimer cargoSubproblemTimer_;
  benchmark::RepeatedTimer robotSubproblemsTimer_;
  benchmark::RepeatedTimer dualUpdateTimer_;
  
  // Total ADMM iterations across all MPC runs
  size_t totalAdmmIterations_{0};

  std::unique_ptr<Initializer> initializerPtr_;

  // Structure to store MPC solve residual data
  struct MpcSolveResidual {
    scalar_t initTime;
    scalar_t totalPrimalResidual;
    std::vector<scalar_t> perConstraintPrimalResidual;
  };
  
  // Buffer to store residual data after each MPC solve
  std::vector<MpcSolveResidual> mpcResidualHistory_;
};

} // namespace multi_robot
} // namespace ocs2