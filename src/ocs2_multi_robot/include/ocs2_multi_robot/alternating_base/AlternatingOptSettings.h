#pragma once

#include <string>
#include <ocs2_core/Types.h>

namespace ocs2 {
namespace multi_robot {

/** Different types of ADMM update */
enum class ADMMUpdateType { CONSENSUS, SEQUENTIAL };

inline ADMMUpdateType fromString(const std::string& updateTypeName) {
  if (updateTypeName == "consensus") {
    return ADMMUpdateType::CONSENSUS;
  } else if (updateTypeName == "sequential") {
    return ADMMUpdateType::SEQUENTIAL;
  } else {
    throw std::runtime_error("[AlternatingOptSettings] Invalid update type: " + updateTypeName);
  }
}

inline std::string toString(const ADMMUpdateType& updateType) {
  if (updateType == ADMMUpdateType::CONSENSUS) {
    return "consensus";
  } else if (updateType == ADMMUpdateType::SEQUENTIAL) {
    return "sequential";
  } else {
    throw std::runtime_error("[AlternatingOptSettings] Invalid update type");
  }
}

struct AlternatingOptSettings {
  int max_iter = 1;
  bool printSolverStatus = true;
  bool printSolverStatistics = false;  // Print benchmarking statistics after solver exits
  int numConsensusConstraints = 0;
  scalar_t primalTolerance = 1e-4;
  vector_t rho;
  ADMMUpdateType updateType = ADMMUpdateType::SEQUENTIAL;
  bool parallelUpdate = false;  // Enable parallel execution of subproblems

  // Varying-penalty (adaptive rho) settings, based on the standard rule:
  //   if ||r^k||^2 > μ ||d^k||^2      ->  ρ^{k+1} = τ_incr ρ^k
  //   if ||d^k||^2 > μ ||r^k||^2      ->  ρ^{k+1} = ρ^k / τ_decr
  //   otherwise                       ->  ρ^{k+1} = ρ^k
  //
  // When enableAdaptiveRho is false, ρ stays fixed.
  bool enableAdaptiveRho = false;
  scalar_t adaptiveRhoMu = 10.0;       // μ > 1
  scalar_t adaptiveRhoTauIncr = 2.0;   // τ_incr > 1
  scalar_t adaptiveRhoTauDecr = 2.0;   // τ_decr > 1

  // Dual reset settings:
  // - If resetDualWhenSingleIteration is true and max_iter == 1, we reset duals to zero
  //   at the end of the ADMM solve, to avoid polluting the next MPC call.
  // - If any subproblem cost exceeds dualResetCostThreshold, we also reset duals.
  bool resetDualWhenSingleIteration = true;
  scalar_t dualResetCostThreshold   = 1e8;

  // MPC residual logging settings:
  // - If logMpcResiduals is true, the solver will log the final primal residual after each MPC solve
  //   to the file specified by mpcResidualLogPath
  bool logMpcResiduals = false;
  std::string mpcResidualLogPath = "/tmp/data/mpc_residuals.txt";
};

AlternatingOptSettings loadAlternatingSettings(const std::string& filename, 
  const std::string& fieldName = "alternating", bool verbose = true);
  
} // namespace multi_robot
} // namespace ocs2