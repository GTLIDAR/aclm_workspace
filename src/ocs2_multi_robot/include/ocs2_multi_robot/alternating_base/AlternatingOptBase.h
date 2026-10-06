#pragma once

#include <iostream>
#include <chrono>
#include <Eigen/Dense>

#include <ocs2_oc/oc_solver/SolverBase.h>
#include <ocs2_oc/oc_data/TimeDiscretization.h>

#include "ocs2_multi_robot/alternating_base/AlternatingOptSettings.h"
#include "ocs2_multi_robot/alternating_base/AlternatingSolverStatus.h"
#include "ocs2_multi_robot/utils/Logger.h"

namespace ocs2 {
namespace multi_robot {

class AlternatingOptBase : public SolverBase {
public:
  AlternatingOptBase(AlternatingOptSettings settings);

  virtual ~AlternatingOptBase() = default;

  virtual void reset() override;

  scalar_t getFinalTime() const override { return primalSolution_.timeTrajectory_.back(); };

  void getPrimalSolution(scalar_t finalTime, PrimalSolution* primalSolutionPtr) const override {
    *primalSolutionPtr = primalSolution_;
  }

  const ocs2::PrimalSolution& getPrimalSolution() const {return primalSolution_;}

  const ProblemMetrics& getSolutionMetrics() const override { return problemMetrics_; }

  size_t getNumIterations() const override { return totalNumIterations_; }

  const OptimalControlProblem& getOptimalControlProblem() const override { 
    throw std::runtime_error("[ADMM Solver] getOptimalControlProblem() not available yet.");
  }

  const PerformanceIndex& getPerformanceIndeces() const override { return totalPerformanceIndeces_; };

  const std::vector<PerformanceIndex>& getIterationsLog() const override {
    throw std::runtime_error("[ADMM Solver] getIterationsLog() not available yet.");
  }

  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[ADMM Solver] getValueFunction() not available yet.");
  };

  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t time, const vector_t& state, const vector_t& input) override {
    throw std::runtime_error("[ADMM Solver] getHamiltonian() not available yet.");
  }

  vector_t getStateInputEqualityConstraintLagrangian(scalar_t time, const vector_t& state) const override {
    throw std::runtime_error("[ADMM Solver] getStateInputEqualityConstraintLagrangian() not available yet.");
  }

  MultiplierCollection getIntermediateDualSolution(scalar_t time) const override {
    throw std::runtime_error("[ADMM Solver] getIntermediateDualSolution() not available yet.");
  }

  SolverLogger getLogger() const {return logger_;}

  const AlternatingOptSettings& getSettings() const { return settings_; }

private:
  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override;

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const ControllerBase* externalControllerPtr) override {
    if (externalControllerPtr == nullptr) {
      runImpl(initTime, initState, finalTime);
    } else {
      throw std::runtime_error("[ADMM Solver] This solver does not support external controller!");
    }
  }

  void runImpl(scalar_t initTime, const vector_t& initState, scalar_t finalTime, const PrimalSolution& primalSolution) override {
    // Copy all except the controller
    primalSolution_.timeTrajectory_ = primalSolution.timeTrajectory_;
    primalSolution_.stateTrajectory_ = primalSolution.stateTrajectory_;
    primalSolution_.inputTrajectory_ = primalSolution.inputTrajectory_;
    primalSolution_.postEventIndices_ = primalSolution.postEventIndices_;
    primalSolution_.modeSchedule_ = primalSolution.modeSchedule_;
    runImpl(initTime, initState, finalTime);
  }

  virtual void AlternatingPreRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  /**
   * Performs one iteration of the alternating optimization.
   * @param[in] iter Current iteration number
   * @param[in] initTime Initial time of the trajectory
   * @param[in] initState Initial state of the trajectory
   * @param[in] finalTime Final time of the trajectory
   * @return The optimized solution for each robot after this iteration
   */
  virtual std::vector<PrimalSolution> iterImpl(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  virtual void AlternatingPostRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) = 0;

  virtual Convergence checkConvergence(int iteration) = 0;

  /** Constructs the primal solution based on the optimized states */
  virtual PrimalSolution toPrimalSolution(std::vector<PrimalSolution> optimizedStates) = 0;

protected:
  AlternatingOptSettings settings_;
  SolverLogger logger_;

  // store solution in ocs2 format
  ocs2::PrimalSolution primalSolution_;

  // Iteration performance log
  PerformanceIndex totalPerformanceIndeces_;

  // The ProblemMetrics associated to primalSolution_
  ProblemMetrics problemMetrics_;

  // Benchmarking
  size_t totalNumIterations_{0};
  int maxNumIterations_{0};

};

} // namespace multi_robot
} // namespace ocs2