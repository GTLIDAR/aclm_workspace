#include "ocs2_multi_robot/alternating_base/ConsensusADMMSolver.h"

#include <algorithm>
#include <atomic>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <sys/stat.h>
#include <sys/types.h>

#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_sqp/SqpSolver.h>

#include "ocs2_core/initialization/Initializer.h"
#include "ocs2_multi_robot/alternating_base/ThreadSafeReferenceManager.h"
#include "ocs2_multi_robot/common/AlternatingTargetTrajectories.h"
#include "ocs2_oc/oc_data/PrimalSolution.h"

namespace ocs2 {
namespace multi_robot {
namespace {
vector_t stackState(const std::vector<PrimalSolution>& robotSolutions,
    const PrimalSolution& cargoSolution,
    size_t robotStateDim, size_t cargoStateDim,
    size_t robotTimeIndex, size_t cargoTimeIndex) {
  vector_t stacked(robotSolutions.size() * robotStateDim + cargoStateDim);

  // Robots: all use the same filtered robotTimeIndex
  for (size_t r = 0; r < robotSolutions.size(); ++r) {
    const auto& traj = robotSolutions[r].stateTrajectory_;
    if (robotTimeIndex < traj.size()) {
      stacked.segment(r * robotStateDim, robotStateDim) = traj[robotTimeIndex];
    } else {
      throw std::runtime_error("[ConsensusADMMSolver] stackState(): robot state time index out of bounds");
    }
  }

  // Cargo: use its own filtered cargoTimeIndex
  if (cargoTimeIndex < cargoSolution.stateTrajectory_.size()) {
    stacked.tail(cargoStateDim) = cargoSolution.stateTrajectory_[cargoTimeIndex];
  } else {
    throw std::runtime_error("[ConsensusADMMSolver] stackState(): cargo state time index out of bounds");
  }

  return stacked;
}

vector_t stackInput(const std::vector<PrimalSolution>& robotSolutions,
    const PrimalSolution& cargoSolution,
    size_t robotInputDim, size_t cargoInputDim,
    size_t robotTimeIndex, size_t cargoTimeIndex) {
  vector_t stacked(robotSolutions.size() * robotInputDim + cargoInputDim);

  // Robots: if input not defined at this index, use zero (no back())
  for (size_t r = 0; r < robotSolutions.size(); ++r) {
    const auto& traj = robotSolutions[r].inputTrajectory_;
    if (robotTimeIndex < traj.size()) {
      stacked.segment(r * robotInputDim, robotInputDim) = traj[robotTimeIndex].segment(0, robotInputDim);
    } else {
      stacked.segment(r * robotInputDim, robotInputDim) = traj.back().segment(0, robotInputDim);
    }
  }

  // Cargo input
  if (cargoTimeIndex < cargoSolution.inputTrajectory_.size()) {
    stacked.tail(cargoInputDim) = cargoSolution.inputTrajectory_[cargoTimeIndex].segment(0, cargoInputDim);
  } else {
    stacked.tail(cargoInputDim) = cargoSolution.inputTrajectory_.back().segment(0, cargoInputDim);
  }

  return stacked;
}

// Helper to identify which time indices to keep (filter out PreEvent nodes; keep PostEvent)
std::vector<size_t> getIntermediateNodeIndices(const PrimalSolution& solution) {
  std::set<size_t> eventIndices;
  // PreEvent is at (postIdx - 1), PostEvent is at postIdx
  for (size_t postIdx : solution.postEventIndices_) {
    if (postIdx > 0) {
    eventIndices.insert(postIdx - 1);  // PreEvent
  }
  // PostEvent is kept (not inserted into eventIndices)
  }

  std::vector<size_t> indices;
  const size_t totalNodes = solution.timeTrajectory_.size();
  indices.reserve(totalNodes);
  for (size_t k = 0; k < totalNodes; ++k) {
    if (eventIndices.count(k) == 0) {
      indices.push_back(k);
    }
  }
  return indices;
}
}  // namespace

ConsensusADMMSolver::ConsensusADMMSolver(const AlternatingInterface& alternatingInterface)
    : AlternatingOptBase(alternatingInterface.getAlternatingSettings()), 
      alternatingInterface_(alternatingInterface),
      numRobots_(alternatingInterface_.getSingleRobotOptimalControlProblems().size()),
      threadPool_(std::max(static_cast<size_t>(1), numRobots_ + 1) - 1, 50) {
  cargoStateDim_ = CARGO_STATE_DIM;
  cargoInputDim_ = numRobots_ * ARM_CONTACT_DIM;
  robotStateDim_ = SINGLE_ROBOT_STATE_DIM;
  robotInputDim_ = ALTERNATING_SINGLE_ROBOT_INPUT_DIM;

  cargoOcpPtr_ = alternatingInterface_.getCargoOptimalControlProblem();
  robotOcpPtrs_ = alternatingInterface_.getSingleRobotOptimalControlProblems();

  cargoInitialState_.setZero(cargoStateDim_);

  robotSolutions_.resize(numRobots_);
  robotInitialStates_.assign(numRobots_, vector_t::Zero(robotStateDim_));

  rho_ = getSettings().rho;
  if (rho_.size() != static_cast<int>(numRobots_)) {
    std::cerr << "[ConsensusADMMSolver] WARNING: rho_ size (" << rho_.size()
              << ") does not match numRobots_ (" << numRobots_
              << "). Resizing using the first value.\n";
    const scalar_t value = (rho_.size() > 0) ? rho_(0) : 1.0;
    rho_ = vector_t::Constant(numRobots_, value);
  }

  // Create thread-safe wrapper for reference manager to allow parallel solver execution
  threadSafeReferenceManager_ = std::make_shared<ThreadSafeReferenceManager>(
      alternatingInterface_.getReferenceManagerPtr());
  
  this->setReferenceManager(threadSafeReferenceManager_);

  cargoSolver_ = std::make_unique<SqpSolver>(alternatingInterface_.getSqpSettings(), *cargoOcpPtr_,
                                             alternatingInterface_.getCargoInitializer());
  cargoSolver_->setReferenceManager(threadSafeReferenceManager_);

  robotSolvers_.reserve(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    auto solver = std::make_unique<SqpSolver>(alternatingInterface_.getSqpSettings(), *robotOcpPtrs_[robot],
                                              alternatingInterface_.getSingleRobotInitializer(robot));
    solver->setReferenceManager(threadSafeReferenceManager_);
    robotSolvers_.push_back(std::move(solver));
  }

  initializerPtr_.reset(alternatingInterface_.getTwoQuadWCargoInitializer().clone());
}

ConsensusADMMSolver::~ConsensusADMMSolver() {
  // Print benchmarking information (only if printSolverStatistics is enabled)
  if (this->getSettings().printSolverStatistics) {
    const auto cargoTotal = cargoSubproblemTimer_.getTotalInMilliseconds();
    const auto robotTotal = robotSubproblemsTimer_.getTotalInMilliseconds();
    const auto dualTotal = dualUpdateTimer_.getTotalInMilliseconds();
    const auto benchmarkTotal = cargoTotal + robotTotal + dualTotal;
    
    if (benchmarkTotal > 0.0) {
      const scalar_t inPercent = 100.0;
      std::cout << "\n========================================\n";
      std::cout << "ADMM Benchmarking Summary\n";
      std::cout << "========================================\n";
      std::cout << "The benchmarking is computed over " << totalAdmmIterations_ << " total ADMM iterations.\n";
      std::cout << "ADMM Benchmarking\t   :\tAverage time [ms]   (% of total runtime)\n";
      std::cout << "\tCargo Subproblem   :\t" << std::setw(10) << std::fixed << std::setprecision(2) 
                << cargoSubproblemTimer_.getAverageInMilliseconds() << " [ms] \t(" 
                << (cargoTotal / benchmarkTotal * inPercent) << "%)\n";
      std::cout << "\tRobot Subproblems  :\t" << std::setw(10) << std::fixed << std::setprecision(2) 
                << robotSubproblemsTimer_.getAverageInMilliseconds() << " [ms] \t(" 
                << (robotTotal / benchmarkTotal * inPercent) << "%)\n";
      std::cout << "\tDual Update        :\t" << std::setw(10) << std::fixed << std::setprecision(2) 
                << dualUpdateTimer_.getAverageInMilliseconds() << " [ms] \t(" 
                << (dualTotal / benchmarkTotal * inPercent) << "%)\n";
      std::cout << "========================================\n\n";
    }
  }

  // Write MPC residual history to file (if logging is enabled)
  if (this->getSettings().logMpcResiduals && !mpcResidualHistory_.empty()) {
    const std::string& filePath = this->getSettings().mpcResidualLogPath;
    
    // Extract directory from file path
    size_t lastSlash = filePath.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
      std::string dataDir = filePath.substr(0, lastSlash);
      struct stat info;
      if (stat(dataDir.c_str(), &info) != 0) {
        // Directory doesn't exist, create it
        #ifdef _WIN32
          _mkdir(dataDir.c_str());
        #else
          mkdir(dataDir.c_str(), 0755);
        #endif
      }
    }
    
    // Write to file
    std::ofstream outFile(filePath);
    
    if (outFile.is_open()) {
      // Write header
      outFile << "# MPC Residual History\n";
      outFile << "# Format: initTime totalPrimalResidual perConstraintResidual_0 perConstraintResidual_1 ...\n";
      outFile << "# Each line represents one MPC solve\n";
      
      // Write data line by line
      for (const auto& residual : mpcResidualHistory_) {
        outFile << std::scientific << std::setprecision(10) << residual.initTime << " ";
        outFile << std::scientific << std::setprecision(10) << residual.totalPrimalResidual << " ";
        
        for (size_t i = 0; i < residual.perConstraintPrimalResidual.size(); ++i) {
          if (i > 0) outFile << " ";
          outFile << std::scientific << std::setprecision(10) << residual.perConstraintPrimalResidual[i];
        }
        outFile << "\n";
      }
      
      outFile.close();
      std::cout << "[ConsensusADMMSolver] Wrote " << mpcResidualHistory_.size() 
                << " MPC residual records to " << filePath << "\n";
    } else {
      std::cerr << "[ConsensusADMMSolver] Warning: Could not open file for writing: " << filePath << "\n";
    }
  }
}

void ConsensusADMMSolver::reset() {
  AlternatingOptBase::reset();

  cargoSolver_->reset();
  for (auto& robotSolver : robotSolvers_) {
    robotSolver->reset();
  }

  previousSolution_.clear();
  previousSolutionForDual_.clear();
  iterationStatusHistory_.clear();
  totalAdmmIterations_ = 0;
  mpcResidualHistory_.clear();
}

void ConsensusADMMSolver::AlternatingPreRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  // Compute the initial states for cargo and robots
  cargoInitialState_ = initState.tail(cargoStateDim_);

  for (size_t robot = 0; robot < numRobots_; ++robot) {
    robotInitialStates_[robot] = initState.segment(robot * robotStateDim_, robotStateDim_);
  }

  // Update reference manager with FULL state before subproblems run
  // This ensures perceptive updates (e.g., convex region selector) receive complete state
  // Individual SqpSolver instances will call preSolverRun() later, but ThreadSafeReferenceManager
  // will skip those calls since the horizon (initTime, finalTime) is the same
  this->getReferenceManager().preSolverRun(initTime, finalTime, initState);

  // Initialize previous solution from reference manager if cold started.
  // This is used for both pre-computations (previousSolution_) and the very
  // first dual-residual computation (previousSolutionForDual_).
  if (previousSolution_.timeTrajectory.empty()) {
    previousSolution_ = AlternatingTargetTrajectories(this->getReferenceManager().getTargetTrajectories(),
                                                    numRobots_ * ARM_CONTACT_DIM);
    previousSolutionForDual_ = previousSolution_;
  }
  updatePreComputations();
}

std::vector<PrimalSolution> ConsensusADMMSolver::iterImpl(int iter, scalar_t initTime, const vector_t& initState,
                                                          scalar_t finalTime) {
  solverStatus_.clear();

  // Primal updates
  const bool useParallel = this->getSettings().parallelUpdate;
  const bool isConsensus = (this->getSettings().updateType == ADMMUpdateType::CONSENSUS);
  
  if (useParallel) {
    // Parallel execution
    if (isConsensus) {
      // Consensus: cargo and all robots run in parallel
      runCargoAndRobotSubproblemsParallel(initTime, finalTime);
    } else {
      // Sequential: only robots run in parallel, cargo runs first sequentially
      runCargoSubproblem(initTime, finalTime);
      if (iter == 0 && robotSolutions_[0].timeTrajectory_.empty()) {
        updatePreviousSolutionWithRefAndCargo();
      } else {
        updatePreviousSolution();
      }
      runRobotSubproblemsParallel(initTime, finalTime);
    }
  } else {
    // Sequential execution (original behavior)
    runCargoSubproblem(initTime, finalTime);

    if (this->getSettings().updateType == ADMMUpdateType::SEQUENTIAL) {
      if (iter == 0 && robotSolutions_[0].timeTrajectory_.empty()) {
        updatePreviousSolutionWithRefAndCargo();
      } else {
        updatePreviousSolution();
      }
    }
    runRobotSubproblems(initTime, finalTime);
  }
  
  solverStatus_.subproblemPerformanceIndices.push_back(cargoSolver_->getPerformanceIndeces());
  for (const auto& robotSolver : robotSolvers_) {
    solverStatus_.subproblemPerformanceIndices.push_back(robotSolver->getPerformanceIndeces());
  }

  // Dual updates
  updateDualVariables();
  updatePreviousSolution();

  // Store the fully updated previousSolution_ (which now corresponds to the
  // current ADMM iterate) for use in the *next* iteration's dual residual
  // computation. In particular, previousSolutionForDual_ is never updated
  // mid-iteration, so it always represents the last complete ADMM iterate.
  previousSolutionForDual_ = previousSolution_;

  // Store status for this iteration
  iterationStatusHistory_.push_back(solverStatus_);

  std::vector<PrimalSolution> solutions;
  solutions.reserve(numRobots_ + 1);
  for (const auto& robotSolution : robotSolutions_) {
    solutions.push_back(robotSolution);
  }
  solutions.push_back(cargoSolution_);
  return solutions;
}

void ConsensusADMMSolver::AlternatingPostRun(int iter, scalar_t initTime, const vector_t& initState, scalar_t finalTime) {
  // Print status summary for all iterations
  if (this->getSettings().printSolverStatus && !iterationStatusHistory_.empty()) {
    std::cout << "\n========================================\n";
    std::cout << "ADMM Solver Status Summary\n";
    std::cout << "========================================\n";
    // Print current rho values (per consensus constraint / robot)
    std::cout << "Current rho per robot: [";
    for (size_t i = 0; i < rho_.size(); ++i) {
      if (i > 0) {
        std::cout << ", ";
      }
      std::cout << std::scientific << std::setprecision(4) << rho_[i];
    }
    std::cout << "]\n";
    std::cout << "----------------------------------------\n";
    std::cout << "Iteration | Primal Residual | Dual Residual | Cargo Cost | Robot Costs\n";
    std::cout << "----------|------------------|--------------|------------|-------------|\n";
    
    for (size_t iter = 0; iter < iterationStatusHistory_.size(); ++iter) {
      const auto& status = iterationStatusHistory_[iter];
      std::cout << std::setw(9) << iter << " | ";
      std::cout << std::setw(16) << std::scientific << std::setprecision(6) << status.totalPrimalResidualSSE << " | ";
      std::cout << std::setw(12) << std::scientific << std::setprecision(6) << status.totalDualResidualSSE << " | ";
      
      // Print cargo cost (first performance index)
      if (!status.subproblemPerformanceIndices.empty()) {
        std::cout << std::setw(10) << std::scientific << std::setprecision(6) 
                  << status.subproblemPerformanceIndices[0].cost << " | ";
        
        // Print robot costs
        if (status.subproblemPerformanceIndices.size() > 1) {
          std::cout << "[";
          for (size_t i = 1; i < status.subproblemPerformanceIndices.size(); ++i) {
            if (i > 1) std::cout << ", ";
            std::cout << std::scientific << std::setprecision(4) 
                      << status.subproblemPerformanceIndices[i].cost;
          }
          std::cout << "]";
        } else {
          std::cout << std::setw(11) << "N/A" << " | ";
          std::cout << std::setw(19) << "N/A";
        }
      } else {
        std::cout << std::setw(10) << "N/A" << " | ";
        std::cout << std::setw(11) << "N/A" << " | ";
        std::cout << std::setw(19) << "N/A";
      }
      std::cout << "\n";
    }
    
    std::cout << "========================================\n\n";
    
    // Print detailed residual summary for each constraint
    std::cout << "========================================\n";
    std::cout << "Per-Constraint Residual Summary (Primal and Dual)\n";
    std::cout << "========================================\n";
    std::cout << "Iteration";
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      std::cout << " | Robot " << robot << " Primal | Robot " << robot << " Dual";
    }
    std::cout << "\n";
    std::cout << "---------";
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      std::cout << " | ------------------- | ------------------";
    }
    std::cout << "\n";
    
    for (size_t iter = 0; iter < iterationStatusHistory_.size(); ++iter) {
      const auto& status = iterationStatusHistory_[iter];
      std::cout << std::setw(9) << iter;
      for (size_t robot = 0; robot < numRobots_; ++robot) {
        if (robot < status.perConstraintResidualSSE.size()) {
          std::cout << " | " << std::setw(18) << std::scientific << std::setprecision(6) 
                    << status.perConstraintResidualSSE[robot];
        } else {
          std::cout << " | " << std::setw(18) << "N/A";
        }

        if (robot < status.perConstraintDualResidualSSE.size()) {
          std::cout << " | " << std::setw(18) << std::scientific << std::setprecision(6)
                    << status.perConstraintDualResidualSSE[robot];
        } else {
          std::cout << " | " << std::setw(18) << "N/A";
        }
      }
      std::cout << "\n";
    }
    std::cout << "========================================\n\n";
  }

  // If only running one ADMM iteration or costs are too large, reset the dual variables to zero
  bool resetDualVariables = false;

  // 1) If configured and we effectively ran a single ADMM iteration (iter == 1)
  if (this->getSettings().resetDualWhenSingleIteration &&
      iter == 1) {
    resetDualVariables = true;
  }

  // 2) If any subproblem cost is above the configured threshold
  if (!resetDualVariables) {
    const scalar_t costThreshold = this->getSettings().dualResetCostThreshold;
    const auto& lastStatus = iterationStatusHistory_.back();
    for (const auto& perf : lastStatus.subproblemPerformanceIndices) {
      if (perf.cost > costThreshold) {
        resetDualVariables = true;
        break;
      }
    }
  }

  if (resetDualVariables) {
    for (auto& dual : dualTrajectory_) {
      dual.setZero();
    }

    for (auto& dual : previousSolution_.dualVariableTrajectory) {
      dual.setZero();
    }
  }

  // Increment total ADMM iterations counter
  totalAdmmIterations_ += iterationStatusHistory_.size();

  // Record final residual data for this MPC solve
  if (!iterationStatusHistory_.empty()) {
    const auto& finalStatus = iterationStatusHistory_.back();
    MpcSolveResidual mpcResidual;
    mpcResidual.initTime = initTime;
    mpcResidual.totalPrimalResidual = finalStatus.totalPrimalResidualSSE;
    mpcResidual.perConstraintPrimalResidual = finalStatus.perConstraintResidualSSE;
    mpcResidualHistory_.push_back(mpcResidual);
  }

  iterationStatusHistory_.clear();
}

Convergence ConsensusADMMSolver::checkConvergence(int iteration) {
  if ((iteration + 1) >= this->getSettings().max_iter) {
    return Convergence::ITERATIONS;
  } else if (solverStatus_.totalPrimalResidualSSE < this->getSettings().primalTolerance) {
    return Convergence::PRIMAL;
  } else {
    return Convergence::FALSE;
  }
}

PrimalSolution ConsensusADMMSolver::toPrimalSolution(std::vector<PrimalSolution> optimizedStates) {
  PrimalSolution combined;
  if (optimizedStates.size() < numRobots_ + 1) {
    return combined;
  }

  const auto& cargo = optimizedStates.back();
  combined.timeTrajectory_ = cargo.timeTrajectory_;
  combined.postEventIndices_ = cargo.postEventIndices_;
  combined.modeSchedule_ = cargo.modeSchedule_;

  const size_t numTimeSteps = cargo.stateTrajectory_.size();
  combined.stateTrajectory_.resize(numTimeSteps);
  combined.inputTrajectory_.resize(numTimeSteps);

  for (size_t t = 0; t < numTimeSteps; ++t) {
    combined.stateTrajectory_[t] = stackState(robotSolutions_, cargo, robotStateDim_, cargoStateDim_, t, t);
    combined.inputTrajectory_[t] = stackInput(robotSolutions_, cargo, SINGLE_ROBOT_INPUT_DIM, cargoInputDim_, t, t);
  }

  // Create feedforward controller from input trajectory (ADMM doesn't compute feedback gains)
  combined.controllerPtr_.reset(new FeedforwardController(combined.timeTrajectory_, combined.inputTrajectory_));

  return combined;
}

void ConsensusADMMSolver::runCargoSubproblem(scalar_t initTime, scalar_t finalTime) {
  if (this->getSettings().printSolverStatus) {
    std::cerr << "  Solving cargo subproblem...\n";
  }
  if (this->getSettings().printSolverStatistics) {
    cargoSubproblemTimer_.startTimer();
  }
  // cargoSolver_->reset();
  cargoSolver_->run(initTime, cargoInitialState_, finalTime);
  cargoSolver_->getPrimalSolution(finalTime, &cargoSolution_);
  if (this->getSettings().printSolverStatistics) {
    cargoSubproblemTimer_.endTimer();
  }
}

void ConsensusADMMSolver::runRobotSubproblems(scalar_t initTime, scalar_t finalTime) {
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.startTimer();
  }
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    if (this->getSettings().printSolverStatus) {
      std::cerr << "  Solving robot " << robot << " subproblem...\n";
    }

    robotSolvers_[robot]->run(initTime, robotInitialStates_[robot], finalTime);
    robotSolvers_[robot]->getPrimalSolution(finalTime, &robotSolutions_[robot]);
  }
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.endTimer();
  }
}

void ConsensusADMMSolver::runRobotSubproblemsParallel(scalar_t initTime, scalar_t finalTime) {
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.startTimer();
  }
  // Shared atomic counter for all threads
  std::atomic_int robotIndex{0};
  
  auto parallelTask = [&](int workerId) {
    int robot = robotIndex++;
    
    while (robot < static_cast<int>(numRobots_)) {
      if (this->getSettings().printSolverStatus) {
        std::cerr << "  Solving robot " << robot << " subproblem (parallel)...\n";
      }
      
      // ThreadSafeReferenceManager protects preSolverRun() calls with a mutex
      // This allows parallel execution while preventing race conditions
      robotSolvers_[robot]->run(initTime, robotInitialStates_[robot], finalTime);
      
      // getPrimalSolution writes to different solution objects, so should be safe
      robotSolvers_[robot]->getPrimalSolution(finalTime, &robotSolutions_[robot]);
      
      robot = robotIndex++;
    }
  };
  
  threadPool_.runParallel(std::move(parallelTask), numRobots_);
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.endTimer();
  }
}

void ConsensusADMMSolver::runCargoAndRobotSubproblemsParallel(scalar_t initTime, scalar_t finalTime) {
  // Run cargo and all robots in parallel
  // Note: In consensus parallel mode, we time the entire parallel execution as robot solving time
  // (since cargo is included in the parallel execution)
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.startTimer();
  }
  const size_t totalTasks = numRobots_ + 1;  // +1 for cargo
  
  // Shared atomic counter for all threads
  std::atomic_int taskIndex{0};
  
  auto parallelTask = [&](int workerId) {
    int task = taskIndex++;
    
    while (task < static_cast<int>(totalTasks)) {
      if (task == 0) {
        // Task 0: cargo
        if (this->getSettings().printSolverStatus) {
          std::cerr << "  Solving cargo subproblem (parallel)...\n";
        }
        // ThreadSafeReferenceManager protects preSolverRun() calls with a mutex
        // This allows parallel execution while preventing race conditions
        cargoSolver_->run(initTime, cargoInitialState_, finalTime);
        cargoSolver_->getPrimalSolution(finalTime, &cargoSolution_);
      } else {
        // Task 1, 2, ...: robots
        const size_t robot = static_cast<size_t>(task - 1);
        if (this->getSettings().printSolverStatus) {
          std::cerr << "  Solving robot " << robot << " subproblem (parallel)...\n";
        }
        // ThreadSafeReferenceManager protects preSolverRun() calls with a mutex
        // This allows parallel execution while preventing race conditions
        robotSolvers_[robot]->run(initTime, robotInitialStates_[robot], finalTime);
        robotSolvers_[robot]->getPrimalSolution(finalTime, &robotSolutions_[robot]);
      }
      
      task = taskIndex++;
    }
  };
  
  threadPool_.runParallel(std::move(parallelTask), totalTasks);
  if (this->getSettings().printSolverStatistics) {
    robotSubproblemsTimer_.endTimer();
  }
}

void ConsensusADMMSolver::updatePreComputations() {
  const auto updateInstance = [&](AlternatingPreComputation* preComp, const std::string& label) {
    // if (preComp == nullptr) {
    //   std::cerr << "[ConsensusADMMSolver] ERROR: " << label << " preComputation is null\n";
    //   return;
    // }
    // std::cerr << "[ConsensusADMMSolver] Updating precomputation (" << label << ") at " << preComp
    //           << " handleCount=" << preComp->getHandleCount() << " numDualVars=" << preComp->getNumDualVariables()
    //           << '\n';
    preComp->updatePreviousSolution(previousSolution_);
  };

  const auto updateOcp = [&](OptimalControlProblem& ocp, const std::string& label) {
    // Update OCP pre-computation
    if (ocp.preComputationPtr) {
      if (auto* preComp = dynamic_cast<AlternatingPreComputation*>(ocp.preComputationPtr.get())) {
        updateInstance(preComp, label + " ocp");
      } else {
        std::cerr << "[ConsensusADMMSolver] ERROR: " << label
                  << " preComputationPtr is not AlternatingPreComputation ("
                  << typeid(*ocp.preComputationPtr).name() << ")\n";
      }
    } else {
      std::cerr << "[ConsensusADMMSolver] ERROR: " << label << " OCP has null preComputationPtr\n";
    }

    // Update dynamics pre-computation (stored separately)
    if (ocp.dynamicsPtr) {
      const auto& dynPreCompBase = ocp.dynamicsPtr->getPreComputation();
      if (const auto* dynPreCompConst = dynamic_cast<const AlternatingPreComputation*>(&dynPreCompBase)) {
        auto* dynPreComp = const_cast<AlternatingPreComputation*>(dynPreCompConst);
        updateInstance(dynPreComp, label + " dynamics");
      } 
      // else {
      //   std::cerr << "[ConsensusADMMSolver] WARN: " << label
      //             << " dynamics preComputation is not AlternatingPreComputation ("
      //             << typeid(dynPreCompBase).name() << ")\n";
      // }
    } else {
      std::cerr << "[ConsensusADMMSolver] ERROR: " << label << " OCP has null dynamicsPtr\n";
    }
  };

  const auto updateSolverOcps = [&](SqpSolver& solver, const std::string& solverLabel) {
    auto& ocps = solver.getOptimalControlProblems();
    for (size_t idx = 0; idx < ocps.size(); ++idx) {
      updateOcp(ocps[idx], solverLabel + "[ocp_" + std::to_string(idx) + "]");
    }
  };

  if (cargoSolver_) {
    updateSolverOcps(*cargoSolver_, "solver cargo");
  } else {
    std::cerr << "[ConsensusADMMSolver] ERROR: cargoSolver_ is null\n";
  }
  for (size_t robot = 0; robot < robotSolvers_.size(); ++robot) {
    const auto& solver = robotSolvers_[robot];
    if (solver) {
      updateSolverOcps(*solver, "solver robot" + std::to_string(robot));
    } else {
      std::cerr << "[ConsensusADMMSolver] ERROR: robotSolvers_[" << robot << "] is null\n";
    }
  }
}

void ConsensusADMMSolver::updateDualVariables() {
  if (this->getSettings().printSolverStatistics) {
    dualUpdateTimer_.startTimer();
  }

  // Filter out PreEvent/PostEvent nodes - only work with intermediate nodes where inputs exist
  const std::vector<size_t> intermediateIndices = getIntermediateNodeIndices(cargoSolution_);
  const size_t filteredTimeSteps = intermediateIndices.size();
  
  if (filteredTimeSteps == 0) {
    if (this->getSettings().printSolverStatistics) {
      dualUpdateTimer_.endTimer();
    }
    return;
  }

  // Build filtered dual and residual trajectories (only intermediate nodes)
  if (dualTrajectory_.empty()) {
    dualTrajectory_ = vector_array_t(filteredTimeSteps, vector_t::Zero(numRobots_ * ARM_CONTACT_DIM));
  }

  residualTrajectory_.clear();
  residualTrajectory_.resize(filteredTimeSteps, vector_t::Zero(numRobots_ * ARM_CONTACT_DIM));

  // Primal residual and scaled dual update:
  // Constraint: u_i^robot + u_i^cargo = 0  (per robot i, per handle dimension)
  // Primal residual: r_i = u_i^robot + u_i^cargo
  // Scaled dual update (standard ADMM): y_i^{k+1} = y_i^k + r_i^{k+1}
  const auto& cargoTrajectory = cargoSolution_.inputTrajectory_;
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t offset = robot * ARM_CONTACT_DIM;
    const auto& robotTrajectory = robotSolutions_[robot].inputTrajectory_;
    
    for (size_t i = 0; i < filteredTimeSteps; ++i) {
      const size_t k = intermediateIndices[i];
      
      // Get inputs at this intermediate node
      vector_t robotArmForce = vector_t::Zero(ARM_CONTACT_DIM);
      if (k < robotTrajectory.size()) {
        const auto& robotInput = robotTrajectory[k];
        robotArmForce = robotInput.segment(SINGLE_ROBOT_INPUT_DIM, ARM_CONTACT_DIM);
      }
      vector_t cargoArmForce = vector_t::Zero(ARM_CONTACT_DIM);
      if (k < cargoTrajectory.size()) {
        const auto& cargoInput = cargoTrajectory[k];
        cargoArmForce = cargoInput.segment(offset, ARM_CONTACT_DIM);
      }
      vector_t residual = robotArmForce + cargoArmForce;
      
      residualTrajectory_[i].segment(offset, ARM_CONTACT_DIM) = residual;

      if (i < dualTrajectory_.size() && dualTrajectory_.size() == filteredTimeSteps) {
        dualTrajectory_[i].segment(offset, ARM_CONTACT_DIM) += residual;
      } else {
        std::cerr << "[ConsensusADMMSolver] updateDualVariables(): dualTrajectory_ size mismatch. Resizing" << std::endl;
        std::cerr << "  Expected size: " << filteredTimeSteps << ", Got size: " << dualTrajectory_.size() << std::endl;
        throw std::runtime_error("[ConsensusADMMSolver] updateDualVariables(): dualTrajectory_ size mismatch");
      }
    }
  }

  // Compute a time-scaled residual metric, consistent with SQP's constraint SSE definition.
  // For uniform time grids with step dt, SQP uses dt * sum_k ||g_k||^2. Here we approximate dt
  // using the average step over the cargo time trajectory.
  scalar_t dt = alternatingInterface_.getSqpSettings().dt;

  const scalar_t totalResidualSSE = getEqConstraintsSSE(residualTrajectory_);
  solverStatus_.totalPrimalResidualSSE = std::sqrt(dt * totalResidualSSE);
  
  // Compute per-constraint (per-robot) residuals
  solverStatus_.perConstraintResidualSSE.resize(numRobots_);
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    const size_t offset = robot * ARM_CONTACT_DIM;
    vector_array_t robotResidualTrajectory(filteredTimeSteps);
    for (size_t i = 0; i < filteredTimeSteps; ++i) {
      robotResidualTrajectory[i] = residualTrajectory_[i].segment(offset, ARM_CONTACT_DIM);
    }
    const scalar_t robotResidualSSE = getEqConstraintsSSE(robotResidualTrajectory);
    solverStatus_.perConstraintResidualSSE[robot] = std::sqrt(dt * robotResidualSSE);
  }

  // ---------------------------------------------------------------------------
  // Dual residual metrics:
  //
  // Standard ADMM (scaled form) for constraint x + z = 0:
  //   r^{k+1} = x^{k+1} + z^{k+1}
  //   s^{k+1} = ρ (z^{k+1} - z^{k})
  //
  // Here we interpret:
  //   x := u_i^robot,  z := u_i^cargo
  // so for each robot i, handle dimension d, and time step k:
  //   s_i^{k+1}(t_k) = ρ_i ∘ (u_{i,\text{cargo}}^{k+1}(t_k) - u_{i,\text{cargo}}^{k}(t_k))
  //
  // previousSolution_ holds the stacked input trajectory from the previous ADMM
  // iteration, so we can reconstruct u_{i,\text{cargo}}^{k}(t_k) from it.
  // ---------------------------------------------------------------------------

  // If we do not have a valid previousSolutionForDual_ (e.g., during the very first
  // iteration), skip dual residual metrics. Note: previousSolutionForDual_ now only
  // contains intermediate nodes (PreEvent/PostEvent filtered out).
  if (!previousSolutionForDual_.inputTrajectory.empty()) {
    // Build mapping from full trajectory index to filtered trajectory index
    // (previousSolutionForDual_ only has intermediate nodes)
    std::map<size_t, size_t> fullToFilteredIndex;
    const std::vector<size_t> intermediateIndices = getIntermediateNodeIndices(cargoSolution_);
    for (size_t i = 0; i < intermediateIndices.size(); ++i) {
      fullToFilteredIndex[intermediateIndices[i]] = i;
    }

    scalar_t totalDualSSE = 0.0;
    std::vector<scalar_t> perRobotDualSSE(numRobots_, 0.0);

    for (size_t robot = 0; robot < numRobots_; ++robot) {
      const size_t offset = robot * ARM_CONTACT_DIM;
      // rho_ is configured per consensus constraint (per robot). Use the
      // scalar rho for this robot across all ARM_CONTACT_DIM dimensions.
      scalar_t rhoScalar = rho_[robot];

      // Only compute dual residuals at intermediate nodes (where inputs exist)
      for (size_t k : intermediateIndices) {
        // Current cargo arm force for this robot at time step k
        vector_t cargoArmForceNow = vector_t::Zero(ARM_CONTACT_DIM);
        if (k < cargoTrajectory.size()) {
          cargoArmForceNow = cargoTrajectory[k].segment(offset, ARM_CONTACT_DIM);
        }

        // Previous stacked input: [robots ; cargo] from the last ADMM iteration
        // Use the filtered index to access previousSolutionForDual_
        const size_t filteredIdx = fullToFilteredIndex.at(k);
        if (filteredIdx >= previousSolutionForDual_.inputTrajectory.size()) {
          continue;  // Safety check
        }
        const auto& prevStackedInput = previousSolutionForDual_.inputTrajectory[filteredIdx];
        const vector_t prevCargoInput = prevStackedInput.tail(cargoInputDim_);
        const vector_t prevCargoArmForce = prevCargoInput.segment(offset, ARM_CONTACT_DIM);

        // Dual residual s_i^{k+1} = ρ_i * (u_cargo^{k+1} - u_cargo^{k})
        const vector_t deltaCargoArmForce = cargoArmForceNow - prevCargoArmForce;
        const vector_t dualRes = rhoScalar * deltaCargoArmForce;

        const scalar_t dualNormSq = dualRes.squaredNorm();
        totalDualSSE += dualNormSq;
        perRobotDualSSE[robot] += dualNormSq;
      }
    }

    solverStatus_.totalDualResidualSSE = std::sqrt(dt * totalDualSSE);
    solverStatus_.perConstraintDualResidualSSE.resize(numRobots_);
    for (size_t robot = 0; robot < numRobots_; ++robot) {
      solverStatus_.perConstraintDualResidualSSE[robot] = std::sqrt(dt * perRobotDualSSE[robot]);
    }

    // -----------------------------------------------------------------------
    // Adaptive rho update (per consensus constraint / per robot)
    //
    // Using the rule (up to a consistent scaling of the residual metrics):
    //   if ||r_i|| > μ ||d_i||      ->  ρ_i^{k+1} = τ_incr ρ_i^k
    //   if ||d_i|| > μ ||r_i||      ->  ρ_i^{k+1} = ρ_i^k / τ_decr
    //   otherwise                   ->  ρ_i^{k+1} = ρ_i^k
    //
    // where r_i and d_i are the per-robot primal and dual residuals.
    // Here we use the already-computed perConstraintResidualSSE and
    // perConstraintDualResidualSSE metrics; since both are scaled in the same
    // way (sqrt(dt * sum ||·||^2)), comparing their squares preserves the
    // intended inequality.
    // -----------------------------------------------------------------------
    if (this->getSettings().enableAdaptiveRho) {
      const scalar_t mu      = this->getSettings().adaptiveRhoMu;
      const scalar_t tauIncr = this->getSettings().adaptiveRhoTauIncr;
      const scalar_t tauDecr = this->getSettings().adaptiveRhoTauDecr;

      for (size_t robot = 0; robot < numRobots_; ++robot) {
        const scalar_t primalMetric = solverStatus_.perConstraintResidualSSE[robot];
        const scalar_t dualMetric   = solverStatus_.perConstraintDualResidualSSE[robot];

        scalar_t rhoVal = rho_[robot];
        if (primalMetric > mu * dualMetric) {
          rhoVal *= tauIncr;
        } else if (dualMetric > mu * primalMetric) {
          rhoVal /= tauDecr;
        }
        rho_[robot] = rhoVal;
      }
    }
  } else {
    // Fallback: no valid previousSolution_ yet (e.g., first iteration). Set dual
    // metrics to zero so they do not carry stale values.
    solverStatus_.totalDualResidualSSE = 0.0;
    solverStatus_.perConstraintDualResidualSSE.assign(numRobots_, 0.0);
  }
  if (this->getSettings().printSolverStatistics) {
    dualUpdateTimer_.endTimer();
  }
}

void ConsensusADMMSolver::updatePreviousSolution() {
  const auto& cargoTimeTrajectory = cargoSolution_.timeTrajectory_;
  if (cargoTimeTrajectory.empty()) {
    throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolution(): cargo time trajectory is empty");
  }

  // Check if time steps match between raw cargo and robot solutions (case 2: subsequent ADMM iterations)
  // Only check: size, initial time, and end time (much faster than filtering and comparing all values)
  const scalar_t dt = alternatingInterface_.getSqpSettings().dt;
  const scalar_t timeTol = 0.1 * dt;  // Tight tolerance for matching time steps
  bool timeStepsMatch = true;
  
  const size_t cargoSize = cargoTimeTrajectory.size();
  const scalar_t cargoInitTime = cargoTimeTrajectory.front();
  const scalar_t cargoEndTime = cargoTimeTrajectory.back();
  
  // Check if all robots have matching size, init time, and end time with cargo
  for (size_t robot = 0; robot < numRobots_ && timeStepsMatch; ++robot) {
    const auto& robotTimeTraj = robotSolutions_[robot].timeTrajectory_;
    if (robotTimeTraj.empty()) {
      throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolution(): robot " + 
                               std::to_string(robot) + " trajectory is empty");
    }
    
    // Check size
    if (robotTimeTraj.size() != cargoSize) {
      timeStepsMatch = false;
      break;
    }
    
    // Check initial time
    if (std::abs(robotTimeTraj.front() - cargoInitTime) > timeTol) {
      timeStepsMatch = false;
      break;
    }
    
    // Check end time
    if (std::abs(robotTimeTraj.back() - cargoEndTime) > timeTol) {
      timeStepsMatch = false;
      break;
    }
  }

  // Filter cargo trajectory to get intermediate nodes (remove PreEvent/PostEvent)
  const std::vector<size_t> cargoIdx = getIntermediateNodeIndices(cargoSolution_);
  const size_t Nf = cargoIdx.size();
  if (Nf == 0) {
    throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolution(): no intermediate nodes found in cargo trajectory");
  }

  // If time steps match, we can use cargo indices directly for robots (same event pattern)
  // No need to filter robot trajectories separately

  // Build filtered trajectories
  scalar_array_t filteredTimeTrajectory(Nf);
  vector_array_t filteredStateTrajectory(Nf);
  vector_array_t filteredInputTrajectory(Nf);
  vector_array_t filteredDualTrajectory(Nf);

  // Ensure dualTrajectory_ is properly sized
  if (dualTrajectory_.size() != Nf) {
    dualTrajectory_.resize(Nf, vector_t::Zero(numRobots_ * ARM_CONTACT_DIM));
  }

  if (timeStepsMatch) {
    // Case 2: Time steps match - use direct indexing (no interpolation needed)
    for (size_t i = 0; i < Nf; ++i) {
      const size_t kc = cargoIdx[i];
      filteredTimeTrajectory[i] = cargoTimeTrajectory[kc];

      // Get cargo state/input directly
      vector_t cargoState = cargoSolution_.stateTrajectory_[kc];
      vector_t cargoInput = cargoSolution_.inputTrajectory_[kc];

      // Get robot states/inputs directly (no interpolation)
      vector_t stackedState(numRobots_ * robotStateDim_ + cargoStateDim_);
      vector_t stackedInput(numRobots_ * robotInputDim_ + cargoInputDim_);

      for (size_t robot = 0; robot < numRobots_; ++robot) {
        // Use cargo indices directly since time steps match (same event pattern)
        const size_t kr = cargoIdx[i];
        stackedState.segment(robot * robotStateDim_, robotStateDim_) = robotSolutions_[robot].stateTrajectory_[kr];
        stackedInput.segment(robot * robotInputDim_, robotInputDim_) = 
            robotSolutions_[robot].inputTrajectory_[kr].segment(0, robotInputDim_);
      }

      // Append cargo state/input
      stackedState.tail(cargoStateDim_) = cargoState;
      stackedInput.tail(cargoInputDim_) = cargoInput.segment(0, cargoInputDim_);

      filteredStateTrajectory[i] = stackedState;
      filteredInputTrajectory[i] = stackedInput;
      filteredDualTrajectory[i] = dualTrajectory_[i];
    }
  } else {
    // Case 1: Mixed time stamps - use interpolation
    for (size_t i = 0; i < Nf; ++i) {
      const size_t kc = cargoIdx[i];
      const scalar_t t = cargoTimeTrajectory[kc];
      filteredTimeTrajectory[i] = t;

      // Get cargo state/input directly from cargo solution
      vector_t cargoState = cargoSolution_.stateTrajectory_[kc];
      vector_t cargoInput = cargoSolution_.inputTrajectory_[kc];

      // Interpolate robot states/inputs at time t from robot trajectories
      vector_t stackedState(numRobots_ * robotStateDim_ + cargoStateDim_);
      vector_t stackedInput(numRobots_ * robotInputDim_ + cargoInputDim_);

      for (size_t robot = 0; robot < numRobots_; ++robot) {
        const auto& robotTimeTraj = robotSolutions_[robot].timeTrajectory_;
        const auto& robotStateTraj = robotSolutions_[robot].stateTrajectory_;
        const auto& robotInputTraj = robotSolutions_[robot].inputTrajectory_;

        // Safety check: ensure robot trajectory is not empty
        if (robotTimeTraj.empty() || robotStateTraj.empty() || robotInputTraj.empty()) {
          throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolution(): robot " + 
                                   std::to_string(robot) + " trajectory is empty");
        }

        // Interpolate robot state at time t
        vector_t robotState = LinearInterpolation::interpolate(t, robotTimeTraj, robotStateTraj);
        stackedState.segment(robot * robotStateDim_, robotStateDim_) = robotState;

        // Interpolate robot input at time t
        vector_t robotInput = LinearInterpolation::interpolate(t, robotTimeTraj, robotInputTraj);
        stackedInput.segment(robot * robotInputDim_, robotInputDim_) = robotInput.segment(0, robotInputDim_);
      }

      // Append cargo state/input
      stackedState.tail(cargoStateDim_) = cargoState;
      stackedInput.tail(cargoInputDim_) = cargoInput.segment(0, cargoInputDim_);

      filteredStateTrajectory[i] = stackedState;
      filteredInputTrajectory[i] = stackedInput;
      filteredDualTrajectory[i] = dualTrajectory_[i];
    }
  }

  previousSolution_ = AlternatingTargetTrajectories(
      std::move(filteredTimeTrajectory),
      std::move(filteredStateTrajectory),
      std::move(filteredInputTrajectory),
      std::move(filteredDualTrajectory));

  updatePreComputations();
}

void ConsensusADMMSolver::updatePreviousSolutionWithRefAndCargo() {
  const auto& timeTrajectory = cargoSolution_.timeTrajectory_;
  if (timeTrajectory.empty()) {
    throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolutionWithRefAndCargo(): time trajectory is empty");
  }

  // Filter out PreEvent/PostEvent nodes - only keep intermediate nodes where inputs actually exist
  const std::vector<size_t> intermediateIndices = getIntermediateNodeIndices(cargoSolution_);
  const size_t filteredTimeSteps = intermediateIndices.size();
 
  if (filteredTimeSteps == 0) {
    throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolutionWithRefAndCargo(): no intermediate nodes found");
  }

  if (dualTrajectory_.empty()) {
    dualTrajectory_ = vector_array_t(filteredTimeSteps, vector_t::Zero(numRobots_ * ARM_CONTACT_DIM));
  }

  // Get reference trajectories from the reference manager
  const auto& refTargetTrajectories = this->getReferenceManager().getTargetTrajectories();

  // Build filtered trajectories (state, input, time, dual) - all same dimension
  scalar_array_t filteredTimeTrajectory(filteredTimeSteps);
  vector_array_t filteredStateTrajectory(filteredTimeSteps);
  vector_array_t filteredInputTrajectory(filteredTimeSteps);
  vector_array_t filteredDualTrajectory(filteredTimeSteps);
  
  // For each intermediate time point, combine robot states/inputs from reference with cargo states/inputs from cargoSolution_
  const size_t robotStateDim = numRobots_ * robotStateDim_;
  const size_t robotInputDim = numRobots_ * robotInputDim_;  // This includes arm force (ALTERNATING_SINGLE_ROBOT_INPUT_DIM)
  
  for (size_t i = 0; i < filteredTimeSteps; ++i) {
    const size_t k = intermediateIndices[i];
    const scalar_t t = timeTrajectory[k];
    
    filteredTimeTrajectory[i] = t;
    
    // Get robot states and inputs from reference (interpolated at time t)
    vector_t refState = refTargetTrajectories.getDesiredState(t);
    vector_t refInput = refTargetTrajectories.getDesiredInput(t);
    
    // Stack robot states/inputs from reference with cargo states/inputs from cargoSolution_
    vector_t stackedState(robotStateDim + cargoStateDim_);
    vector_t stackedInput(robotInputDim + cargoInputDim_);
    
    // Copy robot states from reference (robots come first in the reference state)
    stackedState.head(robotStateDim) = refState.head(robotStateDim);
    stackedInput.head(robotInputDim) = vector_t::Zero(robotInputDim);
    
    // Copy cargo states/inputs from cargoSolution_
    if (k < cargoSolution_.stateTrajectory_.size()) {
      stackedState.tail(cargoStateDim_) = cargoSolution_.stateTrajectory_[k];
    } else {
      stackedState.tail(cargoStateDim_).setZero();
    }
    
    if (k < cargoSolution_.inputTrajectory_.size()) {
      stackedInput.tail(cargoInputDim_) = cargoSolution_.inputTrajectory_[k].segment(0, cargoInputDim_);
    } else {
      stackedInput.tail(cargoInputDim_).setZero();
    }
    
    filteredStateTrajectory[i] = stackedState;
    filteredInputTrajectory[i] = stackedInput;
    
    // Extract dual from dualTrajectory_ if it's already filtered, otherwise use zero
    // (dualTrajectory_ is filtered in updateDualVariables(), but this might be called before that)
    if (i < dualTrajectory_.size() && dualTrajectory_.size() == filteredTimeSteps) {
      filteredDualTrajectory[i] = dualTrajectory_[i];
    } else {
      throw std::runtime_error("[ConsensusADMMSolver] updatePreviousSolutionWithRefAndCargo(): dualTrajectory_ size mismatch");
    }
  }
  
  previousSolution_ = AlternatingTargetTrajectories(filteredTimeTrajectory, filteredStateTrajectory, 
                                                     filteredInputTrajectory, filteredDualTrajectory);
  updatePreComputations();
}

}  // namespace multi_robot
}  // namespace ocs2

