#pragma once

#include <ocs2_mpc/MPC_BASE.h>

#include "ocs2_multi_robot/alternating_base/ConsensusADMMSolver.h"

namespace ocs2 {
namespace multi_robot {

class ConsensusADMMSolverMPC final : public MPC_BASE {
 public:
 ConsensusADMMSolverMPC(const AlternatingInterface& alternatingInterface)
      : MPC_BASE(alternatingInterface.getMpcSettings()) {
    solverPtr_.reset(new ConsensusADMMSolver(alternatingInterface));
  };

  ~ConsensusADMMSolverMPC() override = default;

  ConsensusADMMSolver* getSolverPtr() override { return solverPtr_.get(); }
  const ConsensusADMMSolver* getSolverPtr() const override { return solverPtr_.get(); }

 protected:
  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    if (settings().coldStart_) {
      solverPtr_->reset();
    }
    solverPtr_->run(initTime, initState, finalTime);
  }

 private:
  std::unique_ptr<ConsensusADMMSolver> solverPtr_;
};

}  // namespace multi_robot
}  // namespace ocs2
