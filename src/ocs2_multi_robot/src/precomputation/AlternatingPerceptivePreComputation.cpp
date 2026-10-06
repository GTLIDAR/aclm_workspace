#include "ocs2_multi_robot/precomputation/AlternatingPerceptivePreComputation.h"

namespace ocs2 {
namespace multi_robot {

AlternatingPerceptivePreComputation::AlternatingPerceptivePreComputation(
    SwitchedModelReferenceManagerWithTerrain& referenceManager,
    const std::vector<vector_t>& handlePositions,
    const AlternatingTargetTrajectories& previousSolution,
    size_t numDualVariables,
    std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr,
    size_t numVertices)
    : AlternatingPreComputation(referenceManager, handlePositions, previousSolution, numDualVariables),
      handlePositions_(handlePositions),
      numDualVariables_(numDualVariables),
      numVertices_(numVertices),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)) {
  if (convexRegionSelectorPtr_) {
    // getNumRobots() is available after base class construction
    size_t numRobots = getNumRobots();
    perceptivePreComp_ = std::make_unique<MultiRobotPerceptivePreComputation>(
        *convexRegionSelectorPtr_, numRobots, numVertices);
  }
}

AlternatingPerceptivePreComputation* AlternatingPerceptivePreComputation::clone() const {
  // Get reference manager from base class
  SwitchedModelReferenceManagerWithTerrain* refMgr = const_cast<SwitchedModelReferenceManagerWithTerrain*>(
      AlternatingPreComputation::getReferenceManagerPtr());
  if (!refMgr) {
    throw std::runtime_error("[AlternatingPerceptivePreComputation::clone] Failed to get reference manager");
  }
  
  // Create new instance with empty trajectory (will be updated via updatePreviousSolution)
  AlternatingTargetTrajectories emptyTraj;
  return new AlternatingPerceptivePreComputation(
      *refMgr, handlePositions_, emptyTraj, numDualVariables_, convexRegionSelectorPtr_, numVertices_);
}

void AlternatingPerceptivePreComputation::request(RequestSet request, scalar_t t, 
                                                   const vector_t& x, const vector_t& u) {
  // Call base class to cache previous solution
  AlternatingPreComputation::request(request, t, x, u);
  
  // Update perceptive precomputation if available
  if (perceptivePreComp_ && request.contains(Request::SoftConstraint)) {
    perceptivePreComp_->request(request, t, x, u);
  }
}

}  // namespace multi_robot
}  // namespace ocs2
