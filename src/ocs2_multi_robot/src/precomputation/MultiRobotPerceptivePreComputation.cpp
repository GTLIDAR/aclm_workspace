//
// PreComputation for Multi-robot Perceptive Foot Placement Constraints
//

#include "ocs2_multi_robot/precomputation/MultiRobotPerceptivePreComputation.h"
#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

MultiRobotPerceptivePreComputation::MultiRobotPerceptivePreComputation(
    MultiRobotConvexRegionSelector& convexRegionSelector,
    size_t numRobots,
    size_t numVertices)
    : convexRegionSelectorPtr_(&convexRegionSelector),
      numRobots_(numRobots),
      numVertices_(numVertices),
      totalFeet_(numRobots * QUADRUPED_FOOT_NUM) {
  footPlacementParams_.resize(totalFeet_);
}

MultiRobotPerceptivePreComputation::MultiRobotPerceptivePreComputation(
    const MultiRobotPerceptivePreComputation& other)
    : PreComputation(other),
      convexRegionSelectorPtr_(other.convexRegionSelectorPtr_),
      numRobots_(other.numRobots_),
      numVertices_(other.numVertices_),
      totalFeet_(other.totalFeet_),
      footPlacementParams_(other.footPlacementParams_),
      lastComputedTime_(other.lastComputedTime_) {}

MultiRobotPerceptivePreComputation* MultiRobotPerceptivePreComputation::clone() const {
  return new MultiRobotPerceptivePreComputation(*this);
}

void MultiRobotPerceptivePreComputation::request(RequestSet request, scalar_t t, 
                                                  const vector_t& x, const vector_t& u) {
  // Only compute if soft constraints are requested
  if (!request.contains(Request::SoftConstraint)) {
    return;
  }
  
  // Skip if already computed for this time
  if (std::abs(t - lastComputedTime_) < 1e-9) {
    return;
  }
  
  // Compute foot placement parameters for all feet
  for (size_t footIdx = 0; footIdx < totalFeet_; ++footIdx) {
    computeFootPlacementParameters(footIdx, t);
  }
  
  lastComputedTime_ = t;
}

const FootPlacementParameters& MultiRobotPerceptivePreComputation::getFootPlacementParameters(
    size_t globalFootIndex) const {
  static FootPlacementParameters invalidParams;
  if (globalFootIndex >= totalFeet_) {
    return invalidParams;
  }
  return footPlacementParams_[globalFootIndex];
}

void MultiRobotPerceptivePreComputation::computeFootPlacementParameters(size_t globalFootIndex, 
                                                                         scalar_t time) {
  auto& param = footPlacementParams_[globalFootIndex];
  
  auto projection = convexRegionSelectorPtr_->getProjection(globalFootIndex, time);
  if (projection.regionPtr == nullptr) {
    param.valid = false;
    return;
  }

  auto polygon = convexRegionSelectorPtr_->getConvexPolygon(globalFootIndex, time);
  if (polygon.is_empty()) {
    param.valid = false;
    return;
  }

  matrix_t polytopeA;
  vector_t polytopeB;
  std::tie(polytopeA, polytopeB) = getPolygonConstraint(polygon);
  
  // Transform from terrain frame to world frame
  // p_xy extracts x,y from 3D vector (2x3 matrix)
  matrix_t p_xy = (matrix_t(2, 3) <<
                    1, 0, 0,
                    0, 1, 0).finished();
  
  // Get inverse transform (world to terrain frame)
  const auto& T_pw = projection.regionPtr->transformPlaneToWorld;  // terrain -> world
  matrix3_t R_inv = T_pw.inverse().linear();  // world -> terrain rotation
  vector3_t t_inv = T_pw.inverse().translation();  // world -> terrain translation
  
  // Compute constraint matrices in world frame
  // A_world = A_2d * p_xy * R_inv
  param.a = polytopeA * p_xy * R_inv;  // (numVertices x 3)
  
  // b_world = b_2d + A_2d * (p_xy * t_inv)
  param.b = polytopeB + polytopeA * (p_xy * t_inv);
  param.valid = true;
}

std::pair<matrix_t, vector_t> MultiRobotPerceptivePreComputation::getPolygonConstraint(
    const convex_plane_decomposition::CgalPolygon2d& polygon) const {
  size_t numVerts = polygon.size();
  if (numVerts == 0) {
    return {matrix_t::Zero(numVertices_, 2), vector_t::Constant(numVertices_, 1.0)};
  }

  matrix_t polytopeA = matrix_t::Zero(numVertices_, 2);
  vector_t polytopeB = vector_t::Constant(numVertices_, 1.0);

  for (size_t i = 0; i < std::min(numVerts, numVertices_); i++) {
    size_t j = (i + 1) % numVerts;
    size_t k = (j + 1) % numVerts;
    
    const auto point_a = polygon.vertex(i);
    const auto point_b = polygon.vertex(j);
    const auto point_c = polygon.vertex(k);

    polytopeA.row(i) << point_b.y() - point_a.y(), point_a.x() - point_b.x();
    polytopeB(i) = point_a.y() * point_b.x() - point_a.x() * point_b.y();
    
    if (polytopeA.row(i) * (vector_t(2) << point_c.x(), point_c.y()).finished() + polytopeB(i) < 0) {
      polytopeA.row(i) *= -1;
      polytopeB(i) *= -1;
    }
  }

  return {polytopeA, polytopeB};
}

}  // namespace multi_robot
}  // namespace ocs2
