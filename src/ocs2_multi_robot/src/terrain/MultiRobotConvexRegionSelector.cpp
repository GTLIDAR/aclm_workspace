//
// Multi-robot ConvexRegionSelector implementation
//

#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"

#include <chrono>
#include <cmath>
#include <set>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <ocs2_quadruped/gait/MotionPhaseDefinition.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <ros/ros.h>

namespace {

/**
 * Simplify a polygon boundary by keeping every Nth vertex plus any vertex
 * that deviates significantly from the chord between its kept neighbours.
 * Much more effective than angle-based for curved terrain contours.
 * targetCount: aim for roughly this many vertices in the output.
 * maxDeviation: keep extra vertices that deviate more than this (meters) from the chord.
 */
convex_plane_decomposition::CgalPolygon2d simplifyPolygonRing(
    const convex_plane_decomposition::CgalPolygon2d& polygon,
    size_t targetCount = 40,
    double maxDeviation = 0.02) {
  const size_t n = polygon.size();
  if (n <= targetCount) return polygon;

  // Step size to achieve target count
  const size_t step = std::max<size_t>(n / targetCount, 2);

  // First pass: mark every step-th vertex as kept
  std::vector<bool> keep(n, false);
  for (size_t i = 0; i < n; i += step) {
    keep[i] = true;
  }
  keep[0] = true;

  // Second pass: also keep any vertex that deviates from the chord
  // between its two nearest kept neighbours
  for (size_t i = 0; i < n; ++i) {
    if (keep[i]) continue;
    // Find previous and next kept vertices
    size_t prev_k = (i == 0) ? n - 1 : i - 1;
    while (!keep[prev_k] && prev_k != i) prev_k = (prev_k == 0) ? n - 1 : prev_k - 1;
    size_t next_k = (i + 1) % n;
    while (!keep[next_k] && next_k != i) next_k = (next_k + 1) % n;

    const auto& p = polygon[prev_k];
    const auto& q = polygon[next_k];
    const auto& c = polygon[i];
    // Point-to-line distance
    double ax = CGAL::to_double(q.x() - p.x());
    double ay = CGAL::to_double(q.y() - p.y());
    double bx = CGAL::to_double(c.x() - p.x());
    double by = CGAL::to_double(c.y() - p.y());
    double len2 = ax * ax + ay * ay;
    double dist = (len2 > 1e-12) ? std::abs(ax * by - ay * bx) / std::sqrt(len2) : 0.0;
    if (dist > maxDeviation) {
      keep[i] = true;
    }
  }

  convex_plane_decomposition::CgalPolygon2d result;
  for (size_t i = 0; i < n; ++i) {
    if (keep[i]) result.push_back(polygon[i]);
  }
  if (result.size() < 4) return polygon;
  return result;
}

convex_plane_decomposition::CgalPolygonWithHoles2d simplifyBoundary(
    const convex_plane_decomposition::CgalPolygonWithHoles2d& boundary,
    size_t targetCount = 40,
    double maxDeviation = 0.02) {
  auto outer = simplifyPolygonRing(boundary.outer_boundary(), targetCount, maxDeviation);
  convex_plane_decomposition::CgalPolygonWithHoles2d result(outer);
  for (auto it = boundary.holes_begin(); it != boundary.holes_end(); ++it) {
    result.add_hole(simplifyPolygonRing(*it, targetCount, maxDeviation));
  }
  return result;
}

}  // anonymous namespace

namespace ocs2 {
namespace multi_robot {

MultiRobotConvexRegionSelector::MultiRobotConvexRegionSelector(
    size_t numRobots,
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
    std::shared_ptr<HeightMap> heightMapPtr,
    size_t numVertices,
    double comHeight,
    double minPlanarHeight,
    double maxPlanarHeight)
    : numRobots_(numRobots),
      numVertices_(numVertices),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      heightMapPtr_(std::move(heightMapPtr)),
      comHeight_(comHeight),
      minPlanarHeight_(minPlanarHeight),
      maxPlanarHeight_(maxPlanarHeight) {
  
  const size_t totalFeet = numRobots_ * QUADRUPED_FOOT_NUM;
  
  // Initialize per-foot data structures for stance constraints
  feetProjections_.resize(totalFeet);
  convexPolygons_.resize(totalFeet);
  nominalFootholds_.resize(totalFeet);
  middleTimes_.resize(totalFeet);
  timeEvents_.resize(totalFeet);
}

vector3_t MultiRobotConvexRegionSelector::getFootPositionFromState(size_t globalFootIndex, const vector_t& state) const {
  const size_t robotId = globalFootIndex / QUADRUPED_FOOT_NUM;
  const size_t localFootIdx = globalFootIndex % QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  const size_t footPosOffset = robotStateOffset + 12 + localFootIdx * QUADRUPED_CONTACT_DIM;
  return state.segment<3>(footPosOffset);
}

convex_plane_decomposition::PlanarTerrainProjection MultiRobotConvexRegionSelector::getProjection(
    size_t globalFootIndex, scalar_t time) const {
  if (globalFootIndex >= getTotalFeet() || timeEvents_[globalFootIndex].empty()) {
    return convex_plane_decomposition::PlanarTerrainProjection();
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[globalFootIndex], time);
  if (index < feetProjections_[globalFootIndex].size()) {
    return feetProjections_[globalFootIndex][index];
  }
  return convex_plane_decomposition::PlanarTerrainProjection();
}

convex_plane_decomposition::CgalPolygon2d MultiRobotConvexRegionSelector::getConvexPolygon(
    size_t globalFootIndex, scalar_t time) const {
  if (globalFootIndex >= getTotalFeet() || timeEvents_[globalFootIndex].empty()) {
    return convex_plane_decomposition::CgalPolygon2d();
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[globalFootIndex], time);
  if (index < convexPolygons_[globalFootIndex].size()) {
    return convexPolygons_[globalFootIndex][index];
  }
  return convex_plane_decomposition::CgalPolygon2d();
}

vector3_t MultiRobotConvexRegionSelector::getNominalFoothold(size_t globalFootIndex, scalar_t time) const {
  if (globalFootIndex >= getTotalFeet() || timeEvents_[globalFootIndex].empty()) {
    return vector3_t::Zero();
  }
  const auto index = lookup::findIndexInTimeArray(timeEvents_[globalFootIndex], time);
  if (index < nominalFootholds_[globalFootIndex].size()) {
    return nominalFootholds_[globalFootIndex][index];
  }
  return vector3_t::Zero();
}

bool MultiRobotConvexRegionSelector::isFootPlacementActive(size_t globalFootIndex, scalar_t time) const {
  if (!planarTerrainPtr_ || planarTerrainPtr_->planarRegions.empty()) {
    return false;
  }
  
  if (globalFootIndex >= getTotalFeet()) {
    return false;
  }
  
  auto projection = getProjection(globalFootIndex, time);
  if (projection.regionPtr == nullptr) {
    return false;
  }
  
  auto polygon = getConvexPolygon(globalFootIndex, time);
  return !polygon.is_empty();
}

std::vector<std::vector<bool>> MultiRobotConvexRegionSelector::extractContactFlags(
    const std::vector<size_t>& phaseIDsStock) const {
  const size_t totalFeet = getTotalFeet();
  std::vector<std::vector<bool>> contactFlags(totalFeet);
  
  for (size_t footIdx = 0; footIdx < totalFeet; ++footIdx) {
    const size_t localFootIdx = footIdx % QUADRUPED_FOOT_NUM;
    contactFlags[footIdx].reserve(phaseIDsStock.size());
    
    for (const auto& phaseId : phaseIDsStock) {
      contactFlags[footIdx].push_back(quadruped::modeNumber2StanceLeg(phaseId)[localFootIdx]);
    }
  }
  
  return contactFlags;
}

void MultiRobotConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, 
                                             const vector_t& initState, TargetTrajectories& targetTrajectories) {
  if (!planarTerrainPtr_) {
    return;
  }
  
  if (planarTerrainPtr_->planarRegions.empty()) {
    ROS_WARN_THROTTLE(2.0, "[MultiRobotConvexRegionSelector] No planar regions available yet");
    return;
  }
  
  planarTerrain_ = *planarTerrainPtr_;
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  const auto contactFlagStocks = extractContactFlags(modeSequence);
  const size_t numPhases = modeSequence.size();
  const size_t totalFeet = getTotalFeet();

  double totalProjectionMs = 0.0;
  double totalGrowthMs = 0.0;
  int projectionCount = 0;

  // Find start and final index for each stance phase
  std::vector<std::vector<int>> startIndices(totalFeet);
  std::vector<std::vector<int>> finalIndices(totalFeet);
  for (size_t footIdx = 0; footIdx < totalFeet; footIdx++) {
    startIndices[footIdx] = std::vector<int>(numPhases, 0);
    finalIndices[footIdx] = std::vector<int>(numPhases, 0);
    for (size_t i = 0; i < numPhases; i++) {
      if (contactFlagStocks[footIdx][i]) {
        std::tie(startIndices[footIdx][i], finalIndices[footIdx][i]) = findIndex(i, contactFlagStocks[footIdx]);
      }
    }
  }

  for (size_t footIdx = 0; footIdx < totalFeet; footIdx++) {
    feetProjections_[footIdx].clear();
    convexPolygons_[footIdx].clear();
    nominalFootholds_[footIdx].clear();
    feetProjections_[footIdx].resize(numPhases);
    convexPolygons_[footIdx].resize(numPhases);
    nominalFootholds_[footIdx].resize(numPhases);
    middleTimes_[footIdx].clear();

    const size_t robotId = footIdx / QUADRUPED_FOOT_NUM;
    const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    const double currentRobotBaseZ = initState(robotStateOffset + 2);

    scalar_t lastStandStartTime = NAN;
    
    for (size_t i = 0; i < numPhases; ++i) {
      if (contactFlagStocks[footIdx][i]) {
        // STANCE phase - compute convex region for foot placement constraint
        const int standStartIndex = startIndices[footIdx][i];
        const int standFinalIndex = finalIndices[footIdx][i];
        // Map phase indices to time boundaries:
        // Mode i spans [eventTimes[i-1], eventTimes[i]], with mode 0 starting at initTime.
        // standStartIndex = first phase in stance block, standFinalIndex = last phase + 1
        const scalar_t standStartTime = (standStartIndex == 0) ? initTime : eventTimes[standStartIndex - 1];

        if (!numerics::almost_eq(standStartTime, lastStandStartTime)) {
          // Compute nominal foothold at standStartTime (= end of preceding swing).
          // This is consistent with updateAllRobotsSwingTrajectory() which also computes
          // the nominal foothold at swing-end time, so both use the same touchdown prediction.
          vector3_t nominalFoothold = computeNominalFoothold(footIdx, standStartTime, initState, targetTrajectories);
          
          // Use terrain height at nominal foothold XY to compute the height penalty window.
          // This ensures correct step selection on stairs where the target trajectory may not
          // reflect elevation changes (it commands forward motion at approximately constant height).
          double referenceBaseZ = currentRobotBaseZ;
          if (heightMapPtr_) {
            double terrainAtFoothold = heightMapPtr_->GetHeight(nominalFoothold.x(), nominalFoothold.y());
            referenceBaseZ = terrainAtFoothold + comHeight_;
          }
          const double absoluteMinHeight = referenceBaseZ + minPlanarHeight_;
          const double absoluteMaxHeight = referenceBaseZ + maxPlanarHeight_;
          
          // Height penalty function for terrain projection
          auto penaltyFunction = [absoluteMinHeight, absoluteMaxHeight](const vector3_t& projectedPoint) -> double {
            const double z = projectedPoint.z();
            if (z < absoluteMinHeight || z > absoluteMaxHeight) {
              return std::numeric_limits<double>::max();
            }
            return 0.0;
          };
          
          // Project nominal foothold directly onto planar terrain (single projection)
          const auto tProj0 = std::chrono::steady_clock::now();
          auto projection = getBestPlanarRegionAtPositionInWorld(nominalFoothold, planarTerrain_.planarRegions, penaltyFunction);
          totalProjectionMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tProj0).count();
          projectionCount++;
          
          if (projection.regionPtr != nullptr) {
            scalar_t growthFactor = 1.15;
            convex_plane_decomposition::CgalPolygon2d convexRegion;
            const auto tGrow0 = std::chrono::steady_clock::now();
            try {
              auto simplifiedBdy = simplifyBoundary(projection.regionPtr->boundaryWithInset.boundary);
              convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                  simplifiedBdy, 
                  projection.positionInTerrainFrame, 
                  numVertices_, 
                  growthFactor);
            } catch (...) {
              const double footRadius = 0.10;
              convexRegion = convex_plane_decomposition::createRegularPolygon(
                  projection.positionInTerrainFrame, footRadius, numVertices_);
            }
            totalGrowthMs += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tGrow0).count();
            
            feetProjections_[footIdx][i] = projection;
            convexPolygons_[footIdx][i] = convexRegion;
            nominalFootholds_[footIdx][i] = projection.positionInWorld;
            middleTimes_[footIdx].push_back(standStartTime);
          } else {
            nominalFootholds_[footIdx][i] = nominalFoothold;
          }
          
          lastStandStartTime = standStartTime;
        } else if (i > 0) {
          feetProjections_[footIdx][i] = feetProjections_[footIdx][i - 1];
          convexPolygons_[footIdx][i] = convexPolygons_[footIdx][i - 1];
          nominalFootholds_[footIdx][i] = nominalFootholds_[footIdx][i - 1];
        }
      }
      // SWING phase: no processing needed here - ReferenceManager handles swing trajectory
    }
  }

  for (size_t footIdx = 0; footIdx < totalFeet; footIdx++) {
    timeEvents_[footIdx] = eventTimes;
  }

  // Log per-stage timing breakdown
  size_t rawEdges = 0, simplifiedEdges = 0;
  for (const auto& region : planarTerrain_.planarRegions) {
    const auto& bdy = region.boundaryWithInset.boundary;
    rawEdges += bdy.outer_boundary().size();
    simplifiedEdges += simplifyBoundary(bdy).outer_boundary().size();
  }
  ROS_INFO_THROTTLE(2.0, "[CRSTiming] projection: %.2f ms, growth: %.2f ms | %d projections, edges: %zu->%zu simplified",
                    totalProjectionMs, totalGrowthMs, projectionCount, rawEdges, simplifiedEdges);
}

std::pair<int, int> MultiRobotConvexRegionSelector::findIndex(size_t index, 
                                                               const std::vector<bool>& contactFlagStock) {
  const int numPhases = contactFlagStock.size();

  int startTimesIndex = index;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
    } else {
      break;
    }
  }

  int finalTimesIndex = index + 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip + 1;
    } else {
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

vector3_t MultiRobotConvexRegionSelector::computeNominalFoothold(size_t globalFootIndex, scalar_t time, 
                                                                  const vector_t& initState, 
                                                                  TargetTrajectories& targetTrajectories) {
  const size_t robotId = globalFootIndex / QUADRUPED_FOOT_NUM;
  const size_t localFootIdx = globalFootIndex % QUADRUPED_FOOT_NUM;
  const size_t robotStateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
  
  // Get COM position and yaw from target trajectories or initial state
  vector3_t comPosition;
  scalar_t yaw;
  
  if (!targetTrajectories.timeTrajectory.empty()) {
    const scalar_t clampedTime = std::max(targetTrajectories.timeTrajectory.front(),
                                          std::min(time, targetTrajectories.timeTrajectory.back()));
    const vector_t desiredState = targetTrajectories.getDesiredState(clampedTime);
    
    comPosition << desiredState(robotStateOffset + 0),
                   desiredState(robotStateOffset + 1),
                   desiredState(robotStateOffset + 2);
    yaw = desiredState(robotStateOffset + 6);
  } else {
    comPosition << initState(robotStateOffset + 0),
                   initState(robotStateOffset + 1),
                   initState(robotStateOffset + 2);
    yaw = initState(robotStateOffset + 6);
  }
  
  // Rotation matrix for yaw
  matrix3_t R_yaw;
  R_yaw << std::cos(yaw), -std::sin(yaw), 0.0,
           std::sin(yaw),  std::cos(yaw), 0.0,
           0.0,            0.0,           1.0;
  
  // Get nominal foot offset from COM in body frame and rotate to world frame
  vector3_t nominalOffset = getNominalEePosWrtCom(localFootIdx);
  vector3_t rotatedOffset = R_yaw * nominalOffset;
  
  // Compute nominal foothold in world frame
  scalar_t estimatedGroundZ = comPosition(2) - comHeight_;
  
  vector3_t nominalFoothold;
  nominalFoothold << comPosition(0) + rotatedOffset(0),
                     comPosition(1) + rotatedOffset(1),
                     estimatedGroundZ;
  
  // Get terrain height at nominal position if height map available
  if (heightMapPtr_) {
    nominalFoothold.z() = heightMapPtr_->GetHeight(nominalFoothold.x(), nominalFoothold.y());
  }
  
  return nominalFoothold;
}

vector3_t MultiRobotConvexRegionSelector::projectFootholdOntoTerrain(const vector3_t& nominalFoothold, 
                                                                      scalar_t robotBaseZ) {
  if (!planarTerrainPtr_ || planarTerrainPtr_->planarRegions.empty()) {
    return nominalFoothold;
  }
  
  const auto& regions = planarTerrainPtr_->planarRegions;
  const double absoluteMinHeight = robotBaseZ + minPlanarHeight_;
  const double absoluteMaxHeight = robotBaseZ + maxPlanarHeight_;
  
  // Height penalty function
  auto heightPenaltyFunction = [absoluteMinHeight, absoluteMaxHeight](const vector3_t& p) -> double {
    if (p.z() < absoluteMinHeight || p.z() > absoluteMaxHeight) {
      return std::numeric_limits<double>::max();
    }
    return 0.0;
  };
  
  // Project to nearest planar region
  auto projection = convex_plane_decomposition::getBestPlanarRegionAtPositionInWorld(
      nominalFoothold, regions, heightPenaltyFunction);
  
  if (projection.regionPtr != nullptr) {
    return projection.positionInWorld;
  }
  return nominalFoothold;
}

void MultiRobotConvexRegionSelector::initializeVisualization(ros::NodeHandle& nh, const std::string& frameId) {
  visualizationFrameId_ = frameId;
  footholdPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/cen_opt/predicted_footholds", 1);
  polygonPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/cen_opt/constraint_polygons", 1);
  visualizationInitialized_ = true;
  ROS_INFO("[MultiRobotConvexRegionSelector] Visualization initialized with frame: %s", frameId.c_str());
}

void MultiRobotConvexRegionSelector::publishVisualization(scalar_t currentTime) const {
  if (!visualizationInitialized_) {
    return;
  }
  publishFootholdVisualization(currentTime);
  publishPolygonVisualization(currentTime);
}

void MultiRobotConvexRegionSelector::publishFootholdVisualization(scalar_t currentTime) const {
  if (!visualizationInitialized_) {
    return;
  }
  
  visualization_msgs::MarkerArray markerArray;
  
  // Delete all previous markers
  visualization_msgs::Marker deleteAll;
  deleteAll.header.frame_id = visualizationFrameId_;
  deleteAll.header.stamp = ros::Time::now();
  deleteAll.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.push_back(deleteAll);
  
  std::vector<Color> footColors = {Color::purple, Color::orange, Color::blue, Color::yellow};
  int markerId = 0;
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; ++leg) {
      const size_t globalFootIndex = robot * QUADRUPED_FOOT_NUM + leg;
      const auto rgb = getRGB(footColors[leg]);
      
      // ========== Nominal and projected footholds for stance phases ==========
      for (size_t phaseIdx = 0; phaseIdx < feetProjections_[globalFootIndex].size(); ++phaseIdx) {
        const auto& projection = feetProjections_[globalFootIndex][phaseIdx];
        if (projection.regionPtr == nullptr) continue;
        
        const vector3_t& projected = projection.positionInWorld;
        const vector3_t& nominal = nominalFootholds_[globalFootIndex][phaseIdx];
        
        if (projected.norm() < 0.01) continue;
        
        float alphaFactor = 1.0f - 0.15f * static_cast<float>(phaseIdx);
        alphaFactor = std::max(0.3f, alphaFactor);
        
        // Nominal foothold (smaller, transparent)
        if (nominal.norm() > 0.01) {
          visualization_msgs::Marker nominalMarker;
          nominalMarker.header.frame_id = visualizationFrameId_;
          nominalMarker.header.stamp = ros::Time::now();
          nominalMarker.ns = "nominal_foothold";
          nominalMarker.id = markerId++;
          nominalMarker.type = visualization_msgs::Marker::SPHERE;
          nominalMarker.action = visualization_msgs::Marker::ADD;
          nominalMarker.pose.position.x = nominal.x();
          nominalMarker.pose.position.y = nominal.y();
          nominalMarker.pose.position.z = nominal.z();
          nominalMarker.pose.orientation.w = 1.0;
          nominalMarker.scale.x = 0.04;
          nominalMarker.scale.y = 0.04;
          nominalMarker.scale.z = 0.02;
          nominalMarker.color.r = rgb[0];
          nominalMarker.color.g = rgb[1];
          nominalMarker.color.b = rgb[2];
          nominalMarker.color.a = 0.4f * alphaFactor;
          nominalMarker.lifetime = ros::Duration(0.5);
          markerArray.markers.push_back(nominalMarker);
          
          // Line from nominal to projected (shows terrain adjustment)
          double adjustment = (projected.head<2>() - nominal.head<2>()).norm();
          if (adjustment > 0.01) {
            visualization_msgs::Marker lineMarker;
            lineMarker.header.frame_id = visualizationFrameId_;
            lineMarker.header.stamp = ros::Time::now();
            lineMarker.ns = "foothold_adjustment";
            lineMarker.id = markerId++;
            lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
            lineMarker.action = visualization_msgs::Marker::ADD;
            lineMarker.scale.x = 0.005;
            lineMarker.color.r = rgb[0];
            lineMarker.color.g = rgb[1];
            lineMarker.color.b = rgb[2];
            lineMarker.color.a = 0.6f * alphaFactor;
            lineMarker.lifetime = ros::Duration(0.5);
            
            geometry_msgs::Point p1, p2;
            p1.x = nominal.x(); p1.y = nominal.y(); p1.z = nominal.z();
            p2.x = projected.x(); p2.y = projected.y(); p2.z = projected.z();
            lineMarker.points.push_back(p1);
            lineMarker.points.push_back(p2);
            markerArray.markers.push_back(lineMarker);
          }
        }
        
        // Projected foothold (larger, opaque)
        visualization_msgs::Marker projectedMarker;
        projectedMarker.header.frame_id = visualizationFrameId_;
        projectedMarker.header.stamp = ros::Time::now();
        projectedMarker.ns = "projected_foothold";
        projectedMarker.id = markerId++;
        projectedMarker.type = visualization_msgs::Marker::SPHERE;
        projectedMarker.action = visualization_msgs::Marker::ADD;
        projectedMarker.pose.position.x = projected.x();
        projectedMarker.pose.position.y = projected.y();
        projectedMarker.pose.position.z = projected.z();
        projectedMarker.pose.orientation.w = 1.0;
        projectedMarker.scale.x = 0.06;
        projectedMarker.scale.y = 0.06;
        projectedMarker.scale.z = 0.03;
        projectedMarker.color.r = rgb[0];
        projectedMarker.color.g = rgb[1];
        projectedMarker.color.b = rgb[2];
        projectedMarker.color.a = 0.9f * alphaFactor;
        projectedMarker.lifetime = ros::Duration(0.5);
        markerArray.markers.push_back(projectedMarker);
      }
      
      // ========== Line strip connecting all projected footholds in order ==========
      {
        visualization_msgs::Marker lineStripMarker;
        lineStripMarker.header.frame_id = visualizationFrameId_;
        lineStripMarker.header.stamp = ros::Time::now();
        lineStripMarker.ns = "foothold_trajectory";
        lineStripMarker.id = markerId++;
        lineStripMarker.type = visualization_msgs::Marker::LINE_STRIP;
        lineStripMarker.action = visualization_msgs::Marker::ADD;
        lineStripMarker.scale.x = 0.015;
        lineStripMarker.color.r = rgb[0];
        lineStripMarker.color.g = rgb[1];
        lineStripMarker.color.b = rgb[2];
        lineStripMarker.color.a = 0.8f;
        lineStripMarker.lifetime = ros::Duration(0.5);
        
        for (size_t phaseIdx = 0; phaseIdx < feetProjections_[globalFootIndex].size(); ++phaseIdx) {
          const auto& projection = feetProjections_[globalFootIndex][phaseIdx];
          if (projection.regionPtr == nullptr) continue;
          
          const vector3_t& projected = projection.positionInWorld;
          if (projected.norm() < 0.01) continue;
          
          geometry_msgs::Point p;
          p.x = projected.x();
          p.y = projected.y();
          p.z = projected.z();
          
          // Skip duplicate consecutive points (same stance period produces identical footholds)
          if (!lineStripMarker.points.empty()) {
            const auto& last = lineStripMarker.points.back();
            if (std::abs(p.x - last.x) < 1e-3 &&
                std::abs(p.y - last.y) < 1e-3 &&
                std::abs(p.z - last.z) < 1e-3) {
              continue;
            }
          }
          
          lineStripMarker.points.push_back(p);
        }
        
        if (lineStripMarker.points.size() > 1) {
          markerArray.markers.push_back(lineStripMarker);
        }
      }
    }
  }
  
  footholdPublisher_.publish(markerArray);
}

void MultiRobotConvexRegionSelector::publishPolygonVisualization(scalar_t currentTime) const {
  if (!visualizationInitialized_) {
    return;
  }
  
  visualization_msgs::MarkerArray markerArray;
  
  std::vector<Color> footColors = {Color::purple, Color::orange, Color::blue, Color::yellow};
  int markerId = 0;
  
  for (size_t robot = 0; robot < numRobots_; ++robot) {
    for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; ++leg) {
      const size_t globalFootIndex = robot * QUADRUPED_FOOT_NUM + leg;
      const auto rgb = getRGB(footColors[leg]);
      
      auto projection = getProjection(globalFootIndex, currentTime);
      auto polygon = getConvexPolygon(globalFootIndex, currentTime);
      
      if (projection.regionPtr != nullptr && !polygon.is_empty()) {
        std::vector<geometry_msgs::Point> points;
        points.reserve(polygon.size() + 1);

        // OGRE's BillboardChain requires a valid, non-degenerate line strip.
        for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
          Eigen::Vector3d ptTerrain(CGAL::to_double(it->x()), CGAL::to_double(it->y()), 0.0);
          Eigen::Vector3d ptWorld = projection.regionPtr->transformPlaneToWorld * ptTerrain;
          if (!ptWorld.allFinite()) {
            continue;
          }

          if (!points.empty()) {
            const auto& previous = points.back();
            if (std::hypot(ptWorld.x() - previous.x, ptWorld.y() - previous.y, ptWorld.z() - previous.z) < 1e-6) {
              continue;
            }
          }

          geometry_msgs::Point point;
          point.x = ptWorld.x();
          point.y = ptWorld.y();
          point.z = ptWorld.z();
          points.push_back(point);
        }

        if (points.size() < 3) {
          continue;
        }
        points.push_back(points.front());

        visualization_msgs::Marker polygonMarker;
        polygonMarker.header.frame_id = visualizationFrameId_;
        polygonMarker.header.stamp = ros::Time::now();
        polygonMarker.ns = "constraint_polygon_robot_" + std::to_string(robot);
        polygonMarker.id = markerId++;
        polygonMarker.type = visualization_msgs::Marker::LINE_STRIP;
        polygonMarker.action = visualization_msgs::Marker::ADD;
        polygonMarker.pose.orientation.w = 1.0;
        polygonMarker.scale.x = 0.02;
        polygonMarker.color.r = rgb[0];
        polygonMarker.color.g = rgb[1];
        polygonMarker.color.b = rgb[2];
        polygonMarker.color.a = 0.9f;
        polygonMarker.lifetime = ros::Duration(0.5);
        polygonMarker.points = std::move(points);
        markerArray.markers.push_back(polygonMarker);
      }
    }
  }
  
  polygonPublisher_.publish(markerArray);
}

}  // namespace multi_robot
}  // namespace ocs2
