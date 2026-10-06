//
// Perceptive Reference Manager for Multi-Robot Centroidal MPC implementation
//

#include "ocs2_multi_robot/reference_manager/PerceptiveMultiRobotReferenceManager.h"
#include "ocs2_multi_robot/common/Types.h"

#include <chrono>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_quadruped/gait/MotionPhaseDefinition.h>
#include <ocs2_quadruped/common/Types.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ros/ros.h>

namespace ocs2 {
namespace multi_robot {

PerceptiveMultiRobotReferenceManager::PerceptiveMultiRobotReferenceManager(
    std::shared_ptr<quadruped::GaitSchedule> gaitSchedulePtr,
    std::vector<std::shared_ptr<quadruped::SwingTrajectoryPlanner>> swingTrajectoryPlannerPtrs,
    std::shared_ptr<HeightMap> heightMapPtr,
    std::shared_ptr<MultiRobotConvexRegionSelector> convexRegionSelectorPtr,
    size_t numRobots,
    scalar_t comHeight,
    scalar_t cargoHeightOffset,
    scalar_t baseSwingHeight)
    : SwitchedModelReferenceManagerWithTerrain(std::move(gaitSchedulePtr), 
                                                swingTrajectoryPlannerPtrs.empty() ? nullptr : swingTrajectoryPlannerPtrs[0],
                                                std::move(heightMapPtr), numRobots),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      robotSwingTrajectoryPlannerPtrs_(std::move(swingTrajectoryPlannerPtrs)),
      numRobots_(numRobots),
      comHeight_(comHeight),
      cargoHeightOffset_(cargoHeightOffset),
      baseSwingHeight_(baseSwingHeight) {
  
  if (robotSwingTrajectoryPlannerPtrs_.size() != numRobots_) {
    ROS_WARN("[PerceptiveMultiRobotRefMgr] Number of swing planners (%zu) doesn't match number of robots (%zu).",
             robotSwingTrajectoryPlannerPtrs_.size(), numRobots_);
  }
}

void PerceptiveMultiRobotReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime,
                                                             const vector_t& initState,
                                                             TargetTrajectories& targetTrajectories,
                                                             ModeSchedule& modeSchedule) {
  const size_t expectedFullStateSize = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  if (initState.size() < expectedFullStateSize) {
    ROS_DEBUG_THROTTLE(2.0, "[PerceptiveMultiRobotRefMgr] Partial state size: %zu (expected %zu for %zu robots). "
                       "Skipping perceptive updates (sub-problem call).",
                       initState.size(), expectedFullStateSize, numRobots_);
    const auto timeHorizon = finalTime - initTime;
    modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
    if (targetTrajectories.stateTrajectory.empty()) {
      targetTrajectories.timeTrajectory = {initTime, finalTime};
      targetTrajectories.stateTrajectory = {initState, initState};
    }
    return;
  }

  // Get mode schedule
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  // Use current state as target if none given
  if (targetTrajectories.stateTrajectory.empty()) {
    targetTrajectories.timeTrajectory = {initTime, finalTime};
    targetTrajectories.stateTrajectory = {initState, initState};
  }

  // Update convex region selector - this computes projected footholds
  if (convexRegionSelectorPtr_ && convexRegionSelectorPtr_->getPlanarTerrainPtr()) {
    const auto& planarTerrain = *convexRegionSelectorPtr_->getPlanarTerrainPtr();
    if (!planarTerrain.planarRegions.empty()) {
      const auto tCrs0 = std::chrono::steady_clock::now();
      convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);
      const double crsMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tCrs0).count();
      ROS_INFO_THROTTLE(2.0, "[TerrainTiming] ConvexRegionSelector::update: %.2f ms | %zu planar regions",
                        crsMs, planarTerrain.planarRegions.size());
      convexRegionSelectorPtr_->publishVisualization(initTime);
    }
  }

  // ========== Modify target trajectory base Z & pitch from smooth_planar terrain ==========
  // Query the smooth_planar grid map layer to adjust each robot's base height and pitch to follow terrain slope.
  if (convexRegionSelectorPtr_ && convexRegionSelectorPtr_->getPlanarTerrainPtr()) {
    const auto& gridMap = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
    if (gridMap.exists("smooth_planar")) {
      const scalar_t gradientStep = 0.3;  // [m] step size for finite-difference surface normal
      const size_t cargoStateOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;

      for (size_t tIdx = 0; tIdx < targetTrajectories.stateTrajectory.size(); ++tIdx) {
        auto& state = targetTrajectories.stateTrajectory[tIdx];

        // --- Modify each robot's base Z and pitch ---
        for (size_t robot = 0; robot < numRobots_; ++robot) {
          const size_t rOff = robot * SINGLE_ROBOT_STATE_DIM;
          const grid_map::Position baseXY(state(rOff), state(rOff + 1));

          // Check if position is inside the grid map
          if (!gridMap.isInside(baseXY)) {
            continue;
          }

          // Query smooth_planar height at base XY
          const scalar_t terrainZ = gridMap.atPosition("smooth_planar", baseXY);
          if (std::isnan(terrainZ)) {
            continue;
          }

          // Compute surface normal via central finite differences on smooth_planar
          const grid_map::Position pXp(baseXY(0) + gradientStep, baseXY(1));
          const grid_map::Position pXn(baseXY(0) - gradientStep, baseXY(1));
          const grid_map::Position pYp(baseXY(0), baseXY(1) + gradientStep);
          const grid_map::Position pYn(baseXY(0), baseXY(1) - gradientStep);

          scalar_t dzdx = 0.0, dzdy = 0.0;
          if (gridMap.isInside(pXp) && gridMap.isInside(pXn)) {
            const scalar_t zXp = gridMap.atPosition("smooth_planar", pXp);
            const scalar_t zXn = gridMap.atPosition("smooth_planar", pXn);
            if (!std::isnan(zXp) && !std::isnan(zXn)) {
              dzdx = (zXn - zXp) / (2.0 * gradientStep);  // note sign: matches reference code
            }
          }
          if (gridMap.isInside(pYp) && gridMap.isInside(pYn)) {
            const scalar_t zYp = gridMap.atPosition("smooth_planar", pYp);
            const scalar_t zYn = gridMap.atPosition("smooth_planar", pYn);
            if (!std::isnan(zYp) && !std::isnan(zYn)) {
              dzdy = (zYn - zYp) / (2.0 * gradientStep);
            }
          }

          // Surface normal in world frame (unnormalized, then normalized)
          vector3_t normalWorld(dzdx, dzdy, 1.0);
          normalWorld.normalize();

          // Transform normal into robot body frame to extract pitch
          const scalar_t yaw = state(rOff + 6);
          matrix3_t R_yaw;
          R_yaw << std::cos(yaw), -std::sin(yaw), 0.0,
                   std::sin(yaw),  std::cos(yaw), 0.0,
                   0.0,            0.0,           1.0;
          const vector3_t normalBody = R_yaw.transpose() * normalWorld;

          // Pitch from terrain slope (same formula as reference)
          const scalar_t terrainPitch = std::atan2(normalBody.x(), normalBody.z());

          // Update robot base Z: terrain height + comHeight adjusted by pitch
          state(rOff + 2) = terrainZ + comHeight_ / std::cos(terrainPitch);

          // Update robot pitch
          state(rOff + 7) = terrainPitch;
        }

        // --- Modify cargo base Z and pitch ---
        // Cargo z = average terrain height at nominal robot base positions + cargoHeightOffset
        // This ensures the cargo follows terrain as seen by the robot feet, not at the cargo's own position.
        if (state.size() > cargoStateOffset + CARGO_STATE_DIM - 1) {
          const scalar_t cargoYaw = state(cargoStateOffset + 6);
          const Eigen::Matrix2d R_yaw_2d = (Eigen::Matrix2d() <<
              std::cos(cargoYaw), -std::sin(cargoYaw),
              std::sin(cargoYaw),  std::cos(cargoYaw)).finished();
          const Eigen::Vector2d cargoXY(state(cargoStateOffset), state(cargoStateOffset + 1));

          // Compute average terrain height at nominal robot base positions (clamped to grid map)
          scalar_t avgTerrainZ = 0.0;
          size_t validCount = 0;
          std::vector<scalar_t> robotTerrainHeights;
          for (size_t robot = 0; robot < numRobots_; ++robot) {
            Eigen::Vector2d robotXY = cargoXY;
            if (robot < robotBaseOffsets_.size()) {
              robotXY += R_yaw_2d * robotBaseOffsets_[robot].segment<2>(0);
            }
            // Clamp to grid map boundary for points outside detection region
            grid_map::Position queryPos(robotXY.x(), robotXY.y());
            if (!gridMap.isInside(queryPos)) {
              const grid_map::Position center = gridMap.getPosition();
              const grid_map::Length length = gridMap.getLength();
              const scalar_t margin = 0.05;
              const scalar_t halfX = length.x() / 2.0 - margin;
              const scalar_t halfY = length.y() / 2.0 - margin;
              queryPos.x() = std::max(center.x() - halfX, std::min(center.x() + halfX, queryPos.x()));
              queryPos.y() = std::max(center.y() - halfY, std::min(center.y() + halfY, queryPos.y()));
            }
            const scalar_t terrainZ = gridMap.atPosition("smooth_planar", queryPos);
            if (!std::isnan(terrainZ)) {
              avgTerrainZ += terrainZ;
              robotTerrainHeights.push_back(terrainZ);
              validCount++;
            }
          }

          if (validCount > 0) {
            avgTerrainZ /= validCount;

            // Compute cargo pitch and roll from robot terrain height differences (plane fit)
            scalar_t cargoPitch = 0.0;
            scalar_t cargoRoll = 0.0;
            if (robotTerrainHeights.size() >= 2 && robotBaseOffsets_.size() >= 2) {
              const size_t n = std::min(robotTerrainHeights.size(), robotBaseOffsets_.size());
              scalar_t ox_mean = 0, oy_mean = 0, h_mean = 0;
              for (size_t i = 0; i < n; ++i) {
                ox_mean += robotBaseOffsets_[i](0);
                oy_mean += robotBaseOffsets_[i](1);
                h_mean += robotTerrainHeights[i];
              }
              ox_mean /= n; oy_mean /= n; h_mean /= n;
              
              scalar_t Sxx = 0, Sxy = 0, Syy = 0, Sxh = 0, Syh = 0;
              for (size_t i = 0; i < n; ++i) {
                scalar_t dx = robotBaseOffsets_[i](0) - ox_mean;
                scalar_t dy = robotBaseOffsets_[i](1) - oy_mean;
                scalar_t dh = robotTerrainHeights[i] - h_mean;
                Sxx += dx * dx; Sxy += dx * dy; Syy += dy * dy;
                Sxh += dx * dh; Syh += dy * dh;
              }
              
              scalar_t det = Sxx * Syy - Sxy * Sxy;
              scalar_t a = 0.0, b = 0.0;
              if (std::abs(det) > 1e-10) {
                a = (Syy * Sxh - Sxy * Syh) / det;
                b = (Sxx * Syh - Sxy * Sxh) / det;
              } else if (std::abs(Sxx) > 1e-10) {
                a = Sxh / Sxx;
              } else if (std::abs(Syy) > 1e-10) {
                b = Syh / Syy;
              }
              
              cargoPitch = -std::atan(a);
              cargoRoll = std::atan(b);
              const scalar_t maxAngle = 0.52;
              cargoPitch = std::max(-maxAngle, std::min(maxAngle, cargoPitch));
              cargoRoll = std::max(-maxAngle, std::min(maxAngle, cargoRoll));
            }

            state(cargoStateOffset + 2) = avgTerrainZ + cargoHeightOffset_ / std::cos(cargoPitch);
            state(cargoStateOffset + 7) = cargoPitch;   // cargo pitch
            state(cargoStateOffset + 8) = cargoRoll;    // cargo roll
          }
        }
      }
    }
  }

  // Update swing trajectories for all robots using projected footholds
  updateAllRobotsSwingTrajectory(initTime, initState, targetTrajectories, modeSchedule);
}

void PerceptiveMultiRobotReferenceManager::updateAllRobotsSwingTrajectory(scalar_t initTime,
                                                                           const vector_t& initState,
                                                                           const TargetTrajectories& targetTrajectories,
                                                                           ModeSchedule& modeSchedule) {
  const size_t expectedFullStateSize = numRobots_ * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  if (initState.size() < expectedFullStateSize) {
    ROS_ERROR_THROTTLE(1.0, "[PerceptiveMultiRobotRefMgr] updateAllRobotsSwingTrajectory: Invalid state size.");
    return;
  }

  auto heightMapPtr = getHeightMap();
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;
  const size_t numPhases = modeSequence.size();
  
  // Extract contact flags for each phase
  quadruped::feet_array_t<std::vector<bool>> contactFlagStocks;
  for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; leg++) {
    contactFlagStocks[leg].resize(numPhases);
  }
  for (size_t p = 0; p < numPhases; ++p) {
    const auto contactFlag = quadruped::modeNumber2StanceLeg(modeSequence[p]);
    for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; leg++) {
      contactFlagStocks[leg][p] = contactFlag[leg];
    }
  }
  
  // Process each robot
  for (size_t robotIndex = 0; robotIndex < numRobots_; ++robotIndex) {
    if (robotIndex >= robotSwingTrajectoryPlannerPtrs_.size() || 
        !robotSwingTrajectoryPlannerPtrs_[robotIndex]) {
      ROS_WARN_THROTTLE(2.0, "[PerceptiveMultiRobotRefMgr] Swing planner not available for robot %zu", robotIndex);
      continue;
    }
    
    const size_t robotStateOffset = robotIndex * SINGLE_ROBOT_STATE_DIM;
    
    // Height sequences for swing trajectory planner
    quadruped::feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;
    quadruped::feet_array_t<scalar_array_t> swingHeightSequence;
    quadruped::feet_array_t<std::vector<std::pair<vector3_t, vector3_t>>> swingPositions;
    
    for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; leg++) {
      scalar_array_t liftOffHeights(numPhases, 0.0);
      scalar_array_t touchDownHeights(numPhases, 0.0);
      std::vector<std::pair<vector3_t, vector3_t>> legSwingPositions(numPhases);
      
      // Get current (measured) foot position from state
      const size_t footOffset = robotStateOffset + 12 + leg * 3;
      const vector3_t measuredFootPos(initState(footOffset), initState(footOffset + 1), initState(footOffset + 2));
      
      const size_t globalFootIndex = robotIndex * QUADRUPED_FOOT_NUM + leg;
      
      // ========== Compute per-phase foot positions ==========
      // Swing phases: compute touchdown target using predicted base at swing-end time
      // Stance phases: use measured position (current) or projected foothold from ConvexRegionSelector
      std::vector<vector3_t> positionsPerPhase(numPhases);
      vector3_t lastFootPos = measuredFootPos;
      
      for (size_t p = 0; p < numPhases; ++p) {
        bool isStance = contactFlagStocks[leg][p];
        bool isCurrentPhase = (p == 0);
        
        if (isStance) {
          if (isCurrentPhase) {
            positionsPerPhase[p] = measuredFootPos;
          } else {
            positionsPerPhase[p] = lastFootPos;
          }
          // Update lastFootPos from stance constraint projection (stable terrain position)
          if (convexRegionSelectorPtr_) {
            // Query projection at mid-stance time for this phase
            scalar_t queryTime = (p > 0 && (p - 1) < eventTimes.size()) ? eventTimes[p - 1] + 1e-4 : initTime;
            auto projection = convexRegionSelectorPtr_->getProjection(globalFootIndex, queryTime);
            if (projection.regionPtr != nullptr) {
              lastFootPos = projection.positionInWorld;
              if (!isCurrentPhase) {
                positionsPerPhase[p] = lastFootPos;
              }
            }
          }
        } else {
          // SWING phase: compute touchdown target at swing-end time
          // This mirrors the old computeOptimalFootholds() logic
          vector3_t touchdownPos = lastFootPos;
          
          // Find swing end time (= start of next stance phase)
          scalar_t swingEndTime = eventTimes.back();
          for (size_t np = p + 1; np < numPhases; ++np) {
            if (contactFlagStocks[leg][np]) {
              swingEndTime = (np > 0 && (np - 1) < eventTimes.size()) ? eventTimes[np - 1] : initTime;
              break;
            }
          }
          
          // Predict base position at swing end time from target trajectory
          vector3_t basePosAtSwingEnd;
          scalar_t yawAtSwingEnd;
          if (!targetTrajectories.timeTrajectory.empty()) {
            scalar_t clampedTime = std::max(targetTrajectories.timeTrajectory.front(),
                                            std::min(swingEndTime, targetTrajectories.timeTrajectory.back()));
            vector_t desiredState = targetTrajectories.getDesiredState(clampedTime);
            basePosAtSwingEnd << desiredState(robotStateOffset), desiredState(robotStateOffset + 1), desiredState(robotStateOffset + 2);
            yawAtSwingEnd = desiredState(robotStateOffset + 6);
          } else {
            basePosAtSwingEnd << initState(robotStateOffset), initState(robotStateOffset + 1), initState(robotStateOffset + 2);
            yawAtSwingEnd = initState(robotStateOffset + 6);
          }
          
          // Compute nominal foothold at predicted base
          matrix3_t R_yaw;
          R_yaw << std::cos(yawAtSwingEnd), -std::sin(yawAtSwingEnd), 0.0,
                   std::sin(yawAtSwingEnd),  std::cos(yawAtSwingEnd), 0.0,
                   0.0,                      0.0,                    1.0;
          vector3_t nominalOffset = getNominalEePosWrtCom(leg);
          vector3_t rotatedOffset = R_yaw * nominalOffset;
          
          vector3_t nominalFoothold;
          nominalFoothold << basePosAtSwingEnd(0) + rotatedOffset(0),
                             basePosAtSwingEnd(1) + rotatedOffset(1),
                             basePosAtSwingEnd(2) - comHeight_;
          
          // Get terrain height and project to planar region if available
          if (heightMapPtr) {
            nominalFoothold.z() = heightMapPtr->GetHeight(nominalFoothold.x(), nominalFoothold.y());
          }
          
          // Project to nearest planar region for accurate Z on stepstones
          if (convexRegionSelectorPtr_ && convexRegionSelectorPtr_->getPlanarTerrainPtr()) {
            const auto& planarTerrain = *convexRegionSelectorPtr_->getPlanarTerrainPtr();
            if (!planarTerrain.planarRegions.empty()) {
              // Use terrain height at predicted base XY for robust stair step selection.
              // Target trajectory base Z may not reflect elevation changes on stairs.
              scalar_t robotBaseZ = basePosAtSwingEnd(2);
              if (heightMapPtr) {
                double terrainAtBase = heightMapPtr->GetHeight(basePosAtSwingEnd(0), basePosAtSwingEnd(1));
                robotBaseZ = terrainAtBase + comHeight_;
              }
              double absMinH = robotBaseZ + convexRegionSelectorPtr_->getMinPlanarHeight();
              double absMaxH = robotBaseZ + convexRegionSelectorPtr_->getMaxPlanarHeight();
              auto penaltyFn = [absMinH, absMaxH](const vector3_t& pt) -> double {
                return (pt.z() < absMinH || pt.z() > absMaxH) ? std::numeric_limits<double>::max() : 0.0;
              };
              auto projection = convex_plane_decomposition::getBestPlanarRegionAtPositionInWorld(
                  nominalFoothold, planarTerrain.planarRegions, penaltyFn);
              if (projection.regionPtr != nullptr) {
                touchdownPos = projection.positionInWorld;
              } else {
                touchdownPos = nominalFoothold;
              }
            } else {
              touchdownPos = nominalFoothold;
            }
          } else {
            touchdownPos = nominalFoothold;
          }
          
          positionsPerPhase[p] = touchdownPos;
          lastFootPos = touchdownPos;
        }
      }
      
      // ========== getHeights(): Extract z from positionInWorld ==========
      vector3_t currentLiftoffPos = measuredFootPos;
      
      for (size_t p = 0; p < numPhases; ++p) {
        bool isSwingPhase = !contactFlagStocks[leg][p];
        
        if (isSwingPhase) {
          // Liftoff: z from where the foot lifts off
          liftOffHeights[p] = currentLiftoffPos.z();
          
          // Touchdown: z from projected position
          touchDownHeights[p] = positionsPerPhase[p].z();
          
          // Store positions for adaptive swing height
          legSwingPositions[p] = std::make_pair(currentLiftoffPos, positionsPerPhase[p]);
          
          // After swing, the foot lands at the projected position
          currentLiftoffPos = positionsPerPhase[p];
        } else {
          // Stance phase: use current position z
          liftOffHeights[p] = positionsPerPhase[p].z();
          touchDownHeights[p] = positionsPerPhase[p].z();
          legSwingPositions[p] = std::make_pair(vector3_t::Zero(), vector3_t::Zero());
          
          // Update liftoff position for next swing
          currentLiftoffPos = positionsPerPhase[p];
        }
      }
      
      liftOffHeightSequence[leg] = liftOffHeights;
      touchDownHeightSequence[leg] = touchDownHeights;
      swingPositions[leg] = legSwingPositions;
    }
    
    // Compute adaptive swing heights
    for (size_t leg = 0; leg < QUADRUPED_FOOT_NUM; leg++) {
      scalar_array_t adaptiveSwingHeights(numPhases, baseSwingHeight_);
      
      for (size_t p = 0; p < numPhases; ++p) {
        bool isSwingPhase = !contactFlagStocks[leg][p];
        if (isSwingPhase && heightMapPtr) {
          const auto& liftoffPos = swingPositions[leg][p].first;
          const auto& touchdownPos = swingPositions[leg][p].second;
          adaptiveSwingHeights[p] = computeAdaptiveSwingHeight(
              liftoffPos.x(), liftoffPos.y(), liftoffPos.z(),
              touchdownPos.x(), touchdownPos.y(), touchdownPos.z(),
              baseSwingHeight_, 0.04);
        }
      }
      swingHeightSequence[leg] = adaptiveSwingHeights;
    }
    
    // ========== swingTrajectoryPtr_->update() ==========
    if (heightMapPtr) {
      robotSwingTrajectoryPlannerPtrs_[robotIndex]->update(modeSchedule, liftOffHeightSequence, 
                                                           touchDownHeightSequence, swingHeightSequence);
    } else {
      const scalar_t baseZ = initState(robotStateOffset + 2);
      const scalar_t estimatedGroundZ = baseZ - comHeight_;
      robotSwingTrajectoryPlannerPtrs_[robotIndex]->update(modeSchedule, estimatedGroundZ);
    }
  }
}

scalar_t PerceptiveMultiRobotReferenceManager::computeAdaptiveSwingHeight(
    scalar_t liftoffX, scalar_t liftoffY, scalar_t liftoffZ,
    scalar_t touchdownX, scalar_t touchdownY, scalar_t touchdownZ,
    scalar_t baseSwingHeight, scalar_t clearance) const {
  
  // swingHeight >= |z_touchdown - z_liftoff| + clearance
  scalar_t heightDiff = std::abs(touchdownZ - liftoffZ);
  scalar_t requiredSwingHeight = heightDiff + clearance;
  
  scalar_t adaptiveHeight = std::max(baseSwingHeight, requiredSwingHeight);
  
  // Cap the adaptive swing height to avoid excessively high swings
  const scalar_t maxSwingHeight = 0.25;
  adaptiveHeight = std::min(adaptiveHeight, maxSwingHeight);
  
  return adaptiveHeight;
}

}  // namespace multi_robot
}  // namespace ocs2
