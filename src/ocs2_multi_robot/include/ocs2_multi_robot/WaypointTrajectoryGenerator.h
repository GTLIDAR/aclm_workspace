//
// Waypoint Trajectory Generator
// Loads waypoints from YAML and generates smooth trajectories using Catmull-Rom splines
// Synchronizes timing across all entities (cargo + robots) per segment
//
// Key design: arc-length parameterized splines ensure constant velocity.
// Per-segment synchronization: all entities reach waypoint k at the same time.
// The segment time = max over all entities of (segment_arc_length / maxLinearVelocity,
//                                              segment_yaw_change / maxRotationalVelocity).
//

#pragma once

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <numeric>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_core/Types.h>
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/terrain/HeightMap.h"
#include <grid_map_core/GridMap.hpp>
#include <Eigen/Dense>

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Simple waypoint structure (x, y, yaw)
 */
struct Waypoint {
  scalar_t x{0.0};
  scalar_t y{0.0};
  scalar_t yaw{0.0};
};

/**
 * @brief Arc-length parameterized spline for a single entity.
 * 
 * Stores dense spline samples + cumulative arc-length table per waypoint segment.
 * Allows querying position at any time t ∈ [0, totalDuration] such that
 * the entity moves at constant speed within each segment.
 */
struct ArcLengthSpline {
  std::vector<Waypoint> samples;            // Dense Catmull-Rom samples
  std::vector<scalar_t> cumulativeArcLen;   // cumulativeArcLen[i] = arc length from start to samples[i]
  scalar_t totalArcLength{0.0};
  
  // Per-segment info (segment k = waypoint k → waypoint k+1)
  size_t numSegments{0};
  std::vector<scalar_t> segmentArcLengths;  // Arc length of each segment
  std::vector<scalar_t> segmentYawChanges;  // Absolute yaw change of each segment
  size_t samplesPerSegment{0};              // How many dense samples per segment
  
  /**
   * @brief Query position at a given arc-length distance from start
   */
  Waypoint atArcLength(scalar_t s) const {
    if (samples.empty()) return Waypoint();
    s = std::max(scalar_t(0.0), std::min(totalArcLength, s));
    
    // Binary search for the sample interval containing arc length s
    auto it = std::lower_bound(cumulativeArcLen.begin(), cumulativeArcLen.end(), s);
    size_t idx = std::distance(cumulativeArcLen.begin(), it);
    if (idx == 0) return samples[0];
    if (idx >= samples.size()) return samples.back();
    
    // Linear interpolation within the interval
    scalar_t s0 = cumulativeArcLen[idx - 1];
    scalar_t s1 = cumulativeArcLen[idx];
    scalar_t t = (s1 > s0) ? (s - s0) / (s1 - s0) : 0.0;
    
    Waypoint w;
    w.x = (1.0 - t) * samples[idx - 1].x + t * samples[idx].x;
    w.y = (1.0 - t) * samples[idx - 1].y + t * samples[idx].y;
    w.yaw = samples[idx - 1].yaw + t * normalizeAngle(samples[idx].yaw - samples[idx - 1].yaw);
    return w;
  }
  
  /**
   * @brief Query position at time t given per-segment durations.
   * 
   * Within each segment the entity moves at constant linear speed.
   * segmentStartTimes[k] = absolute start time of segment k.
   */
  Waypoint atTime(scalar_t t, const std::vector<scalar_t>& segmentDurations,
                  const std::vector<scalar_t>& segmentStartTimes) const {
    if (samples.empty() || segmentDurations.empty()) return Waypoint();
    
    scalar_t totalDuration = segmentStartTimes.back() + segmentDurations.back();
    t = std::max(scalar_t(0.0), std::min(totalDuration, t));
    
    // Find which segment we're in
    size_t seg = 0;
    for (size_t k = 0; k < numSegments; ++k) {
      if (t < segmentStartTimes[k] + segmentDurations[k] || k == numSegments - 1) {
        seg = k;
        break;
      }
    }
    
    // Progress within this segment [0, 1]
    scalar_t segProgress = 0.0;
    if (segmentDurations[seg] > 1e-6) {
      segProgress = (t - segmentStartTimes[seg]) / segmentDurations[seg];
    } else {
      segProgress = 1.0;
    }
    segProgress = std::max(scalar_t(0.0), std::min(scalar_t(1.0), segProgress));
    
    // Arc length at start of this segment + progress * segment arc length
    scalar_t arcLenAtSegStart = 0.0;
    for (size_t k = 0; k < seg; ++k) {
      arcLenAtSegStart += segmentArcLengths[k];
    }
    scalar_t targetArcLen = arcLenAtSegStart + segProgress * segmentArcLengths[seg];
    
    return atArcLength(targetArcLen);
  }
  
  static scalar_t normalizeAngle(scalar_t angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
};

/**
 * @brief Waypoint Trajectory Generator
 * 
 * Loads waypoints from YAML files and generates smooth trajectories.
 * 
 * Key features:
 * - Per-segment synchronization: all entities reach waypoint k at the same time
 * - Segment duration = max over all entities of (arc_len/v_lin, yaw_change/v_rot)
 * - Arc-length parameterized splines → constant speed within each segment
 * - No entity ever exceeds maxLinearVelocity or maxRotationalVelocity
 * 
 * Workflow:
 * 1. Load waypoints from YAML (cargo + robots)
 * 2. On first generateTrajectory() call: build arc-length splines + compute segment durations
 * 3. On each call: output trajectory from current time to end
 */
class WaypointTrajectoryGenerator {
public:
  WaypointTrajectoryGenerator(size_t numRobots, 
                               scalar_t comHeight = 0.5,
                               scalar_t cargoHeightOffset = 0.8)
    : numRobots_(numRobots)
    , comHeight_(comHeight)
    , cargoHeightOffset_(cargoHeightOffset)
    , loaded_(false)
    , trajectoryBuilt_(false)
    , trajectoryStartTime_(0.0)
    , trajectoryDuration_(0.0)
    , maxLinearVelocity_(0.3)
    , maxRotationalVelocity_(0.15)
  {
    robotWaypoints_.resize(numRobots);
    hasExplicitWaypoints_.resize(numRobots, false);
  }
  
  /**
   * @brief Reset trajectory - will rebuild on next generateTrajectory() call
   */
  void reset() {
    trajectoryBuilt_ = false;
    cargoArcSpline_ = ArcLengthSpline();
    robotArcSplines_.clear();
    segmentDurations_.clear();
    segmentStartTimes_.clear();
    std::fill(hasExplicitWaypoints_.begin(), hasExplicitWaypoints_.end(), false);
    ROS_INFO("[WaypointGenerator] Trajectory reset");
  }

  /**
   * @brief Load waypoints from YAML file
   */
  bool loadFromYaml(const std::string& yamlFile, const std::string& trajectoryName) {
    if (yamlFile.empty()) {
      ROS_ERROR("[WaypointGenerator] YAML file path is empty");
      return false;
    }

    std::string fullPath = yamlFile;
    if (yamlFile[0] != '/') {
      const auto configPath = ros::package::getPath("ocs2_multi_robot") + "/config/";
      if (yamlFile.rfind("config/", 0) == 0) {
        fullPath = ros::package::getPath("ocs2_multi_robot") + "/" + yamlFile;
      } else {
        fullPath = configPath + (yamlFile.rfind("yaml/", 0) == 0 ? yamlFile : "yaml/" + yamlFile);
      }
    }
    
    try {
      YAML::Node config = YAML::LoadFile(fullPath);
      
      if (!config[trajectoryName]) {
        ROS_ERROR("[WaypointGenerator] Trajectory '%s' not found", trajectoryName.c_str());
        return false;
      }
      
      YAML::Node trajConfig = config[trajectoryName];
      
      // Load cargo waypoints (supports mixed format: simple points + circular arcs)
      cargoWaypoints_.clear();
      if (trajConfig["cargo"] && trajConfig["cargo"]["waypoints"]) {
        for (const auto& wp : trajConfig["cargo"]["waypoints"]) {
          if (wp["start_x"]) {
            // Format 2: circular arc waypoint
            auto arcWaypoints = expandArcToWaypoints(
                wp["start_x"].as<scalar_t>(), wp["start_y"].as<scalar_t>(),
                wp["start_yaw"].as<scalar_t>(),
                wp["center_x"].as<scalar_t>(), wp["center_y"].as<scalar_t>(),
                wp["angle"].as<scalar_t>(), wp["delta_yaw"].as<scalar_t>());
            
            // Skip start point if it matches last waypoint (avoid duplication)
            size_t startIdx = 0;
            if (!cargoWaypoints_.empty()) {
              const auto& last = cargoWaypoints_.back();
              if (std::abs(arcWaypoints[0].x - last.x) < 0.01 &&
                  std::abs(arcWaypoints[0].y - last.y) < 0.01) {
                startIdx = 1;
              }
            }
            for (size_t k = startIdx; k < arcWaypoints.size(); ++k) {
              cargoWaypoints_.push_back(arcWaypoints[k]);
            }
            ROS_INFO("[WaypointGenerator] Expanded arc to %zu waypoints", arcWaypoints.size() - startIdx);
          } else {
            // Format 1: simple waypoint {x, y, yaw}
            Waypoint waypoint;
            waypoint.x = wp["x"].as<scalar_t>();
            waypoint.y = wp["y"].as<scalar_t>();
            waypoint.yaw = wp["yaw"].as<scalar_t>();
            cargoWaypoints_.push_back(waypoint);
          }
        }
      }
      
      // Load robot waypoints (supports both simple {x,y,yaw} and arc {start_x,...} formats)
      for (size_t robot = 0; robot < numRobots_; ++robot) {
        robotWaypoints_[robot].clear();
        hasExplicitWaypoints_[robot] = false;
        std::string robotKey = "robot" + std::to_string(robot + 1);
        
        if (trajConfig[robotKey] && trajConfig[robotKey]["waypoints"]) {
          hasExplicitWaypoints_[robot] = true;
          for (const auto& wp : trajConfig[robotKey]["waypoints"]) {
            if (wp["start_x"]) {
              // Format 2: circular arc waypoint
              auto arcWaypoints = expandArcToWaypoints(
                  wp["start_x"].as<scalar_t>(), wp["start_y"].as<scalar_t>(),
                  wp["start_yaw"].as<scalar_t>(),
                  wp["center_x"].as<scalar_t>(), wp["center_y"].as<scalar_t>(),
                  wp["angle"].as<scalar_t>(), wp["delta_yaw"].as<scalar_t>());
              
              // Skip start point if it matches last waypoint (avoid duplication)
              size_t startIdx = 0;
              if (!robotWaypoints_[robot].empty()) {
                const auto& last = robotWaypoints_[robot].back();
                if (std::abs(arcWaypoints[0].x - last.x) < 0.01 &&
                    std::abs(arcWaypoints[0].y - last.y) < 0.01) {
                  startIdx = 1;
                }
              }
              for (size_t k = startIdx; k < arcWaypoints.size(); ++k) {
                robotWaypoints_[robot].push_back(arcWaypoints[k]);
              }
            } else {
              // Format 1: simple waypoint {x, y, yaw}
              Waypoint waypoint;
              waypoint.x = wp["x"].as<scalar_t>();
              waypoint.y = wp["y"].as<scalar_t>();
              waypoint.yaw = wp["yaw"].as<scalar_t>();
              robotWaypoints_[robot].push_back(waypoint);
            }
          }
        }
      }
      
      loaded_ = true;
      trajectoryBuilt_ = false;  // Force rebuild with new waypoints
      trajectoryName_ = trajectoryName;
      
      ROS_INFO("[WaypointGenerator] Loaded '%s': cargo=%zu waypoints",
               trajectoryName.c_str(), cargoWaypoints_.size());
      for (size_t r = 0; r < numRobots_; ++r) {
        if (hasExplicitWaypoints_[r]) {
          ROS_INFO("  robot%zu=%zu waypoints (explicit trajectory)", r + 1, robotWaypoints_[r].size());
        } else {
          ROS_INFO("  robot%zu: follows cargo with relative offset", r + 1);
        }
      }
      
      return true;
      
    } catch (const YAML::Exception& e) {
      ROS_ERROR("[WaypointGenerator] YAML error: %s", e.what());
      return false;
    }
  }
  
  void setVelocity(scalar_t velocity) { maxLinearVelocity_ = velocity; }
  void setRotationalVelocity(scalar_t rotVel) { maxRotationalVelocity_ = rotVel; }
  bool isLoaded() const { return loaded_; }
  const std::string& getTrajectoryName() const { return trajectoryName_; }
  
  /**
   * @brief Set robot base offsets relative to cargo (for auto-generating robot waypoints)
   * Each offset is a 6D vector: [x, y, z, yaw, pitch, roll] in cargo frame
   */
  void setRobotBaseOffsets(const std::vector<vector_t>& offsets) {
    robotBaseOffsets_ = offsets;
    ROS_INFO("[WaypointGenerator] Set %zu robot base offsets", offsets.size());
  }
  
  /**
   * @brief Set grid map pointer for boundary-clamped height queries
   */
  void setGridMap(const grid_map::GridMap* gridMap) {
    gridMapPtr_ = gridMap;
  }

  /**
   * @brief Generate MPC target trajectory
   * 
   * On first call: builds arc-length splines and per-segment timing.
   * On each call: outputs trajectory from current time to end, sampling
   * each entity's spline at time-based positions that respect velocity limits.
   * 
   * @param observation Current system state
   * @param heightMap Optional height map for terrain-aware z adjustment
   * @param numSamples Number of trajectory samples to output
   * @return TargetTrajectories for MPC
   */
  TargetTrajectories generateTrajectory(const SystemObservation& observation,
                                         const std::shared_ptr<HeightMap>& heightMap = nullptr,
                                         size_t numSamples = 50) const {
    if (!loaded_ || cargoWaypoints_.empty()) {
      return TargetTrajectories();
    }
    
    const vector_t& currentState = observation.state;
    const size_t stateDim = currentState.size();
    const size_t cargoOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
    const size_t inputDim = observation.input.size();
    
    // ===== BUILD TRAJECTORY (once per load) =====
    if (!trajectoryBuilt_) {
      buildTrajectory(observation);
    }
    
    // ===== COMPUTE ELAPSED TIME AND REMAINING DURATION =====
    scalar_t elapsedTime = observation.time - trajectoryStartTime_;
    elapsedTime = std::max(scalar_t(0.0), elapsedTime);
    
    scalar_t remainingDuration = trajectoryDuration_ - elapsedTime;
    // If trajectory is finished, hold at final position with a small horizon
    if (remainingDuration < 0.1) {
      remainingDuration = 1.0;  // Hold horizon
      elapsedTime = trajectoryDuration_;  // Clamp to end
    }
    
    // ===== SAMPLE TRAJECTORY =====
    scalar_t dt = remainingDuration / (numSamples - 1);
    
    scalar_array_t times(numSamples);
    vector_array_t states(numSamples, vector_t::Zero(stateDim));
    vector_array_t inputs(numSamples, vector_t::Zero(inputDim));
    
    for (size_t i = 0; i < numSamples; ++i) {
      times[i] = observation.time + i * dt;
      
      vector_t& state = states[i];
      state = currentState;  // Preserve foot positions etc.
      
      // Cargo x,y,yaw: from current state at i=0 (MPC continuity), from spline for i>0
      scalar_t cargo_x, cargo_y, cargo_yaw;
      if (i == 0) {
        cargo_x = currentState(cargoOffset);
        cargo_y = currentState(cargoOffset + 1);
        cargo_yaw = currentState(cargoOffset + 6);
      } else {
        scalar_t tSample = elapsedTime + i * dt;
        tSample = std::min(tSample, trajectoryDuration_);
        Waypoint cw = cargoArcSpline_.atTime(tSample, segmentDurations_, segmentStartTimes_);
        cargo_x = cw.x;
        cargo_y = cw.y;
        cargo_yaw = cw.yaw;
        state(cargoOffset) = cargo_x;
        state(cargoOffset + 1) = cargo_y;
        state(cargoOffset + 6) = cargo_yaw;
      }
      
      // Compute robot positions: from own spline (explicit waypoints) or cargo + offset (relative)
      Eigen::Matrix2d R_yaw_2d;
      R_yaw_2d << std::cos(cargo_yaw), -std::sin(cargo_yaw),
                  std::sin(cargo_yaw),  std::cos(cargo_yaw);
      
      scalar_t avgTerrainHeight = 0.0;
      size_t terrainSampleCount = 0;
      std::vector<scalar_t> robotTerrainHeights;
      
      for (size_t r = 0; r < numRobots_; ++r) {
        const size_t rOffset = r * SINGLE_ROBOT_STATE_DIM;
        
        scalar_t robot_x, robot_y, robot_yaw;
        
        if (r < hasExplicitWaypoints_.size() && hasExplicitWaypoints_[r]) {
          // Robot has explicit waypoints: follow its own spline trajectory
          if (i == 0) {
            robot_x = currentState(rOffset);
            robot_y = currentState(rOffset + 1);
            robot_yaw = currentState(rOffset + 6);
          } else {
            scalar_t tSample = elapsedTime + i * dt;
            tSample = std::min(tSample, trajectoryDuration_);
            Waypoint rw = robotArcSplines_[r].atTime(tSample, segmentDurations_, segmentStartTimes_);
            robot_x = rw.x;
            robot_y = rw.y;
            robot_yaw = rw.yaw;
          }
        } else {
          // No explicit waypoints: follow cargo with relative offset
          Eigen::Vector2d offset_xy(0.0, 0.0);
          scalar_t offset_yaw = 0.0;
          if (r < robotBaseOffsets_.size()) {
            offset_xy = robotBaseOffsets_[r].segment<2>(0);
            offset_yaw = robotBaseOffsets_[r](3);
          }
          
          Eigen::Vector2d cargo_xy(cargo_x, cargo_y);
          Eigen::Vector2d robot_xy_vec = cargo_xy + R_yaw_2d * offset_xy;
          robot_x = robot_xy_vec.x();
          robot_y = robot_xy_vec.y();
          robot_yaw = cargo_yaw + offset_yaw;
        }
        
        state(rOffset) = robot_x;
        state(rOffset + 1) = robot_y;
        state(rOffset + 6) = robot_yaw;
        state(rOffset + 7) = 0.0;  // Robot pitch = 0 (upright)
        state(rOffset + 8) = 0.0;  // Robot roll = 0 (upright)
        
        if (heightMap) {
          scalar_t robotTerrainH = getHeightClamped(robot_x, robot_y, heightMap);
          state(rOffset + 2) = robotTerrainH + comHeight_;
          avgTerrainHeight += robotTerrainH;
          robotTerrainHeights.push_back(robotTerrainH);
          terrainSampleCount++;
        }
      }
      
      // Cargo z = average terrain height at all robot positions + cargoHeightOffset_
      if (heightMap && terrainSampleCount > 0) {
        avgTerrainHeight /= terrainSampleCount;
        state(cargoOffset + 2) = avgTerrainHeight + cargoHeightOffset_;
        
        // Cargo pitch and roll from robot terrain height differences (plane fit)
        auto orient = computeOrientationFromRobotHeights(robotTerrainHeights, cargo_yaw);
        state(cargoOffset + 7) = orient.first;   // pitch
        state(cargoOffset + 8) = orient.second;  // roll
      }
    }
    
    // Smooth terrain heights along trajectory for continuous motion across grid map boundary
    // Inside 6x6m detection region: uses real-time terrain; outside: uses boundary height.
    // Gaussian smoothing ensures the transition is gradual.
    if (heightMap) {
      smoothTrajectoryHeights(states, cargoOffset, numRobots_);
    }
    
    return TargetTrajectories(times, states, inputs);
  }

private:
  
  /**
   * @brief Smooth trajectory heights (z) and cargo orientation (pitch, roll) using Gaussian filter.
   * Produces smoother, more continuous trajectories especially across grid map boundary.
   * First and last samples are preserved unchanged.
   */
  static void smoothTrajectoryHeights(vector_array_t& states, size_t cargoOffset, size_t numRobots) {
    const size_t n = states.size();
    if (n < 3) return;
    
    // 5-point Gaussian kernel (sigma ~= 1.0 sample spacing)
    const int kernel_half = 2;
    const scalar_t kernel[] = {0.06, 0.24, 0.40, 0.24, 0.06};
    
    auto smoothComponent = [&](size_t stateOffset) {
      std::vector<scalar_t> values(n);
      for (size_t i = 0; i < n; ++i) {
        values[i] = states[i](stateOffset);
      }
      
      std::vector<scalar_t> smoothed(n);
      for (size_t i = 0; i < n; ++i) {
        scalar_t sum = 0.0;
        scalar_t weight_sum = 0.0;
        for (int k = -kernel_half; k <= kernel_half; ++k) {
          int idx = static_cast<int>(i) + k;
          if (idx >= 0 && idx < static_cast<int>(n)) {
            sum += kernel[k + kernel_half] * values[idx];
            weight_sum += kernel[k + kernel_half];
          }
        }
        smoothed[i] = sum / weight_sum;
      }
      
      // Preserve first and last samples (MPC continuity)
      for (size_t i = 1; i < n - 1; ++i) {
        states[i](stateOffset) = smoothed[i];
      }
    };
    
    // Smooth cargo z, pitch, roll
    smoothComponent(cargoOffset + 2);  // cargo z
    smoothComponent(cargoOffset + 7);  // cargo pitch
    smoothComponent(cargoOffset + 8);  // cargo roll
    
    // Smooth each robot's z
    for (size_t robot = 0; robot < numRobots; ++robot) {
      smoothComponent(robot * SINGLE_ROBOT_STATE_DIM + 2);  // robot z
    }
  }
  
  static scalar_t normalizeAngle(scalar_t angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
  
  static scalar_t catmullRom(scalar_t p0, scalar_t p1, scalar_t p2, scalar_t p3, scalar_t t) {
    scalar_t t2 = t * t, t3 = t2 * t;
    return 0.5 * ((2.0 * p1) + (-p0 + p2) * t +
                  (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                  (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
  }
  
  /**
   * @brief Build arc-length parameterized spline from waypoints.
   * 
   * 1. Sample dense Catmull-Rom points (samplesPerSegment per waypoint segment)
   * 2. Compute cumulative arc length
   * 3. Compute per-segment arc lengths and yaw changes
   */
  static ArcLengthSpline buildArcLengthSpline(const std::vector<Waypoint>& waypoints, 
                                               size_t samplesPerSegment = 50) {
    ArcLengthSpline spline;
    if (waypoints.size() < 2) {
      if (!waypoints.empty()) {
        spline.samples = waypoints;
        spline.cumulativeArcLen = {0.0};
      }
      return spline;
    }
    
    const size_t numSeg = waypoints.size() - 1;
    spline.numSegments = numSeg;
    spline.samplesPerSegment = samplesPerSegment;
    spline.segmentArcLengths.resize(numSeg, 0.0);
    spline.segmentYawChanges.resize(numSeg, 0.0);
    
    const size_t totalSamples = numSeg * samplesPerSegment + 1;
    spline.samples.reserve(totalSamples);
    spline.cumulativeArcLen.reserve(totalSamples);
    
    scalar_t cumArc = 0.0;
    scalar_t prevYaw = waypoints[0].yaw;
    
    for (size_t seg = 0; seg < numSeg; ++seg) {
      scalar_t segArc = 0.0;
      scalar_t segYawChange = 0.0;
      
      size_t numPts = (seg == numSeg - 1) ? samplesPerSegment + 1 : samplesPerSegment;
      
      for (size_t j = 0; j < numPts; ++j) {
        // Skip first point of non-first segments (it's the last point of previous segment)
        if (seg > 0 && j == 0) continue;
        
        scalar_t t = static_cast<scalar_t>(j) / samplesPerSegment;
        
        auto clampIdx = [&](int k) -> size_t {
          return static_cast<size_t>(std::max(0, std::min(static_cast<int>(waypoints.size() - 1), k)));
        };
        
        const Waypoint& p0 = waypoints[clampIdx(static_cast<int>(seg) - 1)];
        const Waypoint& p1 = waypoints[seg];
        const Waypoint& p2 = waypoints[seg + 1];
        const Waypoint& p3 = waypoints[clampIdx(static_cast<int>(seg) + 2)];
        
        Waypoint sample;
        sample.x = catmullRom(p0.x, p1.x, p2.x, p3.x, t);
        sample.y = catmullRom(p0.y, p1.y, p2.y, p3.y, t);
        
        // Yaw: linear interpolation with unwrapping
        scalar_t yawDiff = normalizeAngle(p2.yaw - p1.yaw);
        scalar_t rawYaw = p1.yaw + t * yawDiff;
        scalar_t deltaFromPrev = rawYaw - prevYaw;
        while (deltaFromPrev > M_PI) deltaFromPrev -= 2.0 * M_PI;
        while (deltaFromPrev < -M_PI) deltaFromPrev += 2.0 * M_PI;
        sample.yaw = prevYaw + deltaFromPrev;
        
        // Accumulate arc length and yaw change
        if (!spline.samples.empty()) {
          scalar_t dx = sample.x - spline.samples.back().x;
          scalar_t dy = sample.y - spline.samples.back().y;
          scalar_t ds = std::hypot(dx, dy);
          cumArc += ds;
          segArc += ds;
          segYawChange += std::abs(deltaFromPrev);
        }
        
        spline.samples.push_back(sample);
        spline.cumulativeArcLen.push_back(cumArc);
        prevYaw = sample.yaw;
      }
      
      spline.segmentArcLengths[seg] = segArc;
      spline.segmentYawChanges[seg] = segYawChange;
    }
    
    spline.totalArcLength = cumArc;
    return spline;
  }

  /**
   * @brief Build trajectory with per-segment synchronization.
   * 
   * For each segment k (waypoint k → k+1):
   *   segmentDuration[k] = max over all entities of:
   *     max(entity_segment_arc_length / maxLinearVelocity,
   *         entity_segment_yaw_change / maxRotationalVelocity)
   * 
   * This ensures NO entity exceeds velocity limits in ANY segment.
   */
  void buildTrajectory(const SystemObservation& observation) const {
    const vector_t& currentState = observation.state;
    const size_t cargoOffset = numRobots_ * SINGLE_ROBOT_STATE_DIM;
    
    ROS_INFO("[WaypointGenerator] Building arc-length trajectory at t=%.2f", observation.time);
    
    // Get current positions and build complete waypoint paths (current pos + YAML waypoints)
    Waypoint cargoStart;
    cargoStart.x = currentState(cargoOffset);
    cargoStart.y = currentState(cargoOffset + 1);
    cargoStart.yaw = currentState(cargoOffset + 6);
    
    std::vector<Waypoint> cargoPath;
    cargoPath.push_back(cargoStart);
    for (const auto& wp : cargoWaypoints_) {
      cargoPath.push_back(wp);
    }
    
    std::vector<std::vector<Waypoint>> robotPaths(numRobots_);
    for (size_t r = 0; r < numRobots_; ++r) {
      const size_t rOffset = r * SINGLE_ROBOT_STATE_DIM;
      
      Waypoint robotStart;
      robotStart.x = currentState(rOffset);
      robotStart.y = currentState(rOffset + 1);
      robotStart.yaw = currentState(rOffset + 6);
      
      robotPaths[r].push_back(robotStart);
      
      if (r < hasExplicitWaypoints_.size() && hasExplicitWaypoints_[r]) {
        // Use explicit robot waypoints from YAML
        for (const auto& wp : robotWaypoints_[r]) {
          robotPaths[r].push_back(wp);
        }
      } else if (r < robotBaseOffsets_.size()) {
        // Auto-generate robot waypoints from cargo waypoints + offset
        for (const auto& cargoWp : cargoWaypoints_) {
          Eigen::Matrix2d R;
          R << std::cos(cargoWp.yaw), -std::sin(cargoWp.yaw),
               std::sin(cargoWp.yaw),  std::cos(cargoWp.yaw);
          Eigen::Vector2d offset_xy = robotBaseOffsets_[r].segment<2>(0);
          Eigen::Vector2d robot_xy = Eigen::Vector2d(cargoWp.x, cargoWp.y) + R * offset_xy;
          
          Waypoint rw;
          rw.x = robot_xy.x();
          rw.y = robot_xy.y();
          rw.yaw = cargoWp.yaw + robotBaseOffsets_[r](3);
          robotPaths[r].push_back(rw);
        }
        ROS_INFO("[WaypointGenerator] Auto-generated %zu waypoints for robot%zu from cargo + offset",
                 cargoWaypoints_.size(), r + 1);
      }
    }
    
    // All entities must have the same number of waypoints (segments)
    const size_t numSegments = cargoPath.size() - 1;
    for (size_t r = 0; r < numRobots_; ++r) {
      if (robotPaths[r].size() - 1 != numSegments) {
        ROS_ERROR("[WaypointGenerator] Robot%zu has %zu segments but cargo has %zu! Must match.",
                  r + 1, robotPaths[r].size() - 1, numSegments);
        return;
      }
    }
    
    // ===== BUILD ARC-LENGTH SPLINES =====
    const size_t samplesPerSeg = 50;
    cargoArcSpline_ = buildArcLengthSpline(cargoPath, samplesPerSeg);
    
    robotArcSplines_.resize(numRobots_);
    for (size_t r = 0; r < numRobots_; ++r) {
      robotArcSplines_[r] = buildArcLengthSpline(robotPaths[r], samplesPerSeg);
    }
    
    // ===== COMPUTE PER-SEGMENT DURATIONS =====
    segmentDurations_.resize(numSegments);
    segmentStartTimes_.resize(numSegments);
    
    ROS_INFO("  Per-segment timing (maxLinVel=%.2f m/s, maxRotVel=%.2f rad/s):",
             maxLinearVelocity_, maxRotationalVelocity_);
    
    for (size_t k = 0; k < numSegments; ++k) {
      // Cargo time for this segment
      scalar_t segTime = std::max(
        cargoArcSpline_.segmentArcLengths[k] / maxLinearVelocity_,
        cargoArcSpline_.segmentYawChanges[k] / maxRotationalVelocity_
      );
      
      scalar_t cargoSegTime = segTime;
      
      // Robot times for this segment — take the max
      for (size_t r = 0; r < numRobots_; ++r) {
        scalar_t robotLinTime = robotArcSplines_[r].segmentArcLengths[k] / maxLinearVelocity_;
        scalar_t robotRotTime = robotArcSplines_[r].segmentYawChanges[k] / maxRotationalVelocity_;
        scalar_t robotTime = std::max(robotLinTime, robotRotTime);
        segTime = std::max(segTime, robotTime);
      }
      
      // Minimum segment duration for MPC stability
      segTime = std::max(segTime, scalar_t(0.5));
      segmentDurations_[k] = segTime;
      
      ROS_INFO("    Seg %zu: duration=%.2fs (cargo: arc=%.2fm yaw=%.2frad t=%.2fs)",
               k, segTime,
               cargoArcSpline_.segmentArcLengths[k],
               cargoArcSpline_.segmentYawChanges[k],
               cargoSegTime);
      for (size_t r = 0; r < numRobots_; ++r) {
        scalar_t rLinT = robotArcSplines_[r].segmentArcLengths[k] / maxLinearVelocity_;
        scalar_t rRotT = robotArcSplines_[r].segmentYawChanges[k] / maxRotationalVelocity_;
        ROS_INFO("           robot%zu: arc=%.2fm yaw=%.2frad linT=%.2fs rotT=%.2fs",
                 r + 1, robotArcSplines_[r].segmentArcLengths[k],
                 robotArcSplines_[r].segmentYawChanges[k], rLinT, rRotT);
      }
    }
    
    // Compute segment start times (cumulative)
    segmentStartTimes_[0] = 0.0;
    for (size_t k = 1; k < numSegments; ++k) {
      segmentStartTimes_[k] = segmentStartTimes_[k - 1] + segmentDurations_[k - 1];
    }
    
    trajectoryDuration_ = segmentStartTimes_.back() + segmentDurations_.back();
    trajectoryStartTime_ = observation.time;
    trajectoryBuilt_ = true;
    
    // Print velocity check
    ROS_INFO("  Total duration: %.2fs across %zu segments", trajectoryDuration_, numSegments);
    for (size_t k = 0; k < numSegments; ++k) {
      scalar_t cargoVel = cargoArcSpline_.segmentArcLengths[k] / segmentDurations_[k];
      scalar_t cargoRotVel = cargoArcSpline_.segmentYawChanges[k] / segmentDurations_[k];
      ROS_INFO("    Seg %zu: cargo linVel=%.3f m/s (limit %.3f), rotVel=%.3f rad/s (limit %.3f)",
               k, cargoVel, maxLinearVelocity_, cargoRotVel, maxRotationalVelocity_);
      for (size_t r = 0; r < numRobots_; ++r) {
        scalar_t rVel = robotArcSplines_[r].segmentArcLengths[k] / segmentDurations_[k];
        scalar_t rRotVel = robotArcSplines_[r].segmentYawChanges[k] / segmentDurations_[k];
        ROS_INFO("           robot%zu linVel=%.3f m/s, rotVel=%.3f rad/s",
                 r + 1, rVel, rRotVel);
      }
    }
    
    ROS_INFO("[WaypointGenerator] Trajectory built: %.2fs, velocity-safe", trajectoryDuration_);
  }
  
  static std::pair<scalar_t, scalar_t> computeTerrainOrientation(
      scalar_t x, scalar_t y, const std::shared_ptr<HeightMap>& heightMap, scalar_t delta = 0.1) {
    if (!heightMap) return {0.0, 0.0};
    
    scalar_t dz_dx = (heightMap->GetHeight(x + delta, y) - heightMap->GetHeight(x - delta, y)) / (2.0 * delta);
    scalar_t dz_dy = (heightMap->GetHeight(x, y + delta) - heightMap->GetHeight(x, y - delta)) / (2.0 * delta);
    
    scalar_t pitch = -std::atan(dz_dx);
    scalar_t roll = std::atan(dz_dy);
    
    // Clamp to ~30 degrees
    const scalar_t maxAngle = 0.52;
    pitch = std::max(-maxAngle, std::min(maxAngle, pitch));
    roll = std::max(-maxAngle, std::min(maxAngle, roll));
    
    return {pitch, roll};
  }
  
  /**
   * @brief Query terrain height with clamping to grid map boundary.
   * Points outside the grid map are clamped to the nearest boundary point.
   */
  scalar_t getHeightClamped(scalar_t x, scalar_t y,
                            const std::shared_ptr<HeightMap>& heightMap) const {
    if (!heightMap) return 0.0;
    
    if (gridMapPtr_) {
      grid_map::Position pos(x, y);
      if (!gridMapPtr_->isInside(pos)) {
        grid_map::Position center = gridMapPtr_->getPosition();
        grid_map::Length length = gridMapPtr_->getLength();
        const scalar_t margin = 0.05;
        scalar_t half_x = length.x() / 2.0 - margin;
        scalar_t half_y = length.y() / 2.0 - margin;
        pos.x() = std::max(center.x() - half_x, std::min(center.x() + half_x, pos.x()));
        pos.y() = std::max(center.y() - half_y, std::min(center.y() + half_y, pos.y()));
      }
      return heightMap->GetHeight(pos.x(), pos.y());
    }
    
    return heightMap->GetHeight(x, y);
  }
  
  /**
   * @brief Compute terrain orientation with grid map boundary clamping.
   */
  std::pair<scalar_t, scalar_t> computeTerrainOrientationClamped(
      scalar_t x, scalar_t y, const std::shared_ptr<HeightMap>& heightMap, scalar_t delta = 0.1) const {
    if (!heightMap) return {0.0, 0.0};
    
    scalar_t dz_dx = (getHeightClamped(x + delta, y, heightMap) - getHeightClamped(x - delta, y, heightMap)) / (2.0 * delta);
    scalar_t dz_dy = (getHeightClamped(x, y + delta, heightMap) - getHeightClamped(x, y - delta, heightMap)) / (2.0 * delta);
    
    scalar_t pitch = -std::atan(dz_dx);
    scalar_t roll = std::atan(dz_dy);
    
    const scalar_t maxAngle = 0.52;
    pitch = std::max(-maxAngle, std::min(maxAngle, pitch));
    roll = std::max(-maxAngle, std::min(maxAngle, roll));
    
    return {pitch, roll};
  }
  
  /**
   * @brief Compute cargo pitch and roll from robot terrain heights using least-squares plane fit.
   * Robot body-frame offsets + terrain heights define the cargo tilt.
   * @param robotTerrainHeights  Terrain height at each robot's world position
   * @param cargoYaw  Cargo yaw (unused — offsets are already in body frame)
   * @return pair<pitch, roll> in radians (clamped to ±30°)
   */
  std::pair<scalar_t, scalar_t> computeOrientationFromRobotHeights(
      const std::vector<scalar_t>& robotTerrainHeights, scalar_t /*cargoYaw*/) const {
    const size_t n = std::min(robotTerrainHeights.size(), robotBaseOffsets_.size());
    if (n < 2) return {0.0, 0.0};
    
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
    
    scalar_t pitch = -std::atan(a);
    scalar_t roll = std::atan(b);
    const scalar_t maxAngle = 0.52;
    pitch = std::max(-maxAngle, std::min(maxAngle, pitch));
    roll = std::max(-maxAngle, std::min(maxAngle, roll));
    return {pitch, roll};
  }
  
  /**
   * @brief Expand a circular arc waypoint into a sequence of simple waypoints.
   * Arc starts at (start_x, start_y) and sweeps 'angle' radians around (center_x, center_y).
   * @param delta_yaw Yaw change relative to start_yaw (NOT absolute). Yaw evenly interpolates
   *                from start_yaw to start_yaw + delta_yaw over the arc.
   * @param numSamples Number of intermediate points along the arc (total = numSamples + 1)
   */
  static std::vector<Waypoint> expandArcToWaypoints(
      scalar_t start_x, scalar_t start_y, scalar_t start_yaw,
      scalar_t center_x, scalar_t center_y,
      scalar_t angle, scalar_t delta_yaw,
      size_t numSamples = 10) {
    std::vector<Waypoint> result;
    
    // Compute radius from distance between start point and center
    scalar_t radius = std::sqrt((start_x - center_x) * (start_x - center_x) +
                                (start_y - center_y) * (start_y - center_y));
    
    // Compute start angle on the circle
    scalar_t startAngle = std::atan2(start_y - center_y, start_x - center_x);
    
    // delta_yaw is relative to start_yaw (a delta), NOT an absolute yaw.
    // Yaw evenly changes from start_yaw to (start_yaw + delta_yaw).
    const scalar_t yawDelta = delta_yaw;
    
    for (size_t i = 0; i <= numSamples; ++i) {
      scalar_t t = static_cast<scalar_t>(i) / numSamples;
      scalar_t currentAngle = startAngle + t * angle;
      
      Waypoint wp;
      wp.x = center_x + radius * std::cos(currentAngle);
      wp.y = center_y + radius * std::sin(currentAngle);
      wp.yaw = start_yaw + t * yawDelta;
      result.push_back(wp);
    }
    
    ROS_INFO("[WaypointGenerator] Arc: start(%.2f,%.2f) center(%.2f,%.2f) r=%.2f angle=%.2frad -> end(%.2f,%.2f)",
             start_x, start_y, center_x, center_y, radius, angle,
             result.back().x, result.back().y);
    
    return result;
  }
  
  // ===== Member Variables =====
  size_t numRobots_;
  scalar_t comHeight_;
  scalar_t cargoHeightOffset_;
  scalar_t maxLinearVelocity_;
  scalar_t maxRotationalVelocity_;
  bool loaded_;
  std::string trajectoryName_;
  
  // Waypoints from YAML
  std::vector<Waypoint> cargoWaypoints_;
  std::vector<std::vector<Waypoint>> robotWaypoints_;
  
  // Per-robot flag: true if explicit waypoints are defined in YAML
  std::vector<bool> hasExplicitWaypoints_;
  
  // Robot base offsets relative to cargo (for auto-generating robot waypoints)
  std::vector<vector_t> robotBaseOffsets_;
  
  // Optional grid map pointer for boundary-clamped height queries
  const grid_map::GridMap* gridMapPtr_{nullptr};
  
  // Built arc-length splines (mutable for const generateTrajectory)
  mutable bool trajectoryBuilt_;
  mutable scalar_t trajectoryStartTime_;
  mutable scalar_t trajectoryDuration_;
  mutable ArcLengthSpline cargoArcSpline_;
  mutable std::vector<ArcLengthSpline> robotArcSplines_;
  mutable std::vector<scalar_t> segmentDurations_;    // Duration of each segment (synchronized)
  mutable std::vector<scalar_t> segmentStartTimes_;   // Cumulative start time of each segment
};

}  // namespace multi_robot
}  // namespace ocs2
