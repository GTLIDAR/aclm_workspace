#include <pinocchio/fwd.hpp>

// Include CGAL-dependent headers first to avoid BOOST_PARAMETER_MAX_ARITY redefinition warning
#include "ocs2_multi_robot/MultiRobotWCargoInterface.h"

#include "ocs2_multi_robot/MultiRobotGeneralizedTargetTrajectoriesPublisher.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <tuple>
#include <mutex>
#include <ocs2_multi_robot/common/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_msgs/mpc_observation.h>
#include <angles/angles.h>
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include <gazebo_msgs/ModelStates.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace ocs2;
using namespace multi_robot;

namespace
{
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
scalar_t TIME_TO_TARGET;
scalar_t CARGO_HEIGHT_OFFSET = 0.8;  // Height of cargo COM above terrain, loaded from config
scalar_t TERRAIN_GRADIENT_DELTA = 0.1;  // Finite difference step for terrain gradient
std::vector<vector_t> ROBOT_BASE_OFFSETS;
bool offsets_initialized_from_gazebo = false;
size_t num_robots_local = 0;
ros::Subscriber model_states_sub;  // Keep subscriber alive
ros::Subscriber terrain_sub;  // Keep terrain subscriber alive
bool use_perceptive_mode = false;  // Perceptive terrain mode flag
std::shared_ptr<HeightMap> heightMapPtr;  // Height map for terrain-aware trajectory generation
std::shared_ptr<MultiRobotGridMapHeightMap> gridMapHeightMapPtr;  // Concrete type for terrain updates

/**
 * Compute terrain orientation (pitch, roll) from height map using finite differences.
 * @param x Query x position
 * @param y Query y position
 * @param heightMap Height map pointer
 * @param delta Finite difference step size
 * @return pair<pitch, roll> in radians
 */
std::pair<scalar_t, scalar_t> computeTerrainOrientation(
    scalar_t x, scalar_t y,
    const std::shared_ptr<HeightMap>& heightMap,
    scalar_t delta = 0.1)
{
  if (!heightMap) {
    return {0.0, 0.0};
  }
  
  // Query heights at neighboring points using central differences
  scalar_t h_px = heightMap->GetHeight(x + delta, y);  // +x
  scalar_t h_nx = heightMap->GetHeight(x - delta, y);  // -x
  scalar_t h_py = heightMap->GetHeight(x, y + delta);  // +y
  scalar_t h_ny = heightMap->GetHeight(x, y - delta);  // -y
  
  // Compute gradients using central differences
  scalar_t dz_dx = (h_px - h_nx) / (2.0 * delta);
  scalar_t dz_dy = (h_py - h_ny) / (2.0 * delta);
  
  // Convert gradient to pitch (rotation around Y-axis) and roll (rotation around X-axis)
  // For a surface with normal n = (-dz/dx, -dz/dy, 1), normalized
  // pitch = -atan(dz/dx)  (positive slope in +x direction -> negative pitch)
  // roll = atan(dz/dy)    (positive slope in +y direction -> positive roll)
  scalar_t pitch = -std::atan(dz_dx);
  scalar_t roll = std::atan(dz_dy);
  
  // Clamp to reasonable values (max ~30 degrees)
  const scalar_t max_angle = 0.52;  // ~30 degrees
  pitch = std::max(-max_angle, std::min(max_angle, pitch));
  roll = std::max(-max_angle, std::min(max_angle, roll));
  
  return {pitch, roll};
}

/**
 * Compute cargo pitch and roll from robot terrain heights using least-squares plane fit.
 * Robot positions in cargo body frame (offsets) + their terrain heights define the cargo tilt.
 * For 2 robots side-by-side: roll is well-determined, pitch from height gradient.
 * @param robotOffsets  Body-frame offsets for each robot [x,y,...]
 * @param robotTerrainHeights  Terrain height at each robot's world position
 * @param cargoYaw  Cargo yaw for transforming gradient to body frame
 * @return pair<pitch, roll> in radians (clamped to ±30°)
 */
std::pair<scalar_t, scalar_t> computeCargoOrientationFromRobotHeights(
    const std::vector<vector_t>& robotOffsets,
    const std::vector<scalar_t>& robotTerrainHeights,
    scalar_t cargoYaw)
{
  const size_t n = robotTerrainHeights.size();
  if (n < 2) return {0.0, 0.0};
  
  // Compute centroid of body-frame offsets and heights
  scalar_t ox_mean = 0.0, oy_mean = 0.0, h_mean = 0.0;
  for (size_t i = 0; i < n; ++i) {
    ox_mean += robotOffsets[i](0);
    oy_mean += robotOffsets[i](1);
    h_mean += robotTerrainHeights[i];
  }
  ox_mean /= n; oy_mean /= n; h_mean /= n;
  
  // Least-squares plane fit: h - h_mean = a*(ox - ox_mean) + b*(oy - oy_mean)
  // Normal equations: [Sxx Sxy; Sxy Syy] * [a; b] = [Sxh; Syh]
  scalar_t Sxx = 0, Sxy = 0, Syy = 0, Sxh = 0, Syh = 0;
  for (size_t i = 0; i < n; ++i) {
    scalar_t dx = robotOffsets[i](0) - ox_mean;
    scalar_t dy = robotOffsets[i](1) - oy_mean;
    scalar_t dh = robotTerrainHeights[i] - h_mean;
    Sxx += dx * dx; Sxy += dx * dy; Syy += dy * dy;
    Sxh += dx * dh; Syh += dy * dh;
  }
  
  scalar_t det = Sxx * Syy - Sxy * Sxy;
  scalar_t a = 0.0, b = 0.0;  // gradient in body-frame x and y
  if (std::abs(det) > 1e-10) {
    a = (Syy * Sxh - Sxy * Syh) / det;
    b = (Sxx * Syh - Sxy * Sxh) / det;
  } else if (std::abs(Sxx) > 1e-10) {
    // Robots collinear along body-x: only pitch determinable
    a = Sxh / Sxx;
  } else if (std::abs(Syy) > 1e-10) {
    // Robots collinear along body-y: only roll determinable
    b = Syh / Syy;
  }
  
  // a = dz/dx_body, b = dz/dy_body
  scalar_t pitch = -std::atan(a);  // positive slope in +x -> negative pitch
  scalar_t roll = std::atan(b);    // positive slope in +y -> positive roll
  
  // Clamp to ±30 degrees
  const scalar_t maxAngle = 0.52;
  pitch = std::max(-maxAngle, std::min(maxAngle, pitch));
  roll = std::max(-maxAngle, std::min(maxAngle, roll));
  
  return {pitch, roll};
}

/**
 * Query terrain height with clamping to grid map boundary for points outside the detection region.
 * Points inside the grid map: return actual terrain height.
 * Points outside: clamp query to nearest boundary point for smooth connection.
 */
scalar_t getTerrainHeightClamped(scalar_t x, scalar_t y,
                                  const std::shared_ptr<HeightMap>& heightMap,
                                  const grid_map::GridMap* gridMap)
{
  if (!heightMap) return 0.0;
  
  if (gridMap) {
    grid_map::Position pos(x, y);
    if (!gridMap->isInside(pos)) {
      // Clamp to grid map boundary (with small inset to avoid edge NaN)
      grid_map::Position center = gridMap->getPosition();
      grid_map::Length length = gridMap->getLength();
      const scalar_t margin = 0.05;  // 5cm inset from edge
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
 * Compute terrain orientation with clamping to grid map boundary.
 * Uses finite differences with clamped height queries for smooth boundary behavior.
 */
std::pair<scalar_t, scalar_t> computeTerrainOrientationClamped(
    scalar_t x, scalar_t y,
    const std::shared_ptr<HeightMap>& heightMap,
    const grid_map::GridMap* gridMap,
    scalar_t delta = 0.1)
{
  if (!heightMap) {
    return {0.0, 0.0};
  }
  
  scalar_t h_px = getTerrainHeightClamped(x + delta, y, heightMap, gridMap);
  scalar_t h_nx = getTerrainHeightClamped(x - delta, y, heightMap, gridMap);
  scalar_t h_py = getTerrainHeightClamped(x, y + delta, heightMap, gridMap);
  scalar_t h_ny = getTerrainHeightClamped(x, y - delta, heightMap, gridMap);
  
  scalar_t dz_dx = (h_px - h_nx) / (2.0 * delta);
  scalar_t dz_dy = (h_py - h_ny) / (2.0 * delta);
  
  scalar_t pitch = -std::atan(dz_dx);
  scalar_t roll = std::atan(dz_dy);
  
  const scalar_t max_angle = 0.52;
  pitch = std::max(-max_angle, std::min(max_angle, pitch));
  roll = std::max(-max_angle, std::min(max_angle, roll));
  
  return {pitch, roll};
}

/**
 * Smooth trajectory heights (z) and cargo orientation (pitch, roll) using Gaussian filter.
 * Produces smoother, more continuous trajectories over uneven terrain.
 * First and last waypoints are preserved unchanged.
 */
void smoothTrajectoryHeights(vector_array_t& state_trajectory, size_t cargoStateOffset, size_t numRobots)
{
  const size_t n = state_trajectory.size();
  if (n < 3) return;
  
  // 5-point Gaussian kernel (sigma ~= 1.0 waypoint spacing)
  const int kernel_half = 2;
  const scalar_t kernel[] = {0.06, 0.24, 0.40, 0.24, 0.06};
  
  // Lambda to smooth a specific state component across all waypoints
  auto smoothComponent = [&](size_t stateOffset) {
    std::vector<scalar_t> values(n);
    for (size_t i = 0; i < n; ++i) {
      values[i] = state_trajectory[i](stateOffset);
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
    
    // Preserve first and last waypoints (start/end states)
    for (size_t i = 1; i < n - 1; ++i) {
      state_trajectory[i](stateOffset) = smoothed[i];
    }
  };
  
  // Smooth cargo z, pitch, roll
  smoothComponent(cargoStateOffset + 2);  // cargo z
  smoothComponent(cargoStateOffset + 7);  // cargo pitch
  smoothComponent(cargoStateOffset + 8);  // cargo roll
  
  // Smooth each robot's z
  for (size_t robot = 0; robot < numRobots; ++robot) {
    smoothComponent(robot * SINGLE_ROBOT_STATE_DIM + 2);  // robot z
  }
}

}  // namespace

void terrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg)
{
  if (!gridMapHeightMapPtr) {
    return;
  }
  
  grid_map::GridMap gridMap;
  grid_map::GridMapRosConverter::fromMessage(msg->gridmap, gridMap);
  
  std::string elevationLayer = "elevation";
  if (!gridMap.exists(elevationLayer)) {
    // Try to find an elevation-like layer
    for (const auto& layer : gridMap.getLayers()) {
      if (layer.find("elevation") != std::string::npos) {
        elevationLayer = layer;
        break;
      }
    }
  }
  
  if (gridMap.exists(elevationLayer)) {
    gridMapHeightMapPtr->updateGridMap(gridMap, elevationLayer);
    static bool first_update = true;
    if (first_update) {
      ROS_INFO("[TargetTrajectoriesPublisher] Received first terrain update! Grid map size: %.1f x %.1f m",
               gridMap.getLength().x(), gridMap.getLength().y());
      first_update = false;
    }
  } else {
    ROS_WARN_THROTTLE(5.0, "[TargetTrajectoriesPublisher] No elevation layer found in terrain message!");
  }
}

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  if (!offsets_initialized_from_gazebo) {
    int cargo_idx = -1;
    std::vector<int> robot_indices(num_robots_local, -1);
    
    // Find indices for cargo and robots
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "box_with_handles") {
        cargo_idx = i;
      } else {
        for (size_t robot = 0; robot < num_robots_local; ++robot) {
          if (msg->name[i] == "b1z1_" + std::to_string(robot + 1)) {
            robot_indices[robot] = i;
          }
        }
      }
    }
    
    bool all_found = (cargo_idx >= 0);
    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      if (robot_indices[robot] < 0) all_found = false;
    }
    
    if (all_found) {
      // Get cargo pose
      Eigen::Vector3d cargo_pos(msg->pose[cargo_idx].position.x,
                               msg->pose[cargo_idx].position.y,
                               msg->pose[cargo_idx].position.z);
      
      Eigen::Quaterniond cargo_quat(msg->pose[cargo_idx].orientation.w,
                                   msg->pose[cargo_idx].orientation.x,
                                   msg->pose[cargo_idx].orientation.y,
                                   msg->pose[cargo_idx].orientation.z);
      
      Eigen::Vector3d cargo_euler = cargo_quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
      Eigen::Matrix3d R_world_cargo = cargo_quat.toRotationMatrix().transpose();
      
      // Calculate robot offsets
      for (size_t robot = 0; robot < num_robots_local; ++robot) {
        Eigen::Vector3d robot_pos(msg->pose[robot_indices[robot]].position.x,
                                msg->pose[robot_indices[robot]].position.y,
                                msg->pose[robot_indices[robot]].position.z);
        
        Eigen::Quaterniond robot_quat(msg->pose[robot_indices[robot]].orientation.w,
                                    msg->pose[robot_indices[robot]].orientation.x,
                                    msg->pose[robot_indices[robot]].orientation.y,
                                    msg->pose[robot_indices[robot]].orientation.z);
        
        Eigen::Vector3d robot_euler = robot_quat.toRotationMatrix().eulerAngles(2, 1, 0);
        Eigen::Vector3d robot_relative_pos = R_world_cargo * (robot_pos - cargo_pos);
        Eigen::Vector3d robot_relative_ori = robot_euler - cargo_euler;
        
        ROBOT_BASE_OFFSETS[robot].segment<3>(0) = robot_relative_pos;
        ROBOT_BASE_OFFSETS[robot].segment<3>(3) = robot_relative_ori;
      }
      
      offsets_initialized_from_gazebo = true;
      ROS_INFO("Robot base offsets updated from Gazebo model states");
      for (size_t robot = 0; robot < num_robots_local; ++robot) {
        ROS_INFO("Robot%zu offset: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
                 robot + 1,
                 ROBOT_BASE_OFFSETS[robot](0), ROBOT_BASE_OFFSETS[robot](1), ROBOT_BASE_OFFSETS[robot](2),
                 ROBOT_BASE_OFFSETS[robot](3), ROBOT_BASE_OFFSETS[robot](4), ROBOT_BASE_OFFSETS[robot](5));
      }
    }
  }
}

scalar_t estimateTimeToTarget(const vector_t& desired_base_displacement)
{
  const scalar_t& dx = desired_base_displacement(0);
  const scalar_t& dy = desired_base_displacement(1);
  const scalar_t& dyaw = desired_base_displacement(3);
  const scalar_t rotation_time = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotation_time, displacement_time);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& target_pose, const SystemObservation& observation,
                                                  const scalar_t& target_reaching_time)
{
  const vector_t& current_pose = observation.state;
  const size_t stateDim = current_pose.size();
  const size_t cargoStateOffset = num_robots_local * SINGLE_ROBOT_STATE_DIM;
  
  // Always generate multi-waypoint trajectory so robot bases follow curved paths
  // relative to cargo (e.g. when cargo rotates). Applies to BOTH perceptive and non-perceptive modes.
  const size_t numWaypoints = 20;
  scalar_t total_time = target_reaching_time - observation.time;
  if (total_time <= 0.0) total_time = 0.1;  // Safety minimum
  scalar_t dt = total_time / (numWaypoints - 1);
  
  scalar_array_t time_trajectory(numWaypoints);
  vector_array_t state_trajectory(numWaypoints, vector_t::Zero(stateDim));
  
  // Get grid map for bounds checking and clamping (perceptive mode only)
  const grid_map::GridMap* gridMapPtr = nullptr;
  if (use_perceptive_mode && gridMapHeightMapPtr) {
    static bool printed_warning = false;
    if (!gridMapHeightMapPtr->isUpdated() && !printed_warning) {
      ROS_WARN("[TargetTrajectoriesPublisher] Height map not yet updated with terrain data!");
      printed_warning = true;
    }
    gridMapPtr = &gridMapHeightMapPtr->getGridMap();
  }
  
  for (size_t i = 0; i < numWaypoints; ++i) {
    const scalar_t alpha = static_cast<scalar_t>(i) / (numWaypoints - 1);
    time_trajectory[i] = observation.time + i * dt;
    
    // Linear interpolation between current and target pose
    vector_t waypoint = (1.0 - alpha) * current_pose + alpha * target_pose;
    
    scalar_t cargo_x = waypoint(cargoStateOffset);
    scalar_t cargo_y = waypoint(cargoStateOffset + 1);
    scalar_t cargo_yaw = waypoint(cargoStateOffset + 6);
    
    // 2D rotation matrix for cargo yaw
    Eigen::Matrix2d R_yaw_2d;
    R_yaw_2d << std::cos(cargo_yaw), -std::sin(cargo_yaw),
                std::sin(cargo_yaw),  std::cos(cargo_yaw);
    
    // Compute robot x,y positions relative to cargo (both modes)
    std::vector<Eigen::Vector2d> robot_positions(num_robots_local);
    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      Eigen::Vector2d offset_xy = ROBOT_BASE_OFFSETS[robot].segment<2>(0);
      robot_positions[robot] = Eigen::Vector2d(cargo_x, cargo_y) + R_yaw_2d * offset_xy;
    }
    
    if (use_perceptive_mode && heightMapPtr) {
      // === PERCEPTIVE MODE ===
      // Cargo z = average terrain height at all robot positions + CARGO_HEIGHT_OFFSET
      // For points outside the grid map: clamp query to boundary for smooth connection
      scalar_t avg_terrain = 0.0;
      for (size_t robot = 0; robot < num_robots_local; ++robot) {
        avg_terrain += getTerrainHeightClamped(robot_positions[robot].x(), robot_positions[robot].y(),
                                               heightMapPtr, gridMapPtr);
      }
      avg_terrain /= num_robots_local;
      waypoint(cargoStateOffset + 2) = avg_terrain + CARGO_HEIGHT_OFFSET;
      
      // Set each robot's state and collect terrain heights for cargo orientation
      std::vector<scalar_t> robotTerrainHeights(num_robots_local);
      for (size_t robot = 0; robot < num_robots_local; ++robot) {
        const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
        waypoint(robotStateOffset) = robot_positions[robot].x();
        waypoint(robotStateOffset + 1) = robot_positions[robot].y();
        
        // Robot z from terrain at robot position + COM_HEIGHT (clamped for outside grid map)
        scalar_t terrain_h = getTerrainHeightClamped(robot_positions[robot].x(), robot_positions[robot].y(),
                                                      heightMapPtr, gridMapPtr);
        robotTerrainHeights[robot] = terrain_h;
        waypoint(robotStateOffset + 2) = terrain_h + COM_HEIGHT;
        
        waypoint(robotStateOffset + 6) = cargo_yaw + ROBOT_BASE_OFFSETS[robot](3);
        waypoint(robotStateOffset + 7) = 0.0;  // Robot pitch = 0 (upright)
        waypoint(robotStateOffset + 8) = 0.0;  // Robot roll = 0 (upright)
      }
      
      // Cargo pitch and roll from robot terrain height differences (least-squares plane fit)
      scalar_t cargo_pitch, cargo_roll;
      std::tie(cargo_pitch, cargo_roll) = computeCargoOrientationFromRobotHeights(
          ROBOT_BASE_OFFSETS, robotTerrainHeights, cargo_yaw);
      waypoint(cargoStateOffset + 7) = cargo_pitch;
      waypoint(cargoStateOffset + 8) = cargo_roll;
    } else {
      // === NON-PERCEPTIVE MODE (flat terrain) ===
      // Robot x,y positions computed relative to cargo (curved paths when cargo rotates)
      // z values kept from interpolation (current state -> target state)
      for (size_t robot = 0; robot < num_robots_local; ++robot) {
        const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
        waypoint(robotStateOffset) = robot_positions[robot].x();
        waypoint(robotStateOffset + 1) = robot_positions[robot].y();
        // z stays from interpolation (flat terrain assumption)
        
        waypoint(robotStateOffset + 6) = cargo_yaw + ROBOT_BASE_OFFSETS[robot](3);
        waypoint(robotStateOffset + 7) = 0.0;  // Robot pitch = 0 (upright)
        waypoint(robotStateOffset + 8) = 0.0;  // Robot roll = 0 (upright)
      }
    }
    
    state_trajectory[i] = waypoint;
  }
  
  // Smooth terrain heights along trajectory for curved, continuous motion (perceptive mode only)
  if (use_perceptive_mode && heightMapPtr) {
    smoothTrajectoryHeights(state_trajectory, cargoStateOffset, num_robots_local);
  }
  
  // Velocity limiting: ensure no entity exceeds max velocity in any segment
  // (similar to waypoint mode's velocity limiting across all entities)
  scalar_t max_segment_velocity = 0.0;
  scalar_t max_segment_rot_velocity = 0.0;
  for (size_t i = 1; i < numWaypoints; ++i) {
    scalar_t segment_dt = time_trajectory[i] - time_trajectory[i-1];
    if (segment_dt <= 0.0) continue;
    
    // Check all robots
    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      const size_t off = robot * SINGLE_ROBOT_STATE_DIM;
      scalar_t dx = state_trajectory[i](off) - state_trajectory[i-1](off);
      scalar_t dy = state_trajectory[i](off+1) - state_trajectory[i-1](off+1);
      scalar_t vel = std::sqrt(dx*dx + dy*dy) / segment_dt;
      max_segment_velocity = std::max(max_segment_velocity, vel);
    }
    // Check cargo
    scalar_t cdx = state_trajectory[i](cargoStateOffset) - state_trajectory[i-1](cargoStateOffset);
    scalar_t cdy = state_trajectory[i](cargoStateOffset+1) - state_trajectory[i-1](cargoStateOffset+1);
    scalar_t cvel = std::sqrt(cdx*cdx + cdy*cdy) / segment_dt;
    max_segment_velocity = std::max(max_segment_velocity, cvel);
    
    scalar_t dyaw = std::abs(state_trajectory[i](cargoStateOffset+6) - state_trajectory[i-1](cargoStateOffset+6));
    scalar_t rot_vel = dyaw / segment_dt;
    max_segment_rot_velocity = std::max(max_segment_rot_velocity, rot_vel);
  }
  
  // Stretch time proportionally if any velocity limit is exceeded
  scalar_t stretch_factor = 1.0;
  if (max_segment_velocity > TARGET_DISPLACEMENT_VELOCITY && TARGET_DISPLACEMENT_VELOCITY > 0.0) {
    stretch_factor = std::max(stretch_factor, max_segment_velocity / TARGET_DISPLACEMENT_VELOCITY);
  }
  if (max_segment_rot_velocity > TARGET_ROTATION_VELOCITY && TARGET_ROTATION_VELOCITY > 0.0) {
    stretch_factor = std::max(stretch_factor, max_segment_rot_velocity / TARGET_ROTATION_VELOCITY);
  }
  if (stretch_factor > 1.0) {
    scalar_t new_total_time = total_time * stretch_factor;
    scalar_t new_dt = new_total_time / (numWaypoints - 1);
    for (size_t i = 0; i < numWaypoints; ++i) {
      time_trajectory[i] = observation.time + i * new_dt;
    }
  }
  
  // Desired input trajectory
  const vector_array_t input_trajectory(numWaypoints, vector_t::Zero(observation.input.size()));
  return { time_trajectory, state_trajectory, input_trajectory };
}

vector_t getRobotStateInWorldFrame(const vector_t& robot_base_offset, const vector_t& cargo_base_target_state,
                                   const vector_t& current_robot_state)
{
  Eigen::Vector3d cargo_base_target_pos = cargo_base_target_state.segment<3>(0);  // [x, y, z]
  Eigen::Vector3d cargo_base_target_ori = cargo_base_target_state.segment<3>(6);  // [yaw, pitch, roll]

  // IMPORTANT: Only use cargo YAW for robot position transformation (2D rotation)
  // Robot position should maintain relative x,y to cargo, but NOT follow cargo pitch/roll
  double cargo_yaw = cargo_base_target_ori[0];
  
  // 2D rotation matrix using only yaw
  Eigen::Matrix2d R_yaw_2d;
  R_yaw_2d << std::cos(cargo_yaw), -std::sin(cargo_yaw),
              std::sin(cargo_yaw),  std::cos(cargo_yaw);

  vector_t robot_target_state_world(SINGLE_ROBOT_STATE_DIM);
  
  // Start with current robot state to preserve foot positions and other state variables
  robot_target_state_world = current_robot_state;

  // Extract robot offset in cargo frame (x, y only)
  Eigen::Vector2d offset_xy = robot_base_offset.segment<2>(0);
  Eigen::Vector2d cargo_xy = cargo_base_target_pos.segment<2>(0);

  // Transform to world frame using 2D rotation (yaw only)
  Eigen::Vector2d robot_xy_world = cargo_xy + R_yaw_2d * offset_xy;

  // Set the position in the state vector (x, y from 2D transformation)
  robot_target_state_world.segment<2>(0) = robot_xy_world;
  
  // Keep robot z at its current height - will be adjusted by terrain in targetPoseToTargetTrajectories
  // (already preserved from current_robot_state)
  
  // Set target orientation:
  // - Robot yaw = cargo_yaw + offset_yaw (follow cargo heading)
  // - Robot pitch = 0 (robot stays level, does NOT follow cargo pitch on slopes)
  // - Robot roll = 0 (robot stays level, does NOT follow cargo roll on slopes)
  robot_target_state_world(6) = cargo_yaw + robot_base_offset(3);  // yaw
  robot_target_state_world(7) = 0.0;  // pitch = 0 (robot upright)
  robot_target_state_world(8) = 0.0;  // roll = 0 (robot upright)

  return robot_target_state_world;
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation)
{
  const vector_t current_pose = observation.state;
  const size_t stateDim = num_robots_local * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
  
  const vector_t target_pose = [&]() {
    vector_t target(stateDim);
    target.setZero();
    
    const size_t cargoStateOffset = num_robots_local * SINGLE_ROBOT_STATE_DIM;
    scalar_t target_cargo_x = goal(0);
    scalar_t target_cargo_y = goal(1);
    
    target(cargoStateOffset) = target_cargo_x; // cargo com_x
    target(cargoStateOffset + 1) = target_cargo_y; // cargo com_y
    
    target(cargoStateOffset + 6) = goal(3); // cargo yaw from arrow
    
    // Compute cargo z from average terrain height at all robot positions
    if (use_perceptive_mode && heightMapPtr) {
      const grid_map::GridMap* gm = gridMapHeightMapPtr ? &gridMapHeightMapPtr->getGridMap() : nullptr;
      scalar_t target_yaw = goal(3);
      Eigen::Matrix2d R_2d;
      R_2d << std::cos(target_yaw), -std::sin(target_yaw),
              std::sin(target_yaw),  std::cos(target_yaw);
      scalar_t avg_terrain = 0.0;
      std::vector<scalar_t> robotTerrainHeightsGoal(num_robots_local);
      for (size_t r = 0; r < num_robots_local; ++r) {
        Eigen::Vector2d offset_xy = ROBOT_BASE_OFFSETS[r].segment<2>(0);
        Eigen::Vector2d robot_xy = Eigen::Vector2d(target_cargo_x, target_cargo_y) + R_2d * offset_xy;
        scalar_t rh = getTerrainHeightClamped(robot_xy.x(), robot_xy.y(), heightMapPtr, gm);
        robotTerrainHeightsGoal[r] = rh;
        avg_terrain += rh;
      }
      avg_terrain /= num_robots_local;
      target(cargoStateOffset + 2) = avg_terrain + CARGO_HEIGHT_OFFSET;
      
      // Cargo pitch and roll from robot terrain height differences
      scalar_t cargo_pitch, cargo_roll;
      std::tie(cargo_pitch, cargo_roll) = computeCargoOrientationFromRobotHeights(
          ROBOT_BASE_OFFSETS, robotTerrainHeightsGoal, target_yaw);
      target(cargoStateOffset + 7) = cargo_pitch;
      target(cargoStateOffset + 8) = cargo_roll;
    } else {
      target(cargoStateOffset + 2) = current_pose(cargoStateOffset + 2);  // fallback to current z
    }
    // else: pitch and roll remain 0 (from setZero)
    
    ROS_INFO_THROTTLE(1.0, "[Target] Cargo target: (%.2f, %.2f, %.2f), yaw: %.2f deg, pitch: %.2f deg, roll: %.2f deg",
                      target(cargoStateOffset), target(cargoStateOffset + 1), target(cargoStateOffset + 2),
                      target(cargoStateOffset + 6) * 180.0 / M_PI,
                      target(cargoStateOffset + 7) * 180.0 / M_PI,
                      target(cargoStateOffset + 8) * 180.0 / M_PI);

    vector_t cargo_base_target_state = target.segment<12>(cargoStateOffset);

    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
      vector_t current_robot_state = current_pose.segment<SINGLE_ROBOT_STATE_DIM>(robotStateOffset);
      vector_t robot_target_state = getRobotStateInWorldFrame(ROBOT_BASE_OFFSETS[robot], cargo_base_target_state, current_robot_state);
      target.segment<SINGLE_ROBOT_STATE_DIM>(robotStateOffset) = robot_target_state;
    }

    return target;
  }();
  const scalar_t target_reaching_time = observation.time + estimateTimeToTarget(target_pose - current_pose);
  return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmd_vel, const SystemObservation& observation)
{
  const vector_t current_pose = observation.state;
  const size_t cargoStateOffset = num_robots_local * SINGLE_ROBOT_STATE_DIM;
  const Eigen::Matrix<scalar_t, 3, 1> zyx = current_pose.segment<3>(cargoStateOffset + 6); // Extract the ZYX Euler angles from cargo state
  vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmd_vel.head(3); // Translate from robot frame to world frame

  const scalar_t time_to_target = TIME_TO_TARGET;
  const vector_t target_pose = [&]() {
    const size_t stateDim = num_robots_local * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM;
    vector_t target(stateDim);
    target.setZero();
    
    scalar_t target_cargo_x = current_pose(cargoStateOffset) + cmd_vel(0) * time_to_target;
    scalar_t target_cargo_y = current_pose(cargoStateOffset + 1) + cmd_vel(1) * time_to_target;
    
    target(cargoStateOffset) = target_cargo_x; // cargo com_x
    target(cargoStateOffset + 1) = target_cargo_y; // cargo com_y
    
    scalar_t target_yaw_cmd = current_pose(cargoStateOffset + 6) + cmd_vel(3) * time_to_target;
    target(cargoStateOffset + 6) = target_yaw_cmd; // cargo yaw
    
    // Compute cargo z from average terrain height at all robot positions
    if (use_perceptive_mode && heightMapPtr) {
      const grid_map::GridMap* gm = gridMapHeightMapPtr ? &gridMapHeightMapPtr->getGridMap() : nullptr;
      Eigen::Matrix2d R_2d;
      R_2d << std::cos(target_yaw_cmd), -std::sin(target_yaw_cmd),
              std::sin(target_yaw_cmd),  std::cos(target_yaw_cmd);
      scalar_t avg_terrain = 0.0;
      std::vector<scalar_t> robotTerrainHeightsCmd(num_robots_local);
      for (size_t r = 0; r < num_robots_local; ++r) {
        Eigen::Vector2d offset_xy = ROBOT_BASE_OFFSETS[r].segment<2>(0);
        Eigen::Vector2d robot_xy = Eigen::Vector2d(target_cargo_x, target_cargo_y) + R_2d * offset_xy;
        scalar_t rh = getTerrainHeightClamped(robot_xy.x(), robot_xy.y(), heightMapPtr, gm);
        robotTerrainHeightsCmd[r] = rh;
        avg_terrain += rh;
      }
      avg_terrain /= num_robots_local;
      target(cargoStateOffset + 2) = avg_terrain + CARGO_HEIGHT_OFFSET;
      
      // Cargo pitch and roll from robot terrain height differences
      scalar_t cargo_pitch, cargo_roll;
      std::tie(cargo_pitch, cargo_roll) = computeCargoOrientationFromRobotHeights(
          ROBOT_BASE_OFFSETS, robotTerrainHeightsCmd, target_yaw_cmd);
      target(cargoStateOffset + 7) = cargo_pitch;
      target(cargoStateOffset + 8) = cargo_roll;
    } else {
      target(cargoStateOffset + 2) = current_pose(cargoStateOffset + 2);  // fallback to current z
    }
    // else: pitch and roll remain 0 (from setZero)

    vector_t cargo_base_target_state = target.segment<12>(cargoStateOffset);

    for (size_t robot = 0; robot < num_robots_local; ++robot) {
      const size_t robotStateOffset = robot * SINGLE_ROBOT_STATE_DIM;
      vector_t current_robot_state = current_pose.segment<SINGLE_ROBOT_STATE_DIM>(robotStateOffset);
      vector_t robot_target_state = getRobotStateInWorldFrame(ROBOT_BASE_OFFSETS[robot], cargo_base_target_state, current_robot_state);
      target.segment<SINGLE_ROBOT_STATE_DIM>(robotStateOffset) = robot_target_state;
    }

    return target;
  }();

  // target reaching duration
  const scalar_t target_reaching_time = observation.time + time_to_target;
  return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
}

int main(int argc, char* argv[])
{
  const std::string robotName = "legged_robot";

  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nh;

  // Get node parameters
  std::string taskFile = ros::package::getPath("ocs2_multi_robot") + "/config/info/three_quadruped_w_cargo.info";
  nh.getParam("taskFile", taskFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::string libraryFolder = ros::package::getPath("ocs2_multi_robot") + "/auto_generated";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  // Check if perceptive mode is enabled
  nh.getParam("use_perceptive", use_perceptive_mode);
  if (use_perceptive_mode) {
    ROS_INFO("[TargetTrajectoriesPublisher] Perceptive terrain mode ENABLED");
    // Create height map for terrain-aware trajectory generation
    // Use default height based on initial robot state (will be updated by terrain receiver)
    gridMapHeightMapPtr = std::make_shared<MultiRobotGridMapHeightMap>(0.0);
    heightMapPtr = gridMapHeightMapPtr;  // Store as base class pointer for trajectory functions
    
    // Subscribe to terrain updates
    terrain_sub = nh.subscribe<convex_plane_decomposition_msgs::PlanarTerrain>(
        "/convex_plane_decomposition_ros/planar_terrain", 1, terrainCallback);
    ROS_INFO("[TargetTrajectoriesPublisher] Subscribed to terrain topic");
  } else {
    ROS_INFO("[TargetTrajectoriesPublisher] Perceptive terrain mode DISABLED");
  }

  // Load parameters from configuation file
  MultiRobotWCargoInterface ocp(taskFile, libraryFolder, true, false);
  
  // Determine number of robots from interface (we need to add a getter or determine from config)
  // For now, count handles from config
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const auto& cargoModel = pt.get_child("cargo_model");
  num_robots_local = 0;
  for (const auto& pair : cargoModel) {
    if (pair.first.find("handle_") == 0) {
      num_robots_local++;
    }
  }
  
  SystemObservation observation;
  observation.state = ocp.getInitialState();
  // Compute input dimension from number of robots
  const size_t inputDim = num_robots_local * SINGLE_ROBOT_INPUT_DIM + num_robots_local * ARM_CONTACT_DIM;
  observation.input = vector_t::Zero(inputDim);

  loadData::loadCppDataType(taskFile, "com_height.height", COM_HEIGHT);
  loadData::loadCppDataType(taskFile, "cargo_height_offset.height", CARGO_HEIGHT_OFFSET);
  ROS_INFO("[TargetTrajectoriesPublisher] COM_HEIGHT=%.4f, CARGO_HEIGHT_OFFSET=%.4f", COM_HEIGHT, CARGO_HEIGHT_OFFSET);
  ROBOT_BASE_OFFSETS.resize(num_robots_local);
  
  // Always load default offsets from config file
  for (size_t robot = 0; robot < num_robots_local; ++robot) {
    ROBOT_BASE_OFFSETS[robot] = vector_t(6);
    loadData::loadEigenMatrix(taskFile, "initialStateOffset.robot_" + std::to_string(robot + 1), ROBOT_BASE_OFFSETS[robot]);
  }
  ROS_INFO("Loaded default robot offsets from config file");
  
  // Subscribe to model states - will update offsets if Gazebo is running
  model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, modelStatesCallback);
  ROS_INFO("Subscribed to /gazebo/model_states for dynamic offset updates");
  
  loadData::loadCppDataType(taskFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(taskFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  MultiCentroidalTargetTrajectoriesPublisher target_pose_command(nh, robotName, &goalToTargetTrajectories, 
                                                                  &cmdVelToTargetTrajectories,
                                                                  heightMapPtr, COM_HEIGHT, num_robots_local,
                                                                  CARGO_HEIGHT_OFFSET,
                                                                  TARGET_DISPLACEMENT_VELOCITY,
                                                                  TARGET_ROTATION_VELOCITY);
  
  // Pass robot base offsets and grid map to waypoint generator for cargo-relative robot trajectories
  target_pose_command.setRobotBaseOffsets(ROBOT_BASE_OFFSETS);
  if (gridMapHeightMapPtr) {
    target_pose_command.setGridMap(&gridMapHeightMapPtr->getGridMap());
  }
  
  ROS_INFO("[TargetTrajectoriesPublisher] Ready!");
  ROS_INFO("  Mode switching:");
  ROS_INFO("    - Goal mode (default): Use RViz '2D Nav Goal' arrow");
  ROS_INFO("    - Waypoint mode: rosservice call /load_waypoint_trajectory \"yaml_file: 'waypoints.yaml' trajectory_name: 'gap_slope_turn_course' velocity: 0.3\"");
  ROS_INFO("  Perceptive mode: %s", use_perceptive_mode ? "ENABLED" : "DISABLED");
  ROS_INFO("  Velocities: linear=%.2f m/s, rotational=%.2f rad/s", TARGET_DISPLACEMENT_VELOCITY, TARGET_ROTATION_VELOCITY);

  ros::spin();
  
  return 0;
}

