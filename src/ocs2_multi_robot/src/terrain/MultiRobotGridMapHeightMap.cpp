#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include "ocs2_multi_robot/reference_manager/PerceptiveMultiRobotReferenceManager.h"

#include <chrono>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>

namespace ocs2 {
namespace multi_robot {

MultiRobotGridMapHeightMap::MultiRobotGridMapHeightMap(double defaultHeight) 
    : defaultHeight_(defaultHeight) {
  // Initialize with a default flat terrain at specified height
  // Use larger geometry to cover typical multi-robot operating area
  gridMap_.setGeometry(grid_map::Length(100.0, 100.0), 0.05);
  elevationLayer_ = "elevation";
  gridMap_.add(elevationLayer_, defaultHeight);
}

double MultiRobotGridMapHeightMap::GetHeight(double x, double y) const {
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (!gridMap_.isInside(grid_map::Position(x, y))) {
    return defaultHeight_;  // Return default height outside map bounds
  }
  
  try {
    return gridMap_.atPosition(elevationLayer_, grid_map::Position(x, y));
  } catch (const std::out_of_range& e) {
    return defaultHeight_;
  }
}

void MultiRobotGridMapHeightMap::updateGridMap(const grid_map::GridMap& gridMap, const std::string& elevationLayer) {
  std::lock_guard<std::mutex> lock(mutex_);
  gridMap_ = gridMap;
  elevationLayer_ = elevationLayer;
  updated_ = true;
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtX(double x, double y) const {
  // Use delta larger than grid resolution to ensure we sample across cell boundaries
  const double delta = 0.05;  // 5cm, should be >= grid resolution
  return (GetHeight(x + delta, y) - GetHeight(x - delta, y)) / (2.0 * delta);
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtY(double x, double y) const {
  // Use delta larger than grid resolution to ensure we sample across cell boundaries
  const double delta = 0.05;  // 5cm, should be >= grid resolution
  return (GetHeight(x, y + delta) - GetHeight(x, y - delta)) / (2.0 * delta);
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtXX(double x, double y) const {
  const double delta = 0.05;  // Match first-order derivative delta
  return (GetHeight(x + delta, y) - 2.0 * GetHeight(x, y) + GetHeight(x - delta, y)) / (delta * delta);
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtXY(double x, double y) const {
  const double delta = 0.05;  // Match first-order derivative delta
  return (GetHeight(x + delta, y + delta) - GetHeight(x + delta, y - delta) 
         - GetHeight(x - delta, y + delta) + GetHeight(x - delta, y - delta)) / (4.0 * delta * delta);
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtYX(double x, double y) const {
  return GetHeightDerivWrtXY(x, y);
}

double MultiRobotGridMapHeightMap::GetHeightDerivWrtYY(double x, double y) const {
  const double delta = 0.05;  // Match first-order derivative delta
  return (GetHeight(x, y + delta) - 2.0 * GetHeight(x, y) + GetHeight(x, y - delta)) / (delta * delta);
}

// MultiRobotPlanarTerrainReceiverModule implementation
MultiRobotPlanarTerrainReceiverModule::MultiRobotPlanarTerrainReceiverModule(
    ros::NodeHandle& nh, 
    std::shared_ptr<MultiRobotGridMapHeightMap> heightMapPtr,
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
    const std::string& terrainTopic,
    const std::string& elevationLayer)
    : heightMapPtr_(std::move(heightMapPtr)), 
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      elevationLayer_(elevationLayer) {
  // Create planar terrain if not provided
  if (!planarTerrainPtr_) {
    planarTerrainPtr_ = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
  }
  subscriber_ = nh.subscribe(terrainTopic, 1, &MultiRobotPlanarTerrainReceiverModule::terrainCallback, this);
  ROS_INFO_STREAM("[MultiRobotPlanarTerrainReceiverModule] Subscribing to: " << terrainTopic);
  
  // Initialize SDF visualization publishers
  sdfFullPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf/full", 1);
  sdfFreeSpacePublisher_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf/free_space", 1);
  sdfOccupiedPublisher_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf/occupied", 1);
  ROS_INFO_STREAM("[MultiRobotPlanarTerrainReceiverModule] SDF visualization enabled on /sdf/*");
}

void MultiRobotPlanarTerrainReceiverModule::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, 
                                                          const vector_t& /*currentState*/,
                                                          const ReferenceManagerInterface& referenceManager) {
  if (newDataReceived_) {
    std::lock_guard<std::mutex> lock(mutex_);
    newDataReceived_ = false;

    const auto tTotal0 = std::chrono::steady_clock::now();

    // Update the height map with the new grid map data
    if (heightMapPtr_) {
      const auto t0 = std::chrono::steady_clock::now();
      heightMapPtr_->updateGridMap(receivedTerrain_.gridMap, elevationLayer_);
      const double dtMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      ROS_INFO_THROTTLE(2.0, "[TerrainTiming] HeightMap update: %.2f ms", dtMs);
    }
    
    // Update the planar terrain for MultiRobotConvexRegionSelector
    if (planarTerrainPtr_) {
      const auto t0 = std::chrono::steady_clock::now();
      *planarTerrainPtr_ = receivedTerrain_;
      const double dtMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      ROS_INFO_THROTTLE(2.0, "[TerrainTiming] PlanarTerrain copy: %.2f ms | %zu planar regions", 
                        dtMs, receivedTerrain_.planarRegions.size());
    }

    // Update SDF on the reference manager for foot collision constraints
    if (receivedTerrain_.gridMap.exists(elevationLayer_)) {
      try {
        // Cast to PerceptiveMultiRobotReferenceManager to access SDF update
        auto* perceptiveRefMgr = dynamic_cast<const PerceptiveMultiRobotReferenceManager*>(&referenceManager);
        if (perceptiveRefMgr) {
          // Compute SDF height bounds from actual terrain elevation range
          // minHeight/maxHeight define the ABSOLUTE z-coordinate range of the 3D SDF grid
          const auto& elevationData = receivedTerrain_.gridMap.get(elevationLayer_);
          const float terrainMin = elevationData.minCoeffOfFinites();
          const float terrainMax = elevationData.maxCoeffOfFinites();
          const auto& gridSize = receivedTerrain_.gridMap.getSize();
          
          // Check if terrain has actually changed — skip expensive SDF recomputation if not
          const float boundsTol = 1e-3f;
          const bool boundsChanged = std::abs(terrainMin - prevTerrainMin_) > boundsTol ||
                                     std::abs(terrainMax - prevTerrainMax_) > boundsTol ||
                                     (gridSize != prevGridSize_).any();
          
          if (boundsChanged || !cachedSdfPtr_) {
            // SDF should extend from below lowest terrain to well above highest terrain
            // This allows collision checking for feet in the free space above terrain
            const float lowerMargin = 0.1f;   // Small margin below lowest terrain point
            const float upperMargin = 1.0f;   // Large margin above highest terrain for robot COM height + swing
            const float minHeight = terrainMin - lowerMargin;
            const float maxHeight = terrainMax + upperMargin;
            
            // Compute SDF from elevation map with dynamic height bounds
            const auto tSdf0 = std::chrono::steady_clock::now();
            cachedSdfPtr_ = std::make_shared<grid_map::SignedDistanceField>(
                receivedTerrain_.gridMap, elevationLayer_, minHeight, maxHeight);
            const double sdfMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tSdf0).count();
            
            const int numZLayers = static_cast<int>(std::ceil((maxHeight - minHeight) / receivedTerrain_.gridMap.getResolution()));
            ROS_INFO_THROTTLE(2.0, "[TerrainTiming] SDF computation: %.2f ms | grid %dx%d, Z layers %d (range %.2f to %.2f, span %.2f m)",
                              sdfMs, gridSize(0), gridSize(1), numZLayers, minHeight, maxHeight, maxHeight - minHeight);
            
            prevTerrainMin_ = terrainMin;
            prevTerrainMax_ = terrainMax;
            prevGridSize_ = gridSize;
          } else {
            ROS_INFO_THROTTLE(5.0, "[TerrainTiming] SDF cached (terrain unchanged), skipping recomputation");
          }
          
          // Update the reference manager (const_cast needed since preSolverRun takes const ref)
          const_cast<PerceptiveMultiRobotReferenceManager*>(perceptiveRefMgr)->updateSDF(cachedSdfPtr_);
          
          // Publish SDF visualization if enabled (WARNING: very expensive ~70ms for 2M+ voxels)
          if (visualizeSDF_ && cachedSdfPtr_) {
            const auto tViz0 = std::chrono::steady_clock::now();
            publishSDFVisualization(*cachedSdfPtr_);
            const double vizMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tViz0).count();
            ROS_INFO_THROTTLE(2.0, "[TerrainTiming] SDF visualization: %.2f ms", vizMs);
          }
        }
      } catch (const std::exception& e) {
        ROS_WARN_THROTTLE(5.0, "[MultiRobotPlanarTerrainReceiverModule] Failed to create SDF: %s", e.what());
      }
    }

    const double totalMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tTotal0).count();
    ROS_INFO_THROTTLE(2.0, "[TerrainTiming] preSolverRun TOTAL: %.2f ms", totalMs);
  }
}

void MultiRobotPlanarTerrainReceiverModule::publishSDFVisualization(const grid_map::SignedDistanceField& sdf) {
  sensor_msgs::PointCloud2 pointCloud2Msg;
  
  // Publish full SDF point cloud
  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg);
  sdfFullPublisher_.publish(pointCloud2Msg);
  
  // Publish free space (positive SDF values - points away from obstacles)
  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue > 0.0; });
  sdfFreeSpacePublisher_.publish(pointCloud2Msg);
  
  // Publish occupied space (negative or zero SDF values - inside/on obstacles)
  grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float sdfValue) { return sdfValue <= 0.0; });
  sdfOccupiedPublisher_.publish(pointCloud2Msg);
}

void MultiRobotPlanarTerrainReceiverModule::terrainCallback(
    const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  newDataReceived_ = true;
  
  ROS_DEBUG_THROTTLE(2.0, "[MultiRobotPlanarTerrainReceiverModule] Received terrain message with %zu planar regions", 
                    msg->planarRegions.size());

  // Convert ROS message to PlanarTerrain
  receivedTerrain_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));

  // Handle NaN values in elevation data
  if (receivedTerrain_.gridMap.exists(elevationLayer_)) {
    auto& elevationData = receivedTerrain_.gridMap.get(elevationLayer_);
    if (elevationData.hasNaN()) {
      const float inpaint = elevationData.minCoeffOfFinites();
      ROS_WARN_THROTTLE(5.0, "[MultiRobotPlanarTerrainReceiverModule] Map contains NaN values. Applying inpainting.");
      elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
    }
  }
}

}  // namespace multi_robot
}  // namespace ocs2
