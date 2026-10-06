#pragma once

#include <cmath>
#include <limits>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include "ocs2_multi_robot/terrain/HeightMap.h"

namespace ocs2 {
namespace multi_robot {

/**
 * Grid map-based height map that receives terrain from convex_plane_decomposition
 * for multi-robot systems
 */
class MultiRobotGridMapHeightMap : public HeightMap {
 public:
  MultiRobotGridMapHeightMap(double defaultHeight = 0.0);

  ~MultiRobotGridMapHeightMap() override = default;

  double GetHeight(double x, double y) const override;

  void updateGridMap(const grid_map::GridMap& gridMap, const std::string& elevationLayer);

  bool isUpdated() const { return updated_; }
  
  void setUpdated(bool updated) { updated_ = updated; }

  const grid_map::GridMap& getGridMap() const { return gridMap_; }

 protected:
  double GetHeightDerivWrtX(double x, double y) const override;
  double GetHeightDerivWrtY(double x, double y) const override;
  double GetHeightDerivWrtXX(double x, double y) const override;
  double GetHeightDerivWrtXY(double x, double y) const override;
  double GetHeightDerivWrtYX(double x, double y) const override;
  double GetHeightDerivWrtYY(double x, double y) const override;

 private:
  grid_map::GridMap gridMap_;
  std::string elevationLayer_;
  std::atomic<bool> updated_{false};
  mutable std::mutex mutex_;
  double defaultHeight_{0.0};
};

/**
 * Synchronized module that receives PlanarTerrain messages and updates both
 * GridMapHeightMap and PlanarTerrain for MultiRobotConvexRegionSelector
 */
class MultiRobotPlanarTerrainReceiverModule : public SolverSynchronizedModule {
 public:
  MultiRobotPlanarTerrainReceiverModule(ros::NodeHandle& nh, 
                                         std::shared_ptr<MultiRobotGridMapHeightMap> heightMapPtr,
                                         std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr = nullptr,
                                         const std::string& terrainTopic = "/convex_plane_decomposition_ros/planar_terrain",
                                         const std::string& elevationLayer = "elevation");

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {}

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() { return planarTerrainPtr_; }

  /** Set SDF height bounds for 3D collision detection */
  void setSDFHeightBounds(double minHeight, double maxHeight) {
    sdfMinHeight_ = minHeight;
    sdfMaxHeight_ = maxHeight;
  }

  /** Enable/disable SDF visualization */
  void enableSDFVisualization(bool enable) { visualizeSDF_ = enable; }

 private:
  void terrainCallback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg);
  
  /** Publish SDF as point cloud for visualization */
  void publishSDFVisualization(const grid_map::SignedDistanceField& sdf);

  ros::Subscriber subscriber_;
  std::shared_ptr<MultiRobotGridMapHeightMap> heightMapPtr_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::string elevationLayer_;

  convex_plane_decomposition::PlanarTerrain receivedTerrain_;
  std::mutex mutex_;
  std::atomic<bool> newDataReceived_{false};

  // SDF height bounds for 3D collision detection
  double sdfMinHeight_{-0.5};
  double sdfMaxHeight_{0.5};

  // SDF caching — skip recomputation when terrain hasn't changed
  std::shared_ptr<grid_map::SignedDistanceField> cachedSdfPtr_;
  float prevTerrainMin_{std::numeric_limits<float>::quiet_NaN()};
  float prevTerrainMax_{std::numeric_limits<float>::quiet_NaN()};
  Eigen::Array2i prevGridSize_{0, 0};

  // SDF visualization (disabled by default — costs ~70ms per call, blocks MPC solver)
  bool visualizeSDF_{false};
  ros::Publisher sdfFullPublisher_;
  ros::Publisher sdfFreeSpacePublisher_;
  ros::Publisher sdfOccupiedPublisher_;
};

}  // namespace multi_robot
}  // namespace ocs2
