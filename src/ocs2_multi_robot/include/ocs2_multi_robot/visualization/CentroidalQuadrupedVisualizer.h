#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_core/Types.h>
#include "ocs2_multi_robot/common/Types.h"
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include "ocs2_multi_robot/terrain/HeightMapExamples.h"
#include <visualization_msgs/MarkerArray.h>

namespace ocs2 {
namespace multi_robot {

class CentroidalQuadrupedVisualizer : public DummyObserver {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";              // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 200.0;              // Vector scale in N/m
  scalar_t velScale_ = 5.0;                   // Vector scale in m/s
  scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
  std::vector<Color> feetColorMap_ = {Color::blue, Color::orange, Color::yellow, Color::purple};  // Colors for markers per feet

  /**
   *
   * @param pinocchioInterface
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  CentroidalQuadrupedVisualizer(ros::NodeHandle& nodeHandle, std::string terrainType = "Flat",
                                scalar_t maxUpdateFrequency = 100.0,
                                const vector_t& r1_handle = vector_t::Zero(6),
                                const vector_t& r2_handle = vector_t::Zero(6));

  ~CentroidalQuadrupedVisualizer() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;

  void updateCentroidalState(const SystemObservation& observation);

  void updateCentroidalState(const SystemObservation& observation, const vector_t& desiredState);

  void updateFullBodyState(const SystemObservation& observation);

  void publishDesiredState(ros::Time timeStamp, const vector_t& desiredState);

  void publishInitialState(vector_t& initState);

  void publishTerrain();

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishObservation(ros::Time timeStamp, const SystemObservation& observation);

  // void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

  // void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
  //                                      const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);

 private:
  CentroidalQuadrupedVisualizer(const CentroidalQuadrupedVisualizer&) = delete;
  void getTerrainMarker();
  void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const;
  void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);
  void publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
    const vector3_t& ellip_size, Color color, const std::string& frameName);
  void publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
    const vector3_t& ellip_size, Color color, float alpha, const std::string& frameName);
  void publishCargoTransform(ros::Time timeStamp, const vector_t& cargoPose, 
    const vector3_t& cargo_size, Color color, const std::string& frameName);
  void publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags, const std::vector<vector3_t>& feetPositions,
                               const std::vector<vector3_t>& feetForces);
  void publishRobotArmForces(ros::Time timeStamp, const std::vector<vector3_t>& armPositions,
                               const std::vector<vector3_t>& armForces);
  void publishRobotBaseTransforms(ros::Time timeStamp, size_t numRobots, const SystemObservation& observation);

  tf::TransformBroadcaster tfBroadcaster_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  ros::Publisher costDesiredBasePositionPublisher_;
  std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;

  ros::Publisher stateOptimizedPublisher_;

  ros::Publisher currentStatePublisher_;

  ros::Publisher centroidPublisher_;

  ros::Publisher terrainPublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;

  std::shared_ptr<HeightMap> heightMapPtr_;

  visualization_msgs::MarkerArray terrain_msg;

  visualization_msgs::MarkerArray centroid_msgs;
  visualization_msgs::MarkerArray currentState_msgs;  // Batch markers for cartesian/arm forces

  vector_t r1_handle_;
  vector_t r2_handle_;
};

}  // namespace multi_robot
}  // namespace ocs2
