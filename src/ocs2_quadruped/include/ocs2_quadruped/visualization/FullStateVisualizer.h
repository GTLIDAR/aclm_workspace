#pragma once

#include <robot_state_publisher/robot_state_publisher.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_quadruped/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include "ocs2_quadruped/FullState.h"

namespace ocs2 {
namespace quadruped {

class FullStateVisualizer {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";              // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 1000.0;              // Vector scale in N/m
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
  FullStateVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                        const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                        scalar_t maxUpdateFrequency = 100.0);

  ~FullStateVisualizer() = default;

  void update(const SystemObservation& observation, const vector_t& state_desired, const vector_t& input_desired, 
              const PrimalSolution& primalSolution, const CommandData& command);

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishTrajectory(const std::vector<SystemObservation>& system_observation_array, scalar_t speed = 1.0);

  void publishObservation(ros::Time timeStamp, const SystemObservation& observation);

  void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

  void publishDesiredStates(ros::Time timestamp, const vector_t& state_desired, const vector_t& input_desired);

  void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);

 private:
  FullStateVisualizer(const FullStateVisualizer&) = delete;
  void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const;
  void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose);
  void publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags, const std::vector<vector3_t>& feetPositions,
                               const std::vector<vector3_t>& feetForces) const;
  void publishFullStates(ros::Time timestamp, const vector_t& jointAngles, const vector_t &jointVelocities, 
                         const vector_t& basePose, const std::vector<vector3_t>& feetPositions, const std::vector<vector3_t>& feetForces) const;

  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  std::unique_ptr<PinocchioEndEffectorKinematics> endEffectorKinematicsPtr_;

  tf::TransformBroadcaster tfBroadcaster_;
  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  ros::Publisher costDesiredBasePositionPublisher_;
  std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;

  ros::Publisher stateOptimizedPublisher_;

  ros::Publisher currentStatePublisher_;

  ros::Publisher fullStatePublisher_;

  ros::Publisher desiredStatePublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace quadruped
}  // namespace ocs2
