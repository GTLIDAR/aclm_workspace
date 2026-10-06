#pragma once

// IK
#include <ocs2_multi_robot/kinematics/WholeBodyDifferentialIK.h>
#include <limits>

#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_core/Types.h>
#include "ocs2_multi_robot/common/Types.h"
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include "ocs2_multi_robot/terrain/HeightMapExamples.h"
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>


namespace ocs2 {
namespace multi_robot {

class MultiRobotVisualizer : public DummyObserver {
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
  std::vector<Color> feetColorMap_ = {Color::purple, Color::orange, Color::blue, Color::yellow};
  scalar_t timeHorizon;
  scalar_t currentTime;
  scalar_t storeLen = 400;

  /**
   *
   * @param pinocchioInterface
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  MultiRobotVisualizer(ros::NodeHandle& nodeHandle, std::string taskFile,
                       std::string terrainType = "Flat",
                       scalar_t maxUpdateFrequency = 100.0);

  ~MultiRobotVisualizer() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) override;
  void updateObservation(const SystemObservation& observation);
  void updatePolicy(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command);
  void updateBaseTrasform(const SystemObservation& observation);
  void updateRobotHandlePoses(const std::vector<vector3_t>& handle_positions_world,
          const std::vector<Eigen::Quaternion<scalar_t>>& handle_quaternion_world);

  void updateCentroidalState(const SystemObservation& observation);

  void updateFullBodyState(const SystemObservation& observation);

  void publishInitialState(vector_t& initState);

  void publishTerrain();

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

  void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories);

  void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule);
  
  void publishRealTrajectory(ros::Time timeStamp, const vector_t& stateTrajectory);

 private:
  MultiRobotVisualizer(const MultiRobotVisualizer&) = delete;
  void getTerrainMarker();
  void publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles,
                              const std::string& robot_namespace) const;
  void publishBaseTransform(ros::Time timeStamp, const vector_t& basePose,
                            const std::string& robot_namespace);
  void publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
    const vector3_t& ellip_size, Color color, const std::string& frameName);
  void publishCargoTransform(ros::Time timeStamp, const vector_t& cargoPose, 
    const vector3_t& cargo_size, Color color, const std::string& frameName);
  void publishCartesianMarkers(ros::Time timeStamp, const std::vector<bool>& contactFlags, 
    const std::vector<vector3_t>& feetPositions, const std::vector<vector3_t>& feetForces);
  void publishRobotArmForces(ros::Time timeStamp, const std::vector<vector3_t>& armPositions,
                               const std::vector<vector3_t>& armForces);
  void publishDesiredArmPose(ros::Time timeStamp, const std::vector<vector3_t>& position, 
                                const std::vector< Eigen::Quaternion<scalar_t> >& orientation);
  void PublishHandleTF(const SystemObservation& observation);
  void publishRobotBaseTransformsForElevationMapping(ros::Time timeStamp, const SystemObservation& observation);

  tf::TransformBroadcaster tfBroadcaster_;
  
  // Number of robots (determined dynamically from config)
  size_t numRobots_;
  
  // Joint state publishers for all robots (dynamic)
  std::vector<ros::Publisher> robot_joint_states_publishers_;

  // Kinematics interfaces for all robots (dynamic)
  using WholeBodyIK = legged_kinematics::WholeBodyDifferentialIK;
  std::vector<std::unique_ptr<WholeBodyIK>> wholeBodyIk_;
  std::vector<WholeBodyIK::State> ikStates_;
  std::vector<bool> ikInitialized_;
  scalar_t lastIkTime_ = std::numeric_limits<scalar_t>::lowest();

  // Robot namespaces (dynamic)
  std::vector<std::string> robot_namespaces_;

  // Robot publishers (dynamic)
  std::vector<ros::Publisher> robot_costDesiredBasePositionPublishers_;
  std::vector<ros::Publisher> robot_realTrajectoryPublishers_;
  std::vector<std::vector<ros::Publisher>> robot_realfootPositionPublishers_;
  std::vector<std::vector<ros::Publisher>> robot_costDesiredFeetPositionPublishers_;
  std::vector<ros::Publisher> robot_armDesiredPosePublishers_;

  ros::Publisher stateOptimizedPublisher_;
  ros::Publisher currentStatePublisher_;
  ros::Publisher centroidPublisher_;
  ros::Publisher terrainPublisher_;

  // Cargo target trajectory publisher
  ros::Publisher cargoTargetTrajectoryPub_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;

  // Timing diagnostics: MPC frequency and time scale
  ros::WallTime lastWallTime_{ros::WallTime(0)};
  scalar_t lastSimTime_{0.0};
  int timingLogCounter_{0};
  double wallTimeAccumulator_{0.0};
  double simTimeAccumulator_{0.0};
  int timingStepCount_{0};

  // Visualization throttle: limit RViz publish rate to avoid flooding
  ros::WallTime lastVizWallTime_{ros::WallTime(0)};
  static constexpr double minVizPeriod_ = 1.0 / 30.0;  // 30 Hz max visualization rate

  std::shared_ptr<HeightMap> heightMapPtr_;

  visualization_msgs::MarkerArray terrain_msg;

  visualization_msgs::MarkerArray centroid_msgs;  // Batch markers for centroid/cargo transforms
  visualization_msgs::MarkerArray currentState_msgs;  // Batch markers for cartesian/arm forces

  // [robot + cargo]
  long unsigned int recordTimeIndex_;
  std::vector<std::vector<geometry_msgs::Point>> realTrajectoryPoints_;  // Points for the real trajectory visualization
  std::vector<std::vector<std::vector<geometry_msgs::Point>>> realFootTrajectoryPoints_;  // Points for the desired feet trajectory visualization
  std::string taskFile_;
  std::vector<vector_t> robot_handles_;  // All robot handles (dynamic)
  std::vector<vector3_t> handle_positions_world_; // handle positions in world frame
  std::vector<Eigen::Quaternion<scalar_t>> handle_quaternion_world_; // handle quaternion in world frame
  std::string cargoRootFrame_;  // TF frame for cargo URDF root link (e.g. "/cargo/box" or "/cargo/table")
};

}  // namespace multi_robot
}  // namespace ocs2