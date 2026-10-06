#include "ocs2_multi_robot/visualization/CentroidalQuadrupedVisualizer.h"

// OCS2
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// Additional messages not in the helpers file
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace ocs2 {
namespace multi_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalQuadrupedVisualizer::CentroidalQuadrupedVisualizer(ros::NodeHandle& nodeHandle,
                                                             std::string terrainType,
                                                             scalar_t maxUpdateFrequency,
                                                             const vector_t& r1_handle,
                                                             const vector_t& r2_handle)
    : lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency),
      r1_handle_(r1_handle),
      r2_handle_(r2_handle) {
  launchVisualizerNode(nodeHandle);
  heightMapPtr_ = HeightMap::MakeTerrain(terrain_mapping[terrainType]);
  getTerrainMarker();
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  // costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/desiredBaseTrajectory", 1);
  // costDesiredFeetPositionPublishers_.resize(QUADRUPED_FOOT_NUM);
  // costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/desiredFeetTrajectory/LF", 1);
  // costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/desiredFeetTrajectory/RF", 1);
  // costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/desiredFeetTrajectory/LH", 1);
  // costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("/cen_opt/desiredFeetTrajectory/RH", 1);
  // stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/optimizedStateTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/currentState", 1);
  centroidPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/centroid", 1);
  terrainPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/cen_opt/terrain", 1);
  
  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParam("robot_description")) {
    std::cerr << "[LeggedRobotVisualizer] Could not read URDF from: \"robot_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
}

void CentroidalQuadrupedVisualizer::publishInitialState(vector_t& initState) {
  const auto timeStamp = ros::Time::now();
  vector_t centroidPose1(6);
  vector_t centroidPose2(6);
  centroidPose1 << initState.segment(0, 3), initState.segment(6, 3);
  centroidPose2 << initState.segment(24, 3), initState.segment(30, 3);
  vector3_t ellip_size;
  ellip_size << 0.4, 0.2, 0.2;
  publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::red, 0.4, "robot_1_init");
  publishCentroidTransform(timeStamp, centroidPose2, ellip_size, Color::green, 0.4, "robot_2_init");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::updateCentroidalState(const SystemObservation& observation) {
  const auto timeStamp = ros::Time::now();

  centroid_msgs.markers.clear();
  currentState_msgs.markers.clear();
  
  // Extract components from state
  vector_t centroidPose1(6);
  vector_t centroidPose2(6);
  centroidPose1 << observation.state.segment(0, 3), observation.state.segment(6, 3);
  centroidPose2 << observation.state.segment(24, 3), observation.state.segment(30, 3);
  
  // Compute cartesian state and inputs
  std::vector<vector3_t> feetPositions(ROBOTS_FOOT_NUM);
  std::vector<vector3_t> feetForces(ROBOTS_FOOT_NUM);
  for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
    feetForces[i] = observation.input.segment(i*3, 3);
    // std::cout << "Robot 1 Feet forces: " << feetForces[i].transpose() << std::endl;
    feetPositions[i] = observation.state.segment(12 + i*3, 3);
  }
  for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
    feetForces[i+4] = observation.input.segment(SINGLE_ROBOT_INPUT_DIM + i*3, 3);
    // std::cout << "Robot 2 Feet forces: " << feetForces[i + 4].transpose() << std::endl;
    feetPositions[i+4] = observation.state.segment(SINGLE_ROBOT_STATE_DIM + 12 + i*3, 3);
  }

  // Publish
  // Hardcoded contact state for now
  publishCartesianMarkers(timeStamp, contact_flag_t{true, true, true, true, true, true, true, true}, feetPositions, feetForces);

  // Publish equimomental ellipsoid
  if (observation.state.size() == STATE_DIM_AE_WITH_ELLIPSOID) {
    vector_t ellipsoidPose(6);
    ellipsoidPose << observation.state.head(3), observation.state.tail(3);
    const vector3_t ellip_size = observation.state.segment(24, 3);
    publishCentroidTransform(timeStamp, ellipsoidPose, ellip_size, Color::blue, "equimomental_ellipsoid");
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::yellow, "angular_excursion");
  } else if (observation.state.size() == STATE_DIM_ELLIPSOID) {
    const vector3_t ellip_size = observation.state.segment(24, 3);
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::blue, "equimomental_ellipsoid");    
  } else if (observation.state.size() == STATE_DIM_AE) {
    vector3_t ellip_size;
    ellip_size << 0.4, 0.2, 0.2;
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::yellow, "robot1");
    publishCentroidTransform(timeStamp, centroidPose2, ellip_size, Color::blue, "robot2");
  } else {
    vector3_t robot_ellip_size;
    robot_ellip_size << 0.4, 0.2, 0.2;
    publishCentroidTransform(timeStamp, centroidPose1, robot_ellip_size, Color::yellow, "robot1");
    publishCentroidTransform(timeStamp, centroidPose2, robot_ellip_size, Color::blue, "robot2");
    vector_t cargoPose(6);
    cargoPose << observation.state.segment(48, 3), observation.state.segment(54, 3);
    vector3_t cargo_cube_size;
    cargo_cube_size << 1.0, 1.0, 1.0;
    publishCargoTransform(timeStamp, cargoPose, cargo_cube_size, Color::red, "cargo");

     // Compute arm EE target in each robot's z1 base frame and solve IK (single step)
    // Cargo pose in world
    const vector3_t cargo_pos_world = cargoPose.head<3>();
    const vector3_t cargo_eul_zyx = cargoPose.tail<3>();
    const matrix3_t w_R_cargo = getRotationMatrixFromZyxEulerAngles<scalar_t>(cargo_eul_zyx);

    // Handle poses in cargo frame (6D: x y z yaw pitch roll)
    vector3_t r1_pos_cargo = r1_handle_.head<3>();
    vector3_t r2_pos_cargo = r2_handle_.head<3>();
    matrix3_t cargo_R_r1 = getRotationMatrixFromZyxEulerAngles<scalar_t>(r1_handle_.tail<3>());
    matrix3_t cargo_R_r2 = getRotationMatrixFromZyxEulerAngles<scalar_t>(r2_handle_.tail<3>());

    // World poses of handles (used as arm EE targets)
    const vector3_t r1_pos_world = cargo_pos_world + w_R_cargo * r1_pos_cargo;
    const vector3_t r2_pos_world = cargo_pos_world + w_R_cargo * r2_pos_cargo;
    const matrix3_t w_R_r1 = w_R_cargo * cargo_R_r1;
    const matrix3_t w_R_r2 = w_R_cargo * cargo_R_r2;

    // For debugging: publish handle positions to tf
    geometry_msgs::TransformStamped r1ToWorldTransform;
    r1ToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    r1ToWorldTransform.child_frame_id = "handle_1";
    r1ToWorldTransform.transform.translation = getVectorMsg(r1_pos_world);
    Eigen::Quaternion<scalar_t> q_world_r1(w_R_r1);
    r1ToWorldTransform.transform.rotation = getOrientationMsg(q_world_r1);
    tfBroadcaster_.sendTransform(r1ToWorldTransform);
    geometry_msgs::TransformStamped r2ToWorldTransform;
    r2ToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
    r2ToWorldTransform.child_frame_id = "handle_2";
    r2ToWorldTransform.transform.translation = getVectorMsg(r2_pos_world);
    Eigen::Quaternion<scalar_t> q_world_r2(w_R_r2);
    r2ToWorldTransform.transform.rotation = getOrientationMsg(q_world_r2);
    tfBroadcaster_.sendTransform(r2ToWorldTransform);

    // Publish the arm forces
    std::vector<vector3_t> armPositions = {r1_pos_world, r2_pos_world};
    std::vector<vector3_t> armForces;
    armForces.push_back(observation.input.segment(48, 3)); // input[24:27] is force on handle 1
    armForces.push_back(observation.input.segment(54, 3)); // input[27:30] is force on handle 2
    publishRobotArmForces(timeStamp, armPositions, armForces);
    
    // Publish base transforms for elevation mapping (perceptive terrain mode)
    // Determine number of robots from state dimension
    const size_t numRobots = (observation.state.size() - CARGO_STATE_DIM) / SINGLE_ROBOT_STATE_DIM;
    publishRobotBaseTransforms(timeStamp, numRobots, observation);
  }
  centroidPublisher_.publish(centroid_msgs);
  // Publish all batched cartesian markers at once
  currentStatePublisher_.publish(currentState_msgs); 
}
// enum class Color { blue, orange, yellow, purple, green, red, black };
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::updateCentroidalState(const SystemObservation& observation,
                                                          const vector_t& desiredState) {
  const auto timeStamp = ros::Time::now();

  centroid_msgs.markers.clear();
  // Extract components from state
  vector_t centroidPose1(6);
  vector_t centroidPose2(6);
  centroidPose1 << observation.state.segment(0, 3), observation.state.segment(6, 3);
  centroidPose2 << observation.state.segment(24, 3), observation.state.segment(30, 3);
  
  // Compute cartesian state and inputs
  std::vector<vector3_t> feetPositions(ROBOTS_FOOT_NUM);
  std::vector<vector3_t> feetForces(ROBOTS_FOOT_NUM);
  for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
    feetForces[i] = observation.input.segment(i*3, 3);
    feetPositions[i] = observation.state.segment(12 + i*3, 3);
  }
  for (size_t i = 0; i < QUADRUPED_FOOT_NUM; i++) {
    feetForces[i + 4] = observation.input.segment(SINGLE_ROBOT_INPUT_DIM + i*3, 3);
    feetPositions[i + 4] = observation.state.segment(SINGLE_ROBOT_STATE_DIM + 12 + i*3, 3);
  }
  // Publish
  // Hardcoded contact state for now
  publishCartesianMarkers(timeStamp, contact_flag_t{true, true, true, true, true, true, true, true}, feetPositions, feetForces);

  // Publish equimomental ellipsoid
  if (observation.state.size() == STATE_DIM_AE_WITH_ELLIPSOID) {
    vector_t ellipsoidPose(6);
    ellipsoidPose << observation.state.head(3), observation.state.tail(3);
    const vector3_t ellip_size = observation.state.segment(24, 3);
    publishCentroidTransform(timeStamp, ellipsoidPose, ellip_size, Color::blue, "equimomental_ellipsoid");
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::yellow, "angular_excursion");
    publishDesiredState(timeStamp, desiredState);
    
  } else if (observation.state.size() == STATE_DIM_ELLIPSOID) {
    const vector3_t ellip_size = observation.state.segment(24, 3);
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::blue, "equimomental_ellipsoid");    
  } else {
    vector3_t ellip_size;
    ellip_size << 0.4, 0.2, 0.2;
    publishCentroidTransform(timeStamp, centroidPose1, ellip_size, Color::yellow, "angular_excursion");
    publishCentroidTransform(timeStamp, centroidPose2, ellip_size, Color::yellow, "equimomental_ellipsoid");
  }
  centroidPublisher_.publish(centroid_msgs);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishDesiredState(ros::Time timeStamp, const vector_t& desiredState) {
  if (desiredState.rows() != STATE_DIM_AE_WITH_ELLIPSOID) {
    throw std::runtime_error("state size is wrong!");
  }
  // Extract ellipsoid components from state
  vector_t ellipsoidPose(6);
  ellipsoidPose << desiredState.head(3), desiredState.tail(3);
  const vector3_t ellip_size = desiredState.segment(24, 3);

  publishCentroidTransform(timeStamp, ellipsoidPose, ellip_size, Color::red, "desired_ellipsoid");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::updateFullBodyState(const SystemObservation& observation) {
  const auto timeStamp = ros::Time::now();
  
  // Extract components from state
  vector_t basePose = observation.state.head(7);
  vector_t qJoints = observation.state.segment(7, 12);
  
  // Publish
  publishBaseTransform(timeStamp, basePose);
  // Hardcoded contact state for now
  publishJointTransforms(timeStamp, qJoints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) {
  const auto timeStamp = ros::Time::now();
  throw std::runtime_error("Not implemented yet");
  // publishObservation(timeStamp, observation);
  // Disabled for now; need modifications in the future when running mpc
  // publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);
  // publishOptimizedStateTrajectory(timeStamp, primalSolution.timeTrajectory_, primalSolution.stateTrajectory_,
  //                                 primalSolution.modeSchedule_);
}

void CentroidalQuadrupedVisualizer::publishTerrain() {
  terrainPublisher_.publish(terrain_msg);
}

void CentroidalQuadrupedVisualizer::getTerrainMarker() {
  // x-y area patch that should be drawn in rviz
  double dxy   =  0.06;
  double x_min = -1.0;
  double x_max =  4.0;
  double y_min = -1.0;
  double y_max =  1.0;

  visualization_msgs::Marker m;
  int id = 0;
  m.type = visualization_msgs::Marker::CUBE;
  m.scale.z = 0.003;
  m.ns = "terrain";
  m.header.frame_id = frameId_;
//    m.color.r = 245./355; m.color.g  = 222./355; m.color.b  = 179./355; // wheat
  m.color.r = 255./255; m.color.g  = 204./255; m.color.b  = 204./255; // pink
  m.color.a = 0.7;

  visualization_msgs::MarkerArray msg;
  double x =  x_min;
  while (x < x_max) {
    double y = y_min;
    while (y < y_max) {
    // position
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = heightMapPtr_->GetHeight(x,y);

    // orientation
    Eigen::Vector3d n = heightMapPtr_->GetNormalizedBasis(HeightMap::Normal, x, y);
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), n);
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();

    // enlarge surface-path when tilting
    double gain = 1.5;
    m.scale.x = (1+gain*n.cwiseAbs().x())*dxy;
    m.scale.y = (1+gain*n.cwiseAbs().y())*dxy;


    m.id = id++;
    terrain_msg.markers.push_back(m);

    y += dxy;
    }
    x += dxy;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
  const vector3_t& ellip_size, Color color, const std::string& frameName) {
  geometry_msgs::TransformStamped centroidToWorldTransform;
  centroidToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  centroidToWorldTransform.child_frame_id = frameName;

  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(centroidPose.tail<3>()));
  centroidToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  centroidToWorldTransform.transform.translation = getVectorMsg(centroidPose.head<3>());
  tfBroadcaster_.sendTransform(centroidToWorldTransform);

  // Add ellipsoid marker to the batch (don't publish immediately)
  visualization_msgs::Marker ellipsoid;
  ellipsoid.header = getHeaderMsg(frameId_, timeStamp);
  ellipsoid.type = visualization_msgs::Marker::SPHERE;
  ellipsoid.pose.position.x = centroidPose[0];
  ellipsoid.pose.position.y = centroidPose[1];
  ellipsoid.pose.position.z = centroidPose[2];
  ellipsoid.pose.orientation = getOrientationMsg(q_world_base);

  ellipsoid.scale.x = ellip_size[0];
  ellipsoid.scale.y = ellip_size[1];
  ellipsoid.scale.z = ellip_size[2];

  const auto rgb = getRGB(color);
  ellipsoid.color.r = rgb[0];
  ellipsoid.color.g = rgb[1];
  ellipsoid.color.b = rgb[2];
  ellipsoid.color.a = 0.8;
  ellipsoid.ns = frameName;
  centroid_msgs.markers.push_back(ellipsoid);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishCentroidTransform(ros::Time timeStamp, const vector_t& centroidPose, 
  const vector3_t& ellip_size, Color color, float alpha, const std::string& frameName) {
  geometry_msgs::TransformStamped centroidToWorldTransform;
  centroidToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  centroidToWorldTransform.child_frame_id = frameName;

  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(centroidPose.tail<3>()));
  centroidToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  centroidToWorldTransform.transform.translation = getVectorMsg(centroidPose.head<3>());
  tfBroadcaster_.sendTransform(centroidToWorldTransform);

  // Add ellipsoid marker to the batch (don't publish immediately)
  visualization_msgs::Marker ellipsoid;
  ellipsoid.header = getHeaderMsg(frameId_, timeStamp);
  ellipsoid.type = visualization_msgs::Marker::SPHERE;
  ellipsoid.pose.position.x = centroidPose[0];
  ellipsoid.pose.position.y = centroidPose[1];
  ellipsoid.pose.position.z = centroidPose[2];
  ellipsoid.pose.orientation = getOrientationMsg(q_world_base);

  ellipsoid.scale.x = ellip_size[0];
  ellipsoid.scale.y = ellip_size[1];
  ellipsoid.scale.z = ellip_size[2];

  const auto rgb = getRGB(color);
  ellipsoid.color.r = rgb[0];
  ellipsoid.color.g = rgb[1];
  ellipsoid.color.b = rgb[2];
  ellipsoid.color.a = alpha;
  ellipsoid.ns = frameName;
  centroid_msgs.markers.push_back(ellipsoid);
}

void CentroidalQuadrupedVisualizer::publishCargoTransform(ros::Time timeStamp, const vector_t& cargoPose, 
  const vector3_t& cargo_size, Color color, const std::string& frameName) {

  geometry_msgs::TransformStamped cargoToWorldTransform;
  cargoToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  cargoToWorldTransform.child_frame_id = frameName;

  const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(vector3_t(cargoPose.tail<3>()));
  cargoToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  cargoToWorldTransform.transform.translation = getVectorMsg(cargoPose.head<3>());
  tfBroadcaster_.sendTransform(cargoToWorldTransform);

  // Add cube marker to the batch (don't publish immediately)
  visualization_msgs::Marker cube;
  cube.header = getHeaderMsg(frameId_, timeStamp);
  cube.type = visualization_msgs::Marker::CUBE;
  cube.pose.position.x = cargoPose[0];
  cube.pose.position.y = cargoPose[1];
  cube.pose.position.z = cargoPose[2];
  cube.pose.orientation = getOrientationMsg(q_world_base);

  cube.scale.x = cargo_size[0];
  cube.scale.y = cargo_size[1];
  cube.scale.z = cargo_size[2];

  const auto rgb = getRGB(color);
  cube.color.r = rgb[0];
  cube.color.g = rgb[1];
  cube.color.b = rgb[2];
  cube.color.a = 0.8;
  cube.ns = frameName;
  cube.id = 0;

  centroid_msgs.markers.push_back(cube);
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishBaseTransform(ros::Time timeStamp, const vector_t& basePose) {
  geometry_msgs::TransformStamped baseToWorldTransform;
  baseToWorldTransform.header = getHeaderMsg(frameId_, timeStamp);
  baseToWorldTransform.child_frame_id = "base";

  const Eigen::Vector4d q_vector4 = basePose.tail(4);
  Eigen::Quaternion<scalar_t> q_world_base(q_vector4);
  baseToWorldTransform.transform.rotation = getOrientationMsg(q_world_base);
  baseToWorldTransform.transform.translation = getVectorMsg(basePose.head<3>());
  tfBroadcaster_.sendTransform(baseToWorldTransform);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishJointTransforms(ros::Time timeStamp, const vector_t& jointAngles) const {
  if (robotStatePublisherPtr_ != nullptr) {
    std::map<std::string, scalar_t> jointPositions{{"FL_hip_joint", jointAngles[0]}, {"FL_thigh_joint", jointAngles[1]},  {"FL_calf_joint", jointAngles[2]},
                                                   {"FR_hip_joint", jointAngles[3]}, {"FR_thigh_joint", jointAngles[4]},  {"FR_calf_joint", jointAngles[5]},
                                                   {"RL_hip_joint", jointAngles[6]}, {"RL_thigh_joint", jointAngles[7]},  {"RL_calf_joint", jointAngles[8]},
                                                   {"RR_hip_joint", jointAngles[9]}, {"RR_thigh_joint", jointAngles[10]}, {"RR_calf_joint", jointAngles[11]}};
    robotStatePublisherPtr_->publishTransforms(jointPositions, timeStamp);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishCartesianMarkers(ros::Time timeStamp, const contact_flag_t& contactFlags,
                                                    const std::vector<vector3_t>& feetPositions,
                                                    const std::vector<vector3_t>& feetForces) {
  // Add feet positions and forces to the batch (don't publish immediately)
  const size_t startIndex = currentState_msgs.markers.size();
  for (size_t i = 0; i < ROBOTS_FOOT_NUM; ++i) {
    currentState_msgs.markers.emplace_back(
        getFootMarker(feetPositions[i], contactFlags[i], feetColorMap_[i % 4], footMarkerDiameter_, footAlphaWhenLifted_));
    currentState_msgs.markers.emplace_back(getForceMarker(feetForces[i], feetPositions[i], contactFlags[i], Color::green, forceScale_));
  }

  // Center of pressure
  currentState_msgs.markers.emplace_back(getCenterOfPressureMarker(feetForces.begin(), feetForces.end(), feetPositions.begin(),
                                                             contactFlags.begin(), Color::green, copMarkerDiameter_));

  // Support polygon
  currentState_msgs.markers.emplace_back(
      getSupportPolygonMarker(feetPositions.begin(), feetPositions.end(), contactFlags.begin(), Color::black, supportPolygonLineWidth_));

  // Give markers an id and a frame
  assignHeader(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end());
}

void CentroidalQuadrupedVisualizer::publishRobotArmForces(ros::Time timeStamp, const std::vector<vector3_t>& armPositions,
                               const std::vector<vector3_t>& armForces) {
  // Add arm positions and forces to the batch (don't publish immediately)
  // Use different namespaces to avoid ID conflicts with foot markers
  const size_t startIndex = currentState_msgs.markers.size();
  for (size_t i = 0; i < 2; ++i) {
    auto footMarker = getFootMarker(armPositions[i], true, Color::green, footMarkerDiameter_, footAlphaWhenLifted_);
    footMarker.ns = "Arm EE Positions";  // Different namespace to avoid conflicts
    currentState_msgs.markers.emplace_back(footMarker);
    
    auto forceMarker = getForceMarker(armForces[i], armPositions[i], true, Color::green, forceScale_);
    forceMarker.ns = "Arm EE Forces";  // Different namespace to avoid conflicts
    currentState_msgs.markers.emplace_back(forceMarker);
  }

  // Give markers an id and a frame
  assignHeader(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end(), getHeaderMsg(frameId_, timeStamp));
  assignIncreasingId(currentState_msgs.markers.begin() + startIndex, currentState_msgs.markers.end());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalQuadrupedVisualizer::publishRobotBaseTransforms(ros::Time timeStamp, size_t numRobots, 
                                                                const SystemObservation& observation) {
  // Publish base transforms for each robot (used by elevation mapping in perceptive mode)
  // Each robot's base frame is named "base_N" where N is 1-indexed
  // Also publish a primary "base" frame using the first robot's pose for single-robot elevation mapping configs

  for (size_t robotId = 0; robotId < numRobots; ++robotId) {
    const size_t stateOffset = robotId * SINGLE_ROBOT_STATE_DIM;
    
    // Extract COM position and orientation from state
    const vector3_t comPosition = observation.state.segment(stateOffset, 3);
    const vector3_t eulerZyx = observation.state.segment(stateOffset + 6, 3);
    
    const Eigen::Quaternion<scalar_t> q_world_base = getQuaternionFromEulerAnglesZyx(eulerZyx);

    // Publish robot-specific base frame (base_1, base_2, etc.)
    geometry_msgs::TransformStamped robotBaseTransform;
    robotBaseTransform.header = getHeaderMsg(frameId_, timeStamp);
    robotBaseTransform.child_frame_id = "base_" + std::to_string(robotId + 1);
    robotBaseTransform.transform.translation = getVectorMsg(comPosition);
    robotBaseTransform.transform.rotation = getOrientationMsg(q_world_base);
    tfBroadcaster_.sendTransform(robotBaseTransform);

    // Publish primary "base" frame using first robot's pose (for elevation_mapping compatibility)
    if (robotId == 0) {
      geometry_msgs::TransformStamped primaryBaseTransform;
      primaryBaseTransform.header = getHeaderMsg(frameId_, timeStamp);
      primaryBaseTransform.child_frame_id = "base";
      primaryBaseTransform.transform.translation = getVectorMsg(comPosition);
      primaryBaseTransform.transform.rotation = getOrientationMsg(q_world_base);
      tfBroadcaster_.sendTransform(primaryBaseTransform);
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// void CentroidalQuadrupedVisualizer::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories& targetTrajectories) {
//   const auto& stateTrajectory = targetTrajectories.stateTrajectory;
//   const auto& inputTrajectory = targetTrajectories.inputTrajectory;

//   // Reserve com messages
//   std::vector<geometry_msgs::Point> desiredBasePositionMsg;
//   desiredBasePositionMsg.reserve(stateTrajectory.size());

//   // Reserve feet messages
//   feet_array_t<std::vector<geometry_msgs::Point>> desiredFeetPositionMsgs;
//   for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//     desiredFeetPositionMsgs[i].reserve(stateTrajectory.size());
//   }

//   for (size_t j = 0; j < stateTrajectory.size(); j++) {
//     const auto state = stateTrajectory.at(j);
//     vector_t input(centroidalModelInfo_.inputDim);
//     if (j < inputTrajectory.size()) {
//       input = inputTrajectory.at(j);
//     } else {
//       input.setZero();
//     }

//     // Construct base pose msg
//     const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);
//     geometry_msgs::Pose pose;
//     pose.position = getPointMsg(basePose.head<3>());

//     // Fill message containers
//     desiredBasePositionMsg.push_back(pose.position);

//     // Fill feet msgs
//     const auto& model = pinocchioInterface_.getModel();
//     auto& data = pinocchioInterface_.getData();
//     pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
//     pinocchio::updateFramePlacements(model, data);

//     const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
//     for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//       geometry_msgs::Pose footPose;
//       footPose.position = getPointMsg(feetPositions[i]);
//       desiredFeetPositionMsgs[i].push_back(footPose.position);
//     }
//   }

//   // Headers
//   auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::green, trajectoryLineWidth_);
//   comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
//   comLineMsg.id = 0;

//   // Publish
//   costDesiredBasePositionPublisher_.publish(comLineMsg);
//   for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//     auto footLineMsg = getLineMsg(std::move(desiredFeetPositionMsgs[i]), feetColorMap_[i], trajectoryLineWidth_);
//     footLineMsg.header = getHeaderMsg(frameId_, timeStamp);
//     footLineMsg.id = 0;
//     costDesiredFeetPositionPublishers_[i].publish(footLineMsg);
//   }
// }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// void CentroidalQuadrupedVisualizer::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t& mpcTimeTrajectory,
//                                                             const vector_array_t& mpcStateTrajectory, const ModeSchedule& modeSchedule) {
//   if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty()) {
//     return;  // Nothing to publish
//   }

//   // Reserve Feet msg
//   feet_array_t<std::vector<geometry_msgs::Point>> feetMsgs;
//   std::for_each(feetMsgs.begin(), feetMsgs.end(), [&](std::vector<geometry_msgs::Point>& v) { v.reserve(mpcStateTrajectory.size()); });

//   // Reserve Com Msg
//   std::vector<geometry_msgs::Point> mpcComPositionMsgs;
//   mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

//   // Extract Com and Feet from state
//   std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t& state) {
//     const auto basePose = centroidal_model::getBasePose(state, centroidalModelInfo_);

//     // Fill com position and pose msgs
//     geometry_msgs::Pose pose;
//     pose.position = getPointMsg(basePose.head<3>());
//     mpcComPositionMsgs.push_back(pose.position);

//     // Fill feet msgs
//     const auto& model = pinocchioInterface_.getModel();
//     auto& data = pinocchioInterface_.getData();
//     pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_));
//     pinocchio::updateFramePlacements(model, data);

//     const auto feetPositions = endEffectorKinematicsPtr_->getPosition(state);
//     for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//       const auto position = getPointMsg(feetPositions[i]);
//       feetMsgs[i].push_back(position);
//     }
//   });

//   // Convert feet msgs to Array message
//   visualization_msgs::MarkerArray markerArray;
//   markerArray.markers.reserve(centroidalModelInfo_.numThreeDofContacts +
//                               2);  // 1 trajectory per foot + 1 for the future footholds + 1 for the com trajectory
//   for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//     markerArray.markers.emplace_back(getLineMsg(std::move(feetMsgs[i]), feetColorMap_[i], trajectoryLineWidth_));
//     markerArray.markers.back().ns = "EE Trajectories";
//   }
//   markerArray.markers.emplace_back(getLineMsg(std::move(mpcComPositionMsgs), Color::red, trajectoryLineWidth_));
//   markerArray.markers.back().ns = "CoM Trajectory";

//   // Future footholds
//   visualization_msgs::Marker sphereList;
//   sphereList.type = visualization_msgs::Marker::SPHERE_LIST;
//   sphereList.scale.x = footMarkerDiameter_;
//   sphereList.scale.y = footMarkerDiameter_;
//   sphereList.scale.z = footMarkerDiameter_;
//   sphereList.ns = "Future footholds";
//   sphereList.pose.orientation = getOrientationMsg({1., 0., 0., 0.});
//   const auto& eventTimes = modeSchedule.eventTimes;
//   const auto& subsystemSequence = modeSchedule.modeSequence;
//   const auto tStart = mpcTimeTrajectory.front();
//   const auto tEnd = mpcTimeTrajectory.back();
//   for (size_t event = 0; event < eventTimes.size(); ++event) {
//     if (tStart < eventTimes[event] && eventTimes[event] < tEnd) {  // Only publish future footholds within the optimized horizon
//       const auto preEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event]);
//       const auto postEventContactFlags = modeNumber2StanceLeg(subsystemSequence[event + 1]);
//       const auto postEventState = LinearInterpolation::interpolate(eventTimes[event], mpcTimeTrajectory, mpcStateTrajectory);

//       const auto& model = pinocchioInterface_.getModel();
//       auto& data = pinocchioInterface_.getData();
//       pinocchio::forwardKinematics(model, data, centroidal_model::getGeneralizedCoordinates(postEventState, centroidalModelInfo_));
//       pinocchio::updateFramePlacements(model, data);

//       const auto feetPosition = endEffectorKinematicsPtr_->getPosition(postEventState);
//       for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
//         if (!preEventContactFlags[i] && postEventContactFlags[i]) {  // If a foot lands, a marker is added at that location.
//           sphereList.points.emplace_back(getPointMsg(feetPosition[i]));
//           sphereList.colors.push_back(getColor(feetColorMap_[i]));
//         }
//       }
//     }
//   }
//   markerArray.markers.push_back(std::move(sphereList));

//   // Add headers and Id
//   assignHeader(markerArray.markers.begin(), markerArray.markers.end(), getHeaderMsg(frameId_, timeStamp));
//   assignIncreasingId(markerArray.markers.begin(), markerArray.markers.end());

//   stateOptimizedPublisher_.publish(markerArray);
// }

}  // namespace multi_robot
}  // namespace ocs2
