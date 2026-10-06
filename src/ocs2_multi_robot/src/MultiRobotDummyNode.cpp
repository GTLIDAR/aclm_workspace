#include <pinocchio/fwd.hpp>

// Include CGAL-dependent headers first to avoid BOOST_PARAMETER_MAX_ARITY redefinition warning
#include "ocs2_multi_robot/MultiRobotWCargoInterface.h"

#include <ros/init.h>
#include <ros/package.h>


#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_multi_robot/visualization/MultiRobotVisualizer.h>
#include <ocs2_multi_robot/visualization/ObstacleVisualization.h>

#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

using namespace ocs2;
using namespace multi_robot;


int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nh;

  std::string taskFile = ros::package::getPath("ocs2_multi_robot") + "/config/info/three_quadruped_w_cargo.info";
  nh.getParam("taskFile", taskFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  std::string libraryFolder = ros::package::getPath("ocs2_multi_robot") + "/auto_generated/dummy";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  // Robot interface, load setting params
  MultiRobotWCargoInterface ocp(taskFile, libraryFolder, true, false);

  /*
    MRT Model Reference Tracking interface with ROS.
    Sets the topics and data communications between ROS and Dummy Node.
  */
  MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&ocp.getRollout());
  mrt.launchNodes(nh);

  // Visualization to rviz
  std::string terrain_type;
  loadData::loadCppDataType(taskFile, "terrain_settings.terrain_type", terrain_type);
  std::shared_ptr<MultiRobotVisualizer> visualizer(new MultiRobotVisualizer(nh, taskFile, terrain_type));
  
  // Add obstacles visualization (only if enabled)
  std::unique_ptr<ObstacleVisualization> obstacleVisualizationPtr;
  if (ocp.isObstacleAvoidanceEnabled()) {
    obstacleVisualizationPtr = std::make_unique<ObstacleVisualization>(nh, taskFile);
    ROS_INFO("[MultiRobotDummyNode] Obstacle visualization enabled");
  }
  
  // Dummy Simulator
  MRT_ROS_Dummy_Loop DummySimulator(mrt, ocp.mpcSettings().mrtDesiredFrequency_, ocp.mpcSettings().mpcDesiredFrequency_);
  DummySimulator.subscribeObservers({visualizer});

  vector_t target_state = ocp.getInitialState();

  // Initial State
  SystemObservation initObservation;
  initObservation.state = ocp.getInitialState();
  const size_t stateDim = initObservation.state.rows();
  // Compute input dimension from state dimension (stateDim = numRobots * SINGLE_ROBOT_STATE_DIM + CARGO_STATE_DIM)
  // This ensures consistency with the dynamically detected number of robots from the config file
  const size_t numRobots = (stateDim - CARGO_STATE_DIM) / SINGLE_ROBOT_STATE_DIM;
  const size_t inputDim = numRobots * SINGLE_ROBOT_INPUT_DIM + numRobots * ARM_CONTACT_DIM;
  initObservation.input = vector_t::Zero(inputDim);
  initObservation.mode = ModeNumber::STANCE;

  // Initial command
  TargetTrajectories initTargetTrajectories({0.0, 10.0}, {initObservation.state, target_state}, {initObservation.input, initObservation.input});

  // run dummy
  DummySimulator.run(initObservation, initTargetTrajectories);

  // Successful exit
  return 0;
}

