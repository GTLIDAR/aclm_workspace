#include <pinocchio/fwd.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "ocs2_multi_robot/MultiRobotWCargoInterface.h"
#include "ocs2_multi_robot/visualization/ObstacleVisualization.h"
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include "ocs2_quadruped/gait/GaitReceiver.h"


using namespace ocs2;
using namespace multi_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nh;

  // path to config file
  std::string taskFile = ros::package::getPath("ocs2_multi_robot") + "/config/info/three_quadruped_w_cargo.info";
  nh.getParam("taskFile", taskFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  // path to save auto-generated libraries
  std::string libraryFolder = ros::package::getPath("ocs2_multi_robot") + "/auto_generated/mpc";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  /* The optimal control problem formulation*/
  ROS_INFO("good before multi robot interface");
  MultiRobotWCargoInterface ocp(taskFile, libraryFolder, true, true);

  scalar_t timeHorizon;
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon);

  // Gait receiver
  auto gaitReceiverPtr =
    std::make_shared<quadruped::GaitReceiver>(nh, ocp.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, ocp.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);

  // MPC
  SqpMpc mpc(ocp.mpcSettings(), ocp.sqpSettings(), ocp.getOptimalControlProblem(), ocp.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(ocp.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  // Add PlanarTerrainReceiverModule if perceptive mode is enabled
  if (ocp.isPerceptiveMode()) {
    auto terrainReceiverPtr = std::make_shared<MultiRobotPlanarTerrainReceiverModule>(
        nh,
        ocp.getGridMapHeightMapPtr(),
        ocp.getPlanarTerrainPtr(),
        "/convex_plane_decomposition_ros/planar_terrain");
    mpc.getSolverPtr()->addSynchronizedModule(terrainReceiverPtr);
    ROS_INFO("[MultiRobotDummyMpcNode] Added MultiRobotPlanarTerrainReceiverModule for perceptive locomotion");
    
    // Initialize visualization for ConvexRegionSelector
    if (ocp.getConvexRegionSelectorPtr()) {
      ocp.getConvexRegionSelectorPtr()->initializeVisualization(nh, "odom");
      ROS_INFO("[MultiRobotDummyMpcNode] Initialized ConvexRegionSelector visualization");
    }
  }

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nh);

  ros::spin();
  return 0;
}

