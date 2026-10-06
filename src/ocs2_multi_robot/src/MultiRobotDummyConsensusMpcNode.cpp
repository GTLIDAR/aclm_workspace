#include <cstdlib>
#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "ocs2_multi_robot/alternating_base/AlternatingInterface.h"
#include "ocs2_multi_robot/alternating_base/ConsensusADMMSolverMPC.h"
#include "ocs2_multi_robot/terrain/MultiRobotGridMapHeightMap.h"
#include "ocs2_multi_robot/terrain/MultiRobotConvexRegionSelector.h"
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_quadruped/gait/GaitReceiver.h"


using namespace ocs2;
using namespace multi_robot;

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";

  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nh;

  // path to config file
  std::string taskFile = ros::package::getPath("ocs2_multi_robot") + "/config/info/three_quadruped_w_cargo_admm.info";
  nh.getParam("taskFile", taskFile);
  std::cerr << "Loading task file: " << taskFile << std::endl;
  // path to save auto-generated libraries
  std::string libraryFolder = ros::package::getPath("ocs2_multi_robot") + "/auto_generated/admm_mpc";
  std::cerr << "Generated library path: " << libraryFolder << std::endl;

  /* The optimal control problem formulation*/
  ROS_INFO("good before alternating interface");
  AlternatingInterface alternatingInterface(taskFile, libraryFolder, true);

  scalar_t timeHorizon;
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", timeHorizon);

  // Gait receiver
  auto gaitReceiverPtr =
    std::make_shared<quadruped::GaitReceiver>(nh, alternatingInterface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, alternatingInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);

  // MPC
  ConsensusADMMSolverMPC mpc(alternatingInterface);
  // NOTE: Do NOT override the solver's reference manager here.
  // The ConsensusADMMSolver constructor sets up a ThreadSafeReferenceManager that deduplicates
  // preSolverRun() calls by horizon. Overriding with the raw pointer causes sub-problem solvers
  // (cargo: 12-dim state, robot: 24-dim) to propagate their partial state through the wrapper,
  // triggering "Invalid state size" errors in PerceptiveMultiRobotReferenceManager.
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

  // Add PlanarTerrainReceiverModule if perceptive mode is enabled
  if (alternatingInterface.isPerceptiveMode()) {
    auto terrainReceiverPtr = std::make_shared<MultiRobotPlanarTerrainReceiverModule>(
        nh,
        alternatingInterface.getGridMapHeightMapPtr(),
        alternatingInterface.getPlanarTerrainPtr(),
        "/convex_plane_decomposition_ros/planar_terrain");
    mpc.getSolverPtr()->addSynchronizedModule(terrainReceiverPtr);
    ROS_INFO("[MultiRobotDummyConsensusMpcNode] Added MultiRobotPlanarTerrainReceiverModule for perceptive locomotion");
    
    // Initialize visualization for ConvexRegionSelector
    if (alternatingInterface.getConvexRegionSelectorPtr()) {
      alternatingInterface.getConvexRegionSelectorPtr()->initializeVisualization(nh, "odom");
      ROS_INFO("[MultiRobotDummyConsensusMpcNode] Initialized ConvexRegionSelector visualization");
    }
  }

  // Launch MPC ROS node
  MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nh);

  ros::spin();
  return 0;
}

