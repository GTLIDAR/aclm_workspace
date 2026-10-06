cmake_minimum_required(VERSION 3.14)
project(ocs2_multi_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
        ocs2_core
        ocs2_oc
        ocs2_ddp
        ocs2_mpc
        ocs2_sqp
        ocs2_ipm
        ocs2_robotic_tools
        ocs2_quadruped
        ocs2_ros_interfaces
        roscpp
        roslib
        tf
        urdf
        kdl_parser
        robot_state_publisher
        sensor_msgs
        message_generation
        convex_plane_decomposition
        convex_plane_decomposition_msgs
        convex_plane_decomposition_ros
        grid_map_ros
        grid_map_sdf
        elevation_mapping
        point_cloud_io
        )

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3.7 EXACT REQUIRED NO_MODULE)
message(STATUS "ocs2_multi_robot using Eigen ${Eigen3_VERSION}")

find_package(jsoncpp REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

add_service_files(
  FILES
  PreStageMode.srv
  LoadWaypointTrajectory.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        third_party/qpOASES/include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_kinematics
        qpOASES
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
        Boost
        pinocchio
        message_runtime
)

include(cmake/QpOases.cmake)

###########
## Build ##
###########


# set(FLAGS
#         ${OCS2_CXX_FLAGS}
#         ${pinocchio_CFLAGS_OTHER}
#         -Wno-ignored-attributes
#         -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
#         -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
#         -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
#         )

include_directories(
        include
        ${pinocchio_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}_kinematics src/kinematics/WholeBodyDifferentialIK.cpp)
target_compile_features(${PROJECT_NAME}_kinematics PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_kinematics PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_kinematics PUBLIC Eigen3::Eigen qpOASES ${catkin_LIBRARIES})

# Legged robot interface library
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_library(${PROJECT_NAME}
        src/dynamics/CentroidalDynamicsAD.cpp
        src/dynamics/CentroidalDynamicsWithEllipsoidAD.cpp
        src/dynamics/SingleRigidBodyDynamicsAD.cpp
        src/dynamics/TwoQuadrupedSingleRigidBodyDynamicsAD.cpp
        src/dynamics/TwoQuadwCargoSRBDAD.cpp
        src/dynamics/MultiRobotWCargoSRBDAD.cpp
        src/dynamics/distributed/CargowithArmForceAD.cpp
        src/dynamics/distributed/SRBDwithArmForceAD.cpp
        src/cost/AlternatingQuadraticStateTrackingCost.cpp
        src/cost/AlternatingQuadraticTrackingCost.cpp
        src/common/AlternatingTargetTrajectories.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/ZeroVelocityConstraint.cpp
        src/constraint/NormalVelocityConstraint.cpp
        src/constraint/FootPlacementConstraint.cpp
        src/constraint/KinematicsBoxConstraint.cpp
        src/constraint/ArmKinematicsBoxConstraint.cpp
        src/constraint/RobotCargoFormationConstraint.cpp
        src/constraint/MultiRobotArmKinematicsBoxConstraint.cpp
        src/constraint/MultiRobotCargoFormationConstraint.cpp
        src/constraint/EllipsoidMassConstraint.cpp
        src/constraint/distributed/SingleRobotArmKinematicsBoxConstraint.cpp
        src/constraint/distributed/SingleRobotCargoFormationConstraint.cpp
        src/constraint/distributed/CargoRobotFormationConstraint.cpp
        src/constraint/distributed/CargoArmKinematicsBoxConstraint.cpp
        src/constraint/distributed/CargoManipulationFrictionConeConstraint.cpp
        src/constraint/distributed/SingleRobotManipulationFrictionConeConstraint.cpp
        src/constraint/distributed/CargoManipulationTorqueBoxConstraint.cpp
        src/constraint/distributed/SingleRobotManipulationTorqueBoxConstraint.cpp
        src/constraint/ManipulationFrictionConeConstraint.cpp
        src/constraint/ManipulationTorqueBoxConstraint.cpp
        src/initialization/QuadrupedInitializer.cpp
        src/initialization/SingleRobotInitializer.cpp
        src/initialization/CargoInitializer.cpp
        src/initialization/TwoQuadWCargoInitializer.cpp
        src/initialization/MultiRobotWCargoInitializer.cpp
        src/terrain/HeightMap.cpp
        src/terrain/HeightMapExamples.cpp
        src/terrain/MultiRobotConvexRegionSelector.cpp
        src/terrain/MultiRobotGridMapHeightMap.cpp
        src/reference_manager/SwitchedModelReferenceManagerWithTerrain.cpp
        src/reference_manager/PerceptiveMultiRobotReferenceManager.cpp
        src/constraint/MultiRobotPerceptiveFootPlacementConstraint.cpp
        src/constraint/MultiRobotFootCollisionConstraint.cpp
        src/precomputation/MultiRobotPerceptivePreComputation.cpp
        src/precomputation/AlternatingPerceptivePreComputation.cpp
        src/constraint/distributed/SingleRobotPerceptiveFootPlacementConstraint.cpp
        src/visualization/CentroidalQuadrupedVisualizer.cpp
        src/visualization/MultiRobotVisualizer.cpp
        src/MultiRobotWCargoInterface.cpp
        src/robot_interface/MultiRobotInterfaceAbstract.cpp
        src/robot_interface/TwoQuadPreStageController.cpp
        src/utils/Munkres.cpp
        src/alternating_base/AlternatingOptBase.cpp
        src/alternating_base/AlternatingOptSettings.cpp
        src/alternating_base/AlternatingInterface.cpp
        src/alternating_base/ConsensusADMMSolver.cpp
        src/AlternatingPreComputation.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${PROJECT_NAME}_kinematics
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${pinocchio_LIBRARIES}
        jsoncpp
        yaml-cpp
        Eigen3::Eigen
        )

# target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

add_executable(MultiRobotDummyMpcNode
        src/MultiRobotDummyMpcNode.cpp
        )

add_dependencies(MultiRobotDummyMpcNode
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(MultiRobotDummyMpcNode
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
)

add_executable(MultiRobotDummyConsensusMpcNode
        src/MultiRobotDummyConsensusMpcNode.cpp
        )

add_dependencies(MultiRobotDummyConsensusMpcNode
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(MultiRobotDummyConsensusMpcNode
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
)

add_executable(MultiRobotDummyNode
        src/MultiRobotDummyNode.cpp
        )

add_dependencies(MultiRobotDummyNode
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(MultiRobotDummyNode
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
)

add_executable(MultiRobotGeneralizedTargetTrajectoriesPublisher
        src/MultiRobotGeneralizedTargetTrajectoriesPublisher.cpp
        )

add_dependencies(MultiRobotGeneralizedTargetTrajectoriesPublisher
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_generate_messages_cpp
        )

target_link_libraries(MultiRobotGeneralizedTargetTrajectoriesPublisher
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        yaml-cpp
)

add_executable(GaitCommandNode
        src/GaitCommandNode.cpp
        )

add_dependencies(GaitCommandNode
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(GaitCommandNode
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
)

add_executable(CargoPoseTarget
        src/CargoPoseTarget.cpp
        )

add_dependencies(CargoPoseTarget
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(CargoPoseTarget
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
)


#############
## Install ##
#############
catkin_install_python(PROGRAMS script/view_frame_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_kinematics qpOASES
        MultiRobotDummyMpcNode MultiRobotDummyConsensusMpcNode MultiRobotDummyNode
        MultiRobotGeneralizedTargetTrajectoriesPublisher GaitCommandNode CargoPoseTarget
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(DIRECTORY config launch rviz assets doc
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../THIRD_PARTY_NOTICES.md"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../CITATION.cff"
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/licensing
        COMPONENT licensing
        )
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSES/"
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/licensing/LICENSES
        COMPONENT licensing
        )
