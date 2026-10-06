cmake_minimum_required(VERSION 3.10)
project(ocs2_quadruped)

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
        ocs2_robotic_tools
        ocs2_pinocchio_interface
        ocs2_centroidal_model
        ocs2_msgs
        ocs2_ros_interfaces
        roscpp
        roslib
        std_msgs
        message_generation
        cmake_modules
        tf
        urdf
        kdl_parser
        robot_state_publisher
        )

find_package(catkin REQUIRED COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

find_package(Boost REQUIRED COMPONENTS
        system
        filesystem
        )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

add_message_files(FILES FullState.msg Output.msg)
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package(
        INCLUDE_DIRS
        include
        ${EIGEN3_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        DEPENDS
        Boost
        pinocchio
)

###########
## Build ##
###########

# Resolve for the package path at compile time.
configure_file (
        "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
        "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
        )

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

# Legged robot interface library
add_library(${PROJECT_NAME}
        src/common/ModelSettings.cpp
        src/dynamics/LeggedRobotDynamicsAD.cpp
        src/constraint/EndEffectorLinearConstraint.cpp
        src/constraint/EndEffectorConstraint.cpp
        src/constraint/FrictionConeConstraint.cpp
        src/constraint/FrictionConeConstraintInvariant.cpp
        src/constraint/ZeroForceConstraint.cpp
        src/constraint/NormalVelocityConstraintCppAd.cpp
        src/constraint/NormalVelocityConstraintCppAdRaw.cpp
        src/constraint/ZeroVelocityConstraintCppAd.cpp
        src/constraint/FootPlacementConstraint.cpp
        src/constraint/CollisionAvoidanceConstraint.cpp
        src/initialization/LeggedRobotInitializer.cpp
        src/initialization/LocoManipulationInitializer.cpp
        src/reference_manager/SwitchedModelReferenceManager.cpp
        src/foot_planner/CubicSpline.cpp
        src/foot_planner/SplineCpg.cpp
        src/foot_planner/SwingTrajectoryPlanner.cpp
        src/gait/Gait.cpp
        src/gait/GaitSchedule.cpp
        src/gait/ModeSequenceTemplate.cpp
        src/LeggedRobotInterface.cpp
        src/LeggedRobotInterfaceRaw.cpp
        src/LeggedRobotPreComputation.cpp
        src/LeggedRobotPreComputationRaw.cpp
        src/LocoManipulationPreComputation.cpp
        src/LocoManipulationInterface.cpp
        src/gait/GaitKeyboardPublisher.cpp
        src/gait/GaitReceiver.cpp
        src/visualization/LeggedRobotVisualizer.cpp
        src/visualization/FullStateVisualizer.cpp
        src/visualization/FullStateVisualizerManipulation.cpp
        src/visualization/LocoManipulationVisualizer.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${pinocchio_LIBRARIES}
        dl
        jsoncpp
        )
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

function(add_quadruped_node target source)
    add_executable(${target} ${source})
    add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${target} ${PROJECT_NAME} ${catkin_LIBRARIES})
    target_compile_options(${target} PRIVATE ${OCS2_CXX_FLAGS})
endfunction()

add_quadruped_node(legged_robot_ddp_mpc src/LeggedRobotDdpMpcNode.cpp)
add_quadruped_node(legged_robot_sqp_mpc src/LeggedRobotSqpMpcNode.cpp)
add_quadruped_node(legged_robot_sqp_mpc_raw src/LeggedRobotSqpMpcNodeRaw.cpp)
add_quadruped_node(legged_robot_dummy src/LeggedRobotDummyNode.cpp)
add_quadruped_node(legged_robot_dummy_raw src/LeggedRobotDummyNodeRaw.cpp)
add_quadruped_node(legged_robot_target src/LeggedRobotPoseCommandNode.cpp)
add_quadruped_node(legged_robot_gait_command src/LeggedRobotGaitCommandNode.cpp)
add_quadruped_node(legged_target_trajectories_publisher src/target_trajectories_publisher.cpp)
add_quadruped_node(loco_manipulation_target src/LocoManipulationTarget.cpp)
add_quadruped_node(loco_manipulation_keyboard_target src/LocoManipulationKeyboardTarget.cpp)
add_quadruped_node(loco_manipulation_sqp_mpc src/LocoManipulationSqpMpcNode.cpp)
add_quadruped_node(loco_manipulation_dummy src/LocoManipulationDummyNode.cpp)
add_quadruped_node(torque_compute src/TorqueCompute.cpp)
add_quadruped_node(polygon_publisher src/PublishPolygon.cpp)

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target ocs2_legged_robot")
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        legged_robot_ddp_mpc legged_robot_sqp_mpc legged_robot_sqp_mpc_raw
        legged_robot_dummy legged_robot_dummy_raw legged_robot_target legged_robot_gait_command
        legged_target_trajectories_publisher loco_manipulation_target loco_manipulation_keyboard_target
        loco_manipulation_sqp_mpc loco_manipulation_dummy torque_compute polygon_publisher
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(DIRECTORY config launch rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

#############
## Testing ##
#############

#catkin_add_gtest(${PROJECT_NAME}_test
#        test/AnymalFactoryFunctions.cpp
#        test/constraint/testEndEffectorLinearConstraint.cpp
#        test/constraint/testFrictionConeConstraint.cpp
#        test/constraint/testZeroForceConstraint.cpp
#        )
#target_include_directories(${PROJECT_NAME}_test PRIVATE
#        test/include
#        ${PROJECT_BINARY_DIR}/include
#        )
#target_link_libraries(${PROJECT_NAME}_test
#        gtest_main
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${Boost_LIBRARIES}
#        )
#target_compile_options(${PROJECT_NAME}_test PRIVATE ${FLAGS})
