//
// Created by qiayuan on 2022/7/24.
// Modified to support perceptive terrain mode and integrated waypoint trajectories
//

#pragma once

#include <mutex>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "ocs2_multi_robot/terrain/HeightMap.h"
#include "ocs2_multi_robot/common/Types.h"
#include "ocs2_multi_robot/WaypointTrajectoryGenerator.h"
#include "ocs2_multi_robot/LoadWaypointTrajectory.h"

namespace ocs2 {
namespace multi_robot {

/**
 * @brief Integrated Target Trajectories Publisher
 * 
 * Supports two modes:
 * 1. Goal Mode (default): Trajectories generated from RViz goal arrow (/move_base_simple/goal) or cmd_vel
 * 2. Waypoint Mode: Trajectories generated from YAML waypoints loaded via service
 * 
 * Both modes are terrain-aware when height map is available.
 * 
 * Switching logic:
 * - Receiving /move_base_simple/goal -> switches to goal mode
 * - Calling /load_waypoint_trajectory service -> switches to waypoint mode
 */
class MultiCentroidalTargetTrajectoriesPublisher final
{
public:
  using CmdToTargetTrajectories =
      std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;
  
  /**
   * @brief Check if currently in waypoint mode
   */
  bool isWaypointMode() const {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return use_waypoint_mode_;
  }
  
  /**
   * @brief Set robot base offsets for waypoint mode (delegates to generator)
   */
  void setRobotBaseOffsets(const std::vector<vector_t>& offsets) {
    waypointGenerator_.setRobotBaseOffsets(offsets);
  }
  
  /**
   * @brief Set grid map pointer for boundary-clamped height queries in waypoint mode
   */
  void setGridMap(const grid_map::GridMap* gridMap) {
    waypointGenerator_.setGridMap(gridMap);
  }

  MultiCentroidalTargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topic_prefix,
                              CmdToTargetTrajectories goal_to_target_trajectories,
                              CmdToTargetTrajectories cmd_vel_to_target_trajectories,
                              std::shared_ptr<HeightMap> heightMapPtr = nullptr,
                              scalar_t comHeight = 0.5,
                              size_t numRobots = 2,
                              scalar_t cargoHeightOffset = 0.8,
                              scalar_t targetDisplacementVelocity = 0.3,
                              scalar_t targetRotationVelocity = 0.15)
    : goal_to_target_trajectories_(std::move(goal_to_target_trajectories))
    , cmd_vel_to_target_trajectories_(std::move(cmd_vel_to_target_trajectories))
    , heightMapPtr_(heightMapPtr)
    , comHeight_(comHeight)
    , numRobots_(numRobots)
    , tf2_(buffer_)
    , has_goal_(false)
    , use_cmd_vel_(false)
    , use_waypoint_mode_(false)
    , waypointGenerator_(numRobots, comHeight, cargoHeightOffset)
    , defaultLinearVelocity_(targetDisplacementVelocity)
    , defaultRotationalVelocity_(targetRotationVelocity)
  {
    // Set velocities from config
    waypointGenerator_.setVelocity(targetDisplacementVelocity);
    waypointGenerator_.setRotationalVelocity(targetRotationVelocity);
    
    // Trajectories publisher
    target_trajectories_publisher_.reset(new TargetTrajectoriesRosPublisher(nh, topic_prefix));

    // Observation subscriber - shared by both modes
    auto observation_callback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      latest_observation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observation_sub_ =
        nh.subscribe<ocs2_msgs::mpc_observation>(topic_prefix + "_mpc_observation", 1, observation_callback);

    // Goal subscriber - switches to goal mode
    auto goal_callback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock_obs(observation_mutex_);
      if (latest_observation_.time == 0.0)
        return;
      
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("TF failure: %s", ex.what());
        return;
      }

      vector_t cmd_goal = vector_t::Zero(6);
      cmd_goal[0] = pose.pose.position.x;
      cmd_goal[1] = pose.pose.position.y;
      cmd_goal[2] = pose.pose.position.z;
      
      // Extract only YAW from quaternion
      const auto& q = pose.pose.orientation;
      scalar_t siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      scalar_t cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      cmd_goal[3] = std::atan2(siny_cosp, cosy_cosp);
      cmd_goal[4] = 0.0;
      cmd_goal[5] = 0.0;
      
      ROS_INFO("[TargetPublisher] Goal received: (%.2f, %.2f), yaw: %.2f deg",
               cmd_goal[0], cmd_goal[1], cmd_goal[3] * 180.0 / M_PI);

      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (use_waypoint_mode_) {
          ROS_INFO("[TargetPublisher] Switching from WAYPOINT mode to GOAL mode");
        }
        use_waypoint_mode_ = false;
        latest_goal_ = cmd_goal;
        has_goal_ = true;
        use_cmd_vel_ = false;
      }

      const auto trajectories = goal_to_target_trajectories_(cmd_goal, latest_observation_);
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmd_vel_callback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock_obs(observation_mutex_);
      if (latest_observation_.time == 0.0)
        return;

      vector_t cmd_vel = vector_t::Zero(4);
      cmd_vel[0] = msg->linear.x;
      cmd_vel[1] = msg->linear.y;
      cmd_vel[2] = msg->linear.z;
      cmd_vel[3] = msg->angular.z;

      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        // cmd_vel does not change waypoint mode, just updates velocity command
        if (!use_waypoint_mode_) {
          latest_cmd_vel_ = cmd_vel;
          has_goal_ = true;
          use_cmd_vel_ = true;
        }
      }

      if (!use_waypoint_mode_) {
        const auto trajectories = cmd_vel_to_target_trajectories_(cmd_vel, latest_observation_);
        target_trajectories_publisher_->publishTargetTrajectories(trajectories);
      }
    };

    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goal_callback);
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_callback);
    
    // Waypoint mode service - switches to waypoint mode
    waypoint_service_ = nh.advertiseService("/load_waypoint_trajectory", 
                                             &MultiCentroidalTargetTrajectoriesPublisher::loadWaypointCallback, this);
    
    // Waypoint mode topic trigger (alternative to service)
    auto waypoint_trigger_callback = [this](const std_msgs::Bool::ConstPtr& msg) {
      if (msg->data) {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (waypointGenerator_.isLoaded()) {
          use_waypoint_mode_ = true;
          ROS_INFO("[TargetPublisher] Waypoint mode ACTIVATED via topic (trajectory: %s)",
                   waypointGenerator_.getTrajectoryName().c_str());
        } else {
          ROS_WARN("[TargetPublisher] Cannot activate waypoint mode - no trajectory loaded!");
        }
      } else {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        if (use_waypoint_mode_) {
          use_waypoint_mode_ = false;
          ROS_INFO("[TargetPublisher] Waypoint mode DEACTIVATED via topic");
        }
      }
    };
    waypoint_trigger_sub_ = nh.subscribe<std_msgs::Bool>("/waypoint_mode_active", 1, waypoint_trigger_callback);
    
    // Timer to continuously update trajectory (10 Hz)
    update_timer_ = nh.createTimer(ros::Duration(0.1), 
                                    &MultiCentroidalTargetTrajectoriesPublisher::updateCallback, this);
    
    ROS_INFO("[TargetPublisher] Initialized with %zu robots", numRobots);
    ROS_INFO("  - Goal mode: Use RViz '2D Nav Goal' arrow");
    ROS_INFO("  - Waypoint mode: Call '/load_waypoint_trajectory' service");
  }

private:
  /**
   * @brief Service callback to load waypoint trajectory
   */
  bool loadWaypointCallback(ocs2_multi_robot::LoadWaypointTrajectory::Request& req,
                            ocs2_multi_robot::LoadWaypointTrajectory::Response& res) {
    std::string yamlFile = req.yaml_file;
    std::string trajectoryName = req.trajectory_name;
    scalar_t velocity = req.velocity > 0.0 ? req.velocity : defaultLinearVelocity_;
    
    ROS_INFO("[TargetPublisher] Loading waypoint trajectory: %s from %s (velocity: %.2f m/s)",
             trajectoryName.c_str(), yamlFile.c_str(), velocity);
    
    if (waypointGenerator_.loadFromYaml(yamlFile, trajectoryName)) {
      waypointGenerator_.setVelocity(velocity);
      
      {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        use_waypoint_mode_ = true;
      }
      
      res.success = true;
      res.message = "Loaded trajectory '" + trajectoryName + "', switched to waypoint mode";
      ROS_INFO("[TargetPublisher] %s", res.message.c_str());
      
      // Immediately publish first trajectory
      publishWaypointTrajectory();
      
      return true;
    } else {
      res.success = false;
      res.message = "Failed to load trajectory '" + trajectoryName + "' from " + yamlFile;
      ROS_ERROR("[TargetPublisher] %s", res.message.c_str());
      return true;  // Service call succeeded, but loading failed
    }
  }
  
  /**
   * @brief Generate and publish waypoint trajectory
   */
  void publishWaypointTrajectory() {
    std::lock_guard<std::mutex> lock_obs(observation_mutex_);
    if (latest_observation_.time == 0.0) {
      ROS_WARN_THROTTLE(1.0, "[TargetPublisher] Waiting for observation before publishing waypoint trajectory");
      return;
    }
    
    auto trajectories = waypointGenerator_.generateTrajectory(latest_observation_, heightMapPtr_, 50);
    if (!trajectories.timeTrajectory.empty()) {
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    }
  }
  
  /**
   * @brief Timer callback to update trajectory
   */
  void updateCallback(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock_obs(observation_mutex_);
    if (latest_observation_.time == 0.0)
      return;
    
    std::lock_guard<std::mutex> lock(mode_mutex_);
    
    if (use_waypoint_mode_) {
      // Waypoint mode: generate terrain-aware trajectory from waypoints
      auto trajectories = waypointGenerator_.generateTrajectory(latest_observation_, heightMapPtr_, 50);
      if (!trajectories.timeTrajectory.empty()) {
        // Debug: log that we're publishing waypoint trajectory
        static int waypoint_pub_count = 0;
        if (waypoint_pub_count++ % 50 == 0) {  // Log every 5 seconds (at 10Hz)
          ROS_INFO("[TargetPublisher] Publishing waypoint trajectory: %zu points, t=[%.2f, %.2f]",
                   trajectories.timeTrajectory.size(),
                   trajectories.timeTrajectory.front(),
                   trajectories.timeTrajectory.back());
        }
        target_trajectories_publisher_->publishTargetTrajectories(trajectories);
      } else {
        ROS_WARN_THROTTLE(1.0, "[TargetPublisher] Waypoint trajectory is empty!");
      }
    } else if (has_goal_) {
      // Goal mode: generate trajectory from goal/cmd_vel
      TargetTrajectories trajectories;
      if (use_cmd_vel_) {
        trajectories = cmd_vel_to_target_trajectories_(latest_cmd_vel_, latest_observation_);
      } else {
        trajectories = goal_to_target_trajectories_(latest_goal_, latest_observation_);
      }
      target_trajectories_publisher_->publishTargetTrajectories(trajectories);
    }
  }

  // Trajectory generation functions
  CmdToTargetTrajectories goal_to_target_trajectories_, cmd_vel_to_target_trajectories_;
  
  // Waypoint trajectory generator
  WaypointTrajectoryGenerator waypointGenerator_;

  // Publisher
  std::unique_ptr<TargetTrajectoriesRosPublisher> target_trajectories_publisher_;

  // Subscribers
  ::ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_, waypoint_trigger_sub_;
  
  // Service
  ::ros::ServiceServer waypoint_service_;
  
  // Timer
  ::ros::Timer update_timer_;
  
  // TF
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  // Height map
  std::shared_ptr<HeightMap> heightMapPtr_;
  scalar_t comHeight_;
  size_t numRobots_;
  
  // Default velocities from config
  scalar_t defaultLinearVelocity_;
  scalar_t defaultRotationalVelocity_;

  // State
  mutable std::mutex observation_mutex_;
  mutable std::mutex mode_mutex_;
  SystemObservation latest_observation_;
  vector_t latest_goal_;
  vector_t latest_cmd_vel_;
  bool has_goal_;
  bool use_cmd_vel_;
  bool use_waypoint_mode_;
};

}  // namespace multi_robot
}  // namespace ocs2